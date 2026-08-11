#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/error.hpp"
#include "ecs/handle.hpp"
#include "ecs/pool.hpp"

namespace spade {

// Monotonically-assigned component type identity. Assignment order is
// first-call registration order (the first time component_id<T>() runs for
// a given T in this process), NOT typeid -- this is the resurrected v1
// GetUniqueComponentID pattern (see include/Spade/Core/Objects.hpp's
// Internal::GetUniqueComponentID, which hashed typeid; v2 replaces the hash
// with a plain atomic counter, per the engine design spec's ECS section).
//
// IMPORTANT: first-call order is a property of one process's runtime, not
// of the type itself -- two processes (or two translation units that touch
// component_id<T>() for different T first) can assign different ids to the
// same T. Persistent formats (Task 7's snapshot format) must never
// serialize a raw ComponentId; they store the registered component *name*
// and re-resolve it to whatever id this process happens to assign on load.
using ComponentId = uint32_t;

namespace detail {
inline std::atomic<ComponentId> g_next_component_id{0};
}  // namespace detail

template <class T>
ComponentId component_id() {
    static const ComponentId id = detail::g_next_component_id.fetch_add(1, std::memory_order_relaxed);
    return id;
}

// Constraint for Registry::add/get/remove<T>: components are plain stored
// values -- move-constructible so Pool<T> can place them, and additionally
// move-assignable because add()'s overwrite-in-place path (re-adding a
// component to an entity that already has one) move-assigns into the
// existing pool slot rather than destroying and recreating it -- keeping
// that slot's index stable across repeated add() calls. Not references or
// function types.
template <class T>
concept Component = std::is_object_v<T> && std::is_move_constructible_v<T> && std::is_move_assignable_v<T>;

namespace detail {

// Type-erased handle onto a ComponentStorage<T> so Registry can walk every
// registered component type without knowing T at the call site -- needed
// to strip an entity's components when that entity is destroyed.
class IComponentStorage {
public:
    virtual ~IComponentStorage() = default;

    // Removes the component at `entity_index` if present. Returns whether
    // anything was actually removed (false = entity never had this type).
    virtual bool remove(uint32_t entity_index) = 0;
};

// Sparse-set storage for one component type. `pool` is the slot-stable
// value storage with its own free-list and its own slot numbering -- NOT
// the same slot numbers as the owning entities. `sparse` maps an entity's
// slot index to that entity's slot in `pool`, kInvalidSlot when the entity
// has no component of this type. Removing a component tombstones its
// Pool<T> slot (via Pool<T>::destroy) instead of compacting `pool`, so a
// live component's pool slot never moves because some other entity's
// component of the same type was removed elsewhere.
template <class T>
class ComponentStorage final : public IComponentStorage {
public:
    static constexpr uint32_t kInvalidSlot = std::numeric_limits<uint32_t>::max();

    Pool<T> pool;
    std::vector<uint32_t> sparse;

    bool remove(uint32_t entity_index) override {
        if (entity_index >= sparse.size() || sparse[entity_index] == kInvalidSlot) return false;
        pool.destroy(sparse[entity_index]);
        sparse[entity_index] = kInvalidSlot;
        return true;
    }
};

}  // namespace detail

// Owns entity identity (generational slots) and per-type component storage.
//
// Step-boundary protocol for destroy(): Registry works with or without
// begin_step()/end_step() bracketing.
//   - Outside a begin_step()/end_step() pair, destroy() applies
//     immediately: the entity's slot is freed and its generation bumped
//     before destroy() returns.
//   - Between begin_step() and end_step(), destroy() QUEUES: it validates
//     the handle and returns success, but the entity stays alive
//     (is_valid() still true, its components still gettable) until
//     end_step() flushes the queue. Queued destroys are applied in queue
//     (call) order, each with its own generation bump, when end_step()
//     runs. Queuing the same handle more than once within one step is
//     idempotent -- one flush, one generation bump -- so a caller never
//     needs to de-duplicate its own destroy() calls.
// Task 13's Simulation calls begin_step()/end_step() around its per-tick
// pass schedule, so every destroy() issued *during* a step stays visible to
// every system in that same step (nothing disappears mid-pass) while still
// taking effect by the next step.
//
// create() is unconditional and immediate in both modes: per the brief,
// its signature is `create() -> EntityHandle`, not step-gated. Task 5's
// Registry does not defer spawns; a fuller structural queue that also
// defers spawn/world-load is Task 13's concern, layered on top of this.
// add()/get()/remove() are likewise always immediate -- only entity
// destruction has queued semantics here.
//
// Not copyable: component storage is held behind type-erased
// unique_ptr<IComponentStorage>, and giving Registry a deep-copy would need
// a virtual clone() per component type that nothing in this plan needs yet
// (YAGNI). Movable, so it can still be built up and handed off by value.
class Registry {
public:
    Registry() = default;
    ~Registry() = default;
    Registry(const Registry&) = delete;
    Registry& operator=(const Registry&) = delete;
    Registry(Registry&&) = default;
    Registry& operator=(Registry&&) = default;

    // Allocates an entity: reuses the most-recently-freed slot if the
    // free-list is non-empty, else appends a new slot.
    //
    // Generation is bumped exactly once per destroy-then-reuse cycle, and
    // that bump happens at destroy *application* (apply_destroy(), below),
    // not here -- per the step-boundary contract, end_step() applies each
    // queued destroy "with the generation bump", so the bump is destroy's
    // responsibility. The one exception is a slot's very first-ever use:
    // it has no prior destroy to have bumped it, so create() sets its
    // generation to 1 directly here (0 means "never issued" per
    // Handle<Tag>::is_null(), so the first real handle must not be 0).
    EntityHandle create() {
        uint32_t index;
        if (!free_entities_.empty()) {
            index = free_entities_.back();
            free_entities_.pop_back();
        } else {
            index = static_cast<uint32_t>(entities_.size());
            entities_.push_back(EntitySlot{.generation = 1});
        }
        EntitySlot& slot = entities_[index];
        slot.alive = true;
        slot.destroy_pending = false;
        return EntityHandle{.index = index, .generation = slot.generation};
    }

    // Destroys `handle`: immediate outside a step, queued between
    // begin_step()/end_step() (see class docs). Returns not_found if
    // `handle` is already stale -- wrong generation, out of range, or
    // already dead.
    Result<void> destroy(EntityHandle handle) {
        if (!is_valid(handle)) {
            return std::unexpected(Error{Code::not_found, "Registry::destroy: stale entity handle"});
        }
        if (in_step_) {
            EntitySlot& slot = entities_[handle.index];
            if (!slot.destroy_pending) {
                slot.destroy_pending = true;
                destroy_queue_.push_back(handle.index);
            }
            return {};
        }
        apply_destroy(handle.index);
        return {};
    }

    [[nodiscard]] bool is_valid(EntityHandle handle) const noexcept {
        if (handle.is_null()) return false;
        if (handle.index >= entities_.size()) return false;
        const EntitySlot& slot = entities_[handle.index];
        return slot.alive && slot.generation == handle.generation;
    }

    // Opens a step: destroy() starts queuing instead of applying
    // immediately. Callers (Task 13's Simulation) are expected to pair
    // this with a matching end_step() -- Registry is externally
    // synchronized to one caller thread, same as the rest of the engine.
    void begin_step() noexcept { in_step_ = true; }

    // Closes a step: applies every queued destroy, in queue order, each
    // with its own generation bump, then resumes immediate destroy().
    void end_step() {
        in_step_ = false;
        for (uint32_t index : destroy_queue_) {
            apply_destroy(index);
        }
        destroy_queue_.clear();
    }

    // Attaches `value` as entity `handle`'s component of type T, replacing
    // any existing component of that type in place (same pool slot, no
    // realloc) rather than erroring on a duplicate add.
    template <Component T>
    Result<void> add(EntityHandle handle, T value) {
        if (!is_valid(handle)) {
            return std::unexpected(Error{Code::not_found, "Registry::add: stale entity handle"});
        }
        using Storage = detail::ComponentStorage<T>;
        Storage& storage = storage_for<T>();
        if (handle.index < storage.sparse.size() && storage.sparse[handle.index] != Storage::kInvalidSlot) {
            *storage.pool.get(storage.sparse[handle.index]) = std::move(value);
            return {};
        }
        if (handle.index >= storage.sparse.size()) {
            storage.sparse.resize(handle.index + 1, Storage::kInvalidSlot);
        }
        storage.sparse[handle.index] = storage.pool.create(std::move(value));
        return {};
    }

    // Returns a pointer to entity `handle`'s component of type T. Fails
    // with not_found for a stale handle or for a live entity that has no
    // component of this type (including a type never registered at all).
    // POINTER INVALIDATION: the returned T* points into Pool<T>'s backing
    // vector<optional<T>> and is invalidated by any subsequent add<T>() on
    // *any* entity that grows that vector (Pool<T>::create() appending past
    // its current capacity) -- not just a call on this same handle. Slot
    // *indices* stay stable (see pool.hpp); raw pointers returned by get()
    // do not. Do not hold a get() result across an add<T>() call; re-fetch
    // it instead.
    template <Component T>
    Result<T*> get(EntityHandle handle) {
        if (!is_valid(handle)) {
            return std::unexpected(Error{Code::not_found, "Registry::get: stale entity handle"});
        }
        auto it = storages_.find(component_id<T>());
        if (it == storages_.end()) {
            return std::unexpected(Error{Code::not_found, "Registry::get: component type never registered"});
        }
        using Storage = detail::ComponentStorage<T>;
        auto& storage = static_cast<Storage&>(*it->second);
        if (handle.index >= storage.sparse.size() || storage.sparse[handle.index] == Storage::kInvalidSlot) {
            return std::unexpected(Error{Code::not_found, "Registry::get: entity has no component of this type"});
        }
        return storage.pool.get(storage.sparse[handle.index]);
    }

    // Detaches entity `handle`'s component of type T. Fails with not_found
    // for a stale handle or if the entity has no component of this type.
    template <Component T>
    Result<void> remove(EntityHandle handle) {
        if (!is_valid(handle)) {
            return std::unexpected(Error{Code::not_found, "Registry::remove: stale entity handle"});
        }
        auto it = storages_.find(component_id<T>());
        if (it == storages_.end() || !it->second->remove(handle.index)) {
            return std::unexpected(Error{Code::not_found, "Registry::remove: entity has no component of this type"});
        }
        return {};
    }

private:
    struct EntitySlot {
        uint32_t generation = 0;
        bool alive = false;
        bool destroy_pending = false;
    };

    // Frees `index`'s components across every registered component type,
    // then frees the entity slot itself and bumps its generation. Guarded
    // by `alive` so applying an already-applied destroy (defensive; the
    // queue is de-duplicated at destroy()-call time already) is a no-op
    // rather than a double free-list push or a double generation bump.
    void apply_destroy(uint32_t index) {
        EntitySlot& slot = entities_[index];
        if (!slot.alive) return;
        for (auto& [id, storage] : storages_) {
            storage->remove(index);
        }
        slot.alive = false;
        slot.destroy_pending = false;
        slot.generation += 1;
        free_entities_.push_back(index);
    }

    template <Component T>
    detail::ComponentStorage<T>& storage_for() {
        ComponentId id = component_id<T>();
        auto it = storages_.find(id);
        if (it == storages_.end()) {
            auto owned = std::make_unique<detail::ComponentStorage<T>>();
            auto* raw = owned.get();
            storages_.emplace(id, std::move(owned));
            return *raw;
        }
        return static_cast<detail::ComponentStorage<T>&>(*it->second);
    }

    std::vector<EntitySlot> entities_;
    std::vector<uint32_t> free_entities_;

    bool in_step_ = false;
    std::vector<uint32_t> destroy_queue_;

    std::unordered_map<ComponentId, std::unique_ptr<detail::IComponentStorage>> storages_;
};

}  // namespace spade
