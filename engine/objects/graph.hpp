// graph.hpp -- the object pool (24th spec SL3, Plan A Task 3).
//
// ObjectGraph owns the objects an ObjectId names: a recycling slot pool with
// generational handles, so a stale id is REJECTED rather than silently
// addressing whatever now occupies its slot.
//
// SL3, RESTATED AS A CONSTRAINT ON THIS CLASS
// ---------------------------------------------------------------------------
// The object graph is identity and composition, NEVER simulation state.
// Nothing here influences GPU buffer order, pass order, or any value the
// physics computes -- slot assignment in the spade_state arenas does that, and
// this type does not touch it. That is precisely why a structural addition
// this large costs the snapshot and parity estates nothing: `kSnapshotVersion`
// is unchanged and the registry walk does not know this class exists.
//
// GENERATION PARITY: ODD == LIVE, EVEN == DEAD, 0 == NEVER ISSUED
// ---------------------------------------------------------------------------
// Deliberately the same rule as BodyRef's (sim/simulation.hpp), so there is
// ONE convention to learn across the engine rather than two that look alike
// and differ in a corner. create() takes a slot's counter even -> odd,
// destroy() takes it odd -> even.
//
// WHERE IT DELIBERATELY DIVERGES FROM BodyRef, AND WHY -- this is the part a
// reader who knows BodyRef will otherwise get backwards. BodyRef's generations
// live in a REGISTERED array ("body_generation") specifically so they survive
// a snapshot/restore round trip like every other piece of engine state. These
// generations live in a plain member and are NOT registered, and that is not
// an oversight: registering them would put the object graph inside the
// snapshot, which is the one thing SL3 forbids. The counters are rebuilt with
// the graph from its saved description (Task 6), so nothing is lost -- but an
// ObjectId is only meaningful against the graph instance that issued it, and
// does NOT survive a snapshot restore the way a BodyRef does.
//
// POINTER LIFETIME: get() RETURNS A VIEW, NOT A REFERENCE TO HOLD
// ---------------------------------------------------------------------------
// Slots live in a std::vector, so any create() that grows the pool invalidates
// every pointer get() has handed out. The ObjectId is the stable reference;
// the pointer is valid only until the next mutation. Store ids, resolve late.
//
// WHAT THIS CLASS DOES NOT DO: HIERARCHY
// ---------------------------------------------------------------------------
// `Object::parent` is a plain field callers write through get(). This class
// neither validates it nor maintains it: it does not reject a cycle, and
// destroy() does NOT null the parent of the destroyed object's children. A
// child therefore keeps naming a dead handle, which alive() correctly reports
// as dead -- so the dangling state is DETECTABLE, never silently reattached to
// whatever recycles that slot. That is the property worth having at this
// layer, and it is why leaving hierarchy unowned here is safe rather than
// merely unfinished.
//
// It is not free, though, and the bill lands on the serializer: the serializer
// writes `parent` as the parent's INDEX in the objects array, and an object
// whose parent was destroyed has no index to write. to_json() refuses it with
// an Error rather than silently re-rooting the child (serialize.hpp).

#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "objects/component.hpp"
#include "objects/object.hpp"

namespace spade::objects {

class ObjectGraph {
  public:
    // Allocates an object and returns a live handle to it. Reuses the most
    // recently freed slot when one exists.
    [[nodiscard]] ObjectId create(std::string_view name);

    // Frees the slot `id` names. Returns false -- rather than doing nothing
    // quietly -- for a null, out-of-range, or already-destroyed handle, so a
    // double-destroy is a reportable event at the call site.
    bool destroy(ObjectId id);

    [[nodiscard]] bool alive(ObjectId id) const noexcept;

    // nullptr for any handle alive() rejects. See the pointer-lifetime note
    // above before storing the result.
    [[nodiscard]] Object* get(ObjectId id) noexcept;
    [[nodiscard]] const Object* get(ObjectId id) const noexcept;

    // LIVE objects, not slots ever allocated -- the two diverge the moment a
    // slot is recycled.
    [[nodiscard]] std::size_t size() const noexcept { return live_count_; }

    // Every live object, in SLOT ORDER. The order is the point: it is a
    // property of the graph alone, not of the order objects happened to be
    // created in, so a serialized graph is byte-stable for a given graph. The
    // callback receives an ObjectId -- resolve it with get() rather than
    // holding a pointer across the walk.
    template <class Fn>
    void for_each(Fn&& fn) const {
        for (uint32_t i = 0; i < static_cast<uint32_t>(generations_.size()); ++i) {
            const ObjectId id{.index = i, .generation = generations_[i]};
            if (alive(id)) fn(id);
        }
    }

    // -----------------------------------------------------------------------
    // Components (Task 4). One parallel store per type indexed by object slot,
    // plus a per-object bitmask over the ten type ids. The MASK is the source
    // of truth for "does this object have a T": a store always holds a value
    // at every live slot, so only the bit distinguishes "attached" from
    // "never attached", and every accessor below consults it before it reads.
    // -----------------------------------------------------------------------

    // Attaching over an existing T of the same type overwrites it. Returns
    // false for a handle alive() rejects, so a component cannot be hung on a
    // dead object.
    template <class T>
    bool attach(ObjectId id, T value) {
        if (!alive(id)) return false;
        auto& store = storage<T>();
        // Stores grow lazily: a graph that never attaches a FluidComponent
        // never allocates the fluid store at all. Sizing to slots_ here is
        // safe for every later read because id.index < slots_.size() held at
        // this point and slots_ never shrinks.
        if (store.size() < slots_.size()) store.resize(slots_.size());
        store[id.index] = value;
        masks_[id.index] |= mask_bit(component_type_id<T>());
        return true;
    }

    // Clears the type's bit. It deliberately does NOT overwrite the stored
    // value: the bit already makes it unreachable through component<T>(), so
    // scrubbing would be a second site maintaining the same invariant and
    // neither site could then be shown to fail (SL18) -- the same reasoning
    // that put the slot reset in create() alone. A later attach<T>() assigns
    // over whatever remains.
    template <class T>
    bool detach(ObjectId id) {
        if (!alive(id) || !has(id, component_type_id<T>())) return false;
        masks_[id.index] &= ~mask_bit(component_type_id<T>());
        return true;
    }

    // nullptr unless the object is alive AND carries this type. Subject to the
    // same pointer-lifetime rule as get(): a later attach() of the same type
    // can reallocate the store.
    template <class T>
    [[nodiscard]] T* component(ObjectId id) noexcept {
        if (!alive(id) || !has(id, component_type_id<T>())) return nullptr;
        return &storage<T>()[id.index];
    }

    // The const overload delegates through const_cast on `this` rather than
    // duplicating ten const storage() specializations. Safe by inspection: the
    // non-const body above reads only, and the result is handed back const.
    template <class T>
    [[nodiscard]] const T* component(ObjectId id) const noexcept {
        return const_cast<ObjectGraph*>(this)->component<T>(id);
    }

    [[nodiscard]] bool has(ObjectId id, ComponentTypeId type) const noexcept {
        if (!alive(id)) return false;
        return (masks_[id.index] & mask_bit(type)) != 0u;
    }

    // Bit N is type id N -- the layout Plan C's inspector and any future save
    // format read. 0 for a dead or unknown handle.
    [[nodiscard]] uint32_t component_mask(ObjectId id) const noexcept {
        return alive(id) ? masks_[id.index] : 0u;
    }

  private:
    static constexpr uint32_t mask_bit(ComponentTypeId t) noexcept {
        return 1u << static_cast<uint32_t>(t);
    }
    static_assert(kComponentTypeCount <= 32u,
                  "the component mask is a uint32; a 33rd component type needs a wider mask, "
                  "not a bit that shifts off the end and is silently dropped");

    // A function-local static per instantiation would be shared across every
    // ObjectGraph, so these are members reached through an explicit accessor
    // specialization -- written out one per type below rather than macro-
    // generated, so a reader can see every mapping.
    template <class T>
    std::vector<T>& storage() noexcept;

    std::vector<Object> slots_;
    std::vector<uint32_t> generations_;
    std::vector<uint32_t> free_list_;
    std::vector<uint32_t> masks_;
    std::size_t live_count_ = 0;

    std::vector<TransformComponent> transforms_;
    std::vector<BodyComponent> bodies_;
    std::vector<MeshComponent> meshes_;
    std::vector<MaterialComponent> materials_;
    std::vector<ColliderComponent> colliders_;
    std::vector<SensorComponent> sensors_;
    std::vector<ForceElementComponent> force_elements_;
    std::vector<CameraComponent> cameras_;
    std::vector<BehaviorComponent> behaviors_;
    std::vector<FluidComponent> fluids_;
};

// The primary template above is declared and never defined, so a type with no
// specialization here is a hard error rather than a silent default. It is
// reached second, though: attach/detach/component all instantiate
// component_type_id<T>() as well, and ComponentTraits<T> fails to COMPILE for
// an unregistered type -- which is the better diagnostic of the two.
template <> inline std::vector<TransformComponent>& ObjectGraph::storage() noexcept { return transforms_; }
template <> inline std::vector<BodyComponent>& ObjectGraph::storage() noexcept { return bodies_; }
template <> inline std::vector<MeshComponent>& ObjectGraph::storage() noexcept { return meshes_; }
template <> inline std::vector<MaterialComponent>& ObjectGraph::storage() noexcept { return materials_; }
template <> inline std::vector<ColliderComponent>& ObjectGraph::storage() noexcept { return colliders_; }
template <> inline std::vector<SensorComponent>& ObjectGraph::storage() noexcept { return sensors_; }
template <> inline std::vector<ForceElementComponent>& ObjectGraph::storage() noexcept { return force_elements_; }
template <> inline std::vector<CameraComponent>& ObjectGraph::storage() noexcept { return cameras_; }
template <> inline std::vector<BehaviorComponent>& ObjectGraph::storage() noexcept { return behaviors_; }
template <> inline std::vector<FluidComponent>& ObjectGraph::storage() noexcept { return fluids_; }

}  // namespace spade::objects
