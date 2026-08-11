#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "core/error.hpp"

namespace spade {

// ---------------------------------------------------------------------------
// One authoritative state array, as a snapshot walk sees it (engine design
// spec §4 "Snapshot (P8) and readback": "The state registry records every
// authoritative array (name, element type, per-world extents). Snapshot =
// registry walk -> versioned blob").
//
// `name` is the array's stable identity. It is what a snapshot blob stores;
// nothing persistent may key on an array's position in the walk or on a
// runtime-assigned id, for the same reason ecs/registry.hpp forbids
// serializing a raw ComponentId -- those are per-process facts, names are
// not.
//
// `data` is deliberately a mutable pointer even though for_each_array hands
// out a CONST RegisteredArray&: snapshot SAVE reads through it, snapshot
// RESTORE writes through it, and both are registry walks. Constness here
// protects the descriptor (you cannot re-point or resize an array through
// the walk), not the bytes.
// ---------------------------------------------------------------------------
struct RegisteredArray {
    std::string name;
    uint32_t elem_size = 0;           // sizeof(T)
    uint32_t world_count = 0;         // number of world partitions
    uint32_t capacity_per_world = 0;  // per-world extent, in elements
    std::byte* data = nullptr;        // first byte of world 0's partition

    // Total elements across all worlds. Derived, not stored, so the
    // per-world extents stay the single source of truth.
    [[nodiscard]] uint32_t element_count() const noexcept { return world_count * capacity_per_world; }

    [[nodiscard]] std::size_t byte_size() const noexcept {
        return static_cast<std::size_t>(elem_size) * element_count();
    }
};

// ---------------------------------------------------------------------------
// The registered-arrays walker. NOT the ECS entity registry (ecs/registry.hpp
// -- that one owns entity identity and components); this one owns the answer
// to "what is all of the authoritative state?", which is the question a
// snapshot has to get exactly right.
//
// THE "NO UNREGISTERED STATE" INVARIANT, stated precisely:
//   * ArenaSet has exactly one way to obtain storage -- register_array() --
//     and that call registers the array here as part of allocating it. There
//     is no ArenaSet API that hands out arena bytes without a registry
//     entry, so "arena state a snapshot walk would miss" is structurally
//     impossible rather than merely discouraged.
//   * The converse does NOT hold, on purpose: StateRegistry also accepts
//     arrays it does not own (register_array below), because Task 7 must put
//     non-arena state -- rng stream states, for one -- into the same walk.
//     So: everything in an arena is registered; not everything registered
//     lives in an arena.
//
// Walk order is registration order and is deterministic, but it is NOT part
// of any wire contract -- blobs key entries by name (see RegisteredArray).
//
// Externally synchronized to one caller thread, like the rest of the engine.
// ---------------------------------------------------------------------------
class StateRegistry {
public:
    StateRegistry() = default;
    ~StateRegistry() = default;
    StateRegistry(const StateRegistry&) = default;
    StateRegistry& operator=(const StateRegistry&) = default;
    StateRegistry(StateRegistry&&) = default;
    StateRegistry& operator=(StateRegistry&&) = default;

    // Adds `desc` to the walk. Fails with invalid_argument on an empty or
    // duplicate name, a zero elem_size, or a null `data` for a non-empty
    // array -- each of which would produce a snapshot entry that cannot be
    // read back.
    Result<void> register_array(RegisteredArray desc);

    // Adds several arrays as ONE unit: every descriptor is validated (against
    // what is already registered and against the rest of the batch) before
    // any of them is committed, so the call either adds all or adds none.
    //
    // This exists because one ArenaSet array contributes two entries -- its
    // elements and its slot_to_world map -- and a half-registered arena
    // would be exactly the "state the walk misses" this class is here to
    // prevent. There is deliberately no unregister, so atomicity has to come
    // from validating up front rather than from rolling back.
    Result<void> register_arrays(std::vector<RegisteredArray> descs);

    // Calls `visitor(const RegisteredArray&)` once for every registered
    // array, in registration order. This is the snapshot walk.
    template <class F>
    void for_each_array(F&& visitor) const {
        for (const RegisteredArray& array : arrays_) {
            visitor(array);
        }
    }

    [[nodiscard]] std::size_t size() const noexcept { return arrays_.size(); }

    // Lookup by the stable identity. Returns nullptr if absent. The pointer
    // is invalidated by a later register_array().
    [[nodiscard]] const RegisteredArray* find(std::string_view name) const noexcept;

    // Sum of byte_size() over the whole walk -- the payload size of a
    // snapshot blob, minus its header.
    [[nodiscard]] std::size_t total_bytes() const noexcept;

private:
    // Everything that makes a descriptor unusable to a snapshot walk.
    // Shared by both registration entry points so they cannot drift apart.
    [[nodiscard]] Result<void> validate(const RegisteredArray& desc) const;

    std::vector<RegisteredArray> arrays_;
};

}  // namespace spade
