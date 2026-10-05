#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <new>
#include <queue>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "core/error.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"

namespace spade {

// Sentinel stored in an array's slot_to_world map for a slot that is not
// currently allocated. Doubles as the liveness predicate: a slot is live iff
// slot_to_world[slot] != kInvalidWorld.
inline constexpr uint32_t kInvalidWorld = std::numeric_limits<uint32_t>::max();

inline constexpr uint32_t kInvalidArrayIndex = std::numeric_limits<uint32_t>::max();

// Suffix ArenaSet appends to an array's name to name that array's
// slot_to_world map in the state registry: register_array("bodies", ...)
// contributes both "bodies" and "bodies.slot_to_world" to the snapshot walk.
inline constexpr std::string_view kSlotToWorldSuffix = ".slot_to_world";

// ---------------------------------------------------------------------------
// One world's contiguous slot partition inside one global array.
//
// `count` is the partition's SLOT COUNT -- this world's fixed capacity for
// this array -- NOT its live population. Slots inside [begin, begin + count)
// are individually allocated and freed, so live slots are generally not
// contiguous; use ArenaSet::live_count() for the population and the
// slot_to_world map for per-slot liveness.
//
// This is the range a GPU dispatch bounds-checks against so that
// cross-world interaction is structurally impossible (engine design spec
// D8, §4 "Many-worlds").
// ---------------------------------------------------------------------------
struct WorldRange {
    uint32_t begin = 0;
    uint32_t count = 0;

    friend constexpr bool operator==(const WorldRange&, const WorldRange&) noexcept = default;
};

// Untyped array identity: what ArenaSet's partition and slot bookkeeping
// keys on. Every operation that does not touch element VALUES takes one of
// these.
struct ArrayIndex {
    uint32_t value = kInvalidArrayIndex;

    friend constexpr bool operator==(const ArrayIndex&, const ArrayIndex&) noexcept = default;
};

// Typed array identity. Minted only by ArenaSet::register_array<T>() and by
// ArenaSet::typed<T>() over an array already registered with T's size --
// registration being the only way to obtain arena storage, holding one of
// these is proof that the array it names is in the state registry.
//
// The element type rides along as a phantom parameter (same idea as
// core/ids.hpp's Handle<Tag>) so array()/world_slice() need no cast at the
// call site and cannot be handed the wrong element type. It converts
// implicitly to the untyped ArrayIndex because "which array" is strictly
// less information than "which array, of what element type".
template <class T>
struct ArrayId {
    ArrayIndex index{};

    constexpr operator ArrayIndex() const noexcept { return index; }  // NOLINT(google-explicit-constructor)
    [[nodiscard]] constexpr bool is_null() const noexcept { return index.value == kInvalidArrayIndex; }
};

namespace detail {

// Frees storage obtained from the aligned array operator new[] used by
// ArenaSet. Sized/aligned deallocation must be spelled to match the
// allocation form exactly.
struct AlignedArenaDelete {
    void operator()(std::byte* p) const noexcept {
        ::operator delete[](p, std::align_val_t{kStd430StructAlignment});
    }
};

}  // namespace detail

// ---------------------------------------------------------------------------
// ArenaSet -- the authoritative state storage (engine design spec §4 "State
// layout").
//
// Shape: for each registered array, ONE global allocation of
// world_count * capacity_per_world elements, partitioned WORLD-CONTIGUOUSLY:
// world w owns slots [w * capacity_per_world, (w+1) * capacity_per_world).
// Slot indices returned by alloc_slot() are GLOBAL (indices into the whole
// array), which is what lets one dispatch cover all worlds -- the
// slot_to_world map turns a global slot back into its world id.
//
// Fixed capacity, set once: world_count at construction, capacity_per_world
// at register_array(). Nothing ever reallocates mid-run. Overflowing a
// world's partition returns Code::capacity_exceeded -- v1's "load then never
// again" silent overrun becomes an explicit, honest contract.
//
// DETERMINISM. alloc_slot() always returns the LOWEST free slot in the
// requested world's partition, via an ordered (ascending) free list: a
// min-heap of freed slots plus a bump cursor for slots never yet handed out.
// Every freed slot is below the cursor by construction, so "pop the heap
// minimum if the heap is non-empty, else bump" is exactly lowest-index-first.
// Two runs that issue the same alloc/free sequence therefore get identical
// slot assignments, which is a precondition for the determinism-replay
// corpus (P8) -- there is no hash ordering, no pointer ordering, and no
// wall-clock input anywhere in this class.
//
// ZEROING. Storage is zero-filled at registration and each slot is
// zero-filled again when freed. So an arena's bytes are a function of its
// live state and its alloc/free history alone -- never of heap garbage --
// including the tail padding inside WorldParams (see layout.hpp). Snapshot
// blobs are byte-comparable because of this.
//
// REGISTRATION IS THE ONLY DOOR. register_array() and its untyped twin
// register_bytes() both allocate and register; there is no other way to get
// arena bytes, and no way to unregister. Hence "arena state a snapshot walk
// would miss" cannot be expressed. See registry.hpp for the full statement of
// the invariant.
//
// COMPLETENESS OF THE WALK. An arena holds three kinds of state, and the
// registry walk has to reach all of it:
//   1. the element bytes                -- registered under `name`.
//   2. the slot_to_world map            -- registered under
//      `name + kSlotToWorldSuffix`. It is authoritative, not derived: it is
//      the liveness record, and the spec's §4 "slot->world map buffer" that
//      a single all-worlds dispatch reads. Both entries are added in one
//      atomic StateRegistry::register_arrays() call.
//   3. the per-world free lists and live counts -- NOT registered, because
//      they are a pure function of (2). alloc_slot() is exactly "the lowest
//      slot in this partition whose slot_to_world entry is kInvalidWorld":
//      every freed slot lies below the bump cursor, so popping the ordered
//      free list when it is non-empty and bumping otherwise picks the same
//      slot that scanning the map for the lowest free entry would.
//      resync_from_slot_to_world() performs that derivation, which is what
//      makes (3)'s absence from the walk sound rather than merely convenient.
// The array's shape (element size, world count, per-world capacity) rides in
// the RegisteredArray descriptors themselves.
//
// Move-only. Copying would have to deep-copy every arena and re-point every
// registry entry at the copies; nothing needs that yet (YAGNI). Moving is
// safe and cheap, and that is load-bearing rather than incidental: both
// pointers the registry caches per arena live in heap blocks the Arena only
// REFERENCES -- the element bytes through unique_ptr, the slot_to_world map
// through a vector that is sized once at registration and never resized.
// Moving an Arena (into arenas_, during a reallocation of arenas_, or when
// the whole ArenaSet is moved) transfers those blocks without relocating
// them, so the registry's `data` pointers stay valid. Two tests pin this:
// MovingAnArenaSetKeepsRegistryPointersValid and
// LaterRegistrationsDoNotInvalidateEarlierArrayPointers.
//
// Externally synchronized to one caller thread.
// ---------------------------------------------------------------------------
class ArenaSet {
private:
    using AlignedBytes = std::unique_ptr<std::byte[], detail::AlignedArenaDelete>;

    // Ordered free list + bump cursor for one world's partition of one
    // array. `freed` holds GLOBAL slot indices, minimum first.
    struct WorldSlots {
        uint32_t next_unused = 0;  // partition-local: slots [0, next_unused) have been handed out at least once
        uint32_t live = 0;
        std::priority_queue<uint32_t, std::vector<uint32_t>, std::greater<uint32_t>> freed;
    };

    struct Arena {
        std::string name;
        uint32_t elem_size = 0;
        uint32_t capacity_per_world = 0;
        uint32_t world_count = 0;
        AlignedBytes bytes;                   // total_slots() * elem_size, 16-byte aligned, zero-filled
        std::vector<uint32_t> slot_to_world;  // total_slots() entries; kInvalidWorld == free
        std::vector<WorldSlots> worlds;       // world_count entries

        [[nodiscard]] uint32_t total_slots() const noexcept { return world_count * capacity_per_world; }
    };

public:
    explicit ArenaSet(uint32_t world_count) noexcept : world_count_(world_count) {}

    ~ArenaSet() = default;
    ArenaSet(const ArenaSet&) = delete;
    ArenaSet& operator=(const ArenaSet&) = delete;
    ArenaSet(ArenaSet&&) = default;
    ArenaSet& operator=(ArenaSet&&) = default;

    [[nodiscard]] uint32_t world_count() const noexcept { return world_count_; }

    // How many arrays this set owns. The valid ArrayIndex values are exactly
    // [0, array_count()), which is what lets a snapshot restore resync every
    // arena's derived free lists without a list of its own (state/snapshot.hpp).
    [[nodiscard]] uint32_t array_count() const noexcept { return static_cast<uint32_t>(arenas_.size()); }

    // Allocates and registers one world-partitioned array of T. This is the
    // only way to obtain arena storage.
    //
    // `capacity_per_world` is that array's fixed per-world slot count; the
    // global allocation is world_count() * capacity_per_world elements.
    // Contributes TWO entries to the state registry -- `name` for the
    // elements and `name + kSlotToWorldSuffix` for the slot->world map --
    // added atomically, so a clash on either name registers neither.
    //
    // Fails with invalid_argument on a zero capacity or a duplicate/empty
    // name, and with capacity_exceeded if the request does not fit (element
    // count above 2^32-1, or the allocation itself failing).
    template <class T>
    Result<ArrayId<T>> register_array(std::string name, uint32_t capacity_per_world) {
        // Arena slots are raw bytes that are never constructed or destroyed
        // individually; elements are created implicitly by the allocation
        // (implicit-lifetime types) and copied byte-wise by snapshots.
        static_assert(std::is_trivially_copyable_v<T>, "arena element types must be trivially copyable (snapshots memcpy them)");
        static_assert(std::is_trivially_destructible_v<T>, "arena slots are never individually destroyed");
        static_assert(alignof(T) <= kStd430StructAlignment,
                      "arena element alignment must not exceed the arena's 16-byte allocation alignment");

        Result<ArrayIndex> index = register_array_impl(std::move(name), sizeof(T), capacity_per_world);
        if (!index) return std::unexpected(index.error());
        return ArrayId<T>{*index};
    }

    // register_array<T>()'s registration, untyped: the same two registry
    // entries, the same per-world capacity and the same zero-filled storage
    // for an element of `elem_size` bytes. The module API registers its
    // declared arrays through it (sim/module.hpp's ArrayDecl, whose row_size()
    // carries register_array's three static_asserts to the declaration).
    //
    // Fails as register_array() does, and with invalid_argument on an
    // elem_size of 0.
    Result<ArrayIndex> register_bytes(std::string name, uint32_t elem_size, uint32_t capacity_per_world);

    // A typed id for an array already registered, whatever registered it.
    // not_found for an unknown array; invalid_argument if its element size is
    // not sizeof(T) -- the check every typed accessor below repeats.
    template <class T>
    [[nodiscard]] Result<ArrayId<T>> typed(ArrayIndex array) const {
        static_assert(std::is_trivially_copyable_v<T>, "arena element types must be trivially copyable (snapshots memcpy them)");
        static_assert(std::is_trivially_destructible_v<T>, "arena slots are never individually destroyed");
        static_assert(alignof(T) <= kStd430StructAlignment,
                      "arena element alignment must not exceed the arena's 16-byte allocation alignment");
        Result<const Arena*> arena = arena_for(array, static_cast<uint32_t>(sizeof(T)));
        if (!arena) return std::unexpected(arena.error());
        return ArrayId<T>{array};
    }

    // The whole array's bytes, all worlds, free slots included (they read as
    // zeroes): elem_size * world_count * capacity_per_world of them, global
    // slot i at byte i * elem_size. not_found for an unknown array.
    [[nodiscard]] Result<std::span<std::byte>> bytes(ArrayIndex array);
    [[nodiscard]] Result<std::span<const std::byte>> bytes(ArrayIndex array) const;

    // This world's partition of this array: {begin = world * capacity,
    // count = capacity}. Fails with not_found for an unknown array and
    // invalid_argument for a world id at or above world_count().
    [[nodiscard]] Result<WorldRange> range(ArrayIndex array, uint32_t world) const;

    // Allocates the lowest free slot in `world`'s partition and returns its
    // GLOBAL index. Fails with capacity_exceeded when that partition is
    // full; other worlds are unaffected (capacity is per world, not global).
    Result<uint32_t> alloc_slot(ArrayIndex array, uint32_t world);

    // Frees a global slot index: clears its slot_to_world entry, returns it
    // to its world's ordered free list, and zero-fills its bytes. Fails with
    // invalid_argument if `slot` is out of range and not_found if it is not
    // currently allocated (so double-free is reported, never silent).
    Result<void> free_slot(ArrayIndex array, uint32_t slot);

    // Global slot -> world id, kInvalidWorld for free slots. One entry per
    // slot in the whole array. This is the map a single all-worlds dispatch
    // reads (engine design spec §4).
    [[nodiscard]] Result<std::span<const uint32_t>> slot_to_world(ArrayIndex array) const;

    // Live slot count in one world's partition.
    [[nodiscard]] Result<uint32_t> live_count(ArrayIndex array, uint32_t world) const;

    // Rebuilds this array's per-world free lists and live counts from its
    // slot_to_world map. This is the restore-side half of a snapshot: a
    // blob carries the map (it is in the walk) but not the free lists (they
    // are not), so restoring means overwriting the registered bytes and then
    // calling this.
    //
    // The rebuilt (bump cursor, free list) pair is canonical and need not
    // equal the pre-snapshot pair -- e.g. "cursor at 4, slots 2 and 3 freed"
    // rebuilds as "cursor at 2, nothing freed". Those two encode the same
    // free SET, and alloc_slot() only ever consults the free set, so all
    // future allocations are identical. That equivalence is the proof that
    // the free lists carry no information the walk is missing.
    //
    // Fails with invalid_argument if the map is not self-consistent (an
    // entry naming a world whose partition does not contain that slot),
    // which is the check that a corrupt or mismatched blob trips.
    Result<void> resync_from_slot_to_world(ArrayIndex array);

    // The whole array, all worlds, including free slots (which read as
    // zeroes). Element order is slot order, so index i of this span is
    // global slot i.
    template <class T>
    [[nodiscard]] Result<std::span<T>> array(ArrayId<T> id) {
        Result<const Arena*> arena = arena_for(id.index, sizeof(T));
        if (!arena) return std::unexpected(arena.error());
        return std::span<T>(reinterpret_cast<T*>((*arena)->bytes.get()), (*arena)->total_slots());
    }

    template <class T>
    [[nodiscard]] Result<std::span<const T>> array(ArrayId<T> id) const {
        Result<const Arena*> arena = arena_for(id.index, sizeof(T));
        if (!arena) return std::unexpected(arena.error());
        return std::span<const T>(reinterpret_cast<const T*>((*arena)->bytes.get()), (*arena)->total_slots());
    }

    // One world's partition as a span. Index i of this span is global slot
    // range(array, world)->begin + i.
    template <class T>
    [[nodiscard]] Result<std::span<T>> world_slice(ArrayId<T> id, uint32_t world) {
        Result<std::span<T>> all = array(id);
        if (!all) return all;
        Result<WorldRange> r = range(id.index, world);
        if (!r) return std::unexpected(r.error());
        return all->subspan(r->begin, r->count);
    }

    template <class T>
    [[nodiscard]] Result<std::span<const T>> world_slice(ArrayId<T> id, uint32_t world) const {
        Result<std::span<const T>> all = array(id);
        if (!all) return all;
        Result<WorldRange> r = range(id.index, world);
        if (!r) return std::unexpected(r.error());
        return all->subspan(r->begin, r->count);
    }

    // The snapshot walk over everything this set owns. Every array here was
    // registered by register_array(); there is no other path to arena bytes.
    [[nodiscard]] const StateRegistry& registry() const noexcept { return registry_; }

private:
    Result<ArrayIndex> register_array_impl(std::string name, uint32_t elem_size, uint32_t capacity_per_world);

    // Resolves an array index, checking that the caller's element size
    // matches what was registered -- which catches an ArrayId minted by a
    // DIFFERENT ArenaSet whenever the element types differ in size.
    //
    // Returns a pointer to a const Arena even for the mutating accessors:
    // the Arena DESCRIPTOR is what must not change under a caller, while the
    // bytes it points at are exactly what callers are here to mutate (same
    // constness split as RegisteredArray::data).
    [[nodiscard]] Result<const Arena*> arena_for(ArrayIndex array, uint32_t expected_elem_size) const;

    uint32_t world_count_ = 0;
    std::vector<Arena> arenas_;
    StateRegistry registry_;
};

}  // namespace spade
