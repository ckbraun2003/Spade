#pragma once

// ---------------------------------------------------------------------------
// grid_entry.hpp (S6 Task 7) -- the DEVICE ROW of the dynamic broad phase's
// sorted key array, plus the one function that decides how big that array is
// and how it is partitioned.
//
// VULKAN-FREE, deliberately, exactly like compute/sdf_program.hpp and
// compute/step_params.hpp beside it: this header is #included by the generated
// layout_check.gen.hpp (compiled into compute/layout_check.cpp), so pulling
// <volk.h> in to reach a 24-byte POD would be the same mistake step_params.hpp
// exists to avoid.
//
// ===========================================================================
// WHAT A GRID ENTRY IS
//
// physics/grid.hpp's GridEntry -- `{world, cell, slot}` -- is the CPU pass's
// own key record, and physics/grid.cpp:329-335's grid_entry_less() is the
// TOTAL order it is sorted by:
//
//     (world, cell.z, cell.y, cell.x, slot)
//
// GridEntryRow below is that record's device image. It is NOT a mirror of
// physics::GridEntry in the layouts.slang sense: GridEntry holds a nested
// GridCell whose std430 image would be three separate offsets to assert, and
// the CPU struct is a std::vector element with no layout discipline of its own
// (it is scratch, explicitly NOT registered state -- grid.hpp's GridScratch
// note). So the device row is declared HERE, flat, with the layout battery
// state/layout.hpp's style asks for, and layouts.slang mirrors THIS.
//
// ===========================================================================
// WHY THE DEVICE NEEDS A BUFFER FOR IT AT ALL, AND WHY THAT IS LEGAL
//
// The CPU pass keeps its keys in GridScratch (a caller-owned std::vector pair).
// The GPU cannot: the build, the sort and the resolve are three separate
// dispatches with barriers between them, so the keys have to live in device
// memory between them. That buffer (bindings.slang binding 21) is
// DERIVED, BACKEND-INTERNAL storage in exactly the sense the global constraint
// sanctions -- "GPU mirrors/staging/descriptors are backend-internal derived
// storage", "S6 adds NO register_array call". It is never snapshotted, never
// digested, never uploaded from the host, and is rebuilt from scratch by the
// first dispatch of every CollisionDynamic pass, which is the same lifetime
// GridScratch has on the CPU ("they carry no meaning ACROSS calls").
//
// ===========================================================================
// THE SENTINEL, AND WHY EVERY SLOT OF THE DOMAIN CARRIES A KEY
//
// std::sort takes a size; a bitonic network takes a POWER OF TWO. So the
// device domain is padded, and every padding lane -- along with every body the
// CPU's build stage SKIPS (inactive, freed, or with no representable cell;
// grid.cpp:370-381) -- is written a SENTINEL key whose world is
// kGridInvalidWorld (0xFFFFFFFF == state/arenas.hpp's kInvalidWorld).
//
// Two properties make that exactly equivalent to the CPU's "these entries do
// not exist":
//
//   * kGridInvalidWorld is GREATER, as an unsigned compare, than every real
//     world id (world ids are < world_count, and world_count * body_capacity
//     must fit a uint32 -- sim/world_set.cpp's slot-limit check), so every
//     sentinel sorts strictly AFTER every live key. The live prefix of the
//     sorted array is therefore byte-for-byte the CPU's sorted `entries`
//     vector, and the resolve stage simply never walks past it.
//   * `slot` is set to the entry's OWN INDEX for a sentinel, which makes the
//     order TOTAL over the padded domain too (indices are unique). That is
//     what physics/grid.hpp calls "the whole determinism argument for this
//     pass -- there is nothing left for an implementation to choose": with a
//     total order the sorted permutation is unique, so the bitonic network's
//     result cannot depend on the network, the local size, or the schedule.
//
// ===========================================================================
// THE TWO DOMAIN SHAPES -- `batch_dynamic_collision` IS A DISPATCH-SHAPE
// DECISION AND NOTHING ELSE (checkpoint-1 ruling).
//
// physics/schedule.cpp:59-73 runs the CPU pass in one of two shapes:
//
//   BATCHED   (WorldSetLayout::uniform_dynamic_params): ONE sweep over
//             ctx.all_bodies, keys leading with the world id, cross-world
//             pairs structurally impossible (D8).
//   PER-WORLD: one sweep per world, over that world's slice.
//
// grid_domain_of() below turns that flag into a partition of the device key
// array, and NOTHING ELSE. In particular it does NOT select where the pass
// reads its material from: both shapes index `contact_params[world]` and
// `grid_params[world]` per world (bindings 19/20), because a per-batch record
// was removed by Task 6b and is not coming back.
//
//   BATCHED   segment = next_pow2(world_count * body_capacity)
//             segment_slots = world_count * body_capacity
//             entry_count = segment            -- ONE segment, sorted whole
//   PER-WORLD segment = next_pow2(body_capacity)
//             segment_slots = body_capacity
//             entry_count = world_count * segment  -- N segments, each sorted
//                                                     independently
//
// AND THE TWO PRODUCE THE SAME PHYSICS, which is worth writing down because it
// is the claim the two shower parity tests measure rather than assume. Within
// one world the two orders agree: the batched key leads with a world id that
// is constant across that world's entries, so it reduces to (z, y, x, slot),
// and the global slot w*body_capacity + local is strictly monotone in the
// local slot the per-world sweep uses -- so the same pairs are visited in the
// same sequence, and `ea.slot < eb.slot` picks the same half of each pair.
// Across worlds there is nothing to agree about: find_run() compares the world
// id exactly, so no pair ever spans two worlds, and the batched sweep is
// therefore literally world 0's per-world sweep followed by world 1's.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "compute/backend.hpp"

namespace spade::compute {

// state/arenas.hpp's kInvalidWorld, restated here rather than included: this
// header is on layout_check.cpp's include path and must stay as small as
// step_params.hpp is. tests/test_gpu_parity.cpp asserts the two are equal, so
// the restatement cannot drift.
inline constexpr uint32_t kGridInvalidWorld = 0xFFFFFFFFu;

// ---------------------------------------------------------------------------
// One sorted key. Six 4-byte lanes, 24 bytes, std430 alignment 4 (no vector
// member) -- see layouts.slang's GridEntryRow for the mirror the generated
// asserts hold this to.
//
// `cell_*` are SIGNED and unbounded, matching physics/grid.hpp's GridCell (its
// FIX 3: v1 clamped cell indices into [0, gridDim) and thereby made every
// far-away body a mutual neighbour of every other).
// ---------------------------------------------------------------------------
struct GridEntryRow {
    uint32_t world = kGridInvalidWorld;  // owning world, or the sentinel
    int32_t cell_x = 0;                  // floor(pos.x / cell_size)
    int32_t cell_y = 0;
    int32_t cell_z = 0;
    uint32_t slot = 0;  // GLOBAL body slot for a live entry; the entry's own index for a sentinel
    uint32_t _r0 = 0;   // reserved; must stay 0 (and what makes every byte belong to a named field)
};

static_assert(std::is_standard_layout_v<GridEntryRow>);
static_assert(std::is_trivially_copyable_v<GridEntryRow>);
static_assert(sizeof(GridEntryRow) == 24, "std430 array stride: six 4-byte lanes, no tail pad");
static_assert(alignof(GridEntryRow) == 4, "std430 base alignment for an all-scalar struct");
static_assert(offsetof(GridEntryRow, world) == 0);
static_assert(offsetof(GridEntryRow, cell_x) == 4);
static_assert(offsetof(GridEntryRow, cell_y) == 8);
static_assert(offsetof(GridEntryRow, cell_z) == 12);
static_assert(offsetof(GridEntryRow, slot) == 16);
static_assert(offsetof(GridEntryRow, _r0) == 20);
static_assert(sizeof(GridEntryRow::world) + sizeof(GridEntryRow::cell_x) + sizeof(GridEntryRow::cell_y) +
                      sizeof(GridEntryRow::cell_z) + sizeof(GridEntryRow::slot) + sizeof(GridEntryRow::_r0) ==
                  sizeof(GridEntryRow),
              "GridEntryRow has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// How the device key array is sized and partitioned for one StepShape. Every
// field is ENTRIES, not bytes.
//
// `ok == false` means the shape asks for a key array this backend refuses to
// allocate -- see kMaxGridEntries below. StepRecorder/StateMirror both turn
// that into Code::capacity_exceeded rather than allocating 48 GB and finding
// out from the driver.
// ---------------------------------------------------------------------------
struct GridDomain {
    uint32_t entry_count = 0;    // N: total entries (segments * segment)
    uint32_t segment = 0;        // entries per independently-sorted segment; a power of two, >= 1
    uint32_t segment_slots = 0;  // body slots mapped into one segment
    bool ok = true;
};

// A ceiling that is absurd for this engine and finite for the allocator: 2^26
// entries is 1.6 GB of keys, against a device with ~1 GB of shared memory. A
// shape that reaches it is a configuration error, not a workload.
inline constexpr uint64_t kMaxGridEntries = 1ull << 26;

// Smallest power of two >= v, with next_pow2(0) == 1 so `segment` is never
// zero and the kernels' `e / segment` is never a division by zero. Computed in
// 64 bits and returned narrowed by the caller's own ceiling check.
[[nodiscard]] constexpr uint64_t next_pow2_u64(uint64_t v) noexcept {
    uint64_t p = 1;
    while (p < v) p <<= 1;
    return p;
}

[[nodiscard]] constexpr GridDomain grid_domain_of(const StepShape& shape) noexcept {
    const uint64_t slots = static_cast<uint64_t>(shape.world_count) * static_cast<uint64_t>(shape.body_capacity);

    // The two shapes, exactly as this file's header sets them out.
    const uint64_t segment_slots = shape.batch_dynamic_collision ? slots : shape.body_capacity;
    const uint64_t segments = shape.batch_dynamic_collision ? 1u : shape.world_count;
    const uint64_t segment = next_pow2_u64(segment_slots);
    const uint64_t entries = segments * segment;

    GridDomain domain{};
    if (entries > kMaxGridEntries) {
        domain.ok = false;
        return domain;
    }
    domain.entry_count = static_cast<uint32_t>(entries);
    domain.segment = static_cast<uint32_t>(segment);
    domain.segment_slots = static_cast<uint32_t>(segment_slots);
    return domain;
}

}  // namespace spade::compute
