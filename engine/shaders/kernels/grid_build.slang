// ---------------------------------------------------------------------------
// grid_build.slang (S6 Task 7) -- STAGE 1 OF THREE of the CollisionDynamic
// pass: one sorted key per active, in-range body. The port of
// physics/grid.cpp:362-381 (the CPU pass's BUILD loop), v1's GridBuild.comp.
//
// ONE THREAD PER KEY-ARRAY ENTRY, not per body slot -- and the difference
// matters, because the entry array is POWER-OF-TWO PADDED (a bitonic network
// needs that; std::sort does not) and every padding lane must be written a
// sentinel by SOMEBODY. Dispatching over the entry domain makes that
// automatic: every lane of the domain is written exactly once, by exactly one
// thread, in the same dispatch, with no separate clear pass and no lane left
// holding the previous substep's key.
//
// ===========================================================================
// WHAT A THREAD WRITES, IN THE CPU'S OWN ORDER OF TESTS
//
// grid.cpp:370-381 skips a body on three conditions, "in increasing order of
// cost: a bit test, an integer compare, and only then the three divisions of
// the cell computation":
//
//     if ((body.flags & body_flags::active) == 0u) continue;         :372
//     const uint32_t world = slot_to_world[i];
//     if (world == kInvalidWorld) continue;                          :376
//     if (!grid_cell_of(body.pos, cell_size, cell)) continue;        :378
//
// This kernel performs the same three, in the same order, and turns each
// `continue` into a SENTINEL write rather than an omission -- see
// compute/grid_entry.hpp for why a sentinel is exactly equivalent to the CPU's
// "this entry does not exist" (it sorts strictly after every live key, because
// the world field is compared unsigned and 0xFFFFFFFF exceeds every real world
// id, so the live prefix of the sorted array is byte-for-byte the CPU's
// sorted vector).
//
// ONE DEPARTURE, AND IT IS AN ORDER-OF-READS ONE ONLY. The CPU has already
// selected a world before it enters this loop, so `cell_size` is a call-wide
// constant it captured at grid.cpp:360. A GPU thread does not know its world
// until it reads the map, so it reads `grid_params[world].cell_size` AFTER the
// world test rather than before the loop. That changes which reads happen, not
// which entries are produced: the flag test and the world test are both
// independent of cell_size, so the set of surviving bodies -- and every key in
// it -- is identical.
//
// PER-WORLD cell_size, NEVER A PER-BATCH ONE (checkpoint-1 ruling): binding
// 20's `grid_params[world]`, exactly as collision_static.slang reads
// `contact_params[world]`. A heterogeneous world set may legitimately want
// different cell sizes per world, and PassParams has had nowhere to carry a
// per-batch record since Task 6b removed the field.
//
// ===========================================================================
// THE SLOT MAPPING, AND WHY IT IS TWO MULTIPLIES RATHER THAN A MODE FLAG
//
// The key domain is `segments` segments of `grid_segment` entries each; a
// segment's first `grid_segment_slots` entries map to body slots and the rest
// are padding. compute/grid_entry.hpp's grid_domain_of() sets the three so
// that BOTH dispatch shapes fall out of the same two lines here:
//
//   BATCHED   one segment, grid_segment_slots = world_count * body_capacity,
//             so seg == 0 and slot == off == the global body slot.
//   PER-WORLD world_count segments, grid_segment_slots = body_capacity, so
//             seg == the world index and slot == seg*body_capacity + off ==
//             the global body slot of that world's local slot `off`.
//
// In both, `slot` ends up the GLOBAL arena slot, which is what makes the key's
// `slot` tiebreak mean the same thing in both shapes (and the same thing the
// CPU's batched sweep means by it).
//
// ===========================================================================
// DIVISION AND SQRT SITES: three OpFDiv, all inside grid_cell_of()'s
// `pos / cell_size` -- the componentwise division grid.cpp:312 spells, and
// which its own comment insists on ("Division rather than a hoisted reciprocal
// multiply: the two are not fp32-equivalent"). No sqrt anywhere in this file.
//
// NO WORKGROUP-SHARED ANYTHING: one entry per thread, no cross-lane traffic,
// so this kernel's result is invariant under the workgroup_size knob by
// construction (T9's A7 obligation, built in now rather than retrofitted).
// ---------------------------------------------------------------------------

import layouts;
import bindings;

static const uint kInvalidWorld = 0xFFFFFFFFu;
static const uint kBodyFlagActive = 1u;

// physics/grid.hpp's kMaxCellCoord. 2e9 rather than INT32_MAX for the two
// reasons stated there: it keeps the float->int conversion defined, and it
// leaves headroom for the +-1 neighbour offsets the resolve stage adds. It is
// exactly representable in fp32 (15625000 * 2^7), so the comparison has no
// rounding subtlety.
static const float kMaxCellCoord = 2.0e9f;

[shader("compute")]
// LOCAL SIZE COMES FROM THE BUILD, NOT FROM THIS LINE (S6 Task 9b).
// cmake/SpadeSlang.cmake's spade_slang_kernel_variants() compiles this kernel
// once per size in SPADE_SLANG_WORKGROUP_SIZES (= compute/backend.hpp's
// kSupportedWorkgroupSizes), passing -DSPADE_WG=<size>, so that
// BackendDesc::workgroup_size selects a module that was actually compiled for
// it. There is DELIBERATELY no #ifndef fallback: a missing -D must fail the
// build loudly, not silently produce a 64-wide kernel that step_recorder.cpp
// then dispatches on a grid divided by some other number.
[numthreads(SPADE_WG, 1, 1)]
void grid_build(uint3 tid : SV_DispatchThreadID) {
    const uint e = tid.x;
    if (e >= pass_params.grid_entry_count) {
        return;
    }

    // The sentinel, written unless every test below passes. `slot = e` keeps
    // the sort order TOTAL over the padded domain (entry indices are unique),
    // which is what makes the sorted permutation unique and therefore
    // independent of the network, the local size and the schedule --
    // physics/grid.hpp's own determinism argument, carried onto the device.
    GridEntryRow row;
    row.world = kInvalidWorld;
    row.cell_x = 0;
    row.cell_y = 0;
    row.cell_z = 0;
    row.slot = e;
    row._r0 = 0u;

    const uint seg = e / pass_params.grid_segment;
    const uint off = e % pass_params.grid_segment;

    // `off < grid_segment_slots` is the padding test; the second bound is the
    // arena's own extent, which the batched shape does not otherwise imply
    // (its grid_segment_slots IS world_count*body_capacity, so the two agree,
    // but spelling both keeps this correct if either ever stops implying the
    // other).
    if (off < pass_params.grid_segment_slots) {
        const uint slot = seg * pass_params.grid_segment_slots + off;
        if (slot < pass_params.world_count * pass_params.body_capacity) {
            // grid.cpp:372 -- the bit test first. An inert slot costs this and
            // nothing else.
            if ((bodies[slot].flags & kBodyFlagActive) != 0u) {
                // grid.cpp:375-376 -- a freed slot takes no part even if
                // something left its flag set.
                const uint world = bodies_slot_to_world[slot];
                if (world != kInvalidWorld) {
                    // grid.cpp:360 / :378, with the read order noted in the
                    // header: per-world cell_size, then the cell computation.
                    const float cell_size = grid_params[world].cell_size;
                    const float3 pos = bodies[slot].pos;

                    // grid_cell_of(), grid.cpp:306-327, inlined. Componentwise
                    // division then floor, then the range check BEFORE the
                    // cast, spelled as a negated conjunction so a NaN -- from
                    // a NaN position, or from the 0/0 a zero cell_size
                    // produces at the origin -- FAILS it and the body is
                    // skipped rather than converted to an unspecified integer.
                    const float fx = floor(pos.x / cell_size);
                    const float fy = floor(pos.y / cell_size);
                    const float fz = floor(pos.z / cell_size);

                    const bool in_range = (fx >= -kMaxCellCoord && fx <= kMaxCellCoord) &&
                                          (fy >= -kMaxCellCoord && fy <= kMaxCellCoord) &&
                                          (fz >= -kMaxCellCoord && fz <= kMaxCellCoord);
                    if (in_range) {
                        row.world = world;
                        row.cell_x = (int)fx;
                        row.cell_y = (int)fy;
                        row.cell_z = (int)fz;
                        row.slot = slot;
                    }
                }
            }
        }
    }

    grid_entries[e] = row;
}
