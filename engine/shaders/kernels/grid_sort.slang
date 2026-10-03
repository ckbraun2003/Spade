// ---------------------------------------------------------------------------
// grid_sort.slang (S6 Task 7) -- STAGE 2 OF THREE of the dynamic_contact.resolve
// pass: ONE compare-exchange stage of a bitonic sorting network over the key
// array grid_build.slang just wrote. The port of physics/grid.cpp:383-389 --
// the CPU's single `std::sort(entries, grid_entry_less)` -- and of v1's
// BitonicSort.comp, which was "iteratively dispatched over a power-of-two-
// padded buffer" exactly as this is.
//
// ===========================================================================
// THE KEY ORDER IS grid_entry_less() AND NOTHING ELSE
//
// physics/grid.cpp:329-335, verbatim:
//
//     bool grid_entry_less(const GridEntry& a, const GridEntry& b) {
//         if (a.world  != b.world ) return a.world  < b.world;    // :330
//         if (a.cell.z != b.cell.z) return a.cell.z < b.cell.z;   // :331
//         if (a.cell.y != b.cell.y) return a.cell.y < b.cell.y;   // :332
//         if (a.cell.x != b.cell.x) return a.cell.x < b.cell.x;   // :333
//         return a.slot < b.slot;   // unique -> the order is TOTAL     :334
//     }
//
// key_less() below is that function, field for field, with the SIGNEDNESS
// preserved: `world` and `slot` are uint32 and compare unsigned (which is what
// puts the 0xFFFFFFFF sentinel after every live key), the three cell
// coordinates are int32 and compare SIGNED (grid.hpp's FIX 3: cells are signed
// and unbounded, and a body at a negative coordinate must sort before the
// origin, not after everything).
//
// The z/y/x nesting is grid.hpp's own and is load-bearing rather than
// arbitrary: "the z/y/x nesting makes cells that are adjacent along x
// contiguous in the sorted array, which mirrors the row-major cell
// linearization a GPU grid uses, so the S6 port sorts the same sequence into
// the same layout".
//
// ===========================================================================
// WHY A TOTAL ORDER MAKES THE NETWORK IRRELEVANT TO THE ANSWER
//
// std::sort is not stable, and grid.hpp says why that is fine: `slot` last
// makes the order TOTAL, "so no two entries ever compare equivalent, the
// sorted permutation is UNIQUE, and std::sort's unspecified choice of
// algorithm (and its unstable partitioning) cannot affect the result. That is
// the whole determinism argument for this pass -- there is nothing left for an
// implementation to choose."
//
// That argument transfers to this file WITHOUT WEAKENING, and it is the reason
// a bitonic network is an acceptable stand-in for std::sort rather than merely
// a similar-looking one: the padded lanes are sentinels whose `slot` is their
// own index (compute/grid_entry.hpp), so the order is total over the PADDED
// domain too, the sorted permutation of the whole array is unique, and this
// network's answer is std::sort's answer for the live prefix -- element for
// element, not merely as a multiset.
//
// It is also what makes this kernel WORKGROUP-SIZE INVARIANT by construction
// (T9's A7 obligation): there is no shared memory, no barrier(), and no
// reduction here at all. Each thread's partner is a function of its GLOBAL
// index and the stage's (k, j) alone, so the same (k, j) chain over the same
// input produces the same output at any local size.
//
// ===========================================================================
// THE NETWORK, AND WHERE THE STAGE LIST COMES FROM
//
// The textbook bitonic sort over a power-of-two domain of size S:
//
//     for (k = 2; k <= S; k <<= 1)
//         for (j = k >> 1; j > 0; j >>= 1)
//             for each index i in parallel:
//                 ixj = i ^ j
//                 if (ixj > i)
//                     ascending = ((i & k) == 0)
//                     compare-exchange (i, ixj) in that direction
//
// The two loops are HOST-SIDE and are unrolled into the recorded command
// buffer once, at StepRecorder::create() -- log2(S)*(log2(S)+1)/2 dispatches
// with a memory barrier between each pair, all inside the ONE command buffer
// that is recorded once per shape and resubmitted verbatim (spec section 9).
// `k` and `j` reach this kernel as PassParams lanes baked into each dispatch's
// push constant, which is precisely what a record-once design can do with a
// value that varies per DISPATCH but not per SUBMIT.
//
// `ixj > i` is what makes a stage race-free without any synchronization: each
// unordered partner pair is handled by exactly one of its two threads, and no
// two pairs share an element within a stage.
//
// SEGMENTED, BECAUSE THE DOMAIN MAY BE. compute/grid_entry.hpp's
// grid_domain_of() partitions the array into `grid_entry_count/grid_segment`
// independently-sorted segments -- one whole-domain segment in the BATCHED
// shape, one per world in the PER-WORLD shape. The network index `i` above is
// therefore the offset WITHIN a segment (`off` below) and every access is
// rebased by the segment's start, so a compare-exchange can never cross a
// segment boundary. In the batched shape there is a single segment and the
// rebasing is the identity.
//
// ===========================================================================
// NO FLOATING-POINT ARITHMETIC IN THIS FILE AT ALL. A sort is integer
// comparison and a swap; the keys' cell coordinates were already floored to
// integers by grid_build.slang. So there is nothing here for NoContraction to
// decorate and nothing a denormal mode could change -- the parity content of
// this kernel is entirely in key_less() reproducing grid_entry_less().
// ---------------------------------------------------------------------------

import layouts;
import bindings;

// physics/grid.cpp:329-335, field for field. See this file's header for why
// the world/slot comparisons are UNSIGNED and the three cell comparisons are
// SIGNED, and why the z/y/x nesting is not interchangeable with x/y/z.
bool key_less(GridEntryRow a, GridEntryRow b) {
    if (a.world != b.world) return a.world < b.world;
    if (a.cell_z != b.cell_z) return a.cell_z < b.cell_z;
    if (a.cell_y != b.cell_y) return a.cell_y < b.cell_y;
    if (a.cell_x != b.cell_x) return a.cell_x < b.cell_x;
    return a.slot < b.slot;
}

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
void grid_sort(uint3 tid : SV_DispatchThreadID) {
    const uint e = tid.x;
    if (e >= pass_params.grid_entry_count) {
        return;
    }

    const uint segment = pass_params.grid_segment;
    const uint base = (e / segment) * segment;
    const uint off = e % segment;

    const uint partner = off ^ pass_params.sort_j;
    // Exactly one thread of each partner pair does the work, which is what
    // makes a stage race-free with no synchronization at all.
    if (partner <= off) {
        return;
    }

    const uint ia = base + off;
    const uint ib = base + partner;

    const GridEntryRow a = grid_entries[ia];
    const GridEntryRow b = grid_entries[ib];

    // ((off & k) == 0) selects the direction this half of the bitonic sequence
    // is being merged in. The whole array ends up ASCENDING under key_less(),
    // which is std::sort's own convention with the same comparator.
    const bool ascending = (off & pass_params.sort_k) == 0u;
    const bool swap = ascending ? key_less(b, a) : key_less(a, b);

    if (swap) {
        grid_entries[ia] = b;
        grid_entries[ib] = a;
    }
}
