// ---------------------------------------------------------------------------
// collision_dynamic.slang (S6 Task 7) -- STAGE 3 OF THREE of the
// CollisionDynamic pass: the SWEEP. The port of physics/grid.cpp:391-484 (the
// OFFSETS and RESOLVE stages) plus resolve_pair() (grid.cpp:89-260), v1's
// GridOffsets.comp + GridCollision.comp.
//
// ===========================================================================
// ONE THREAD PER WORLD. THIS IS THE DESIGN DECISION OF THIS FILE AND IT IS
// FORCED BY THE PARITY CONTRACT, NOT CHOSEN FOR CONVENIENCE.
//
// physics/grid.hpp states the constraint outright, in the PARITY paragraph:
//
//     "the response is Gauss-Seidel (each pair sees the velocities the earlier
//      pairs left), so the sorted entry order, the fixed dz/dy/dx neighbour
//      order, and the lower-slot-resolves rule together determine the answer.
//      A GPU mirror that resolves pairs concurrently is a DIFFERENT (Jacobi)
//      solver and will not reproduce these numbers; making S6 bit-identical
//      means reproducing this sweep, which the sorted key array is exactly the
//      right structure for."
//
// So the sweep over one world's entries is SEQUENTIAL here, exactly as it is
// on the CPU: entry i is fully resolved -- all 27 neighbour cells, all
// candidates, in order -- before entry i+1 begins, and each pair reads the
// velocities and positions the earlier pairs left. What IS parallel is the
// only axis along which the CPU pass is itself decomposable: WORLDS. Keying on
// the world id is what makes "cross-world interaction structurally
// impossible" (D8), and grid.cpp's find_run() compares the world id EXACTLY
// (grid.cpp:299), so no pair ever spans two worlds and no two worlds' sweeps
// can observe each other. Running them concurrently is therefore not an
// approximation of the CPU's sequential world loop -- it is the same
// computation with independent parts overlapped.
//
// THE COST IS REAL AND IS REPORTED RATHER THAN HIDDEN: a world's sweep runs on
// ONE lane, so a 100-body world is ~100 * 27 neighbour lookups of serial work
// per substep. The spec's dev-GPU rate target is RECORDED, not gated
// (user-ratified posture), and this task's own brief asks for a port that
// "matches the CPU bit-for-bit in structure" -- the sort and the build, which
// are the O(n log n) and O(n) stages, ARE fully parallel, and they are what D8
// calls the payoff. A parallel Gauss-Seidel sweep that still reproduced this
// exact order would need a dependency-graph schedule over same-body pairs; it
// is not in this task's scope and would be a different (and much larger)
// design.
//
// NO WORKGROUP-SHARED ANYTHING, and therefore no workgroup_size dependence
// (T9's A7 obligation): a world's whole sweep happens in one lane, with no
// barrier(), no groupshared array and no cross-lane reduction, so the same
// dispatch at local size 32, 64 or 128 executes the identical sequence of
// reads and writes.
//
// ===========================================================================
// WHERE A WORLD'S ENTRIES ARE, IN BOTH DISPATCH SHAPES
//
// compute/grid_entry.hpp's grid_domain_of() partitions the key array into
// `grid_entry_count / grid_segment` segments. The BATCHED shape has ONE
// segment holding every world's keys, sorted globally -- and because the key
// LEADS with the world id, world w's keys are a contiguous span of it. The
// PER-WORLD shape has one segment per world. Either way the span this thread
// sweeps is found the same way: bound the segment, then binary-search it for
// the half-open range of keys whose world is exactly w. In the per-world shape
// that range is the segment's live prefix; in the batched shape it is world
// w's slice of the whole array. One code path, no mode flag.
//
// ===========================================================================
// THE OFFSETS STAGE IS FOLDED INTO THE NEIGHBOUR LOOKUP, AND THAT IS EXACT
//
// The CPU builds an explicit RUN array (grid.cpp:403-411: one record per
// distinct (world, cell), in sorted order) and then binary-searches IT
// (find_run, grid.cpp:291-302: std::lower_bound on the key fields, then an
// EXACT world + all-three-cell-integers comparison). This kernel skips the
// intermediate array and binary-searches the SORTED ENTRIES directly for the
// same key.
//
// THE TWO ARE THE SAME SET, IN THE SAME ORDER, and the argument is short
// enough to check: the CPU's run for key K is by construction the maximal
// contiguous span of sorted entries whose key equals K (that is exactly what
// the forward scan at grid.cpp:403-411 computes -- "a run ends exactly where
// the key changes"), and it is scanned in index order (grid.cpp:473). A lower
// bound on the same key over the same sorted array lands on the first element
// of that span, and scanning forward while the key still matches walks the
// same elements in the same order. The exactness that FIX 1 is about -- "the
// key IS the cell -- world id and all three signed cell integers, compared
// exactly, never hashed" -- is preserved verbatim: cell_key_less() and the
// scan's terminating comparison both compare all three signed integers.
//
// Skipping the run array is not an optimization so much as an admission that
// on the device it would be a fourth dispatch producing a redundant index: the
// entries are already the sorted structure the search needs.
//
// ===========================================================================
// PER-WORLD MATERIAL, FROM A BUFFER (checkpoint-1 ruling), exactly as
// collision_static.slang reads it: `contact_params[world]` (binding 19), never
// a per-batch push-constant record -- PassParams has had nowhere to carry one
// since Task 6b removed the field. The CPU's batched form passes world 0's
// record (schedule.cpp:63-64) and is only ever taken when every world's record
// is byte-identical (WorldSetLayout::uniform_dynamic_params), so reading per
// world is correct in BOTH shapes and is the only thing that is correct in the
// heterogeneous one.
//
// ===========================================================================
// DIVISION AND SQRT SITES in this file, each against the CPU line it mirrors:
//   SQRT  std::sqrt(dist2)                          (grid.cpp:128)
//         glm::length(v_t)                          (grid.cpp:218)
//   DIV   (ma * mb) / (ma + mb)                     (grid.cpp:144)
//         m_eff / ma, m_eff / mb                    (grid.cpp:145-146)
//         d / dist                                  (grid.cpp:133)
//         v_t / v_t_len                             (grid.cpp:221)
// Downstream kernels MAY use both (global constraint); this is why CPU<->GPU
// here is banded rather than bit-identical.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
// glm_min/glm_max/glm_dot3/glm_length3 AND effective_proxy_radius -- ONE
// definition, shared with collision_static.slang (S6 Task 8, T7 review minor
// M3). Through Task 7 both kernels carried their own copies under a comment
// asserting they "must not be able to read the same law two different ways";
// that is now true by construction rather than by assertion.
import mirrors;

static const uint kInvalidWorld = 0xFFFFFFFFu;

// grid.cpp:329-333's key order, minus the slot tiebreak: the comparison
// find_run() makes (grid.cpp:271-276's run_key_less), restricted to the cell
// because the world is already constant over the searched range. SIGNED, and
// z/y/x nested, for the reasons grid_sort.slang's header sets out.
bool cell_key_less(GridEntryRow e, int cz, int cy, int cx) {
    if (e.cell_z != cz) return e.cell_z < cz;
    if (e.cell_y != cy) return e.cell_y < cy;
    return e.cell_x < cx;
}

// ---------------------------------------------------------------------------
// resolve_pair -- grid.cpp:89-260, step for step. `ia` and `ib` are distinct
// GLOBAL body slots (the caller's lower-slot rule guarantees it).
//
// THIS SEQUENCE IS THE PARITY CONTRACT (grid.hpp): the same steps, in this
// order, with these groupings, in fp32. Every mass weight, every guard and
// every `!(...)` NaN spelling below is grid.cpp's own; see that file for the
// derivation of the two weights and for why the exchange conserves momentum by
// construction.
//
// The two bodies' `vel` are held in locals across steps 1 and 2 and stored
// once, which is EXACTLY equivalent to the CPU's in-place mutation: nothing
// else reads either body between those two steps (this thread owns the whole
// world, and the pair is resolved to completion before the next one starts).
// `pos` is likewise read once at the top -- step 3 subtracts from the value
// that was current when the function was entered, which is what the CPU's
// `a.pos -= ...` does too, since nothing has written it in between.
// ---------------------------------------------------------------------------
void resolve_pair(uint ia, uint ib, ContactParams pp) {
    // -1. PER-PAIR CONTACT DISTANCE (D-S6-2), grid.cpp:100-103. Summed rather
    //     than doubled: two spheres of DIFFERENT radii touch at the sum of
    //     their radii, and ra == rb == default recovers the old 2*proxy_radius
    //     exactly.
    const float ra = effective_proxy_radius(bodies[ia].proxy_radius, pp.proxy_radius);
    const float rb = effective_proxy_radius(bodies[ib].proxy_radius, pp.proxy_radius);
    const float contact_dist = ra + rb;
    const float contact_dist2 = contact_dist * contact_dist;

    // 0. NARROW PHASE, grid.cpp:123-133. Squared distance first, so a
    //    non-touching candidate costs no sqrt. Both tests are `!(...)` so a
    //    NaN coordinate FAILS to be a contact and returns, instead of falling
    //    through into a division by a NaN distance. The lower bound is |d| > 0
    //    EXACTLY, not an epsilon: coincident bodies have no defined separating
    //    direction.
    const float3 pa = bodies[ia].pos;
    const float3 pb = bodies[ib].pos;
    const float3 d = pb - pa;
    const float dist2 = glm_dot3(d, d);
    if (!(dist2 < contact_dist2)) return;
    if (!(dist2 > 0.0f)) return;

    const float dist = sqrt(dist2);

    // Unit normal, pointing from a to b: `+n` is the direction b must be
    // pushed and a pushed away from (grid.cpp:130-133).
    const float3 n = float3(d.x / dist, d.y / dist, d.z / dist);

    // Penetration depth of the two sphere proxies, positive by the test above
    // (grid.cpp:137).
    const float depth = contact_dist - dist;

    // The two mass weights (grid.cpp:142-146). w_a + w_b == 1 and
    // ma*w_a == mb*w_b == m_eff, which is what makes every step below equal
    // and opposite by construction rather than by accident.
    const float ma = bodies[ia].mass;
    const float mb = bodies[ib].mass;
    const float m_eff = (ma * mb) / (ma + mb);
    const float w_a = m_eff / ma;
    const float w_b = m_eff / mb;

    float3 va = bodies[ia].vel;
    float3 vb = bodies[ib].vel;

    // 1. NORMAL IMPULSE, only for an APPROACHING pair (grid.cpp:168-175).
    //    v_rel_n >= 0 means they are already separating, and impulsing them
    //    again would be the classic double-hit energy pump. `j_n` is
    //    MASS-NORMALIZED (metres per second, not newton-seconds) and the two
    //    dimensionless weights split it inversely by mass.
    const float3 v_rel = vb - va;
    const float v_rel_n = glm_dot3(v_rel, n);
    float j_n = 0.0f;
    if (v_rel_n < 0.0f) {
        j_n = -(1.0f + pp.restitution_e) * v_rel_n;
        va = va - (j_n * w_a) * n;
        vb = vb + (j_n * w_b) * n;
    }

    // 2. COULOMB FRICTION on the RELATIVE tangential velocity that remains
    //    AFTER step 1 (grid.cpp:215-224). The cap mu*j_n is 0 when step 1 did
    //    not fire, so a separating pair has no friction -- out of the algebra,
    //    not out of a branch. The `min` against |v_t| is the STICTION FLOOR:
    //    without it a large mu would subtract more than the whole relative
    //    tangential velocity and REVERSE it.
    const float3 v_rel_post = vb - va;
    const float v_n_post = glm_dot3(v_rel_post, n);
    const float3 v_t = v_rel_post - v_n_post * n;
    const float v_t_len = glm_length3(v_t);
    if (v_t_len > 0.0f) {
        const float dv_t = glm_min(pp.friction_mu * j_n, v_t_len);
        const float3 t = float3(v_t.x / v_t_len, v_t.y / v_t_len, v_t.z / v_t_len);
        va = va + (dv_t * w_a) * t;
        vb = vb - (dv_t * w_b) * t;
    }
    bodies[ia].vel = va;
    bodies[ib].vel = vb;

    // 3. POSITIONAL CORRECTION (Baumgarte-style, grid.cpp:257-259), SPLIT
    //    INVERSELY BY MASS so the pair's centre of mass is preserved. No
    //    velocity contribution -- that is what keeps it from adding energy.
    //    Spelled with max on the SUBTRACTION rather than a branch on
    //    depth > slop, so this is branch-free. Applied whether or not step 1
    //    fired.
    const float correction = pp.baumgarte_beta * glm_max(depth - pp.slop, 0.0f);
    bodies[ia].pos = pa - (correction * w_a) * n;
    bodies[ib].pos = pb + (correction * w_b) * n;
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
void collision_dynamic(uint3 tid : SV_DispatchThreadID) {
    const uint world = tid.x;
    if (world >= pass_params.world_count) {
        return;
    }

    // -- this world's segment of the key array (see the header) -------------
    const uint segment = pass_params.grid_segment;
    const uint segments = pass_params.grid_entry_count / segment;
    // One segment means the BATCHED shape: every world's keys share it, and
    // the binary search below is what separates them. Otherwise segment w is
    // world w's own.
    const uint seg_index = (segments == 1u) ? 0u : world;
    const uint seg_begin = seg_index * segment;
    const uint seg_end = seg_begin + segment;

    // -- this world's half-open range of live keys within that segment ------
    //
    // Two lower bounds over a range sorted by the leading `world` field:
    // [first index whose world is >= w, first index whose world is > w). The
    // second is spelled as a strict `>` rather than as a bound on w+1 so no
    // arithmetic can run into the 0xFFFFFFFF sentinel value.
    uint lo = seg_begin;
    {
        uint l = seg_begin;
        uint r = seg_end;
        while (l < r) {
            const uint m = l + (r - l) / 2u;
            if (grid_entries[m].world < world) { l = m + 1u; } else { r = m; }
        }
        lo = l;
    }
    uint hi = lo;
    {
        uint l = lo;
        uint r = seg_end;
        while (l < r) {
            const uint m = l + (r - l) / 2u;
            if (grid_entries[m].world <= world) { l = m + 1u; } else { r = m; }
        }
        hi = l;
    }

    // grid.cpp:454-459 -- the material, hoisted once per world exactly as the
    // CPU hoists it outside the sweep. PER-WORLD (binding 19), never a
    // per-batch record; see the header.
    const ContactParams material = contact_params[world];

    // -- THE SWEEP, grid.cpp:461-484 ----------------------------------------
    //
    // Entries in sorted order; for each, the 27 cells of the 3x3x3
    // neighbourhood in the fixed dz/dy/dx order, each cell's run scanned in
    // sorted order, each candidate resolved from the LOWER-SLOT side only.
    // Every pair is therefore resolved exactly once (grid.cpp's three-clause
    // argument at :426-439), and the response is applied IN PLACE, which is
    // what makes this Gauss-Seidel.
    //
    // The neighbour arithmetic needs no overflow guard: cell coordinates are
    // bounded by kMaxCellCoord (2e9), which leaves room for the +-1.
    for (uint i = lo; i < hi; ++i) {
        const GridEntryRow ea = grid_entries[i];

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = ea.cell_x + dx;
                    const int ny = ea.cell_y + dy;
                    const int nz = ea.cell_z + dz;

                    // find_run(), grid.cpp:291-302: lower bound on the cell
                    // key, then an EXACT compare of all three signed integers.
                    // The run is the maximal contiguous span that matches --
                    // exactly the record the CPU's offsets stage would have
                    // built for this cell.
                    uint k = lo;
                    {
                        uint l = lo;
                        uint r = hi;
                        while (l < r) {
                            const uint m = l + (r - l) / 2u;
                            if (cell_key_less(grid_entries[m], nz, ny, nx)) { l = m + 1u; } else { r = m; }
                        }
                        k = l;
                    }

                    for (; k < hi; ++k) {
                        const GridEntryRow eb = grid_entries[k];
                        // The run ends exactly where the key changes.
                        if (eb.cell_x != nx || eb.cell_y != ny || eb.cell_z != nz) break;

                        // Lower-slot-resolves (grid.cpp:477). Also the
                        // self-exclusion, since slots are unique.
                        if (!(ea.slot < eb.slot)) continue;

                        resolve_pair(ea.slot, eb.slot, material);
                    }
                }
            }
        }
    }
}
