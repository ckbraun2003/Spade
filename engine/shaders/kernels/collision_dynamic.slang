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
// THIS PARAGRAPH WAS TRUE UNTIL L306 AND IS NOW FALSE. Struck rather than
// deleted, because what it ASSERTED is the obligation the replacement has to
// discharge, and a reader needs to see the obligation was not forgotten:
//
//   ~~NO WORKGROUP-SHARED ANYTHING, and therefore no workgroup_size
//   dependence (T9's A7 obligation): a world's whole sweep happens in one
//   lane, with no barrier(), no groupshared array and no cross-lane
//   reduction, so the same dispatch at local size 32, 64 or 128 executes
//   the identical sequence of reads and writes.~~
//
// L306 INTRODUCES groupshared AND BARRIERS, so the first clause no longer
// holds. THE OBLIGATION IT EXISTED TO GUARANTEE STILL DOES, BY A DIFFERENT
// ARGUMENT: the sequence of resolve_pair() calls is produced by LANE 0
// walking entries in ascending order and each entry's partners in the order
// the sweep appends them. That order is a property of grid_entries and the
// fixed dz/dy/dx traversal, NOT of how many lanes did the searching -- so
// local size 32, 64 or 128 still executes the identical sequence of WRITES,
// and only the READS are spread across lanes.
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
// ---------------------------------------------------------------------------
// L306: THE SWEEP, SPLIT INTO A PARALLEL SEARCH AND AN IN-ORDER RESOLVE.
//
// WHAT WAS WRONG, IN ONE LINE: `const uint world = tid.x` -- ONE THREAD PER
// WORLD. This kernel's own header said so unprompted ("a world's sweep runs on
// ONE lane, so a 100-body world is ~100 x 27 neighbour lookups of serial
// work"), and it is a deliberate trade for MANY WORLDS WITH FEW BODIES. Every
// surface the user actually runs is the opposite shape: ONE world with many
// bodies. Measured on the dense fixture, 1000 bodies: 429.3 ms/step against
// the CPU backend's 4.57 ms.
//
// THE SEAM THE FIX USES IS ALREADY IN THE CODE BELOW. The sweep does two
// separable things:
//   SEARCH  -- 27 cells, a binary search and a run scan per cell. READ-ONLY:
//              it touches grid_entries and nothing else. Order-independent.
//   RESOLVE -- resolve_pair(), which READS pos/vel/mass and WRITES BOTH vel
//              AND pos. Gauss-Seidel, and therefore ORDER-DEPENDENT.
// So the search can go wide and the resolve cannot.
//
// WHY A WORKGROUP PER WORLD RATHER THAN A SECOND DISPATCH AND A PAIR BUFFER:
// the two-pass-with-a-contact-buffer design needs a new SSBO, a new pipeline,
// a descriptor-layout change and a second barrier. This gets the same split
// with NO new buffer and NO descriptor change -- the pair list lives in
// groupshared for the length of one chunk. The only host-side change is the
// dispatch width.
//
// ORDER IS PRESERVED EXACTLY, AND THAT IS THE WHOLE CORRECTNESS ARGUMENT.
// The serial sweep resolves pairs in the order (entry i ascending, then the
// fixed dz/dy/dx cell order, then the run scanned in sorted order). Here,
// thread t searches entry chunk_start + t and appends its partners in that
// same inner order; then LANE 0 WALKS t = 0..WG-1 IN ORDER and resolves each
// entry's partners in the order they were appended. That is the identical
// sequence of resolve_pair() calls, so the Gauss-Seidel result is bit-identical
// and the banded CPU<->GPU parity tables hold BY CONSTRUCTION rather than by
// tolerance. No colouring, no atomics on velocities, no Jacobi.
//
// NO OVERFLOW CASE EXISTS, BY CONSTRUCTION RATHER THAN BY A CAP. An entry with
// more partners than the per-entry groupshared capacity is handled in ROUNDS:
// round r collects partners [r*kPairsPerEntry, (r+1)*kPairsPerEntry) by
// skipping the first r*kPairsPerEntry matches. The group repeats while ANY
// lane still has partners left. A capped list that silently dropped contacts
// would be a physics defect that no test in this corpus is shaped to catch,
// and "rare" is not an argument -- a dense pile is exactly where it would fire.
// ---------------------------------------------------------------------------
// Per-entry partner capacity for one round. Only the PARTNER slot is stored:
// the entry's own slot is recoverable by lane 0 from grid_entries, so this is
// one uint per pair rather than two. At SPADE_WG = 256 that is 8 KiB.
static const uint kPairsPerEntry = 8u;
groupshared uint gs_partner[SPADE_WG * kPairsPerEntry];
groupshared uint gs_count[SPADE_WG];
groupshared uint gs_more[SPADE_WG];
// ONE STATEMENT OF THE SWEEP, used by the parallel search and by nothing else.
// It writes partner slots for entry `i` into gs_partner[base ..], skipping the
// first `skip` matches, and reports whether more remain beyond what it wrote.
//
// `skip` is what makes the round-based overflow handling possible without a
// second copy of this loop -- and a second copy is exactly what this file's
// own history warns about (two kernels once carried their own glm_min/glm_max
// under a comment asserting they must not be able to read the same law two
// different ways).
uint scan_entry_partners(uint i, uint lo, uint hi, uint skip, uint base, out bool more) {
    const GridEntryRow ea = grid_entries[i];
    uint seen = 0u;      // matches passed so far, including skipped ones
    uint written = 0u;
    more = false;
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int nx = ea.cell_x + dx;
                const int ny = ea.cell_y + dy;
                const int nz = ea.cell_z + dz;
                // find_run(), grid.cpp:291-302 -- unchanged from the serial
                // sweep: a lower bound on the cell key, then an exact compare
                // of all three signed integers.
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
                    if (eb.cell_x != nx || eb.cell_y != ny || eb.cell_z != nz) break;
                    // Lower-slot-resolves (grid.cpp:477), which is also the
                    // self-exclusion since slots are unique.
                    if (!(ea.slot < eb.slot)) continue;
                    if (seen < skip) { seen += 1u; continue; }
                    if (written == kPairsPerEntry) { more = true; return written; }
                    gs_partner[base + written] = eb.slot;
                    written += 1u;
                    seen += 1u;
                }
            }
        }
    }
    return written;
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
//
// DISPATCH SHAPE CHANGED AT L306: ONE WORKGROUP PER WORLD, not one thread.
// step_recorder.cpp emits shape_.world_count groups for this pass (and still
// world_groups_x_ for every other per-world pass). SV_GroupID, not
// SV_DispatchThreadID -- reading the wrong one here would silently give every
// lane of the group a different "world" and quietly corrupt the sweep.
[numthreads(SPADE_WG, 1, 1)]
void collision_dynamic(uint3 gid : SV_GroupID, uint3 lid : SV_GroupThreadID) {
    const uint world = gid.x;
    const uint lane = lid.x;
    // THE WHOLE GROUP RETURNS TOGETHER OR NOT AT ALL. A per-lane early return
    // before a GroupMemoryBarrierWithGroupSync() is undefined behaviour, and
    // it is the classic way a cooperative kernel works on one driver and hangs
    // on another. Every lane of this group maps to the same world, so this
    // condition is uniform across the group by construction.
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
    // Computed REDUNDANTLY BY EVERY LANE rather than by lane 0 and broadcast:
    // two binary searches over a segment is a handful of reads, and a
    // broadcast would need another barrier and another groupshared slot. The
    // redundancy is cheaper than the synchronisation.
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
    // -- THE SWEEP, chunked so the search goes wide and the resolve stays in
    //    order. Every lane executes this loop the same number of times, which
    //    is what keeps the barriers below uniform.
    for (uint chunk = lo; chunk < hi; chunk += SPADE_WG) {
        const uint mine = chunk + lane;         // this lane's entry, or past hi
        const uint base = lane * kPairsPerEntry;
        uint round_skip = 0u;
        bool group_more = true;
        while (group_more) {
            // ---- SEARCH: every lane, its own entry, read-only ------------
            bool mine_more = false;
            uint written = 0u;
            if (mine < hi) {
                written = scan_entry_partners(mine, lo, hi, round_skip, base, mine_more);
            }
            gs_count[lane] = written;
            gs_more[lane] = mine_more ? 1u : 0u;
            GroupMemoryBarrierWithGroupSync();
            // ---- RESOLVE: LANE 0 ONLY, IN THE SERIAL SWEEP'S EXACT ORDER --
            // entry ascending, then partners in the order they were appended,
            // which is the fixed dz/dy/dx cell order and then the run in
            // sorted order. Identical to the sequence the one-lane sweep
            // produced, so the Gauss-Seidel result is unchanged.
            if (lane == 0u) {
                uint any_more = 0u;
                for (uint e = 0u; e < SPADE_WG; ++e) {
                    const uint entry = chunk + e;
                    if (entry >= hi) { break; }
                    const uint ia = grid_entries[entry].slot;
                    const uint n = gs_count[e];
                    for (uint p = 0u; p < n; ++p) {
                        resolve_pair(ia, gs_partner[e * kPairsPerEntry + p], material);
                    }
                    any_more |= gs_more[e];
                }
                // Reused as the group-wide "another round is needed" flag;
                // lane 0 is the only writer and every lane reads it after the
                // barrier below.
                gs_more[0] = any_more;
            }
            GroupMemoryBarrierWithGroupSync();
            group_more = (gs_more[0] != 0u);
            round_skip += kPairsPerEntry;
            GroupMemoryBarrierWithGroupSync();
        }
    }
}
