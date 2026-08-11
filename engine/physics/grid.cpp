#include "physics/grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <glm/glm.hpp>

#include "physics/integrator.hpp"  // body_flags::active

namespace spade::physics {

namespace {

// ---------------------------------------------------------------------------
// The per-world material constants, hoisted once per call so the per-pair op
// sequence below is exactly what a GPU thread executes after reading its
// world's param row -- the same hoisting contacts.cpp does, and for the same
// parity reason.
//
// `contact_dist` and `contact_dist2` are derived rather than stored in
// ContactParams: the record holds a body's RADIUS, and every pair test wants
// the pair's contact DIAMETER and its square. Deriving them here (once) rather
// than per pair keeps the squaring out of the inner loop and, more to the
// point, keeps a single spelling of "2 * proxy_radius" in the pass.
// ---------------------------------------------------------------------------
struct PairParams {
    float contact_dist = 0.0f;   // 2 * proxy_radius, m
    float contact_dist2 = 0.0f;  // (2 * proxy_radius)^2, m^2
    float e = 0.0f;
    float mu = 0.0f;
    float beta = 0.0f;
    float slop = 0.0f;
};

// ---------------------------------------------------------------------------
// Resolves ONE pair. `a` and `b` are distinct bodies (the caller's lower-slot
// rule guarantees it, so these two references never alias).
//
// THIS SEQUENCE IS THE PARITY CONTRACT (see grid.hpp): the Slang mirror must
// perform these same steps, in this order, with these groupings, in fp32.
//
// EVERYTHING BELOW IS MASS-WEIGHTED, and the weights are the one structural
// difference from contacts.cpp. The Newton impulse for two spheres, with no
// moment-arm terms (limitation 1: the response is linear-only), is
//
//     J = -(1 + e) * v_rel_n / (1/ma + 1/mb) = -(1 + e) * v_rel_n * m_eff
//
// with the reduced mass m_eff = ma*mb/(ma+mb). Writing the velocity change of
// each body as J/m gives
//
//     dv_a = -(1 + e) * v_rel_n * (m_eff/ma) * n
//     dv_b = +(1 + e) * v_rel_n * (m_eff/mb) * n
//
// so the whole pass can be expressed in the mass-normalized speed
// j_n = -(1+e)*v_rel_n (metres per second, NOT newton-seconds -- exactly the
// j_n of contacts.cpp) scaled by the two dimensionless weights
//
//     w_a = m_eff/ma = mb/(ma+mb)      w_b = m_eff/mb = ma/(ma+mb)
//
// which sum to 1 and split every effect inversely by mass: the heavier body
// moves less. contacts.cpp IS this function with w_a = 1 and w_b = 0 -- an
// infinitely massive world takes none of the impulse -- which is why the two
// files' step 1/2/3 comments describe the same three operations.
//
// MOMENTUM IS CONSERVED BY CONSTRUCTION, not by accident: every step applies
// `+x*w_a` to one body and `-x*w_b` to the other, and
// ma*w_a == mb*w_b == m_eff, so the two mass-weighted changes cancel
// identically. In fp32 the EXCHANGE cancels to rounding (ma*(m_eff/ma)
// reproduces m_eff to within a rounding or two), and exactly when ma == mb --
// then the two weights are the same bit pattern, so the two velocity deltas are
// exact negatives of one another. What each body's own accumulator rounds that
// delta to when it lands in `vel` is a separate, unavoidable eps; the 1k shower
// test bounds the accumulation of those over 200 substeps rather than claiming
// they do not exist.
//
// PRECONDITION: mass > 0 on both bodies. Same contract as the integrator's,
// unguarded for the same reason -- mass is stored directly rather than as an
// inverse, so 0 means "degenerate", not "infinite", and the physics inner loop
// does not validate its arguments.
// ---------------------------------------------------------------------------
void resolve_pair(BodyState& a, BodyState& b, const PairParams& pp) noexcept {
    // -----------------------------------------------------------------------
    // 0. NARROW PHASE. Squared distance first, so the overwhelmingly common
    //    "candidate from a neighbouring cell that is not actually touching"
    //    case costs no sqrt. v1's GridCollision.comp did the same; it is the
    //    one thing in that shader's inner loop that needed no fixing.
    //
    //    Both tests are spelled `!(...)` so a NaN coordinate FAILS to be a
    //    contact and returns, instead of comparing false on both sides and
    //    falling through into a division by a NaN distance.
    //
    //    The lower bound is the pair analogue of contacts.cpp's zero-gradient
    //    skip: two coincident bodies have no defined separating direction, and
    //    an invented axis would inject momentum in a direction nothing asked
    //    for. Note this is |d| > 0 exactly, not v1's `distSq > 0.000001`
    //    epsilon -- the arithmetic below is well-conditioned for any non-zero
    //    distance, and a magic epsilon would silently make near-coincident
    //    bodies pass through each other.
    // -----------------------------------------------------------------------
    const glm::vec3 d = b.pos - a.pos;
    const float dist2 = glm::dot(d, d);
    if (!(dist2 < pp.contact_dist2)) return;
    if (!(dist2 > 0.0f)) return;

    const float dist = std::sqrt(dist2);

    // Unit normal, pointing from a to b. Sign convention for the whole
    // function: `+n` is the direction b must be pushed and a must be pushed
    // away from.
    const glm::vec3 n = d / dist;

    // Penetration depth of the two sphere proxies, metres. Positive by the
    // test above.
    const float depth = pp.contact_dist - dist;

    // The two mass weights (see the derivation above). Computed once and
    // reused by all three steps, which is also what makes the equal-and-
    // opposite property visible in the code rather than merely true of it.
    const float ma = a.mass;
    const float mb = b.mass;
    const float m_eff = (ma * mb) / (ma + mb);
    const float w_a = m_eff / ma;
    const float w_b = m_eff / mb;

    // -----------------------------------------------------------------------
    // 1. NORMAL IMPULSE. Only an APPROACHING pair gets one: v_rel_n >= 0 means
    //    the two are already separating (or sliding exactly past each other),
    //    and impulsing them again would be the classic double-hit that turns a
    //    single bounce into an energy pump. The guard is what makes it safe to
    //    keep resolving a pair that stays overlapped for several substeps while
    //    the positional correction finishes pushing them clear.
    //
    //    Post-condition when it fires: dot(vel_b - vel_a, n) == -e * v_rel_n,
    //    i.e. the separating speed is e times the approach speed -- the
    //    definition of the coefficient of restitution, and the same
    //    post-condition contacts.cpp's step 1 establishes against the world.
    //
    //    v1 additionally zeroed the restitution below a hard-coded 0.5 m/s
    //    approach speed ("resting threshold", GridCollision.comp:197). That is
    //    dropped: it is a magic constant with no physical reading, its effect
    //    is a discontinuity in the response at an arbitrary speed, and the
    //    approach guard plus the slop band already deliver the resting
    //    behaviour it was reaching for.
    // -----------------------------------------------------------------------
    const glm::vec3 v_rel = b.vel - a.vel;
    const float v_rel_n = glm::dot(v_rel, n);
    float j_n = 0.0f;
    if (v_rel_n < 0.0f) {
        j_n = -(1.0f + pp.e) * v_rel_n;
        a.vel -= (j_n * w_a) * n;
        b.vel += (j_n * w_b) * n;
    }

    // -----------------------------------------------------------------------
    // 2. COULOMB FRICTION, applied to the RELATIVE tangential velocity that
    //    remains after step 1. (The normal impulse only adds along n, so the
    //    tangential part is analytically unchanged by it; recomputing here
    //    rather than before step 1 is what the pinned order says, and it is the
    //    form a solver with a non-trivial effective-mass matrix would need.)
    //
    //    The cap is mu * j_n -- the velocity-space reading of
    //    |f_t| <= mu * |f_n|, in the same mass-normalized units step 1 used, so
    //    the comparison is dimensionally sound without reintroducing m_eff.
    //    Two consequences, both inherited verbatim from contacts.cpp:
    //
    //      * A SEPARATING PAIR HAS NO FRICTION. j_n is 0 when step 1 did not
    //        fire, so the cap is 0 and this step is a no-op. That falls out of
    //        the algebra rather than needing its own branch, and it is
    //        physically right: no normal impulse, no friction.
    //      * THE `min` AGAINST |v_t| IS THE STICTION FLOOR and it is
    //        load-bearing: without it a large mu would subtract more than the
    //        whole relative tangential velocity and REVERSE it, i.e. friction
    //        would accelerate the pair's relative slide backwards. Capped at
    //        |v_t|, the worst case is an exact stop of the RELATIVE tangential
    //        motion -- exact up to fp32 rounding, so a residue of order
    //        eps*|v_t| can survive with either sign. It cannot grow: the next
    //        substep's cap is at least as large as the residue.
    //
    //    Because w_a + w_b == 1, the relative tangential velocity loses exactly
    //    dv_t and the two bodies split that loss inversely by mass -- so a
    //    light body scuffing past a heavy one does nearly all the slowing down,
    //    which is the correct limit and the one v1's per-body averaging got
    //    wrong.
    //
    //    v1 instead built a separate tangential impulse jTangent with its own
    //    reduced-mass division and compared |jTangent| against j*friction
    //    (GridCollision.comp:206-219). That is the same law written twice; the
    //    velocity-space form here is one multiply and one min, and it is
    //    literally the same expression the static pass uses, so the two passes
    //    cannot drift apart.
    // -----------------------------------------------------------------------
    const glm::vec3 v_rel_post = b.vel - a.vel;
    const float v_n_post = glm::dot(v_rel_post, n);
    const glm::vec3 v_t = v_rel_post - v_n_post * n;
    const float v_t_len = glm::length(v_t);
    if (v_t_len > 0.0f) {
        const float dv_t = glm::min(pp.mu * j_n, v_t_len);
        const glm::vec3 t = v_t / v_t_len;
        a.vel += (dv_t * w_a) * t;
        b.vel -= (dv_t * w_b) * t;
    }

    // -----------------------------------------------------------------------
    // 3. POSITIONAL CORRECTION (Baumgarte-style, applied to `pos` directly),
    //    SPLIT INVERSELY BY MASS. Steps 1 and 2 are velocity-level: they stop
    //    the pair converging but cannot undo the overlap already present when
    //    the contact was discovered. This pushes a beta-fraction of the EXCESS
    //    penetration out along n each substep, so a deep first hit decays
    //    geometrically as (1 - beta)^k.
    //
    //    FOUR PROPERTIES, ALL DELIBERATE:
    //      * NO VELOCITY CONTRIBUTION. Moving `pos` without touching `vel` is
    //        what keeps this from adding energy: the classic velocity-form bias
    //        would show up as a real, restitution-scaled bounce.
    //      * THE CENTRE OF MASS IS PRESERVED. The two displacements are
    //        -corr*w_a and +corr*w_b along n, and ma*w_a == mb*w_b == m_eff, so
    //        ma*da + mb*db cancels identically -- to fp32 rounding in general,
    //        and exactly when ma == mb. A pair drifting sideways because
    //        de-penetration moved their centre of mass is the classic
    //        positional-correction artefact; it cannot happen here.
    //        (Their SEPARATION grows by corr*(w_a + w_b) == corr, which is what
    //        makes the total correction independent of the mass ratio.)
    //      * CLAMPED AT ZERO by the max: penetration inside the slop band is
    //        left exactly alone, so a pair that is overlapping but not being
    //        driven deeper is a bit-exact no-op for this pass.
    //      * APPLIED WHETHER OR NOT STEP 1 FIRED. A rebounding pair that is
    //        still overlapping keeps being pushed apart; that is the point of
    //        separating the position and velocity levels.
    //
    //    max(depth - slop, 0) is spelled with glm::max on the SUBTRACTION (not
    //    a branch on depth > slop) so the GPU mirror is branch-free -- again
    //    identical to contacts.cpp's step 3.
    // -----------------------------------------------------------------------
    const float correction = pp.beta * glm::max(depth - pp.slop, 0.0f);
    a.pos -= (correction * w_a) * n;
    b.pos += (correction * w_b) * n;
}

// ---------------------------------------------------------------------------
// Ordering over RUN records, on the key fields only. Identical in shape to
// grid_entry_less() minus the slot tiebreak -- which is what makes the run
// array sorted by construction (it is built by scanning the sorted entries in
// order) and therefore binary-searchable.
//
// The keys of distinct runs are distinct, so this is a strict TOTAL order over
// the run array and lower_bound() lands on the only candidate.
// ---------------------------------------------------------------------------
bool run_key_less(const GridCellRun& a, const GridCellRun& b) noexcept {
    if (a.world != b.world) return a.world < b.world;
    if (a.cell.z != b.cell.z) return a.cell.z < b.cell.z;
    if (a.cell.y != b.cell.y) return a.cell.y < b.cell.y;
    return a.cell.x < b.cell.x;
}

// ---------------------------------------------------------------------------
// The OFFSET lookup: the run for an exact (world, cell) key, or nullptr.
//
// THIS FUNCTION IS THE FIX. v1 looked its neighbour up by hash bucket
// (`gridHead[GetHash(neighbor)]`) and then trusted a bucket-index comparison to
// delimit the run, so a neighbour cell could reach an unrelated cell's bodies
// and, worse, two distinct neighbours could reach the SAME run and resolve
// everything in it twice. Here the search compares the world id and all three
// signed cell integers exactly, so the run this returns belongs to that cell
// and to no other -- and 27 distinct neighbour cells necessarily yield 27
// distinct (or absent) runs, which is what makes "each candidate is visited at
// most once per body" a structural property rather than a probabilistic one.
// ---------------------------------------------------------------------------
const GridCellRun* find_run(const std::vector<GridCellRun>& runs, uint32_t world,
                            const GridCell& cell) noexcept {
    GridCellRun probe{};
    probe.world = world;
    probe.cell = cell;

    const auto it = std::lower_bound(runs.begin(), runs.end(), probe, run_key_less);
    if (it == runs.end()) return nullptr;
    if (it->world != world) return nullptr;
    if (!(it->cell == cell)) return nullptr;  // EXACT compare -- never a hash
    return &*it;
}

}  // namespace

bool grid_cell_of(const glm::vec3& pos, float cell_size, GridCell& out) noexcept {
    // Component-wise division then floor, matching v1's
    // `ivec3(floor(offsetPos / cellSize))` minus the global-bounds offset and
    // the clamp (grid.hpp, FIX 3). Division rather than a hoisted reciprocal
    // multiply: the two are not fp32-equivalent, and division is what the spec
    // and the v1 shader both spell.
    const glm::vec3 f = glm::floor(pos / cell_size);

    // Range check BEFORE the cast (see kMaxCellCoord). Spelled as a negated
    // conjunction so a NaN -- from a NaN position, or from the 0/0 a zero
    // cell_size produces at the origin -- fails it and the body is skipped
    // rather than converted to an unspecified integer. A zero cell_size sends
    // every other position to +-inf, which fails the same test.
    if (!(f.x >= -kMaxCellCoord && f.x <= kMaxCellCoord)) return false;
    if (!(f.y >= -kMaxCellCoord && f.y <= kMaxCellCoord)) return false;
    if (!(f.z >= -kMaxCellCoord && f.z <= kMaxCellCoord)) return false;

    out.x = static_cast<int32_t>(f.x);
    out.y = static_cast<int32_t>(f.y);
    out.z = static_cast<int32_t>(f.z);
    return true;
}

bool grid_entry_less(const GridEntry& a, const GridEntry& b) noexcept {
    if (a.world != b.world) return a.world < b.world;
    if (a.cell.z != b.cell.z) return a.cell.z < b.cell.z;
    if (a.cell.y != b.cell.y) return a.cell.y < b.cell.y;
    if (a.cell.x != b.cell.x) return a.cell.x < b.cell.x;
    return a.slot < b.slot;  // unique -> the order is TOTAL (see grid.hpp)
}

void resolve_dynamic_contacts(std::span<BodyState> bodies, std::span<const uint32_t> slot_to_world,
                              const GridParams& grid, const ContactParams& params,
                              GridScratch& scratch) noexcept {
    // clear() keeps capacity, so after the warmup substeps that grow these to
    // their high-water mark this pass performs no allocation at all (grid.hpp,
    // GridScratch). It is also what makes the scratch carry no meaning across
    // calls.
    scratch.entries.clear();
    scratch.runs.clear();

    // Bounds-safe tail, on both counts:
    //   * only the prefix BOTH spans cover participates. The documented
    //     precondition is that they are the same length; truncating is the
    //     defined behaviour if they are not.
    //   * GridEntry::slot is a uint32_t, matching the arena's slot indices, so
    //     the span index is clamped to what that can hold rather than wrapped
    //     into a wrong -- but still in-range, hence undetectable -- slot. A
    //     span this long is 512 GB of BodyState and cannot arise from an arena;
    //     the clamp exists so that the cast below is provably lossless rather
    //     than merely unlikely to lose.
    const std::size_t n = std::min({bodies.size(), slot_to_world.size(),
                                    static_cast<std::size_t>(std::numeric_limits<uint32_t>::max())});

    const float cell_size = grid.cell_size;

    // -----------------------------------------------------------------------
    // 1. BUILD -- one key per active, in-range body (v1: GridBuild.comp).
    //
    // The three skip conditions are grid.hpp's three kinds of skipped body, in
    // increasing order of cost: a bit test, an integer compare, and only then
    // the three divisions of the cell computation. A zero-filled arena slot
    // therefore costs one bit test and nothing else.
    // -----------------------------------------------------------------------
    for (std::size_t i = 0; i < n; ++i) {
        const BodyState& body = bodies[i];
        if ((body.flags & body_flags::active) == 0u) continue;

        const uint32_t world = slot_to_world[i];
        if (world == kInvalidWorld) continue;

        GridCell cell{};
        if (!grid_cell_of(body.pos, cell_size, cell)) continue;

        scratch.entries.push_back(GridEntry{world, cell, static_cast<uint32_t>(i)});
    }

    // -----------------------------------------------------------------------
    // 2. SORT (v1: BitonicSort.comp, iteratively dispatched over a
    //    power-of-two-padded buffer). std::sort here, over a total order, so
    //    the output permutation is unique and there is no padding tail to guard
    //    -- see grid_entry_less() and grid.hpp's FIX 3.
    // -----------------------------------------------------------------------
    std::sort(scratch.entries.begin(), scratch.entries.end(), grid_entry_less);

    const uint32_t entry_count = static_cast<uint32_t>(scratch.entries.size());

    // -----------------------------------------------------------------------
    // 3. OFFSETS -- one run per distinct (world, cell) (v1: GridOffsets.comp,
    //    which wrote a head index per hash bucket into a 2^21-entry table that
    //    had to be cleared every frame by a separate dispatch).
    //
    //    A single forward scan suffices precisely because the entries are
    //    sorted: equal keys are adjacent, so a run ends exactly where the key
    //    changes. The comparison is again EXACT -- world plus all three cell
    //    integers -- so a run never merges two cells.
    // -----------------------------------------------------------------------
    for (uint32_t i = 0; i < entry_count; ++i) {
        const GridEntry& e = scratch.entries[i];
        if (!scratch.runs.empty() && scratch.runs.back().world == e.world &&
            scratch.runs.back().cell == e.cell) {
            ++scratch.runs.back().count;
        } else {
            scratch.runs.push_back(GridCellRun{e.world, e.cell, i, 1});
        }
    }

    // -----------------------------------------------------------------------
    // 4. RESOLVE (v1: GridCollision.comp).
    //
    // THE SWEEP. Entries are visited in sorted order; for each, the 27 cells of
    // the 3x3x3 neighbourhood are looked up in a fixed dz/dy/dx order and their
    // runs scanned in sorted order. The response is applied IN PLACE, so this
    // is a Gauss-Seidel sweep: a pair sees the velocities the earlier pairs
    // left. That is a deliberate departure from v1, which accumulated every
    // contact per body and applied the average (a Jacobi step that does not
    // conserve momentum -- grid.hpp, FIX 4). The price is that the answer
    // depends on the sweep order; the sweep order is fully determined by the
    // sort, which is why the sort has to be a total one.
    //
    // EACH PAIR IS RESOLVED EXACTLY ONCE, and the argument is worth writing
    // down because it is the property duplicate-prone broad phases get wrong:
    //
    //   * every unordered pair in contact is CONSIDERED exactly twice, once
    //     from each endpoint -- if b is within one cell of a then a is within
    //     one cell of b, so each finds the other in exactly one of its 27
    //     neighbour cells (cells are disjoint, so exactly one contains b);
    //   * `ea.slot < eb.slot` resolves it from the lower-slot side only, and
    //     since slots are unique it also excludes the self-pair without a
    //     separate `i == k` test (v1 needed one);
    //   * within one endpoint's sweep no candidate is visited twice, because
    //     the 27 neighbour offsets are distinct cells and find_run() maps
    //     distinct cells to distinct runs. THIS is the clause hash-bucket
    //     lookup breaks (grid.hpp, FIX 1).
    //
    // The neighbour arithmetic needs no overflow guard: cell coordinates are
    // bounded by kMaxCellCoord, which leaves room for the +-1.
    //
    // THE BROAD PHASE IS BUILT ONCE, FROM PRE-SWEEP POSITIONS, and is not
    // rebuilt as the positional correction moves bodies during the sweep. The
    // NARROW phase always reads current positions, so nothing is resolved
    // against a stale geometry; what the fixed cells mean is that a body pushed
    // across a cell boundary mid-sweep is not re-examined against its new
    // neighbours until the next substep. At a correction of beta*(depth - slop)
    // -- a fraction of an overlap, itself a fraction of a cell -- that is a
    // sub-substep lag, not a missed contact, and rebuilding mid-sweep would
    // make the pass's cost unbounded and its result order-of-motion dependent.
    // -----------------------------------------------------------------------
    PairParams pp{};
    pp.contact_dist = 2.0f * params.proxy_radius;
    pp.contact_dist2 = pp.contact_dist * pp.contact_dist;
    pp.e = params.restitution_e;
    pp.mu = params.friction_mu;
    pp.beta = params.baumgarte_beta;
    pp.slop = params.slop;

    for (uint32_t i = 0; i < entry_count; ++i) {
        const GridEntry ea = scratch.entries[i];

        for (int32_t dz = -1; dz <= 1; ++dz) {
            for (int32_t dy = -1; dy <= 1; ++dy) {
                for (int32_t dx = -1; dx <= 1; ++dx) {
                    const GridCell neighbour{ea.cell.x + dx, ea.cell.y + dy, ea.cell.z + dz};

                    const GridCellRun* run = find_run(scratch.runs, ea.world, neighbour);
                    if (run == nullptr) continue;

                    const uint32_t end = run->begin + run->count;
                    for (uint32_t k = run->begin; k < end; ++k) {
                        const GridEntry& eb = scratch.entries[k];

                        // Lower-slot-resolves. Also the self-exclusion.
                        if (!(ea.slot < eb.slot)) continue;

                        resolve_pair(bodies[ea.slot], bodies[eb.slot], pp);
                    }
                }
            }
        }
    }
}

}  // namespace spade::physics
