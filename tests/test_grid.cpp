#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/rng.hpp"
#include "physics/contacts.hpp"
#include "physics/grid.hpp"
#include "physics/integrator.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"

// ---------------------------------------------------------------------------
// CollisionDynamic pass tests (engine design spec D3, D8, section 5
// "Collision").
//
// The references here are derived from the PHYSICS or from a hand-evaluated
// closed form, not from the implementation: the reduced-mass impulse, the e^2
// kinetic-energy ratio of a head-on equal-mass bounce, the mass-inverse
// positional split, conservation of momentum and of the centre of mass.
//
// TWO TESTS ARE NAMED FOR THE V1 BUG this pass exists to fix -- the hash-bucket
// collision in v1's GridCollision.comp. They reconstruct v1's hash from the
// frozen shader source (see V1GridHash below) and use it to pick genuinely
// adversarial cells, rather than asserting against cells picked to look scary.
//
// Nothing here reads a clock, and the only randomness is a fixed-seed
// spade::rng stream (global constraints: no std::random_device, no rand()).
// ---------------------------------------------------------------------------

namespace {

using spade::BodyState;
using spade::WorldParams;
using spade::physics::ContactParams;
using spade::physics::GridCell;
using spade::physics::GridParams;
using spade::physics::GridScratch;
using spade::physics::grid_cell_of;
using spade::physics::grid_entry_less;
using spade::physics::integrate_bodies;
using spade::physics::resolve_dynamic_contacts;
using spade::physics::resolve_dynamic_contacts_jacobi;

namespace body_flags = spade::physics::body_flags;

// 1 kHz substep, as in test_contacts.cpp -- "1 kHz" is the physical statement,
// and every tolerance below is far above one ulp of h.
constexpr float kH = 1e-3f;

// The default material used by the closed-form pair tests. proxy_radius 0.25 m
// makes the contact DIAMETER 0.5 m, and the cell size is set equal to it --
// the cheapest setting that satisfies grid.hpp's cell_size >= 2*proxy_radius
// precondition.
constexpr float kRadius = 0.25f;
constexpr float kContactDist = 2.0f * kRadius;
constexpr float kBeta = 0.2f;
constexpr float kSlop = 1e-3f;

// ---------------------------------------------------------------------------
// v1's spatial hash, reconstructed from the FROZEN v1 shader source so the
// adversarial tests below pick cells that genuinely aliased rather than cells
// that merely look distant:
//
//   assets/shaders/[SYSTEM]GridBuild.comp:29-36      (build side)
//   assets/shaders/[SYSTEM]GridCollision.comp:71-78  (neighbour-scan side)
//
//       uint GetHash(ivec3 cell) {
//           const uint p1 = 73856093u; p2 = 19349663u; p3 = 83492791u;
//           uint n = (uint(cell.x) * p1) ^ (uint(cell.y) * p2) ^ (uint(cell.z) * p3);
//           return n % hashTableSize;
//       }
//
// and the table size v1 spelled FOUR separate times as `1 << 21`
// (src/Core/Engine.cpp lines 197, 279, 340 and 399 -- the "4x-duplicated
// constant" the design spec calls out).
//
// GLSL's uint(int) is a two's-complement reinterpretation, which is exactly
// what C++'s static_cast<uint32_t> of a negative int32_t is defined to do, and
// the multiplications wrap in both languages. So this is a faithful mirror, not
// an approximation.
//
// NOTE WHAT IS *NOT* MIRRORED: v1 derived its ivec3 by offsetting the position
// by `globalBounds` and CLAMPING into [0, gridDim) before hashing. That cell
// derivation is one of the things the port replaces (grid.hpp, FIX 3); the
// aliasing property under test is a property of the HASH, over whatever cell it
// is handed.
// ---------------------------------------------------------------------------
constexpr uint32_t kV1HashTableSize = 1u << 21;

uint32_t V1GridHash(const GridCell& c) {
    constexpr uint32_t p1 = 73856093u;
    constexpr uint32_t p2 = 19349663u;
    constexpr uint32_t p3 = 83492791u;
    const uint32_t n = (static_cast<uint32_t>(c.x) * p1) ^ (static_cast<uint32_t>(c.y) * p2) ^
                       (static_cast<uint32_t>(c.z) * p3);
    return n % kV1HashTableSize;
}

// ---------------------------------------------------------------------------
// Bodies and helpers
// ---------------------------------------------------------------------------

// Value-initialized first so the std430 pad bytes are deterministically zero --
// the determinism and "untouched" tests memcmp whole BodyStates.
BodyState MakeBody(glm::vec3 pos, glm::vec3 vel, float mass = 1.0f) {
    BodyState body{};
    body.pos = pos;
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    body.vel = vel;
    body.mass = mass;
    body.inv_inertia_diag = glm::vec3(1.0f);
    body.flags = body_flags::active;  // WITHOUT THIS THE BODY NEVER MOVES
    return body;
}

ContactParams MakeContacts(float e, float mu, float radius = kRadius) {
    ContactParams cp{};
    cp.restitution_e = e;
    cp.friction_mu = mu;
    cp.baumgarte_beta = kBeta;
    cp.slop = kSlop;
    cp.proxy_radius = radius;
    return cp;
}

GridParams MakeGrid(float cell_size) {
    GridParams gp{};
    gp.cell_size = cell_size;
    return gp;
}

// A position anywhere strictly inside `cell` for the given cell size: the cell
// origin plus a fraction of an edge. Used by the v1-hash tests, which have to
// place bodies in NAMED cells and must not be one ulp away from the wrong one.
glm::vec3 PointInCell(const GridCell& cell, float cell_size, glm::vec3 frac) {
    return glm::vec3(static_cast<float>(cell.x) + frac.x, static_cast<float>(cell.y) + frac.y,
                     static_cast<float>(cell.z) + frac.z) *
           cell_size;
}

// Accumulate in double so the TEST's own summation contributes no error to a
// claim about the PASS's error. Momentum, kinetic energy, and the per-body
// momentum scale the drift is measured against.
glm::dvec3 TotalMomentum(std::span<const BodyState> bodies) {
    glm::dvec3 p(0.0);
    for (const BodyState& b : bodies) {
        p += glm::dvec3(b.vel) * static_cast<double>(b.mass);
    }
    return p;
}

double KineticEnergy(std::span<const BodyState> bodies) {
    double ke = 0.0;
    for (const BodyState& b : bodies) {
        ke += 0.5 * static_cast<double>(b.mass) * glm::dot(glm::dvec3(b.vel), glm::dvec3(b.vel));
    }
    return ke;
}

double MomentumScale(std::span<const BodyState> bodies) {
    double s = 0.0;
    for (const BodyState& b : bodies) {
        s += static_cast<double>(b.mass) * glm::length(glm::dvec3(b.vel));
    }
    return s;
}

glm::dvec3 CentreOfMassTimesMass(std::span<const BodyState> bodies) {
    glm::dvec3 c(0.0);
    for (const BodyState& b : bodies) {
        c += glm::dvec3(b.pos) * static_cast<double>(b.mass);
    }
    return c;
}

// ---------------------------------------------------------------------------
// The 1k free-space shower (the momentum/energy fixture).
//
// ZERO GRAVITY AND NO STATIC WORLD, so the only thing that can change the
// cloud's total momentum is this pass. A uniform +x DRIFT is added on top of
// the random velocities for a stated reason: without it the random directions
// cancel and |P| is ~sqrt(N) rather than ~N, which would make a relative drift
// bound a statement about the cancellation rather than about the solver. With
// the drift, |P| is O(N*m*v) and "drift as a fraction of total momentum" means
// what it says.
// ---------------------------------------------------------------------------
constexpr uint32_t kShowerCount = 1000;
constexpr float kShowerRadius = 0.03f;
constexpr float kShowerCell = 2.0f * kShowerRadius;
constexpr float kShowerBoxHalf = 0.5f;  // 1 m cube
constexpr float kShowerSpeed = 1.0f;    // random component, per axis
constexpr float kShowerDrift = 1.0f;    // uniform +x drift
constexpr uint64_t kShowerSeed = 0xC0FFEEULL;

struct Cloud {
    std::vector<BodyState> bodies;
    std::vector<uint32_t> slot_to_world;
};

Cloud MakeShower(uint32_t count, uint64_t seed) {
    // One domain-tagged stream, drawn in a fixed order. All randomness in the
    // engine and its tests flows through spade::rng (global constraints).
    spade::rng::Stream s = spade::rng::make_stream(seed, "test.grid.shower", 0);

    Cloud cloud;
    cloud.bodies.reserve(count);
    cloud.slot_to_world.assign(count, 0u);

    for (uint32_t i = 0; i < count; ++i) {
        const float px = kShowerBoxHalf * (2.0f * s.next_float() - 1.0f);
        const float py = kShowerBoxHalf * (2.0f * s.next_float() - 1.0f);
        const float pz = kShowerBoxHalf * (2.0f * s.next_float() - 1.0f);
        const float vx = kShowerDrift + kShowerSpeed * (2.0f * s.next_float() - 1.0f);
        const float vy = kShowerSpeed * (2.0f * s.next_float() - 1.0f);
        const float vz = kShowerSpeed * (2.0f * s.next_float() - 1.0f);
        // Masses spread over a 5x range so the mass weights are exercised: with
        // equal masses w_a == w_b == 0.5 and several of the fp32 cancellations
        // below would be exact for the wrong reason.
        const float mass = 0.5f + 2.0f * s.next_float();

        cloud.bodies.push_back(MakeBody(glm::vec3(px, py, pz), glm::vec3(vx, vy, vz), mass));
    }
    return cloud;
}

// One schedule substep for a free-space cloud: CollisionDynamic, then
// Integrate. This IS the ordering contract (grid.hpp / integrator.hpp);
// CollisionStatic would sit immediately before, and is absent here because
// there is no static world.
void Step(Cloud& cloud, const GridParams& gp, const ContactParams& cp, const WorldParams& wp,
          GridScratch& scratch) {
    resolve_dynamic_contacts(cloud.bodies, cloud.slot_to_world, gp, cp, scratch);
    integrate_bodies(cloud.bodies, wp, kH);
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. A single pair against hand-derived closed forms.
// ---------------------------------------------------------------------------

TEST(GridDynamicCollision, HeadOnEqualMassPairMatchesDirectPairMath) {
    // Both bodies in cell (0,0,0) with cell_size == the contact diameter, so
    // this exercises the SAME-CELL path.
    //
    // Hand derivation, all quantities exactly representable in fp32:
    //   d = pb - pa = (0.4, 0, 0), |d| = 0.4, n = +x, depth = 0.5 - 0.4 = 0.1
    //   ma = mb = 1  =>  m_eff = 0.5, w_a = w_b = 0.5
    //   v_rel = vb - va = (-2, 0, 0), v_rel_n = -2  (approaching)
    //   j_n   = -(1 + 0.5) * (-2) = 3
    //   va -= 3*0.5*n = 1.5  =>  va = (1 - 1.5) = -0.5
    //   vb += 3*0.5*n = 1.5  =>  vb = (-1 + 1.5) = +0.5
    //   friction: the post-impulse relative velocity is purely normal, so
    //             |v_t| == 0 and step 2 is a no-op regardless of mu
    //   corr = beta * (depth - slop) = 0.2 * 0.099 = 0.0198, split 50/50
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.4f);
    const GridParams gp = MakeGrid(kContactDist);

    std::vector<BodyState> bodies{
        MakeBody(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
        MakeBody(glm::vec3(0.4f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f))};
    const std::vector<uint32_t> worlds{0u, 0u};
    const double ke0 = KineticEnergy(bodies);

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    EXPECT_NEAR(bodies[0].vel.x, -0.5f, 1e-6f);
    EXPECT_NEAR(bodies[1].vel.x, +0.5f, 1e-6f);
    EXPECT_NEAR(bodies[0].vel.y, 0.0f, 1e-6f);
    EXPECT_NEAR(bodies[1].vel.y, 0.0f, 1e-6f);

    const float corr = kBeta * ((kContactDist - 0.4f) - kSlop);
    EXPECT_NEAR(bodies[0].pos.x, -0.5f * corr, 1e-7f);
    EXPECT_NEAR(bodies[1].pos.x, 0.4f + 0.5f * corr, 1e-7f);

    // The physics-level statements the numbers above are supposed to satisfy,
    // asserted separately so an edit to the numbers cannot quietly break them:
    // momentum conservation (the two velocity deltas are exact negatives at
    // equal mass; what survives is each body's own accumulation rounding), and
    // the e^2 kinetic-energy ratio a head-on equal-mass bounce has in its own
    // centre-of-momentum frame.
    EXPECT_NEAR(bodies[0].vel.x + bodies[1].vel.x, 0.0f, 1e-6f);
    const double ke = KineticEnergy(bodies);
    EXPECT_NEAR(ke / ke0, static_cast<double>(cp.restitution_e) * cp.restitution_e, 1e-6);
}

TEST(GridDynamicCollision, UnequalMassPairSplitsImpulseAndCorrectionInverselyByMass) {
    // ma = 1, mb = 3  =>  m_eff = 0.75, w_a = 0.75, w_b = 0.25.
    //   v_rel = -2 (a moving at +2 into a stationary b), e = 0.5
    //   j_n = 3;  va -= 3*0.75 = 2.25 => -0.25;  vb += 3*0.25 = 0.75
    // and the positional correction splits the same way: the heavy body moves
    // a THIRD as far as the light one.
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.0f);
    const GridParams gp = MakeGrid(kContactDist);

    std::vector<BodyState> bodies{
        MakeBody(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(2.0f, 0.0f, 0.0f), /*mass=*/1.0f),
        MakeBody(glm::vec3(0.4f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), /*mass=*/3.0f)};
    const std::vector<uint32_t> worlds{0u, 0u};

    const glm::dvec3 p0 = TotalMomentum(bodies);
    const glm::dvec3 c0 = CentreOfMassTimesMass(bodies);

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    EXPECT_NEAR(bodies[0].vel.x, -0.25f, 1e-6f);
    EXPECT_NEAR(bodies[1].vel.x, +0.75f, 1e-6f);

    const float corr = kBeta * ((kContactDist - 0.4f) - kSlop);
    EXPECT_NEAR(bodies[0].pos.x, -0.75f * corr, 1e-7f);
    EXPECT_NEAR(bodies[1].pos.x, 0.4f + 0.25f * corr, 1e-7f);

    // Momentum and the centre of mass both survive, which is the whole point of
    // the weights summing to 1.
    EXPECT_NEAR(TotalMomentum(bodies).x, p0.x, 1e-6);
    EXPECT_NEAR(CentreOfMassTimesMass(bodies).x, c0.x, 1e-6);

    // ...and the pair really did separate by the full correction, independently
    // of how it was split.
    const float separation = bodies[1].pos.x - bodies[0].pos.x;
    EXPECT_NEAR(separation, 0.4f + corr, 1e-6f);
}

// ---------------------------------------------------------------------------
// D-S6-2 -- the per-body proxy override, both directions, on a two-body pair.
//
// World default proxy_radius = 0.1 m (uniform contact_dist = 0.2 m). e = 0,
// mu = 0, equal unit masses, so a firing pair's post-contact velocities are
// the same closed form HeadOnEqualMassPairMatchesDirectPairMath above uses
// with e = 0: w_a = w_b = 0.5, j_n = -(1+0)*v_rel_n, and for a pair
// approaching head-on at 1 m/s each (v_rel_n = -2) that lands both bodies at
// vel.x == 0.0f EXACTLY. A pair that does NOT overlap the contact test is left
// byte-for-byte untouched (grid.hpp's own contract), so both velocities stay
// exactly at their initial +-1.
// ---------------------------------------------------------------------------
TEST(GridDynamicCollision, PerBodyProxyRadiusOverridesWorldDefaultBothDirections) {
    const ContactParams cp = MakeContacts(/*e=*/0.0f, /*mu=*/0.0f, /*radius=*/0.1f);
    const GridParams gp = MakeGrid(/*cell_size=*/2.0f);  // generous -- not the footgun under test
    const std::vector<uint32_t> worlds{0u, 0u};

    // Direction 1: override PRODUCES a contact the world default would not.
    // |d| = 0.5 m; uniform-default contact_dist = 0.2 m (0.5 > 0.2, no
    // contact); body 0's override (0.45) makes contact_dist = 0.45 + 0.1 =
    // 0.55 m > 0.5 m -- contact fires.
    {
        std::vector<BodyState> bodies{
            MakeBody(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            MakeBody(glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f))};
        bodies[0].proxy_radius = 0.45f;  // bodies[1] stays sentinel (0 -> world default 0.1)

        GridScratch scratch;
        resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

        EXPECT_EQ(bodies[0].vel.x, 0.0f) << "override should have produced a contact";
        EXPECT_EQ(bodies[1].vel.x, 0.0f) << "override should have produced a contact";
    }

    // Direction 2 (vice versa): override SUPPRESSES a contact the world
    // default would have produced. |d| = 0.15 m; uniform-default contact_dist
    // = 0.2 m (0.15 < 0.2, contact); body 0's override (0.02, tiny) makes
    // contact_dist = 0.02 + 0.1 = 0.12 m < 0.15 m -- no contact.
    {
        std::vector<BodyState> bodies{
            MakeBody(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            MakeBody(glm::vec3(0.15f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f))};
        bodies[0].proxy_radius = 0.02f;

        GridScratch scratch;
        resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

        EXPECT_EQ(bodies[0].vel.x, 1.0f) << "override should have suppressed the contact";
        EXPECT_EQ(bodies[1].vel.x, -1.0f) << "override should have suppressed the contact";
    }
}

TEST(GridDynamicCollision, FrictionRemovesTheCappedRelativeTangentialVelocity) {
    // n = +x. Approach speed 2 m/s, relative tangential speed 1 m/s along -y.
    //   e = 0  =>  j_n = 2;  mu = 0.3  =>  cap = 0.6 < |v_t| = 1, so the cap
    //   binds and the RELATIVE tangential speed drops from 1 to 0.4.
    //   Equal masses => each body absorbs half of the 0.6: a gains +0.3 y,
    //   b loses 0.3 y (v_t points along -y, so a is pushed along -y... with
    //   v_t = (0,-1,0) and t = v_t/|v_t| = (0,-1,0): a.vel += 0.3*t and
    //   b.vel -= 0.3*t).
    const ContactParams cp = MakeContacts(/*e=*/0.0f, /*mu=*/0.3f);
    const GridParams gp = MakeGrid(kContactDist);

    std::vector<BodyState> bodies{
        MakeBody(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f)),
        MakeBody(glm::vec3(0.4f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f))};
    const std::vector<uint32_t> worlds{0u, 0u};

    const double ke0 = KineticEnergy(bodies);

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    // Normal: e = 0 => the relative normal velocity is exactly cancelled.
    EXPECT_NEAR(bodies[1].vel.x - bodies[0].vel.x, 0.0f, 1e-6f);

    // Tangential: 1 - 0.6 = 0.4 of relative slide left, split evenly.
    EXPECT_NEAR(bodies[0].vel.y, 0.7f, 1e-6f);
    EXPECT_NEAR(bodies[1].vel.y, 0.3f, 1e-6f);
    EXPECT_NEAR(bodies[1].vel.y - bodies[0].vel.y, -0.4f, 1e-6f);

    // Momentum conserved in both components; energy strictly removed.
    EXPECT_NEAR(TotalMomentum(bodies).x, 0.0, 1e-6);
    EXPECT_NEAR(TotalMomentum(bodies).y, 1.0, 1e-6);
    EXPECT_LT(KineticEnergy(bodies), ke0);
}

TEST(GridDynamicCollision, AdjacentCellPairIsFoundJustLikeASameCellPair) {
    // The identical geometry of the head-on test, translated so the two bodies
    // straddle a cell boundary: a in cell (0,0,0), b in cell (1,0,0). The
    // neighbour-gather path must produce the same physics as the same-cell
    // path, which is the minimum claim a broad phase has to make.
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.0f);
    const GridParams gp = MakeGrid(kContactDist);  // 0.5 m cells

    std::vector<BodyState> bodies{
        MakeBody(glm::vec3(0.4f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
        MakeBody(glm::vec3(0.8f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f))};
    const std::vector<uint32_t> worlds{0u, 0u};

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    // The cells really are different (otherwise the test proves nothing).
    ASSERT_EQ(scratch.runs.size(), 2u);

    EXPECT_NEAR(bodies[0].vel.x, -0.5f, 1e-6f);
    EXPECT_NEAR(bodies[1].vel.x, +0.5f, 1e-6f);
}

// ---------------------------------------------------------------------------
// 2. Three bodies in one cell: all three pairs, each exactly once, in the
//    documented sweep order.
//
// THE REFERENCE IS THE PASS ITSELF, DRIVEN ONE PAIR AT A TIME. Re-deriving the
// three Gauss-Seidel steps by hand would be re-implementing resolve_pair() in
// the test; instead the reference activates exactly two bodies at a time, in
// the order the sweep visits the pairs -- (0,1), (0,2), (1,2) -- and applies
// them to the same evolving state. If the sweep resolved a pair twice, skipped
// one, or took them in a different order, the byte comparison fails.
// ---------------------------------------------------------------------------

TEST(GridDynamicCollision, ThreeBodiesInOneCellResolveAllThreePairsExactlyOnce) {
    const ContactParams cp = MakeContacts(/*e=*/0.3f, /*mu=*/0.2f);
    const GridParams gp = MakeGrid(1.0f);  // one cell holds the whole triangle

    // An equilateral triangle of side 0.4 m (< the 0.5 m contact diameter, so
    // all three pairs are in contact), sitting well inside cell (0,0,0), with
    // velocities that make every pair approach.
    const auto make = [] {
        std::vector<BodyState> v{
            MakeBody(glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(0.6f, 0.4f, 0.0f), 1.0f),
            MakeBody(glm::vec3(0.9f, 0.5f, 0.5f), glm::vec3(-0.5f, 0.3f, 0.1f), 2.0f),
            MakeBody(glm::vec3(0.7f, 0.84641016f, 0.5f), glm::vec3(0.1f, -0.7f, -0.2f), 3.0f)};
        return v;
    };
    const std::vector<uint32_t> worlds{0u, 0u, 0u};

    std::vector<BodyState> full = make();
    GridScratch scratch;
    resolve_dynamic_contacts(full, worlds, gp, cp, scratch);

    // All three really are in one cell, and every body moved.
    ASSERT_EQ(scratch.runs.size(), 1u);
    ASSERT_EQ(scratch.entries.size(), 3u);

    std::vector<BodyState> ref = make();
    const std::size_t pairs[3][2] = {{0, 1}, {0, 2}, {1, 2}};
    for (const auto& pair : pairs) {
        for (BodyState& b : ref) b.flags = 0u;
        ref[pair[0]].flags = body_flags::active;
        ref[pair[1]].flags = body_flags::active;
        resolve_dynamic_contacts(ref, worlds, gp, cp, scratch);
    }
    for (BodyState& b : ref) b.flags = body_flags::active;

    EXPECT_EQ(std::memcmp(full.data(), ref.data(), full.size() * sizeof(BodyState)), 0)
        << "the 3-body sweep is not the sequence of its three pairs";

    const std::vector<BodyState> initial = make();
    for (std::size_t i = 0; i < full.size(); ++i) {
        EXPECT_NE(std::memcmp(&full[i], &initial[i], sizeof(BodyState)), 0)
            << "body " << i << " took part in no pair";
    }

    // Equal-and-opposite applications, three times over: total momentum and the
    // centre of mass are unchanged to fp32 rounding.
    EXPECT_NEAR(glm::length(TotalMomentum(full) - TotalMomentum(initial)), 0.0, 1e-6);
    EXPECT_NEAR(glm::length(CentreOfMassTimesMass(full) - CentreOfMassTimesMass(initial)), 0.0,
                1e-6);
}

// ---------------------------------------------------------------------------
// 3. The 1k free-space shower: momentum conservation and no energy gain.
// ---------------------------------------------------------------------------

TEST(GridDynamicCollision, ThousandSphereShowerConservesMomentumAndNeverGainsEnergy) {
    const ContactParams cp = MakeContacts(/*e=*/0.4f, /*mu=*/0.3f, kShowerRadius);
    const GridParams gp = MakeGrid(kShowerCell);
    WorldParams wp{};  // ZERO gravity: this pass is the only thing that can
                       // change the cloud's momentum.

    Cloud cloud = MakeShower(kShowerCount, kShowerSeed);
    const std::vector<BodyState> initial = cloud.bodies;

    const glm::dvec3 p0 = TotalMomentum(cloud.bodies);
    const double ke0 = KineticEnergy(cloud.bodies);
    const double scale0 = MomentumScale(cloud.bodies);
    ASSERT_GT(glm::length(p0), 1.0);  // the drift makes |P| O(N*m*v), see MakeShower

    GridScratch scratch;
    scratch.reserve(kShowerCount);
    constexpr int kSteps = 200;
    for (int step = 0; step < kSteps; ++step) {
        Step(cloud, gp, cp, wp, scratch);
    }

    // The test must not be vacuous: a cloud that never collided would conserve
    // momentum trivially.
    int changed = 0;
    for (std::size_t i = 0; i < cloud.bodies.size(); ++i) {
        if (cloud.bodies[i].vel != initial[i].vel) ++changed;
    }
    // 885 of the 1000 collide at least once over these 200 substeps; the bar is
    // set well below that so a small fp difference cannot make it flaky, and
    // well above zero so a broad phase that found nothing cannot pass.
    EXPECT_GT(changed, 400) << "too few bodies collided for this to be a conservation test";

    const glm::dvec3 p1 = TotalMomentum(cloud.bodies);
    const double drift = glm::length(p1 - p0);

    // THE BOUND, AND WHY IT IS THE ONE THE MATH GIVES RATHER THAN THE BRIEF'S
    // CEILING. Momentum is conserved BY CONSTRUCTION here -- every pair applies
    // +x*w_a to one body and -x*w_b to the other, with ma*w_a == mb*w_b ==
    // m_eff -- so the only residue is fp32 rounding, of order eps per impulse,
    // accumulating as a random walk over the substeps. Measured on this fixture
    // (identically in debug and release): drift 6.93e-6 against a momentum
    // scale of 2033, i.e. 3.4e-9 relative -- five ULPs' worth, not a percent.
    // The asserted bound is 1e-7 of that scale, ~30x above what the arithmetic
    // delivers: tight enough that any real asymmetry in the response (a missing
    // equal-and-opposite, a Jacobi-style average like v1's) fails it
    // immediately, loose enough not to be a rounding tripwire.
    //
    // The brief's 1%-of-|P| ceiling is asserted too, separately, so the
    // requirement as written is visibly met even if the tight bound is ever
    // re-tuned.
    EXPECT_LT(drift, 1e-7 * scale0) << "momentum drift " << drift << " over " << kSteps
                                    << " steps (momentum scale " << scale0 << ")";
    EXPECT_LT(drift, 0.01 * glm::length(p0)) << "brief's 1% ceiling";

    // Energy: restitution below 1 and a dissipative friction cap make every
    // individual pair operation non-increasing in kinetic energy, and the
    // positional correction touches only positions. There is no source, so the
    // total can only fall (the epsilon absorbs fp32 rounding on a sum of 1000
    // terms).
    const double ke1 = KineticEnergy(cloud.bodies);
    EXPECT_LE(ke1, ke0 * (1.0 + 1e-6));
    // ...and it really did fall: the measured ratio is 0.727, so the bar at 0.9
    // states "real dissipation happened" without pinning the exact number,
    // which is a property of the fixture rather than of the solver.
    EXPECT_LT(ke1, 0.9 * ke0) << "a colliding cloud with e < 1 and mu > 0 must lose energy";

    // Nothing went non-finite.
    for (const BodyState& b : cloud.bodies) {
        ASSERT_TRUE(std::isfinite(b.pos.x) && std::isfinite(b.vel.x));
    }
}

// ---------------------------------------------------------------------------
// 4. Cross-world isolation -- the batching-invariance canary (D8).
// ---------------------------------------------------------------------------

TEST(GridDynamicCollision, BodiesInDifferentWorldsAtOverlappingCoordinatesDoNotInteract) {
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.3f);
    const GridParams gp = MakeGrid(kContactDist);

    // Two bodies 1 cm apart -- deep inside the 0.5 m contact diameter, and in
    // the same cell -- but in different worlds. If world id were not part of
    // the key this would be a violent contact.
    std::vector<BodyState> bodies{
        MakeBody(glm::vec3(0.20f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
        MakeBody(glm::vec3(0.21f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f))};
    const std::vector<uint32_t> worlds{0u, 1u};
    const std::vector<BodyState> before = bodies;

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    // Both entries were built (so the bodies were not skipped for some other
    // reason) but landed in two different runs.
    ASSERT_EQ(scratch.entries.size(), 2u);
    EXPECT_EQ(scratch.runs.size(), 2u);
    EXPECT_EQ(scratch.entries[0].cell, scratch.entries[1].cell)
        << "the two worlds must genuinely overlap in space for this to be a test";

    EXPECT_EQ(std::memcmp(bodies.data(), before.data(), bodies.size() * sizeof(BodyState)), 0);
}

TEST(GridDynamicCollision, BatchingNWorldsIntoOneCallIsByteIdenticalToNSeparateCalls) {
    // The stronger form of the same claim: two worlds that each have real
    // internal contacts, at deliberately OVERLAPPING coordinates, resolved in
    // one all-worlds sweep and again one world at a time. Any leakage across
    // the world boundary -- in the broad phase, in the sweep order, or in the
    // scratch -- shows up as a byte difference.
    const ContactParams cp = MakeContacts(/*e=*/0.4f, /*mu=*/0.25f);
    const GridParams gp = MakeGrid(kContactDist);

    constexpr uint32_t kPerWorld = 8;
    const auto make = [] {
        std::vector<BodyState> v;
        // Two identical clusters, once per world: same coordinates, different
        // velocities and masses so the two worlds are not trivially equal.
        for (uint32_t w = 0; w < 2; ++w) {
            for (uint32_t i = 0; i < kPerWorld; ++i) {
                const float f = static_cast<float>(i);
                const glm::vec3 pos(0.3f * f, 0.1f * f, 0.05f * f);
                const glm::vec3 vel(0.5f - 0.2f * f, (w == 0 ? 0.3f : -0.4f) * f, 0.1f);
                v.push_back(MakeBody(pos, vel, 1.0f + 0.5f * f + static_cast<float>(w)));
            }
        }
        return v;
    };

    std::vector<uint32_t> worlds;
    for (uint32_t w = 0; w < 2; ++w) {
        for (uint32_t i = 0; i < kPerWorld; ++i) worlds.push_back(w);
    }

    GridScratch scratch;

    std::vector<BodyState> batched = make();
    resolve_dynamic_contacts(batched, worlds, gp, cp, scratch);
    ASSERT_EQ(scratch.entries.size(), static_cast<std::size_t>(2u * kPerWorld));

    std::vector<BodyState> separate = make();
    for (uint32_t w = 0; w < 2; ++w) {
        const std::span<BodyState> slice(separate.data() + w * kPerWorld, kPerWorld);
        const std::span<const uint32_t> map(worlds.data() + w * kPerWorld, kPerWorld);
        resolve_dynamic_contacts(slice, map, gp, cp, scratch);
    }

    EXPECT_EQ(std::memcmp(batched.data(), separate.data(), batched.size() * sizeof(BodyState)), 0)
        << "batching N worlds changed the answer";

    // Not vacuous: the clusters really did collide.
    const std::vector<BodyState> initial = make();
    EXPECT_NE(std::memcmp(batched.data(), initial.data(), batched.size() * sizeof(BodyState)), 0);
}

// ---------------------------------------------------------------------------
// 5. THE V1 HASH-BUCKET COLLISION BUG (assets/shaders/[SYSTEM]GridCollision.comp
//    line 157: `if (gridPairs[k].cellID != neighborHash) break;` -- a
//    comparison of BUCKET INDICES, not of cells).
//
//    Both tests below pick their cells by RUNNING v1's hash, so the aliasing
//    they exercise is the real thing rather than a plausible-looking stand-in.
// ---------------------------------------------------------------------------

TEST(GridDynamicCollision, V1HashBucketCollisionBug_DistantCellsShareABucketButNeverAKey) {
    // Cells (-3,-1,3) and (-3,1,-3) are 6 cells apart in Chebyshev distance --
    // nowhere near neighbours -- and both hash to v1 bucket 1991341.
    const GridCell cell_a{-3, -1, 3};
    const GridCell cell_b{-3, 1, -3};
    ASSERT_EQ(V1GridHash(cell_a), V1GridHash(cell_b))
        << "the premise of this test is that v1 would have put these in one bucket";

    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.3f);
    const GridParams gp = MakeGrid(kContactDist);

    std::vector<BodyState> bodies{
        MakeBody(PointInCell(cell_a, kContactDist, glm::vec3(0.5f)), glm::vec3(1.0f, 2.0f, 3.0f)),
        MakeBody(PointInCell(cell_b, kContactDist, glm::vec3(0.5f)), glm::vec3(-1.0f, 0.5f, 0.0f))};
    const std::vector<uint32_t> worlds{0u, 0u};
    const std::vector<BodyState> before = bodies;

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    // THE STRUCTURAL ASSERTION, and it is the discriminating one. Behaviourally
    // this configuration is a CANARY rather than a discriminator: the two
    // bodies are metres apart, so even a hash-keyed implementation's narrow
    // phase would reject them on distance. What a hash-keyed implementation
    // could NOT do is keep their keys apart -- and that is what is checked
    // here. (The behavioural half of the bug is the next test.)
    ASSERT_EQ(scratch.entries.size(), 2u);
    EXPECT_NE(scratch.entries[0].cell, scratch.entries[1].cell);
    EXPECT_EQ(scratch.runs.size(), 2u) << "two aliasing cells must still be two runs";
    EXPECT_TRUE((scratch.entries[0].cell == cell_a && scratch.entries[1].cell == cell_b) ||
                (scratch.entries[0].cell == cell_b && scratch.entries[1].cell == cell_a));

    // ...and nothing happened.
    EXPECT_EQ(std::memcmp(bodies.data(), before.data(), bodies.size() * sizeof(BodyState)), 0);
}

TEST(GridDynamicCollision, V1HashBucketCollisionBug_AliasedNeighbourCellResolvesThePairOnce) {
    // THE BEHAVIOURAL FORM OF THE BUG, and the reason exact compare matters at
    // all. Cell (0,0,0) has 26 neighbours; two of them -- (-1,-1,1) and
    // (-1,1,-1) -- hash to the SAME v1 bucket, 1592181. A body in (0,0,0)
    // scanning its 27 neighbours under bucket equality therefore visits that
    // bucket's run TWICE, so a partner sitting in (-1,-1,1) is resolved twice
    // in one substep: double the positional correction (and, at other
    // restitutions and friction settings, double the impulse).
    const GridCell own{0, 0, 0};
    const GridCell partner{-1, -1, 1};
    const GridCell alias{-1, 1, -1};
    ASSERT_EQ(V1GridHash(partner), V1GridHash(alias))
        << "the premise of this test is that these two neighbours shared a v1 bucket";
    ASSERT_NE(partner, alias);
    // Both really are neighbours of `own` (Chebyshev distance 1).
    ASSERT_LE(std::abs(partner.x - own.x), 1);
    ASSERT_LE(std::abs(alias.z - own.z), 1);

    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.3f);
    const GridParams gp = MakeGrid(kContactDist);  // 0.5 m cells

    // a in cell (0,0,0), b in cell (-1,-1,1), overlapping and at rest so the
    // ONLY effect is the positional correction -- which, unlike the impulse, is
    // applied unconditionally and therefore accumulates if the pair is resolved
    // more than once.
    const glm::vec3 pa = PointInCell(own, kContactDist, glm::vec3(0.2f, 0.2f, 0.8f));
    const glm::vec3 pb = PointInCell(partner, kContactDist, glm::vec3(0.8f, 0.8f, 0.2f));

    std::vector<BodyState> bodies{MakeBody(pa, glm::vec3(0.0f)), MakeBody(pb, glm::vec3(0.0f))};
    const std::vector<uint32_t> worlds{0u, 0u};

    const float d0 = glm::length(pb - pa);
    ASSERT_LT(d0, kContactDist) << "the pair must actually be in contact";
    ASSERT_GT(d0, kSlop);

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    // Cells are as intended.
    ASSERT_EQ(scratch.runs.size(), 2u);

    // ONE correction: the separation grows by exactly beta*(depth - slop).
    // A second application would add beta*(1-beta)*(depth-slop) on top, i.e.
    // 1.8x the expected growth -- far outside this tolerance.
    const float corr = kBeta * ((kContactDist - d0) - kSlop);
    const float d1 = glm::length(bodies[1].pos - bodies[0].pos);
    EXPECT_NEAR(d1 - d0, corr, 1e-6f) << "expected exactly one de-penetration, got " << (d1 - d0)
                                      << " against a single correction of " << corr;

    // Equal masses, so the split is symmetric and the midpoint does not move.
    EXPECT_NEAR((bodies[0].pos.x + bodies[1].pos.x), (pa.x + pb.x), 1e-7f);

    // Velocities were zero and the pair is not approaching, so no impulse and
    // no friction fired -- another way a double resolution would have shown up.
    EXPECT_FLOAT_EQ(glm::length(bodies[0].vel), 0.0f);
    EXPECT_FLOAT_EQ(glm::length(bodies[1].vel), 0.0f);
}

// ---------------------------------------------------------------------------
// 6. Determinism (charter P8, D11) and the sort's total order.
// ---------------------------------------------------------------------------

TEST(GridDynamicCollision, TwoIdenticalRunsAreByteIdentical) {
    const ContactParams cp = MakeContacts(/*e=*/0.4f, /*mu=*/0.3f, kShowerRadius);
    const GridParams gp = MakeGrid(kShowerCell);
    WorldParams wp{};

    constexpr int kSteps = 40;

    Cloud a = MakeShower(kShowerCount, kShowerSeed);
    GridScratch scratch_a;  // cold
    for (int step = 0; step < kSteps; ++step) Step(a, gp, cp, wp, scratch_a);

    Cloud b = MakeShower(kShowerCount, kShowerSeed);
    // Deliberately a DIFFERENT scratch history: pre-warmed on an unrelated
    // population, so if any state leaked across calls through the scratch the
    // two runs would diverge.
    GridScratch scratch_b;
    {
        Cloud warmup = MakeShower(64, kShowerSeed ^ 0x5A5AULL);
        resolve_dynamic_contacts(warmup.bodies, warmup.slot_to_world, gp, cp, scratch_b);
    }
    for (int step = 0; step < kSteps; ++step) Step(b, gp, cp, wp, scratch_b);

    EXPECT_EQ(std::memcmp(a.bodies.data(), b.bodies.data(), a.bodies.size() * sizeof(BodyState)), 0);
}

TEST(GridDynamicCollision, EntryOrderIsATotalOrderAndTheSortIsAscending) {
    const ContactParams cp = MakeContacts(/*e=*/0.4f, /*mu=*/0.3f, kShowerRadius);
    const GridParams gp = MakeGrid(kShowerCell);

    Cloud cloud = MakeShower(kShowerCount, kShowerSeed);
    // Two worlds, interleaved, so the world field is actually exercised as the
    // most significant key component.
    for (std::size_t i = 0; i < cloud.slot_to_world.size(); ++i) {
        cloud.slot_to_world[i] = static_cast<uint32_t>(i % 2);
    }

    GridScratch scratch;
    resolve_dynamic_contacts(cloud.bodies, cloud.slot_to_world, gp, cp, scratch);

    ASSERT_EQ(scratch.entries.size(), static_cast<std::size_t>(kShowerCount));
    for (std::size_t i = 1; i < scratch.entries.size(); ++i) {
        const auto& prev = scratch.entries[i - 1];
        const auto& cur = scratch.entries[i];
        // Strictly ascending: with the unique `slot` tiebreak no two entries
        // compare equivalent, so the sorted permutation is unique and no
        // implementation choice inside std::sort can affect it.
        EXPECT_TRUE(grid_entry_less(prev, cur)) << "entries " << (i - 1) << "," << i;
        EXPECT_FALSE(grid_entry_less(cur, prev));
    }

    // Irreflexivity, and the run array's agreement with the entry array.
    EXPECT_FALSE(grid_entry_less(scratch.entries[0], scratch.entries[0]));
    uint32_t covered = 0;
    for (const auto& run : scratch.runs) {
        EXPECT_EQ(run.begin, covered);
        EXPECT_GT(run.count, 0u);
        for (uint32_t k = run.begin; k < run.begin + run.count; ++k) {
            EXPECT_EQ(scratch.entries[k].world, run.world);
            EXPECT_EQ(scratch.entries[k].cell, run.cell);
        }
        covered += run.count;
    }
    EXPECT_EQ(static_cast<std::size_t>(covered), scratch.entries.size());
}

// ---------------------------------------------------------------------------
// 7. The skip contracts, and the documented limits.
// ---------------------------------------------------------------------------

TEST(GridDynamicCollision, InactiveAndFreedBodiesAreLeftByteIdentical) {
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.3f);
    const GridParams gp = MakeGrid(kContactDist);

    std::vector<BodyState> bodies{
        MakeBody(glm::vec3(0.00f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
        MakeBody(glm::vec3(0.10f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f)),  // inactive
        MakeBody(glm::vec3(0.15f, 0.0f, 0.0f), glm::vec3(-2.0f, 0.0f, 0.0f)),  // freed slot
        MakeBody(glm::vec3(0.20f, 0.0f, 0.0f), glm::vec3(-3.0f, 0.0f, 0.0f)),  // other flag bits
    };
    bodies[1].flags = 0u;
    bodies[3].flags = 1u << 7;  // some future predicate, but NOT `active`
    const std::vector<uint32_t> worlds{0u, 0u, spade::kInvalidWorld, 0u};
    const std::vector<BodyState> before = bodies;

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    // Only body 0 was eligible, so it had no partner and nothing moved at all.
    EXPECT_EQ(scratch.entries.size(), 1u);
    EXPECT_EQ(std::memcmp(bodies.data(), before.data(), bodies.size() * sizeof(BodyState)), 0);
}

TEST(GridDynamicCollision, CoincidentBodiesAreSkippedRatherThanDividedByZero) {
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.3f);
    const GridParams gp = MakeGrid(kContactDist);

    // Exactly the same position: no defined separating direction, the pair
    // analogue of contacts.cpp's zero-gradient skip.
    std::vector<BodyState> bodies{MakeBody(glm::vec3(0.25f), glm::vec3(1.0f, 0.0f, 0.0f)),
                                  MakeBody(glm::vec3(0.25f), glm::vec3(-1.0f, 0.0f, 0.0f))};
    const std::vector<uint32_t> worlds{0u, 0u};
    const std::vector<BodyState> before = bodies;

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    EXPECT_EQ(std::memcmp(bodies.data(), before.data(), bodies.size() * sizeof(BodyState)), 0);
    EXPECT_FALSE(std::isnan(bodies[0].pos.x));
    EXPECT_FALSE(std::isnan(bodies[0].vel.x));
}

TEST(GridDynamicCollision, NonRepresentableCellsAreSkippedNotClamped) {
    // grid.hpp FIX 3: v1 CLAMPED out-of-range cells into the boundary cells,
    // which made every far-away body a mutual neighbour. Here they drop out.
    GridCell cell{};
    EXPECT_TRUE(grid_cell_of(glm::vec3(0.0f), 1.0f, cell));
    EXPECT_EQ(cell, (GridCell{0, 0, 0}));
    EXPECT_TRUE(grid_cell_of(glm::vec3(-0.5f, 1.5f, -2.5f), 1.0f, cell));
    EXPECT_EQ(cell, (GridCell{-1, 1, -3}));

    // Beyond kMaxCellCoord, infinite, NaN, and the degenerate cell sizes that
    // produce those -- all rejected, none converted to an unspecified integer.
    EXPECT_FALSE(grid_cell_of(glm::vec3(1e18f, 0.0f, 0.0f), 1.0f, cell));
    EXPECT_FALSE(
        grid_cell_of(glm::vec3(0.0f, std::numeric_limits<float>::infinity(), 0.0f), 1.0f, cell));
    EXPECT_FALSE(grid_cell_of(glm::vec3(0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN()), 1.0f,
                              cell));
    EXPECT_FALSE(grid_cell_of(glm::vec3(1.0f, 2.0f, 3.0f), 0.0f, cell));

    // ...and a body at such a position takes no part in the pass rather than
    // colliding with everything else that was clamped to the same place.
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.3f);
    const GridParams gp = MakeGrid(kContactDist);
    std::vector<BodyState> bodies{
        MakeBody(glm::vec3(1e18f, 0.0f, 0.0f), glm::vec3(0.0f)),
        MakeBody(glm::vec3(1e18f, 0.0f, 0.0f), glm::vec3(0.0f)),
    };
    const std::vector<uint32_t> worlds{0u, 0u};
    const std::vector<BodyState> before = bodies;

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    EXPECT_TRUE(scratch.entries.empty());
    EXPECT_EQ(std::memcmp(bodies.data(), before.data(), bodies.size() * sizeof(BodyState)), 0);
}

TEST(GridDynamicCollision, MismatchedSpanLengthsTruncateRatherThanOverrun) {
    // The bounds-safe tail. Documented precondition is equal lengths; the
    // defined behaviour when they differ is that only the common prefix
    // participates.
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.3f);
    const GridParams gp = MakeGrid(kContactDist);

    std::vector<BodyState> bodies{MakeBody(glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
                                  MakeBody(glm::vec3(0.4f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f)),
                                  MakeBody(glm::vec3(0.8f, 0.0f, 0.0f), glm::vec3(0.0f))};
    const std::vector<uint32_t> worlds{0u, 0u};  // shorter than `bodies`
    const BodyState untouched = bodies[2];

    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    EXPECT_EQ(scratch.entries.size(), 2u);
    EXPECT_EQ(std::memcmp(&bodies[2], &untouched, sizeof(BodyState)), 0);
}

TEST(GridDynamicCollision, CellSizeBelowContactDiameterMissesContacts) {
    // grid.hpp's loudest precondition, pinned so the limit is recorded
    // behaviour rather than folklore. The 27-cell gather only reaches partners
    // within one cell, so a contact diameter larger than the cell size is
    // silently lost for any pair that straddles a gap.
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.3f);  // contact diameter 0.5 m

    std::vector<BodyState> bodies{MakeBody(glm::vec3(0.05f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
                                  MakeBody(glm::vec3(0.45f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f))};
    const std::vector<uint32_t> worlds{0u, 0u};
    const std::vector<BodyState> before = bodies;

    GridScratch scratch;

    // cell_size = 0.5 == the contact diameter: the pair is found (cells 0 and 0
    // here, but adjacent cells would work too).
    std::vector<BodyState> ok = bodies;
    resolve_dynamic_contacts(ok, worlds, MakeGrid(kContactDist), cp, scratch);
    EXPECT_NE(std::memcmp(ok.data(), before.data(), ok.size() * sizeof(BodyState)), 0);

    // cell_size = 0.1, well below the 0.5 contact diameter: the two bodies are
    // 4 cells apart, outside the 27-neighbourhood, so the contact is MISSED.
    std::vector<BodyState> missed = bodies;
    resolve_dynamic_contacts(missed, worlds, MakeGrid(0.1f), cp, scratch);
    EXPECT_EQ(std::memcmp(missed.data(), before.data(), missed.size() * sizeof(BodyState)), 0)
        << "cell_size < 2*proxy_radius is documented to lose contacts; if this now passes, "
           "grid.hpp's precondition needs updating";
}

// ---------------------------------------------------------------------------
// 8. GridParams' layout discipline (coordinator ruling: pass-parameter value
//    structs stay in their pass header but carry layout.hpp's battery).
//
// The static_asserts in grid.hpp are the real enforcement -- a violation is a
// build error, so this file compiling at all is already coverage. What they
// CANNOT express is the runtime half: that the array stride a std430 upload
// would use is sizeof, and that a defaulted record has a fully determined byte
// image. Mirrors Contacts.ContactParamsIsStd430SafeAndByteDetermined.
// ---------------------------------------------------------------------------

TEST(GridDynamicCollision, GridParamsIsStd430SafeAndByteDetermined) {
    static_assert(sizeof(GridParams) == 16, "one 16-byte std430 row");
    static_assert(alignof(GridParams) == 16, "std430 base alignment");

    const GridParams arr[2]{};
    const std::ptrdiff_t stride =
        reinterpret_cast<const char*>(&arr[1]) - reinterpret_cast<const char*>(&arr[0]);
    EXPECT_EQ(stride, static_cast<std::ptrdiff_t>(sizeof(GridParams)));
    EXPECT_EQ(reinterpret_cast<std::uintptr_t>(&arr[0]) % alignof(GridParams), 0u);

    // The whole 16-byte image of a defaulted record, field by field in declared
    // order -- pinning the DEFAULTS and the ORDER together.
    const GridParams gp{};
    const float expected[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    EXPECT_EQ(std::memcmp(&gp, expected, sizeof(GridParams)), 0);

    EXPECT_EQ(gp._r0, 0.0f);
    EXPECT_EQ(gp._r1, 0.0f);
    EXPECT_EQ(gp._r2, 0.0f);
}

// ---------------------------------------------------------------------------
// PAIR SYMMETRY -- the precondition for a Jacobi GATHER, tested before the
// solver that depends on it exists.
//
// THE PLANNED SOLVER, AND WHY THIS IS ITS CHEAPEST FALSIFIER. Parallelising
// the contact resolve means abandoning the sequential sweep, and the design
// chosen is Jacobi with a PER-BODY GATHER: every body re-runs the search over
// its own 27-cell neighbourhood, computes each contact's impulse from the
// start-of-iteration state, and sums. No per-body contact lists, no atomics,
// no sort -- the accumulation order is the search order, which is
// deterministic by construction and independent of how many lanes run.
//
// That design rests entirely on one unproven claim: THE SAME CONTACT,
// COMPUTED FROM BODY A'S SIDE AND FROM BODY B'S SIDE, MUST AGREE. Today's
// sweep computes each pair ONCE, under `ea.slot < eb.slot`, so the claim has
// never been exercised -- the lower slot is always `a` and nothing ever asks
// what the other side would have produced.
//
// The argument for it is that every step is an exact IEEE negation: d = pb -
// pa versus pa - pb negates exactly, dot(d,d) is identical because (-x)^2 ==
// x^2, ra + rb is exactly commutative, and v_rel negates exactly. THAT IS AN
// ARGUMENT. This branch reverted a GPU kernel (b157ebcd) whose correctness
// rested on an ordering argument in exactly the right words, checked against
// its author's intent rather than against emitted behaviour. So the argument
// gets a test before the solver gets a line.
//
// ⭐ THE FIXTURE IS DELIBERATELY ASYMMETRIC AND THAT IS THE WHOLE TEST.
// Unequal masses, an off-axis separation, and a tangential velocity so
// friction actually engages. A head-on equal-mass collision along an axis is
// symmetric BY ACCIDENT: a and b are interchangeable in it, so it would pass
// under an implementation with a genuine side-dependence and prove nothing.
// The companion case below asserts the asymmetry itself, so that if someone
// later simplifies this scene into the tidy symmetric one, that goes red
// instead of this going quiet.
// ---------------------------------------------------------------------------
namespace {

// Two bodies in contact, sharing nothing: different masses, separated
// off-axis, each carrying velocity that is neither parallel nor
// perpendicular to the contact normal.
struct AsymmetricPair {
    BodyState a;
    BodyState b;
};

AsymmetricPair MakeAsymmetricPair() {
    AsymmetricPair p{};
    // |d| = sqrt(0.30^2 + 0.12^2 + 0.05^2) ~= 0.3270, inside kContactDist.
    p.a = MakeBody(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.25f, -0.75f, 0.5f), 0.6f);
    p.b = MakeBody(glm::vec3(0.30f, 0.12f, 0.05f), glm::vec3(-0.5f, 0.875f, -1.5f), 1.4f);
    return p;
}

} // namespace

TEST(GridPairSymmetry, TheSameContactAgreesBitwiseFromEitherBodysSide) {
    const ContactParams cp = MakeContacts(/*e=*/0.35f, /*mu=*/0.55f);
    const GridParams gp = MakeGrid(kContactDist);
    const std::vector<uint32_t> worlds{0u, 0u};

    // Order A: the pair as authored, so slot 0 is `a` in resolve_pair.
    const AsymmetricPair p0 = MakeAsymmetricPair();
    std::vector<BodyState> forward{p0.a, p0.b};
    GridScratch s0;
    resolve_dynamic_contacts(forward, worlds, gp, cp, s0);

    // Order B: the SAME two bodies, array positions swapped, so the body that
    // was `a` is now `b`. Nothing physical changed -- only which side of
    // resolve_pair each body arrives on.
    const AsymmetricPair p1 = MakeAsymmetricPair();
    std::vector<BodyState> reversed{p1.b, p1.a};
    GridScratch s1;
    resolve_dynamic_contacts(reversed, worlds, gp, cp, s1);

    // forward[0] and reversed[1] are the same body. BIT-IDENTICAL, not NEAR:
    // the claim the gather rests on is exactness, and a tolerance here would
    // pass for a side-dependence small enough to hide and large enough to
    // diverge over a run.
    EXPECT_EQ(std::memcmp(&forward[0].vel, &reversed[1].vel, sizeof(glm::vec3)), 0)
        << "body A's velocity depends on which side of the contact it was computed from";
    EXPECT_EQ(std::memcmp(&forward[0].pos, &reversed[1].pos, sizeof(glm::vec3)), 0)
        << "body A's position correction depends on which side it was computed from";
    EXPECT_EQ(std::memcmp(&forward[1].vel, &reversed[0].vel, sizeof(glm::vec3)), 0)
        << "body B's velocity depends on which side of the contact it was computed from";
    EXPECT_EQ(std::memcmp(&forward[1].pos, &reversed[0].pos, sizeof(glm::vec3)), 0)
        << "body B's position correction depends on which side it was computed from";
}

// The fixture's own discrimination check -- the reason the case above means
// anything. If the two bodies came out of the sweep with mirror-image states,
// swapping their slots could not tell a correct implementation from a
// side-dependent one.
TEST(GridPairSymmetry, TheFixtureIsAsymmetricEnoughToDetectASideDependence) {
    const ContactParams cp = MakeContacts(/*e=*/0.35f, /*mu=*/0.55f);
    const GridParams gp = MakeGrid(kContactDist);
    const std::vector<uint32_t> worlds{0u, 0u};

    const AsymmetricPair p = MakeAsymmetricPair();
    std::vector<BodyState> bodies{p.a, p.b};
    GridScratch scratch;
    resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);

    // Unequal masses mean unequal velocity deltas: the impulse is shared by
    // inverse mass, so the lighter body must move more. If these were equal
    // the scene would be interchangeable under a <-> b.
    const glm::vec3 dv_a = bodies[0].vel - p.a.vel;
    const glm::vec3 dv_b = bodies[1].vel - p.b.vel;
    ASSERT_GT(glm::length(dv_a), 0.0f) << "no contact was resolved -- the fixture is not touching";
    ASSERT_GT(std::fabs(glm::length(dv_a) - glm::length(dv_b)), 1e-4f)
        << "THE FIXTURE HAS STOPPED DISCRIMINATING. The two bodies now receive equal-magnitude "
           "velocity changes, which means the scene is interchangeable under a <-> b and the "
           "symmetry case above would pass even against an implementation whose result genuinely "
           "depends on which side computed it. The usual cause is the masses having been made "
           "equal, or the separation having been put back on an axis.";

    // And the contact must engage friction, or the tangential half of
    // resolve_pair -- the part with the most opportunity for side-dependence,
    // since it reads the POST-normal-impulse relative velocity -- is never
    // exercised at all.
    const glm::vec3 d = glm::normalize(p.b.pos - p.a.pos);
    const glm::vec3 v_rel = p.b.vel - p.a.vel;
    const glm::vec3 v_tan = v_rel - d * glm::dot(v_rel, d);
    ASSERT_GT(glm::length(v_tan), 1e-3f)
        << "the approach is purely normal, so the friction branch never runs and the symmetry "
           "case exercises only half of resolve_pair";
}

// ===========================================================================
// THE JACOBI GATHER -- resolve_dynamic_contacts_jacobi().
//
// Nothing in the engine calls it yet. It exists alongside the Gauss-Seidel
// sweep so that the solver change can be built and measured before it is
// switched on, and these four cases are the pre-registered partition: which
// results MUST move, which MUST NOT, and why each answer distinguishes a
// mistake from the intended change.
//
// ⭐ THE PARTITION IS THE DIAGNOSIS. A two-body scene has no second contact
// for Gauss-Seidel to have seen first, so the two solvers must agree BIT FOR
// BIT there; a three-body scene where one body has two simultaneous contacts
// is exactly where they must disagree. A failure in the first group therefore
// means the ARITHMETIC is wrong, and a failure in the second means the
// COUPLING did not actually change -- two different repairs, told apart before
// anyone has to guess between them.
// ===========================================================================

TEST(GridJacobiGather, IsByteIdenticalToTheSweepWhenOnlyOnePairIsInFlight) {
    const ContactParams cp = MakeContacts(/*e=*/0.35f, /*mu=*/0.55f);
    const GridParams gp = MakeGrid(kContactDist);
    const std::vector<uint32_t> worlds{0u, 0u};

    // The asymmetric fixture, for the reason the symmetry cases above use it:
    // unequal masses, off-axis separation, and a tangential approach so the
    // friction branch actually runs. A tidy head-on pair would exercise only
    // half of the per-pair kernel.
    const AsymmetricPair p = MakeAsymmetricPair();
    std::vector<BodyState> swept{p.a, p.b};
    std::vector<BodyState> gathered{p.a, p.b};

    GridScratch scratch;
    resolve_dynamic_contacts(swept, worlds, gp, cp, scratch);
    resolve_dynamic_contacts_jacobi(gathered, worlds, gp, cp, scratch);

    ASSERT_NE(std::memcmp(&swept[0], &p.a, sizeof(BodyState)), 0)
        << "the fixture resolved no contact at all, so this case would pass against any pair of "
           "implementations that both do nothing";

    // BIT-IDENTICAL, not NEAR, and that is the point of the test. The obvious
    // way to write a gather -- have the pair kernel RETURN a delta and add it
    // to the snapshot velocity -- computes vel + ((0 - x) + y) where the sweep
    // computes ((vel - x) + y). Same values, different grouping, and in fp32
    // not the same number. A tolerance would pass that regrouping; a memcmp
    // does not, which is why this assertion is worth more than a NEAR.
    EXPECT_EQ(std::memcmp(swept.data(), gathered.data(), swept.size() * sizeof(BodyState)), 0)
        << "THE GATHER DISAGREES WITH THE SWEEP ON A SINGLE PAIR, where there is no second "
           "contact for the sweep to have seen first and the two must therefore be the same "
           "arithmetic. This is an error in the per-pair kernel or in how its result is "
           "accumulated -- NOT the Jacobi/Gauss-Seidel difference, which cannot show up here.";
}

TEST(GridJacobiGather, DiffersFromTheSweepWhenABodyHasTwoSimultaneousContacts) {
    const ContactParams cp = MakeContacts(/*e=*/0.3f, /*mu=*/0.2f);
    const GridParams gp = MakeGrid(1.0f);  // one cell holds the whole triangle
    const std::vector<uint32_t> worlds{0u, 0u, 0u};

    // The same equilateral triangle the sweep's three-pair case uses: every
    // body is in contact with both others, so every body has two simultaneous
    // contacts and the two solvers cannot agree.
    const auto make = [] {
        std::vector<BodyState> v{
            MakeBody(glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(0.6f, 0.4f, 0.0f), 1.0f),
            MakeBody(glm::vec3(0.9f, 0.5f, 0.5f), glm::vec3(-0.5f, 0.3f, 0.1f), 2.0f),
            MakeBody(glm::vec3(0.7f, 0.84641016f, 0.5f), glm::vec3(0.1f, -0.7f, -0.2f), 3.0f)};
        return v;
    };

    std::vector<BodyState> swept = make();
    std::vector<BodyState> gathered = make();

    GridScratch scratch;
    resolve_dynamic_contacts(swept, worlds, gp, cp, scratch);
    resolve_dynamic_contacts_jacobi(gathered, worlds, gp, cp, scratch);

    const std::vector<BodyState> initial = make();
    ASSERT_NE(std::memcmp(gathered.data(), initial.data(), initial.size() * sizeof(BodyState)), 0)
        << "the gather resolved nothing, so the inequality below would hold for the wrong reason";

    // ⭐⭐⭐ THIS IS THE ANTI-VACUITY CONTROL FOR THE WHOLE SOLVER CHANGE. If
    // it goes GREEN -- if the gather agrees with the sweep here -- then what
    // was built is not a Jacobi solver at all, and every other case in this
    // group would pass just as happily against a second copy of the sweep. A
    // test suite where the change is invisible is the failure this branch has
    // paid for more than once.
    EXPECT_NE(std::memcmp(swept.data(), gathered.data(), swept.size() * sizeof(BodyState)), 0)
        << "THE GATHER AGREES WITH THE GAUSS-SEIDEL SWEEP ON A BODY WITH TWO SIMULTANEOUS "
           "CONTACTS, WHICH IT CANNOT DO IF IT IS ACTUALLY A JACOBI SOLVER. Either the pair "
           "math is reading already-updated state instead of the snapshot, or this function is "
           "not the one under test.";
}

TEST(GridJacobiGather, ConservesMomentumAndTheCentreOfMassOnTheThreeBodyTriangle) {
    const ContactParams cp = MakeContacts(/*e=*/0.3f, /*mu=*/0.2f);
    const GridParams gp = MakeGrid(1.0f);
    const std::vector<uint32_t> worlds{0u, 0u, 0u};

    std::vector<BodyState> bodies{
        MakeBody(glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(0.6f, 0.4f, 0.0f), 1.0f),
        MakeBody(glm::vec3(0.9f, 0.5f, 0.5f), glm::vec3(-0.5f, 0.3f, 0.1f), 2.0f),
        MakeBody(glm::vec3(0.7f, 0.84641016f, 0.5f), glm::vec3(0.1f, -0.7f, -0.2f), 3.0f)};

    const glm::dvec3 p0 = TotalMomentum(bodies);
    const glm::dvec3 c0 = CentreOfMassTimesMass(bodies);

    GridScratch scratch;
    resolve_dynamic_contacts_jacobi(bodies, worlds, gp, cp, scratch);

    // THIS IS WHERE grid.hpp's FIX 4 GETS TESTED RATHER THAN ARGUED. v1 also
    // gathered per body -- and then applied the AVERAGE of a body's contacts,
    // so the two halves of one pair were scaled by different neighbour counts,
    // were not equal and opposite, and a closed cloud's momentum drifted. This
    // gather SUMS, so each pair's two mass-weighted changes still cancel
    // identically and the only residue is the fp32 rounding of two separate
    // accumulations: O(eps) and unbiased, where v1's was systematic and O(1).
    //
    // The bound is the sweep's own (1e-6, the three-pair case above), NOT a
    // looser one chosen to fit: if summing had reintroduced v1's defect it
    // would show up here as a failure at this tolerance, and moving the
    // tolerance to accommodate it would be how the defect survives.
    EXPECT_NEAR(glm::length(TotalMomentum(bodies) - p0), 0.0, 1e-6);
    EXPECT_NEAR(glm::length(CentreOfMassTimesMass(bodies) - c0), 0.0, 1e-6);
}

TEST(GridJacobiGather, BothSolversWritePosAndVelAndNothingElse) {
    const ContactParams cp = MakeContacts(/*e=*/0.35f, /*mu=*/0.55f);
    const GridParams gp = MakeGrid(kContactDist);
    const std::vector<uint32_t> worlds{0u, 0u};

    const AsymmetricPair p = MakeAsymmetricPair();
    const std::vector<BodyState> initial{p.a, p.b};

    // THIS IS NOT A TIDINESS CHECK -- IT IS THE GPU SHADOW BUFFER'S PREMISE.
    // The gather needs a start-of-iteration copy of the state its pair math
    // reads. It snapshots pos and vel ONLY, and reads mass, the proxy radius
    // and the flags straight off the live array, on the ground that this pass
    // cannot change them. If that ground is false the snapshot is stale for
    // whichever field moved, and the Slang mirror inherits the same mistake
    // with a shadow buffer sized 24 bytes per body instead of 128.
    //
    // Asserted for BOTH solvers, so that "the gather did not widen the write
    // set" is measured rather than assumed.
    for (int which = 0; which < 2; ++which) {
        std::vector<BodyState> bodies = initial;
        GridScratch scratch;
        if (which == 0) {
            resolve_dynamic_contacts(bodies, worlds, gp, cp, scratch);
        } else {
            resolve_dynamic_contacts_jacobi(bodies, worlds, gp, cp, scratch);
        }

        ASSERT_NE(std::memcmp(bodies.data(), initial.data(), initial.size() * sizeof(BodyState)), 0)
            << "solver " << which << " resolved no contact, so the masked comparison below would "
                                     "pass against a function that does nothing at all";

        // Put pos and vel back; if the solver touched anything else, what is
        // left still differs from the input.
        std::vector<BodyState> masked = bodies;
        for (std::size_t i = 0; i < masked.size(); ++i) {
            masked[i].pos = initial[i].pos;
            masked[i].vel = initial[i].vel;
        }

        EXPECT_EQ(std::memcmp(masked.data(), initial.data(), initial.size() * sizeof(BodyState)), 0)
            << "solver " << which
            << " WROTE A FIELD OTHER THAN pos OR vel. The Jacobi snapshot shadows pos and vel "
               "only and reads everything else live, so whatever moved is now read stale by the "
               "gather -- and the GPU mirror's per-body shadow buffer is sized on the same claim.";
    }
}
