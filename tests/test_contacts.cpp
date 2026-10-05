#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/contacts.hpp"
#include "physics/integrator.hpp"
#include "state/layout.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// CollisionStatic pass tests (engine design spec D3, section 5 "Collision").
//
// The references here are derived from the PHYSICS, not from the
// implementation: mu*g for the sliding deceleration, e^2 for the bounce apex
// ratio, slop + g*h^2/beta for the resting penetration. Where the discrete
// scheme biases a continuous-time reference, the bias is named and bounded in
// the comment rather than hidden inside a loose tolerance.
//
// Every test steps the world the way the schedule does -- CollisionStatic
// BEFORE Integrate -- because the friction law's exactness depends on that
// order (contacts see one substep of gravity in the normal velocity). The
// handful of tests that assert on a SINGLE resolve call say so explicitly.
//
// Nothing here is random and nothing reads a clock.
// ---------------------------------------------------------------------------

namespace {

using spade::BodyState;
using spade::SdfNode;
using spade::SdfProgram;
using spade::SdfTransform;
using spade::WorldParams;
using spade::physics::ContactParams;
using spade::physics::integrate_bodies;
using spade::physics::resolve_static_contacts;

namespace body_flags = spade::physics::body_flags;

// 1 kHz substep, per the brief. Not a power of two (unlike the integrator
// tests' 1/1024) because "1 kHz" is the physical statement being tested; the
// tolerances below are all far above one ulp of h.
constexpr float kH = 1e-3f;
constexpr float kG = 9.81f;
constexpr float kRadius = 0.1f;  // sphere-proxy radius, m
constexpr float kSlop = 1e-3f;
constexpr float kBeta = 0.2f;

// A resting body under gravity does not settle at exactly `slop`: it sinks
// g*h^2 per substep (Integrate applies gravity, then the next CollisionStatic
// cancels the velocity) while the Baumgarte correction lifts it
// beta*(depth - slop). Equilibrium is where those balance:
//
//     beta * (depth - slop) = g * h^2   =>   depth = slop + g*h^2/beta
//
// which is 1 mm + 49 um here. Tests assert against slop + this excess, with
// margin, rather than against a magic number.
constexpr float kRestExcess = kG * kH * kH / kBeta;  // 4.905e-5 m

// ---------------------------------------------------------------------------
// SDF program builders. Programs are hand-built (the aggregate is public and
// sdf.hpp sanctions it) rather than routed through WorldBuilder, so each test
// states its geometry in one line and nothing else can perturb it. validate()
// is called on every one -- sdf.hpp's precondition for eval/gradient/sample is
// "has passed validate()", and a test that skipped it would be relying on UB.
// ---------------------------------------------------------------------------

SdfNode PlaneNode(glm::vec3 n, float offset) {
    SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::plane);
    node.op = static_cast<uint32_t>(spade::SdfOp::none);
    node.transform = 0;
    node.params = glm::vec4(n, offset);
    return node;
}

SdfNode OpNode(spade::SdfOp op, float k = 0.0f) {
    SdfNode node{};
    node.op = static_cast<uint32_t>(op);
    node.params = glm::vec4(k, 0.0f, 0.0f, 0.0f);
    return node;
}

void ExpectValid(const SdfProgram& prog) {
    const spade::Result<uint32_t> depth = prog.validate();
    ASSERT_TRUE(depth.has_value()) << "hand-built program is invalid: " << depth.error().context;
}

// Ground plane: solid is the half space { dot(p, n) <= offset }, so
// n = +z, offset = 0 makes z <= 0 solid and phi(p) == p.z above it.
SdfProgram GroundPlane() {
    SdfProgram prog;
    prog.transforms.push_back(SdfTransform{});  // identity
    prog.nodes.push_back(PlaneNode(glm::vec3(0.0f, 0.0f, 1.0f), 0.0f));
    ExpectValid(prog);
    return prog;
}

// Floor (z <= 0) UNION wall (x <= 0): a right-angle corner whose free region
// is the quarter space { x > 0, z > 0 }. phi == min(p.z, p.x).
SdfProgram FloorAndWallCorner() {
    SdfProgram prog;
    prog.transforms.push_back(SdfTransform{});
    prog.nodes.push_back(PlaneNode(glm::vec3(0.0f, 0.0f, 1.0f), 0.0f));  // a: floor
    prog.nodes.push_back(PlaneNode(glm::vec3(1.0f, 0.0f, 0.0f), 0.0f));  // b: wall
    prog.nodes.push_back(OpNode(spade::SdfOp::union_));
    ExpectValid(prog);
    return prog;
}

// The same corner, SMOOTH-unioned with blend radius k. The point of this one
// is that smooth_union blends the two branches' gradients (gb + (ga-gb)*h),
// so in the blend region the field's gradient is SHORTER THAN UNIT -- the
// case that catches an implementation which assumes |grad| == 1.
SdfProgram SmoothCorner(float k) {
    SdfProgram prog;
    prog.transforms.push_back(SdfTransform{});
    prog.nodes.push_back(PlaneNode(glm::vec3(0.0f, 0.0f, 1.0f), 0.0f));  // a: floor
    prog.nodes.push_back(PlaneNode(glm::vec3(1.0f, 0.0f, 0.0f), 0.0f));  // b: wall
    prog.nodes.push_back(OpNode(spade::SdfOp::smooth_union, k));
    ExpectValid(prog);
    return prog;
}

// A unit-radius solid sphere at the origin: a CURVED obstacle whose normal is
// radial and varies along the contact, unlike every plane above.
SdfProgram UnitSphereObstacle() {
    SdfProgram prog;
    prog.transforms.push_back(SdfTransform{});
    SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    node.op = static_cast<uint32_t>(spade::SdfOp::none);
    node.transform = 0;
    node.params = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
    prog.nodes.push_back(node);
    ExpectValid(prog);
    return prog;
}

// ---------------------------------------------------------------------------
// Bodies and stepping
// ---------------------------------------------------------------------------

// Value-initialized first so the std430 pad bytes are deterministically zero
// -- the determinism and "untouched" tests memcmp whole BodyStates.
BodyState MakeUnitBody(glm::vec3 pos, glm::vec3 vel) {
    BodyState body{};
    body.pos = pos;
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    body.vel = vel;
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    body.flags = body_flags::active;  // WITHOUT THIS THE BODY NEVER MOVES
    return body;
}

WorldParams MakeWorld(glm::vec3 gravity) {
    WorldParams params{};
    params.gravity = gravity;
    return params;
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

// One schedule substep: CollisionStatic, then Integrate. This IS the ordering
// contract -- see the file header.
void Step(std::vector<BodyState>& bodies, const SdfProgram& prog, const ContactParams& cp,
          const WorldParams& wp) {
    resolve_static_contacts(bodies, prog, cp, kH);
    integrate_bodies(bodies, wp, kH);
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. Drop from 10 m onto a plane at 1 kHz: no tunnelling on the way in, rest
//    within tolerance on the way out, and no jitter once resting.
// ---------------------------------------------------------------------------

TEST(Contacts, DropFromTenMetresRestsWithoutTunnelling) {
    const SdfProgram prog = GroundPlane();
    const WorldParams wp = MakeWorld(glm::vec3(0.0f, 0.0f, -kG));
    const ContactParams cp = MakeContacts(/*e=*/0.0f, /*mu=*/0.3f);

    std::vector<BodyState> bodies{MakeUnitBody(glm::vec3(0.0f, 0.0f, 10.0f), glm::vec3(0.0f))};

    // Impact speed from the 9.9 m of free fall between the start height and
    // first contact (phi == radius). The per-substep advance at impact is
    // v_impact*h = 13.9 mm -- an order below the 100 mm proxy radius, which is
    // WHY discrete contact detection cannot miss this surface. That margin is
    // the real content of "no tunnelling at 1 kHz"; the assertions below
    // measure it rather than assuming it.
    const float v_impact = std::sqrt(2.0f * kG * (10.0f - kRadius));
    const float max_expected_penetration = v_impact * kH;

    float min_phi = 10.0f;
    float worst_penetration = 0.0f;
    float max_speed_early = 0.0f;  // steps 2000..2500, after settling
    float max_speed_late = 0.0f;   // steps 3500..4000
    float min_z_late = 10.0f;
    float max_z_late = -10.0f;

    constexpr int kSteps = 4000;  // 4 s: ~1.42 s of fall, then 2.6 s at rest
    for (int i = 0; i < kSteps; ++i) {
        Step(bodies, prog, cp, wp);

        const float phi = spade::eval(prog, bodies[0].pos);
        min_phi = std::min(min_phi, phi);
        worst_penetration = std::max(worst_penetration, kRadius - phi);

        const float speed = glm::length(bodies[0].vel);
        if (i >= 2000 && i < 2500) {
            max_speed_early = std::max(max_speed_early, speed);
        }
        if (i >= 3500) {
            max_speed_late = std::max(max_speed_late, speed);
            min_z_late = std::min(min_z_late, bodies[0].pos.z);
            max_z_late = std::max(max_z_late, bodies[0].pos.z);
        }
    }

    // NO TUNNELLING, in its strongest form: the body ORIGIN never entered the
    // solid at any sampled substep, so the proxy never passed through the
    // surface. Measured min phi on this box: 0.0886 m.
    EXPECT_GT(min_phi, 0.0f) << "body origin crossed the surface -- tunnelled";

    // ...and the proxy's own worst overlap stayed within one substep of travel
    // at the impact speed, with 50% headroom. Measured: 11.4 mm against a
    // 20.9 mm bound. This is the assertion that would fail if contact
    // detection were, say, one substep late.
    EXPECT_LT(worst_penetration, 1.5f * max_expected_penetration);

    // REST. Velocity after Integrate is exactly one substep of gravity
    // (CollisionStatic zeroed it, Integrate re-applied g*h), i.e. 9.81 mm/s --
    // the floor of what any scheme that applies gravity before testing contact
    // can achieve. 2*g*h leaves 2x margin without admitting a real bounce.
    EXPECT_LT(glm::length(bodies[0].vel), 2.0f * kG * kH);

    // Resting penetration sits in the slop band plus the predicted excess.
    const float final_penetration = kRadius - spade::eval(prog, bodies[0].pos);
    EXPECT_GT(final_penetration, 0.0f);
    EXPECT_LT(final_penetration, kSlop + 2.0f * kRestExcess);

    // NO JITTER GROWTH: the late window is no livelier than the early one, and
    // the resting position is stationary to within a micrometre. A solver that
    // fought itself (correction pumping the impulse) would show up here long
    // before it showed up in the tolerances above.
    EXPECT_LE(max_speed_late, max_speed_early + 1e-6f);
    EXPECT_LT(max_z_late - min_z_late, 1e-6f);
}

// ---------------------------------------------------------------------------
// 1b. D-S6-2 -- the per-body proxy override, both directions, in one call.
//
// A ground plane, phi(p) == p.z (GroundPlane()'s own doc). World default
// proxy_radius = 0.1 m. Two bodies, both approaching at -1 m/s along z with
// e = 0 and mu = 0, so a firing contact is a CLOSED FORM: v_n = -1 exactly,
// j_n = -(1+0)*(-1) = 1 exactly, vel.z becomes -1 + 1 = 0 EXACTLY (no
// rounding -- both operands and the result are exactly representable). A
// contact that does NOT fire leaves the body byte-for-byte untouched
// (contacts.hpp's own contract), so vel.z stays exactly -1.
//
//   body A at z = 0.15: world default (0.1) does NOT reach it (0.15 >= 0.1,
//   no contact) but a per-body override of 0.2 DOES (0.15 < 0.2) --
//   DIRECTION 1: the override PRODUCES a contact the default would not.
//
//   body B at z = 0.05: world default (0.1) DOES reach it (0.05 < 0.1,
//   contact) but a per-body override of 0.02 does NOT (0.05 >= 0.02) --
//   DIRECTION 2 (vice versa): the override SUPPRESSES a contact the default
//   would have produced.
//
// A third body, C, has NO override (BodyState::proxy_radius left at its
// MakeUnitBody() zero-fill) at the SAME z = 0.05 as B: it exists to pin the
// sentinel semantics inline -- 0 defers to the world default, so C behaves
// exactly as B would have WITHOUT its override, i.e. it contacts.
// ---------------------------------------------------------------------------
TEST(Contacts, PerBodyProxyRadiusOverridesWorldDefaultBothDirections) {
    const SdfProgram prog = GroundPlane();
    const ContactParams cp = MakeContacts(/*e=*/0.0f, /*mu=*/0.0f, /*radius=*/0.1f);

    BodyState a = MakeUnitBody(glm::vec3(0.0f, 0.0f, 0.15f), glm::vec3(0.0f, 0.0f, -1.0f));
    a.proxy_radius = 0.2f;  // override: default (0.1) would miss; this hits

    BodyState b = MakeUnitBody(glm::vec3(0.0f, 0.0f, 0.05f), glm::vec3(0.0f, 0.0f, -1.0f));
    b.proxy_radius = 0.02f;  // override: default (0.1) would hit; this misses

    BodyState c = MakeUnitBody(glm::vec3(0.0f, 0.0f, 0.05f), glm::vec3(0.0f, 0.0f, -1.0f));
    // c.proxy_radius left at 0 -- the sentinel; falls back to cp.proxy_radius.

    std::vector<BodyState> bodies{a, b, c};
    resolve_static_contacts(bodies, prog, cp, kH);

    EXPECT_EQ(bodies[0].vel.z, 0.0f) << "A: per-body override should have produced a contact";
    EXPECT_EQ(bodies[1].vel.z, -1.0f) << "B: per-body override should have suppressed the contact";
    EXPECT_EQ(bodies[2].vel.z, 0.0f) << "C: sentinel 0 should fall back to the world default (contact)";
}

// ---------------------------------------------------------------------------
// 2. Bounce apex ratio == e^2.
//
// Energy at the apex is m*g*H, and the impulse scales the approach speed by e,
// so H1/H0 = (e*v)^2 / v^2 = e^2 -- a statement about the RESTITUTION LAW that
// is independent of g, of the mass, and of the drop height. Two values of e,
// because a single one could be matched by a wrong exponent.
//
// Discrete-scheme bias, in the direction the measurement lands: the impulse is
// applied at a substep boundary, by which time the body has already sunk up to
// v_impact*h below the contact height, so the rebound starts LOW and the apex
// comes out slightly under e^2 (partially offset by the positional correction
// pushing the still-overlapping body up during the first substeps of the
// rebound). Symplectic Euler contributes a further n/(n+1) deficit in v^2 at
// impact. Measured here: -0.59% at e = 0.5 and -0.43% at e = 0.7, against the
// brief's 5% -- roughly 9x margin.
// ---------------------------------------------------------------------------

TEST(Contacts, BounceApexRatioMatchesRestitutionSquared) {
    const SdfProgram prog = GroundPlane();
    const WorldParams wp = MakeWorld(glm::vec3(0.0f, 0.0f, -kG));

    for (const float e : {0.5f, 0.7f}) {
        SCOPED_TRACE(testing::Message() << "e = " << e);

        const ContactParams cp = MakeContacts(e, /*mu=*/0.0f);
        constexpr float kStartZ = 2.0f;
        std::vector<BodyState> bodies{
            MakeUnitBody(glm::vec3(0.0f, 0.0f, kStartZ), glm::vec3(0.0f))};

        bool touched = false;
        float apex = 0.0f;
        for (int i = 0; i < 2500; ++i) {
            Step(bodies, prog, cp, wp);
            if (spade::eval(prog, bodies[0].pos) < kRadius) {
                touched = true;
            }
            if (touched) {
                // Every subsequent bounce is lower than the first (e < 1), so
                // the running max after first contact IS the first apex -- no
                // fragile local-maximum detection needed.
                apex = std::max(apex, bodies[0].pos.z);
            }
        }
        ASSERT_TRUE(touched);

        // Heights measured from the CONTACT height (z == radius), which is the
        // datum the impulse acts at, not from the plane.
        const float h0 = kStartZ - kRadius;
        const float h1 = apex - kRadius;
        const float ratio = h1 / h0;

        EXPECT_NEAR(ratio, e * e, 0.05f * e * e) << "apex " << apex << " (h1 = " << h1 << ")";

        // Guard against a degenerate pass: the body genuinely left the surface
        // and rose by several proxy radii, rather than the ratio being read
        // off a body that never separated. (Deliberately NOT phrased as a
        // fraction of e^2*h0 -- that is what EXPECT_NEAR above already tests,
        // and the discrete scheme lands a fraction of a percent BELOW e^2.)
        EXPECT_GT(h1, 4.0f * kRadius);
    }
}

// ---------------------------------------------------------------------------
// 3. Coulomb friction: a sliding body decelerates at exactly mu*g.
//
// Per substep the contact sees v_n = -g*h (one substep of gravity, applied by
// the PREVIOUS Integrate), so j_n = g*h and the Coulomb cap removes mu*g*h of
// tangential speed. Over N = T/h substeps that is mu*g*T -- independent of h,
// which is what makes "decelerates at mu*g" a real claim rather than a
// coincidence of this substep size.
// ---------------------------------------------------------------------------

TEST(Contacts, SlidingBodyDeceleratesAtMuG) {
    const SdfProgram prog = GroundPlane();
    const WorldParams wp = MakeWorld(glm::vec3(0.0f, 0.0f, -kG));
    constexpr float kMu = 0.3f;
    const ContactParams cp = MakeContacts(/*e=*/0.0f, kMu);

    // Starts just barely in contact (0.1 mm, inside the slop band) so the
    // positional correction is dormant and only friction is under test.
    //
    // kT is DERIVED from the substep count, not the other way round: `kH` is
    // 1e-3f, which is not exactly 1/1000 in binary, so static_cast<int>(0.5f /
    // kH) truncates to 499 and would silently shift the reference by a whole
    // substep of friction.
    constexpr float kV0 = 5.0f;
    constexpr int kSteps = 500;
    constexpr float kT = static_cast<float>(kSteps) * kH;  // 0.5 s
    std::vector<BodyState> bodies{
        MakeUnitBody(glm::vec3(0.0f, 0.0f, kRadius - 1e-4f), glm::vec3(kV0, 0.0f, 0.0f))};

    for (int i = 0; i < kSteps; ++i) {
        Step(bodies, prog, cp, wp);
    }

    const float expected = kV0 - kMu * kG * kT;  // 3.5285 m/s
    EXPECT_NEAR(bodies[0].vel.x, expected, 0.05f * expected);

    // The one-substep bias, named and bounded rather than absorbed by the 5%:
    // the first substep starts with v_z == 0, so no normal impulse fires and
    // that substep contributes no friction. Exactly one substep is missing, so
    // the overshoot must be mu*g*h = 2.94 mm/s out of a 1.47 m/s total loss
    // (0.083%). Measured: 2.9237 mm/s -- within 1% of the prediction, and the
    // 2x bound below would catch a scheme that missed two.
    EXPECT_GT(bodies[0].vel.x, expected);
    EXPECT_LT(bodies[0].vel.x - expected, 2.0f * kMu * kG * kH);

    // Motion stayed in the plane and the body neither sank nor was launched.
    EXPECT_NEAR(bodies[0].vel.y, 0.0f, 1e-6f);
    const float penetration = kRadius - bodies[0].pos.z;
    EXPECT_GT(penetration, 0.0f);
    EXPECT_LT(penetration, kSlop + 2.0f * kRestExcess);
}

// ---------------------------------------------------------------------------
// 4. Friction can stop a body but must never reverse it.
//
// The Coulomb cap is min(mu*j_n, |v_t|) and the |v_t| floor is the whole
// safety property: without it, a large mu or a hard landing subtracts more
// than the tangential velocity and friction ACCELERATES the body backwards.
// ---------------------------------------------------------------------------

TEST(Contacts, FrictionStopsButNeverReversesTangentialVelocity) {
    const SdfProgram prog = GroundPlane();
    const ContactParams cp = MakeContacts(/*e=*/0.0f, /*mu=*/5.0f);

    // (a) Single resolve, hard landing: the uncapped cap would be
    //     mu*j_n = 5*10 = 50 m/s against a 0.5 m/s tangential velocity, i.e.
    //     100x too much. The result must be an EXACT stop, not -49.5 m/s.
    {
        std::vector<BodyState> bodies{MakeUnitBody(
            glm::vec3(0.0f, 0.0f, kRadius - 0.5f * kSlop), glm::vec3(0.5f, 0.0f, -10.0f))};
        resolve_static_contacts(bodies, prog, cp, kH);

        EXPECT_EQ(bodies[0].vel.x, 0.0f);
        EXPECT_EQ(bodies[0].vel.y, 0.0f);
        EXPECT_EQ(bodies[0].vel.z, 0.0f);  // e == 0: the normal impulse kills v_n too
    }

    // (b) Two seconds of sliding with the same absurd mu: v_x decays to zero
    //     (in ~41 ms) and STAYS there -- never negative at any substep.
    {
        const WorldParams wp = MakeWorld(glm::vec3(0.0f, 0.0f, -kG));
        std::vector<BodyState> bodies{MakeUnitBody(
            glm::vec3(0.0f, 0.0f, kRadius - 0.5f * kSlop), glm::vec3(2.0f, 0.0f, 0.0f))};

        float min_vx = 2.0f;
        for (int i = 0; i < 2000; ++i) {
            Step(bodies, prog, cp, wp);
            min_vx = std::min(min_vx, bodies[0].vel.x);
        }
        EXPECT_GE(min_vx, 0.0f) << "friction reversed the tangential velocity";
        EXPECT_EQ(bodies[0].vel.x, 0.0f);
    }

    // (c) The same stop with a DIAGONAL tangential velocity, which is where
    //     the exactness of (a) and (b) stops being free: v_t/|v_t| is not
    //     representable, so |v_t| * normalize(v_t) reproduces v_t only to
    //     within a couple of roundings. The stop is therefore exact up to a
    //     residue of order eps*|v_t| -- a millionth of a metre per second,
    //     which the NEXT substep's cap removes again -- and NOT a reversal at
    //     anything like the original speed. This bound is the honest form of
    //     "friction never reverses": pinned here so a future change to the
    //     normalize/scale grouping cannot quietly turn it into something
    //     larger.
    {
        const glm::vec3 v_t0(3.0f, -2.0f, 0.0f);
        std::vector<BodyState> bodies{MakeUnitBody(
            glm::vec3(0.0f, 0.0f, kRadius - 0.5f * kSlop), v_t0 + glm::vec3(0.0f, 0.0f, -10.0f))};
        resolve_static_contacts(bodies, prog, cp, kH);

        const glm::vec3 v_t1(bodies[0].vel.x, bodies[0].vel.y, 0.0f);
        EXPECT_LT(glm::length(v_t1), 4.0f * 1.19209290e-7f * glm::length(v_t0));
        EXPECT_EQ(bodies[0].vel.z, 0.0f);
    }
}

// ---------------------------------------------------------------------------
// 5. A sphere settles stably in a right-angle corner.
//
// THE SINGLE-CONTACT MODEL AT A CORNER. The union reports one distance,
// min(phi_floor, phi_wall), and one gradient (that branch's), so only the
// NEAREST face is resolved each substep. That is not a failure mode here, it
// is a Gauss-Seidel sweep: the branch chosen is always the more penetrated
// one, correcting it grows its distance until the other becomes nearest, and
// the alternation drives both penetrations into the slop band. The gradient at
// the seam is still exactly unit length -- the hard union SELECTS a branch
// rather than averaging the two, so there is no short-normal case here (the
// smooth-union test below covers the case where there is).
//
// This test therefore asserts STABILITY -- both faces respected, no jitter, no
// creep -- and deliberately NOT an exact trajectory, since which face wins on
// which substep is a property of the union, not of the physics.
// ---------------------------------------------------------------------------

TEST(Contacts, SphereSettlesStablyInACorner) {
    const SdfProgram prog = FloorAndWallCorner();
    const WorldParams wp = MakeWorld(glm::vec3(0.0f, 0.0f, -kG));
    const ContactParams cp = MakeContacts(/*e=*/0.0f, /*mu=*/0.3f);

    // Launched at the corner: it falls onto the floor, slides in -x under
    // friction, hits the wall, and must end up touching both.
    std::vector<BodyState> bodies{
        MakeUnitBody(glm::vec3(1.0f, 0.0f, 1.0f), glm::vec3(-2.0f, 0.0f, 0.0f))};

    float max_speed_early = 0.0f;
    float max_speed_late = 0.0f;
    float min_x_late = 10.0f, max_x_late = -10.0f;
    float min_z_late = 10.0f, max_z_late = -10.0f;
    float worst_phi = 10.0f;

    constexpr int kSteps = 6000;
    for (int i = 0; i < kSteps; ++i) {
        Step(bodies, prog, cp, wp);
        worst_phi = std::min(worst_phi, spade::eval(prog, bodies[0].pos));

        const float speed = glm::length(bodies[0].vel);
        if (i >= 4000 && i < 5000) {
            max_speed_early = std::max(max_speed_early, speed);
        }
        if (i >= 5000) {
            max_speed_late = std::max(max_speed_late, speed);
            min_x_late = std::min(min_x_late, bodies[0].pos.x);
            max_x_late = std::max(max_x_late, bodies[0].pos.x);
            min_z_late = std::min(min_z_late, bodies[0].pos.z);
            max_z_late = std::max(max_z_late, bodies[0].pos.z);
        }
    }

    // BOTH faces are respected simultaneously -- this is the assertion the
    // single-contact model has to earn. phi_floor == pos.z and
    // phi_wall == pos.x for this program, so the two depths are read directly.
    const float depth_floor = kRadius - bodies[0].pos.z;
    const float depth_wall = kRadius - bodies[0].pos.x;
    EXPECT_GT(depth_floor, 0.0f);
    EXPECT_GT(depth_wall, 0.0f);
    EXPECT_LT(depth_floor, kSlop + 2.0f * kRestExcess);
    EXPECT_LT(depth_wall, kSlop + 2.0f * kRestExcess);

    // Never inside either solid at any substep, on the way in or at rest.
    EXPECT_GT(worst_phi, 0.0f);

    // At rest, and no jitter: the alternating branch selection did not turn
    // into a limit cycle. Measured on this box: the position is bit-stable
    // over the last 1000 substeps and the speed is exactly g*h.
    EXPECT_LT(glm::length(bodies[0].vel), 2.0f * kG * kH);
    EXPECT_LE(max_speed_late, max_speed_early + 1e-6f);
    EXPECT_LT(max_x_late - min_x_late, 1e-6f);
    EXPECT_LT(max_z_late - min_z_late, 1e-6f);
}

// ---------------------------------------------------------------------------
// 6. Curved obstacle: the normal is the (radial) surface normal, not an axis.
//
// A body dropped off-centre onto a unit sphere must be deflected sideways --
// the impulse cancels only the RADIAL velocity component and leaves the
// tangential one, so the body slides down the flank and off. A normal taken
// from anything but the field gradient (an axis, a fixed up-vector) would
// stop it dead instead.
// ---------------------------------------------------------------------------

TEST(Contacts, CurvedObstacleDeflectsAlongTheRadialNormal) {
    const SdfProgram prog = UnitSphereObstacle();
    const WorldParams wp = MakeWorld(glm::vec3(0.0f, 0.0f, -kG));
    const ContactParams cp = MakeContacts(/*e=*/0.0f, /*mu=*/0.0f);

    std::vector<BodyState> bodies{MakeUnitBody(glm::vec3(0.3f, 0.0f, 2.0f), glm::vec3(0.0f))};

    float min_phi = 10.0f;
    for (int i = 0; i < 1500; ++i) {
        Step(bodies, prog, cp, wp);
        min_phi = std::min(min_phi, spade::eval(prog, bodies[0].pos));
    }

    // Slid off the flank rather than sticking where it landed.
    EXPECT_GT(bodies[0].pos.x, 1.0f);
    EXPECT_GT(bodies[0].vel.x, 0.5f);
    EXPECT_LT(bodies[0].pos.z, 0.0f);  // fell past the obstacle's equator and away

    // Never sank into the obstacle: the grazing contact was resolved without
    // the proxy passing through the curved surface.
    EXPECT_GT(min_phi, 0.5f * kRadius);
}

// ---------------------------------------------------------------------------
// 7. The gradient is NORMALIZED, not assumed unit.
//
// smooth_union blends the two branches' gradients (gb + (ga - gb)*h), so
// halfway along the blend of two perpendicular unit normals the field gradient
// has length 1/sqrt(2). Using it raw would scale every impulse by that factor.
// This is a SINGLE resolve call so the arithmetic is checkable by hand.
// ---------------------------------------------------------------------------

TEST(Contacts, NonUnitGradientIsNormalizedBeforeTheImpulse) {
    constexpr float kK = 0.5f;
    const SdfProgram prog = SmoothCorner(kK);

    // On the diagonal: phi_floor == phi_wall == 0.3, so the blend weight is
    // exactly 0.5 and grad == 0.5*(1,0,0) + 0.5*(0,0,1), length 0.7071.
    const glm::vec3 p(0.3f, 0.0f, 0.3f);
    const glm::vec3 raw_grad = spade::gradient(prog, p);
    const float raw_len = glm::length(raw_grad);
    ASSERT_LT(raw_len, 0.99f) << "test is vacuous unless the gradient is short here";
    ASSERT_GT(raw_len, 0.5f);

    const float phi = spade::eval(prog, p);
    ASSERT_LT(phi, 0.2f);  // 0.175: inside a 0.2 m proxy
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.0f, /*radius=*/0.2f);

    const glm::vec3 n = raw_grad / raw_len;
    const glm::vec3 v0(-1.0f, 0.0f, -1.0f);
    std::vector<BodyState> bodies{MakeUnitBody(p, v0)};

    resolve_static_contacts(bodies, prog, cp, kH);

    const float v_n0 = glm::dot(v0, n);  // -1.41421
    ASSERT_LT(v_n0, 0.0f);

    // Restitution holds with respect to the UNIT normal: the separating speed
    // is e times the approach speed. With the raw gradient the factor would be
    // off by raw_len (0.707), leaving the body still approaching at
    // -0.354 m/s instead of separating at +0.707 m/s.
    EXPECT_NEAR(glm::dot(bodies[0].vel, n), -cp.restitution_e * v_n0, 1e-5f);

    // ...and the velocity change is (1+e)*|v_n| along n exactly.
    const glm::vec3 dv = bodies[0].vel - v0;
    EXPECT_NEAR(glm::length(dv), (1.0f + cp.restitution_e) * std::abs(v_n0), 1e-5f);
    EXPECT_NEAR(glm::length(glm::cross(dv, n)), 0.0f, 1e-5f);

    // The positional correction is beta * (depth - slop) along the same unit
    // normal -- also normalized, also hand-checkable.
    const float depth = cp.proxy_radius - phi;
    EXPECT_NEAR(glm::length(bodies[0].pos - p), kBeta * (depth - kSlop), 1e-6f);
}

// ---------------------------------------------------------------------------
// 8. Positional-correction boundary cases: dormant inside the slop band, and
//    applied to a SEPARATING contact (where no impulse fires).
// ---------------------------------------------------------------------------

TEST(Contacts, PenetrationInsideSlopIsLeftExactlyAlone) {
    const SdfProgram prog = GroundPlane();
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.9f);

    // Half a slop deep, at rest: in contact (phi < radius) but with nothing to
    // correct and nothing to impulse. Must be a bit-exact no-op, which is what
    // makes a resting body's position stationary rather than dithering.
    std::vector<BodyState> bodies{
        MakeUnitBody(glm::vec3(0.0f, 0.0f, kRadius - 0.5f * kSlop), glm::vec3(0.0f))};
    const BodyState before = bodies[0];

    resolve_static_contacts(bodies, prog, cp, kH);

    EXPECT_EQ(std::memcmp(&bodies[0], &before, sizeof(BodyState)), 0);
}

TEST(Contacts, SeparatingContactGetsCorrectionButNoImpulseAndNoFriction) {
    const SdfProgram prog = GroundPlane();
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.9f);

    // Still overlapping (5 mm) but already moving away. A second impulse here
    // would be the classic double-hit energy pump; friction must also vanish,
    // because j_n == 0 means there is no normal force to be proportional to.
    const glm::vec3 p(0.0f, 0.0f, kRadius - 5.0f * kSlop);
    const glm::vec3 v(1.0f, 0.0f, 3.0f);
    std::vector<BodyState> bodies{MakeUnitBody(p, v)};

    resolve_static_contacts(bodies, prog, cp, kH);

    EXPECT_EQ(bodies[0].vel.x, v.x);  // no friction without a normal impulse
    EXPECT_EQ(bodies[0].vel.y, v.y);
    EXPECT_EQ(bodies[0].vel.z, v.z);  // no second impulse

    // The correction still runs: position and velocity are separate levels.
    const float depth = kRadius - spade::eval(prog, p);
    EXPECT_NEAR(bodies[0].pos.z - p.z, kBeta * (depth - kSlop), 1e-7f);
}

TEST(Contacts, TangentBodyIsNotAContact) {
    const SdfProgram prog = GroundPlane();
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.9f);

    // phi == radius exactly: the test is strict `<`, so this is not a contact
    // and nothing at all happens -- including to a body moving fast towards
    // the surface, which will be caught next substep instead.
    std::vector<BodyState> bodies{
        MakeUnitBody(glm::vec3(0.0f, 0.0f, kRadius), glm::vec3(1.0f, 0.0f, -4.0f))};
    const BodyState before = bodies[0];

    resolve_static_contacts(bodies, prog, cp, kH);

    EXPECT_EQ(std::memcmp(&bodies[0], &before, sizeof(BodyState)), 0);
}

// ---------------------------------------------------------------------------
// 9. The LINEAR-ONLY ruling (contacts.hpp limitation 1), pinned as a test:
//    the pass writes `pos` and `vel` and NOTHING else. In particular a sliding
//    sphere gains no angular velocity -- it never starts rolling.
// ---------------------------------------------------------------------------

TEST(Contacts, ResponseIsLinearOnlyAndWritesNothingButPosAndVel) {
    const SdfProgram prog = GroundPlane();
    const ContactParams cp = MakeContacts(/*e=*/0.3f, /*mu=*/0.6f);

    BodyState body = MakeUnitBody(glm::vec3(0.0f, 0.0f, kRadius - 5.0f * kSlop),
                                  glm::vec3(4.0f, -1.0f, -2.0f));
    body.orient = glm::normalize(glm::quat(0.6f, 0.1f, 0.4f, -0.3f));
    body.omega_body = glm::vec3(0.3f, -0.7f, 1.1f);
    body.force_acc = glm::vec3(1.0f, 2.0f, 3.0f);
    body.torque_acc = glm::vec3(0.4f, 0.5f, 0.6f);
    body.specific_force = glm::vec3(7.0f, 8.0f, 9.0f);

    std::vector<BodyState> bodies{body};

    // Resolve repeatedly WITHOUT integrating, so nothing else can be the
    // author of a change: only this pass runs.
    for (int i = 0; i < 8; ++i) {
        resolve_static_contacts(bodies, prog, cp, kH);
    }

    // Something must actually have happened, or the comparison below is
    // vacuous: the first call cancelled v_z and bit into v_x with friction.
    EXPECT_NE(bodies[0].vel.z, -2.0f);
    EXPECT_LT(bodies[0].vel.x, 4.0f);

    // Now take the expected state to be the ORIGINAL with pos and vel patched
    // to whatever the pass produced, and compare byte-wise. Any write to any
    // other field -- an angular impulse, a torque, a cleared accumulator, a
    // stray pad byte -- fails this.
    BodyState expected = body;
    expected.pos = bodies[0].pos;
    expected.vel = bodies[0].vel;
    EXPECT_EQ(std::memcmp(&bodies[0], &expected, sizeof(BodyState)), 0)
        << "the pass wrote a field other than pos/vel";

    // Spelled out separately for the reader, since this is the ruling itself:
    // the sphere slid under friction and did not start rolling.
    EXPECT_EQ(bodies[0].omega_body.x, 0.3f);
    EXPECT_EQ(bodies[0].omega_body.y, -0.7f);
    EXPECT_EQ(bodies[0].omega_body.z, 1.1f);
}

// ---------------------------------------------------------------------------
// 10. Determinism (charter P8, D11) and the flags contract.
// ---------------------------------------------------------------------------

TEST(Contacts, TwoIdenticalRunsAreByteIdentical) {
    const SdfProgram prog = FloorAndWallCorner();
    const WorldParams wp = MakeWorld(glm::vec3(0.0f, 0.0f, -kG));
    const ContactParams cp = MakeContacts(/*e=*/0.4f, /*mu=*/0.35f);

    // Built twice from the same code path rather than copied, so this checks
    // "the output is a function of the input", not "memcpy works".
    const auto make_world = []() {
        std::vector<BodyState> bodies(4);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const float k = static_cast<float>(i);
            bodies[i] = MakeUnitBody(glm::vec3(0.4f + 0.3f * k, 0.1f * k, 0.6f + 0.2f * k),
                                     glm::vec3(-1.0f - 0.2f * k, 0.3f, -2.0f + 0.1f * k));
            bodies[i].mass = 1.0f + 0.25f * k;
        }
        // One tombstoned slot in the middle of the span: the skip path is part
        // of what has to be deterministic.
        bodies[2].flags = 0u;
        return bodies;
    };

    std::vector<BodyState> run_a = make_world();
    std::vector<BodyState> run_b = make_world();

    for (int i = 0; i < 1500; ++i) {
        Step(run_a, prog, cp, wp);
        Step(run_b, prog, cp, wp);
    }

    ASSERT_EQ(run_a.size(), run_b.size());
    EXPECT_EQ(std::memcmp(run_a.data(), run_b.data(), run_a.size() * sizeof(BodyState)), 0);

    // Guard against the comparison passing because nothing collided: body 0
    // bounced (e = 0.4) down to rest on the floor. Its resting penetration is
    // slightly SHALLOWER than the e == 0 case -- with restitution the
    // steady-state post-impulse velocity is u = e*g*h/(1+e), so the per-substep
    // sink is (g*h - u)*h rather than g*h^2 and the equilibrium is
    // slop + (g*h^2/beta)/(1+e). Measured: 1.035 mm.
    EXPECT_LT(kRadius - run_a[0].pos.z, kSlop + 2.0f * kRestExcess);
    EXPECT_GT(kRadius - run_a[0].pos.z, 0.0f);
}

TEST(Contacts, InactiveBodiesAreLeftByteIdentical) {
    const SdfProgram prog = GroundPlane();
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.4f);

    // All three are deeply penetrating and moving into the surface, so the
    // only thing separating them is the flags bit.
    std::vector<BodyState> bodies(3);
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        bodies[i] = MakeUnitBody(glm::vec3(0.0f, 0.0f, -0.05f), glm::vec3(2.0f, 0.0f, -6.0f));
    }
    bodies[1].flags = 0u;         // tombstoned / never spawned
    bodies[2].flags = 1u << 3;    // some OTHER predicate bit, but not `active`

    const BodyState before_zero_flags = bodies[1];
    const BodyState before_other_flag = bodies[2];

    resolve_static_contacts(bodies, prog, cp, kH);

    EXPECT_EQ(std::memcmp(&bodies[1], &before_zero_flags, sizeof(BodyState)), 0);
    EXPECT_EQ(std::memcmp(&bodies[2], &before_other_flag, sizeof(BodyState)), 0);

    // The active neighbour was resolved, so the span really was traversed --
    // and it was resolved through a NEGATIVE phi (origin inside the solid),
    // where depth = radius - phi = 0.15 m exceeds the proxy radius.
    EXPECT_GT(bodies[0].vel.z, 0.0f);
    EXPECT_NEAR(bodies[0].pos.z - (-0.05f), kBeta * ((kRadius + 0.05f) - kSlop), 1e-6f);
}

// ---------------------------------------------------------------------------
// 11. ContactParams' layout discipline (coordinator ruling: pass-parameter
//     value structs stay in their pass header but carry layout.hpp's battery).
//
// The static_asserts in contacts.hpp are the real enforcement -- a violation is
// a build error, so this file compiling at all is already coverage. What they
// CANNOT express is the runtime half: that the array stride a std430 upload
// would use is sizeof, and that a defaulted record has a fully determined byte
// image (the reserved lanes carry initializers rather than picking up whatever
// the stack held), which is what makes the record hashable and uploadable
// byte-wise. Mirrors test_state.cpp's
// StateLayout.WorldParamsTailPaddingIsZeroInAnArena, for the same reason.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// PHY-7: the IMU's specific force includes the contact response. The contact
// passes add each body's velocity change to a transient scratch; Integrate
// adds it over h to the specific force (not to vel, which already took the
// impulse) and zeroes EVERY slot of the scratch, so it is zero at every
// substep boundary (Core's condition 1).
// ---------------------------------------------------------------------------
TEST(Contacts, TheContactScratchIsZeroAtTheBoundaryForEverySlot) {
    const SdfProgram prog = GroundPlane();
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.4f);
    const WorldParams wp = MakeWorld(glm::vec3(0.0f, 0.0f, -kG));

    std::vector<BodyState> bodies(2, MakeUnitBody(glm::vec3(0.0f, 0.0f, 0.05f), glm::vec3(1.0f, 0.0f, -2.0f)));
    std::vector<glm::vec3> dv(bodies.size(), glm::vec3(0.0f));
    const glm::vec3 vel_in = bodies[0].vel;

    resolve_static_contacts(bodies, prog, cp, kH, dv);
    ASSERT_NE(dv[0], glm::vec3(0.0f)) << "the contact pass wrote no velocity change";
    EXPECT_EQ(dv[0], bodies[0].vel - vel_in) << "the scratch must hold the change the state took, exactly";
    EXPECT_EQ(dv[1], dv[0]);

    bodies[1].flags = 0u;  // skipped by Integrate from here, as a despawn between the passes would leave it
    integrate_bodies(bodies, wp, kH, dv);
    EXPECT_EQ(dv[0], glm::vec3(0.0f)) << "an active body's contact change outlived its substep";
    EXPECT_EQ(dv[1], glm::vec3(0.0f))
        << "a body Integrate skips kept its contact change: it would read as a phantom contact later";
}

TEST(Contacts, IntegrateAddsTheContactChangeToTheSpecificForceAndNothingElse) {
    const SdfProgram prog = GroundPlane();
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.4f);
    const WorldParams wp = MakeWorld(glm::vec3(0.0f, 0.0f, -kG));
    // A literal unit quaternion, 30 degrees about Z: the body frame differs
    // from the world's, so the change is rotated as the force is.
    const glm::quat q = glm::normalize(glm::quat(0.9659258f, 0.0f, 0.0f, 0.2588190f));

    std::vector<BodyState> bodies{MakeUnitBody(glm::vec3(0.0f, 0.0f, 0.05f), glm::vec3(1.0f, 0.0f, -2.0f))};
    bodies[0].orient = q;
    std::vector<glm::vec3> dv(1, glm::vec3(0.0f));
    resolve_static_contacts(bodies, prog, cp, kH, dv);
    const glm::vec3 change = dv[0];
    ASSERT_NE(change, glm::vec3(0.0f));

    std::vector<BodyState> plain = bodies;
    integrate_bodies(bodies, wp, kH, dv);
    integrate_bodies(plain, wp, kH);

    EXPECT_EQ(bodies[0].specific_force, glm::conjugate(q) * (change / kH));
    BodyState a = bodies[0];
    BodyState b = plain[0];
    a.specific_force = glm::vec3(0.0f);
    b.specific_force = glm::vec3(0.0f);
    EXPECT_EQ(std::memcmp(&a, &b, sizeof(BodyState)), 0)
        << "Integrate changed more than the specific force: vel took the impulse in the contact pass already";
}

TEST(Contacts, ContactParamsIsStd430SafeAndByteDetermined) {
    static_assert(sizeof(ContactParams) == 32, "two 16-byte std430 rows");
    static_assert(alignof(ContactParams) == 16, "std430 base alignment");

    // Array stride == sizeof: what uploading an array of these verbatim needs.
    // A gap between elements would break the indexing a GPU thread does to
    // reach its world's row.
    const ContactParams arr[2]{};
    const std::ptrdiff_t stride =
        reinterpret_cast<const char*>(&arr[1]) - reinterpret_cast<const char*>(&arr[0]);
    EXPECT_EQ(stride, static_cast<std::ptrdiff_t>(sizeof(ContactParams)));
    EXPECT_EQ(reinterpret_cast<std::uintptr_t>(&arr[0]) % alignof(ContactParams), 0u);

    // A defaulted record's whole 32-byte image, field by field in declared
    // order. This pins the DEFAULTS and the ORDER together: a reordering that
    // moved the initializers with it would still satisfy the offsetof asserts
    // (those follow the declaration) but fail here.
    const ContactParams cp{};
    const float expected[8] = {0.0f, 0.0f, kBeta, kSlop, 0.0f, 0.0f, 0.0f, 0.0f};
    EXPECT_EQ(std::memcmp(&cp, expected, sizeof(ContactParams)), 0);

    // ...and the reserved lanes really are zero, stated separately so the
    // intent survives an edit to `expected`.
    EXPECT_EQ(cp._r0, 0.0f);
    EXPECT_EQ(cp._r1, 0.0f);
    EXPECT_EQ(cp._r2, 0.0f);
}

// ---------------------------------------------------------------------------
// 12. Degenerate inputs that must not produce NaNs or motion.
// ---------------------------------------------------------------------------

TEST(Contacts, EmptyWorldProducesNoContacts) {
    const SdfProgram empty;  // no nodes: eval returns kSdfEmptyDistance
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.4f);

    std::vector<BodyState> bodies{
        MakeUnitBody(glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(1.0f, 0.0f, -5.0f))};
    const BodyState before = bodies[0];

    resolve_static_contacts(bodies, empty, cp, kH);

    EXPECT_EQ(std::memcmp(&bodies[0], &before, sizeof(BodyState)), 0);
}

TEST(Contacts, ZeroGradientPointIsSkippedRatherThanImpulsedIntoNaN) {
    // The centre of a sphere primitive is the one place a well-formed program
    // has no defined normal (every direction is a subgradient); sdf.cpp
    // reports a zero gradient there. A naive normalize() would be 0/0.
    const SdfProgram prog = UnitSphereObstacle();
    const ContactParams cp = MakeContacts(/*e=*/0.5f, /*mu=*/0.4f);

    std::vector<BodyState> bodies{MakeUnitBody(glm::vec3(0.0f), glm::vec3(1.0f, 2.0f, 3.0f))};
    const BodyState before = bodies[0];

    resolve_static_contacts(bodies, prog, cp, kH);

    EXPECT_EQ(std::memcmp(&bodies[0], &before, sizeof(BodyState)), 0);
    EXPECT_FALSE(std::isnan(bodies[0].pos.x));
    EXPECT_FALSE(std::isnan(bodies[0].vel.x));
}
