#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/forces.hpp"
#include "physics/integrator.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"
#include "world/medium.hpp"

// ---------------------------------------------------------------------------
// Force-element infrastructure + DragBody (engine design spec S3's
// ForceElements pass; coordinator resolution for this task). Two force laws
// share one row type (DragBodyRow, physics/forces.hpp) and one entry point
// (apply_drag()):
//
//   quadratic      F = -1/2 * rho * Cd * A * |v_rel| * v_rel, WORLD frame.
//   componentwise  kat's EXISTING drag convention -- see
//                  configs/physics.yaml (worktree root), keys `drag_coeff`
//                  and `drag_mode`, cross-checked below against
//                  controller/dynamics/quadrotor.py's _drag_force_jit() and
//                  interface/src/physics/quadrotor.cpp's drag_accel(). The
//                  DragComponentwiseSingleStepMatchesPhysicsYamlConvention
//                  test is the one that pins this reading numerically.
//
// Both laws are exercised the same two ways every physics law in this test
// tree is: an independent closed-form check (terminal velocity under gravity
// -- the brief's required proof for quadratic, and repeated for
// componentwise as a stronger cross-check than a single formula evaluation
// alone would be), and the structural contracts (disabled/inactive skip,
// the torque's offset-and-frame handling, determinism) that a closed form
// cannot see.
// ---------------------------------------------------------------------------

namespace {

using spade::BodyState;
using spade::ConstantMedium;
using spade::WorldParams;
using spade::physics::apply_drag;
using spade::physics::DragBodyRow;
namespace drag_mode = spade::physics::drag_mode;
namespace body_flags = spade::physics::body_flags;

void ExpectVec3Near(const glm::vec3& actual, const glm::vec3& expected, float atol) {
    EXPECT_NEAR(actual.x, expected.x, atol);
    EXPECT_NEAR(actual.y, expected.y, atol);
    EXPECT_NEAR(actual.z, expected.z, atol);
}

// A live unit body: identity attitude, 1 kg, at rest, origin, no accumulated
// wrench -- same shape as test_integrator.cpp's MakeUnitBody, value-
// initialized first so std430 pad bytes are deterministically zero (the
// determinism test below memcmps whole BodyStates).
BodyState MakeUnitBody() {
    BodyState body{};
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    body.flags = body_flags::active;
    return body;
}

WorldParams MakeParams(const glm::vec3& gravity, float air_density, const glm::vec3& wind) {
    WorldParams params{};
    params.gravity = gravity;
    params.air_density = air_density;
    params.wind = wind;
    return params;
}

// A default-constructed DragBodyRow is all-zero (enabled == 0, i.e. inert),
// same convention as a zero-filled arena slot. Callers set only what their
// scenario needs.
DragBodyRow MakeElem(uint32_t body_slot, uint32_t mode, const glm::vec3& coeffs, float area,
                     const glm::vec3& local_pos = glm::vec3(0.0f)) {
    DragBodyRow elem{};
    elem.body_slot = body_slot;
    elem.enabled = 1u;
    elem.mode = mode;
    elem.area = area;
    elem.local_pos = local_pos;
    elem.local_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    elem.coeffs = coeffs;
    return elem;
}

// Substep size exact in binary (2^-10) -- same choice test_integrator.cpp
// makes, and for the same reason: the terminal-velocity tests below run
// thousands of substeps, so this keeps the only error in play genuine fp32
// accumulation rather than a non-representable `h`.
constexpr float kH = 1.0f / 1024.0f;

}  // namespace

// ---------------------------------------------------------------------------
// Layout + registered-state contracts (mirrors test_state.cpp's
// StateLayout.StaticAssertedSizesAreVisibleAtRuntime and the registered-array
// tests -- DragBodyRow is state a snapshot walk must cover, so it has to be
// usable with ArenaSet::register_array exactly like BodyState is).
// ---------------------------------------------------------------------------

TEST(ForceElements, DragBodyRowLayoutSizesAreVisibleAtRuntime) {
    EXPECT_EQ(sizeof(DragBodyRow), 64u);
    EXPECT_EQ(alignof(DragBodyRow), 16u);
    EXPECT_EQ(offsetof(DragBodyRow, body_slot), 0u);
    EXPECT_EQ(offsetof(DragBodyRow, enabled), 4u);
    EXPECT_EQ(offsetof(DragBodyRow, mode), 8u);
    EXPECT_EQ(offsetof(DragBodyRow, area), 12u);
    EXPECT_EQ(offsetof(DragBodyRow, local_pos), 16u);
    EXPECT_EQ(offsetof(DragBodyRow, local_orient), 32u);
    EXPECT_EQ(offsetof(DragBodyRow, coeffs), 48u);
}

TEST(ForceElements, DragBodyRowIsRegisteredStateAndZeroFillsInert) {
    spade::ArenaSet arenas(1);
    const auto elems_id = arenas.register_array<DragBodyRow>("drag_bodies", 4);
    ASSERT_TRUE(elems_id.has_value());

    const spade::RegisteredArray* entry = arenas.registry().find("drag_bodies");
    ASSERT_NE(entry, nullptr) << "an unregistered drag-element array is state a snapshot would miss";
    EXPECT_EQ(entry->elem_size, sizeof(DragBodyRow));
    EXPECT_EQ(entry->capacity_per_world, 4u);

    const auto slot = arenas.alloc_slot(*elems_id, 0);
    ASSERT_TRUE(slot.has_value());
    const auto span = arenas.array(*elems_id);
    ASSERT_TRUE(span.has_value());

    // ArenaSet zero-fills on registration and alloc leaves the bytes alone,
    // so a fresh slot's `enabled` is 0 -- inert by construction, same as
    // BodyState::flags == 0 leaving a body untouched (physics/integrator.hpp).
    EXPECT_EQ((*span)[*slot].enabled, 0u);
    EXPECT_EQ((*span)[*slot].mode, 0u);
    EXPECT_EQ((*span)[*slot].body_slot, 0u);
}

// ---------------------------------------------------------------------------
// Quadratic mode: terminal velocity vs the closed form (task-required proof).
//
// At terminal velocity the drag magnitude exactly cancels weight:
//   1/2 * rho * Cd * A * v_t^2 = m * g  =>  v_t = sqrt(2*m*g / (rho*Cd*A))
//
// A body dropped from rest under gravity + this drag element approaches v_t
// as v(t) = v_t * tanh(t / tau), tau = v_t / g -- so simulating for many
// multiples of tau converges to v_t to far better than the required 1%
// regardless of the exact integrator error, and the test is checking the
// FORCE LAW, not the integrator (test_integrator.cpp already owns the
// integrator's own accuracy).
// ---------------------------------------------------------------------------

TEST(ForceElements, DragQuadraticTerminalVelocityMatchesClosedForm) {
    constexpr float kMass = 0.5f;      // kg
    constexpr float kGravity = 9.81f;  // m/s^2
    constexpr float kRho = 1.225f;     // kg/m^3
    constexpr float kCd = 5.0f;        // dimensionless
    constexpr float kArea = 0.1f;      // m^2

    const WorldParams params = MakeParams(glm::vec3(0.0f, 0.0f, -kGravity), kRho, glm::vec3(0.0f));
    const ConstantMedium medium;

    BodyState body = MakeUnitBody();
    body.mass = kMass;
    const DragBodyRow elem = MakeElem(0, drag_mode::quadratic, glm::vec3(kCd, 0.0f, 0.0f), kArea);

    std::vector<BodyState> bodies{body};
    const std::vector<DragBodyRow> elems{elem};

    // tau = v_t/g ~= 0.408 s here; 4 s is ~9.8 tau, where tanh has converged
    // to within double-precision epsilon of 1 -- far tighter than the 1%
    // this test demands, so the bound below is genuinely testing the force
    // law rather than "did we run long enough".
    constexpr int kSteps = 4096;  // T = 4.0 s
    for (int i = 0; i < kSteps; ++i) {
        apply_drag(bodies, elems, medium, params, kH);
        spade::physics::integrate_bodies(bodies, params, kH);
    }

    const double v_t = std::sqrt(2.0 * kMass * kGravity / (kRho * kCd * kArea));  // ~4.002 m/s

    EXPECT_NEAR(static_cast<double>(bodies[0].vel.z), -v_t, v_t * 0.01) << "v_t = " << v_t;
    ExpectVec3Near(glm::vec3(bodies[0].vel.x, bodies[0].vel.y, 0.0f), glm::vec3(0.0f), 1e-4f);

    // Guard against a vacuous pass (e.g. drag silently not applying, so the
    // body just free-falls and never approaches any finite speed).
    EXPECT_GT(v_t, 1.0);
    EXPECT_LT(v_t, 100.0);
}

// ---------------------------------------------------------------------------
// Componentwise mode: matches kat's existing drag convention.
//
// configs/physics.yaml (worktree root), the `drag_coeff`/`drag_mode` keys:
//
//   drag_coeff: 0.0280 # drag coefficient (kg/m for quadratic/componentwise)
//   drag_mode: componentwise # ... 'componentwise' (F_i=-cd*|v_i|*v_i)
//
// and the file's header comment on those same keys: "componentwise quadratic
// drag, F_i = -cd*|v_i|*v_i per BODY axis (matches the sim's per-axis
// VelocityDragCoefficients)" (S6 hygiene: the header comment used to drop the
// minus sign; reconciled in configs/physics.yaml, see forces.cpp's matching
// note). Cross-checked against the two engines that
// implement it: controller/dynamics/quadrotor.py's _drag_force_jit()
// "componentwise" branch (rotate v into body frame, apply
// drag_coeff*|v_i|*v_i per axis) and interface/src/physics/quadrotor.cpp's
// drag_accel() (`k * |vb_i|*vb_i` per axis, SUBTRACTED by its caller -- the
// yaml's explicit minus sign, spelled as a subtraction there instead of a
// literal '-').
//
// This test uses the yaml's own numbers (mass and drag_coeff straight from
// configs/physics.yaml) so the check is against the real config, not a
// stand-in. Velocity is nonzero and DISTINCT on all three axes so a bug that
// coupled the axes (the exact defect the yaml's header comment says the
// componentwise form exists to avoid -- see its "MEASURED 2026-07-29" note
// on the magnitude-coupled form being wrong by up to 3.4x) would show up as
// a wrong ratio between components, not just a wrong scale.
// ---------------------------------------------------------------------------

TEST(ForceElements, DragComponentwiseSingleStepMatchesPhysicsYamlConvention) {
    constexpr float kDragCoeff = 0.0280f;  // configs/physics.yaml: drag_coeff
    const WorldParams params = MakeParams(glm::vec3(0.0f), 1.225f, glm::vec3(0.0f));
    const ConstantMedium medium;

    BodyState body = MakeUnitBody();
    body.mass = 0.700f;  // configs/physics.yaml: mass
    body.vel = glm::vec3(12.0f, -7.5f, 3.25f);
    const DragBodyRow elem =
        MakeElem(0, drag_mode::componentwise, glm::vec3(kDragCoeff), /*area=*/0.0f /* unused in this mode */);

    std::vector<BodyState> bodies{body};
    const std::vector<DragBodyRow> elems{elem};
    apply_drag(bodies, elems, medium, params, kH);

    // F_i = -cd * |v_i| * v_i per axis, in double so the reference is not
    // itself computed with the code under test.
    const double c = static_cast<double>(kDragCoeff);
    const glm::dvec3 v(body.vel);
    const glm::dvec3 expected(-c * std::abs(v.x) * v.x, -c * std::abs(v.y) * v.y, -c * std::abs(v.z) * v.z);

    ExpectVec3Near(bodies[0].force_acc, glm::vec3(expected), 1e-4f);

    // Identity local pose (the common case, per the coordinator's
    // resolution): zero torque despite a very much nonzero force.
    ExpectVec3Near(bodies[0].torque_acc, glm::vec3(0.0f), 0.0f);
    EXPECT_GT(glm::length(bodies[0].force_acc), 1.0f);
}

// Bonus cross-check beyond the single formula evaluation above: the SAME
// componentwise law, run to a physical steady state under gravity. With
// identity orientation and motion confined to one axis, componentwise mode
// degenerates to a 1-D version of the quadratic law but WITHOUT the
// rho/Cd/A split (configs/physics.yaml's drag_coeff is one lumped kg/m
// coefficient, not rho*Cd*A) -- so its terminal velocity is
// v_t = sqrt(m*g / c), not sqrt(2*m*g / (rho*Cd*A)). A test that only ever
// checked componentwise via a single evaluation could not tell "right
// formula" from "right formula, wrong sign that happens to cancel once";
// running it to convergence under gravity is the same kind of independent
// proof the quadratic test above gives.
TEST(ForceElements, DragComponentwiseTerminalVelocityMatchesClosedForm) {
    constexpr float kMass = 0.5f;
    constexpr float kGravity = 9.81f;
    constexpr float kCoeff = 5.0f;  // kg/m, test-scaled for fast convergence (not the yaml's literal 0.028)

    const WorldParams params = MakeParams(glm::vec3(0.0f, 0.0f, -kGravity), 1.225f, glm::vec3(0.0f));
    const ConstantMedium medium;

    BodyState body = MakeUnitBody();
    body.mass = kMass;
    const DragBodyRow elem = MakeElem(0, drag_mode::componentwise, glm::vec3(kCoeff), /*area=*/0.0f);

    std::vector<BodyState> bodies{body};
    const std::vector<DragBodyRow> elems{elem};

    constexpr int kSteps = 2048;  // T = 2.0 s; tau = v_t/g ~= 0.101 s here, so ~19.8 tau
    for (int i = 0; i < kSteps; ++i) {
        apply_drag(bodies, elems, medium, params, kH);
        spade::physics::integrate_bodies(bodies, params, kH);
    }

    const double v_t = std::sqrt(static_cast<double>(kMass) * kGravity / kCoeff);  // ~0.990 m/s
    EXPECT_NEAR(static_cast<double>(bodies[0].vel.z), -v_t, v_t * 0.01) << "v_t = " << v_t;
    ExpectVec3Near(glm::vec3(bodies[0].vel.x, bodies[0].vel.y, 0.0f), glm::vec3(0.0f), 1e-5f);
}

// ---------------------------------------------------------------------------
// Componentwise mode under a GENUINELY ROTATED body (task review finding,
// 2026-08-09). Every test above this point uses MakeUnitBody()'s identity
// orientation. Since glm::conjugate(identity) == identity, a
// rotation-DIRECTION bug -- forces.cpp using `body.orient` where
// `glm::conjugate(body.orient)` belongs, or the reverse -- is INVISIBLE at
// identity orientation (both choices agree there), and
// TwoIdenticalRunsAreByteIdentical only checks that two runs of the SAME
// (possibly wrong) code agree with each other, not with an outside
// reference. Componentwise mode is the one law whose whole reason for
// existing is per-axis anisotropy expressed in the BODY frame (see its
// configs/physics.yaml citation above), so it is the one regime a
// rotation-direction bug changes the answer for -- these two tests are that
// regime, independently verified.
//
// `coeffs` below is HAND-SET, not from configs/physics.yaml: that file ships
// one scalar drag_coeff (isotropic across axes by construction), so it alone
// cannot exercise a per-axis law. DragBodyRow::coeffs being a vec3 -- not a
// float -- is exactly what makes a hand-set anisotropic vector legitimate
// test data here despite not coming from the shipped config.
//
// Both tests were confirmed to fail (and only these two, out of the full
// suite) under each of two hand-introduced mutations -- swapping
// `glm::conjugate(body.orient) * v_rel` for `body.orient * v_rel`, and
// separately swapping `body.orient * F_body` for
// `glm::conjugate(body.orient) * F_body` -- then reverted; see the fix
// report appended to task-15-report.md for the exact ctest output.
// ---------------------------------------------------------------------------

TEST(ForceElements, DragComponentwiseWithRotatedOrientMatchesIndependentClosedForm) {
    const glm::vec3 world_vel(10.0f, 4.0f, -6.0f);  // distinct, mixed-sign on every axis
    const glm::vec3 coeffs(0.02f, 0.05f, 0.008f);   // anisotropic; see header comment above

    BodyState body = MakeUnitBody();
    body.orient = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f));  // +90 deg about z
    body.vel = world_vel;

    const WorldParams params = MakeParams(glm::vec3(0.0f), 1.225f, glm::vec3(0.0f));
    const ConstantMedium medium;
    const DragBodyRow elem = MakeElem(0, drag_mode::componentwise, coeffs, /*area=*/0.0f);

    std::vector<BodyState> bodies{body};
    const std::vector<DragBodyRow> elems{elem};
    apply_drag(bodies, elems, medium, params, kH);

    // INDEPENDENT closed form: the STANDARD rotation-about-z matrix, derived
    // by hand (cos 90 deg = 0, sin 90 deg = 1) -- NOT glm::conjugate or
    // glm::quat's operator*, which is what forces.cpp itself uses, so this
    // genuinely checks the implementation instead of restating it. Same
    // posture as test_integrator.cpp's SpecificForceAtRestOnSupportIsMinus
    // GravityInBodyFrame, which hand-derives its expected frame transform
    // the same way rather than calling glm rotation helpers on the result.
    //
    //   R(90) = [[0,-1,0],[1,0,0],[0,0,1]]                world = R(90) * body
    //   R(90)^T = R(-90) = [[0,1,0],[-1,0,0],[0,0,1]]      body = R(90)^T * world
    const glm::dvec3 v_rel(world_vel);  // zero wind
    const glm::dvec3 v_rel_body(v_rel.y, -v_rel.x, v_rel.z);

    const glm::dvec3 c(coeffs);
    const glm::dvec3 F_body_expected(-c.x * std::abs(v_rel_body.x) * v_rel_body.x,
                                      -c.y * std::abs(v_rel_body.y) * v_rel_body.y,
                                      -c.z * std::abs(v_rel_body.z) * v_rel_body.z);
    const glm::dvec3 F_world_expected(-F_body_expected.y, F_body_expected.x, F_body_expected.z);

    ExpectVec3Near(bodies[0].force_acc, glm::vec3(F_world_expected), 1e-4f);

    // Guards against a vacuous pass and against the IDENTITY-orientation
    // answer sneaking through instead: at identity this same v_rel/coeffs
    // would give force_acc == (-2.0, -0.8, -0.288), nowhere near the rotated
    // result on any axis.
    EXPECT_GT(std::abs(F_world_expected.x), 1.0);
    EXPECT_GT(std::abs(F_world_expected.y), 0.1);
    ExpectVec3Near(bodies[0].torque_acc, glm::vec3(0.0f), 0.0f);  // local_pos == 0 (COM)
}

TEST(ForceElements, DragComponentwiseWithRotatedOrientAndOffsetProducesExpectedTorque) {
    const glm::vec3 world_vel(10.0f, 4.0f, -6.0f);
    const glm::vec3 coeffs(0.02f, 0.05f, 0.008f);
    const glm::vec3 r(0.02f, -0.01f, 0.05f);  // BODY-frame offset, all three axes nonzero

    BodyState body = MakeUnitBody();
    body.orient = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f));
    body.vel = world_vel;

    const WorldParams params = MakeParams(glm::vec3(0.0f), 1.225f, glm::vec3(0.0f));
    const ConstantMedium medium;
    const DragBodyRow elem = MakeElem(0, drag_mode::componentwise, coeffs, /*area=*/0.0f, r);

    std::vector<BodyState> bodies{body};
    const std::vector<DragBodyRow> elems{elem};
    apply_drag(bodies, elems, medium, params, kH);

    // Same independent derivation as the test above for the rotated,
    // anisotropic body-frame force; torque is then a plain cross product
    // against it -- no further rotation needed, since torque_acc and r are
    // both already body frame.
    const glm::dvec3 v_rel(world_vel);
    const glm::dvec3 v_rel_body(v_rel.y, -v_rel.x, v_rel.z);
    const glm::dvec3 c(coeffs);
    const glm::dvec3 F_body_expected(-c.x * std::abs(v_rel_body.x) * v_rel_body.x,
                                      -c.y * std::abs(v_rel_body.y) * v_rel_body.y,
                                      -c.z * std::abs(v_rel_body.z) * v_rel_body.z);

    const glm::dvec3 a(r);
    const glm::dvec3 torque_expected(a.y * F_body_expected.z - a.z * F_body_expected.y,
                                      a.z * F_body_expected.x - a.x * F_body_expected.z,
                                      a.x * F_body_expected.y - a.y * F_body_expected.x);

    ExpectVec3Near(bodies[0].torque_acc, glm::vec3(torque_expected), 1e-5f);
    EXPECT_GT(glm::length(bodies[0].torque_acc), 1e-3f) << "must not be vacuously near zero";
}

// ---------------------------------------------------------------------------
// Zero-wind vs with-wind asymmetry: drag responds to RELATIVE airspeed, not
// ground speed, so a tailwind must reduce it and a headwind must increase
// it relative to the still-air case -- a sign a bug that read `body.vel`
// straight through (ignoring `medium.sample(...).wind` entirely, as kat's
// own wind-less model does) could not produce by accident.
// ---------------------------------------------------------------------------

TEST(ForceElements, DragQuadraticWindAsymmetryIsSane) {
    constexpr float kRho = 1.225f;
    constexpr float kCd = 1.0f;
    constexpr float kArea = 0.1f;
    const ConstantMedium medium;

    BodyState body = MakeUnitBody();
    body.vel = glm::vec3(20.0f, 0.0f, 0.0f);
    const DragBodyRow elem = MakeElem(0, drag_mode::quadratic, glm::vec3(kCd, 0.0f, 0.0f), kArea);
    const std::vector<DragBodyRow> elems{elem};

    auto drag_force_x = [&](const glm::vec3& wind) {
        std::vector<BodyState> bodies{body};
        const WorldParams params = MakeParams(glm::vec3(0.0f), kRho, wind);
        apply_drag(bodies, elems, medium, params, kH);
        return bodies[0].force_acc.x;
    };

    const float still = drag_force_x(glm::vec3(0.0f));               // |v_rel| = 20
    const float tailwind = drag_force_x(glm::vec3(10.0f, 0.0f, 0.0f));  // |v_rel| = 10
    const float headwind = drag_force_x(glm::vec3(-10.0f, 0.0f, 0.0f)); // |v_rel| = 30

    // All three are drag opposing +x motion, so all are negative (magnitude
    // scales with v_rel^2); ordering by MAGNITUDE is the asymmetry claim.
    EXPECT_LT(still, 0.0f);
    EXPECT_LT(tailwind, 0.0f);
    EXPECT_LT(headwind, 0.0f);
    EXPECT_LT(std::abs(tailwind), std::abs(still)) << "a tailwind must reduce drag below the still-air case";
    EXPECT_LT(std::abs(still), std::abs(headwind)) << "a headwind must increase drag above the still-air case";

    // And the magnitudes match the closed form exactly (double reference),
    // not merely the right order.
    const double half_rho_cd_a = 0.5 * kRho * kCd * kArea;
    EXPECT_NEAR(static_cast<double>(still), -half_rho_cd_a * 20.0 * 20.0, 1e-3);
    EXPECT_NEAR(static_cast<double>(tailwind), -half_rho_cd_a * 10.0 * 10.0, 1e-3);
    EXPECT_NEAR(static_cast<double>(headwind), -half_rho_cd_a * 30.0 * 30.0, 1e-3);
}

// ---------------------------------------------------------------------------
// Structural contracts: disabled elements, inactive bodies, the torque's
// offset-and-frame handling, and determinism.
// ---------------------------------------------------------------------------

TEST(ForceElements, DisabledElementIsInert) {
    const WorldParams params = MakeParams(glm::vec3(0.0f), 1.225f, glm::vec3(0.0f));
    const ConstantMedium medium;

    BodyState body = MakeUnitBody();
    body.vel = glm::vec3(50.0f, -30.0f, 10.0f);  // plenty of relative airspeed if the element fired

    DragBodyRow elem = MakeElem(0, drag_mode::quadratic, glm::vec3(2.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.0f, 0.0f, 1.0f));
    elem.enabled = 0u;  // the case under test

    std::vector<BodyState> bodies{body};
    const std::vector<DragBodyRow> elems{elem};
    apply_drag(bodies, elems, medium, params, kH);

    ExpectVec3Near(bodies[0].force_acc, glm::vec3(0.0f), 0.0f);
    ExpectVec3Near(bodies[0].torque_acc, glm::vec3(0.0f), 0.0f);
}

TEST(ForceElements, InactiveBodyIsSkipped) {
    const WorldParams params = MakeParams(glm::vec3(0.0f), 1.225f, glm::vec3(0.0f));
    const ConstantMedium medium;

    BodyState active_body = MakeUnitBody();
    active_body.vel = glm::vec3(20.0f, 0.0f, 0.0f);

    BodyState inactive_body = MakeUnitBody();
    inactive_body.flags = 0u;  // the case under test
    inactive_body.vel = glm::vec3(20.0f, 0.0f, 0.0f);
    const BodyState before = inactive_body;

    std::vector<BodyState> bodies{active_body, inactive_body};
    const std::vector<DragBodyRow> elems{
        MakeElem(0, drag_mode::quadratic, glm::vec3(1.0f, 0.0f, 0.0f), 0.1f),
        MakeElem(1, drag_mode::quadratic, glm::vec3(1.0f, 0.0f, 0.0f), 0.1f),
    };
    apply_drag(bodies, elems, medium, params, kH);

    // The active neighbour did accumulate a force, so the span was really
    // traversed and the skip is not vacuous.
    EXPECT_LT(bodies[0].force_acc.x, 0.0f);

    // The inactive body is untouched byte-for-byte -- same posture as
    // test_integrator.cpp's InactiveBodiesAreLeftByteIdentical.
    EXPECT_EQ(std::memcmp(&bodies[1], &before, sizeof(BodyState)), 0);
}

TEST(ForceElements, ElementAtIdentityPoseProducesZeroTorque) {
    // "If local pose is identity/COM (the common case), torque is zero" --
    // the coordinator's resolution, checked directly: a substantial force
    // (this is the same setup as the wind-asymmetry test's still-air case)
    // must still contribute nothing to torque_acc when r == 0.
    const WorldParams params = MakeParams(glm::vec3(0.0f), 1.225f, glm::vec3(0.0f));
    const ConstantMedium medium;

    BodyState body = MakeUnitBody();
    body.vel = glm::vec3(20.0f, 0.0f, 0.0f);
    const DragBodyRow elem =
        MakeElem(0, drag_mode::quadratic, glm::vec3(1.0f, 0.0f, 0.0f), 0.1f, /*local_pos=*/glm::vec3(0.0f));

    std::vector<BodyState> bodies{body};
    const std::vector<DragBodyRow> elems{elem};
    apply_drag(bodies, elems, medium, params, kH);

    EXPECT_LT(bodies[0].force_acc.x, -1.0f) << "the force itself must be substantial, or this test proves nothing";
    ExpectVec3Near(bodies[0].torque_acc, glm::vec3(0.0f), 0.0f);
}

TEST(ForceElements, ElementAtOffsetProducesExpectedBodyFrameTorqueSign) {
    // r = +1 m along body +z (e.g. an element mounted above the COM);
    // identity orientation, so body frame == world frame here and F_body ==
    // F_world. Drag opposes the body's +x motion, so F_body lands on -x.
    //
    // torque = cross(r, F_body) = cross((0,0,1), (Fx,0,0))
    //        = (0*0 - 1*0, 1*Fx - 0*0, 0*0 - 0*Fx) = (0, Fx, 0)
    //
    // i.e. the SAME sign as Fx (negative), landing on the y axis -- a flipped
    // cross-product argument order (cross(F_body, r) instead of
    // cross(r, F_body)) would instead give (0, -Fx, 0), positive, which is
    // exactly the defect this signed check (not just a magnitude check)
    // catches.
    const WorldParams params = MakeParams(glm::vec3(0.0f), 1.225f, glm::vec3(0.0f));
    const ConstantMedium medium;

    BodyState body = MakeUnitBody();
    body.vel = glm::vec3(15.0f, 0.0f, 0.0f);
    constexpr float kCd = 0.8f;
    constexpr float kArea = 0.05f;
    constexpr float kRho = 1.225f;
    const DragBodyRow elem =
        MakeElem(0, drag_mode::quadratic, glm::vec3(kCd, 0.0f, 0.0f), kArea, /*local_pos=*/glm::vec3(0.0f, 0.0f, 1.0f));

    std::vector<BodyState> bodies{body};
    const std::vector<DragBodyRow> elems{elem};
    apply_drag(bodies, elems, medium, params, kH);

    const double Fx = -0.5 * kRho * kCd * kArea * 15.0 * 15.0;  // ~-5.5125 N
    ExpectVec3Near(bodies[0].force_acc, glm::vec3(static_cast<float>(Fx), 0.0f, 0.0f), 1e-4f);
    ExpectVec3Near(bodies[0].torque_acc, glm::vec3(0.0f, static_cast<float>(Fx), 0.0f), 1e-4f);

    EXPECT_LT(bodies[0].torque_acc.y, -1.0f) << "sign check must not be vacuously near zero";
}

TEST(ForceElements, TwoIdenticalRunsAreByteIdentical) {
    const WorldParams params = MakeParams(glm::vec3(0.0f, 0.0f, -9.81f), 1.225f, glm::vec3(1.5f, -0.5f, 0.25f));
    const ConstantMedium medium;

    const auto make_world = []() {
        std::vector<BodyState> bodies(3);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const float k = static_cast<float>(i);
            bodies[i] = MakeUnitBody();
            bodies[i].pos = glm::vec3(k, -0.5f * k, 1.0f);
            bodies[i].vel = glm::vec3(3.0f + k, -2.0f, 0.5f * k);
            bodies[i].orient = glm::normalize(glm::quat(0.7f, 0.1f * k, 0.2f, -0.05f * k));
        }
        return bodies;
    };

    const std::vector<DragBodyRow> elems{
        MakeElem(0, drag_mode::quadratic, glm::vec3(0.9f, 0.0f, 0.0f), 0.08f, glm::vec3(0.05f, 0.0f, -0.02f)),
        MakeElem(1, drag_mode::componentwise, glm::vec3(0.03f, 0.04f, 0.02f), 0.0f, glm::vec3(0.0f, 0.1f, 0.0f)),
        // A disabled element on body 2: exercises the skip path inside the
        // same determinism run.
        [] {
            DragBodyRow e = MakeElem(2, drag_mode::quadratic, glm::vec3(1.0f, 0.0f, 0.0f), 0.1f);
            e.enabled = 0u;
            return e;
        }(),
    };

    std::vector<BodyState> run_a = make_world();
    std::vector<BodyState> run_b = make_world();

    for (int i = 0; i < 100; ++i) {
        apply_drag(run_a, elems, medium, params, kH);
        apply_drag(run_b, elems, medium, params, kH);
        spade::physics::integrate_bodies(run_a, params, kH);
        spade::physics::integrate_bodies(run_b, params, kH);
    }

    ASSERT_EQ(run_a.size(), run_b.size());
    EXPECT_EQ(std::memcmp(run_a.data(), run_b.data(), run_a.size() * sizeof(BodyState)), 0);

    // Guard against the comparison passing because nothing moved.
    EXPECT_GT(glm::length(run_a[0].vel - glm::vec3(3.0f, -2.0f, 0.0f)), 1e-3f);
}
