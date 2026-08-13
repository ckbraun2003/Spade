#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/fp32_math.hpp"
#include "core/time.hpp"
#include "physics/integrator.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"
#include "state/snapshot.hpp"
#include "vehicles/rotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// RotorElement (engine design D5; spec S6). Read vehicles/rotor.hpp's sections
// 1-7 first -- the derivation lives there, and every reference value in this
// file is generated FROM that derivation rather than transcribed from a run.
//
// THE THREE LEVELS THIS FILE TESTS AT, deliberately separated:
//
//   1. THE DEFINING EQUATIONS. The inflow factor is checked against MOMENTUM
//      THEORY ITSELF -- lambda^2 + x lambda - 1 = 0 on the normal-working-state
//      branch and lambda^2 + x lambda + 1 = 0 on the windmill-brake branch --
//      not against a restatement of the closed-form root. A sign slip inside
//      the quadratic formula fails these; a transcription of the same formula
//      into the test would not.
//   2. INDEPENDENT CLOSED FORMS IN DOUBLE. The momentum reference table, the
//      Cheeseman-Bennett factor and the lag coefficient are each recomputed
//      here in double precision from the equations in the header, and the
//      fp32 implementation is held to them.
//   3. THE WHOLE PASS. Everything above is then re-observed THROUGH
//      apply_rotors() -- force_acc / T_static -- so the tests prove the pass
//      wires the physics up, not merely that the helpers exist.
//
// Plus one external fidelity tripwire (the Johnson/Castles-Gray empirical fit)
// that says where this model stands relative to measurement, clearly labelled
// as a tripwire rather than a correctness proof.
// ---------------------------------------------------------------------------

namespace {

using spade::BodyState;
using spade::ConstantMedium;
using spade::SdfProgram;
using spade::WorldParams;
using spade::vehicles::apply_rotors;
using spade::vehicles::RotorRow;
namespace body_flags = spade::physics::body_flags;

// --- scenario constants ----------------------------------------------------

// Substep exact in binary (2^-10), the same choice test_integrator.cpp and
// test_forces.cpp make: the multi-thousand-step tests below then carry only
// genuine fp32 accumulation, not a non-representable `h`.
constexpr float kH = 1.0f / 1024.0f;

constexpr float kRho = 1.225f;           // kg/m^3, ISA sea level
constexpr float kRadius = 0.15f;         // m, a 12-inch-class rotor
constexpr float kThrustCoeff = 1.0e-5f;  // N s^2  -> 10 N at 1000 rad/s
constexpr float kTorqueCoeff = 2.0e-7f;  // N m s^2 -> 0.2 N m at 1000 rad/s
constexpr float kOmega = 1000.0f;        // rad/s (about 9550 rpm)

constexpr double kPi = 3.14159265358979323846;

// --- double-precision references, derived from rotor.hpp's equations --------

// (H): v_h = sqrt(T / (2 rho A)), A = pi R^2.
[[nodiscard]] double hover_induced_velocity_ref(double thrust, double density, double radius) {
    return std::sqrt(thrust / (2.0 * density * kPi * radius * radius));
}

// The static thrust the curve gives at `omega`, in double.
[[nodiscard]] double thrust_static_ref(double omega) {
    return static_cast<double>(kThrustCoeff) * omega * omega;
}

// The piecewise inflow curve of rotor.hpp section 2, in double. Used as the
// "momentum reference table" the whole-pass test is held to.
//
// NOTE WHAT MAKES THIS A LEGITIMATE REFERENCE RATHER THAN A RESTATEMENT: the
// two momentum branches below are independently CHECKED against their own
// defining quadratics by
// InflowCurveSatisfiesTheMomentumTheoryEquationOnBothValidBranches, which
// evaluates the equation rather than the root. The band is linear by
// definition and has nothing to check.
[[nodiscard]] double inflow_factor_ref(double x) {
    if (x <= -2.0) {
        return -0.5 * x - std::sqrt(0.25 * x * x - 1.0);  // (WB)
    }
    if (x < -1.0) {
        const double lambda_onset = 0.5 + std::sqrt(1.25);  // (NWS) at x = -1, i.e. (1+sqrt(5))/2
        return 1.0 + (x + 2.0) * (lambda_onset - 1.0);      // linear band
    }
    return -0.5 * x + std::sqrt(0.25 * x * x + 1.0);  // (NWS)
}

// Cheeseman-Bennett, in double, WITHOUT the clamp -- so a test can check the
// clamp separately from the curve it clamps.
[[nodiscard]] double ground_factor_unclamped_ref(double distance, double radius) {
    const double q = radius / (4.0 * distance);
    return 1.0 / (1.0 - q * q);
}

// alpha = 1 - exp(-theta), computed cancellation-free in double via expm1 --
// a genuinely different evaluation route from the implementation's fp32
// Taylor/closed-form pair, which is the point.
[[nodiscard]] double lag_alpha_ref(double theta) { return -std::expm1(-theta); }

// The Johnson / Castles-Gray empirical fit for the descent region,
// -2 <= x <= 0. EXTERNAL to this engine: it is measurement, not this model.
[[nodiscard]] double johnson_empirical_lambda(double x) {
    return 1.0 - 1.125 * x - 1.372 * x * x - 1.718 * x * x * x - 0.655 * x * x * x * x;
}

// --- fixtures --------------------------------------------------------------

void ExpectVec3Near(const glm::vec3& actual, const glm::vec3& expected, float atol) {
    EXPECT_NEAR(actual.x, expected.x, atol);
    EXPECT_NEAR(actual.y, expected.y, atol);
    EXPECT_NEAR(actual.z, expected.z, atol);
}

// A live unit body: identity attitude, 1 kg, at rest, at the origin, no
// accumulated wrench. Value-initialized first so std430 pad bytes are
// deterministically zero (several tests below memcmp whole BodyStates).
[[nodiscard]] BodyState MakeUnitBody() {
    BodyState body{};
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    body.flags = body_flags::active;
    return body;
}

[[nodiscard]] WorldParams MakeParams(const glm::vec3& gravity, const glm::vec3& wind) {
    WorldParams params{};
    params.gravity = gravity;
    params.air_density = kRho;
    params.wind = wind;
    return params;
}

// A default-constructed RotorRow is all-zero (enabled == 0, i.e. inert), the
// same convention a zero-filled arena slot has. This factory produces the
// standard test rotor: at the body COM, thrusting along body +Y, already at
// its commanded speed (so the lag update is a no-op and the tests below see
// the curve, not the transient), and with NO lag time constant unless a test
// asks for one.
[[nodiscard]] RotorRow MakeRotor(uint32_t body_slot, float spin_dir = 1.0f,
                                 const glm::vec3& local_pos = glm::vec3(0.0f),
                                 const glm::quat& local_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
    RotorRow rotor{};
    rotor.body_slot = body_slot;
    rotor.enabled = 1u;
    rotor.tau = 0.0f;  // no lag: omega tracks omega_cmd within the step
    rotor.radius = kRadius;
    rotor.local_pos = local_pos;
    rotor.spin_dir = spin_dir;
    rotor.local_orient = local_orient;
    rotor.omega = kOmega;
    rotor.omega_cmd = kOmega;
    rotor.thrust_coeff = kThrustCoeff;
    rotor.torque_coeff = kTorqueCoeff;
    return rotor;
}

// An empty SDF program: valid (SdfProgram::validate() accepts it and reports
// depth 0) and infinitely far from everything, so f_ground == 1 and the
// inflow model can be observed on its own.
[[nodiscard]] SdfProgram EmptyWorld() {
    SdfProgram program{};
    EXPECT_TRUE(program.validate().has_value());
    return program;
}

// The viewer's `gate` assembly (engine/tools/viewer/scenes.cpp scene_gate),
// MINUS its ground plane: a torus ring plus two box posts, and nothing else.
// Dropping the plane is what makes "over open space" genuinely open, so the
// ground-effect source test has something to contrast against.
//
// Post geometry, which the test's expected distances depend on: half extents
// (0.15, 1.5, 0.15) centred at (-1.8, 1.5, 0) and (1.8, 1.5, 0), so each post's
// top face is the square y = 3.0, x in [+-1.8 +- 0.15], z in [-0.15, 0.15].
[[nodiscard]] SdfProgram GateWorldSdf() {
    spade::SdfPose ring_pose;
    ring_pose.position = glm::vec3(0.0f, 1.8f, 0.0f);
    ring_pose.rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    const glm::vec3 post_half(0.15f, 1.5f, 0.15f);
    spade::SdfPose left_post;
    left_post.position = glm::vec3(-1.8f, 1.5f, 0.0f);
    spade::SdfPose right_post;
    right_post.position = glm::vec3(1.8f, 1.5f, 0.0f);

    spade::WorldBuilder wb;
    wb.name("gate_world")
        .environment(spade::Environment{})
        .capacities(spade::Capacities{1, 1, 1, 1})
        .torus(1.5f, 0.15f, ring_pose)
        .box(post_half, left_post)
        .union_()
        .box(post_half, right_post)
        .union_();

    const auto desc = wb.build();
    EXPECT_TRUE(desc.has_value());
    if (!desc) return SdfProgram{};
    return desc->sdf;
}

// Runs one substep of the pass on a single body/rotor pair and returns the
// body's accumulated wrench. The workhorse of the whole-pass tests.
struct Wrench {
    glm::vec3 force{0.0f};
    glm::vec3 torque{0.0f};
};

[[nodiscard]] Wrench ApplyOnce(BodyState body, RotorRow rotor, const SdfProgram& world, const WorldParams& params,
                               float h = kH) {
    const ConstantMedium medium;
    std::vector<BodyState> bodies{body};
    std::vector<RotorRow> rotors{rotor};
    apply_rotors(bodies, rotors, world, medium, params, h);
    return Wrench{bodies[0].force_acc, bodies[0].torque_acc};
}

}  // namespace

// ===========================================================================
// Layout and registered-state contracts.
//
// RotorRow carries genuine dynamic state (omega), so "is it in the registry"
// is not a formality here: a snapshot that missed it would restore a vehicle
// whose rotors are at the wrong speed. The last test in this section proves
// the round trip end to end rather than only that the array is registered.
// ===========================================================================

TEST(RotorElement, RotorRowLayoutSizesAreVisibleAtRuntime) {
    EXPECT_EQ(sizeof(RotorRow), 80u);
    EXPECT_EQ(alignof(RotorRow), 16u);
    EXPECT_EQ(offsetof(RotorRow, body_slot), 0u);
    EXPECT_EQ(offsetof(RotorRow, enabled), 4u);
    EXPECT_EQ(offsetof(RotorRow, tau), 8u);
    EXPECT_EQ(offsetof(RotorRow, radius), 12u);
    EXPECT_EQ(offsetof(RotorRow, local_pos), 16u);
    EXPECT_EQ(offsetof(RotorRow, spin_dir), 28u);
    EXPECT_EQ(offsetof(RotorRow, local_orient), 32u);
    EXPECT_EQ(offsetof(RotorRow, omega), 48u);
    EXPECT_EQ(offsetof(RotorRow, omega_cmd), 52u);
    EXPECT_EQ(offsetof(RotorRow, thrust_coeff), 56u);
    EXPECT_EQ(offsetof(RotorRow, torque_coeff), 60u);
    EXPECT_EQ(offsetof(RotorRow, _r0), 64u);
}

TEST(RotorElement, RotorRowIsRegisteredStateAndZeroFillsInert) {
    spade::ArenaSet arenas(1);
    const auto rotors_id = arenas.register_array<RotorRow>("rotors", 4);
    ASSERT_TRUE(rotors_id.has_value());

    const spade::RegisteredArray* entry = arenas.registry().find("rotors");
    ASSERT_NE(entry, nullptr) << "an unregistered rotor array is STATE a snapshot would miss";
    EXPECT_EQ(entry->elem_size, sizeof(RotorRow));
    EXPECT_EQ(entry->capacity_per_world, 4u);

    const auto slot = arenas.alloc_slot(*rotors_id, 0);
    ASSERT_TRUE(slot.has_value());
    const auto span = arenas.array(*rotors_id);
    ASSERT_TRUE(span.has_value());

    // Zero-filled on registration: a fresh slot is inert, its shaft stopped.
    EXPECT_EQ((*span)[*slot].enabled, 0u);
    EXPECT_EQ((*span)[*slot].omega, 0.0f);
    EXPECT_EQ((*span)[*slot].omega_cmd, 0.0f);
}

TEST(RotorElement, OmegaIsWrittenBackIntoTheArenaRow) {
    // The pass must mutate the ARENA's bytes, not a copy: that is what makes
    // the shaft speed snapshot-covered rather than merely typed.
    spade::ArenaSet arenas(1);
    const auto bodies_id = arenas.register_array<BodyState>("bodies", 1);
    const auto rotors_id = arenas.register_array<RotorRow>("rotors", 1);
    ASSERT_TRUE(bodies_id.has_value());
    ASSERT_TRUE(rotors_id.has_value());
    ASSERT_TRUE(arenas.alloc_slot(*bodies_id, 0).has_value());
    ASSERT_TRUE(arenas.alloc_slot(*rotors_id, 0).has_value());

    auto bodies = arenas.world_slice(*bodies_id, 0);
    auto rotors = arenas.world_slice(*rotors_id, 0);
    ASSERT_TRUE(bodies.has_value());
    ASSERT_TRUE(rotors.has_value());

    (*bodies)[0] = MakeUnitBody();
    (*rotors)[0] = MakeRotor(0);
    (*rotors)[0].tau = 0.0625f;
    (*rotors)[0].omega = 0.0f;
    (*rotors)[0].omega_cmd = kOmega;

    const ConstantMedium medium;
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));
    apply_rotors(*bodies, *rotors, world, medium, params, kH);

    // Re-read through a FRESH span over the same arena, so this cannot pass by
    // having mutated a local copy.
    const auto reread = arenas.array(*rotors_id);
    ASSERT_TRUE(reread.has_value());
    EXPECT_GT((*reread)[0].omega, 0.0f);
    EXPECT_LT((*reread)[0].omega, kOmega);
}

TEST(RotorElement, ShaftSpeedSurvivesASnapshotRoundTripBitExactly) {
    // The brief's requirement, proven end to end: snapshot mid-spin-up, run
    // on, restore, run the same steps again, and demand BYTE equality of both
    // the bodies and the rotor rows. A shaft speed living outside the registry
    // would make the two runs diverge.
    spade::ArenaSet arenas(1);
    const auto bodies_result = arenas.register_array<BodyState>("bodies", 1);
    const auto rotors_result = arenas.register_array<RotorRow>("rotors", 1);
    ASSERT_TRUE(bodies_result.has_value());
    ASSERT_TRUE(rotors_result.has_value());
    const spade::ArrayId<BodyState> bodies_id = *bodies_result;
    const spade::ArrayId<RotorRow> rotors_id = *rotors_result;
    ASSERT_TRUE(arenas.alloc_slot(bodies_id, 0).has_value());
    ASSERT_TRUE(arenas.alloc_slot(rotors_id, 0).has_value());
    {
        auto bodies = arenas.world_slice(bodies_id, 0);
        auto rotors = arenas.world_slice(rotors_id, 0);
        ASSERT_TRUE(bodies.has_value());
        ASSERT_TRUE(rotors.has_value());
        (*bodies)[0] = MakeUnitBody();
        (*rotors)[0] = MakeRotor(0);
        (*rotors)[0].tau = 0.0625f;
        (*rotors)[0].omega = 0.0f;
        (*rotors)[0].omega_cmd = kOmega;
    }

    const ConstantMedium medium;
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f, -9.80665f, 0.0f), glm::vec3(0.3f, -0.2f, 0.1f));

    const auto step = [&](int n) {
        for (int i = 0; i < n; ++i) {
            auto bodies = arenas.world_slice(bodies_id, 0);
            auto rotors = arenas.world_slice(rotors_id, 0);
            apply_rotors(*bodies, *rotors, world, medium, params, kH);
            spade::physics::integrate_bodies(*bodies, params, kH);
        }
    };

    step(20);  // mid spin-up: omega is strictly between 0 and the command
    const auto mid = arenas.array(rotors_id);
    ASSERT_TRUE(mid.has_value());
    ASSERT_GT((*mid)[0].omega, 1.0f);
    ASSERT_LT((*mid)[0].omega, kOmega - 1.0f) << "the snapshot must be taken DURING the transient";

    const auto blob = spade::save(arenas, spade::Tick{20});
    ASSERT_TRUE(blob.has_value());

    step(40);
    const RotorRow rotor_a = (*arenas.array(rotors_id))[0];
    const BodyState body_a = (*arenas.array(bodies_id))[0];

    ASSERT_TRUE(spade::restore(arenas, *blob).has_value());
    step(40);
    const RotorRow rotor_b = (*arenas.array(rotors_id))[0];
    const BodyState body_b = (*arenas.array(bodies_id))[0];

    EXPECT_EQ(std::memcmp(&rotor_a, &rotor_b, sizeof(RotorRow)), 0);
    EXPECT_EQ(std::memcmp(&body_a, &body_b, sizeof(BodyState)), 0);
}

// ===========================================================================
// The inflow curve, checked against momentum theory's own equations.
// ===========================================================================

TEST(RotorElement, InflowCurveSatisfiesTheMomentumTheoryEquationOnBothValidBranches) {
    // The strongest available check, and the reason this file does not simply
    // restate the quadratic formula: substitute the returned lambda back into
    // the equation it is supposed to solve.
    //
    //   normal working state   T = 2 rho A (V_c + v_i) v_i   =>  (x + L) L = 1
    //   windmill brake state   the streamtube reverses       =>  (x + L) L = -1
    //
    // A sign slip inside the root, or the WRONG root of the same quadratic,
    // fails one of these (the wrong root is negative, which the positivity
    // check below catches).
    constexpr float kVh = 4.0f;

    for (int i = 0; i <= 90; ++i) {
        const double x = -1.0 + 0.1 * static_cast<double>(i);  // -1 .. 8, the (NWS) branch
        const float lambda = spade::vehicles::rotor_inflow_factor(static_cast<float>(x) * kVh, kVh);
        const double residual = (x + lambda) * lambda - 1.0;
        EXPECT_NEAR(residual, 0.0, 2e-5) << "normal working state, x = " << x;
        EXPECT_GT(lambda, 0.0f) << "the physical root is the positive one, x = " << x;
    }

    for (int i = 0; i <= 100; ++i) {
        const double x = -2.0 - 0.1 * static_cast<double>(i);  // -2 .. -12, the (WB) branch
        const float lambda = spade::vehicles::rotor_inflow_factor(static_cast<float>(x) * kVh, kVh);
        const double residual = (x + lambda) * lambda + 1.0;
        EXPECT_NEAR(residual, 0.0, 2e-5) << "windmill brake state, x = " << x;
        EXPECT_GT(lambda, 0.0f) << "the physical root is the positive one, x = " << x;
    }

    // FAR FIELD, where the TEXTBOOK form of each root -- a difference of two
    // nearly equal positives -- silently collapses to zero in fp32 (rotor.hpp
    // section 2, "Conditioning"). The same residual is the sharpest possible
    // detector of that: a lambda wrongly evaluated as 0 at x = 1e4 leaves a
    // residual of -1, not of 1e-8. These cases fail on the naive spelling and
    // pass on the conjugate one, which is the whole reason the conjugate form
    // is there.
    for (const double x : {1.0e2, 1.0e3, 1.0e4, 1.0e6, 1.0e8}) {
        const float climbing = spade::vehicles::rotor_inflow_factor(static_cast<float>(x) * kVh, kVh);
        EXPECT_NEAR((x + climbing) * climbing - 1.0, 0.0, 1e-5) << "far-field climb, x = " << x;
        EXPECT_GT(climbing, 0.0f) << "far-field climb, x = " << x;

        const float descending = spade::vehicles::rotor_inflow_factor(static_cast<float>(-x) * kVh, kVh);
        EXPECT_NEAR((-x + descending) * descending + 1.0, 0.0, 1e-5) << "far-field descent, x = " << x;
        EXPECT_GT(descending, 0.0f) << "far-field descent, x = " << x;
    }
}

TEST(RotorElement, InflowCurveIsHoverNeutralPeaksAtTheGoldenRatioAndIsContinuous) {
    constexpr float kVh = 4.0f;
    const auto lambda = [](double x) {
        return spade::vehicles::rotor_inflow_factor(static_cast<float>(x) * kVh, kVh);
    };

    // Hover is EXACTLY neutral: the thrust curve is used as the static curve
    // it was measured as, with no silent rescaling.
    EXPECT_FLOAT_EQ(lambda(0.0), 1.0f);

    // The curve's maximum is the vortex-ring onset value (1+sqrt(5))/2, and
    // the exported constant must agree with the implementation.
    EXPECT_NEAR(lambda(-1.0), spade::vehicles::kRotorInflowFactorMax, 1e-6f);
    EXPECT_NEAR(lambda(-1.0), 0.5 + std::sqrt(1.25), 1e-6);

    // Continuity across both branch boundaries. The band exists precisely so
    // that (NWS)'s unbounded continuation does not meet (WB) with a jump.
    //
    // At x = -1 the join is an ordinary kink -- both one-sided slopes are
    // finite -- so a symmetric epsilon straddles it linearly.
    EXPECT_NEAR(lambda(-1.0 + 1e-4), lambda(-1.0 - 1e-4), 1e-3f);

    // At x = -2 it is NOT: (WB) leaves its boundary with an INFINITE slope
    // (its square root behaves like sqrt(-(x+2))), so the approach from below
    // is 1 - sqrt(eps) while the approach from above is 1 + 0.618 eps. Both
    // one-sided limits are 1 -- the curve IS continuous -- but only the
    // square-root rate makes that a checkable claim, so it is checked as one.
    // See rotor.hpp section 2: this is momentum theory's own kink, not the
    // band's.
    EXPECT_NEAR(lambda(-2.0), 1.0f, 1e-6f) << "(WB) must reach exactly 1 where its root vanishes";
    for (const double eps : {1e-2, 1e-3, 1e-4}) {
        EXPECT_NEAR(lambda(-2.0 + eps), 1.0, 1.2 * eps) << "band side, eps = " << eps;
        EXPECT_NEAR(lambda(-2.0 - eps), 1.0, 1.5 * std::sqrt(eps)) << "(WB) side, eps = " << eps;
        EXPECT_LT(lambda(-2.0 - eps), 1.0f) << "(WB) drops below 1 immediately, eps = " << eps;
    }

    // Bounded everywhere, including far outside the modelled envelope.
    for (int i = -400; i <= 400; ++i) {
        const double x = 0.05 * static_cast<double>(i);
        const float f = lambda(x);
        EXPECT_GE(f, 0.0f) << "x = " << x;
        EXPECT_LE(f, spade::vehicles::kRotorInflowFactorMax + 1e-6f) << "x = " << x;
    }

    // Monotone decreasing on [-1, inf): more climb is always less thrust.
    float previous = lambda(-1.0);
    for (int i = 1; i <= 200; ++i) {
        const float f = lambda(-1.0 + 0.05 * static_cast<double>(i));
        EXPECT_LE(f, previous + 1e-6f) << "step " << i;
        previous = f;
    }
}

TEST(RotorElement, InflowCurveBandIsLinearBetweenTheTwoMomentumSolutions) {
    constexpr float kVh = 4.0f;
    const auto lambda = [](double x) {
        return spade::vehicles::rotor_inflow_factor(static_cast<float>(x) * kVh, kVh);
    };

    // A straight line in x from lambda = 1 at x = -2 to (NWS)'s value at
    // x = -1: the second difference must vanish across the whole band. This is
    // the "clamped linear approximation" claim, checked as linearity rather
    // than as three sampled points.
    for (int i = 1; i < 20; ++i) {
        const double x = -2.0 + 0.05 * static_cast<double>(i);
        const double expect = 1.0 + (x + 2.0) * (0.5 + std::sqrt(1.25) - 1.0);
        EXPECT_NEAR(static_cast<double>(lambda(x)), expect, 1e-5) << "band, x = " << x;
    }

    // And it is genuinely a CLAMP: continuing (NWS) instead would reach
    // 1 + sqrt(2) = 2.414 at x = -2, which the band's endpoint (1.0) is
    // nowhere near.
    EXPECT_LT(lambda(-2.0), 1.5f);
}

TEST(RotorElement, InflowCurveTracksTheJohnsonEmpiricalFitWithinDocumentedBounds) {
    // A FIDELITY TRIPWIRE, not a correctness proof: it compares this model to
    // MEASUREMENT (the Castles & Gray data as fitted by Johnson), and its job
    // is to fail loudly if the curve ever silently moves relative to the
    // numbers rotor.hpp's accuracy table publishes.
    //
    // The reference is first self-validated on a property of the real curve
    // that is independent of the transcription: the empirical induced velocity
    // returns to the hover value at BOTH momentum-valid boundaries.
    EXPECT_NEAR(johnson_empirical_lambda(0.0), 1.0, 1e-12);
    EXPECT_NEAR(johnson_empirical_lambda(-2.0), 1.0, 0.03);

    constexpr float kVh = 4.0f;
    const auto lambda = [](double x) {
        return static_cast<double>(spade::vehicles::rotor_inflow_factor(static_cast<float>(x) * kVh, kVh));
    };

    // Wherever momentum theory is actually used (-1 <= x <= 0) this model is
    // within 12% of measurement.
    for (int i = 0; i <= 20; ++i) {
        const double x = -0.05 * static_cast<double>(i);
        const double reference = johnson_empirical_lambda(x);
        EXPECT_LE(std::abs(lambda(x) - reference) / reference, 0.12) << "x = " << x;
    }

    // Inside the band the linear approximation is up to 37% LOW, and that is
    // the documented, deliberate direction -- so the bound is one-sided on top
    // of being bounded in magnitude.
    for (int i = 0; i <= 20; ++i) {
        const double x = -1.0 - 0.05 * static_cast<double>(i);
        const double reference = johnson_empirical_lambda(x);
        EXPECT_LE(std::abs(lambda(x) - reference) / reference, 0.40) << "x = " << x;
    }
    for (int i = 0; i <= 40; ++i) {
        const double x = -0.05 * static_cast<double>(i);
        EXPECT_LE(lambda(x), johnson_empirical_lambda(x) + 1e-6)
            << "this model must never OVER-predict descent thrust relative to measurement, x = " << x;
    }
}

TEST(RotorElement, InflowFactorIsTotalOnDegenerateInputs) {
    using spade::vehicles::rotor_inflow_factor;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    // No disc loading -> the neutral factor, never a division by zero.
    EXPECT_FLOAT_EQ(rotor_inflow_factor(5.0f, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(rotor_inflow_factor(-5.0f, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(rotor_inflow_factor(5.0f, -1.0f), 1.0f);
    EXPECT_FLOAT_EQ(rotor_inflow_factor(0.0f, nan), 1.0f);

    // A NaN already in the state stays neutral rather than being propagated.
    EXPECT_FLOAT_EQ(rotor_inflow_factor(nan, 4.0f), 1.0f);

    // A HUGE ratio is a real limit, and lambda tends to 0 at both ends of the
    // curve -- so 0, not 1. This is also the arithmetic guard: both branches
    // form 0.25*x*x, which OVERFLOWS fp32 above |x| ~ 3.7e19, and the climb
    // branch would then evaluate -inf + inf = NaN. Every case below reaches
    // that regime by a different route -- an infinite ratio, an overflowing
    // ratio, and a merely enormous but perfectly finite one.
    EXPECT_FLOAT_EQ(rotor_inflow_factor(inf, 4.0f), 0.0f);
    EXPECT_FLOAT_EQ(rotor_inflow_factor(-inf, 4.0f), 0.0f);
    EXPECT_FLOAT_EQ(rotor_inflow_factor(1e30f, 1e-20f), 0.0f) << "the ratio itself overflows here";
    EXPECT_FLOAT_EQ(rotor_inflow_factor(-1e30f, 1e-20f), 0.0f);
    EXPECT_FLOAT_EQ(rotor_inflow_factor(1e30f, 1e-3f), 0.0f) << "x = 1e33: finite, but x*x is not";
    EXPECT_FLOAT_EQ(rotor_inflow_factor(-1e30f, 1e-3f), 0.0f);

    // Just inside the guard the curve is still evaluated, and is still both
    // finite and asymptotically 1/|x|.
    const float x_near = 0.5f * spade::vehicles::kRotorMaxInflowRatio;
    EXPECT_TRUE(std::isfinite(rotor_inflow_factor(x_near, 1.0f)));
    EXPECT_NEAR(static_cast<double>(rotor_inflow_factor(x_near, 1.0f)), 1.0 / x_near, 1e-12);
    EXPECT_TRUE(std::isfinite(rotor_inflow_factor(-x_near, 1.0f)));
}

// ===========================================================================
// The whole pass: thrust against the momentum reference table.
// ===========================================================================

TEST(RotorElement, StaticThrustAtHoverMatchesTheCurve) {
    // The brief's first requirement. At rest, in still air, far from any
    // geometry: both corrections are exactly 1, so the pass must hand back the
    // curve itself.
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));

    const Wrench w = ApplyOnce(MakeUnitBody(), MakeRotor(0), world, params);

    // 10 N, up to kThrustCoeff's own fp32 representation (1e-5f is
    // 9.99999975e-6, so the product is 9.9999997 rather than a round 10).
    const double t_static = thrust_static_ref(kOmega);
    ASSERT_NEAR(t_static, 10.0, 1e-5);
    ExpectVec3Near(w.force, glm::vec3(0.0f, static_cast<float>(t_static), 0.0f), 1e-4f);

    // The shaft reaction opposes the rotor's own rotation: spin_dir = +1 about
    // body +Y gives a -Y couple on the airframe.
    const double q = static_cast<double>(kTorqueCoeff) * kOmega * kOmega;  // 0.2 N m
    ExpectVec3Near(w.torque, glm::vec3(0.0f, static_cast<float>(-q), 0.0f), 1e-5f);
    EXPECT_LT(w.torque.y, -0.1f) << "sign check must not be vacuously near zero";
}

TEST(RotorElement, ClimbReducesAndDescentIncreasesThrustPerTheMomentumReferenceTable) {
    // The brief's second requirement, observed THROUGH the pass: for each
    // axial speed, force_acc / T_static must equal the momentum reference
    // table's lambda(x) to 2%.
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));

    const double t_static = thrust_static_ref(kOmega);
    const double v_h = hover_induced_velocity_ref(t_static, kRho, kRadius);
    ASSERT_NEAR(v_h, 7.5989, 1e-3) << "the scenario's hover induced velocity, for the record";

    // The engine's own (H) must agree with the double reference first -- a
    // wrong v_h would move every x below and hide a curve error.
    EXPECT_NEAR(static_cast<double>(spade::vehicles::rotor_hover_induced_velocity(
                    static_cast<float>(t_static), kRho, kRadius)),
                v_h, v_h * 1e-5);

    // x = V_c / v_h, spanning fast climb, hover, the whole vortex-ring band,
    // and deep windmill-brake descent.
    const double xs[] = {4.0, 2.0, 1.0, 0.5, 0.25, 0.0, -0.25, -0.5, -0.75, -1.0, -1.25, -1.5, -1.75, -2.0, -3.0, -6.0};

    for (const double x : xs) {
        BodyState body = MakeUnitBody();
        body.vel = glm::vec3(0.0f, static_cast<float>(x * v_h), 0.0f);  // axial: the thrust axis is body/world +Y
        const Wrench w = ApplyOnce(body, MakeRotor(0), world, params);

        const double measured = static_cast<double>(w.force.y) / t_static;
        const double expected = inflow_factor_ref(x);
        EXPECT_NEAR(measured, expected, expected * 0.02) << "x = " << x;

        // No lateral thrust anywhere on the sweep.
        EXPECT_NEAR(w.force.x, 0.0f, 1e-5f);
        EXPECT_NEAR(w.force.z, 0.0f, 1e-5f);
    }

    // The qualitative claim the table encodes, asserted separately so a table
    // that silently became flat would still fail:
    const auto factor_at = [&](double x) {
        BodyState body = MakeUnitBody();
        body.vel = glm::vec3(0.0f, static_cast<float>(x * v_h), 0.0f);
        return static_cast<double>(ApplyOnce(body, MakeRotor(0), world, params).force.y) / t_static;
    };
    EXPECT_LT(factor_at(1.0), 0.75) << "climb must REDUCE thrust, substantially";
    EXPECT_LT(factor_at(0.25), 1.0);
    EXPECT_GT(factor_at(-0.25), 1.0) << "slow descent must INCREASE thrust";
    EXPECT_GT(factor_at(-1.0), 1.5) << "descent at the vortex-ring onset is the peak";
    // ... and in the deep windmill-brake state it falls away again, which is
    // the physically right answer for a rotor being driven by the flow (its
    // bluff-body drag is DragBody's job, not the rotor's).
    EXPECT_LT(factor_at(-6.0), 0.5);
}

TEST(RotorElement, AxialInflowUsesAirRelativeVelocityNotGroundSpeed) {
    // A stationary rotor in an updraft must behave exactly like a descending
    // rotor in still air -- the two are the same flow. A bug that read
    // `body.vel` straight through (ignoring the Medium, as a wind-less model
    // would) could not produce this.
    const SdfProgram world = EmptyWorld();
    const double t_static = thrust_static_ref(kOmega);
    const double v_h = hover_induced_velocity_ref(t_static, kRho, kRadius);

    const float half_vh = static_cast<float>(0.5 * v_h);

    // Updraft (+Y wind) == descent: more thrust.
    const Wrench up = ApplyOnce(MakeUnitBody(), MakeRotor(0), world,
                                MakeParams(glm::vec3(0.0f), glm::vec3(0.0f, half_vh, 0.0f)));
    // Downdraft == climb: less thrust.
    const Wrench down = ApplyOnce(MakeUnitBody(), MakeRotor(0), world,
                                  MakeParams(glm::vec3(0.0f), glm::vec3(0.0f, -half_vh, 0.0f)));
    // Still air: the curve's neutral point.
    const Wrench still = ApplyOnce(MakeUnitBody(), MakeRotor(0), world,
                                   MakeParams(glm::vec3(0.0f), glm::vec3(0.0f)));

    EXPECT_GT(up.force.y, still.force.y);
    EXPECT_LT(down.force.y, still.force.y);
    EXPECT_NEAR(static_cast<double>(up.force.y) / t_static, inflow_factor_ref(-0.5), 0.02 * 1.29);
    EXPECT_NEAR(static_cast<double>(down.force.y) / t_static, inflow_factor_ref(0.5), 0.02 * 0.79);

    // A pure crosswind is not axial inflow at all: it must not change thrust.
    const Wrench cross = ApplyOnce(MakeUnitBody(), MakeRotor(0), world,
                                   MakeParams(glm::vec3(0.0f), glm::vec3(20.0f, 0.0f, -15.0f)));
    EXPECT_NEAR(cross.force.y, still.force.y, 1e-4f);
}

TEST(RotorElement, AxialInflowIncludesTheLeverArmTermForAnOffsetRotor) {
    // A rolling airframe drives an offset rotor up and down through the air
    // even when its COM is still: v_station = v + R (omega_body x r). Here
    // omega_body = (0,0,W) and r = (L,0,0) give omega_body x r = (0, W L, 0),
    // i.e. a pure climb of W*L at the rotor -- chosen equal to v_h, so the
    // expected factor is exactly lambda(1) = (sqrt(5)-1)/2.
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));

    const double t_static = thrust_static_ref(kOmega);
    const double v_h = hover_induced_velocity_ref(t_static, kRho, kRadius);
    constexpr float kArm = 0.2f;

    BodyState body = MakeUnitBody();
    body.omega_body = glm::vec3(0.0f, 0.0f, static_cast<float>(v_h) / kArm);

    const Wrench w = ApplyOnce(body, MakeRotor(0, 1.0f, glm::vec3(kArm, 0.0f, 0.0f)), world, params);

    EXPECT_NEAR(static_cast<double>(w.force.y) / t_static, inflow_factor_ref(1.0), 0.02);

    // Guard against a vacuous pass: without the lever-arm term the factor
    // would be 1.0, not 0.618.
    EXPECT_LT(w.force.y, 0.8 * t_static);
}

// ===========================================================================
// The RPM lag.
// ===========================================================================

TEST(RotorElement, LagAlphaMatchesTheExactCoefficientAcrossTheSeriesThreshold) {
    // alpha = 1 - exp(-h/tau), checked against a cancellation-free double
    // reference (-expm1) on BOTH sides of the Taylor/closed-form switch, and
    // at step ratios far below anything a trajectory test could resolve.
    for (int i = -70; i <= 16; ++i) {
        const double theta = std::pow(10.0, 0.25 * static_cast<double>(i));  // 1e-17.5 .. 1e4
        if (theta > 40.0) break;
        const float alpha = spade::vehicles::rotor_lag_alpha(static_cast<float>(theta), 1.0f);
        const double reference = lag_alpha_ref(theta);
        EXPECT_NEAR(static_cast<double>(alpha), reference, std::max(reference * 2e-6, 1e-12))
            << "theta = " << theta;
        EXPECT_GE(alpha, 0.0f);
        EXPECT_LE(alpha, 1.0f) << "alpha must stay a contraction at every step ratio";
    }

    // The switch itself must not be visible: the two branches agree at the
    // threshold to fp32 precision.
    const float below = spade::vehicles::rotor_lag_alpha(
        std::nextafter(spade::vehicles::kRotorLagSeriesThreshold, 0.0f), 1.0f);
    const float above = spade::vehicles::rotor_lag_alpha(spade::vehicles::kRotorLagSeriesThreshold, 1.0f);
    EXPECT_NEAR(below, above, 2e-7f);

    // WHERE the switch sits is a margin choice, not a correctness one, and
    // rotor.cpp's note on one_minus_exp_neg says so in as many words -- this is
    // that claim CHECKED rather than asserted (task review round 1, 2026-08-09).
    //
    // This file switches at 0.25; world/medium.cpp's IDENTICAL series switches
    // at 0.125. Both lie below the crossover at which the truncated series
    // stops being the more accurate branch, so at BOTH points BOTH branches
    // land within a whisker of the double reference -- which is exactly what
    // "either threshold would have been correct" means. If a future change
    // pushed either threshold past the crossover, the closed-form column below
    // is the one that would start to win, and this is where that shows up
    // instead of in a golden digest.
    //
    // THE CROSSOVER WAS RE-DERIVED AT S5 TASK 1, when the closed-form branch
    // stopped calling std::exp and started calling fp32_math's exp32, and then
    // CORRECTED in that task's review round 1. The two steps point opposite
    // ways, so both are recorded:
    //
    //   * THE ANALYTIC MODEL (rotor.cpp) compares the series' truncation
    //     x^7/5040 against the exponential's absolute error E, giving
    //     x* = (5040 E)^(1/7). With E over [0.5, 1.5] ulp that is a band
    //     [0.2843, 0.3326] rather than the single 0.314 the std::exp-era
    //     comment quoted.
    //   * THAT MODEL IS OPTIMISTIC ABOUT THE SERIES. It omits the series' own
    //     Horner rounding -- six multiply-adds' worth -- while charging the
    //     closed form its full error. Measuring BOTH branches as the engine
    //     spells them against a double -expm1 reference, the real crossover is
    //     x = 0.260 on the mean and 0.275 on the worst case: BELOW the model
    //     band's lower edge.
    //
    // THE GUARD THEREFORE ANCHORS ON THE MEASUREMENT, 0.260, not on the model.
    // Anchoring at 0.2843 would have green-lit a future threshold of, say,
    // 0.28 -- where the closed form is in fact the better branch, i.e. where
    // this assertion's own stated meaning would be false.
    //
    // 0.25 < 0.260 still holds, so no constant moves. But the margin is 4%,
    // not the 14% the model implied, and rotor.cpp's tabulation puts the
    // advantage at 0.25 at 1.37x rather than 4x. This threshold is nearer
    // optimal than anyone intended, which is worth knowing before it is nudged.
    EXPECT_LT(spade::vehicles::kRotorLagSeriesThreshold, 0.260f)
        << "the series must still be the MEASURED better branch where this file hands "
           "over -- not merely the better branch under the truncation-only model";
    for (const double x : {0.125, 0.25}) {
        const double reference = lag_alpha_ref(x);
        const double from_impl = spade::vehicles::rotor_lag_alpha(static_cast<float>(x), 1.0f);
        // The closed-form branch AS THIS ENGINE SPELLS IT -- exp32, not a
        // libm. Comparing against std::exp here would check a branch the
        // engine no longer has.
        const double from_closed_form = 1.0f - spade::math::exp32(-static_cast<float>(x));
        EXPECT_LE(std::abs(from_impl - reference) / reference, 2e-6) << "series/impl at x = " << x;
        EXPECT_LE(std::abs(from_closed_form - reference) / reference, 2e-6) << "closed form at x = " << x;
    }
}

TEST(RotorElement, LagAlphaIsTotalOnDegenerateInputs) {
    using spade::vehicles::rotor_lag_alpha;
    const float nan = std::numeric_limits<float>::quiet_NaN();

    EXPECT_FLOAT_EQ(rotor_lag_alpha(0.0f, 0.05f), 0.0f);   // no time passes
    EXPECT_FLOAT_EQ(rotor_lag_alpha(-1.0f, 0.05f), 0.0f);
    EXPECT_FLOAT_EQ(rotor_lag_alpha(nan, 0.05f), 0.0f);
    EXPECT_FLOAT_EQ(rotor_lag_alpha(kH, 0.0f), 1.0f);      // no lag: instant tracking
    EXPECT_FLOAT_EQ(rotor_lag_alpha(kH, -1.0f), 1.0f);
    EXPECT_FLOAT_EQ(rotor_lag_alpha(kH, nan), 1.0f);
}

TEST(RotorElement, MeasuredRpmTimeConstantMatchesTau) {
    // The brief's third requirement. tau and h are both exact binary fractions
    // so that exactly 64 substeps land on t = tau; the measurement is then the
    // model's, not the arithmetic's.
    constexpr float kTau = 1.0f / 16.0f;  // 62.5 ms
    static_assert(kTau / kH == 64.0f, "the sweep below must land exactly on t = tau");

    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));
    const ConstantMedium medium;

    std::vector<BodyState> bodies{MakeUnitBody()};
    std::vector<RotorRow> rotors{MakeRotor(0)};
    rotors[0].tau = kTau;
    rotors[0].omega = 0.0f;
    rotors[0].omega_cmd = kOmega;

    for (int i = 0; i < 64; ++i) {
        apply_rotors(bodies, rotors, world, medium, params, kH);
    }

    // A first-order lag reaches 1 - 1/e of its command in exactly one tau.
    const double fraction = static_cast<double>(rotors[0].omega) / kOmega;
    const double expected = 1.0 - std::exp(-1.0);  // 0.632120558...
    EXPECT_NEAR(fraction, expected, 0.05 * expected);

    // Stated the other way round -- the quantity the brief names. Inverting
    // the exponential turns the 5% band on omega into a 5% band on tau, which
    // is the stronger reading of "measured time constant".
    const double tau_measured = -static_cast<double>(kTau) / std::log(1.0 - fraction);
    EXPECT_NEAR(tau_measured, static_cast<double>(kTau), 0.05 * kTau) << "measured tau = " << tau_measured;

    // Continuing to 5 tau must settle on the command.
    for (int i = 0; i < 4 * 64; ++i) {
        apply_rotors(bodies, rotors, world, medium, params, kH);
    }
    EXPECT_NEAR(static_cast<double>(rotors[0].omega) / kOmega, 1.0 - std::exp(-5.0), 0.01);
}

TEST(RotorElement, RpmLagIsTheExactSolutionNotAnEulerStep) {
    // At a COARSE step ratio the exact coefficient and the spec's first-order
    // one visibly disagree, which is what makes this test able to tell them
    // apart at all (at 1 kHz with a 20 ms rotor they agree to 2.5%).
    //
    //   theta = 0.5   exact 1 - e^-0.5 = 0.39347   Euler 0.5     (+27%)
    //   theta = 3     exact 1 - e^-3   = 0.95021   Euler 3       (overshoot)
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));
    const ConstantMedium medium;

    const auto one_step = [&](float tau, float h) {
        std::vector<BodyState> bodies{MakeUnitBody()};
        std::vector<RotorRow> rotors{MakeRotor(0)};
        rotors[0].tau = tau;
        rotors[0].omega = 0.0f;
        rotors[0].omega_cmd = kOmega;
        apply_rotors(bodies, rotors, world, medium, params, h);
        return static_cast<double>(rotors[0].omega) / kOmega;
    };

    EXPECT_NEAR(one_step(0.1f, 0.05f), lag_alpha_ref(0.5), 1e-5);
    EXPECT_LT(one_step(0.1f, 0.05f), 0.45) << "an explicit-Euler step would give 0.5 here";

    // The stability claim: no step ratio, however coarse, overshoots the
    // command. Explicit Euler reaches 3x the command at theta = 3 and rings.
    for (const float theta : {0.5f, 1.0f, 2.0f, 3.0f, 10.0f, 1000.0f}) {
        const double reached = one_step(0.1f, 0.1f * theta);
        EXPECT_GE(reached, 0.0) << "theta = " << theta;
        EXPECT_LE(reached, 1.0 + 1e-6) << "theta = " << theta << ": the lag must never overshoot";
    }
    EXPECT_NEAR(one_step(0.1f, 0.3f), lag_alpha_ref(3.0), 1e-5);
}

TEST(RotorElement, LaggedShaftSpeedDrivesTheThrustCurve) {
    // The lag is not cosmetic: while omega is climbing, thrust must follow
    // omega^2 through the SAME curve, not jump to the commanded value.
    constexpr float kTau = 1.0f / 16.0f;
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));
    const ConstantMedium medium;

    std::vector<BodyState> bodies{MakeUnitBody()};
    std::vector<RotorRow> rotors{MakeRotor(0)};
    rotors[0].tau = kTau;
    rotors[0].omega = 0.0f;
    rotors[0].omega_cmd = kOmega;

    for (int i = 0; i < 32; ++i) {  // half a tau
        bodies[0].force_acc = glm::vec3(0.0f);
        bodies[0].torque_acc = glm::vec3(0.0f);
        apply_rotors(bodies, rotors, world, medium, params, kH);
    }

    const double omega_now = static_cast<double>(rotors[0].omega);
    EXPECT_NEAR(omega_now / kOmega, lag_alpha_ref(0.5), 1e-4);
    EXPECT_NEAR(static_cast<double>(bodies[0].force_acc.y), thrust_static_ref(omega_now), 1e-4);
    EXPECT_LT(bodies[0].force_acc.y, 0.25f * static_cast<float>(thrust_static_ref(kOmega)))
        << "thrust follows omega^2, so half the speed is a quarter the thrust";
}

// ===========================================================================
// Ground effect, and its SDF source.
// ===========================================================================

TEST(RotorElement, GroundFactorMatchesCheesemanBennettAndSaturatesAtTheClamp) {
    using spade::vehicles::rotor_ground_factor;
    using spade::vehicles::kRotorMaxGroundFactor;

    // Out of ground effect: the brief's z > 4R bound. At exactly 4R the model
    // gives 1.0039, so "approximately 1" is a 0.5% claim, not a hand wave.
    for (const double z_over_r : {4.0, 6.0, 10.0, 50.0}) {
        const double z = z_over_r * kRadius;
        const float f = rotor_ground_factor(static_cast<float>(z), kRadius);
        EXPECT_NEAR(static_cast<double>(f), ground_factor_unclamped_ref(z, kRadius), 1e-5);
        EXPECT_NEAR(f, 1.0f, 0.005f) << "z/R = " << z_over_r;
        EXPECT_GE(f, 1.0f) << "ground effect can only ever ADD thrust";
    }

    // In ground effect: greater than 1, and matching the closed form until the
    // clamp binds at z = 0.559 R.
    for (const double z_over_r : {0.6, 0.8, 1.0, 1.5, 2.0, 3.0}) {
        const double z = z_over_r * kRadius;
        const double reference = ground_factor_unclamped_ref(z, kRadius);
        const float f = rotor_ground_factor(static_cast<float>(z), kRadius);
        EXPECT_NEAR(static_cast<double>(f), reference, reference * 1e-5) << "z/R = " << z_over_r;
        EXPECT_GT(f, 1.0f) << "z/R = " << z_over_r;
        EXPECT_LE(f, kRotorMaxGroundFactor);
    }
    EXPECT_GT(rotor_ground_factor(kRadius, kRadius), 1.06f) << "z = R is 1/(1-1/16^... ) = 1.0667";

    // Below 0.559 R the unclamped model runs away (it diverges at R/4); the
    // clamp is what makes that a bounded, sane answer.
    EXPECT_GT(ground_factor_unclamped_ref(0.5 * kRadius, kRadius), 1.33);
    for (const double z_over_r : {0.55, 0.5, 0.3, 0.26, 0.25, 0.1, 0.001}) {
        EXPECT_FLOAT_EQ(rotor_ground_factor(static_cast<float>(z_over_r * kRadius), kRadius),
                        kRotorMaxGroundFactor)
            << "z/R = " << z_over_r;
    }

    // Degenerate distances and radii are total, and inside the solid saturates
    // rather than inverting sign.
    EXPECT_FLOAT_EQ(rotor_ground_factor(0.0f, kRadius), kRotorMaxGroundFactor);
    EXPECT_FLOAT_EQ(rotor_ground_factor(-1.0f, kRadius), kRotorMaxGroundFactor);
    EXPECT_FLOAT_EQ(rotor_ground_factor(std::numeric_limits<float>::quiet_NaN(), kRadius),
                    kRotorMaxGroundFactor);
    EXPECT_FLOAT_EQ(rotor_ground_factor(1.0f, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(rotor_ground_factor(1.0f, -1.0f), 1.0f);
    EXPECT_FLOAT_EQ(rotor_ground_factor(std::numeric_limits<float>::max(), kRadius), 1.0f);
}

TEST(RotorElement, GroundEffectComesFromTheWorldSdfDistanceNotFromAltitude) {
    // The brief's fifth requirement, and D5's "the collision field IS the
    // proximity query" made falsifiable.
    //
    // The `gate` world's LEFT POST is a box whose top face is y = 3.0. A rotor
    // hovering 0.12 m above that face is 0.12 m from the world -- but 3.12 m
    // above the world ORIGIN, and there is no ground plane in this world at
    // all. So:
    //
    //   * an implementation reading the SDF gets z = 0.12, R/4z = 0.3125, and
    //     f_ground = 1/(1 - 0.09766) = 1.1082;
    //   * an implementation reading pos.y as an altitude would get z = 3.12
    //     and f_ground = 1.00014, i.e. no effect;
    //   * an implementation with no ground model at all would get 1.0.
    //
    // The three are three ratios apart, so this test cannot pass by accident.
    const SdfProgram gate = GateWorldSdf();
    ASSERT_FALSE(gate.empty());
    ASSERT_TRUE(gate.validate().has_value());

    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));
    const double t_static = thrust_static_ref(kOmega);

    constexpr float kClearance = 0.12f;
    const glm::vec3 above_post(-1.8f, 3.0f + kClearance, 0.0f);
    const glm::vec3 open_space(10.0f, 3.0f + kClearance, 0.0f);

    // The premise, checked against the field itself before anything is
    // concluded from it.
    ASSERT_NEAR(spade::eval(gate, above_post), kClearance, 1e-4f);
    ASSERT_GT(spade::eval(gate, open_space), 4.0f * kRadius);

    BodyState over_post = MakeUnitBody();
    over_post.pos = above_post;
    BodyState over_open = MakeUnitBody();
    over_open.pos = open_space;

    const Wrench near_geometry = ApplyOnce(over_post, MakeRotor(0), gate, params);
    const Wrench far_from_it = ApplyOnce(over_open, MakeRotor(0), gate, params);

    const double expected_igr = ground_factor_unclamped_ref(kClearance, kRadius);
    ASSERT_NEAR(expected_igr, 1.1082, 1e-3);

    EXPECT_NEAR(static_cast<double>(near_geometry.force.y) / t_static, expected_igr, expected_igr * 1e-4)
        << "the rotor over the gate's post must feel ground effect";
    EXPECT_NEAR(static_cast<double>(far_from_it.force.y) / t_static, 1.0, 1e-4)
        << "the rotor over open space must feel none";
    EXPECT_GT(near_geometry.force.y, far_from_it.force.y * 1.05f);

    // The altitude reading is excluded explicitly: it would have produced
    // essentially the open-space answer at the in-ground-effect station.
    EXPECT_LT(ground_factor_unclamped_ref(static_cast<double>(above_post.y), kRadius), 1.001);
}

TEST(RotorElement, GroundEffectIsSampledAtTheRotorStationNotTheBodyOrigin) {
    // Spec S6: "Aero force elements read Medium at their body-frame station" --
    // and the same is true of the SDF probe, which is what makes a low-slung
    // rotor on a tall airframe feel the ground the airframe's origin does not.
    // Every test above this point mounts the rotor AT the COM, where
    // p_rotor == body.pos and the distinction is invisible.
    //
    // Both halves put the rotor at the same world point -- 0.12 m above the
    // gate's left post, f_ground = 1.1082 -- by two different routes.
    const SdfProgram gate = GateWorldSdf();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));
    const double t_static = thrust_static_ref(kOmega);
    const double expected = ground_factor_unclamped_ref(0.12, kRadius);

    // (a) Identity attitude, rotor slung 0.88 m below a body whose own origin
    //     is a full metre above the post -- comfortably OUT of ground effect
    //     (1 m is 6.7 R, where the factor is 1.0014).
    BodyState slung = MakeUnitBody();
    slung.pos = glm::vec3(-1.8f, 4.0f, 0.0f);
    ASSERT_GT(spade::eval(gate, slung.pos), 4.0f * kRadius) << "the BODY must be out of ground effect";

    const Wrench w_slung =
        ApplyOnce(slung, MakeRotor(0, 1.0f, glm::vec3(0.0f, -0.88f, 0.0f)), gate, params);
    EXPECT_NEAR(static_cast<double>(w_slung.force.y) / t_static, expected, expected * 1e-3);

    // (b) The offset rotated by the BODY's attitude: 180 deg about world +Z
    //     maps the body-frame offset (0,-0.88,0) onto world (0,+0.88,0), so a
    //     body at y = 2.24 puts its rotor at y = 3.12, i.e. 0.12 m above the
    //     post. An implementation that forgot `body.orient *` would probe
    //     (-1.8, 1.36, 0) -- INSIDE the post -- and report the 1.25 clamp.
    //
    //     R_z(180) = [[-1,0,0],[0,-1,0],[0,0,1]], hand-derived as elsewhere in
    //     this file; it also flips the thrust axis onto world -Y, which is why
    //     the magnitude is taken below.
    BodyState inverted = MakeUnitBody();
    inverted.pos = glm::vec3(-1.8f, 3.12f - 0.88f, 0.0f);
    inverted.orient = glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    const Wrench w_inverted =
        ApplyOnce(inverted, MakeRotor(0, 1.0f, glm::vec3(0.0f, -0.88f, 0.0f)), gate, params);
    EXPECT_NEAR(static_cast<double>(-w_inverted.force.y) / t_static, expected, expected * 1e-3);
    EXPECT_LT(w_inverted.force.y, 0.0f) << "thrust follows the inverted body, so it points down";
    EXPECT_LT(static_cast<double>(-w_inverted.force.y) / t_static, 1.2)
        << "an un-rotated offset would land inside the post and report the 1.25 clamp";
}

// ===========================================================================
// The wrench: axes, frames, moments and the shaft reaction.
// ===========================================================================

TEST(RotorElement, TiltedMountThrustsAlongTheRotatedLocalAxis) {
    // local_orient is the field physics/forces.hpp carried unread for exactly
    // this element. A +90 deg rotation about local +X carries the local +Y
    // thrust axis onto +Z:
    //
    //   R_x(90) = [[1,0,0],[0,0,-1],[0,1,0]],  R_x(90) . (0,1,0) = (0,0,1)
    //
    // hand-derived rather than obtained from glm, the same posture
    // test_forces.cpp takes for its rotation checks.
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));

    const glm::quat tilt = glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    const Wrench w = ApplyOnce(MakeUnitBody(), MakeRotor(0, 1.0f, glm::vec3(0.0f), tilt), world, params);

    const float t_static = static_cast<float>(thrust_static_ref(kOmega));
    ExpectVec3Near(w.force, glm::vec3(0.0f, 0.0f, t_static), 1e-3f);

    // The shaft reaction follows the MOUNT, not the body axis.
    const float q = static_cast<float>(static_cast<double>(kTorqueCoeff) * kOmega * kOmega);
    ExpectVec3Near(w.torque, glm::vec3(0.0f, 0.0f, -q), 1e-4f);
}

TEST(RotorElement, RotatedBodyThrustsAlongTheWorldRotatedAxis) {
    // Body attitude enters force_acc (world frame) but NOT torque_acc (body
    // frame). +90 deg about world +Z carries body +Y onto world -X:
    //
    //   R_z(90) = [[0,-1,0],[1,0,0],[0,0,1]],  R_z(90) . (0,1,0) = (-1,0,0)
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));

    BodyState body = MakeUnitBody();
    body.orient = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    const Wrench w = ApplyOnce(body, MakeRotor(0), world, params);

    const float t_static = static_cast<float>(thrust_static_ref(kOmega));
    ExpectVec3Near(w.force, glm::vec3(-t_static, 0.0f, 0.0f), 1e-3f);

    // Body frame is unmoved by the body's own attitude: still a -Y couple.
    const float q = static_cast<float>(static_cast<double>(kTorqueCoeff) * kOmega * kOmega);
    ExpectVec3Near(w.torque, glm::vec3(0.0f, -q, 0.0f), 1e-4f);
}

TEST(RotorElement, OffsetRotorProducesTheExpectedMomentAndReaction) {
    // r = (L, 0, 0), thrust along body +Y:
    //   cross(r, F) = cross((L,0,0),(0,T,0)) = (0, 0, L T)
    // plus the shaft reaction -spin Q along +Y. A flipped cross-product
    // argument order would put -L T on z, which the SIGNED check below catches.
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));
    constexpr float kArm = 0.2f;

    const Wrench w = ApplyOnce(MakeUnitBody(), MakeRotor(0, 1.0f, glm::vec3(kArm, 0.0f, 0.0f)), world, params);

    const double t_static = thrust_static_ref(kOmega);
    const double q = static_cast<double>(kTorqueCoeff) * kOmega * kOmega;
    ExpectVec3Near(w.torque,
                   glm::vec3(0.0f, static_cast<float>(-q), static_cast<float>(kArm * t_static)), 1e-4f);
    EXPECT_GT(w.torque.z, 1.0f) << "sign check must not be vacuously near zero";
}

TEST(RotorElement, CounterRotatingPairCancelsYawAndDifferentialSteersIt) {
    // The reason spin_dir exists. Two rotors on a +-x arm, opposite senses,
    // equal speed: the two moments cancel, the two shaft reactions cancel, and
    // only 2T of lift survives. Unequal speeds then produce net yaw with the
    // sign of the faster rotor's reaction.
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));
    const ConstantMedium medium;
    constexpr float kArm = 0.2f;

    const auto run = [&](float omega_left, float omega_right) {
        std::vector<BodyState> bodies{MakeUnitBody()};
        std::vector<RotorRow> rotors{
            MakeRotor(0, +1.0f, glm::vec3(-kArm, 0.0f, 0.0f)),
            MakeRotor(0, -1.0f, glm::vec3(+kArm, 0.0f, 0.0f)),
        };
        rotors[0].omega = rotors[0].omega_cmd = omega_left;
        rotors[1].omega = rotors[1].omega_cmd = omega_right;
        apply_rotors(bodies, rotors, world, medium, params, kH);
        return Wrench{bodies[0].force_acc, bodies[0].torque_acc};
    };

    const Wrench balanced = run(kOmega, kOmega);
    const float t_static = static_cast<float>(thrust_static_ref(kOmega));
    ExpectVec3Near(balanced.force, glm::vec3(0.0f, 2.0f * t_static, 0.0f), 1e-3f);
    ExpectVec3Near(balanced.torque, glm::vec3(0.0f), 1e-4f);

    // A 10% differential steers yaw toward the faster rotor's reaction: the
    // left rotor (spin +1) reacts -Y, so speeding it up gives net -Y.
    const Wrench yawing = run(1.1f * kOmega, kOmega);
    EXPECT_LT(yawing.torque.y, -1e-3f);
    EXPECT_GT(yawing.force.y, balanced.force.y) << "and it also lifts more, which is why yaw needs a mixer";
}

TEST(RotorElement, HoverTrimHoldsStationUnderGravity) {
    // The engine design spec's own S12 validation item ("hover trim: thrust =
    // weight => station-keeping"), run through this element. Four rotors on a
    // +-0.2 m cross, alternating senses, each trimmed to a quarter of the
    // weight: the vehicle must simply stay put for a second.
    //
    // It is also the closed loop the inflow model creates: if the vehicle
    // starts to sink, x goes negative, lambda rises and thrust rises. Trim is
    // therefore a STABLE fixed point of this model in the axial direction, and
    // a sign error in f_inflow would turn it into a divergent one.
    constexpr float kMass = 0.7f;
    constexpr float kGravity = 9.80665f;
    constexpr float kArm = 0.2f;
    const float per_rotor = kMass * kGravity / 4.0f;
    const float k_t = per_rotor / (kOmega * kOmega);

    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f, -kGravity, 0.0f), glm::vec3(0.0f));
    const ConstantMedium medium;

    std::vector<BodyState> bodies{MakeUnitBody()};
    bodies[0].mass = kMass;
    bodies[0].inv_inertia_diag = glm::vec3(1.0f / 0.01f);

    std::vector<RotorRow> rotors{
        MakeRotor(0, +1.0f, glm::vec3(+kArm, 0.0f, 0.0f)),
        MakeRotor(0, -1.0f, glm::vec3(-kArm, 0.0f, 0.0f)),
        MakeRotor(0, +1.0f, glm::vec3(0.0f, 0.0f, +kArm)),
        MakeRotor(0, -1.0f, glm::vec3(0.0f, 0.0f, -kArm)),
    };
    for (RotorRow& r : rotors) {
        r.thrust_coeff = k_t;
        r.torque_coeff = kTorqueCoeff;
    }

    for (int i = 0; i < 1024; ++i) {  // 1.0 s
        apply_rotors(bodies, rotors, world, medium, params, kH);
        spade::physics::integrate_bodies(bodies, params, kH);
    }

    EXPECT_LT(glm::length(bodies[0].pos), 1e-3f) << "pos = " << bodies[0].pos.y;
    EXPECT_LT(glm::length(bodies[0].vel), 1e-3f);
    EXPECT_LT(glm::length(bodies[0].omega_body), 1e-4f) << "the counter-rotating cross must not spin up";

    // Not vacuous: the same second of gravity with the rotors off is a 4.9 m
    // fall.
    std::vector<BodyState> falling{bodies[0]};
    falling[0].pos = glm::vec3(0.0f);
    falling[0].vel = glm::vec3(0.0f);
    falling[0].force_acc = glm::vec3(0.0f);
    falling[0].torque_acc = glm::vec3(0.0f);
    for (int i = 0; i < 1024; ++i) {
        spade::physics::integrate_bodies(falling, params, kH);
    }
    EXPECT_LT(falling[0].pos.y, -4.0f);
}

// ===========================================================================
// Structural contracts: skips, degenerate shaft speeds, determinism.
// ===========================================================================

TEST(RotorElement, DisabledRotorIsInertIncludingItsOwnShaftSpeed) {
    // A disabled rotor must be frozen, not free-running: the omega update is
    // inside the skip, so its row is byte-for-byte untouched.
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));
    const ConstantMedium medium;

    std::vector<BodyState> bodies{MakeUnitBody()};
    std::vector<RotorRow> rotors{MakeRotor(0)};
    rotors[0].enabled = 0u;
    rotors[0].tau = 0.05f;
    rotors[0].omega = 0.0f;
    rotors[0].omega_cmd = kOmega;
    const RotorRow before = rotors[0];

    apply_rotors(bodies, rotors, world, medium, params, kH);

    ExpectVec3Near(bodies[0].force_acc, glm::vec3(0.0f), 0.0f);
    ExpectVec3Near(bodies[0].torque_acc, glm::vec3(0.0f), 0.0f);
    EXPECT_EQ(std::memcmp(&rotors[0], &before, sizeof(RotorRow)), 0);
}

TEST(RotorElement, InactiveBodyIsSkippedAndItsRotorFrozen) {
    const SdfProgram world = EmptyWorld();
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(0.0f));
    const ConstantMedium medium;

    BodyState inactive = MakeUnitBody();
    inactive.flags = 0u;  // the case under test
    std::vector<BodyState> bodies{MakeUnitBody(), inactive};
    const BodyState inactive_before = bodies[1];

    std::vector<RotorRow> rotors{MakeRotor(0), MakeRotor(1)};
    rotors[1].tau = 0.05f;
    rotors[1].omega = 0.0f;
    rotors[1].omega_cmd = kOmega;
    const RotorRow rotor_before = rotors[1];

    apply_rotors(bodies, rotors, world, medium, params, kH);

    // The active neighbour did accumulate, so the span was really traversed.
    EXPECT_GT(bodies[0].force_acc.y, 1.0f);
    EXPECT_EQ(std::memcmp(&bodies[1], &inactive_before, sizeof(BodyState)), 0);
    EXPECT_EQ(std::memcmp(&rotors[1], &rotor_before, sizeof(RotorRow)), 0);
}

TEST(RotorElement, StoppedRotorProducesNoWrenchAndNoNaN) {
    // omega == 0 makes v_h == 0, so x = v_axial / v_h is the division the
    // inflow model must never actually perform. It is also the state every
    // freshly allocated arena slot is in.
    const WorldParams params = MakeParams(glm::vec3(0.0f), glm::vec3(3.0f, -7.0f, 2.0f));

    for (const SdfProgram& world : {EmptyWorld(), GateWorldSdf()}) {
        BodyState body = MakeUnitBody();
        body.pos = glm::vec3(-1.8f, 3.01f, 0.0f);  // deep in ground effect for the gate world
        body.vel = glm::vec3(1.0f, -12.0f, 0.5f);  // and descending hard

        RotorRow rotor = MakeRotor(0);
        rotor.omega = 0.0f;
        rotor.omega_cmd = 0.0f;

        const Wrench w = ApplyOnce(body, rotor, world, params);
        ExpectVec3Near(w.force, glm::vec3(0.0f), 0.0f);
        ExpectVec3Near(w.torque, glm::vec3(0.0f), 0.0f);
        EXPECT_TRUE(std::isfinite(w.force.y));
        EXPECT_TRUE(std::isfinite(w.torque.y));
    }

    // A zero radius is the other way to get v_h == 0 (no disc area), and a
    // zero density is the third (a vacuum world).
    RotorRow no_disc = MakeRotor(0);
    no_disc.radius = 0.0f;
    BodyState climbing = MakeUnitBody();
    climbing.vel = glm::vec3(0.0f, 30.0f, 0.0f);
    const Wrench w = ApplyOnce(climbing, no_disc, EmptyWorld(), params);
    EXPECT_TRUE(std::isfinite(w.force.y));
    EXPECT_NEAR(static_cast<double>(w.force.y), thrust_static_ref(kOmega), 1e-3)
        << "with no disc area there is no inflow correction to make";
}

TEST(RotorElement, TwoIdenticalRunsAreByteIdentical) {
    const SdfProgram world = GateWorldSdf();
    const WorldParams params = MakeParams(glm::vec3(0.0f, -9.80665f, 0.0f), glm::vec3(1.5f, -0.5f, 0.25f));
    const ConstantMedium medium;

    const auto make_bodies = []() {
        std::vector<BodyState> bodies(2);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const float k = static_cast<float>(i);
            bodies[i] = MakeUnitBody();
            bodies[i].pos = glm::vec3(-1.8f + k, 3.2f + 0.5f * k, 0.1f * k);
            bodies[i].vel = glm::vec3(0.3f + k, -1.5f, 0.25f * k);
            bodies[i].omega_body = glm::vec3(0.2f, -0.1f * k, 0.05f);
            bodies[i].orient = glm::normalize(glm::quat(0.7f, 0.1f * k, 0.2f, -0.05f * k));
        }
        return bodies;
    };
    const auto make_rotors = []() {
        std::vector<RotorRow> rotors{
            MakeRotor(0, +1.0f, glm::vec3(0.2f, 0.0f, 0.0f)),
            MakeRotor(0, -1.0f, glm::vec3(-0.2f, 0.0f, 0.05f),
                      glm::angleAxis(glm::radians(12.0f), glm::vec3(1.0f, 0.0f, 0.0f))),
            MakeRotor(1, +1.0f, glm::vec3(0.0f, 0.05f, 0.2f)),
            [] {  // a disabled row, exercising the skip inside the same run
                RotorRow r = MakeRotor(1);
                r.enabled = 0u;
                return r;
            }(),
        };
        rotors[0].tau = 0.03f;
        rotors[0].omega = 0.0f;
        rotors[1].tau = 0.07f;
        rotors[1].omega = 400.0f;
        rotors[2].tau = 0.05f;
        rotors[2].omega_cmd = 800.0f;
        return rotors;
    };

    std::vector<BodyState> bodies_a = make_bodies();
    std::vector<BodyState> bodies_b = make_bodies();
    std::vector<RotorRow> rotors_a = make_rotors();
    std::vector<RotorRow> rotors_b = make_rotors();

    for (int i = 0; i < 200; ++i) {
        apply_rotors(bodies_a, rotors_a, world, medium, params, kH);
        apply_rotors(bodies_b, rotors_b, world, medium, params, kH);
        spade::physics::integrate_bodies(bodies_a, params, kH);
        spade::physics::integrate_bodies(bodies_b, params, kH);
    }

    ASSERT_EQ(bodies_a.size(), bodies_b.size());
    EXPECT_EQ(std::memcmp(bodies_a.data(), bodies_b.data(), bodies_a.size() * sizeof(BodyState)), 0);
    EXPECT_EQ(std::memcmp(rotors_a.data(), rotors_b.data(), rotors_a.size() * sizeof(RotorRow)), 0);

    // Guard against the comparison passing because nothing happened.
    EXPECT_GT(rotors_a[0].omega, 100.0f);
    EXPECT_GT(glm::length(bodies_a[0].pos - glm::vec3(-1.8f, 3.2f, 0.0f)), 1e-3f);
}
