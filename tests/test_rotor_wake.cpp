#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <glm/glm.hpp>

#include "vehicles/rotor.hpp"
#include "vehicles/rotor_wake.hpp"

// ---------------------------------------------------------------------------
// rotor_wake_velocity() (vehicles/rotor_wake.hpp). Read that header's model
// section first. Reference values here are recomputed in double from its
// equations -- the vortex-cylinder axial profile, momentum theory's inflow
// roots, continuity -- rather than read back from a run.
//
// The function is visualisation grade and nothing in the step reads it, so
// these tests pin physics and totality, not a parity band.
// ---------------------------------------------------------------------------

namespace {

using spade::vehicles::rotor_wake_velocity;
using spade::vehicles::RotorWakeInput;

// The sandbox drone's rotor at roughly hover: 1.2e-5 * 452^2 = 2.45 N, a
// quarter of 1 kg * g.
RotorWakeInput hover_rotor() {
    RotorWakeInput in;
    in.hub_world = glm::vec3(0.0f);
    in.thrust_axis_world = glm::vec3(0.0f, 1.0f, 0.0f);
    in.radius = 0.12f;
    in.thrust_coeff = 1.2e-5f;
    in.omega = 452.0f;
    in.density = 1.225f;
    return in;
}

// (H) in double: v_h = sqrt(T / (2 rho pi R^2)).
double v_hover_ref(const RotorWakeInput& in) {
    const double t = double(in.thrust_coeff) * double(in.omega) * double(in.omega);
    const double r = in.radius;
    return std::sqrt(t / (2.0 * double(in.density) * 3.14159265358979323846 * r * r));
}

// The vortex-cylinder axial profile u(s) / v_i, in double, written the
// textbook way (double has the digits to spare at the stations used here).
double build_ref(double s, double R) { return 1.0 + s / std::sqrt(s * s + R * R); }

bool finite(const glm::vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

}  // namespace

// --- the axial profile, in still air ---------------------------------------

TEST(RotorWake, AtTheDiscTheInducedVelocityIsVi) {
    const auto in = hover_rotor();
    const glm::vec3 v = rotor_wake_velocity(in, in.hub_world);
    const double v_h = v_hover_ref(in);  // lambda(0) = 1: hover in still air
    EXPECT_NEAR(-v.y, v_h, 1e-5 * v_h);  // flows opposite the thrust axis
    EXPECT_EQ(v.x, 0.0f);
    EXPECT_EQ(v.z, 0.0f);
}

TEST(RotorWake, OnTheAxisItFollowsTheVortexCylinderProfile) {
    const auto in = hover_rotor();
    const double v_h = v_hover_ref(in);
    const double R = in.radius;
    for (const double s_over_R : {0.25, 0.5, 1.0, 3.0, 10.0, 100.0}) {
        const double s = s_over_R * R;
        const glm::vec3 v = rotor_wake_velocity(in, glm::vec3(0.0f, float(-s), 0.0f));
        EXPECT_NEAR(-v.y, v_h * build_ref(s, R), 1e-5 * v_h) << "s/R = " << s_over_R;
    }
    // 100 R downstream is within 0.005% of momentum theory's far wake, 2 v_i.
    const glm::vec3 far = rotor_wake_velocity(in, glm::vec3(0.0f, -100.0f * in.radius, 0.0f));
    EXPECT_NEAR(-far.y, 2.0 * v_h, 1e-4 * v_h);
}

TEST(RotorWake, FarUpstreamItVanishes) {
    const auto in = hover_rotor();
    const glm::vec3 v = rotor_wake_velocity(in, glm::vec3(0.0f, 100.0f * in.radius, 0.0f));
    EXPECT_LT(glm::length(v), 1e-4f * float(v_hover_ref(in)));
}

TEST(RotorWake, UpstreamTheProfileKeepsItsDigits) {
    // 1000 R upstream the true profile is R^2 / (q (q - s)) ~ 5e-7. The direct
    // fp32 form 1 + s/q lands in the last few ulps of 1 there and is ~10% off;
    // the conjugate form the implementation uses is good to a rounding.
    const auto in = hover_rotor();
    const double R = in.radius;
    const double s = -1000.0 * R;
    const double q = std::sqrt(s * s + R * R);
    const double expected = v_hover_ref(in) * (R * R) / (q * (q - s));
    const glm::vec3 v = rotor_wake_velocity(in, glm::vec3(0.0f, float(-s), 0.0f));
    EXPECT_NEAR(-v.y / expected, 1.0, 1e-4);
}

// --- the tube --------------------------------------------------------------

TEST(RotorWake, OutsideTheTubeItVanishes) {
    const auto in = hover_rotor();
    const glm::vec3 v = rotor_wake_velocity(in, glm::vec3(5.0f * in.radius, -0.05f, 0.0f));
    EXPECT_LT(glm::length(v), 0.01f * float(v_hover_ref(in)));
}

TEST(RotorWake, InStillAirTheTubeContractsToTheMomentumTheoryRadius) {
    // Continuity in still air: r_tube = R sqrt(v_i / u), i.e. R / sqrt(2) far
    // downstream. The smooth edge is exactly 1/2 AT r_tube, so measuring the
    // half-speed radius measures the contraction.
    const auto in = hover_rotor();
    const double R = in.radius;
    const double s = 200.0 * R;
    const double r_tube = R * std::sqrt(1.0 / build_ref(s, R));
    const float on_axis = -rotor_wake_velocity(in, glm::vec3(0.0f, float(-s), 0.0f)).y;
    const float at_edge = -rotor_wake_velocity(in, glm::vec3(float(r_tube), float(-s), 0.0f)).y;
    EXPECT_NEAR(at_edge / on_axis, 0.5f, 1e-3f);
    EXPECT_NEAR(r_tube / R, 1.0 / std::sqrt(2.0), 1e-4);  // the classical far-wake contraction

    const float inside = -rotor_wake_velocity(in, glm::vec3(0.6f * in.radius, float(-s), 0.0f)).y;
    const float outside = -rotor_wake_velocity(in, glm::vec3(0.85f * in.radius, float(-s), 0.0f)).y;
    EXPECT_GT(inside / on_axis, 0.9f);
    EXPECT_LT(outside / on_axis, 0.1f);
}

TEST(RotorWake, AClimbingRotorsSlipstreamContractsLess) {
    // Climbing at 2 v_h (air flowing down through the disc at that speed), the
    // freestream carries most of the mass flow: r_tube = R sqrt((V + v_i) /
    // (V + 2 v_i)) = 0.924 R far downstream, against 0.707 R in still air. So
    // a station at 0.88 R is inside the climbing slipstream and outside the
    // hovering one.
    auto climb = hover_rotor();
    const float v_h = float(v_hover_ref(climb));
    climb.freestream = glm::vec3(0.0f, -2.0f * v_h, 0.0f);
    const auto hover = hover_rotor();

    const float s = 50.0f * climb.radius;
    const glm::vec3 off(0.88f * climb.radius, -s, 0.0f);
    const glm::vec3 on(0.0f, -s, 0.0f);
    const float climb_ratio = glm::length(rotor_wake_velocity(climb, off)) / glm::length(rotor_wake_velocity(climb, on));
    const float hover_ratio = glm::length(rotor_wake_velocity(hover, off)) / glm::length(rotor_wake_velocity(hover, on));
    EXPECT_GT(climb_ratio, 0.5f);
    EXPECT_LT(hover_ratio, 0.1f);
}

// --- the disc follows the engine's inflow curve ---------------------------

TEST(RotorWake, TheDiscVelocityFollowsTheEngineInflowCurve) {
    // v_i = v_h lambda(x), lambda from momentum theory in double: the normal
    // working state root 1 / (x/2 + sqrt(x^2/4 + 1)) in climb, the windmill
    // brake root -x/2 - sqrt(x^2/4 - 1) in fast descent. These are the same
    // roots rotor.hpp section 2 derives, recomputed rather than called.
    auto in = hover_rotor();
    const double v_h = v_hover_ref(in);

    in.freestream = glm::vec3(0.0f, float(-2.0 * v_h), 0.0f);  // climbing at 2 v_h: x = 2
    const double lambda_climb = 1.0 / (1.0 + std::sqrt(2.0));
    EXPECT_NEAR(-rotor_wake_velocity(in, in.hub_world).y, v_h * lambda_climb, 1e-5 * v_h);

    in.freestream = glm::vec3(0.0f, float(3.0 * v_h), 0.0f);  // descending at 3 v_h: x = -3
    const double lambda_wb = 1.5 - std::sqrt(1.25);
    EXPECT_NEAR(-rotor_wake_velocity(in, in.hub_world).y, v_h * lambda_wb, 1e-5 * v_h);
}

TEST(RotorWake, InTheWindmillBrakeStateTheWakeIsOnTheThrustSide) {
    // Descending at 3 v_h the flow passes UP through the disc. The far-wake air
    // velocity F + 2 v_i n is still upward, so the wake (where the disc slows
    // the oncoming air) lies above the rotor, and below it is undisturbed. The
    // flow is decelerating, so continuity EXPANDS the tube:
    // r_tube = R sqrt((3 v_h - v_i) / (3 v_h - 2 v_i)) = 1.082 R far up.
    auto in = hover_rotor();
    const double v_h = v_hover_ref(in);
    in.freestream = glm::vec3(0.0f, float(3.0 * v_h), 0.0f);
    const double v_i = v_h * (1.5 - std::sqrt(1.25));
    const double R = in.radius;
    const double s = 50.0 * R;

    const glm::vec3 above = rotor_wake_velocity(in, glm::vec3(0.0f, float(s), 0.0f));
    EXPECT_NEAR(-above.y, v_i * build_ref(s, R), 1e-5 * v_h);  // still pushing along -axis
    const glm::vec3 below = rotor_wake_velocity(in, glm::vec3(0.0f, float(-s), 0.0f));
    EXPECT_LT(glm::length(below), 1e-3f * float(v_i));

    const glm::vec3 wide = rotor_wake_velocity(in, glm::vec3(1.05f * in.radius, float(s), 0.0f));
    EXPECT_GT(glm::length(wide) / glm::length(above), 0.5f);  // inside an expanded tube
}

// --- skew and orientation --------------------------------------------------

TEST(RotorWake, CrosswindSkewsTheWakeDownwind) {
    // A 5 m/s crosswind leaves lambda at 1 (no axial component) and convects
    // the wake along the far-wake air velocity F + 2 v_i n. On that line the
    // full axial profile appears, still directed along -axis. At the same
    // depth directly below the hub -- where the still-air wake would be --
    // there is next to nothing.
    auto in = hover_rotor();
    in.freestream = glm::vec3(5.0f, 0.0f, 0.0f);
    const double v_i = v_hover_ref(in);
    const glm::vec3 far_wake = in.freestream + glm::vec3(0.0f, float(-2.0 * v_i), 0.0f);
    const glm::vec3 axis_dir = glm::normalize(far_wake);

    const double s = 0.6;
    const glm::vec3 on_skewed_axis = axis_dir * float(s);
    const glm::vec3 v = rotor_wake_velocity(in, on_skewed_axis);
    EXPECT_NEAR(-v.y, v_i * build_ref(s, in.radius), 1e-4 * v_i);
    EXPECT_EQ(v.x, 0.0f);  // induced velocity is normal to the disc, skew or not
    EXPECT_EQ(v.z, 0.0f);

    const glm::vec3 straight_below(0.0f, on_skewed_axis.y, 0.0f);
    EXPECT_LT(glm::length(rotor_wake_velocity(in, straight_below)), 1e-3f * float(v_i));
}

TEST(RotorWake, ATiltedRotorBlowsAlongItsOwnAxisFromItsOwnHub) {
    // Thrust along +X (and deliberately not unit length: the axis is
    // normalised), hub away from the origin. Everything is the still-air
    // hover result, rotated and translated.
    auto in = hover_rotor();
    in.hub_world = glm::vec3(1.0f, 2.0f, -3.0f);
    in.thrust_axis_world = glm::vec3(2.0f, 0.0f, 0.0f);
    const double v_h = v_hover_ref(in);
    const double R = in.radius;

    const glm::vec3 at_hub = rotor_wake_velocity(in, in.hub_world);
    EXPECT_NEAR(-at_hub.x, v_h, 1e-5 * v_h);
    EXPECT_EQ(at_hub.y, 0.0f);
    EXPECT_EQ(at_hub.z, 0.0f);

    const double s = 10.0 * R;
    const glm::vec3 downstream = rotor_wake_velocity(in, in.hub_world + glm::vec3(float(-s), 0.0f, 0.0f));
    EXPECT_NEAR(-downstream.x, v_h * build_ref(s, R), 1e-5 * v_h);
}

// --- totality --------------------------------------------------------------

TEST(RotorWake, AStoppedRotorInducesNothing) {
    auto in = hover_rotor();
    in.omega = 0.0f;
    EXPECT_EQ(rotor_wake_velocity(in, glm::vec3(0.0f, -0.5f, 0.0f)), glm::vec3(0.0f));
}

TEST(RotorWake, DegenerateInputsInduceExactlyNothing) {
    constexpr float kInf = std::numeric_limits<float>::infinity();
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    const glm::vec3 p(0.0f, -0.3f, 0.0f);
    const glm::vec3 zero(0.0f);

    auto check = [&](const char* what, auto mutate, glm::vec3 point) {
        auto in = hover_rotor();
        mutate(in);
        EXPECT_EQ(rotor_wake_velocity(in, point), zero) << what;
    };
    check("zero radius", [](RotorWakeInput& in) { in.radius = 0.0f; }, p);
    check("negative radius", [](RotorWakeInput& in) { in.radius = -0.1f; }, p);
    check("NaN radius", [&](RotorWakeInput& in) { in.radius = kNaN; }, p);
    check("vacuum", [](RotorWakeInput& in) { in.density = 0.0f; }, p);
    check("negative k_T", [](RotorWakeInput& in) { in.thrust_coeff = -1.2e-5f; }, p);
    check("omega so large k_T w^2 overflows", [](RotorWakeInput& in) { in.omega = 1.0e25f; }, p);
    check("infinite omega", [&](RotorWakeInput& in) { in.omega = kInf; }, p);
    check("zero thrust axis", [](RotorWakeInput& in) { in.thrust_axis_world = glm::vec3(0.0f); }, p);
    check("NaN thrust axis", [&](RotorWakeInput& in) { in.thrust_axis_world.y = kNaN; }, p);
    check("NaN freestream", [&](RotorWakeInput& in) { in.freestream.x = kNaN; }, p);
    check("freestream whose square overflows", [](RotorWakeInput& in) { in.freestream.x = 1.0e20f; }, p);
    check("NaN hub", [&](RotorWakeInput& in) { in.hub_world.z = kNaN; }, p);
    check("NaN point", [](RotorWakeInput&) {}, glm::vec3(0.0f, kNaN, 0.0f));
    check("infinite point", [](RotorWakeInput&) {}, glm::vec3(0.0f, -kInf, 0.0f));
    check("point whose square overflows", [](RotorWakeInput&) {}, glm::vec3(0.0f, -1.0e20f, 0.0f));
}

TEST(RotorWake, NeverANaNAcrossEveryInflowBranchAndAnyPoint) {
    // Sweep the freestream through climb, hover, the vortex-ring band (where
    // continuity has no one-signed streamtube and the still-air ratio is used),
    // the windmill brake state and edgewise flow, and probe points through and
    // around the disc. Every answer is finite and no larger than the most any
    // inflow state can induce, 2 * lambda_max * v_h.
    const auto base = hover_rotor();
    const float v_h = float(v_hover_ref(base));
    const float bound = 2.0f * spade::vehicles::kRotorInflowFactorMax * v_h * 1.0001f;
    const float R = base.radius;
    for (const float mag_over_vh : {0.0f, 0.3f, 1.0f, 1.5f, 1.99f, 2.0f, 2.01f, 3.0f, 10.0f, 1.0e6f}) {
        for (int k = 0; k < 16; ++k) {
            const float a = float(k) * 0.392699082f;  // pi / 8 steps, in the X-Y plane
            auto in = base;
            in.freestream = mag_over_vh * v_h * glm::vec3(std::sin(a), std::cos(a), 0.0f);
            for (const glm::vec3 p : {glm::vec3(0.0f), glm::vec3(R, 0.0f, 0.0f), glm::vec3(0.0f, -R, 0.0f),
                                      glm::vec3(0.0f, R, 0.0f), glm::vec3(-3.0f * R, -10.0f * R, R),
                                      glm::vec3(10.0f * R, 10.0f * R, -10.0f * R), glm::vec3(0.0f, -1.0e4f, 0.0f)}) {
                const glm::vec3 v = rotor_wake_velocity(in, p);
                ASSERT_TRUE(finite(v)) << "|F|/v_h = " << mag_over_vh << ", k = " << k;
                EXPECT_LE(glm::length(v), bound) << "|F|/v_h = " << mag_over_vh << ", k = " << k;
            }
        }
    }
}
