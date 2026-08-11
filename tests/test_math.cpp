#include <gtest/gtest.h>

#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/math_ops.hpp"

namespace {

using spade::math::gyroscopic_torque;
using spade::math::inertia_world;
using spade::math::integrate_orientation;

void ExpectQuatNear(const glm::quat& actual, const glm::quat& expected, float atol) {
    EXPECT_NEAR(actual.w, expected.w, atol);
    EXPECT_NEAR(actual.x, expected.x, atol);
    EXPECT_NEAR(actual.y, expected.y, atol);
    EXPECT_NEAR(actual.z, expected.z, atol);
}

void ExpectVec3Near(const glm::vec3& actual, const glm::vec3& expected, float atol) {
    EXPECT_NEAR(actual.x, expected.x, atol);
    EXPECT_NEAR(actual.y, expected.y, atol);
    EXPECT_NEAR(actual.z, expected.z, atol);
}

void ExpectMat3Near(const glm::mat3& actual, const glm::mat3& expected, float atol) {
    for (int col = 0; col < 3; ++col) {
        for (int row = 0; row < 3; ++row) {
            EXPECT_NEAR(actual[col][row], expected[col][row], atol)
                << "at column " << col << ", row " << row;
        }
    }
}

// Reference exp-map computed independently in double precision -- NOT via
// integrate_orientation's own Taylor-series branch -- so the comparison is
// an honest closed-form check rather than the function checking itself.
glm::quat ClosedFormDelta(const glm::vec3& omega_body, float dt) {
    const double omega_len = static_cast<double>(glm::length(omega_body));
    const double half_angle = 0.5 * omega_len * static_cast<double>(dt);
    const glm::vec3 axis = omega_len > 0.0 ? glm::normalize(omega_body) : glm::vec3(0.0f);
    const float w = static_cast<float>(std::cos(half_angle));
    const float s = static_cast<float>(std::sin(half_angle));
    return glm::quat(w, s * axis);
}

}  // namespace

// ---------------------------------------------------------------------------
// integrate_orientation: exp-map vs. closed-form rotation for constant omega.
// ---------------------------------------------------------------------------

TEST(IntegrateOrientation, NinetyDegreeStepAboutSingleAxis) {
    const glm::quat q0(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::vec3 omega(0.0f, 0.0f, 1.0f);  // rad/s about +z
    const float dt = glm::half_pi<float>();   // theta = |omega|*dt == pi/2

    const glm::quat actual = integrate_orientation(q0, omega, dt);
    const glm::quat expected = glm::normalize(q0 * ClosedFormDelta(omega, dt));

    ExpectQuatNear(actual, expected, 1e-6f);
}

TEST(IntegrateOrientation, NinetyDegreeStepAboutCombinedAxis) {
    const glm::quat q0(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::vec3 axis = glm::normalize(glm::vec3(1.0f, 2.0f, -1.0f));
    const float target_theta = glm::half_pi<float>();
    const float omega_mag = 3.0f;  // rad/s
    const glm::vec3 omega = axis * omega_mag;
    const float dt = target_theta / omega_mag;  // theta == pi/2 exactly

    const glm::quat actual = integrate_orientation(q0, omega, dt);
    const glm::quat expected = glm::normalize(q0 * ClosedFormDelta(omega, dt));

    ExpectQuatNear(actual, expected, 1e-6f);
}

TEST(IntegrateOrientation, NonIdentityStartPinsBodyFrameRightMultiplication) {
    // Every other correctness test in this file uses q0 == identity (or, for
    // ZeroAngularVelocityIsIdentityStep, omega == 0, making delta itself the
    // identity quaternion) -- both make q0*delta and delta*q0 numerically
    // identical, since the identity commutes with everything. None of those
    // tests can catch a regression that silently switched
    // integrate_orientation to the world-frame left-multiplication
    // convention (delta * q0) instead of the spec'd body-frame q (x)
    // exp(0.5*dt*omega_body) (right-multiplication, D1). This test picks a
    // non-identity q0 whose rotation axis is not parallel to omega's, so
    // q0*delta and delta*q0 provably diverge (quaternions commute only when
    // their axes are parallel) -- it fails if the implementation were
    // delta * q0.
    const glm::quat q0 = glm::normalize(glm::quat(0.6f, 0.1f, 0.4f, -0.3f));
    const glm::vec3 omega(0.5f, -0.8f, 1.2f);  // rad/s; axis not parallel to q0's
    const float dt = 0.2f;                     // theta ~ 17.5 deg -- comfortably non-trivial

    const glm::quat actual = integrate_orientation(q0, omega, dt);
    const glm::quat expected = glm::normalize(q0 * ClosedFormDelta(omega, dt));

    ExpectQuatNear(actual, expected, 1e-6f);
}

TEST(IntegrateOrientation, TinyAngleBranchMatchesClosedForm) {
    const glm::quat q0(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::vec3 omega(0.0f, 0.0f, 1e-4f);  // rad/s
    const float dt = 1e-3f;                    // theta = 1e-7, below the 1e-6 switchover

    const glm::quat actual = integrate_orientation(q0, omega, dt);
    const glm::quat expected = glm::normalize(q0 * ClosedFormDelta(omega, dt));

    ExpectQuatNear(actual, expected, 1e-6f);
}

TEST(IntegrateOrientation, ZeroAngularVelocityIsIdentityStep) {
    // theta == 0 exactly -- the tiny-angle branch must not divide by zero.
    const glm::quat q0 = glm::normalize(glm::quat(0.7f, 0.1f, -0.2f, 0.3f));
    const glm::quat actual = integrate_orientation(q0, glm::vec3(0.0f), 1e-3f);

    ExpectQuatNear(actual, q0, 1e-6f);
}

TEST(IntegrateOrientation, NormPreservedOverManySteps) {
    glm::quat q(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::vec3 omega(0.3f, 0.5f, -0.2f);
    const float dt = 1e-3f;

    for (int i = 0; i < 100000; ++i) {
        q = integrate_orientation(q, omega, dt);
    }

    EXPECT_NEAR(glm::length(q), 1.0f, 1e-5f);
}

// ---------------------------------------------------------------------------
// gyroscopic_torque: -(omega x I*omega).
// ---------------------------------------------------------------------------

TEST(GyroscopicTorque, ZeroForSphericalInertia) {
    const glm::mat3 I_body(2.0f);  // 2 * identity: spherical inertia
    const glm::vec3 omega(0.7f, -1.3f, 2.1f);

    const glm::vec3 torque = gyroscopic_torque(I_body, omega);

    ExpectVec3Near(torque, glm::vec3(0.0f), 1e-6f);
}

TEST(GyroscopicTorque, MatchesHandComputedValue) {
    const glm::mat3 I_body(
        1.0f, 0.0f, 0.0f,
        0.0f, 2.0f, 0.0f,
        0.0f, 0.0f, 3.0f);
    const glm::vec3 omega(1.0f, 1.0f, 1.0f);

    // I*omega = (1, 2, 3); omega x I*omega = (1*3-1*2, 1*1-1*3, 1*2-1*1) = (1, -2, 1);
    // gyroscopic_torque = -(1, -2, 1) = (-1, 2, -1).
    const glm::vec3 expected(-1.0f, 2.0f, -1.0f);

    ExpectVec3Near(gyroscopic_torque(I_body, omega), expected, 1e-6f);
}

// ---------------------------------------------------------------------------
// inertia_world: rotate a body-frame tensor into world frame.
// ---------------------------------------------------------------------------

TEST(InertiaWorld, IdentityOrientationIsUnchanged) {
    const glm::mat3 I_body(
        1.0f, 0.0f, 0.0f,
        0.0f, 2.0f, 0.0f,
        0.0f, 0.0f, 3.0f);
    const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);

    ExpectMat3Near(inertia_world(I_body, identity), I_body, 1e-6f);
}

TEST(InertiaWorld, NinetyDegreeYawSwapsTransverseMoments) {
    // Rotating the body frame 90 degrees about z maps body-x -> world-y and
    // body-y -> world-(-x), so a diagonal tensor's x/y principal moments
    // swap: I_world = R * diag(a,b,c) * R^T == diag(b, a, c).
    const glm::mat3 I_body(
        1.0f, 0.0f, 0.0f,
        0.0f, 5.0f, 0.0f,
        0.0f, 0.0f, 9.0f);
    const glm::quat yaw90 = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f));

    const glm::mat3 expected(
        5.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 9.0f);

    ExpectMat3Near(inertia_world(I_body, yaw90), expected, 1e-5f);
}

TEST(InertiaWorld, PrincipalMomentsOverloadMatchesDiagonalMatrix) {
    const glm::vec3 principal(1.0f, 5.0f, 9.0f);
    const glm::mat3 I_body(
        1.0f, 0.0f, 0.0f,
        0.0f, 5.0f, 0.0f,
        0.0f, 0.0f, 9.0f);
    const glm::quat q = glm::normalize(glm::quat(0.6f, 0.1f, 0.4f, -0.3f));

    ExpectMat3Near(inertia_world(principal, q), inertia_world(I_body, q), 1e-6f);
}

// ---------------------------------------------------------------------------
// Torque-free symmetric top: precession rate of the transverse angular
// velocity component vs. the analytic rate
//   Omega = omega_z * (I_axial - I_transverse) / I_transverse,
// integrated with the library's own gyroscopic_torque over many substeps
// (explicit Euler on domega/dt = I_body^-1 * gyroscopic_torque(I_body,
// omega); I_body is diagonal here so the per-axis division is a scalar
// divide). This exercises gyroscopic_torque as a physically-behaved ODE
// term, not just a single-call formula check.
// ---------------------------------------------------------------------------

TEST(GyroscopicTorque, TorqueFreeSymmetricTopPrecessionRateMatchesAnalytic) {
    const float I_transverse = 1.0f;
    const float I_axial = 2.0f;
    const glm::mat3 I_body(
        I_transverse, 0.0f, 0.0f,
        0.0f, I_transverse, 0.0f,
        0.0f, 0.0f, I_axial);

    glm::vec3 omega(0.1f, 0.0f, 1.0f);  // transverse component along +x, spin about z
    const float dt = 1e-4f;
    constexpr int kSteps = 10000;  // T = 1.0s

    const float omega_z0 = omega.z;
    const float transverse_mag0 = glm::length(glm::vec2(omega.x, omega.y));
    const float start_angle = std::atan2(omega.y, omega.x);

    for (int i = 0; i < kSteps; ++i) {
        const glm::vec3 torque = gyroscopic_torque(I_body, omega);
        const glm::vec3 domega(torque.x / I_transverse, torque.y / I_transverse, torque.z / I_axial);
        omega += domega * dt;
    }

    const float analytic_rate = omega_z0 * (I_axial - I_transverse) / I_transverse;  // rad/s
    const float expected_angle = start_angle + analytic_rate * (kSteps * dt);
    const float actual_angle = std::atan2(omega.y, omega.x);

    EXPECT_NEAR(actual_angle, expected_angle, 1e-3f);
    EXPECT_NEAR(omega.z, omega_z0, 1e-3f);  // I_x == I_y => omega_z is conserved
    EXPECT_NEAR(glm::length(glm::vec2(omega.x, omega.y)), transverse_mag0, 1e-3f);
}
