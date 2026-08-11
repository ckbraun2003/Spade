#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/integrator.hpp"
#include "state/layout.hpp"

// ---------------------------------------------------------------------------
// Integrate pass tests (engine design spec D1, §5; charter P1/P2).
//
// The expected values here are derived independently of the implementation --
// in double precision, from the closed form of the DISCRETE scheme, not by
// re-running the integrator's own arithmetic. Where a continuous-time closed
// form is the honest reference (the ballistic arc), both are checked: the
// discrete one tightly, and the continuous one to within symplectic Euler's
// analytically-predicted O(h) position offset, so the test proves the body
// traces a real ballistic arc rather than merely being self-consistent.
// ---------------------------------------------------------------------------

namespace {

using spade::BodyState;
using spade::WorldParams;
using spade::physics::integrate_bodies;

namespace body_flags = spade::physics::body_flags;

void ExpectVec3Near(const glm::vec3& actual, const glm::vec3& expected, float atol) {
    EXPECT_NEAR(actual.x, expected.x, atol);
    EXPECT_NEAR(actual.y, expected.y, atol);
    EXPECT_NEAR(actual.z, expected.z, atol);
}

void ExpectQuatNear(const glm::quat& actual, const glm::quat& expected, float atol) {
    EXPECT_NEAR(actual.w, expected.w, atol);
    EXPECT_NEAR(actual.x, expected.x, atol);
    EXPECT_NEAR(actual.y, expected.y, atol);
    EXPECT_NEAR(actual.z, expected.z, atol);
}

// A live unit body: identity attitude, 1 kg, unit principal moments, at rest,
// no accumulated wrench. Value-initialized first so the std430 pad bytes are
// deterministically zero -- the determinism test memcmps whole BodyStates.
BodyState MakeUnitBody() {
    BodyState body{};
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    body.flags = body_flags::active;
    return body;
}

WorldParams MakeParams(const glm::vec3& gravity) {
    WorldParams params{};
    params.gravity = gravity;
    return params;
}

// Rotational kinetic energy in the body frame, 0.5 * omega . (I * omega),
// accumulated in double so the reference does not itself drift.
double RotationalKineticEnergy(const glm::vec3& omega_body, const glm::dvec3& I_diag) {
    const glm::dvec3 w(omega_body);
    return 0.5 * (w.x * w.x * I_diag.x + w.y * w.y * I_diag.y + w.z * w.z * I_diag.z);
}

// A substep size that is exact in binary (2^-10), so the test's reference
// arithmetic and the integrator's agree about what `h` even is. Everything
// left over is genuine fp32 accumulation error, which is what these
// tolerances are meant to be measuring.
constexpr float kH = 1.0f / 1024.0f;

}  // namespace

// ---------------------------------------------------------------------------
// Ballistic arc. Under constant acceleration a, symplectic Euler (velocity
// updated first, position from the UPDATED velocity) has the exact discrete
// solution
//
//     v_n = v0 + n*a*h
//     p_n = p0 + n*h*v0 + a*h^2*n(n+1)/2
//
// because p_n = p0 + h * sum_{k=1..n} v_k and sum_{k=1..n} k = n(n+1)/2. Note
// the (n+1), not n: that extra a*h^2*n/2 == a*h*T/2 is exactly the O(h)
// offset from the continuous solution p0 + v0*T + a*T^2/2, checked separately
// below. Getting it right is the whole difference between symplectic Euler
// and explicit Euler, so this test would fail if step 5 used the pre-update
// velocity.
// ---------------------------------------------------------------------------

TEST(Integrator, BallisticArcMatchesDiscreteClosedForm) {
    constexpr int kSteps = 1024;  // T = 1.0 s exactly
    const glm::vec3 gravity(0.0f, 0.0f, -9.81f);
    const WorldParams params = MakeParams(gravity);

    BodyState body = MakeUnitBody();
    body.pos = glm::vec3(0.0f);
    body.vel = glm::vec3(8.0f, 0.0f, 4.0f);

    const glm::vec3 p0 = body.pos;
    const glm::vec3 v0 = body.vel;

    std::vector<BodyState> bodies{body};
    for (int i = 0; i < kSteps; ++i) {
        // No external force: force_acc stays zero (step 7 clears it anyway),
        // so the only acceleration is gravity.
        integrate_bodies(bodies, params, kH);
    }

    const double h = static_cast<double>(kH);
    const double n = static_cast<double>(kSteps);
    const glm::dvec3 a(gravity);

    const glm::dvec3 expected_vel = glm::dvec3(v0) + n * a * h;
    const glm::dvec3 expected_pos = glm::dvec3(p0) + n * h * glm::dvec3(v0) + a * h * h * (n * (n + 1.0) / 2.0);

    // Position: the task's specified 1e-4 m. Nothing but fp32 accumulation
    // separates the two -- both describe the same discrete scheme -- so this
    // is measuring 1024 roundings of `pos += vel*h` at a ~6 m scale. Measured
    // worst axis on this box (MSVC, both configs): 2.2e-5 m, a 4.6x margin.
    ExpectVec3Near(bodies[0].pos, glm::vec3(expected_pos), 1e-4f);

    // Velocity gets its own, looser bound rather than borrowing the position
    // one: it is a different quantity in different units (m/s), and 1024
    // roundings of `vel += a*h` at a ~6 m/s scale is intrinsically the larger
    // error of the two -- ulp(5.81f) is 4.8e-7, so even a well-behaved walk
    // lands near 1e-5..1e-4. Measured here: 4.6e-5 m/s, an 11x margin. Reusing
    // 1e-4 would have left only 2.2x, which is not enough headroom for another
    // compiler's FMA contraction (GCC's default -ffp-contract=fast may fuse
    // the multiply-add in steps 3 and 5) to be safely absorbed.
    ExpectVec3Near(bodies[0].vel, glm::vec3(expected_vel), 5e-4f);
}

TEST(Integrator, BallisticArcTracksContinuousSolutionToPredictedFirstOrderOffset) {
    // Same run as above, compared against the CONTINUOUS closed form. The
    // integrator is first-order accurate in position, so it does not match
    // exactly -- but the discrepancy is not free to be anything: it must be
    // the a*h*T/2 that the discrete solution's (n+1) term predicts. Checking
    // the offset rather than just loosening the tolerance is what makes this
    // a real trajectory test.
    constexpr int kSteps = 1024;
    const glm::vec3 gravity(0.0f, 0.0f, -9.81f);
    const WorldParams params = MakeParams(gravity);

    BodyState body = MakeUnitBody();
    body.vel = glm::vec3(8.0f, 0.0f, 4.0f);

    const glm::dvec3 v0(body.vel);

    std::vector<BodyState> bodies{body};
    for (int i = 0; i < kSteps; ++i) {
        integrate_bodies(bodies, params, kH);
    }

    const double h = static_cast<double>(kH);
    const double T = static_cast<double>(kSteps) * h;
    const glm::dvec3 a(gravity);

    const glm::dvec3 continuous = v0 * T + a * (T * T) / 2.0;
    const glm::dvec3 predicted_offset = a * h * T / 2.0;  // == -4.79e-3 m in z

    ExpectVec3Near(bodies[0].pos, glm::vec3(continuous + predicted_offset), 1e-4f);

    // And the offset is real, not lost in the tolerance: z is displaced from
    // the continuous arc by ~4.8 mm, two orders above the 1e-4 m tolerance.
    EXPECT_GT(std::abs(predicted_offset.z), 1e-3);
    EXPECT_GT(std::abs(static_cast<double>(bodies[0].pos.z) - continuous.z), 1e-3);
}

// ---------------------------------------------------------------------------
// Torque-free symmetric top: the gyroscopic term drives a steady precession
// of omega in the body frame, and rotational kinetic energy is a conserved
// quantity of the true dynamics. The angular DOF uses explicit (not
// symplectic) Euler -- D1 buys its accuracy with substeps -- so energy grows
// slowly at O(n * (Omega*h)^2); this pins that growth below 0.1% over 100k
// substeps rather than pretending it is zero.
// ---------------------------------------------------------------------------

TEST(Integrator, FreeSymmetricTopEnergyDriftUnderOneTenthPercent) {
    constexpr int kSteps = 100000;
    constexpr float kSubstep = 1e-4f;  // T = 10 s

    const glm::dvec3 I_diag(1.0, 1.0, 2.0);  // symmetric top: I_x == I_y != I_z
    const WorldParams params = MakeParams(glm::vec3(0.0f));  // torque-free AND force-free

    BodyState body = MakeUnitBody();
    body.inv_inertia_diag = glm::vec3(1.0f, 1.0f, 0.5f);
    body.omega_body = glm::vec3(0.1f, 0.0f, 1.0f);  // spin about z, small transverse tilt

    const double energy0 = RotationalKineticEnergy(body.omega_body, I_diag);
    const float omega_z0 = body.omega_body.z;

    std::vector<BodyState> bodies{body};
    for (int i = 0; i < kSteps; ++i) {
        integrate_bodies(bodies, params, kSubstep);
    }

    const double energy = RotationalKineticEnergy(bodies[0].omega_body, I_diag);
    const double relative_drift = std::abs(energy - energy0) / energy0;

    // Bound from the task brief. The analytic expectation for explicit Euler
    // on a rotation at precession rate Omega = omega_z*(I_a - I_t)/I_t = 1
    // rad/s is that the transverse component's amplitude grows by a factor
    // ~exp(n*(Omega*h)^2/2) = 1 + 5.0e-4, which inflates the transverse share
    // of the energy (0.005 of a 1.005 total) by ~2x that, i.e. ~5.0e-6
    // relative. Measured on this box: 4.98e-6 -- three-figure agreement with
    // the prediction, and a 200x margin against the 1e-3 bound. A regression
    // that made the angular update first-order-wrong would blow past this
    // long before the tolerance mattered.
    EXPECT_LT(relative_drift, 1e-3) << "rotational KE drifted from " << energy0 << " to " << energy;

    // The component of angular velocity along the symmetry axis is exactly
    // conserved by the continuous dynamics. NOTE what this does and does not
    // prove: it follows from I_x == I_y alone, which makes the gyroscopic
    // term's z component (omega_x*I_t*omega_y - omega_y*I_t*omega_x)
    // identically zero for EITHER sign convention. It witnesses that the
    // z-axis coupling vanishes; it is NOT a witness of the term's sign. The
    // signed phase check below is what pins the sign.
    EXPECT_NEAR(bodies[0].omega_body.z, omega_z0, 1e-5f);

    // SIGNED precession phase -- the assertion that catches a flipped
    // gyroscopic sign at the call site. Euler's equations for a torque-free
    // symmetric top, I = (I_t, I_t, I_a), reduce the transverse pair to
    //
    //     I_t * d(omega_x)/dt = -(omega x I*omega)_x = -Omega * omega_y
    //     I_t * d(omega_y)/dt = -(omega x I*omega)_y = +Omega * omega_x
    //
    // i.e. d/dt (omega_x + i*omega_y) = i*Omega*(omega_x + i*omega_y), a
    // COUNTERCLOCKWISE rotation of the transverse component at the precession
    // rate Omega = omega_z*(I_a - I_t)/I_t = 1 rad/s. So from
    // (0.1, 0) at t = 0, the exact solution is 0.1*(cos(Omega t), sin(Omega t)),
    // which at T = 10 s is (-0.0839072, -0.0544021).
    //
    // Flipping the sign of the gyroscopic term (using +omega x I*omega)
    // reverses the rotation direction: omega_x is unchanged, because cos is
    // even, but omega_y lands at +0.0544 instead of -0.0544. That is a 0.109
    // discrepancy against a 1e-3 tolerance -- and it is exactly why the
    // previous magnitude-only |omega_y| > 1e-3 check could not see it.
    //
    // Tolerance budget: the amplitude growth measured above (+5.0e-4 relative)
    // displaces each component by at most 0.1*5.0e-4 = 5e-5, and the discrete
    // scheme's phase deficit is ~n*(Omega*h)^3/3 = 3.3e-8 rad -- negligible.
    // 1e-3 leaves ~20x margin over the dominant term.
    const double Omega = static_cast<double>(omega_z0) * (I_diag.z - I_diag.x) / I_diag.x;
    const double T = static_cast<double>(kSteps) * static_cast<double>(kSubstep);
    const double transverse0 = 0.1;

    EXPECT_NEAR(static_cast<double>(bodies[0].omega_body.x), transverse0 * std::cos(Omega * T), 1e-3);
    EXPECT_NEAR(static_cast<double>(bodies[0].omega_body.y), transverse0 * std::sin(Omega * T), 1e-3);
}

// ---------------------------------------------------------------------------
// Specific-force semantics. An accelerometer measures specific force --
// non-gravitational acceleration -- in the body frame it is bolted to. The
// two canonical readings are free fall (zero) and rest on a support (+1 g
// along the support normal, expressed in body axes).
// ---------------------------------------------------------------------------

TEST(Integrator, SpecificForceIsZeroInFreeFall) {
    const glm::vec3 gravity(0.0f, 0.0f, -9.81f);
    const WorldParams params = MakeParams(gravity);

    BodyState body = MakeUnitBody();
    // A non-identity attitude, so a bug that rotated gravity into the body
    // frame instead of capturing zero would show up on all three axes.
    body.orient = glm::normalize(glm::quat(0.6f, 0.1f, 0.4f, -0.3f));

    std::vector<BodyState> bodies{body};
    integrate_bodies(bodies, params, kH);

    // Nothing but gravity acts, so the accelerometer reads zero -- this is
    // the assertion that fails if gravity were folded in before the capture.
    ExpectVec3Near(bodies[0].specific_force, glm::vec3(0.0f), 1e-6f);

    // ...while the body is nonetheless accelerating: gravity did reach the
    // dynamics, it just did not reach the sensor.
    ExpectVec3Near(bodies[0].vel, gravity * kH, 1e-6f);
}

TEST(Integrator, SpecificForceAtRestOnSupportIsMinusGravityInBodyFrame) {
    const glm::vec3 gravity(0.0f, 0.0f, -9.81f);
    const WorldParams params = MakeParams(gravity);

    BodyState body = MakeUnitBody();
    // Roll +90 deg about body/world +x. This quaternion is body->world, and
    // a rotation of +90 deg about x maps y -> z and z -> -y. Its inverse
    // therefore maps world +z -> body +y, which is what pins the direction
    // of the frame transform: the reading must land on body +y. Had the
    // implementation rotated by `orient` instead of its conjugate, world +z
    // would map to body -y and this test would fail with a flipped sign.
    body.orient = glm::angleAxis(glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
    // The support's normal force, world frame: N = -m*g, i.e. +9.81 N along
    // world +z for this 1 kg body.
    body.force_acc = -body.mass * gravity;

    std::vector<BodyState> bodies{body};
    integrate_bodies(bodies, params, kH);

    // -g expressed in body axes: |g| along body +y, by the mapping above.
    ExpectVec3Near(bodies[0].specific_force, glm::vec3(0.0f, 9.81f, 0.0f), 1e-5f);

    // Cross-check the same claim frame-independently: the reading has
    // magnitude |g| and, rotated back to world, points along +z.
    EXPECT_NEAR(glm::length(bodies[0].specific_force), 9.81f, 1e-5f);
    ExpectVec3Near(bodies[0].orient * bodies[0].specific_force, -gravity, 1e-5f);

    // Support exactly cancels weight, so the body does not move.
    ExpectVec3Near(bodies[0].vel, glm::vec3(0.0f), 1e-6f);
    ExpectVec3Near(bodies[0].pos, glm::vec3(0.0f), 1e-6f);
}

TEST(Integrator, SpecificForceUsesPreUpdateOrientation) {
    // The capture must use the orientation at the START of the substep (the
    // attitude at the sample instant, and the same one the force passes used
    // to build force_acc), not the one step 6 produces. This body spins fast
    // enough that one substep rotates it a quarter turn, so the two choices
    // give visibly different answers on different axes.
    const WorldParams params = MakeParams(glm::vec3(0.0f));

    BodyState body = MakeUnitBody();
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // identity at the sample instant
    body.omega_body = glm::vec3(0.0f, 0.0f, glm::half_pi<float>() / kH);  // +90 deg per substep
    body.force_acc = glm::vec3(3.0f, 0.0f, 0.0f);                        // world +x, 3 N on 1 kg

    std::vector<BodyState> bodies{body};
    integrate_bodies(bodies, params, kH);

    // Pre-update orientation is the identity, so the body-frame reading is
    // the world-frame acceleration unchanged. With the POST-update
    // orientation (a +90 deg yaw) it would instead have been (0, -3, 0).
    ExpectVec3Near(bodies[0].specific_force, glm::vec3(3.0f, 0.0f, 0.0f), 1e-5f);

    // The quarter turn really did happen this substep -- otherwise the two
    // candidate answers would coincide and the test would prove nothing.
    const glm::quat expected_orient = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f));
    ExpectQuatNear(bodies[0].orient, expected_orient, 1e-5f);
}

// ---------------------------------------------------------------------------
// Spherical inertia: the gyroscopic term is identically zero (I*omega is
// parallel to omega), so the angular DOF decouples into three independent
// scalar integrations and the whole substep reduces to constant-acceleration
// translation plus fixed-axis rotation -- both with exact closed forms. A
// cheap anchor that the componentwise inverse-inertia multiply and the
// orientation update compose correctly.
//
// THIS IS ALSO THE TEST THAT CARRIES THE NON-UNIT CONSTANTS, deliberately.
// Every other value-asserting test here runs at mass == 1 and
// inv_inertia_diag == 1, where m, 1/m, I and 1/I are all the same number and
// two parity-critical mutations become invisible: `force_acc / mass` silently
// becoming `force_acc * mass`, and step 4c's `inv_I` silently becoming the
// guarded reciprocal `I_diag`. Choosing mass = 2.5 kg and
// inv_inertia_diag = 0.25 (i.e. I = 4 kg m^2) separates all four constants --
// the closed forms below simply carry them -- so both mutations move the
// asserted values by more than 4x their tolerances. Keep them non-unit and
// mutually distinct.
// ---------------------------------------------------------------------------

TEST(Integrator, SphericalInertiaReducesToTranslationPlusFixedAxisRotation) {
    constexpr int kSteps = 512;
    constexpr float kMass = 2.5f;     // kg -- deliberately not 1, and not 1/kInvInertia
    constexpr float kInvI = 0.25f;    // 1/(kg m^2) -- I = 4 kg m^2 on every axis
    constexpr float kTorqueZ = 2.0f;  // N m about body +z
    constexpr float kForceX = 3.0f;   // N along world +x
    const WorldParams params = MakeParams(glm::vec3(0.0f));

    BodyState body = MakeUnitBody();
    body.mass = kMass;
    body.inv_inertia_diag = glm::vec3(kInvI);  // still spherical: gyroscopic term is exactly zero

    std::vector<BodyState> bodies{body};
    for (int i = 0; i < kSteps; ++i) {
        // Re-applied every substep, because step 7 consumes the accumulators
        // -- which is exactly what the force passes do ahead of Integrate.
        bodies[0].force_acc = glm::vec3(kForceX, 0.0f, 0.0f);
        bodies[0].torque_acc = glm::vec3(0.0f, 0.0f, kTorqueZ);
        integrate_bodies(bodies, params, kH);
    }

    const double h = static_cast<double>(kH);
    const double n = static_cast<double>(kSteps);
    const double sum_k = n * (n + 1.0) / 2.0;  // sum_{k=1..n} k

    // Angular: omega_n = n*h*alpha with alpha = tau * (1/I) = 2.0 * 0.25 =
    // 0.5 rad/s^2, so omega_z ends at 0.25 rad/s. Every substep's rotation
    // shares the +z axis, and rotations about a common axis commute and add,
    // so the total attitude is a single rotation by
    // h * sum_k omega_k = alpha * h^2 * n(n+1)/2 = 0.06262 rad.
    //
    // The inverse inertia enters as a MULTIPLY here. If step 4c used the
    // guarded reciprocal I_diag (== 4) instead of inv_I (== 0.25), alpha
    // would be 8.0 and omega_z would end at 4.0 rather than 0.25 -- a 16x
    // error against a 1e-5 tolerance.
    const double alpha = static_cast<double>(kTorqueZ) * static_cast<double>(kInvI);
    const double expected_omega_z = n * h * alpha;
    const double expected_angle = alpha * h * h * sum_k;

    EXPECT_NEAR(static_cast<double>(bodies[0].omega_body.z), expected_omega_z, 1e-5);
    ExpectVec3Near(glm::vec3(bodies[0].omega_body.x, bodies[0].omega_body.y, 0.0f), glm::vec3(0.0f), 1e-6f);

    const glm::quat expected_orient =
        glm::angleAxis(static_cast<float>(expected_angle), glm::vec3(0.0f, 0.0f, 1.0f));
    ExpectQuatNear(bodies[0].orient, expected_orient, 1e-5f);

    // Translational: same discrete closed form as the ballistic case, with
    // the acceleration coming from force_acc rather than gravity -- and with
    // mass carried through, which is what makes step 1's DIVISION observable.
    // accel_x = 3.0 N / 2.5 kg = 1.2 m/s^2, so vel ends at 0.6 m/s and pos at
    // 0.1503 m. Had step 1 multiplied by mass instead of dividing, accel_x
    // would be 7.5 and pos would land at 0.939 m -- 6.25x off, against a
    // 1e-4 m tolerance.
    const double accel_x = static_cast<double>(kForceX) / static_cast<double>(kMass);
    EXPECT_NEAR(static_cast<double>(bodies[0].pos.x), accel_x * h * h * sum_k, 1e-4);
    EXPECT_NEAR(static_cast<double>(bodies[0].vel.x), accel_x * n * h, 1e-4);
}

// ---------------------------------------------------------------------------
// Determinism (charter P8, D11) and the flags contract.
// ---------------------------------------------------------------------------

TEST(Integrator, TwoIdenticalRunsAreByteIdentical) {
    const WorldParams params = MakeParams(glm::vec3(0.0f, 0.0f, -9.81f));

    // Built twice from the same code path rather than copied, so this checks
    // "the output is a function of the input", not "memcpy works".
    const auto make_world = []() {
        std::vector<BodyState> bodies(4);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const float k = static_cast<float>(i);
            bodies[i] = MakeUnitBody();
            bodies[i].pos = glm::vec3(k, -k, 2.0f * k);
            bodies[i].vel = glm::vec3(0.5f * k, 1.0f, -0.25f * k);
            bodies[i].orient = glm::normalize(glm::quat(0.6f + 0.05f * k, 0.1f, 0.4f - 0.03f * k, -0.3f));
            bodies[i].omega_body = glm::vec3(0.3f, -0.7f + 0.1f * k, 1.1f);
            bodies[i].mass = 1.0f + 0.25f * k;
            bodies[i].inv_inertia_diag = glm::vec3(1.0f, 0.5f, 1.0f / 3.0f);
        }
        // One tombstoned slot in the middle of the span: the skip path is
        // part of what has to be deterministic.
        bodies[2].flags = 0u;
        return bodies;
    };

    std::vector<BodyState> run_a = make_world();
    std::vector<BodyState> run_b = make_world();

    for (int i = 0; i < 200; ++i) {
        for (std::size_t b = 0; b < run_a.size(); ++b) {
            const glm::vec3 wrench(0.1f * static_cast<float>(b), 0.2f, -0.05f);
            run_a[b].force_acc = wrench;
            run_a[b].torque_acc = wrench * 0.5f;
            run_b[b].force_acc = wrench;
            run_b[b].torque_acc = wrench * 0.5f;
        }
        integrate_bodies(run_a, params, kH);
        integrate_bodies(run_b, params, kH);
    }

    // Whole-struct byte comparison, pads included. BodyState has no implicit
    // padding (layout.hpp asserts it) and both vectors were value-initialized,
    // so every byte here is meaningful and comparable.
    ASSERT_EQ(run_a.size(), run_b.size());
    EXPECT_EQ(std::memcmp(run_a.data(), run_b.data(), run_a.size() * sizeof(BodyState)), 0);

    // Guard against the comparison passing because nothing moved.
    EXPECT_GT(glm::length(run_a[0].pos), 1e-3f);
}

TEST(Integrator, InactiveBodiesAreLeftByteIdentical) {
    const WorldParams params = MakeParams(glm::vec3(0.0f, 0.0f, -9.81f));

    std::vector<BodyState> bodies(3);
    bodies[0] = MakeUnitBody();  // active: proves the pass ran at all

    // A zeroed/tombstoned slot, and a slot carrying some OTHER predicate bit
    // but not `active` -- the latter fails if the skip were written as a
    // "flags != 0" test rather than a bit test.
    bodies[1] = MakeUnitBody();
    bodies[1].flags = 0u;
    bodies[2] = MakeUnitBody();
    bodies[2].flags = 1u << 3;

    for (std::size_t i = 1; i < bodies.size(); ++i) {
        bodies[i].vel = glm::vec3(5.0f, 0.0f, 0.0f);
        bodies[i].omega_body = glm::vec3(0.0f, 0.0f, 2.0f);
        bodies[i].force_acc = glm::vec3(1.0f, 2.0f, 3.0f);
        bodies[i].torque_acc = glm::vec3(0.4f, 0.5f, 0.6f);
    }

    const BodyState before_zero_flags = bodies[1];
    const BodyState before_other_flag = bodies[2];

    integrate_bodies(bodies, params, kH);

    // Untouched means untouched: pose, velocities, AND the accumulators,
    // which step 7 must not have cleared for a skipped body.
    EXPECT_EQ(std::memcmp(&bodies[1], &before_zero_flags, sizeof(BodyState)), 0);
    EXPECT_EQ(std::memcmp(&bodies[2], &before_other_flag, sizeof(BodyState)), 0);

    // The active neighbour did move, so the span really was traversed.
    ExpectVec3Near(bodies[0].vel, glm::vec3(0.0f, 0.0f, -9.81f) * kH, 1e-6f);
}

TEST(Integrator, AccumulatorsAreClearedForActiveBodies) {
    const WorldParams params = MakeParams(glm::vec3(0.0f));

    BodyState body = MakeUnitBody();
    body.force_acc = glm::vec3(1.0f, 2.0f, 3.0f);
    body.torque_acc = glm::vec3(0.4f, 0.5f, 0.6f);

    std::vector<BodyState> bodies{body};
    integrate_bodies(bodies, params, kH);

    ExpectVec3Near(bodies[0].force_acc, glm::vec3(0.0f), 0.0f);
    ExpectVec3Near(bodies[0].torque_acc, glm::vec3(0.0f), 0.0f);

    // The wrench was consumed before being cleared, not instead of.
    ExpectVec3Near(bodies[0].vel, glm::vec3(1.0f, 2.0f, 3.0f) * kH, 1e-6f);
    ExpectVec3Near(bodies[0].omega_body, glm::vec3(0.4f, 0.5f, 0.6f) * kH, 1e-6f);
}

TEST(Integrator, ZeroInverseInertiaFreezesRotationWithoutPoisoningState) {
    // inv_inertia_diag == 0 on every axis means infinite inertia: a
    // rotation-locked body. The guarded reciprocal in step 4a must keep this
    // finite -- an unguarded 1/0 would make I_body infinite, the gyroscopic
    // term infinite, and 0 * inf a NaN that silently destroys the world.
    const WorldParams params = MakeParams(glm::vec3(0.0f, 0.0f, -9.81f));

    BodyState body = MakeUnitBody();
    body.inv_inertia_diag = glm::vec3(0.0f);
    body.omega_body = glm::vec3(0.3f, -0.7f, 1.1f);
    body.torque_acc = glm::vec3(5.0f, -2.0f, 8.0f);

    const glm::vec3 omega0 = body.omega_body;

    std::vector<BodyState> bodies{body};
    for (int i = 0; i < 100; ++i) {
        bodies[0].torque_acc = glm::vec3(5.0f, -2.0f, 8.0f);
        integrate_bodies(bodies, params, kH);
    }

    ExpectVec3Near(bodies[0].omega_body, omega0, 0.0f);  // exactly frozen
    EXPECT_FALSE(std::isnan(bodies[0].orient.w));
    EXPECT_FALSE(std::isnan(bodies[0].pos.z));
    EXPECT_NEAR(glm::length(bodies[0].orient), 1.0f, 1e-5f);

    // Translation is unaffected by the locked rotation.
    EXPECT_LT(bodies[0].pos.z, 0.0f);
}
