#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "core/rng.hpp"
#include "physics/integrator.hpp"
#include "sensors/imu.hpp"
#include "sensors/rings.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/snapshot.hpp"
#include "testing/replay.hpp"
#include "world/builder.hpp"

// ---------------------------------------------------------------------------
// Task 19 -- IMU synthesis and the tick-stamped output rings.
//
// FIVE BURDENS OF PROOF:
//
//   * THE PHYSICS IS THE RIGHT PHYSICS, AND THE SIGNS ARE THE RIGHT SIGNS.
//     Checked against closed forms in configurations chosen so the answer is
//     EXACT in fp32 -- a body held static reads +g OPPOSITE gravity, a body in
//     free fall reads zero, a spinning body's gyro reads omega, an off-axis
//     mount adds exactly omega x (omega x r). No tolerance is hiding anything.
//
//   * THE ERROR MODEL HAS THE CONFIGURED MAGNITUDE. Checked by ALLAN VARIANCE
//     at the shortest cluster -- the standard IMU-characterization statistic,
//     and the one that separates white noise from a bias walk: the first
//     difference annihilates a slowly varying bias, so AVAR(1) measures sigma_a
//     alone; run again with sigma_a = 0 and the same statistic measures
//     sigma_ba alone.
//
//   * THE RATE DIVIDER DIVIDES. Checked by counting: N substeps at divider k
//     produce exactly N/k samples, and the tick stamps land where the "fire on
//     the substep that completes the period" rule says they must.
//
//   * THE RING IS A RING (editor tech spec TA5). Checked by overrunning it:
//     poll-since returns only the resident tail, reports what was dropped, and
//     never hands back a sample the wrap has overwritten.
//
//   * THE WHOLE THING SURVIVES A SNAPSHOT. Checked with the replay harness:
//     digest equality across a restore into a FRESH Simulation, plus byte
//     equality of the samples the resumed run goes on to produce.
//
// STATISTICS AND fp64. The Allan-variance helper accumulates in double. That is
// not a violation of the engine's fp32 rule -- which binds engine state and
// engine math -- it is a test computing a statistic ABOUT fp32 data, where a
// naive fp32 accumulation over 20,000 terms would be the largest error in the
// measurement.
// ---------------------------------------------------------------------------

namespace {

using spade::BodyRef;
using spade::BodySpawn;
using spade::Capacities;
using spade::Environment;
using spade::ImuPoll;
using spade::ImuSensorRef;
using spade::ImuSensorSpawn;
using spade::Simulation;
using spade::SnapshotBlob;
using spade::Tick;
using spade::TurbulenceLevel;
using spade::WorldBuilder;
using spade::WorldInstanceDesc;
using spade::WorldSetDesc;
using spade::sensors::ImuSample;
using spade::sensors::kRingDepth;
using spade::sensors::SampleIndex;
using spade::testing::Scenario;

// GoogleTest plumbing for spade::Result -- prints the Error's context on
// failure. Same shape as test_determinism.cpp's; macros are per-TU, so the two
// definitions do not collide.
template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code)
                                       << "] " << r.error().context;
}

#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

// The error code of a Result that is EXPECTED to have failed, with -1 standing
// for "it succeeded" -- calling .error() on a Result holding a value is UB, so
// a negative test that accidentally passes must not reach for it.
template <class T>
[[nodiscard]] int code_of(const spade::Result<T>& r) {
    return r ? -1 : static_cast<int>(r.error().code);
}

[[nodiscard]] constexpr int code(spade::Code c) { return static_cast<int>(c); }

// The engine is Y-UP: world/builder.hpp's Environment defaults gravity to
// (0, -9.80665, 0). Spelled here because every sign assertion below is stated
// relative to it.
constexpr float kG = 9.80665f;

// ---------------------------------------------------------------------------
// World construction
// ---------------------------------------------------------------------------

[[nodiscard]] Environment default_environment() {
    Environment env;
    env.gravity = glm::vec3(0.0f, -kG, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    return env;
}

[[nodiscard]] Capacities capacities(uint32_t bodies, uint32_t elements, uint32_t sensors) {
    Capacities caps;
    caps.bodies = bodies;
    caps.force_elements = elements;
    caps.sensors = sensors;
    caps.contacts = 1;  // declared, unused: contacts are not an arena array in v1
    return caps;
}

[[nodiscard]] spade::physics::ContactParams no_contacts() {
    spade::physics::ContactParams c;
    c.restitution_e = 0.0f;
    c.friction_mu = 0.0f;
    c.proxy_radius = 0.0f;
    return c;
}

[[nodiscard]] spade::physics::GridParams unit_grid() {
    spade::physics::GridParams g;
    g.cell_size = 1.0f;
    return g;
}

// A world with NO GEOMETRY, no turbulence and no contact response, so the only
// things that can move a body are gravity and an explicit wrench -- and the
// only stochastic system in it is the IMU. Every physics assertion below
// depends on that isolation.
[[nodiscard]] spade::Result<WorldSetDesc> void_world_set(uint32_t bodies, uint32_t sensors,
                                                         uint64_t seed) {
    const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                      .name("void")
                                                      .environment(default_environment())
                                                      .capacities(capacities(bodies, 1, sensors))
                                                      .build();
    if (!world) return std::unexpected(world.error());

    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = seed;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts = no_contacts();
    instance.grid = unit_grid();
    return WorldSetDesc{{instance}};
}

// A unit-mass, unit-inertia body at the origin. UNIT INERTIA IS LOAD-BEARING in
// the spinning tests: integrate_bodies() forms the gyroscopic torque as
// omega x (I*omega), and with I the identity that product is omega x omega,
// whose every component is `a*b - b*a` on IDENTICAL fp32 operands and therefore
// EXACTLY zero. So omega is bit-stable across the run and the gyro assertions
// can be tight instead of drifting.
//
// Unit mass matters for the same reason on the linear side: integrate_bodies()
// divides force_acc by mass, and division by 1.0f is exact.
[[nodiscard]] BodySpawn unit_body() {
    BodySpawn body;
    body.pos = glm::vec3(0.0f, 100.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    return body;
}

// The wrench that holds a unit-mass body exactly stationary against gravity:
// force_acc becomes m*|g| upward, so accel_ext + gravity is exactly zero and
// neither vel nor pos changes. A body resting on an SDF reads +g as well, since
// PHY-7 (ImuContact.*, sensors/imu.hpp section 5); this wrench keeps the body
// off every contact, so its reading is force_acc's alone.
[[nodiscard]] glm::vec3 antigravity_force(float mass) { return glm::vec3(0.0f, mass * kG, 0.0f); }

// A rotation of +90 degrees about the body's Z axis, as a (w, x, y, z)
// quaternion. Used as a MOUNT pose (mount -> body), so reading a body-frame
// vector v in the mount frame applies the inverse: Rz(-90) v = (v.y, -v.x, v.z),
// which is written out by hand at the assertion sites rather than recomputed
// with glm -- an expectation derived from the implementation proves nothing.
[[nodiscard]] glm::quat mount_yaw90() {
    const float s = std::sqrt(0.5f);
    return glm::quat(s, 0.0f, 0.0f, s);
}

// ---------------------------------------------------------------------------
// Sample collection
// ---------------------------------------------------------------------------

// Steps `sim` `steps` times, polling `ref` every `block` steps and appending
// every sample. FAILS THE TEST IF ANYTHING WAS DROPPED, which is what makes the
// collected vector a complete record rather than a sampled one -- callers pick
// a block small enough that the ring cannot wrap between polls.
//
// `pre_step` runs before each step and is handed the tick about to execute, so
// a caller can drive an input script.
template <class PreStep>
void collect_samples(Simulation& sim, ImuSensorRef ref, uint64_t steps, uint64_t block,
                     std::vector<ImuSample>& out, PreStep pre_step) {
    ASSERT_GT(block, 0u);
    std::vector<ImuSample> buffer(kRingDepth);
    SampleIndex since = 0;
    uint64_t done = 0;
    while (done < steps) {
        const uint64_t n = std::min(block, steps - done);
        for (uint64_t i = 0; i < n; ++i) {
            pre_step(sim, done + i);
            ASSERT_OK(sim.step(1));
        }
        done += n;

        const spade::Result<ImuPoll> poll = sim.poll_imu(ref, since, buffer);
        ASSERT_OK(poll);
        ASSERT_EQ(poll->dropped, 0u) << "the poll block outran the ring: raise the poll rate";
        out.insert(out.end(), poll->samples.begin(), poll->samples.end());
        since = poll->next_since;
    }
}

// The same, with no input script.
void collect_samples(Simulation& sim, ImuSensorRef ref, uint64_t steps, uint64_t block,
                     std::vector<ImuSample>& out) {
    collect_samples(sim, ref, steps, block, out, [](Simulation&, uint64_t) {});
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

// The Allan variance at the shortest cluster (m = 1 sample):
//
//     AVAR(1) = 0.5 * mean((x[k+1] - x[k])^2)
//
// Two exact consequences, and they are what the two noise tests key on:
//
//   * for x = constant + white(sigma):        AVAR(1) = sigma^2
//   * for x = constant + randomwalk(sigma):   AVAR(1) = sigma^2 / 2
//
// The first difference is what makes both true: it kills any constant offset
// (the true signal, and a bias that has already accumulated), leaving only the
// increments.
[[nodiscard]] double allan_variance_1(const std::vector<float>& x) {
    if (x.size() < 2) return 0.0;
    double acc = 0.0;
    for (std::size_t k = 1; k < x.size(); ++k) {
        const double d = static_cast<double>(x[k]) - static_cast<double>(x[k - 1]);
        acc += d * d;
    }
    return 0.5 * acc / static_cast<double>(x.size() - 1);
}

[[nodiscard]] double mean_of(const std::vector<float>& x) {
    double acc = 0.0;
    for (const float v : x) acc += static_cast<double>(v);
    return x.empty() ? 0.0 : acc / static_cast<double>(x.size());
}

enum class Axis : int { x = 0, y = 1, z = 2 };

[[nodiscard]] std::vector<float> accel_axis(const std::vector<ImuSample>& samples, Axis axis) {
    std::vector<float> out;
    out.reserve(samples.size());
    for (const ImuSample& s : samples) out.push_back(s.accel[static_cast<int>(axis)]);
    return out;
}

[[nodiscard]] std::vector<float> gyro_axis(const std::vector<ImuSample>& samples, Axis axis) {
    std::vector<float> out;
    out.reserve(samples.size());
    for (const ImuSample& s : samples) out.push_back(s.gyro[static_cast<int>(axis)]);
    return out;
}

// How long the two noise tests run. 20,000 samples puts the relative standard
// error of an AVAR(1) estimate at roughly 2/sqrt(N) ~ 1.4% (the first
// differences carry a lag-1 correlation of -0.5, which halves the effective
// independent count), i.e. under 1% on the DEVIATION -- comfortably inside the
// brief's 10% band.
//
// AND THE RESULT IS NOT A RANDOM VARIABLE ANYWAY: the seed is fixed and the
// engine is deterministic, so these tests pass or fail identically on every
// run. The 10% band is headroom for the estimator's bias and for a future
// re-seed, not for flakiness.
constexpr uint64_t kNoiseSamples = 20000;
constexpr uint64_t kPollBlock = 32;  // < kRingDepth, so nothing is ever dropped

}  // namespace

// ===========================================================================
// 1. THE SIGN CONVENTION
// ===========================================================================

// An accelerometer measures SPECIFIC FORCE, so a body held stationary in
// gravity reads +g along the axis OPPOSITE gravity -- (0, +9.80665, 0) in the
// body frame of a level body in this Y-up engine. (Written in the Z-up spelling
// the spec text uses, that is the familiar "(0, 0, +g)"; the axis differs, the
// sign is the contract.) The gyro reads zero.
//
// The second sensor carries a +90-degree yaw MOUNT, so the same body-frame
// (0, +g, 0) must arrive as (+g, 0, 0) in ITS frame -- which pins the direction
// of the mount rotation (conjugate of mount->body), the one thing a COM-mounted
// identity sensor cannot distinguish.
TEST(Imu, StaticLevelBodyReadsPlusGOppositeGravityAndZeroRotation) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 2, 0x5A11FEEDULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    const spade::Result<ImuSensorRef> level = sim->add_imu_sensor(*body, ImuSensorSpawn{});
    ASSERT_OK(level);

    ImuSensorSpawn yawed;
    yawed.mount_orient = mount_yaw90();
    const spade::Result<ImuSensorRef> rotated = sim->add_imu_sensor(*body, yawed);
    ASSERT_OK(rotated);
    ASSERT_OK(sim->flush_structural());

    const auto hold = [&body](Simulation& s, uint64_t) {
        EXPECT_OK(s.apply_wrench(*body, antigravity_force(1.0f), glm::vec3(0.0f)));
    };

    std::vector<ImuSample> level_samples;
    std::vector<ImuSample> rotated_samples;
    // Both sensors are driven by the same run; collect_samples() steps, so the
    // second call would step a second time. Step once and poll both instead.
    std::vector<ImuSample> buffer(kRingDepth);
    constexpr uint64_t kSteps = 50;
    for (uint64_t t = 0; t < kSteps; ++t) {
        hold(*sim, t);
        ASSERT_OK(sim->step(1));
    }
    {
        const spade::Result<ImuPoll> poll = sim->poll_imu(*level, 0, buffer);
        ASSERT_OK(poll);
        level_samples.assign(poll->samples.begin(), poll->samples.end());
    }
    {
        const spade::Result<ImuPoll> poll = sim->poll_imu(*rotated, 0, buffer);
        ASSERT_OK(poll);
        rotated_samples.assign(poll->samples.begin(), poll->samples.end());
    }

    ASSERT_EQ(level_samples.size(), kSteps);
    ASSERT_EQ(rotated_samples.size(), kSteps);

    for (std::size_t i = 0; i < level_samples.size(); ++i) {
        const ImuSample& s = level_samples[i];
        EXPECT_NEAR(s.accel.x, 0.0f, 1e-6f) << "sample " << i;
        EXPECT_NEAR(s.accel.y, kG, 1e-5f) << "sample " << i << ": a static accelerometer must read "
                                             "+g OPPOSITE gravity, not -g and not 0";
        EXPECT_NEAR(s.accel.z, 0.0f, 1e-6f) << "sample " << i;
        EXPECT_NEAR(s.gyro.x, 0.0f, 1e-6f) << "sample " << i;
        EXPECT_NEAR(s.gyro.y, 0.0f, 1e-6f) << "sample " << i;
        EXPECT_NEAR(s.gyro.z, 0.0f, 1e-6f) << "sample " << i;
        EXPECT_EQ(s.index, static_cast<SampleIndex>(i + 1)) << "indices start at 1 and never skip";
        EXPECT_EQ(s.tick, i) << "one substep per step, so sample i+1 belongs to tick i";
    }

    // Rz(-90) applied to (0, +g, 0) is (+g, 0, 0) -- written by hand, not
    // recomputed with the quaternion the implementation uses.
    for (const ImuSample& s : rotated_samples) {
        EXPECT_NEAR(s.accel.x, kG, 1e-5f) << "the mount rotation went the wrong way (or not at all)";
        EXPECT_NEAR(s.accel.y, 0.0f, 1e-5f);
        EXPECT_NEAR(s.accel.z, 0.0f, 1e-6f);
    }

    // ...and the body really did stay static, so the readings above are a
    // statement about the sensor rather than about a drifting body.
    const spade::Result<const spade::BodyState*> state = sim->body(*body);
    ASSERT_OK(state);
    EXPECT_EQ((*state)->vel, glm::vec3(0.0f));
    EXPECT_EQ((*state)->pos, unit_body().pos);
}

// The other half of the convention: an accelerometer in FREE FALL reads ZERO on
// every axis, not -g. This is the case that would fail if the pass reconstructed
// specific force by subtracting gravity instead of reading the integrator's
// capture.
TEST(Imu, FreeFallingBodyReadsZeroSpecificForce) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, 0xFA11ULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);
    const spade::Result<ImuSensorRef> imu = sim->add_imu_sensor(*body, ImuSensorSpawn{});
    ASSERT_OK(imu);
    ASSERT_OK(sim->flush_structural());

    std::vector<ImuSample> samples;
    collect_samples(*sim, *imu, 40, kPollBlock, samples);
    ASSERT_EQ(samples.size(), 40u);

    for (const ImuSample& s : samples) {
        EXPECT_EQ(s.accel, glm::vec3(0.0f)) << "free fall must read exactly zero, not -g";
        EXPECT_EQ(s.gyro, glm::vec3(0.0f));
    }

    // The body really was falling -- otherwise "reads zero" would be trivially
    // true of a body that never moved.
    const spade::Result<const spade::BodyState*> state = sim->body(*body);
    ASSERT_OK(state);
    EXPECT_LT((*state)->vel.y, -0.3f);
}

// ===========================================================================
// 2. THE GYRO, AND THE MOUNT ROTATION
// ===========================================================================

TEST(Imu, SpinningBodyGyroMeasuresBodyAngularVelocityInTheMountFrame) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 2, 0x5A1EULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    const glm::vec3 omega(0.7f, -1.3f, 2.1f);
    BodySpawn body = unit_body();
    body.omega_body = omega;
    const spade::Result<BodyRef> ref = sim->spawn(0, body);
    ASSERT_OK(ref);

    const spade::Result<ImuSensorRef> aligned = sim->add_imu_sensor(*ref, ImuSensorSpawn{});
    ASSERT_OK(aligned);
    ImuSensorSpawn yawed;
    yawed.mount_orient = mount_yaw90();
    const spade::Result<ImuSensorRef> rotated = sim->add_imu_sensor(*ref, yawed);
    ASSERT_OK(rotated);
    ASSERT_OK(sim->flush_structural());

    constexpr uint64_t kSteps = 40;
    ASSERT_OK(sim->step(kSteps));

    std::vector<ImuSample> buffer(kRingDepth);
    const spade::Result<ImuPoll> aligned_poll = sim->poll_imu(*aligned, 0, buffer);
    ASSERT_OK(aligned_poll);
    ASSERT_EQ(aligned_poll->samples.size(), kSteps);
    for (const ImuSample& s : aligned_poll->samples) {
        // Unit inertia makes omega bit-stable (see unit_body()), so this is an
        // equality up to the quaternion rotation's own rounding.
        EXPECT_NEAR(s.gyro.x, omega.x, 1e-6f);
        EXPECT_NEAR(s.gyro.y, omega.y, 1e-6f);
        EXPECT_NEAR(s.gyro.z, omega.z, 1e-6f);
        // A COM mount on a body with no external force still reads zero accel:
        // the centripetal term needs a lever arm.
        EXPECT_NEAR(glm::length(s.accel), 0.0f, 1e-6f);
    }

    std::vector<ImuSample> rotated_out(kRingDepth);
    const spade::Result<ImuPoll> rotated_poll = sim->poll_imu(*rotated, 0, rotated_out);
    ASSERT_OK(rotated_poll);
    ASSERT_EQ(rotated_poll->samples.size(), kSteps);
    for (const ImuSample& s : rotated_poll->samples) {
        // Rz(-90) v = (v.y, -v.x, v.z), by hand.
        EXPECT_NEAR(s.gyro.x, omega.y, 1e-6f);
        EXPECT_NEAR(s.gyro.y, -omega.x, 1e-6f);
        EXPECT_NEAR(s.gyro.z, omega.z, 1e-6f);
    }
}

// ===========================================================================
// 2b. WHICH INSTANT EACH CHANNEL IS TAKEN AT (sensors/imu.hpp section 3)
//
// One ImuSample MIXES INSTANTS: accel is a substep-START quantity (the
// integrator captures specific force against the pre-update orientation, on
// purpose) while gyro is a substep-END one (omega_body has already absorbed
// this substep's angular acceleration by the time this pass runs). The gap is
// exactly alpha*h. Section 3 of imu.hpp states the bound and why it is accepted
// rather than fixed.
//
// THESE TWO TESTS EXIST BECAUSE NOTHING ELSE IN THIS FILE CAN CATCH A CHANGE TO
// EITHER INSTANT. Every other gyro assertion above runs at ZERO TORQUE with unit
// inertia -- deliberately, to get bit-exact expectations -- and that is exactly
// the configuration in which pre- and post-update omega are the same number.
// Likewise every other accel assertion runs on a body that is either not
// rotating or has no applied force, so the pre- and post-update orientations
// agree. Each test below breaks one of those degeneracies so that the timing
// choice is a thing the suite can see.
// ===========================================================================

// NONZERO TORQUE, so the two candidate omegas differ by exactly h*I^-1*tau.
//
// The expectation is BIT-EXACT, which is worth spelling out because it is what
// makes "which of the two" an equality rather than a judgement call. With unit
// inertia I is the identity, so math::gyroscopic_torque() is -cross(w, I*w) =
// -cross(w, w), whose every component is `a*b - b*a` on identical fp32 operands
// and therefore exactly zero. The integrator's step 4c is then
// omega += (inv_I * torque_total) * h, which for tau = (0,0,1) and inv_I = 1 is
// exactly omega.z += h -- the same fp32 recurrence the loop below runs.
TEST(Imu, GyroReadsThePostUpdateOmegaOfItsSubstep) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, 0x71C5ULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    // omega starts at exactly zero, so every rad/s the gyro reports came from
    // the torque below and from nothing else.
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);
    const spade::Result<ImuSensorRef> imu = sim->add_imu_sensor(*body, ImuSensorSpawn{});
    ASSERT_OK(imu);
    ASSERT_OK(sim->flush_structural());

    constexpr uint64_t kSteps = 20;
    for (uint64_t t = 0; t < kSteps; ++t) {
        // BODY-frame torque, per layout.hpp's accumulator ruling.
        ASSERT_OK(sim->apply_wrench(*body, glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
        ASSERT_OK(sim->step(1));
    }

    std::vector<ImuSample> out(kRingDepth);
    const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, 0, out);
    ASSERT_OK(poll);
    ASSERT_EQ(poll->samples.size(), kSteps);

    const float h = sim->substep_h();
    float post = 0.0f;
    for (std::size_t i = 0; i < poll->samples.size(); ++i) {
        const float pre = post;   // omega at the START of this substep
        post = post + h;          // omega at its END -- the integrator's own update
        ASSERT_NE(pre, post) << "the test setup stopped discriminating the two instants";

        const ImuSample& s = poll->samples[i];
        EXPECT_EQ(s.gyro.z, post)
            << "sample " << (i + 1)
            << ": the gyro must report the SUBSTEP-END omega (imu.hpp section 3). "
               "If this now reads " << pre << " the timing changed -- update section 3 "
               "and the accel's instant to match, do not just retune this number.";
        // Stated as its own assertion so a failure says WHICH value it read.
        EXPECT_NE(s.gyro.z, pre) << "sample " << (i + 1) << ": gyro read the pre-update omega";

        // No force was applied, so the accel channel stays silent and cannot be
        // what is carrying this test.
        EXPECT_EQ(s.accel, glm::vec3(0.0f));
    }

    // The torque really did spin the body up over the run, so the assertions
    // above were made against a moving target rather than a rounding artefact.
    EXPECT_GT(post, 10.0f * h);
}

// A SPINNING BODY UNDER A WORLD-FRAME FORCE, so the pre- and post-update
// orientations disagree about where that force points in the body frame.
//
// ONE STEP ONLY, from an identity attitude: the substep-START orientation is
// therefore exactly identity and the reading is the world force verbatim, with
// accel.z exactly zero. Had the pass used the POST-update orientation, the
// body would have turned omega*h = 0.01 rad about Y first and the reading would
// be about (3*cos 0.01, 0, 3*sin 0.01) -- an accel.z near 0.03, four orders of
// magnitude outside the tolerance below.
TEST(Imu, AccelUsesTheSubstepStartOrientation) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, 0xA1157ULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    BodySpawn spawn = unit_body();
    spawn.omega_body = glm::vec3(0.0f, 10.0f, 0.0f);  // spinning about body Y
    const spade::Result<BodyRef> body = sim->spawn(0, spawn);
    ASSERT_OK(body);
    const spade::Result<ImuSensorRef> imu = sim->add_imu_sensor(*body, ImuSensorSpawn{});
    ASSERT_OK(imu);
    ASSERT_OK(sim->flush_structural());

    ASSERT_OK(sim->apply_wrench(*body, glm::vec3(3.0f, 0.0f, 0.0f), glm::vec3(0.0f)));
    ASSERT_OK(sim->step(1));

    std::vector<ImuSample> out(kRingDepth);
    const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, 0, out);
    ASSERT_OK(poll);
    ASSERT_EQ(poll->samples.size(), 1u);
    const ImuSample& s = poll->samples[0];

    EXPECT_FLOAT_EQ(s.accel.x, 3.0f);
    EXPECT_NEAR(s.accel.y, 0.0f, 1e-6f);
    EXPECT_NEAR(s.accel.z, 0.0f, 1e-6f)
        << "the accelerometer used the POST-update orientation: one substep of rotation "
           "is being smeared into every sample (integrator.cpp step 2; imu.hpp section 3)";

    // No torque, so omega is untouched and the gyro's own instant is not what
    // this test is measuring.
    EXPECT_FLOAT_EQ(s.gyro.y, 10.0f);

    // THE BODY REALLY DID ROTATE during the step -- without this the test would
    // pass just as happily on a body that never turned, and would be pinning
    // nothing at all.
    const spade::Result<const spade::BodyState*> state = sim->body(*body);
    ASSERT_OK(state);
    EXPECT_GT(std::abs((*state)->orient.y), 1e-4f)
        << "the body did not rotate, so pre- and post-update orientations agree "
           "and this test cannot discriminate them";
}

// ===========================================================================
// 3. THE LEVER ARM
// ===========================================================================

// An off-COM mount on a spinning body reads the centripetal term
// omega x (omega x r). With omega = (0, 0, 3) and r = (0.5, 0, 0) that is
// exactly (-4.5, 0, 0) -- INWARD, toward the spin axis, which is the sign the
// specific-force formulation gives (the proof mass is being pulled around the
// circle, and the accelerometer reads the force doing the pulling).
//
// Every quantity here is exact in fp32: omega has a single non-zero component
// so both cross products are exact, and 3*3*0.5 = 4.5 is representable.
//
// The COM sensor on the SAME body reading exactly zero is the control: it shows
// the term comes from the lever arm and not from anywhere else.
TEST(Imu, OffAxisMountAddsTheCentripetalTermAndACoMMountDoesNot) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 2, 0xCE47ULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    BodySpawn body = unit_body();
    body.omega_body = glm::vec3(0.0f, 0.0f, 3.0f);
    const spade::Result<BodyRef> ref = sim->spawn(0, body);
    ASSERT_OK(ref);

    ImuSensorSpawn offset;
    offset.mount_pos = glm::vec3(0.5f, 0.0f, 0.0f);
    const spade::Result<ImuSensorRef> lever = sim->add_imu_sensor(*ref, offset);
    ASSERT_OK(lever);
    const spade::Result<ImuSensorRef> centre = sim->add_imu_sensor(*ref, ImuSensorSpawn{});
    ASSERT_OK(centre);
    ASSERT_OK(sim->flush_structural());

    constexpr uint64_t kSteps = 20;
    ASSERT_OK(sim->step(kSteps));

    std::vector<ImuSample> buffer(kRingDepth);
    const spade::Result<ImuPoll> lever_poll = sim->poll_imu(*lever, 0, buffer);
    ASSERT_OK(lever_poll);
    ASSERT_EQ(lever_poll->samples.size(), kSteps);
    for (const ImuSample& s : lever_poll->samples) {
        EXPECT_FLOAT_EQ(s.accel.x, -4.5f) << "centripetal term missing, or pointing outward";
        EXPECT_FLOAT_EQ(s.accel.y, 0.0f);
        EXPECT_FLOAT_EQ(s.accel.z, 0.0f);
        EXPECT_FLOAT_EQ(s.gyro.z, 3.0f) << "a lever arm must not change the gyro";
    }

    std::vector<ImuSample> centre_out(kRingDepth);
    const spade::Result<ImuPoll> centre_poll = sim->poll_imu(*centre, 0, centre_out);
    ASSERT_OK(centre_poll);
    ASSERT_EQ(centre_poll->samples.size(), kSteps);
    for (const ImuSample& s : centre_poll->samples) {
        EXPECT_EQ(s.accel, glm::vec3(0.0f)) << "a COM mount has no lever arm and must read zero";
    }
}

// ===========================================================================
// 4. THE NOISE MODEL
// ===========================================================================

// White noise only (sigma_ba = sigma_bg = 0): AVAR(1) == sigma^2 on every axis,
// and the sample mean still sits on the true value.
TEST(Imu, WhiteNoiseAllanVarianceMatchesTheConfiguredSigma) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, 0x9013E5EULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    ImuSensorSpawn spec;
    spec.sigma_a = 0.05f;   // m/s^2 per sample
    spec.sigma_g = 0.01f;   // rad/s per sample
    const spade::Result<ImuSensorRef> imu = sim->add_imu_sensor(*body, spec);
    ASSERT_OK(imu);
    ASSERT_OK(sim->flush_structural());

    std::vector<ImuSample> samples;
    collect_samples(*sim, *imu, kNoiseSamples, kPollBlock, samples,
                    [&body](Simulation& s, uint64_t) {
                        EXPECT_OK(s.apply_wrench(*body, antigravity_force(1.0f), glm::vec3(0.0f)));
                    });
    ASSERT_EQ(samples.size(), kNoiseSamples);

    const Axis axes[3] = {Axis::x, Axis::y, Axis::z};
    for (const Axis axis : axes) {
        const std::vector<float> a = accel_axis(samples, axis);
        const std::vector<float> g = gyro_axis(samples, axis);
        EXPECT_NEAR(std::sqrt(allan_variance_1(a)), static_cast<double>(spec.sigma_a),
                    0.1 * static_cast<double>(spec.sigma_a))
            << "accel axis " << static_cast<int>(axis);
        EXPECT_NEAR(std::sqrt(allan_variance_1(g)), static_cast<double>(spec.sigma_g),
                    0.1 * static_cast<double>(spec.sigma_g))
            << "gyro axis " << static_cast<int>(axis);
    }

    // Zero-mean noise: the estimate of the mean has standard error
    // sigma/sqrt(N) ~ 3.5e-4, so a 5-sigma band is 2e-3 and catches a noise
    // model that is accidentally one-sided.
    EXPECT_NEAR(mean_of(accel_axis(samples, Axis::y)), static_cast<double>(kG), 2e-3);
    EXPECT_NEAR(mean_of(accel_axis(samples, Axis::x)), 0.0, 2e-3);
    EXPECT_NEAR(mean_of(gyro_axis(samples, Axis::z)), 0.0, 5e-4);

    // The bias states never moved, because their sigmas are zero. A pass that
    // walked the bias regardless would still pass the AVAR check above (a
    // random walk contributes to AVAR(1) too), so this is the assertion that
    // separates the two channels.
    const spade::Result<const spade::sensors::ImuSensorRow*> row = sim->imu_sensor(*imu);
    ASSERT_OK(row);
    EXPECT_EQ((*row)->bias_a, glm::vec3(0.0f));
    EXPECT_EQ((*row)->bias_g, glm::vec3(0.0f));
}

// Bias random walk only (sigma_a = sigma_g = 0). The first difference of a pure
// random walk is its own increment, so AVAR(1) == sigma_ba^2 / 2 -- a DIFFERENT
// relation from the white-noise case, which is what makes this test able to
// tell the two error sources apart rather than just re-measuring "some noise".
TEST(Imu, BiasRandomWalkAllanVarianceMatchesTheConfiguredSigma) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, 0xB1A5DULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    ImuSensorSpawn spec;
    spec.sigma_ba = 0.002f;  // m/s^2 per sample
    spec.sigma_bg = 0.001f;  // rad/s per sample
    const spade::Result<ImuSensorRef> imu = sim->add_imu_sensor(*body, spec);
    ASSERT_OK(imu);
    ASSERT_OK(sim->flush_structural());

    std::vector<ImuSample> samples;
    collect_samples(*sim, *imu, kNoiseSamples, kPollBlock, samples,
                    [&body](Simulation& s, uint64_t) {
                        EXPECT_OK(s.apply_wrench(*body, antigravity_force(1.0f), glm::vec3(0.0f)));
                    });
    ASSERT_EQ(samples.size(), kNoiseSamples);

    const Axis axes[3] = {Axis::x, Axis::y, Axis::z};
    for (const Axis axis : axes) {
        const std::vector<float> a = accel_axis(samples, axis);
        const std::vector<float> g = gyro_axis(samples, axis);
        // sqrt(2 * AVAR(1)) recovers the walk's per-sample step sigma.
        EXPECT_NEAR(std::sqrt(2.0 * allan_variance_1(a)), static_cast<double>(spec.sigma_ba),
                    0.1 * static_cast<double>(spec.sigma_ba))
            << "accel axis " << static_cast<int>(axis);
        EXPECT_NEAR(std::sqrt(2.0 * allan_variance_1(g)), static_cast<double>(spec.sigma_bg),
                    0.1 * static_cast<double>(spec.sigma_bg))
            << "gyro axis " << static_cast<int>(axis);
    }

    // The walk actually wandered, and stayed inside the band a walk of this
    // length can reach: |b_N| is Rayleigh-ish with scale sigma*sqrt(N), so 6x
    // that is a ceiling a correct implementation cannot touch while a
    // mis-scaled one (h instead of sqrt(h), say) would sail past it.
    const spade::Result<const spade::sensors::ImuSensorRow*> row = sim->imu_sensor(*imu);
    ASSERT_OK(row);
    const float walk_scale = spec.sigma_ba * std::sqrt(static_cast<float>(kNoiseSamples));
    EXPECT_GT(glm::length((*row)->bias_a), 0.1f * walk_scale) << "the accel bias never moved";
    EXPECT_LT(glm::length((*row)->bias_a), 6.0f * walk_scale) << "the accel bias walk is mis-scaled";
    EXPECT_GT(glm::length((*row)->bias_g), 0.0f) << "the gyro bias never moved";
}

// The stream derivation is per-SENSOR (domain tag + sensor index), so two
// identically configured sensors on the same body must see independent noise --
// otherwise a two-IMU vehicle would have perfectly correlated errors, which is
// the failure mode core/rng.hpp's domain-separation note warns about.
TEST(Imu, TwoSensorsOnOneBodyDrawIndependentNoise) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 2, 0x7401ULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    ImuSensorSpawn spec;
    spec.sigma_a = 0.05f;
    const spade::Result<ImuSensorRef> first = sim->add_imu_sensor(*body, spec);
    ASSERT_OK(first);
    const spade::Result<ImuSensorRef> second = sim->add_imu_sensor(*body, spec);
    ASSERT_OK(second);
    ASSERT_OK(sim->flush_structural());

    ASSERT_OK(sim->step(kRingDepth));

    std::vector<ImuSample> a(kRingDepth);
    std::vector<ImuSample> b(kRingDepth);
    const spade::Result<ImuPoll> poll_a = sim->poll_imu(*first, 0, a);
    ASSERT_OK(poll_a);
    const spade::Result<ImuPoll> poll_b = sim->poll_imu(*second, 0, b);
    ASSERT_OK(poll_b);
    ASSERT_EQ(poll_a->samples.size(), poll_b->samples.size());
    ASSERT_GT(poll_a->samples.size(), 0u);

    std::size_t differing = 0;
    for (std::size_t i = 0; i < poll_a->samples.size(); ++i) {
        if (poll_a->samples[i].accel != poll_b->samples[i].accel) ++differing;
    }
    EXPECT_EQ(differing, poll_a->samples.size())
        << "two sensors are sharing a stream: every sample should differ, "
        << differing << " of " << poll_a->samples.size() << " did";
}

// The noise is a function of the WORLD SEED, and of nothing else stochastic in
// this world (turbulence is off and there are no force elements). Two seeds must
// therefore give different samples -- and with the sigmas zeroed, IDENTICAL
// ones, which is what proves the difference came from the noise rather than
// from the seed perturbing the trajectory.
TEST(Imu, NoiseIsSeededFromTheWorldSeed) {
    const auto run = [](uint64_t seed, float sigma, std::vector<ImuSample>& out) {
        const spade::Result<WorldSetDesc> set = void_world_set(1, 1, seed);
        ASSERT_OK(set);
        spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
        ASSERT_OK(sim);
        const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
        ASSERT_OK(body);
        ImuSensorSpawn spec;
        spec.sigma_a = sigma;
        spec.sigma_g = sigma;
        const spade::Result<ImuSensorRef> imu = sim->add_imu_sensor(*body, spec);
        ASSERT_OK(imu);
        ASSERT_OK(sim->flush_structural());
        ASSERT_OK(sim->step(32));
        std::vector<ImuSample> buffer(kRingDepth);
        const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, 0, buffer);
        ASSERT_OK(poll);
        out.assign(poll->samples.begin(), poll->samples.end());
    };

    std::vector<ImuSample> noisy_a;
    std::vector<ImuSample> noisy_b;
    run(0x11111111ULL, 0.05f, noisy_a);
    run(0x22222222ULL, 0.05f, noisy_b);
    ASSERT_EQ(noisy_a.size(), 32u);
    ASSERT_EQ(noisy_b.size(), 32u);
    std::size_t differing = 0;
    for (std::size_t i = 0; i < noisy_a.size(); ++i) {
        if (noisy_a[i].accel != noisy_b[i].accel) ++differing;
    }
    EXPECT_EQ(differing, noisy_a.size()) << "the IMU stream ignores the world seed";

    std::vector<ImuSample> clean_a;
    std::vector<ImuSample> clean_b;
    run(0x11111111ULL, 0.0f, clean_a);
    run(0x22222222ULL, 0.0f, clean_b);
    ASSERT_EQ(clean_a.size(), clean_b.size());
    for (std::size_t i = 0; i < clean_a.size(); ++i) {
        EXPECT_EQ(clean_a[i].accel, clean_b[i].accel)
            << "sample " << i << ": with the sigmas zeroed the seed must change nothing";
        EXPECT_EQ(clean_a[i].gyro, clean_b[i].gyro) << "sample " << i;
    }
}

// ===========================================================================
// 5. THE RATE DIVIDER
// ===========================================================================

// N substeps at divider k produce EXACTLY N/k samples. Run with substeps > 1 so
// the test also pins that the divider counts SUBSTEPS, not steps -- a divider
// implemented on the step counter would give 50/25/12/10 here instead of
// 200/100/50/40.
TEST(Imu, RateDividerProducesExactlyNOverKSamples) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 4, 0x4A7EULL);
    ASSERT_OK(set);
    constexpr uint64_t kSteps = 50;
    constexpr uint32_t kSubsteps = 4;
    constexpr uint64_t kSubstepsTotal = kSteps * kSubsteps;  // 200

    spade::Result<Simulation> sim = Simulation::create(*set, 4'000'000, kSubsteps);
    ASSERT_OK(sim);

    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    const uint32_t dividers[4] = {1, 2, 4, 5};
    ImuSensorRef refs[4];
    for (std::size_t i = 0; i < 4; ++i) {
        ImuSensorSpawn spec;
        spec.rate_divider = dividers[i];
        const spade::Result<ImuSensorRef> ref = sim->add_imu_sensor(*body, spec);
        ASSERT_OK(ref);
        refs[i] = *ref;
    }
    ASSERT_OK(sim->flush_structural());
    ASSERT_OK(sim->step(kSteps));

    for (std::size_t i = 0; i < 4; ++i) {
        const spade::Result<const spade::sensors::ImuSensorRow*> row = sim->imu_sensor(refs[i]);
        ASSERT_OK(row);
        EXPECT_EQ((*row)->last_index, kSubstepsTotal / dividers[i])
            << "divider " << dividers[i] << " produced the wrong sample count";
        // Every period completed exactly, so no partial period is left over.
        EXPECT_EQ((*row)->phase, 0u) << "divider " << dividers[i];
    }

    // TICK STAMPS. A sensor fires on the substep that COMPLETES its period, so
    // sample j (1-based) at divider k is produced at global substep j*k - 1 and
    // therefore carries tick (j*k - 1) / substeps.
    std::vector<ImuSample> buffer(kRingDepth);
    for (std::size_t i = 0; i < 4; ++i) {
        const spade::Result<ImuPoll> poll = sim->poll_imu(refs[i], 0, buffer);
        ASSERT_OK(poll) << "divider " << dividers[i];
        ASSERT_FALSE(poll->samples.empty());
        for (const ImuSample& s : poll->samples) {
            const uint64_t substep = s.index * dividers[i] - 1u;
            EXPECT_EQ(s.tick, substep / kSubsteps)
                << "divider " << dividers[i] << ", sample " << s.index;
        }
    }
}

// ===========================================================================
// 6. THE RING (editor tech spec TA5)
// ===========================================================================

// Overruns the ring and checks every poll-since case. The samples are made
// individually identifiable by driving the body with a wrench that depends on
// the tick: with unit mass and identity attitude, accel.x is EXACTLY the tick
// the sample was produced in -- which also proves SensorSynthesis runs AFTER
// Integrate within the same substep, rather than one substep behind.
TEST(Imu, RingWrapsAndPollSinceReturnsOnlyResidentSamples) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, 0x21B6ULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);
    const spade::Result<ImuSensorRef> imu = sim->add_imu_sensor(*body, ImuSensorSpawn{});
    ASSERT_OK(imu);
    ASSERT_OK(sim->flush_structural());

    constexpr uint64_t kSteps = 100;  // > kRingDepth, so the ring wraps
    for (uint64_t t = 0; t < kSteps; ++t) {
        ASSERT_OK(sim->apply_wrench(*body, glm::vec3(static_cast<float>(t), 0.0f, 0.0f),
                                    glm::vec3(0.0f)));
        ASSERT_OK(sim->step(1));
    }

    const SampleIndex last = kSteps;
    const SampleIndex oldest = last - kRingDepth + 1;  // 37

    // (a) Poll everything: only the resident tail comes back, and the loss is
    //     reported rather than hidden.
    {
        std::vector<ImuSample> out(2 * kRingDepth);
        const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, 0, out);
        ASSERT_OK(poll);
        ASSERT_EQ(poll->samples.size(), kRingDepth);
        EXPECT_EQ(poll->dropped, oldest - 1);
        EXPECT_EQ(poll->next_since, last);
        for (std::size_t i = 0; i < poll->samples.size(); ++i) {
            const ImuSample& s = poll->samples[i];
            EXPECT_EQ(s.index, oldest + i) << "samples must come back oldest first, in index order";
            EXPECT_EQ(s.tick, s.index - 1);
            // The payload identifies its own tick, so a wrap that aliased two
            // samples onto one slot would show up here rather than as a
            // plausible-looking number.
            EXPECT_FLOAT_EQ(s.accel.x, static_cast<float>(s.index - 1));
        }
    }

    // (b) Up to date: nothing new, and the cursor does not move.
    {
        std::vector<ImuSample> out(kRingDepth);
        const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, last, out);
        ASSERT_OK(poll);
        EXPECT_TRUE(poll->samples.empty());
        EXPECT_EQ(poll->dropped, 0u);
        EXPECT_EQ(poll->next_since, last);
    }

    // (c) A since_index from the future (a caller that outlived a restore):
    //     empty, and still no cursor movement.
    {
        std::vector<ImuSample> out(kRingDepth);
        const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, last + 500, out);
        ASSERT_OK(poll);
        EXPECT_TRUE(poll->samples.empty());
        EXPECT_EQ(poll->next_since, last + 500);
    }

    // (d) The normal case: a caller two samples behind gets exactly those two.
    {
        std::vector<ImuSample> out(kRingDepth);
        const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, last - 2, out);
        ASSERT_OK(poll);
        ASSERT_EQ(poll->samples.size(), 2u);
        EXPECT_EQ(poll->dropped, 0u);
        EXPECT_EQ(poll->samples[0].index, last - 1);
        EXPECT_EQ(poll->samples[1].index, last);
        EXPECT_EQ(poll->next_since, last);
    }

    // (e) A buffer too small to drain the ring: the OLDEST samples that fit
    //     come back and the cursor stops there, so the next poll continues
    //     rather than skipping.
    {
        std::vector<ImuSample> out(10);
        const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, 0, out);
        ASSERT_OK(poll);
        ASSERT_EQ(poll->samples.size(), 10u);
        EXPECT_EQ(poll->dropped, oldest - 1);
        EXPECT_EQ(poll->samples[0].index, oldest);
        EXPECT_EQ(poll->next_since, oldest + 9);

        const spade::Result<ImuPoll> again = sim->poll_imu(*imu, poll->next_since, out);
        ASSERT_OK(again);
        ASSERT_EQ(again->samples.size(), 10u);
        EXPECT_EQ(again->dropped, 0u) << "a truncated poll must not lose the samples it deferred";
        EXPECT_EQ(again->samples[0].index, oldest + 10);
    }

    // (f) An empty output buffer is a legal no-op, not an out-of-range write.
    {
        const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, 0, std::span<ImuSample>());
        ASSERT_OK(poll);
        EXPECT_TRUE(poll->samples.empty());
        EXPECT_EQ(poll->next_since, 0u);
    }
}

// ===========================================================================
// 7. LIFECYCLE
// ===========================================================================

TEST(Imu, DespawningABodyFreesItsSensorsAndZeroesTheirRings) {
    const spade::Result<WorldSetDesc> set = void_world_set(2, 2, 0xDEADULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);
    const spade::Result<ImuSensorRef> first = sim->add_imu_sensor(*body, ImuSensorSpawn{});
    ASSERT_OK(first);
    const spade::Result<ImuSensorRef> second = sim->add_imu_sensor(*body, ImuSensorSpawn{});
    ASSERT_OK(second);
    ASSERT_OK(sim->flush_structural());
    ASSERT_OK(sim->step(10));

    EXPECT_EQ(sim->live_imu_sensor_count(0).value_or(0u), 2u);
    {
        std::vector<ImuSample> out(kRingDepth);
        const spade::Result<ImuPoll> poll = sim->poll_imu(*first, 0, out);
        ASSERT_OK(poll);
        ASSERT_EQ(poll->samples.size(), 10u);
    }

    ASSERT_OK(sim->despawn(*body));
    ASSERT_OK(sim->flush_structural());

    // Both sensors went with the body -- no leak, no orphan.
    EXPECT_EQ(sim->live_imu_sensor_count(0).value_or(99u), 0u);
    {
        std::vector<ImuSample> out(kRingDepth);
        const spade::Result<ImuPoll> poll = sim->poll_imu(*first, 0, out);
        EXPECT_EQ(code_of(poll), code(spade::Code::not_found));
    }

    // ...and their rings read as zeroes, like every other freed slot in the
    // engine. Without the explicit clear these would still hold ten live
    // samples, in every snapshot, forever.
    {
        const spade::Result<std::span<const ImuSample>> ring =
            sim->arenas().array(sim->imu_ring_array());
        ASSERT_OK(ring);
        ImuSample zero;
        std::memset(&zero, 0, sizeof(zero));
        for (const ImuSample& s : *ring) {
            EXPECT_EQ(std::memcmp(&s, &zero, sizeof(ImuSample)), 0)
                << "a freed sensor left samples behind in the ring";
        }
    }

    // A new sensor in the recycled slot starts clean: index 1 again, and no
    // inherited history.
    const spade::Result<BodyRef> replacement = sim->spawn(0, unit_body());
    ASSERT_OK(replacement);
    const spade::Result<ImuSensorRef> reused = sim->add_imu_sensor(*replacement, ImuSensorSpawn{});
    ASSERT_OK(reused);
    EXPECT_EQ(reused->slot, first->slot) << "lowest-free-first should have recycled the slot";
    ASSERT_OK(sim->flush_structural());
    ASSERT_OK(sim->step(5));

    std::vector<ImuSample> out(kRingDepth);
    const spade::Result<ImuPoll> poll = sim->poll_imu(*reused, 0, out);
    ASSERT_OK(poll);
    ASSERT_EQ(poll->samples.size(), 5u);
    EXPECT_EQ(poll->samples[0].index, 1u);
    EXPECT_EQ(poll->dropped, 0u);
}

TEST(Imu, RejectsMisconfiguredSensors) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, 0xBAD0ULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);
    ASSERT_OK(sim->flush_structural());

    {
        ImuSensorSpawn spec;
        spec.rate_divider = 0;
        EXPECT_EQ(code_of(sim->add_imu_sensor(*body, spec)), code(spade::Code::invalid_argument));
    }
    {
        ImuSensorSpawn spec;
        spec.sigma_a = -1.0f;
        EXPECT_EQ(code_of(sim->add_imu_sensor(*body, spec)), code(spade::Code::invalid_argument));
    }
    {
        ImuSensorSpawn spec;
        spec.sigma_bg = std::numeric_limits<float>::quiet_NaN();
        EXPECT_EQ(code_of(sim->add_imu_sensor(*body, spec)), code(spade::Code::invalid_argument));
    }
    {
        ImuSensorSpawn spec;
        spec.mount_pos = glm::vec3(std::numeric_limits<float>::infinity(), 0.0f, 0.0f);
        EXPECT_EQ(code_of(sim->add_imu_sensor(*body, spec)), code(spade::Code::invalid_argument));
    }
    {
        ImuSensorSpawn spec;
        spec.mount_orient = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
        EXPECT_EQ(code_of(sim->add_imu_sensor(*body, spec)), code(spade::Code::invalid_argument));
    }

    // The world declares one sensor; the second is refused rather than silently
    // overrunning the partition.
    ASSERT_OK(sim->add_imu_sensor(*body, ImuSensorSpawn{}));
    EXPECT_EQ(code_of(sim->add_imu_sensor(*body, ImuSensorSpawn{})),
              code(spade::Code::capacity_exceeded));

    // A dead body cannot gain a sensor.
    ASSERT_OK(sim->flush_structural());
    ASSERT_OK(sim->despawn(*body));
    EXPECT_EQ(code_of(sim->add_imu_sensor(*body, ImuSensorSpawn{})), code(spade::Code::not_found));

    // Neither can a hand-built sensor ref address anything.
    std::vector<ImuSample> out(kRingDepth);
    EXPECT_EQ(code_of(sim->poll_imu(ImuSensorRef{7, 0}, 0, out)), code(spade::Code::not_found));
    EXPECT_EQ(code_of(sim->poll_imu(ImuSensorRef{0, 999}, 0, out)), code(spade::Code::not_found));
}

// ===========================================================================
// 8. DETERMINISM AND SNAPSHOT RESUME (charter P2/P4)
// ===========================================================================

namespace {

using RefTable = std::shared_ptr<std::vector<ImuSensorRef>>;

// Two worlds (via replicate, so their rng roots are domain-separated), each
// holding two bodies, each body carrying two noisy IMUs at different rates and
// different mount poses -- plus a scripted per-tick wrench so the true signal
// is not constant. Everything the sensor system holds across a substep (bias
// walk, divider phase, rng stream, ring cursor, ring contents) is in play.
[[nodiscard]] Scenario imu_scenario(RefTable refs) {
    Scenario s;
    s.name = "imu_replay";
    s.dt_ns = 2'000'000;  // 2 ms step
    s.substeps = 2;       // 1 ms substep
    s.steps = 300;

    s.build = []() -> spade::Result<WorldSetDesc> {
        const spade::Result<spade::WorldDesc> world = WorldBuilder()
                                                          .name("void")
                                                          .environment(default_environment())
                                                          .capacities(capacities(2, 1, 4))
                                                          .build();
        if (!world) return std::unexpected(world.error());

        WorldInstanceDesc prototype;
        prototype.world = *world;
        prototype.turbulence = spade::dryden_params(TurbulenceLevel::light);
        prototype.contacts = no_contacts();
        prototype.grid = unit_grid();
        return spade::replicate(prototype, 2, 0x1D5EEDULL);
    };

    s.setup = [refs](Simulation& sim) -> spade::Result<void> {
        refs->clear();
        for (uint32_t w = 0; w < sim.world_count(); ++w) {
            for (uint32_t i = 0; i < 2; ++i) {
                BodySpawn body = unit_body();
                body.pos = glm::vec3(0.5f * static_cast<float>(i), 10.0f, 0.0f);
                body.omega_body = glm::vec3(0.2f, 0.4f * static_cast<float>(i), -0.3f);
                const spade::Result<BodyRef> ref = sim.spawn(w, body);
                if (!ref) return std::unexpected(ref.error());

                ImuSensorSpawn fast;
                fast.sigma_a = 0.02f;
                fast.sigma_g = 0.004f;
                fast.sigma_ba = 0.001f;
                fast.sigma_bg = 0.0005f;
                fast.mount_pos = glm::vec3(0.03f, -0.01f, 0.02f);
                const spade::Result<ImuSensorRef> a = sim.add_imu_sensor(*ref, fast);
                if (!a) return std::unexpected(a.error());
                refs->push_back(*a);

                ImuSensorSpawn slow = fast;
                slow.rate_divider = 3;  // does not divide the substep count evenly
                slow.mount_orient = mount_yaw90();
                const spade::Result<ImuSensorRef> b = sim.add_imu_sensor(*ref, slow);
                if (!b) return std::unexpected(b.error());
                refs->push_back(*b);
            }
        }
        return {};
    };

    s.input = [](Simulation& sim, Tick tick) -> spade::Result<void> {
        for (uint32_t w = 0; w < sim.world_count(); ++w) {
            for (uint32_t i = 0; i < 2; ++i) {
                const spade::Result<BodyRef> ref = sim.body_ref_at(w, i);
                if (!ref) continue;
                // Deterministic, transcendental-free, and independent of the
                // world index (so it cannot confound a per-world comparison).
                const float a = static_cast<float>((tick.value + i) % 7u) - 3.0f;
                if (spade::Result<void> r = sim.apply_wrench(
                        *ref, glm::vec3(0.1f * a, kG + 0.05f * a, 0.0f), glm::vec3(0.0f, 0.01f * a, 0.0f));
                    !r) {
                    return r;
                }
            }
        }
        return {};
    };

    return s;
}

// Every resident sample of every sensor, in ref order -- the user-visible
// half of the replay guarantee.
[[nodiscard]] std::vector<ImuSample> drain_all(const Simulation& sim,
                                               const std::vector<ImuSensorRef>& refs) {
    std::vector<ImuSample> all;
    std::vector<ImuSample> buffer(kRingDepth);
    for (const ImuSensorRef& ref : refs) {
        const spade::Result<ImuPoll> poll = sim.poll_imu(ref, 0, buffer);
        if (!poll) continue;
        all.insert(all.end(), poll->samples.begin(), poll->samples.end());
    }
    return all;
}

}  // namespace

TEST(Imu, SeededRunsAreIdenticalAndSurviveASnapshotRestore) {
    RefTable refs = std::make_shared<std::vector<ImuSensorRef>>();
    const Scenario scenario = imu_scenario(refs);

    // (a) The same scenario twice: same digest, same samples.
    const spade::Result<uint64_t> first = spade::testing::run_scenario(scenario);
    ASSERT_OK(first);
    const spade::Result<uint64_t> second = spade::testing::run_scenario(scenario);
    ASSERT_OK(second);
    EXPECT_EQ(*first, *second);

    // (b) The replay guarantee. Snapshot at the halfway point, restore into a
    //     FRESH Simulation -- so the resumed run cannot benefit from state the
    //     original object still happened to hold -- and replay the same inputs.
    const uint64_t k = scenario.steps / 2;

    spade::Result<Simulation> reference = spade::testing::start_scenario(scenario);
    ASSERT_OK(reference);
    ASSERT_OK(spade::testing::advance_scenario(scenario, *reference, scenario.steps));
    const uint64_t expected_digest = spade::testing::state_digest(*reference);
    const std::vector<ImuSensorRef> sensor_refs = *refs;
    ASSERT_FALSE(sensor_refs.empty());
    const std::vector<ImuSample> expected_samples = drain_all(*reference, sensor_refs);
    ASSERT_FALSE(expected_samples.empty());

    spade::Result<Simulation> interrupted = spade::testing::start_scenario(scenario);
    ASSERT_OK(interrupted);
    ASSERT_OK(spade::testing::advance_scenario(scenario, *interrupted, k));
    const spade::Result<SnapshotBlob> blob = interrupted->snapshot();
    ASSERT_OK(blob);
    EXPECT_EQ(blob->tick().value, k);

    spade::Result<Simulation> resumed = spade::testing::start_scenario(scenario);
    ASSERT_OK(resumed);
    ASSERT_OK(resumed->restore(*blob));
    EXPECT_EQ(resumed->tick().value, k);
    ASSERT_OK(spade::testing::advance_scenario(scenario, *resumed, scenario.steps));

    // The digest covers the sensor rows and the ring bytes structurally (the
    // registry walk reaches every registered array), so this alone would catch
    // a bias, a phase, a stream or a sample that did not ride the blob.
    EXPECT_EQ(spade::testing::state_digest(*resumed), expected_digest);

    // And the same claim stated the way a consumer experiences it: the samples
    // the resumed run went on to produce are byte-identical.
    const std::vector<ImuSample> resumed_samples = drain_all(*resumed, sensor_refs);
    ASSERT_EQ(resumed_samples.size(), expected_samples.size());
    EXPECT_EQ(std::memcmp(resumed_samples.data(), expected_samples.data(),
                          resumed_samples.size() * sizeof(ImuSample)),
              0)
        << "the resumed run's IMU output diverged from the uninterrupted run's";

    // The scenario must actually be exercising the sensors, or all of the above
    // is a statement about an empty ring.
    const spade::Result<const spade::sensors::ImuSensorRow*> row = resumed->imu_sensor(sensor_refs[0]);
    ASSERT_OK(row);
    EXPECT_GT((*row)->last_index, 0u);
    EXPECT_GT(glm::length((*row)->bias_a), 0.0f) << "the bias walk never ran";
}

// A world stepped alone and the same world stepped inside a set must produce
// identical per-world state INCLUDING their sensors -- which is only true
// because the noise stream is keyed on the WORLD-LOCAL sensor slot rather than
// the global one. Keyed globally, world 1's sensors would draw a different
// sequence depending on how many sensors world 0 declared.
TEST(Imu, SensorNoiseIsBatchingInvariant) {
    RefTable refs = std::make_shared<std::vector<ImuSensorRef>>();
    const Scenario scenario = imu_scenario(refs);

    spade::Result<Simulation> both = spade::testing::start_scenario(scenario);
    ASSERT_OK(both);
    ASSERT_OK(spade::testing::advance_scenario(scenario, *both, scenario.steps));

    const spade::Result<WorldSetDesc> full = scenario.build();
    ASSERT_OK(full);
    ASSERT_EQ(full->worlds.size(), 2u);

    for (uint32_t w = 0; w < 2; ++w) {
        WorldSetDesc solo;
        solo.worlds.push_back(full->worlds[w]);
        spade::Result<Simulation> alone = Simulation::create(solo, scenario.dt_ns, scenario.substeps);
        ASSERT_OK(alone) << "world " << w;
        ASSERT_OK(scenario.setup(*alone)) << "world " << w;
        ASSERT_OK(alone->flush_structural()) << "world " << w;
        ASSERT_OK(spade::testing::advance_scenario(scenario, *alone, scenario.steps)) << "world " << w;

        EXPECT_EQ(spade::testing::world_digest(*alone, 0), spade::testing::world_digest(*both, w))
            << "world " << w << " diverged between the solo and batched runs";
    }
}

// ===========================================================================
// 9. RESEED (S5 Task 4; engine design Addendum A3)
//
// Simulation::reseed(scene_seed) rewrites every world's rng root and
// re-derives the two things that had already cached a stream from the old one
// -- the world's DrydenState and every LIVE sensor's ImuSensorRow::noise.
// This section owns the SENSOR half of that claim; the seed derivation
// itself, the whole-set determinism and the snapshot interaction are pinned
// in test_m1b_bar.cpp's A3 conformance case.
//
// The claim has two halves and they pull in opposite directions, which is
// exactly why one test asserts both:
//
//   * THE FUTURE CHANGES. A sensor that was ALREADY LIVE before the reseed
//     must draw its next samples from a stream derived from the NEW world
//     seed -- not from the old one it was carrying, and not from a stream
//     that would only be right for a sensor added afterwards.
//   * THE PAST DOES NOT. Its ring, its ring cursor, its divider phase and
//     its accumulated bias states are history and stay exactly as they were
//     (sim/simulation.hpp's reseed doc comment says so in those words).
//
// Both are checked against a stream this file constructs and draws from
// ITSELF, following sensors/imu.hpp's stated model -- so a reseed that
// re-derived the stream slightly differently (wrong seed, global instead of
// world-local slot, a reset bias) fails here rather than agreeing with
// itself.
// ===========================================================================

namespace {

// Three standard normals in the pinned x, y, z order. sensors/imu.cpp's
// draw_gauss3 exists to force exactly this sequencing, and the order is part
// of the contract rather than an implementation detail.
[[nodiscard]] glm::vec3 gauss3(spade::rng::Stream& stream) {
    const float x = stream.next_gauss();
    const float y = stream.next_gauss();
    const float z = stream.next_gauss();
    return glm::vec3(x, y, z);
}

struct Sigmas {
    float a = 0.0f;
    float g = 0.0f;
    float ba = 0.0f;
    float bg = 0.0f;
};

// One emitted sample of a COM-mounted, identity-mount sensor on a body in
// FREE FALL, from sensors/imu.hpp section 4's model alone.
//
// Free fall is what makes the prediction COMPLETE rather than approximate:
// specific_force is exactly zero there (FreeFallingBodyReadsZeroSpecificForce,
// above) and an untorqued body's omega stays exactly zero, so accel_true and
// gyro_true vanish and the sample IS the noise. Twelve draws per sample in
// the header's pinned order (accel bias walk, gyro bias walk, accel white,
// gyro white); the walks advance once per SAMPLE; both sums are grouped left
// to right exactly as section 4 writes them, because fp32 addition is not
// associative and the grouping is therefore part of the expectation.
[[nodiscard]] ImuSample predict_free_fall_sample(spade::rng::Stream& stream, const Sigmas& s,
                                                 glm::vec3& bias_a, glm::vec3& bias_g) {
    const glm::vec3 walk_a = gauss3(stream);
    const glm::vec3 walk_g = gauss3(stream);
    const glm::vec3 white_a = gauss3(stream);
    const glm::vec3 white_g = gauss3(stream);

    bias_a += s.ba * walk_a;
    bias_g += s.bg * walk_g;

    ImuSample out;
    std::memset(&out, 0, sizeof(out));
    out.accel = glm::vec3(0.0f) + bias_a + s.a * white_a;
    out.gyro = glm::vec3(0.0f) + bias_g + s.g * white_g;
    return out;
}

}  // namespace

TEST(Imu, ReseedRederivesALiveSensorsStreamAndLeavesItsHistoryAlone) {
    // The world's root is set DIRECTLY here (void_world_set writes
    // WorldInstanceDesc::seed, which create() copies into WorldParams), so
    // the sensor's original stream is imu_noise_stream(kOldWorldSeed, 0) with
    // no derivation in between -- which is what lets the "before" half below
    // be predicted as exactly as the "after" half.
    constexpr uint64_t kOldWorldSeed = 0x0DDC0FFEEULL;
    constexpr uint64_t kSceneSeed = 0xA3F00DULL;
    // reseed() derives the world root from the SCENE seed by replicate()'s
    // formula -- re-written here from sim/world_set.hpp's pinned text, with
    // the domain tag spelled as a literal so a change to the pinned constant
    // fails rather than following along.
    const uint64_t new_world_seed =
        spade::rng::splitmix64(kSceneSeed ^ spade::rng::fnv1a64("world") ^ 0ULL);
    ASSERT_NE(new_world_seed, kOldWorldSeed);

    constexpr Sigmas kSigmas{0.05f, 0.01f, 0.001f, 0.0005f};
    constexpr uint64_t kBefore = 8;  // samples drawn before the reseed
    constexpr uint64_t kAfter = 6;   // ...and after it

    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, kOldWorldSeed);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);

    // FREE FALL: no wrench, no drag element, no turbulence coupling. The
    // samples are pure noise, so the prediction below is complete.
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);
    ImuSensorSpawn spec;
    spec.sigma_a = kSigmas.a;
    spec.sigma_g = kSigmas.g;
    spec.sigma_ba = kSigmas.ba;
    spec.sigma_bg = kSigmas.bg;
    const spade::Result<ImuSensorRef> imu = sim->add_imu_sensor(*body, spec);
    ASSERT_OK(imu);
    ASSERT_OK(sim->flush_structural());
    ASSERT_EQ(imu->slot, 0u) << "the world-local slot the streams below are keyed on";

    ASSERT_OK(sim->step(kBefore));

    std::vector<ImuSample> buffer(kRingDepth);
    std::vector<ImuSample> before;
    {
        const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, 0, buffer);
        ASSERT_OK(poll);
        ASSERT_EQ(poll->samples.size(), kBefore);
        before.assign(poll->samples.begin(), poll->samples.end());
    }

    // The pre-reseed run, predicted from the OLD root. This is not padding:
    // it proves the model in predict_free_fall_sample() is the right model
    // BEFORE it is used to judge the post-reseed samples, and it leaves
    // `old_stream` sitting exactly where the sensor's own stream sits -- which
    // is what makes the negative control at the end possible.
    spade::rng::Stream old_stream = spade::sensors::imu_noise_stream(kOldWorldSeed, 0);
    glm::vec3 bias_a(0.0f);
    glm::vec3 bias_g(0.0f);
    for (std::size_t i = 0; i < before.size(); ++i) {
        const ImuSample want = predict_free_fall_sample(old_stream, kSigmas, bias_a, bias_g);
        ASSERT_EQ(before[i].accel, want.accel) << "pre-reseed sample " << i;
        ASSERT_EQ(before[i].gyro, want.gyro) << "pre-reseed sample " << i;
    }
    const glm::vec3 bias_a_at_reseed = bias_a;
    const glm::vec3 bias_g_at_reseed = bias_g;
    ASSERT_NE(bias_a_at_reseed, glm::vec3(0.0f)) << "the bias walk must have actually accumulated, "
                                                    "or 'history is preserved' is vacuous";

    // The row's own view of that history, captured for the "untouched" half.
    spade::sensors::ImuSensorRow row_before;
    {
        const spade::Result<const spade::sensors::ImuSensorRow*> row = sim->imu_sensor(*imu);
        ASSERT_OK(row);
        row_before = **row;
    }
    ASSERT_EQ(row_before.bias_a, bias_a_at_reseed) << "the engine's bias state and the predicted "
                                                     "one must agree before the reseed";
    ASSERT_EQ(row_before.bias_g, bias_g_at_reseed);

    // ---------------------------------------------------------------------
    // THE RESEED.
    // ---------------------------------------------------------------------
    ASSERT_OK(sim->reseed(kSceneSeed));

    // THE PAST IS UNTOUCHED. Bias states, ring cursor and divider phase are
    // exactly what they were; only `noise` moved.
    {
        const spade::Result<const spade::sensors::ImuSensorRow*> row = sim->imu_sensor(*imu);
        ASSERT_OK(row);
        EXPECT_EQ((*row)->bias_a, row_before.bias_a) << "reseed rewound the accumulated accel bias";
        EXPECT_EQ((*row)->bias_g, row_before.bias_g) << "reseed rewound the accumulated gyro bias";
        EXPECT_EQ((*row)->last_index, row_before.last_index) << "reseed moved the ring cursor";
        EXPECT_EQ((*row)->phase, row_before.phase) << "reseed re-phased the rate divider";
        EXPECT_NE(std::memcmp(&(*row)->noise, &row_before.noise, sizeof(spade::rng::Stream)), 0)
            << "reseed did not touch the sensor's stream at all";
    }

    // ...AND SO IS THE RING. The samples the caller has not consumed yet are
    // readings that genuinely happened; a reseed is not entitled to them.
    {
        const spade::Result<ImuPoll> poll = sim->poll_imu(*imu, 0, buffer);
        ASSERT_OK(poll);
        ASSERT_EQ(poll->samples.size(), before.size());
        for (std::size_t i = 0; i < before.size(); ++i) {
            EXPECT_EQ(std::memcmp(&poll->samples[i], &before[i], sizeof(ImuSample)), 0)
                << "reseed rewrote ring sample " << i;
        }
    }

    // ---------------------------------------------------------------------
    // THE FUTURE IS THE NEW STREAM'S -- continued from the OLD bias state.
    // ---------------------------------------------------------------------
    ASSERT_OK(sim->step(kAfter));

    std::vector<ImuSample> after;
    {
        // `since_index` is EXCLUSIVE (sensors/rings.hpp: "samples with index >
        // since_index"), so passing the last pre-reseed index asks for exactly
        // what came after it.
        const spade::Result<ImuPoll> poll =
            sim->poll_imu(*imu, static_cast<SampleIndex>(kBefore), buffer);
        ASSERT_OK(poll);
        ASSERT_EQ(poll->samples.size(), kAfter);
        ASSERT_EQ(poll->dropped, 0u);
        after.assign(poll->samples.begin(), poll->samples.end());
    }

    spade::rng::Stream new_stream = spade::sensors::imu_noise_stream(new_world_seed, 0);
    glm::vec3 new_bias_a = bias_a_at_reseed;  // history carried, not reset
    glm::vec3 new_bias_g = bias_g_at_reseed;
    for (std::size_t i = 0; i < after.size(); ++i) {
        const ImuSample want = predict_free_fall_sample(new_stream, kSigmas, new_bias_a, new_bias_g);
        EXPECT_EQ(after[i].accel, want.accel)
            << "post-reseed sample " << i
            << ": a live sensor is not drawing imu_noise_stream(new world seed, world-local slot) "
               "continued from its pre-reseed bias";
        EXPECT_EQ(after[i].gyro, want.gyro) << "post-reseed sample " << i;
    }

    // THE NEGATIVE CONTROL. Had the reseed not reached this already-live row,
    // the sensor would have gone on drawing from where its OLD stream stood --
    // so those are the samples this run must NOT have produced.
    glm::vec3 stale_bias_a = bias_a_at_reseed;
    glm::vec3 stale_bias_g = bias_g_at_reseed;
    std::size_t matching_stale = 0;
    for (std::size_t i = 0; i < after.size(); ++i) {
        const ImuSample stale =
            predict_free_fall_sample(old_stream, kSigmas, stale_bias_a, stale_bias_g);
        if (after[i].accel == stale.accel) ++matching_stale;
    }
    EXPECT_EQ(matching_stale, 0u)
        << matching_stale << " of " << after.size()
        << " post-reseed samples match the OLD stream's continuation: the reseed re-derived "
           "WorldParams::seed but left this already-live sensor row behind";
}

// ---------------------------------------------------------------------------
// PHY-7: the specific force includes the contact response. To an
// accelerometer a floor's reaction is a force like any other: at rest it reads
// +g up, and through an impact it reads the velocity change less gravity,
// sample by sample (Kat's report; physics/plans/2026-10-05-imu-contact-
// specific-force-plan.md). The body below never rotates and the IMU sits at
// its COM, so body, mount and world axes coincide.
// ---------------------------------------------------------------------------
namespace {

constexpr float kBallRadius = 0.1f;

[[nodiscard]] spade::Result<WorldSetDesc> contact_world_set(uint32_t bodies, uint32_t sensors, float restitution,
                                                            bool floor) {
    WorldBuilder builder;
    builder.name(floor ? "floor" : "open")
        .environment(default_environment())
        .capacities(capacities(bodies, 1, sensors));
    if (floor) builder.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    const spade::Result<spade::WorldDesc> world = builder.build();
    if (!world) return std::unexpected(world.error());

    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0xF1002ULL;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts = no_contacts();
    instance.contacts.restitution_e = restitution;
    instance.contacts.proxy_radius = kBallRadius;
    instance.grid = unit_grid();
    return WorldSetDesc{{instance}};
}

// One substep per step; after each, the body's newest sample against the
// state: (v_k - v_{k-1}) / h - gravity. Counts the steps that disagree and
// keeps the first, so a red run reports one line, not hundreds.
struct Tracked {
    BodyRef body;
    ImuSensorRef imu;
    SampleIndex since = 0;
    glm::vec3 v_prev{0.0f};
    glm::vec3 peak{0.0f};  // the largest |component| seen, signed
    int mismatches = 0;
    std::string first{};
};

void step_and_check(Simulation& sim, std::span<Tracked> tracked, int steps) {
    const float h = sim.substep_h();
    const glm::vec3 g = default_environment().gravity;
    std::vector<ImuSample> buffer(kRingDepth);
    for (Tracked& t : tracked) {
        const spade::Result<const spade::BodyState*> state = sim.body(t.body);
        ASSERT_OK(state);
        t.v_prev = (*state)->vel;
    }
    for (int k = 0; k < steps; ++k) {
        ASSERT_OK(sim.step(1));
        for (Tracked& t : tracked) {
            const spade::Result<ImuPoll> poll = sim.poll_imu(t.imu, t.since, buffer);
            ASSERT_OK(poll);
            ASSERT_EQ(poll->samples.size(), 1u);
            t.since = poll->next_since;
            const spade::Result<const spade::BodyState*> state = sim.body(t.body);
            ASSERT_OK(state);
            const glm::vec3 v = (*state)->vel;
            const glm::vec3 want = (v - t.v_prev) / h - g;
            const glm::vec3 got = poll->samples[0].accel;
            const float tol = 1e-2f + 1e-4f * glm::length(want);
            if (glm::length(got - want) > tol) {
                if (t.mismatches++ == 0) {
                    t.first = "step " + std::to_string(k) + ": reads (" + std::to_string(got.x) + ", " +
                              std::to_string(got.y) + ", " + std::to_string(got.z) + "), the state says (" +
                              std::to_string(want.x) + ", " + std::to_string(want.y) + ", " +
                              std::to_string(want.z) + ")";
                }
            }
            for (int i = 0; i < 3; ++i) {
                if (std::fabs(got[i]) > std::fabs(t.peak[i])) t.peak[i] = got[i];
            }
            t.v_prev = v;
        }
    }
}

}  // namespace

TEST(ImuContact, AtRestOnTheFloorItReadsPlusGUp) {
    const spade::Result<WorldSetDesc> set = contact_world_set(1, 1, /*restitution=*/0.0f, /*floor=*/true);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    BodySpawn spawn = unit_body();
    spawn.pos = glm::vec3(0.0f, kBallRadius, 0.0f);
    const spade::Result<BodyRef> body = sim->spawn(0, spawn);
    ASSERT_OK(body);
    const spade::Result<ImuSensorRef> imu = sim->add_imu_sensor(*body, ImuSensorSpawn{});
    ASSERT_OK(imu);
    ASSERT_OK(sim->flush_structural());

    std::vector<ImuSample> samples;
    collect_samples(*sim, *imu, 1000, kPollBlock, samples);
    ASSERT_EQ(samples.size(), 1000u);
    glm::vec3 mean(0.0f);
    for (std::size_t i = samples.size() - 64; i < samples.size(); ++i) mean += samples[i].accel;
    mean /= 64.0f;
    EXPECT_NEAR(mean.x, 0.0f, 1e-4f);
    EXPECT_NEAR(mean.y, kG, 1e-3f) << "an IMU resting on the floor must read +g up; 0 is the contact-blind "
                                      "reading PHY-7 removes";
    EXPECT_NEAR(mean.z, 0.0f, 1e-4f);
}

TEST(ImuContact, ThroughABounceItReadsTheVelocityChangeLessGravity) {
    const spade::Result<WorldSetDesc> set = contact_world_set(1, 1, /*restitution=*/0.5f, /*floor=*/true);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    BodySpawn spawn = unit_body();
    spawn.pos = glm::vec3(0.0f, 0.5f, 0.0f);
    const spade::Result<BodyRef> body = sim->spawn(0, spawn);
    ASSERT_OK(body);
    const spade::Result<ImuSensorRef> imu = sim->add_imu_sensor(*body, ImuSensorSpawn{});
    ASSERT_OK(imu);
    ASSERT_OK(sim->flush_structural());

    // 0.7 s: the fall (0.29 s), the first bounce, the second impact (~0.57 s).
    Tracked t{*body, *imu};
    step_and_check(*sim, std::span<Tracked>(&t, 1), 700);
    EXPECT_EQ(t.mismatches, 0) << t.first;
    // The impact: about (1 + e) * sqrt(2 g (0.5 - r)) / h = 1.5 * 2.8 / 1e-3, some 430 g, at full size (no
    // range clamp).
    EXPECT_GT(t.peak.y, 100.0f * kG);
}

TEST(ImuContact, TwoBodiesThatCollideReadEqualAndOppositeReactions) {
    const spade::Result<WorldSetDesc> set = contact_world_set(2, 2, /*restitution=*/1.0f, /*floor=*/false);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    BodySpawn left = unit_body();
    left.pos = glm::vec3(-0.5f, 100.0f, 0.0f);
    left.vel = glm::vec3(2.0f, 0.0f, 0.0f);
    BodySpawn right = unit_body();
    right.pos = glm::vec3(0.0f, 100.0f, 0.0f);
    const spade::Result<BodyRef> a = sim->spawn(0, left);
    ASSERT_OK(a);
    const spade::Result<BodyRef> b = sim->spawn(0, right);
    ASSERT_OK(b);
    const spade::Result<ImuSensorRef> imu_a = sim->add_imu_sensor(*a, ImuSensorSpawn{});
    ASSERT_OK(imu_a);
    const spade::Result<ImuSensorRef> imu_b = sim->add_imu_sensor(*b, ImuSensorSpawn{});
    ASSERT_OK(imu_b);
    ASSERT_OK(sim->flush_structural());

    // They touch after (0.5 - 2r) / 2 = 0.15 s; equal masses and e = 1 swap
    // their velocities, a 2 m/s change each in one substep.
    Tracked t[2] = {{*a, *imu_a}, {*b, *imu_b}};
    step_and_check(*sim, t, 300);
    EXPECT_EQ(t[0].mismatches, 0) << "the moving body: " << t[0].first;
    EXPECT_EQ(t[1].mismatches, 0) << "the struck body: " << t[1].first;
    EXPECT_LT(t[0].peak.x, -1000.0f);
    EXPECT_GT(t[1].peak.x, 1000.0f);
}
