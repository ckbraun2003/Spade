// test_sensor_errors.cpp -- the turn-on offset, the range and the quantum of the existing IMU, and the turn-on offset of the existing GNSS (O140 phase 1:
// docs/design/physics/plans/2026-11-21-fc-physics.md section 1.3; the placement agreed with Core, who owns the rows). Everything here is OFF at zero: the
// tests that pin the defaults say so, and the determinism corpus is the other half of that claim (no digest moves).
//
// WHAT IS MEASURED, AND HOW. The turn-on offsets are drawn at spawn and at every reseed() from a second pinned stream, so the tests reseed one simulation many
// times and read the bias state each time: a standard normal scaled by the stated sigma, so the mean is 0 and the sample deviation is the sigma, to a bound from
// the sample size. The converter is measured on TWO simulations of the same seed and slot, one clamping only and one clamping and rounding, so the noise under
// both is the same bytes and the difference of the two is the rounding error alone: bounded by half a quantum, with the deviation of a uniform, quantum / sqrt(12).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "core/rng.hpp"
#include "physics/integrator.hpp"
#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "sensors/rings.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "gpu_skip.hpp"

namespace {

using spade::BodyRef;
using spade::BodySpawn;
using spade::Capacities;
using spade::Environment;
using spade::GnssSensorRef;
using spade::GnssSensorSpawn;
using spade::ImuPoll;
using spade::ImuSensorRef;
using spade::ImuSensorSpawn;
using spade::Simulation;
using spade::TurbulenceLevel;
using spade::WorldBuilder;
using spade::WorldInstanceDesc;
using spade::WorldSetDesc;
using spade::sensors::ImuSample;
using spade::sensors::kRingDepth;
using spade::sensors::SampleIndex;

constexpr float kG = 9.80665f;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] " << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

// A world with no geometry, no turbulence and no contact: the only stochastic system in it is the sensor under test.
[[nodiscard]] spade::Result<WorldSetDesc> void_world_set(uint32_t bodies, uint32_t sensors, uint64_t seed) {
    Environment env;
    env.gravity = glm::vec3(0.0f, -kG, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    Capacities caps;
    caps.bodies = bodies;
    caps.force_elements = 1;
    caps.sensors = sensors;
    caps.contacts = 1;
    const spade::Result<spade::WorldDesc> world = WorldBuilder().name("void").environment(env).capacities(caps).build();
    if (!world) return std::unexpected(world.error());
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = seed;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    spade::physics::ContactParams contacts;
    contacts.restitution_e = 0.0f;
    contacts.friction_mu = 0.0f;
    contacts.proxy_radius = 0.0f;
    instance.contacts = contacts;
    spade::physics::GridParams grid;
    grid.cell_size = 1.0f;
    instance.grid = grid;
    return WorldSetDesc{{instance}};
}

[[nodiscard]] BodySpawn unit_body() {
    BodySpawn body;
    body.pos = glm::vec3(0.0f, 100.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    return body;
}

// A reseed seed for the k-th episode: spread, so that neighbouring k are not neighbouring seeds.
[[nodiscard]] uint64_t episode_seed(uint32_t k) { return 0x9E3779B97F4A7C15ULL * (static_cast<uint64_t>(k) + 1u) ^ 0xD1B54A32D192ED03ULL; }

struct Moments {
    double mean = 0.0;
    double sd = 0.0;
};
[[nodiscard]] Moments moments(const std::vector<double>& x) {
    Moments m;
    for (const double v : x) m.mean += v;
    m.mean /= static_cast<double>(x.size());
    double acc = 0.0;
    for (const double v : x) acc += (v - m.mean) * (v - m.mean);
    m.sd = std::sqrt(acc / static_cast<double>(x.size() - 1));
    return m;
}

[[nodiscard]] double correlation(const std::vector<double>& a, const std::vector<double>& b) {
    const Moments ma = moments(a);
    const Moments mb = moments(b);
    double acc = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) acc += (a[i] - ma.mean) * (b[i] - mb.mean);
    return acc / (static_cast<double>(a.size() - 1) * ma.sd * mb.sd);
}

// Steps `steps` times polling every `block` steps, returning every sample. Fails the test if the ring dropped any.
void collect(Simulation& sim, ImuSensorRef ref, uint64_t steps, std::vector<ImuSample>& out) {
    constexpr uint64_t kBlock = 32;  // < kRingDepth: nothing is ever dropped
    std::vector<ImuSample> buffer(kRingDepth);
    SampleIndex since = 0;
    for (uint64_t done = 0; done < steps; done += kBlock) {
        for (uint64_t i = 0; i < kBlock; ++i) ASSERT_OK(sim.step(1));
        const spade::Result<ImuPoll> poll = sim.poll_imu(ref, since, buffer);
        ASSERT_OK(poll);
        ASSERT_EQ(poll->dropped, 0u);
        out.insert(out.end(), poll->samples.begin(), poll->samples.end());
        since = poll->next_since;
    }
}

// One simulation, one free-falling unit body (specific force 0, no spin: a noise-free IMU reads exactly its bias) and one IMU.
struct ImuRig {
    spade::Result<Simulation> sim = std::unexpected(spade::Error{});
    BodyRef body{};
    ImuSensorRef imu{};
};
[[nodiscard]] bool make_imu_rig(ImuRig& rig, const ImuSensorSpawn& spawn, uint64_t seed, const BodySpawn& body = unit_body()) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, seed);
    if (!set) return false;
    rig.sim = Simulation::create(*set, 1'000'000, 1);
    if (!rig.sim) return false;
    const spade::Result<BodyRef> ref = rig.sim->spawn(0, body);
    if (!ref) return false;
    rig.body = *ref;
    const spade::Result<ImuSensorRef> imu = rig.sim->add_imu_sensor(rig.body, spawn);
    if (!imu) return false;
    rig.imu = *imu;
    return static_cast<bool>(rig.sim->flush_structural());
}

[[nodiscard]] const spade::sensors::ImuSensorRow& imu_row(const ImuRig& rig) { return **rig.sim->imu_sensor(rig.imu); }

// The body held stationary against gravity (the force of m*g upward, applied before EVERY step): the sensor reads +g on y and whatever spin the body has.
[[nodiscard]] std::vector<ImuSample> held_samples(ImuRig& rig, uint32_t steps) {
    for (uint32_t i = 0; i < steps; ++i) {
        EXPECT_OK(rig.sim->apply_wrench(rig.body, glm::vec3(0.0f, kG, 0.0f), glm::vec3(0.0f)));
        EXPECT_OK(rig.sim->step(1));
    }
    std::vector<ImuSample> buffer(kRingDepth);
    const spade::Result<ImuPoll> poll = rig.sim->poll_imu(rig.imu, 0, buffer);
    if (!poll) return {};
    return std::vector<ImuSample>(poll->samples.begin(), poll->samples.end());
}

}  // namespace

// ===========================================================================
// THE IMU'S TURN-ON OFFSET
// ===========================================================================

// The offset is a standard normal times the stated sigma, per axis and per channel, and each reseed draws a fresh one: over 3,000 episodes (9,000 draws a
// channel) the mean is 0 to within 5 standard errors and the deviation is the sigma to within 5 standard errors of a deviation (0.0075 sigma, so 4%), and
// the six components are uncorrelated to within 5/sqrt(3000) = 0.09.
TEST(ImuTurnOn, TheOffsetsOverManySeedsHaveTheStatedMeanAndSigma) {
    ImuSensorSpawn spawn;
    spawn.turn_on_sigma_a = 0.20f;  // m/s^2
    spawn.turn_on_sigma_g = 0.02f;  // rad/s
    ImuRig rig;
    ASSERT_TRUE(make_imu_rig(rig, spawn, 0x5EED0001ULL));

    constexpr uint32_t kEpisodes = 3000;
    std::vector<double> a[3];
    std::vector<double> g[3];
    for (uint32_t k = 0; k < kEpisodes; ++k) {
        ASSERT_OK(rig.sim->reseed(episode_seed(k)));
        const spade::sensors::ImuSensorRow& row = imu_row(rig);
        for (int axis = 0; axis < 3; ++axis) {
            a[axis].push_back(static_cast<double>(row.bias_a[axis]));
            g[axis].push_back(static_cast<double>(row.bias_g[axis]));
        }
    }
    const double n = static_cast<double>(kEpisodes);
    for (int axis = 0; axis < 3; ++axis) {
        const Moments ma = moments(a[axis]);
        const Moments mg = moments(g[axis]);
        EXPECT_NEAR(ma.mean, 0.0, 5.0 * 0.20 / std::sqrt(n)) << "accel axis " << axis;
        EXPECT_NEAR(ma.sd, 0.20, 0.04 * 0.20) << "accel axis " << axis;
        EXPECT_NEAR(mg.mean, 0.0, 5.0 * 0.02 / std::sqrt(n)) << "gyro axis " << axis;
        EXPECT_NEAR(mg.sd, 0.02, 0.04 * 0.02) << "gyro axis " << axis;
    }
    const double bound = 5.0 / std::sqrt(n);
    EXPECT_LT(std::fabs(correlation(a[0], a[1])), bound);
    EXPECT_LT(std::fabs(correlation(a[0], a[2])), bound);
    EXPECT_LT(std::fabs(correlation(a[1], a[2])), bound);
    EXPECT_LT(std::fabs(correlation(g[0], g[1])), bound);
    EXPECT_LT(std::fabs(correlation(a[0], g[0])), bound) << "the two channels draw different normals";
}

// The offset reaches the output: a free-falling body reads zero specific force, so a noise-free sensor's first sample IS its turn-on offset.
TEST(ImuTurnOn, ReachesTheFirstSample) {
    ImuSensorSpawn spawn;
    spawn.turn_on_sigma_a = 0.20f;
    spawn.turn_on_sigma_g = 0.02f;
    ImuRig rig;
    ASSERT_TRUE(make_imu_rig(rig, spawn, 0x5EED0002ULL));
    ASSERT_OK(rig.sim->reseed(episode_seed(7)));
    const glm::vec3 bias_a = imu_row(rig).bias_a;
    const glm::vec3 bias_g = imu_row(rig).bias_g;
    ASSERT_GT(glm::length(bias_a), 0.0f) << "a turn-on sigma above 0 draws a non-zero offset";

    std::vector<ImuSample> samples;
    collect(*rig.sim, rig.imu, 32, samples);
    ASSERT_FALSE(samples.empty());
    for (int axis = 0; axis < 3; ++axis) {
        EXPECT_NEAR(samples.front().accel[axis], bias_a[axis], 1.0e-5f) << "axis " << axis;
        EXPECT_NEAR(samples.front().gyro[axis], bias_g[axis], 1.0e-6f) << "axis " << axis;
    }
}

// The offset is a function of the seed and the sensor's slot alone: the same reseed gives the same bytes in any simulation and any number of times, another
// seed gives another offset, and taking it never moves the sensor's own noise stream (so a sensor without one keeps every digest).
TEST(ImuTurnOn, IsAFunctionOfTheSeedAloneAndLeavesTheNoiseStreamWhereItWas) {
    ImuSensorSpawn with_on;
    with_on.sigma_a = 0.05f;
    with_on.turn_on_sigma_a = 0.20f;
    with_on.turn_on_sigma_g = 0.02f;
    ImuSensorSpawn without_on = with_on;
    without_on.turn_on_sigma_a = 0.0f;
    without_on.turn_on_sigma_g = 0.0f;

    ImuRig one;
    ImuRig two;
    ImuRig plain;
    ASSERT_TRUE(make_imu_rig(one, with_on, 0x5EED0003ULL));
    ASSERT_TRUE(make_imu_rig(two, with_on, 0x5EED0003ULL));
    ASSERT_TRUE(make_imu_rig(plain, without_on, 0x5EED0003ULL));

    ASSERT_OK(one.sim->reseed(episode_seed(11)));
    ASSERT_OK(two.sim->reseed(episode_seed(11)));
    ASSERT_OK(plain.sim->reseed(episode_seed(11)));
    EXPECT_EQ(imu_row(one).bias_a, imu_row(two).bias_a);
    EXPECT_EQ(imu_row(one).bias_g, imu_row(two).bias_g);
    EXPECT_NE(imu_row(one).bias_a, glm::vec3(0.0f));

    const glm::vec3 first = imu_row(one).bias_a;
    ASSERT_OK(one.sim->reseed(episode_seed(12)));
    EXPECT_NE(imu_row(one).bias_a, first) << "another seed, another offset";
    ASSERT_OK(one.sim->reseed(episode_seed(11)));
    EXPECT_EQ(imu_row(one).bias_a, first) << "the same seed again, the same offset";

    EXPECT_EQ(std::memcmp(&imu_row(one).noise, &imu_row(plain).noise, sizeof(spade::rng::Stream)), 0)
        << "the offset comes from its own stream: the sensor's own draw position is the same with and without it";
}

// A channel without a turn-on sigma keeps its history across a reseed while the other is redrawn, and a sensor with neither keeps all of it but its noise
// stream: a reseed of a defaults-off row writes only `noise` (the pre-existing contract, which no digest of such a row may move past).
TEST(ImuTurnOn, AChannelWithoutASigmaKeepsItsHistoryAndADefaultsOffRowKeepsAllOfIt) {
    ImuSensorSpawn walking;
    walking.sigma_ba = 0.01f;
    walking.sigma_bg = 0.001f;
    walking.turn_on_sigma_a = 0.20f;  // accel redrawn at a reseed; gyro (no sigma) not
    ImuRig half;
    ASSERT_TRUE(make_imu_rig(half, walking, 0x5EED0004ULL));
    std::vector<ImuSample> samples;
    collect(*half.sim, half.imu, 64, samples);
    const glm::vec3 gyro_history = imu_row(half).bias_g;
    const glm::vec3 accel_before = imu_row(half).bias_a;
    ASSERT_NE(gyro_history, glm::vec3(0.0f)) << "the gyro bias has walked";
    ASSERT_OK(half.sim->reseed(episode_seed(21)));
    EXPECT_EQ(imu_row(half).bias_g, gyro_history) << "no gyro turn-on sigma: its bias is history";
    EXPECT_NE(imu_row(half).bias_a, accel_before) << "the accel channel starts afresh";

    ImuSensorSpawn off;
    off.sigma_ba = 0.01f;
    off.sigma_bg = 0.001f;
    ImuRig plain;
    ASSERT_TRUE(make_imu_rig(plain, off, 0x5EED0005ULL));
    std::vector<ImuSample> more;
    collect(*plain.sim, plain.imu, 64, more);
    const spade::sensors::ImuSensorRow before = imu_row(plain);
    ASSERT_OK(plain.sim->reseed(episode_seed(22)));
    const spade::sensors::ImuSensorRow& after = imu_row(plain);
    EXPECT_EQ(after.bias_a, before.bias_a);
    EXPECT_EQ(after.bias_g, before.bias_g);
    EXPECT_EQ(after.phase, before.phase);
    EXPECT_EQ(after.last_index, before.last_index);
    EXPECT_NE(std::memcmp(&after.noise, &before.noise, sizeof(spade::rng::Stream)), 0) << "the noise stream is re-derived";
}

// ===========================================================================
// THE IMU'S RANGE AND QUANTUM
// ===========================================================================

// Past its full scale a channel reads exactly +-range, on either side; inside it the value is untouched.
TEST(ImuConverter, TheRangeSaturatesAtExactlyPlusAndMinusRange) {
    for (const float spin : {3.0f, -3.0f}) {
        BodySpawn body = unit_body();
        body.omega_body = glm::vec3(0.0f, 0.0f, spin);
        ImuSensorSpawn spawn;
        spawn.range_a = 5.0f;  // m/s^2: the body is held at +g = 9.80665, past it
        spawn.range_g = 2.0f;  // rad/s: the spin is 3, past it
        ImuRig rig;
        ASSERT_TRUE(make_imu_rig(rig, spawn, 0x5EED0006ULL, body));
        const std::vector<ImuSample> samples = held_samples(rig, 8);
        ASSERT_FALSE(samples.empty());
        EXPECT_EQ(samples.back().gyro.z, spin > 0.0f ? 2.0f : -2.0f) << "spin " << spin;
        EXPECT_EQ(samples.back().gyro.x, 0.0f);
        EXPECT_EQ(samples.back().accel.y, 5.0f) << "held at +g, past a range of 5";
    }
    BodySpawn body = unit_body();
    body.omega_body = glm::vec3(0.0f, 0.0f, 3.0f);
    ImuSensorSpawn roomy;
    roomy.range_a = 16.0f * kG;
    roomy.range_g = 10.0f;
    ImuRig rig;
    ASSERT_TRUE(make_imu_rig(rig, roomy, 0x5EED0007ULL, body));
    const std::vector<ImuSample> samples = held_samples(rig, 8);
    ASSERT_FALSE(samples.empty());
    EXPECT_EQ(samples.back().gyro.z, 3.0f) << "inside the range the value is untouched";
    EXPECT_NEAR(samples.back().accel.y, kG, 1.0e-4f);
}

// The converter's lattice and its error. Two simulations of the same seed and slot: the first clamps only (a range, no bits), the second clamps and rounds
// (8 bits). Their noise is the same bytes, so the difference of the two readings is the rounding error alone. With a range of 16 m/s^2 and 8 bits the quantum is
// 32 / 256 = 0.125 m/s^2; with 4 rad/s it is 8 / 256 = 0.03125 rad/s. Every rounded reading is a whole number of quanta; the error never passes half a quantum;
// its mean is 0 and its deviation is quantum / sqrt(12) (0.0361 and 0.00902), each to within 3%, over 60,000 pairs.
TEST(ImuConverter, QuantisedReadingsLieOnTheLatticeAndTheRoundingErrorIsUniform) {
    ImuSensorSpawn clamp_only;
    clamp_only.sigma_a = 0.5f;
    clamp_only.sigma_g = 0.1f;
    clamp_only.range_a = 16.0f;
    clamp_only.range_g = 4.0f;
    ImuSensorSpawn rounded = clamp_only;
    rounded.adc_bits = 8.0f;
    ImuRig a;
    ImuRig b;
    ASSERT_TRUE(make_imu_rig(a, clamp_only, 0x5EED0008ULL));
    ASSERT_TRUE(make_imu_rig(b, rounded, 0x5EED0008ULL));
    constexpr uint64_t kSteps = 20000;
    std::vector<ImuSample> sa;
    std::vector<ImuSample> sb;
    collect(*a.sim, a.imu, kSteps, sa);
    collect(*b.sim, b.imu, kSteps, sb);
    ASSERT_EQ(sa.size(), sb.size());
    ASSERT_GE(sa.size(), kSteps);

    constexpr double kQa = 0.125;
    constexpr double kQg = 0.03125;
    std::vector<double> ea;
    std::vector<double> eg;
    for (std::size_t i = 0; i < sa.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            const double xa = static_cast<double>(sb[i].accel[axis]);
            const double xg = static_cast<double>(sb[i].gyro[axis]);
            EXPECT_EQ(std::fmod(xa, kQa), 0.0) << "accel sample " << i << " axis " << axis << " is " << xa;
            EXPECT_EQ(std::fmod(xg, kQg), 0.0) << "gyro sample " << i << " axis " << axis << " is " << xg;
            ea.push_back(xa - static_cast<double>(sa[i].accel[axis]));
            eg.push_back(xg - static_cast<double>(sa[i].gyro[axis]));
            if (testing::Test::HasFailure()) return;  // one lattice failure is the finding; not 60,000 lines of it
        }
    }
    for (const double e : ea) ASSERT_LE(std::fabs(e), 0.5 * kQa + 1.0e-6);
    for (const double e : eg) ASSERT_LE(std::fabs(e), 0.5 * kQg + 1.0e-7);
    const Moments ma = moments(ea);
    const Moments mg = moments(eg);
    const double n = static_cast<double>(ea.size());
    EXPECT_NEAR(ma.mean, 0.0, 5.0 * (kQa / std::sqrt(12.0)) / std::sqrt(n));
    EXPECT_NEAR(ma.sd, kQa / std::sqrt(12.0), 0.03 * kQa / std::sqrt(12.0));
    EXPECT_NEAR(mg.mean, 0.0, 5.0 * (kQg / std::sqrt(12.0)) / std::sqrt(n));
    EXPECT_NEAR(mg.sd, kQg / std::sqrt(12.0), 0.03 * kQg / std::sqrt(12.0));
}

// All zero is the old sensor: a spawn that states the new fields as zeros produces the same bytes as one that does not state them, sample for sample.
TEST(ImuConverter, AllZeroIsTheSensorItWasBeforeThem) {
    ImuSensorSpawn plain;
    plain.sigma_a = 0.05f;
    plain.sigma_g = 0.01f;
    plain.sigma_ba = 0.001f;
    ImuSensorSpawn zeros = plain;
    zeros.range_a = 0.0f;
    zeros.range_g = 0.0f;
    zeros.adc_bits = 0.0f;
    zeros.turn_on_sigma_a = 0.0f;
    zeros.turn_on_sigma_g = 0.0f;
    ImuRig a;
    ImuRig b;
    ASSERT_TRUE(make_imu_rig(a, plain, 0x5EED0009ULL));
    ASSERT_TRUE(make_imu_rig(b, zeros, 0x5EED0009ULL));
    std::vector<ImuSample> sa;
    std::vector<ImuSample> sb;
    collect(*a.sim, a.imu, 640, sa);
    collect(*b.sim, b.imu, 640, sb);
    ASSERT_EQ(sa.size(), sb.size());
    for (std::size_t i = 0; i < sa.size(); ++i) {
        EXPECT_EQ(std::memcmp(&sa[i], &sb[i], sizeof(ImuSample)), 0) << "sample " << i;
    }
    EXPECT_EQ(std::memcmp(&imu_row(a), &imu_row(b), sizeof(spade::sensors::ImuSensorRow)), 0) << "the rows are byte-identical";
}

// Each bad field is refused by name, at the add_ call.
TEST(ImuConverter, EachBadFieldIsRefusedByName) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, 0x5EED000AULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    const auto refused = [&](const char* field, auto&& edit) {
        ImuSensorSpawn spawn;
        edit(spawn);
        const spade::Result<ImuSensorRef> r = sim->add_imu_sensor(*body, spawn);
        ASSERT_FALSE(r.has_value()) << field << " was accepted";
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << field;
        EXPECT_NE(r.error().context.find(field), std::string::npos) << field << " is not named in: " << r.error().context;
    };
    const float nan = std::nanf("");
    refused("range_a", [](ImuSensorSpawn& s) { s.range_a = -1.0f; });
    refused("range_a", [&](ImuSensorSpawn& s) { s.range_a = nan; });
    refused("range_g", [](ImuSensorSpawn& s) { s.range_g = -0.5f; });
    refused("range_g", [&](ImuSensorSpawn& s) { s.range_g = nan; });
    refused("adc_bits", [](ImuSensorSpawn& s) { s.range_a = 16.0f; s.adc_bits = 8.5f; });
    refused("adc_bits", [](ImuSensorSpawn& s) { s.range_a = 16.0f; s.adc_bits = 25.0f; });
    refused("adc_bits", [](ImuSensorSpawn& s) { s.range_a = 16.0f; s.adc_bits = -1.0f; });
    refused("adc_bits", [](ImuSensorSpawn& s) { s.adc_bits = 8.0f; });  // bits with no range on either channel quantise nothing: a caller's mistake
    refused("turn_on_sigma_a", [](ImuSensorSpawn& s) { s.turn_on_sigma_a = -0.1f; });
    refused("turn_on_sigma_g", [&](ImuSensorSpawn& s) { s.turn_on_sigma_g = nan; });
}

// ===========================================================================
// THE GNSS TURN-ON OFFSET
// ===========================================================================

namespace {
struct GnssRig {
    spade::Result<Simulation> sim = std::unexpected(spade::Error{});
    BodyRef body{};
    GnssSensorRef gnss{};
};
[[nodiscard]] bool make_gnss_rig(GnssRig& rig, const GnssSensorSpawn& spawn, uint64_t seed) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, seed);
    if (!set) return false;
    rig.sim = Simulation::create(*set, 1'000'000, 1);
    if (!rig.sim) return false;
    const spade::Result<BodyRef> ref = rig.sim->spawn(0, unit_body());
    if (!ref) return false;
    rig.body = *ref;
    const spade::Result<GnssSensorRef> gnss = rig.sim->add_gnss_sensor(rig.body, spawn);
    if (!gnss) return false;
    rig.gnss = *gnss;
    return static_cast<bool>(rig.sim->flush_structural());
}
[[nodiscard]] const spade::sensors::GnssSensorRow& gnss_row(const GnssRig& rig) { return **rig.sim->gnss_sensor(rig.gnss); }
}  // namespace

// With the flag set, the bias starts at sigma_bias times a standard normal per axis, afresh at every reseed: over 3,000 episodes (3,000 draws an axis) the mean
// is 0 to within 5 standard errors and the deviation is the sigma_bias (3 m) to within 5 standard errors of a deviation (0.013 sigma, so 6.5%).
TEST(GnssTurnOn, TheBiasStartsAtSigmaBiasPerAxisAndIsRedrawnAtEveryReseed) {
    GnssSensorSpawn spawn;
    spawn.rate_divider = 100;
    spawn.bias_tau_s = 100.0f;
    spawn.sigma_bias = 3.0f;
    spawn.turn_on_bias = 1;
    GnssRig rig;
    ASSERT_TRUE(make_gnss_rig(rig, spawn, 0x5EED0011ULL));

    constexpr uint32_t kEpisodes = 3000;
    std::vector<double> axis[3];
    for (uint32_t k = 0; k < kEpisodes; ++k) {
        ASSERT_OK(rig.sim->reseed(episode_seed(k)));
        for (int i = 0; i < 3; ++i) axis[i].push_back(static_cast<double>(gnss_row(rig).bias[i]));
    }
    const double n = static_cast<double>(kEpisodes);
    for (int i = 0; i < 3; ++i) {
        const Moments m = moments(axis[i]);
        EXPECT_NEAR(m.mean, 0.0, 5.0 * 3.0 / std::sqrt(n)) << "axis " << i;
        EXPECT_NEAR(m.sd, 3.0, 0.065 * 3.0) << "axis " << i;
    }
    EXPECT_LT(std::fabs(correlation(axis[0], axis[1])), 5.0 / std::sqrt(n));
    EXPECT_LT(std::fabs(correlation(axis[1], axis[2])), 5.0 / std::sqrt(n));
}

// Without the flag nothing about the bias changes (it starts at 0 and a reseed leaves its history), and the noise stream is where it was with the flag.
TEST(GnssTurnOn, WithoutTheFlagAReseedLeavesTheBiasAndTheStreamIsUnmoved) {
    GnssSensorSpawn off;
    off.rate_divider = 10;
    off.bias_tau_s = 100.0f;
    off.sigma_bias = 3.0f;
    GnssSensorSpawn on = off;
    on.turn_on_bias = 1;

    GnssRig plain;
    GnssRig flagged;
    ASSERT_TRUE(make_gnss_rig(plain, off, 0x5EED0012ULL));
    ASSERT_TRUE(make_gnss_rig(flagged, on, 0x5EED0012ULL));
    EXPECT_EQ(gnss_row(plain).bias, glm::vec3(0.0f)) << "without the flag the bias starts at 0";

    ASSERT_OK(plain.sim->step(200));
    const glm::vec3 history = gnss_row(plain).bias;
    ASSERT_NE(history, glm::vec3(0.0f)) << "the bias has been driven";
    ASSERT_OK(plain.sim->reseed(episode_seed(31)));
    ASSERT_OK(flagged.sim->reseed(episode_seed(31)));
    EXPECT_EQ(gnss_row(plain).bias, history) << "a reseed without the flag leaves the bias";
    EXPECT_NE(gnss_row(flagged).bias, glm::vec3(0.0f));
    EXPECT_EQ(std::memcmp(&gnss_row(plain).noise, &gnss_row(flagged).noise, sizeof(spade::rng::Stream)), 0)
        << "the offset comes from its own stream: the noise stream is the same with and without the flag";
}

TEST(GnssTurnOn, TheFlagIsZeroOrOne) {
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, 0x5EED0013ULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1);
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);
    GnssSensorSpawn spawn;
    spawn.turn_on_bias = 2;
    const spade::Result<GnssSensorRef> r = sim->add_gnss_sensor(*body, spawn);
    ASSERT_FALSE(r.has_value()) << "turn_on_bias 2 was accepted";
    EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
    EXPECT_NE(r.error().context.find("turn_on_bias"), std::string::npos) << r.error().context;
}

// ===========================================================================
// A VULKAN WORLD REFUSES THE CONVERTER (the GPU twin has none; the turn-on is an initial state and needs no refusal)
// ===========================================================================

TEST(GpuSensorErrors, AVulkanWorldRefusesARangeOrBitsByNameAndAcceptsATurnOn) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::Result<WorldSetDesc> set = void_world_set(1, 1, 0x5EED0014ULL);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 1, spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, unit_body());
    ASSERT_OK(body);

    const auto refused = [&](const char* field, auto&& edit) {
        ImuSensorSpawn spawn;
        edit(spawn);
        const spade::Result<ImuSensorRef> r = sim->add_imu_sensor(*body, spawn);
        ASSERT_FALSE(r.has_value()) << field << " was accepted on a Vulkan world";
        EXPECT_EQ(r.error().code, spade::Code::unavailable) << field;
        EXPECT_NE(r.error().context.find(field), std::string::npos) << r.error().context;
    };
    refused("range_a", [](ImuSensorSpawn& s) { s.range_a = 16.0f; });
    refused("range_g", [](ImuSensorSpawn& s) { s.range_g = 4.0f; });
    // bits alone is refused before it gets here (invalid_argument: it needs a range), so with a range the range is what a Vulkan world names first
    refused("range_a", [](ImuSensorSpawn& s) { s.range_a = 16.0f; s.adc_bits = 8.0f; });

    ImuSensorSpawn on;
    on.turn_on_sigma_a = 0.2f;
    EXPECT_OK(sim->add_imu_sensor(*body, on)) << "a turn-on offset is only an initial state: no refusal";
}
