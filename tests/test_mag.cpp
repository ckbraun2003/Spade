// ---------------------------------------------------------------------------
// test_mag.cpp -- the magnetometer (sensors/mag.hpp, the opt-in module `mag`, Phase 1 of the flight-computer work, O144): its layout, the field helper in
// Physics' world frame (x north, y up, z east), the truth rotation, the sensor in a world (the field in the sensor's axes, hard and soft iron, the emission
// clock, noise and its seeds, the turn-on bias and its reseed), the refusals, snapshot and restore, and what the module is not.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "sensors/mag.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

namespace {

using spade::BodyRef;
using spade::BodySpawn;
using spade::Capacities;
using spade::Environment;
using spade::RowRef;
using spade::Simulation;
using spade::TurbulenceLevel;
using spade::WorldBuilder;
using spade::WorldInstanceDesc;
using spade::WorldSetDesc;
using spade::sensors::MagSample;
using spade::sensors::MagSensorRow;
using spade::sensors::MagSensorSpawn;

constexpr float kHalfPi = 1.57079632679f;

#define ASSERT_OK(expr)                                                                                  \
    do {                                                                                                 \
        const auto& r_ = (expr);                                                                         \
        ASSERT_TRUE(r_.has_value()) << #expr << " failed: " << (r_ ? "" : r_.error().context);          \
    } while (false)

[[nodiscard]] spade::modules::ModuleSet standard_plus_mag() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::mag_module());
    return set;
}

[[nodiscard]] spade::Result<WorldSetDesc> still_world(uint64_t seed) {
    Environment env;
    env.gravity = glm::vec3(0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    Capacities caps;
    caps.bodies = 4;
    caps.force_elements = 1;
    caps.sensors = 2;
    caps.contacts = 1;
    const spade::Result<spade::WorldDesc> world = WorldBuilder().name("still").environment(env).capacities(caps).build();
    if (!world) return std::unexpected(world.error());
    spade::physics::ContactParams contacts;
    contacts.restitution_e = 0.0f;
    contacts.friction_mu = 0.0f;
    contacts.proxy_radius = 0.0f;
    spade::physics::GridParams grid;
    grid.cell_size = 1.0f;
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = seed;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts = contacts;
    instance.grid = grid;
    return WorldSetDesc{{instance}};
}

struct Rig {
    Simulation sim;
    BodyRef body;
    RowRef row;
    spade::sensors::SampleIndex since = 0;
};

[[nodiscard]] std::optional<Rig> make_rig(const MagSensorSpawn& spawn, const glm::quat& body_orient = glm::quat(1, 0, 0, 0),
                                          uint64_t seed = 0x5EEDu, std::string* why = nullptr) {
    const spade::Result<WorldSetDesc> set = still_world(seed);
    if (!set) return std::nullopt;
    spade::Result<Simulation> sim = Simulation::create(*set, 2'000'000, 2, {}, standard_plus_mag());
    if (!sim) return std::nullopt;
    BodySpawn b;
    b.orient = body_orient;
    const spade::Result<BodyRef> body = sim->spawn(0, b);
    if (!body) return std::nullopt;
    const spade::Result<RowRef> row = sim->attach_row(*body, "mag_sensors", spawn);
    if (!row) {
        if (why) *why = row.error().context;
        return std::nullopt;
    }
    if (!sim->flush_structural()) return std::nullopt;
    return Rig{std::move(*sim), *body, *row, 0};
}

[[nodiscard]] std::vector<MagSample> poll(const Simulation& sim, const RowRef& row, spade::sensors::SampleIndex since) {
    const auto rows = sim.module_rows<MagSensorRow>("mag_sensors", row.world_index);
    const auto ring = sim.module_rows<MagSample>("mag_ring", row.world_index);
    if (!rows || !ring) return {};
    std::vector<MagSample> out(spade::sensors::kRingDepth);
    const auto window = std::span<const MagSample>(*ring).subspan(std::size_t{row.slot} * spade::sensors::kRingDepth,
                                                                   spade::sensors::kRingDepth);
    const auto r = spade::sensors::ring_poll<MagSample>(window, (*rows)[row.slot].last_index, since, out);
    out.resize(r.samples.size());
    return out;
}

[[nodiscard]] std::vector<MagSample> run(Rig& rig, uint64_t steps) {
    std::vector<MagSample> all;
    for (uint64_t t = 0; t < steps; ++t) {
        EXPECT_TRUE(rig.sim.step(1).has_value());
        for (const MagSample& s : poll(rig.sim, rig.row, rig.since)) {
            all.push_back(s);
            rig.since = s.index;
        }
    }
    return all;
}

void expect_vec(const glm::vec3& got, const glm::vec3& want, float tol, const char* what) {
    EXPECT_NEAR(got.x, want.x, tol) << what << " x";
    EXPECT_NEAR(got.y, want.y, tol) << what << " y";
    EXPECT_NEAR(got.z, want.z, tol) << what << " z";
}

}  // namespace

// --- layout, the helper, the truth rotation ----------------------------------

TEST(MagLayout, TheRowAndTheSampleAreTheStd430RowsTheyClaim) {
    EXPECT_EQ(sizeof(MagSensorRow), 176u);
    EXPECT_EQ(sizeof(MagSample), 32u);
    EXPECT_EQ(offsetof(MagSensorRow, noise), 144u);
    EXPECT_EQ(spade::sensors::sensor_kind::mag, 5u);
    EXPECT_EQ(spade::sensors::sensor_kind::count, 6u);
    EXPECT_TRUE(spade::sensors::sensor_kind::is_live(spade::sensors::sensor_kind::mag));
}

TEST(MagField, IsInPhysicsFrameXNorthYUpZEast) {
    expect_vec(spade::sensors::mag_field_world(0.0f, 0.0f, 50.0f), glm::vec3(50, 0, 0), 1e-3f, "north, level");
    expect_vec(spade::sensors::mag_field_world(0.0f, kHalfPi, 50.0f), glm::vec3(0, -50, 0), 1e-3f, "straight down is -y");
    expect_vec(spade::sensors::mag_field_world(kHalfPi, 0.0f, 50.0f), glm::vec3(0, 0, 50), 1e-3f, "declination east is +z");
    // The accepted doc's vector: NED (0.2, 0, 0.4) gauss = (20, 0, 40) uT = world (20, -40, 0).
    const float strength = std::sqrt(20.0f * 20.0f + 40.0f * 40.0f);
    const float inclination = std::atan2(40.0f, 20.0f);
    expect_vec(spade::sensors::mag_field_world(0.0f, inclination, strength), glm::vec3(20, -40, 0), 2e-3f, "the doc's vector");
}

TEST(MagField, FollowsTheWorldsOwnFrameConvention) {
    using spade::sensors::MagWorldFrame;
    // The same place, (N, E, D) = (20, 0, 40) uT, in the two conventions: compass (N, -D, E) and anchored (E, -D, -N).
    const float strength = std::sqrt(20.0f * 20.0f + 40.0f * 40.0f);
    const float inclination = std::atan2(40.0f, 20.0f);
    expect_vec(spade::sensors::mag_field_world(0.0f, inclination, strength, MagWorldFrame::compass_x_north), glm::vec3(20, -40, 0), 2e-3f, "compass");
    expect_vec(spade::sensors::mag_field_world(0.0f, inclination, strength, MagWorldFrame::anchored_world), glm::vec3(0, -40, -20), 2e-3f, "anchored");
    // an east-pointing field (declination 90 degrees, level): compass +z, anchored +x
    expect_vec(spade::sensors::mag_field_world(kHalfPi, 0.0f, 50.0f, MagWorldFrame::compass_x_north), glm::vec3(0, 0, 50), 1e-3f, "east, compass");
    expect_vec(spade::sensors::mag_field_world(kHalfPi, 0.0f, 50.0f, MagWorldFrame::anchored_world), glm::vec3(50, 0, 0), 1e-3f, "east, anchored");
}

TEST(MagTruth, IsTheWorldFieldInTheSensorsAxes) {
    const glm::vec3 field(20.0f, -40.0f, 0.0f);
    const glm::quat id(1, 0, 0, 0);
    expect_vec(spade::sensors::mag_truth(id, id, field), field, 1e-5f, "level, unmounted");
    const glm::quat yaw90 = glm::angleAxis(kHalfPi, glm::vec3(0, 1, 0));  // x -> -z, z -> +x
    // world north (+x) lies along this body's +z axis
    expect_vec(spade::sensors::mag_truth(yaw90, id, glm::vec3(20, 0, 0)), glm::vec3(0, 0, 20), 1e-4f, "yawed body");
    // body and mount both yawed: a half turn, so north is the sensor's -x
    expect_vec(spade::sensors::mag_truth(yaw90, yaw90, glm::vec3(20, 0, 0)), glm::vec3(-20, 0, 0), 1e-4f, "yawed body and mount");
}

// --- the sensor in a world ---------------------------------------------------

TEST(Mag, ReadsTheWorldFieldInItsAxes) {
    MagSensorSpawn s;
    s.field_world_ut = glm::vec3(20.0f, -40.0f, 0.0f);
    std::optional<Rig> rig = make_rig(s);
    ASSERT_TRUE(rig.has_value());
    const std::vector<MagSample> got = run(*rig, 1);
    ASSERT_EQ(got.size(), 2u);
    EXPECT_EQ(got[0].index, 1u);
    EXPECT_EQ(got[0].tick, 0u);
    expect_vec(got[0].field_body_ut, glm::vec3(20, -40, 0), 1e-4f, "level");

    std::optional<Rig> yawed = make_rig(s, glm::angleAxis(kHalfPi, glm::vec3(0, 1, 0)));
    ASSERT_TRUE(yawed.has_value());
    const std::vector<MagSample> gy = run(*yawed, 1);
    ASSERT_FALSE(gy.empty());
    expect_vec(gy[0].field_body_ut, glm::vec3(0, -40, 20), 1e-4f, "yawed 90 degrees about up");
}

TEST(Mag, HardIronAddsAndSoftIronMultiplies) {
    MagSensorSpawn s;
    s.field_world_ut = glm::vec3(20.0f, -40.0f, 0.0f);
    s.soft_iron = glm::mat3(2.0f);
    s.hard_iron_ut = glm::vec3(1.0f, 2.0f, 3.0f);
    std::optional<Rig> rig = make_rig(s);
    ASSERT_TRUE(rig.has_value());
    const std::vector<MagSample> got = run(*rig, 1);
    ASSERT_FALSE(got.empty());
    expect_vec(got[0].field_body_ut, glm::vec3(41, -78, 3), 1e-4f, "2 * B + h");

    MagSensorSpawn rot;  // a soft-iron that swaps x and y: columns (0,1,0), (1,0,0), (0,0,1)
    rot.field_world_ut = glm::vec3(20.0f, -40.0f, 5.0f);
    rot.soft_iron = glm::mat3(glm::vec3(0, 1, 0), glm::vec3(1, 0, 0), glm::vec3(0, 0, 1));
    std::optional<Rig> r2 = make_rig(rot);
    ASSERT_TRUE(r2.has_value());
    const std::vector<MagSample> g2 = run(*r2, 1);
    ASSERT_FALSE(g2.empty());
    expect_vec(g2[0].field_body_ut, glm::vec3(-40, 20, 5), 1e-4f, "the matrix acts on the field, columns as given");
}

TEST(Mag, EmitsOnItsOwnClockWithMonotonicIndicesAndTickStamps) {
    MagSensorSpawn s;
    s.rate_divider = 3u;
    s.field_world_ut = glm::vec3(20.0f, 0.0f, 0.0f);
    std::optional<Rig> rig = make_rig(s);
    ASSERT_TRUE(rig.has_value());
    const std::vector<MagSample> got = run(*rig, 6);
    ASSERT_EQ(got.size(), 4u);
    const uint64_t ticks[] = {1, 2, 4, 5};
    for (std::size_t i = 0; i < got.size(); ++i) {
        EXPECT_EQ(got[i].index, i + 1) << i;
        EXPECT_EQ(got[i].tick, ticks[i]) << i;
    }
}

TEST(Mag, WhiteNoiseHasItsSigmaPerAxis) {
    MagSensorSpawn s;
    s.field_world_ut = glm::vec3(20.0f, -40.0f, 0.0f);
    s.sigma_white_ut = 0.5f;
    std::optional<Rig> rig = make_rig(s);
    ASSERT_TRUE(rig.has_value());
    const std::vector<MagSample> got = run(*rig, 2500);
    ASSERT_GE(got.size(), 4000u);
    glm::dvec3 mean(0.0);
    for (const MagSample& g : got) mean += glm::dvec3(g.field_body_ut);
    mean /= static_cast<double>(got.size());
    glm::dvec3 var(0.0);
    for (const MagSample& g : got) {
        const glm::dvec3 d = glm::dvec3(g.field_body_ut) - mean;
        var += d * d;
    }
    var /= static_cast<double>(got.size());
    for (int axis = 0; axis < 3; ++axis) EXPECT_NEAR(std::sqrt(var[axis]), 0.5, 0.025) << axis;
    EXPECT_NEAR(mean.x, 20.0, 0.05);
    EXPECT_NEAR(mean.y, -40.0, 0.05);
}

TEST(Mag, TheSameSeedGivesTheSameBytesAndAnotherSeedDoesNot) {
    MagSensorSpawn s;
    s.field_world_ut = glm::vec3(20.0f, -40.0f, 0.0f);
    s.sigma_white_ut = 0.3f;
    s.sigma_bias_ut = 1.0f;
    s.bias_tau_s = 60.0f;
    std::optional<Rig> a = make_rig(s, glm::quat(1, 0, 0, 0), 7u);
    std::optional<Rig> b = make_rig(s, glm::quat(1, 0, 0, 0), 7u);
    std::optional<Rig> c = make_rig(s, glm::quat(1, 0, 0, 0), 8u);
    ASSERT_TRUE(a && b && c);
    const std::vector<MagSample> ga = run(*a, 50);
    const std::vector<MagSample> gb = run(*b, 50);
    const std::vector<MagSample> gc = run(*c, 50);
    ASSERT_EQ(ga.size(), gb.size());
    ASSERT_FALSE(ga.empty());
    EXPECT_EQ(std::memcmp(ga.data(), gb.data(), ga.size() * sizeof(MagSample)), 0);
    EXPECT_NE(std::memcmp(ga.data(), gc.data(), ga.size() * sizeof(MagSample)), 0);
}

TEST(Mag, TheTurnOnBiasIsAConstantOffsetPerSeedAndIsRedrawnAtEveryReseed) {
    MagSensorSpawn s;
    s.field_world_ut = glm::vec3(20.0f, -40.0f, 0.0f);
    s.sigma_bias_ut = 2.0f;
    s.bias_tau_s = 1.0e6f;
    std::optional<Rig> rig = make_rig(s, glm::quat(1, 0, 0, 0), 11u);
    ASSERT_TRUE(rig.has_value());
    const glm::vec3 truth(20.0f, -40.0f, 0.0f);
    const std::vector<MagSample> first = run(*rig, 3);
    ASSERT_GE(first.size(), 3u);
    const glm::vec3 offset = first[0].field_body_ut - truth;
    EXPECT_GT(glm::length(offset), 0.0f) << "a turn-on bias was drawn on three axes";
    for (const MagSample& g : first) expect_vec(g.field_body_ut - truth, offset, 1e-3f, "constant over the run");

    ASSERT_OK(rig->sim.reseed(99u));
    ASSERT_OK(rig->sim.step(1));
    const std::vector<MagSample> second = poll(rig->sim, rig->row, first.back().index);
    ASSERT_FALSE(second.empty());
    EXPECT_GT(glm::length((second[0].field_body_ut - truth) - offset), 0.1f) << "another seed, another offset";

    ASSERT_OK(rig->sim.reseed(99u));
    ASSERT_OK(rig->sim.step(1));
    const std::vector<MagSample> third = poll(rig->sim, rig->row, second.back().index);
    ASSERT_FALSE(third.empty());
    expect_vec(third[0].field_body_ut, second[0].field_body_ut, 1e-3f, "the same scene seed, the same offset");
}

// --- refusals ----------------------------------------------------------------

TEST(MagRefusals, EachBadFieldIsRefusedByName) {
    struct Case {
        const char* what;
        MagSensorSpawn s;
    };
    std::vector<Case> cases;
    auto add = [&](const char* what, auto edit) {
        MagSensorSpawn s;
        s.field_world_ut = glm::vec3(20.0f, -40.0f, 0.0f);
        edit(s);
        cases.push_back({what, s});
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    add("rate_divider 0", [](MagSensorSpawn& s) { s.rate_divider = 0; });
    add("a NaN field", [&](MagSensorSpawn& s) { s.field_world_ut.y = nan; });
    add("a NaN hard iron", [&](MagSensorSpawn& s) { s.hard_iron_ut.z = nan; });
    add("a NaN soft iron", [&](MagSensorSpawn& s) { s.soft_iron[1][1] = nan; });
    add("a non-unit mount orientation", [](MagSensorSpawn& s) { s.mount_orient = glm::quat(2, 0, 0, 0); });
    add("a NaN mount orientation", [&](MagSensorSpawn& s) { s.mount_orient = glm::quat(nan, 0, 0, 0); });
    add("a negative sigma_white", [](MagSensorSpawn& s) { s.sigma_white_ut = -1.0f; });
    add("a negative sigma_bias", [](MagSensorSpawn& s) { s.sigma_bias_ut = -1.0f; });
    add("a NaN tau", [&](MagSensorSpawn& s) { s.bias_tau_s = nan; });
    for (const Case& c : cases) {
        std::string why;
        const std::optional<Rig> rig = make_rig(c.s, glm::quat(1, 0, 0, 0), 0x5EEDu, &why);
        EXPECT_FALSE(rig.has_value()) << c.what << " was accepted";
        EXPECT_FALSE(why.empty()) << c.what << " has no message";
    }
}

// --- state, and what the module is not ---------------------------------------

TEST(Mag, ARestoredSnapshotContinuesWithTheSameBytes) {
    MagSensorSpawn s;
    s.field_world_ut = glm::vec3(20.0f, -40.0f, 0.0f);
    s.sigma_white_ut = 0.3f;
    s.sigma_bias_ut = 1.0f;
    s.bias_tau_s = 60.0f;
    std::optional<Rig> rig = make_rig(s, glm::quat(1, 0, 0, 0), 21u);
    ASSERT_TRUE(rig.has_value());
    (void)run(*rig, 10);
    const spade::Result<spade::SnapshotBlob> blob = rig->sim.snapshot();
    ASSERT_OK(blob);
    const spade::sensors::SampleIndex mark = rig->since;
    const std::vector<MagSample> after = run(*rig, 10);
    ASSERT_OK(rig->sim.restore(*blob));
    std::vector<MagSample> replay;
    spade::sensors::SampleIndex since = mark;
    for (int t = 0; t < 10; ++t) {
        ASSERT_OK(rig->sim.step(1));
        for (const MagSample& g : poll(rig->sim, rig->row, since)) {
            replay.push_back(g);
            since = g.index;
        }
    }
    ASSERT_EQ(after.size(), replay.size());
    ASSERT_FALSE(after.empty());
    EXPECT_EQ(std::memcmp(after.data(), replay.data(), after.size() * sizeof(MagSample)), 0);
}

TEST(MagModule, IsNotInTheStandardSetAndIsCpuOnly) {
    for (const spade::modules::ModuleDesc& m : spade::modules::standard_modules()) EXPECT_NE(m.name, "mag");
    const spade::modules::ModuleDesc m = spade::modules::mag_module();
    EXPECT_EQ(m.name, "mag");
    EXPECT_EQ(m.grade.cpu, spade::Grade::reference);
    EXPECT_EQ(m.grade.vulkan, spade::Grade::absent) << "a Vulkan world that holds the module is refused (L6)";
}
