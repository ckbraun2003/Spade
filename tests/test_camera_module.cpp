// ---------------------------------------------------------------------------
// test_camera_module.cpp -- the camera as a sensor kind in a world (Phase 3.2 of rendering/camera-sensor-design: sensors/camera.hpp's row, the opt-in module
// `camera`, its due-frame ring). The pixels are Rendering's; what Core holds is the row, the schedule and the stamp: when a frame falls due, and from which tick.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "sensors/camera.hpp"
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
using spade::sensors::CameraFrameDue;
using spade::sensors::CameraSensorRow;
using spade::sensors::CameraSensorSpawn;

#define ASSERT_OK(expr)                                                                                  \
    do {                                                                                                 \
        const auto& r_ = (expr);                                                                         \
        ASSERT_TRUE(r_.has_value()) << #expr << " failed: " << (r_ ? "" : r_.error().context);          \
    } while (false)

[[nodiscard]] spade::modules::ModuleSet standard_plus_camera() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::camera_module());
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
    caps.cameras = 2;
    caps.camera_pixels = 2 * 64 * 48;
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

// A 64 x 48 pinhole at the given divider; the rest is the row's own defaults.
[[nodiscard]] CameraSensorSpawn good_spawn(uint32_t rate_divider = 4) {
    CameraSensorSpawn s;
    s.rate_divider = rate_divider;
    s.width = 64;
    s.height = 48;
    s.fx = 50.0f;
    s.fy = 50.0f;
    s.cx = 32.0f;
    s.cy = 24.0f;
    return s;
}

struct Rig {
    Simulation sim;
    BodyRef body;
    std::vector<RowRef> rows;
    uint64_t seed = 0;
};

// dt 2 ms, 2 substeps (h = 1 ms); a body at the origin; the cameras in `spawns`.
[[nodiscard]] std::optional<Rig> make_rig(const std::vector<CameraSensorSpawn>& spawns, uint64_t seed = 0x5EEDu, std::string* why = nullptr) {
    const spade::Result<WorldSetDesc> set = still_world(seed);
    if (!set) return std::nullopt;
    spade::Result<Simulation> sim = Simulation::create(*set, 2'000'000, 2, {}, standard_plus_camera());
    if (!sim) {
        if (why) *why = sim.error().context;
        return std::nullopt;
    }
    const spade::Result<BodyRef> body = sim->spawn(0, BodySpawn{});
    if (!body) return std::nullopt;
    std::vector<RowRef> rows;
    for (const CameraSensorSpawn& spawn : spawns) {
        const spade::Result<RowRef> row = sim->attach_row(*body, "camera_sensors", spawn);
        if (!row) {
            if (why) *why = row.error().context;
            return std::nullopt;
        }
        rows.push_back(*row);
    }
    if (!sim->flush_structural()) return std::nullopt;
    return Rig{std::move(*sim), *body, rows, seed};
}

// Every frame newer than `since`, in index order.
[[nodiscard]] std::vector<CameraFrameDue> poll(const Simulation& sim, const RowRef& row, spade::sensors::SampleIndex since) {
    const auto rows = sim.module_rows<CameraSensorRow>("camera_sensors", row.world_index);
    const auto ring = sim.module_rows<CameraFrameDue>("camera_frames", row.world_index);
    if (!rows || !ring) return {};
    std::vector<CameraFrameDue> out(spade::sensors::kRingDepth);
    const auto window = std::span<const CameraFrameDue>(*ring).subspan(std::size_t{row.slot} * spade::sensors::kRingDepth, spade::sensors::kRingDepth);
    const auto r = spade::sensors::ring_poll<CameraFrameDue>(window, (*rows)[row.slot].last_index, since, out);
    out.resize(r.samples.size());
    return out;
}

// Step `steps` ticks, one at a time, collecting camera `which`'s frames (the ring is only 64 deep).
[[nodiscard]] std::vector<CameraFrameDue> run(Rig& rig, std::size_t which, uint64_t steps) {
    std::vector<CameraFrameDue> all;
    spade::sensors::SampleIndex since = 0;
    for (uint64_t t = 0; t < steps; ++t) {
        EXPECT_TRUE(rig.sim.step(1).has_value());
        for (const CameraFrameDue& f : poll(rig.sim, rig.rows[which], since)) {
            all.push_back(f);
            since = f.index;
        }
    }
    return all;
}

}  // namespace

TEST(CameraModule, TheDueFrameIsAFixedSixteenByteRecord) {
    EXPECT_EQ(sizeof(CameraFrameDue), 16u);
    EXPECT_EQ(offsetof(CameraFrameDue, index), 0u);
    EXPECT_EQ(offsetof(CameraFrameDue, tick), 8u);
    EXPECT_EQ(sizeof(CameraSensorSpawn), 108u);
}

TEST(CameraModule, TheModuleIsOptInAndNotInTheStandardSet) {
    const spade::Result<WorldSetDesc> set = still_world(1);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 2'000'000, 2);  // the standard modules only
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, BodySpawn{});
    ASSERT_OK(body);
    const spade::Result<RowRef> row = sim->attach_row(*body, "camera_sensors", good_spawn());
    ASSERT_FALSE(row.has_value());
    EXPECT_NE(row.error().context.find("camera_sensors"), std::string::npos) << row.error().context;
}

TEST(CameraModule, AttachWritesTheRowFromTheSpawnAndMakesItLive) {
    CameraSensorSpawn spawn = good_spawn(6);
    spawn.mount_pos = glm::vec3(0.1f, 0.02f, -0.3f);
    spawn.shutter_fraction = 0.5f;
    spawn.mode = spade::sensors::camera_mode::grey;
    spawn.lens_kind = spade::sensors::lens_kind::kb4;
    spawn.k1 = 0.01f;
    spawn.k2 = -0.002f;
    spawn.blur_samples = 4;
    spawn.aa_samples = 2;
    spawn.exposure_gain = 1.5f;
    spawn.cull_mask = 0x7Eu;
    spawn.channels = spade::sensors::camera_channel::image | spade::sensors::camera_channel::instance_id;
    std::optional<Rig> rig = make_rig({spawn});
    ASSERT_TRUE(rig.has_value());
    const auto rows = rig->sim.module_rows<CameraSensorRow>("camera_sensors", 0);
    ASSERT_OK(rows);
    const CameraSensorRow& row = (*rows)[rig->rows[0].slot];
    EXPECT_EQ(row.kind, spade::sensors::sensor_kind::camera);
    EXPECT_EQ(row.rate_divider, 6u);
    EXPECT_EQ(row.phase, 0u);
    EXPECT_EQ(row.mount_pos, spawn.mount_pos);
    EXPECT_EQ(row.shutter_fraction, 0.5f);
    EXPECT_EQ(row.lens_kind, spade::sensors::lens_kind::kb4);
    EXPECT_EQ(row.width, 64u);
    EXPECT_EQ(row.height, 48u);
    EXPECT_EQ(row.mode, spade::sensors::camera_mode::grey);
    EXPECT_EQ(row.fx, 50.0f);
    EXPECT_EQ(row.cy, 24.0f);
    EXPECT_EQ(row.k1, 0.01f);
    EXPECT_EQ(row.k2, -0.002f);
    EXPECT_EQ(row.blur_samples, 4u);
    EXPECT_EQ(row.aa_samples, 2u);
    EXPECT_EQ(row.exposure_gain, 1.5f);
    EXPECT_EQ(row.cull_mask, 0x7Eu);
    EXPECT_EQ(row.channels, spawn.channels);
    EXPECT_EQ(row.last_index, 0u);
    EXPECT_EQ(spade::sensors::camera_row_issue(row, 1), "") << "a row the module wrote passes the row's own validation";
}

TEST(CameraModule, AFrameFallsDueEveryDividerSubstepsStampedWithItsTick) {
    std::optional<Rig> rig = make_rig({good_spawn(4)});  // 4 substeps = 2 ticks (2 substeps a tick)
    ASSERT_TRUE(rig.has_value());
    const std::vector<CameraFrameDue> frames = run(*rig, 0, 8);
    ASSERT_EQ(frames.size(), 4u);
    for (std::size_t i = 0; i < frames.size(); ++i) {
        EXPECT_EQ(frames[i].index, i + 1) << "indices count from 1";
        EXPECT_EQ(frames[i].tick, 2 * i + 1) << "the tick whose last substep completes the period";
    }
}

TEST(CameraModule, TwoCamerasKeepTheirOwnClocksAndIndices) {
    std::optional<Rig> rig = make_rig({good_spawn(2), good_spawn(6)});
    ASSERT_TRUE(rig.has_value());
    EXPECT_NE(rig->rows[0].slot, rig->rows[1].slot);
    // run both from one set of steps: poll each as the world advances
    std::vector<CameraFrameDue> fast;
    std::vector<CameraFrameDue> slow;
    spade::sensors::SampleIndex since_fast = 0;
    spade::sensors::SampleIndex since_slow = 0;
    for (int t = 0; t < 12; ++t) {
        ASSERT_OK(rig->sim.step(1));
        for (const CameraFrameDue& f : poll(rig->sim, rig->rows[0], since_fast)) {
            fast.push_back(f);
            since_fast = f.index;
        }
        for (const CameraFrameDue& f : poll(rig->sim, rig->rows[1], since_slow)) {
            slow.push_back(f);
            since_slow = f.index;
        }
    }
    EXPECT_EQ(fast.size(), 12u) << "a divider of 2 substeps is every tick";
    EXPECT_EQ(slow.size(), 4u) << "6 substeps is every 3 ticks";
    for (std::size_t i = 0; i < slow.size(); ++i) EXPECT_EQ(slow[i].tick, 3 * i + 2);
}

TEST(CameraModule, TheNoiseStreamIsTheWorldLocalSlotStreamAndAReseedRedrawsIt) {
    std::optional<Rig> rig = make_rig({good_spawn(), good_spawn()});
    ASSERT_TRUE(rig.has_value());
    const uint64_t world_seed = (*rig->sim.world_params(0))->seed;
    {
        const auto rows = rig->sim.module_rows<CameraSensorRow>("camera_sensors", 0);
        ASSERT_OK(rows);
        for (std::size_t i = 0; i < 2; ++i) {
            const spade::rng::Stream want = spade::sensors::camera_noise_stream(world_seed, rig->rows[i].slot);
            const CameraSensorRow& row = (*rows)[rig->rows[i].slot];
            EXPECT_EQ(row.noise.state, want.state) << i;
        }
        EXPECT_NE((*rows)[rig->rows[0].slot].noise.state, (*rows)[rig->rows[1].slot].noise.state) << "each camera has its own stream";
    }
    ASSERT_OK(rig->sim.step(4));
    ASSERT_OK(rig->sim.reseed(99u));
    const uint64_t reseeded = (*rig->sim.world_params(0))->seed;
    EXPECT_NE(reseeded, world_seed);
    const auto rows = rig->sim.module_rows<CameraSensorRow>("camera_sensors", 0);
    ASSERT_OK(rows);
    EXPECT_EQ((*rows)[rig->rows[0].slot].noise.state, spade::sensors::camera_noise_stream(reseeded, rig->rows[0].slot).state);
}

TEST(CameraModule, ASpawnTheRowRulesRefuseIsRefusedAtAttachNamingTheField) {
    const auto refused = [](auto&& edit, const char* field) {
        CameraSensorSpawn s = good_spawn();
        edit(s);
        std::string why;
        const std::optional<Rig> rig = make_rig({s}, 0x5EEDu, &why);
        EXPECT_FALSE(rig.has_value()) << field;
        EXPECT_NE(why.find(field), std::string::npos) << field << " not in: " << why;
    };
    refused([](CameraSensorSpawn& s) { s.rate_divider = 0; }, "rate_divider");
    refused([](CameraSensorSpawn& s) { s.width = 0; }, "width");
    refused([](CameraSensorSpawn& s) { s.height = 0; }, "height");
    refused([](CameraSensorSpawn& s) { s.lens_kind = 7; }, "lens_kind");
    refused([](CameraSensorSpawn& s) { s.mode = 5; }, "mode");
    refused([](CameraSensorSpawn& s) { s.fx = 0.0f; }, "fx");
    refused([](CameraSensorSpawn& s) { s.k1 = 0.1f; }, "k1");  // a pinhole has no distortion
    refused([](CameraSensorSpawn& s) { s.channels = 0; }, "channels");
    refused([](CameraSensorSpawn& s) { s.channels = 0x8u; }, "channels");
    refused([](CameraSensorSpawn& s) { s.blur_samples = 0; }, "blur_samples");
    refused([](CameraSensorSpawn& s) { s.exposure_gain = -1.0f; }, "exposure_gain");
    refused([](CameraSensorSpawn& s) { s.cull_mask = 0x1FFu; }, "cull_mask");
    refused([](CameraSensorSpawn& s) { s.mount_orient = glm::quat(2.0f, 0.0f, 0.0f, 0.0f); }, "mount_orient");
    refused([](CameraSensorSpawn& s) { s.mount_pos.x = std::numeric_limits<float>::quiet_NaN(); }, "mount_pos");
}
