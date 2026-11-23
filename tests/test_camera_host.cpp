// ---------------------------------------------------------------------------
// test_camera_host.cpp -- DRAFT of 3.3's RED: the CameraHost (sim/camera_host.hpp): the camera spec mapped onto a row, whole-tick rates, due frames with their
// capture time at the exposure midpoint, a step that loses no frame to the 64-deep ring, a per-env reset in place, labels and keypoints held host-side, the
// rotors' omega and psi, and psi across a save and restore.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "fc_host_test_rig.hpp"
#include "physics/blade_azimuth.hpp"
#include "sensors/camera.hpp"
#include "sim/camera_host.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "testing/replay.hpp"

namespace {

using spade::CameraHost;
using spade::CameraReset;
using spade::CameraSpec;
using spade::sensors::CameraSensorRow;

[[nodiscard]] spade::modules::ModuleSet standard_plus_camera() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::camera_module());
    return set;
}

struct CamRig {
    spade::Simulation sim;
    spade::BodyRef body;
};

// dt 2 ms, 2 substeps; a still world with room for two cameras; one body.
[[nodiscard]] std::optional<CamRig> make_cam_rig() {
    const spade::Result<spade::WorldSetDesc> set = fc_host_test::still_world();
    if (!set) return std::nullopt;
    spade::WorldSetDesc desc = *set;
    // the fc rig's world declares no camera capacity: rebuild its capacities with cameras
    // (the draft leaves this to the real test: still_world gains a cameras parameter)
    spade::Result<spade::Simulation> sim = spade::Simulation::create(desc, 2'000'000, 2, {}, standard_plus_camera());
    if (!sim) return std::nullopt;
    const spade::Result<spade::BodyRef> body = sim->spawn(0, spade::BodySpawn{});
    if (!body) return std::nullopt;
    return CamRig{std::move(*sim), *body};
}

[[nodiscard]] CameraSpec spec(uint32_t divider = 4, double exposure_s = 0.001) {
    CameraSpec s;
    s.width = 64;
    s.height = 48;
    s.fx = 50.0f;
    s.fy = 50.0f;
    s.cx = 32.0f;
    s.cy = 24.0f;
    s.rate_divider = divider;
    s.exposure_s = exposure_s;
    return s;
}

}  // namespace

TEST(CameraHost, ASpecIsMappedOntoTheRowAndAWholeTickRateIsRequired) {
    std::optional<CamRig> rig = make_cam_rig();
    ASSERT_TRUE(rig.has_value());
    spade::Result<CameraHost> host = CameraHost::open(rig->sim);
    ASSERT_TRUE(host.has_value());
    const spade::Result<uint32_t> cam = host->add_camera(rig->body, spec(4, 0.001));
    ASSERT_TRUE(cam.has_value()) << cam.error().context;
    const auto rows = rig->sim.module_rows<CameraSensorRow>("camera_sensors", 0);
    ASSERT_TRUE(rows.has_value());
    const CameraSensorRow& row = (*rows)[0];
    EXPECT_EQ(row.rate_divider, 4u);
    EXPECT_EQ(row.width, 64u);
    // 1 ms of a 4-substep period of 2 ms substeps (dt 2 ms / 2 = h 1 ms): the period is 4 ms, so the shutter fraction is 0.25
    EXPECT_NEAR(row.shutter_fraction, 0.25f, 1e-6f);

    const spade::Result<uint32_t> odd = host->add_camera(rig->body, spec(3));  // 3 substeps is a tick and a half
    ASSERT_FALSE(odd.has_value());
    EXPECT_NE(odd.error().context.find("rate_divider"), std::string::npos) << odd.error().context;
    EXPECT_NE(odd.error().context.find("whole number of ticks"), std::string::npos) << odd.error().context;
}

TEST(CameraHost, DueFramesCarryTheirTickTheSequenceAndTheExposureMidpoint) {
    std::optional<CamRig> rig = make_cam_rig();
    ASSERT_TRUE(rig.has_value());
    spade::Result<CameraHost> host = CameraHost::open(rig->sim);
    ASSERT_TRUE(host.has_value());
    ASSERT_TRUE(host->add_camera(rig->body, spec(4, 0.001)).has_value());
    const spade::Result<spade::StepReport> report = host->step(10);
    ASSERT_TRUE(report.has_value()) << report.error().context;
    EXPECT_EQ(report->ticks, 10u);
    ASSERT_EQ(report->due.size(), 5u);
    for (std::size_t i = 0; i < report->due.size(); ++i) {
        const spade::DueFrame& f = report->due[i];
        EXPECT_EQ(f.camera, 0u);
        EXPECT_EQ(f.frame_seq, i + 1);
        EXPECT_EQ(f.tick, 2 * i + 1);
        // the tick ends at (tick + 1) * 2 ms; the exposure is 1 ms and the labelled pose is at its midpoint: 0.5 ms before the end
        EXPECT_EQ(f.capture_time_ns, static_cast<int64_t>((f.tick + 1) * 2'000'000 - 500'000));
    }
}

TEST(CameraHost, AStepOfManyTicksLosesNoFrameToTheSixtyFourDeepRing) {
    std::optional<CamRig> rig = make_cam_rig();
    ASSERT_TRUE(rig.has_value());
    spade::Result<CameraHost> host = CameraHost::open(rig->sim);
    ASSERT_TRUE(host.has_value());
    ASSERT_TRUE(host->add_camera(rig->body, spec(2, 0.0)).has_value());  // a frame every tick
    const spade::Result<spade::StepReport> report = host->step(200);
    ASSERT_TRUE(report.has_value()) << report.error().context;
    ASSERT_EQ(report->due.size(), 200u);
    for (std::size_t i = 0; i < 200; ++i) EXPECT_EQ(report->due[i].frame_seq, i + 1) << "no gap, so no drop";
}

TEST(CameraHost, AResetChangesTheMountAndTheLensInPlaceAndKeepsTheCameraItsRingAndItsSequence) {
    std::optional<CamRig> rig = make_cam_rig();
    ASSERT_TRUE(rig.has_value());
    spade::Result<CameraHost> host = CameraHost::open(rig->sim);
    ASSERT_TRUE(host.has_value());
    ASSERT_TRUE(host->add_camera(rig->body, spec(4)).has_value());
    ASSERT_TRUE(host->step(6).has_value());
    const auto before = *rig->sim.module_rows<CameraSensorRow>("camera_sensors", 0);
    const CameraSensorRow was = before[0];

    CameraReset reset;
    reset.mount_orient = glm::normalize(glm::quat(0.9f, 0.0f, 0.0f, 0.2f));
    ASSERT_TRUE(host->reset_camera(0, reset).has_value());
    const CameraSensorRow now = (*rig->sim.module_rows<CameraSensorRow>("camera_sensors", 0))[0];
    EXPECT_EQ(now.mount_orient, reset.mount_orient);
    EXPECT_EQ(now.last_index, was.last_index) << "the frame sequence continues";
    EXPECT_EQ(now.noise.state, was.noise.state) << "the camera's stream is kept";
    EXPECT_EQ(now.fx, was.fx) << "the lens is untouched unless asked";

    reset.set_lens = true;
    reset.fx = 60.0f;
    reset.fy = 60.0f;
    reset.cx = 32.0f;
    reset.cy = 24.0f;
    ASSERT_TRUE(host->reset_camera(0, reset).has_value());
    EXPECT_EQ((*rig->sim.module_rows<CameraSensorRow>("camera_sensors", 0))[0].fx, 60.0f);

    // a refused reset changes nothing
    CameraReset bad = reset;
    bad.fx = 0.0f;
    const spade::Result<void> r = host->reset_camera(0, bad);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("fx"), std::string::npos) << r.error().context;
    EXPECT_EQ((*rig->sim.module_rows<CameraSensorRow>("camera_sensors", 0))[0].fx, 60.0f);
    CameraReset not_unit;
    not_unit.mount_orient = glm::quat(2.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(host->reset_camera(0, not_unit).has_value());
}

TEST(CameraHost, ALabelAndLocalKeypointsAreHeldHostSideAndMoveNoDigest) {
    std::optional<CamRig> rig = make_cam_rig();
    ASSERT_TRUE(rig.has_value());
    spade::Result<CameraHost> host = CameraHost::open(rig->sim);
    ASSERT_TRUE(host.has_value());
    const uint64_t digest = spade::testing::state_digest(rig->sim);
    ASSERT_TRUE(host->set_label(rig->body, 7).has_value());
    const std::vector<glm::vec3> corners = {{-1, 1, 0}, {1, 1, 0}, {1, -1, 0}, {-1, -1, 0}};
    ASSERT_TRUE(host->set_keypoints(rig->body, corners).has_value());
    EXPECT_EQ(*host->label_of(rig->body), 7u);
    const auto got = host->keypoints_of(rig->body);
    ASSERT_TRUE(got.has_value());
    ASSERT_EQ(got->size(), 4u);
    EXPECT_EQ((*got)[2], corners[2]);
    EXPECT_EQ(spade::testing::state_digest(rig->sim), digest) << "nothing here is in the registered state";

    std::vector<glm::vec3> bad = corners;
    bad[1].y = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(host->set_keypoints(rig->body, bad).has_value());
    EXPECT_FALSE(host->label_of(spade::BodyRef{}).has_value()) << "a null body has no label";
    EXPECT_EQ(host->keypoints_of(rig->body)->size(), 4u) << "a refused set leaves the old points";
}

TEST(CameraHost, TheRotorsReportOmegaAndTheBladeAzimuthByThePinnedRule) {
    std::optional<fc_host_test::Rig> rig = fc_host_test::make_rig(8);
    ASSERT_TRUE(rig.has_value());
    spade::Result<CameraHost> host = CameraHost::open(rig->sim);
    ASSERT_TRUE(host.has_value());
    const float cmd[4] = {500.0f, 520.0f, 500.0f, 480.0f};
    ASSERT_TRUE(rig->sim.set_rotor_commands(rig->vehicle, cmd).has_value());
    ASSERT_TRUE(host->step(8).has_value());
    const spade::Result<spade::RotorReading> reading = host->rotors(rig->vehicle);
    ASSERT_TRUE(reading.has_value()) << reading.error().context;
    ASSERT_EQ(reading->omega.size(), 4u);
    const float h = static_cast<float>(static_cast<double>(fc_host_test::kDt) * 1e-9);
    for (std::size_t r = 0; r < 4; ++r) {
        EXPECT_EQ(reading->omega[r], cmd[r]) << "tau 0: the shaft speed is the command";
        float psi = 0.0f;
        for (int t = 0; t < 8; ++t) ASSERT_TRUE(spade::physics::advance_blade_azimuth(psi, cmd[r], h));
        EXPECT_EQ(reading->psi[r], psi) << "rotor " << r;
    }
}

TEST(CameraHost, PsiAfterASaveAndARestoreMidRunEqualsTheUninterruptedRun) {
    const float cmd[4] = {500.0f, 520.0f, 500.0f, 480.0f};
    std::vector<float> straight;
    {
        std::optional<fc_host_test::Rig> rig = fc_host_test::make_rig(8);
        ASSERT_TRUE(rig.has_value());
        spade::Result<CameraHost> host = CameraHost::open(rig->sim);
        ASSERT_TRUE(host.has_value());
        ASSERT_TRUE(rig->sim.set_rotor_commands(rig->vehicle, cmd).has_value());
        ASSERT_TRUE(host->step(40).has_value());
        straight = host->rotors(rig->vehicle)->psi;
    }
    std::optional<fc_host_test::Rig> rig = fc_host_test::make_rig(8);
    ASSERT_TRUE(rig.has_value());
    spade::Result<CameraHost> host = CameraHost::open(rig->sim);
    ASSERT_TRUE(host.has_value());
    ASSERT_TRUE(rig->sim.set_rotor_commands(rig->vehicle, cmd).has_value());
    ASSERT_TRUE(host->step(17).has_value());
    const auto blob = rig->sim.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    const std::vector<std::byte> host_state = host->save();
    ASSERT_TRUE(host->step(9).has_value());  // wander off, so the restore has something to undo
    ASSERT_TRUE(rig->sim.restore(*blob).has_value());
    ASSERT_TRUE(host->load(host_state).has_value());
    ASSERT_TRUE(host->step(23).has_value());
    EXPECT_EQ(host->rotors(rig->vehicle)->psi, straight) << "psi follows the restore because the host's state is saved beside the simulation's";
}

TEST(CameraHost, ASimulationRestoredWithoutTheHostsStateIsRefusedByNameAtTheNextStep) {
    // The contract: omega comes back with the Simulation, psi with the CameraHost. Restoring only the former is DETECTED (the tick jumped backwards behind
    // the host's back), refused by name, and the remedy (load the host state saved beside the snapshot) works.
    const float cmd[4] = {500.0f, 520.0f, 500.0f, 480.0f};
    std::optional<fc_host_test::Rig> rig = fc_host_test::make_rig(8);
    ASSERT_TRUE(rig.has_value());
    spade::Result<CameraHost> host = CameraHost::open(rig->sim);
    ASSERT_TRUE(host.has_value());
    ASSERT_TRUE(rig->sim.set_rotor_commands(rig->vehicle, cmd).has_value());
    ASSERT_TRUE(host->step(10).has_value());
    const auto blob = rig->sim.snapshot();
    ASSERT_TRUE(blob.has_value());
    const std::vector<std::byte> host_state = host->save();
    ASSERT_TRUE(host->step(10).has_value());

    ASSERT_TRUE(rig->sim.restore(*blob).has_value());
    const spade::Result<StepReport> refused = host->step(1);
    ASSERT_FALSE(refused.has_value()) << "the Simulation went back to tick 10 behind the host's back";
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("the simulation was restored without the camera host's state"), std::string::npos) << refused.error().context;
    EXPECT_EQ(rig->sim.tick().value, 10u) << "a refused step ran nothing";

    ASSERT_TRUE(host->load(host_state).has_value());
    EXPECT_TRUE(host->step(1).has_value()) << "with its own state loaded the host steps on";

    // a Simulation that moved FORWARD behind the host's back is the same mistake
    ASSERT_TRUE(rig->sim.step(3).has_value());
    const spade::Result<StepReport> ahead = host->step(1);
    ASSERT_FALSE(ahead.has_value());
    EXPECT_NE(ahead.error().context.find("restored without the camera host's state"), std::string::npos) << ahead.error().context;
}

TEST(CameraHost, ASampleTimeIsTheEndOfItsTick) {
    std::optional<CamRig> rig = make_cam_rig();
    ASSERT_TRUE(rig.has_value());
    spade::Result<CameraHost> host = CameraHost::open(rig->sim);
    ASSERT_TRUE(host.has_value());
    EXPECT_EQ(host->sample_time_ns(0), 2'000'000);
    EXPECT_EQ(host->sample_time_ns(7), 16'000'000);
}

TEST(CameraHost, TheBladeCountAndPolePairsAreGivenOnceAndReturnedBesideOmegaAndPsi) {
    std::optional<fc_host_test::Rig> rig = fc_host_test::make_rig(8);
    ASSERT_TRUE(rig.has_value());
    spade::Result<CameraHost> host = CameraHost::open(rig->sim);
    ASSERT_TRUE(host.has_value());
    const auto before = host->rotors(rig->vehicle);
    ASSERT_TRUE(before.has_value());
    const std::size_t n = before->omega.size();
    ASSERT_GT(n, 0u);
    EXPECT_EQ(before->blade_count, std::vector<uint32_t>(n, 0u)) << "not given reads 0";

    std::vector<spade::RotorInfo> info(n, spade::RotorInfo{.blade_count = 3, .pole_pairs = 7});
    ASSERT_TRUE(host->set_rotor_info(rig->vehicle, info).has_value());
    const auto after = host->rotors(rig->vehicle);
    ASSERT_TRUE(after.has_value());
    EXPECT_EQ(after->blade_count, std::vector<uint32_t>(n, 3u));
    EXPECT_EQ(after->pole_pairs, std::vector<uint32_t>(n, 7u));

    // refused naming the field: a count out of range, a wrong length
    std::vector<spade::RotorInfo> bad = info;
    bad[0].blade_count = 17;
    const auto too_many_blades = host->set_rotor_info(rig->vehicle, bad);
    ASSERT_FALSE(too_many_blades.has_value());
    EXPECT_NE(too_many_blades.error().context.find("blade_count"), std::string::npos) << too_many_blades.error().context;
    bad = info;
    bad[0].pole_pairs = 65;
    const auto too_many_poles = host->set_rotor_info(rig->vehicle, bad);
    ASSERT_FALSE(too_many_poles.has_value());
    EXPECT_NE(too_many_poles.error().context.find("pole_pairs"), std::string::npos) << too_many_poles.error().context;
    const auto wrong_length = host->set_rotor_info(rig->vehicle, std::span<const spade::RotorInfo>(info).first(n - 1));
    ASSERT_FALSE(wrong_length.has_value());

    // the info rides in the host's saved state, so a restored host answers the same
    const std::vector<std::byte> saved = host->save();
    spade::Result<CameraHost> other = CameraHost::open(rig->sim);
    ASSERT_TRUE(other.has_value());
    ASSERT_TRUE(other->load(saved).has_value());
    EXPECT_EQ(other->rotors(rig->vehicle)->pole_pairs, std::vector<uint32_t>(n, 7u));
}
