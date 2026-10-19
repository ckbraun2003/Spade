// The drone sim box's view (sandbox/drone_view.hpp): the drone's parts, the
// panel and the keys. Display-free (SL15b). The analytic air field and its
// heatmap slice are retired (INT-9), and their tests with them; the
// field-layer drawing they exercised is Rendering's, in test_render_field.cpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"

#include "../sandbox/drone_view.hpp"

using namespace spade::sandbox;

namespace {

// An empty world's scene with the drone bound into it -- what the sandbox's
// drone scene starts from. `world` is an out-parameter because the scene's
// sdf pointer borrows it. Void, so a failed setup ASSERTs (stops the test)
// instead of dereferencing an empty Result; callers ASSERT_NO_FATAL_FAILURE.
void drone_scene(spade::WorldDesc& world, DroneDrawBinding& binding, spade::render::RenderScene& out) {
    auto built = spade::WorldBuilder().name("drone_view").environment(spade::Environment{})
                     .capacities(spade::Capacities{1, 5, 1, 1}).build();
    ASSERT_TRUE(built.has_value());
    world = *built;
    auto scene = spade::render::scene_from_world(world, {});
    ASSERT_TRUE(scene.has_value());
    out = std::move(*scene);
    binding = bind_drone_scene(out, drone_stand_params());
}

}  // namespace

TEST(SandboxDroneView, TheDronesPartsAreWhereTheAirframeSaysAndUnscaled) {
    const auto params = drone_stand_params();
    DroneDrawBinding b;
    b.body_mesh = 1;
    b.arm_mesh = 2;
    b.rotor_mesh = 3;
    b.nose_material = 7;
    b.arm_material = 8;
    b.rotor_material = 9;
    std::vector<spade::render::DrawItem> items;
    append_drone_items(b, params, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), items);
    ASSERT_EQ(items.size(), 9u);  // body, then (arm, disc) per rotor

    for (std::size_t i = 0; i < 4; ++i) {
        const auto& arm = items[1 + 2 * i];
        const auto& disc = items[2 + 2 * i];
        EXPECT_EQ(arm.material_override, i == 0 ? 7u : 8u) << "rotor " << i;
        // The front disc carries the heading too: from above the discs hide the arms.
        EXPECT_EQ(disc.material_override, i == 0 ? 7u : 9u) << "rotor " << i;
        // The arm mesh lies along +X from -L/2 to +L/2; its +X end must reach the hub.
        const glm::vec3 hub = spade::vehicles::quadrotor_arm_offset(params, i);
        const glm::vec3 tip = glm::vec3(arm.local_to_world * glm::vec4(0.5f * params.arm_length, 0.0f, 0.0f, 1.0f));
        EXPECT_NEAR(glm::length(tip - glm::vec3(hub.x, 0.0f, hub.z)), 0.0f, 1e-5f) << "rotor " << i;
        const glm::vec3 centre = glm::vec3(disc.local_to_world[3]);
        EXPECT_NEAR(centre.x, hub.x, 1e-6f);
        EXPECT_NEAR(centre.z, hub.z, 1e-6f);
        // Lit parts carry no scale: every basis column is unit length.
        for (const auto* item : {&arm, &disc}) {
            for (int c = 0; c < 3; ++c)
                EXPECT_NEAR(glm::length(glm::vec3(item->local_to_world[c])), 1.0f, 1e-6f) << "rotor " << i;
        }
    }
}

// --------------------------------------------------------------------------
// The panel and the keys (the logic the window loop only calls)
// --------------------------------------------------------------------------

TEST(SandboxDronePanel, ADraggedSliderDoesNotRebuildUntilReleased) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    DronePanelModel panel;
    panel.edited = drone->options();
    ASSERT_TRUE(drone->step_fixed(20).has_value());

    panel.ui_item_active = true;
    for (int frame = 0; frame < 30; ++frame) {  // dragging wind from 0 to 15 m/s
        panel.edited.wind_speed_mps = 0.5f * static_cast<float>(frame);
        EXPECT_FALSE(apply_panel_edits(*drone, panel)) << "rebuilt mid-drag at frame " << frame;
    }
    EXPECT_EQ(drone->readouts().tick, 20u) << "the running simulation was replaced during the drag";

    panel.ui_item_active = false;  // released
    EXPECT_TRUE(apply_panel_edits(*drone, panel));
    EXPECT_EQ(drone->options().wind_speed_mps, 14.5f);
    EXPECT_FALSE(apply_panel_edits(*drone, panel)) << "rebuilt again with nothing changed";
    EXPECT_TRUE(panel.status.empty());
}

// The live smoke's drone_box.turbulence/applied and /calm-again, headless:
// the panel's turbulence reaches the running stand, and putting every option
// back to its default calms the air again.
TEST(SandboxDronePanel, TurbulenceAppliesAndTheDefaultsCalmTheAirAgain) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    ASSERT_TRUE(drone->step_fixed(20).has_value());
    DronePanelModel panel;
    panel.edited = drone->options();
    panel.edited.turbulence = spade::TurbulenceLevel::moderate;
    EXPECT_TRUE(apply_panel_edits(*drone, panel)) << "a turbulence edit did not rebuild the stand";
    EXPECT_EQ(drone->options().turbulence, spade::TurbulenceLevel::moderate);
    EXPECT_TRUE(panel.status.empty()) << panel.status;

    panel.edited = DronePhysicsOptions{};
    EXPECT_TRUE(apply_panel_edits(*drone, panel));
    EXPECT_TRUE(drone->options() == DronePhysicsOptions{}) << "turbulence "
                                                           << static_cast<int>(drone->options().turbulence)
                                                           << ", wind " << drone->options().wind_speed_mps << " m/s";
}

TEST(SandboxDronePanel, SelectingVulkanIsRefusedVisiblyAndTheStandStaysOnCpu) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    ASSERT_TRUE(drone->step_fixed(20).has_value());
    DronePanelModel panel;
    panel.edited = drone->options();
    panel.edited.vulkan = true;
    EXPECT_FALSE(apply_panel_edits(*drone, panel));
    EXPECT_FALSE(panel.status.empty()) << "the refusal must reach the panel";
    EXPECT_FALSE(panel.edited.vulkan) << "the backend widget must snap back to what is running";
    EXPECT_FALSE(drone->options().vulkan);
    EXPECT_EQ(drone->readouts().tick, 20u);
    // A retried refusal also returns false, so the return value cannot tell;
    // a retry would refill `status`, so clear it and look.
    panel.status.clear();
    EXPECT_FALSE(apply_panel_edits(*drone, panel));
    EXPECT_TRUE(panel.status.empty()) << "the refusal is retried every frame";
}

TEST(SandboxDronePanel, TheKeysNudgeHoldClampAndLevel) {
    AttitudeTarget t;
    FrameInput in;
    in.attitude_pitch = 1.0f;
    in.attitude_roll = -1.0f;
    in.attitude_yaw = 1.0f;
    for (int i = 0; i < 600; ++i) nudge_attitude(t, in, 1.0f / 60.0f);  // held for 10 s
    EXPECT_FLOAT_EQ(t.pitch, kMaxTiltRad);
    EXPECT_FLOAT_EQ(t.roll, -kMaxTiltRad);
    EXPECT_LE(std::fabs(t.yaw), 3.14159265f + 1e-6f) << "yaw did not wrap";

    const AttitudeTarget held = t;
    nudge_attitude(t, FrameInput{}, 1.0f);  // released: the target holds
    EXPECT_EQ(t.pitch, held.pitch);
    EXPECT_EQ(t.yaw, held.yaw);

    FrameInput level;
    level.level_pressed = true;
    nudge_attitude(t, level, 1.0f / 60.0f);
    EXPECT_EQ(t.pitch, 0.0f);
    EXPECT_EQ(t.roll, 0.0f);
    EXPECT_EQ(t.yaw, held.yaw) << "R levels the drone; it does not reset the heading";

    FrameInput captured = in;
    captured.ui_captured_keyboard = true;
    nudge_attitude(t, captured, 1.0f);
    EXPECT_EQ(t.pitch, 0.0f) << "keys typed into a panel field flew the drone";
}
