// The drone sim box's view (sandbox/drone_view.hpp): the air field, the
// heatmap palette and slice, and the drone's parts. Display-free (SL15b);
// the two render cases run the CPU reference rasterizer.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"

#include "../sandbox/drone_view.hpp"

using namespace spade::sandbox;

namespace {

constexpr uint32_t kW = 96, kH = 64;

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

spade::render::Camera camera_on_z(float distance) {
    spade::render::Camera c;
    c.position = glm::vec3(0.0f, 0.0f, distance);  // identity orientation looks down -Z, at the origin
    return c;
}

std::vector<uint8_t> render_cpu(const spade::render::RenderScene& scene, const spade::render::Camera& camera) {
    std::vector<uint8_t> px(static_cast<size_t>(kW) * kH * 4u, 0u);
    spade::render::RenderTarget target{
        .pixels = px,
        .width = kW,
        .height = kH,
        .stride = kW * 4u,
        .format = spade::render::PixelFormat::bgrx8,
    };
    spade::render::RenderOptions options;
    options.mode = spade::render::DrawMode::shaded;
    options.shadows = false;
    options.overlays = false;
    EXPECT_TRUE(spade::render::render(scene, camera, options, target).has_value());
    return px;
}

}  // namespace

TEST(SandboxDroneView, WithRotorsStoppedTheFieldIsTheMedium) {
    DronePhysicsOptions o;
    o.wind_speed_mps = 4.0f;
    o.throttle = 0.0f;
    auto drone = DroneSim::create(o);
    ASSERT_TRUE(drone.has_value());
    ASSERT_TRUE(drone->step_fixed(200).has_value());
    auto field = air_field_from(*drone);
    ASSERT_TRUE(field.has_value());
    EXPECT_NEAR(glm::length(field->medium.wind - glm::vec3(4.0f, 0.0f, 0.0f)), 0.0f, 1e-5f);
    for (const glm::vec3 p : {glm::vec3(0.0f), glm::vec3(0.0f, -0.6f, 0.0f), glm::vec3(0.3f, 0.2f, -0.4f)})
        EXPECT_NEAR(glm::length(field->velocity(p) - field->medium.wind), 0.0f, 1e-4f);
}

TEST(SandboxDroneView, AtHoverTheDownwashIsBelowTheRotors) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    ASSERT_TRUE(drone->step_fixed(200).has_value());
    auto field = air_field_from(*drone);
    ASSERT_TRUE(field.has_value());
    const glm::vec3 below_rotor0 = field->velocity(glm::vec3(0.18f, -0.4f, 0.0f));
    const glm::vec3 above_rotor0 = field->velocity(glm::vec3(0.18f, 0.4f, 0.0f));
    EXPECT_LT(below_rotor0.y, -1.0f);
    EXPECT_GT(glm::length(below_rotor0), glm::length(above_rotor0));
}

TEST(SandboxDroneView, SpeedBinsCoverTheRangeAndSurviveAZeroMaximum) {
    EXPECT_EQ(speed_bin(0.0f, 10.0f), 0u);
    EXPECT_EQ(speed_bin(10.0f, 10.0f), kHeatmapBins - 1);
    EXPECT_EQ(speed_bin(50.0f, 10.0f), kHeatmapBins - 1);
    EXPECT_EQ(speed_bin(3.0f, 0.0f), 0u);
    EXPECT_EQ(speed_bin(3.0f, std::nanf("")), 0u);
    EXPECT_EQ(speed_bin(4.0f, 10.0f), 12u);  // 4/10 * 32 = 12.8
}

TEST(SandboxDroneView, ViridisHitsItsEndStops) {
    EXPECT_EQ(viridis(0.0f), glm::vec3(68.0f, 1.0f, 84.0f) / 255.0f);
    EXPECT_EQ(viridis(1.0f), glm::vec3(253.0f, 231.0f, 37.0f) / 255.0f);
    EXPECT_EQ(viridis(-3.0f), viridis(0.0f));
    EXPECT_EQ(viridis(7.0f), viridis(1.0f));
}

// A uniform field (rotors stopped, 4 m/s of wind, a manual 10 m/s range) puts
// every cell in bin 12, so whichever cell the centre pixel lands in, its bytes
// must be exactly that bin's palette colour -- the unlit material path, end
// to end, with nothing approximate in between.
TEST(SandboxDroneView, AHeatmapPixelIsExactlyItsPaletteColour) {
    spade::WorldDesc world;
    DroneDrawBinding b;
    spade::render::RenderScene scene;
    ASSERT_NO_FATAL_FAILURE(drone_scene(world, b, scene));
    const spade::render::Camera camera = camera_on_z(3.0f);

    AirField field;
    field.medium.density = 1.225f;
    field.medium.wind = glm::vec3(4.0f, 0.0f, 0.0f);
    const float observed =
        append_slice_items(b, camera_facing_slice(camera, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)), field,
                           10.0f, scene.dynamics);
    EXPECT_FLOAT_EQ(observed, 4.0f);
    ASSERT_EQ(scene.dynamics.size(), static_cast<size_t>(kSliceCells) * kSliceCells);
    for (const auto& item : scene.dynamics) ASSERT_EQ(item.material_override, b.heatmap_base + 12u);

    const std::vector<uint8_t> px = render_cpu(scene, camera);
    const size_t at = (static_cast<size_t>(kH / 2) * kW + kW / 2) * 4u;
    const glm::vec3 want = heatmap_color(12u);
    EXPECT_EQ(px[at + 0], spade::render::to_byte(want.b));
    EXPECT_EQ(px[at + 1], spade::render::to_byte(want.g));
    EXPECT_EQ(px[at + 2], spade::render::to_byte(want.r));
}

// The test above proves the palette's bytes reach the pixel, but in a uniform
// field ANY cell would do. Here the field is lopsided -- one hovering rotor's
// plume at x = +0.5 -- and two mirror-image pixels must each show the bin of
// the cell their own ray hits: one in the plume, one in still air. A slice that
// was mirrored, shifted or mis-scaled shows the wrong colour in at least one.
// (Rendering's review.) The ray is raster_cpu's own: pixel centre at +0.5,
// NDC y up, direction (x_ndc * aspect / f, y_ndc / f, -1) for an identity
// camera orientation.
TEST(SandboxDroneView, EachPixelShowsTheCellItsRayHits) {
    spade::WorldDesc world;
    DroneDrawBinding b;
    spade::render::RenderScene scene;
    ASSERT_NO_FATAL_FAILURE(drone_scene(world, b, scene));
    const spade::render::Camera camera = camera_on_z(3.0f);

    AirField field;
    field.medium.density = 1.225f;
    field.rotors[0].hub_world = glm::vec3(0.5f, 0.0f, 0.0f);
    field.rotors[0].thrust_axis_world = glm::vec3(0.0f, 1.0f, 0.0f);
    field.rotors[0].radius = 0.12f;
    field.rotors[0].thrust_coeff = 1.2e-5f;
    field.rotors[0].omega = 452.0f;
    field.rotors[0].density = 1.225f;
    const float range = 12.0f;
    const SliceSpec s = camera_facing_slice(camera, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    ASSERT_EQ(s.right, glm::vec3(1.0f, 0.0f, 0.0f));
    append_slice_items(b, s, field, range, scene.dynamics);
    const std::vector<uint8_t> px = render_cpu(scene, camera);

    const double f = 1.0 / std::tan(0.5 * static_cast<double>(camera.fov_y_radians));
    const double aspect = static_cast<double>(kW) / static_cast<double>(kH);
    uint32_t bins[2] = {0u, 0u};
    const uint32_t pixel_x[2] = {57u, kW - 1u - 57u};  // aimed into the plume, and its mirror image
    for (int k = 0; k < 2; ++k) {
        const uint32_t x = pixel_x[k], y = 46u;
        const double x_ndc = 2.0 * (x + 0.5) / kW - 1.0;
        const double y_ndc = 1.0 - 2.0 * (y + 0.5) / kH;
        // The ray from (0, 0, 3) along (x_ndc*aspect/f, y_ndc/f, -1) meets z = 0 at t = 3.
        const double hit_x = 3.0 * x_ndc * aspect / f;
        const double hit_y = 3.0 * y_ndc / f;
        const double ci = (hit_x / s.width + 0.5) * kSliceCells;
        const double cj = ((hit_y - s.center.y) / s.height + 0.5) * kSliceCells;
        // Far enough inside its cell that rounding cannot move it to a neighbour.
        ASSERT_GT(ci - std::floor(ci), 0.1);
        ASSERT_LT(ci - std::floor(ci), 0.9);
        ASSERT_GT(cj - std::floor(cj), 0.1);
        ASSERT_LT(cj - std::floor(cj), 0.9);
        const auto i = static_cast<uint32_t>(ci), j = static_cast<uint32_t>(cj);
        bins[k] = speed_bin(glm::length(field.velocity(slice_cell_center(s, i, j))), range);
        const glm::vec3 want = heatmap_color(bins[k]);
        const size_t at = (static_cast<size_t>(y) * kW + x) * 4u;
        EXPECT_EQ(px[at + 0], spade::render::to_byte(want.b)) << "pixel " << x << " cell " << i << "," << j;
        EXPECT_EQ(px[at + 1], spade::render::to_byte(want.g)) << "pixel " << x << " cell " << i << "," << j;
        EXPECT_EQ(px[at + 2], spade::render::to_byte(want.r)) << "pixel " << x << " cell " << i << "," << j;
    }
    EXPECT_GT(bins[0], bins[1] + 8u) << "the two pixels must see different air, or a mirror would pass";
}

TEST(SandboxDroneView, TheHoverSliceSpansSeveralBins) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    ASSERT_TRUE(drone->step_fixed(200).has_value());
    auto field = air_field_from(*drone);
    ASSERT_TRUE(field.has_value());
    spade::WorldDesc world;
    DroneDrawBinding b;
    spade::render::RenderScene scene;
    ASSERT_NO_FATAL_FAILURE(drone_scene(world, b, scene));
    const float observed =
        append_slice_items(b, camera_facing_slice(camera_on_z(3.0f), glm::vec3(0.0f), drone->readouts().orientation),
                           *field, 0.0f, scene.dynamics);
    EXPECT_GT(observed, 1.0f);
    std::set<uint32_t> bins;
    for (const auto& item : scene.dynamics) bins.insert(item.material_override - b.heatmap_base);
    EXPECT_GT(bins.size(), 4u) << "a hovering drone's slice should not be one colour";
    EXPECT_TRUE(bins.count(0u)) << "still air above the drone should be the bottom bin";
    EXPECT_TRUE(bins.count(kHeatmapBins - 1)) << "auto range: the fastest cell is the top bin";
}

TEST(SandboxDroneView, StandardAndHeatmapViewsDiffer) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    ASSERT_TRUE(drone->step_fixed(200).has_value());
    auto field = air_field_from(*drone);
    ASSERT_TRUE(field.has_value());

    spade::WorldDesc world;
    DroneDrawBinding b;
    spade::render::RenderScene scene;
    ASSERT_NO_FATAL_FAILURE(drone_scene(world, b, scene));
    const spade::render::Camera camera = camera_on_z(1.5f);
    append_drone_items(b, drone->params(), glm::vec3(0.0f), drone->readouts().orientation, scene.dynamics);
    const std::vector<uint8_t> standard = render_cpu(scene, camera);
    append_slice_items(b, camera_facing_slice(camera, glm::vec3(0.0f), drone->readouts().orientation), *field, 0.0f,
                       scene.dynamics);
    const std::vector<uint8_t> heatmap = render_cpu(scene, camera);
    EXPECT_NE(standard, heatmap);
}

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

TEST(SandboxDroneView, TheSliceHoldsTheThrustAxisAndAnArmAndFacesTheCamera) {
    for (const glm::quat q : {glm::quat(1.0f, 0.0f, 0.0f, 0.0f), attitude_quat(AttitudeTarget{0.7f, 0.3f, -0.5f})}) {
        const glm::vec3 thrust = q * glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 arm_x = q * glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 arm_z = q * glm::vec3(0.0f, 0.0f, 1.0f);
        for (const glm::vec3 cam_pos : {glm::vec3(2.0f, 1.5f, -1.0f), glm::vec3(-0.5f, 0.2f, 3.0f),
                                        glm::vec3(1.0f, -2.0f, 1.0f)}) {
            spade::render::Camera camera;
            camera.position = cam_pos;
            const SliceSpec s = camera_facing_slice(camera, glm::vec3(0.0f), q);
            const glm::vec3 to_cam = glm::normalize(cam_pos);
            EXPECT_NEAR(glm::length(s.up - thrust), 0.0f, 1e-6f);
            const bool on_x = std::fabs(std::fabs(glm::dot(s.right, arm_x)) - 1.0f) < 1e-5f;
            const bool on_z = std::fabs(std::fabs(glm::dot(s.right, arm_z)) - 1.0f) < 1e-5f;
            EXPECT_TRUE(on_x || on_z) << "right is not an arm axis";
            const glm::vec3 normal = glm::cross(s.right, s.up);
            EXPECT_GE(glm::dot(normal, to_cam), 0.0f) << "the slice is seen from behind (mirrored)";
            // The more square of the two pair planes: its normal is the other arm.
            const glm::vec3 other = on_x ? arm_x : arm_z;  // the other plane's normal is +-this axis
            EXPECT_GE(std::fabs(glm::dot(normal, to_cam)) + 1e-6f, std::fabs(glm::dot(other, to_cam)));
            EXPECT_NEAR(glm::length(s.center - thrust * -0.5f), 0.0f, 1e-6f);
        }
    }
    spade::render::Camera at_drone;  // camera on the drone itself: degenerate, must stay finite
    const SliceSpec d = camera_facing_slice(at_drone, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    EXPECT_TRUE(std::isfinite(d.right.x) && std::isfinite(d.right.y) && std::isfinite(d.right.z));
}

namespace {
// The fastest air the slice shows more than 10 cm below the rotor plane.
float max_speed_below_rotors(const AirField& field, const SliceSpec& s, const glm::vec3& drone_pos) {
    float best = 0.0f;
    for (uint32_t j = 0; j < kSliceCells; ++j) {
        for (uint32_t i = 0; i < kSliceCells; ++i) {
            const glm::vec3 p = slice_cell_center(s, i, j);
            if (glm::dot(p - drone_pos, s.up) < -0.1f) best = std::max(best, glm::length(field.velocity(p)));
        }
    }
    return best;
}
}  // namespace

// The test the camera-on-Z cases hid: a camera-facing VERTICAL plane missed
// both plumes 28-62 degrees from an arm (Physics measured 0.03 m/s below the
// drone at 45). The body-aligned slice must catch a plume pair from every
// azimuth, level and rolled.
TEST(SandboxDroneView, TheSliceCatchesThePlumeFromEveryAzimuth) {
    for (const float roll : {0.0f, 0.7853982f}) {
        auto drone = DroneSim::create(DronePhysicsOptions{});
        ASSERT_TRUE(drone.has_value());
        drone->target = AttitudeTarget{0.0f, 0.0f, roll};
        ASSERT_TRUE(drone->step_fixed(roll == 0.0f ? 200u : 1500u).has_value());
        const glm::quat q = drone->readouts().orientation;
        auto field = air_field_from(*drone);
        ASSERT_TRUE(field.has_value());
        for (const float deg : {0.0f, 30.0f, 45.0f, 60.0f, 90.0f}) {
            const float a = glm::radians(deg);
            spade::render::Camera camera;
            camera.position = 3.0f * glm::vec3(std::sin(a), 0.2f, std::cos(a));
            const SliceSpec s = camera_facing_slice(camera, glm::vec3(0.0f), q);
            EXPECT_GT(max_speed_below_rotors(*field, s, glm::vec3(0.0f)), 5.0f)
                << "azimuth " << deg << " deg, roll " << roll << " rad";
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
