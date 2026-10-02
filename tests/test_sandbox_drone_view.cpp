// The drone sim box's view (sandbox/drone_view.hpp): the air field, the
// heatmap palette and slice, and the drone's parts. Display-free (SL15b);
// the two render cases run the CPU reference rasterizer.

#include <gtest/gtest.h>

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
        append_slice_items(b, camera_facing_slice(camera, glm::vec3(0.0f)), field, 10.0f, scene.dynamics);
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
        append_slice_items(b, camera_facing_slice(camera_on_z(3.0f), glm::vec3(0.0f)), *field, 0.0f, scene.dynamics);
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
    append_slice_items(b, camera_facing_slice(camera, glm::vec3(0.0f)), *field, 0.0f, scene.dynamics);
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
    std::vector<spade::render::DrawItem> items;
    append_drone_items(b, params, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), items);
    ASSERT_EQ(items.size(), 9u);  // body, then (arm, disc) per rotor

    for (std::size_t i = 0; i < 4; ++i) {
        const auto& arm = items[1 + 2 * i];
        const auto& disc = items[2 + 2 * i];
        EXPECT_EQ(arm.material_override, i == 0 ? 7u : 8u) << "rotor " << i;
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

TEST(SandboxDroneView, TheSliceFacesTheCameraAndStaysUpright) {
    spade::render::Camera camera;
    camera.position = glm::vec3(2.0f, 1.5f, -1.0f);
    const SliceSpec s = camera_facing_slice(camera, glm::vec3(0.0f));
    const glm::vec3 normal = glm::cross(s.right, s.up);
    const glm::vec3 to_cam = glm::normalize(glm::vec3(camera.position.x, 0.0f, camera.position.z));
    EXPECT_NEAR(glm::dot(normal, to_cam), 1.0f, 1e-5f);
    EXPECT_EQ(s.up, glm::vec3(0.0f, 1.0f, 0.0f));
    EXPECT_NEAR(s.right.y, 0.0f, 1e-7f);

    camera.position = glm::vec3(0.0f, 4.0f, 0.0f);  // straight overhead: degenerate, must stay finite
    const SliceSpec top = camera_facing_slice(camera, glm::vec3(0.0f));
    EXPECT_TRUE(std::isfinite(top.right.x) && std::isfinite(top.right.z));
}
