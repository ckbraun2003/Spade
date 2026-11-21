// test_camera_capacities.cpp -- Capacities::cameras and ::camera_pixels (Phase 3.1 of rendering/camera-sensor-design, section 11 item 2): declared like the
// other capacities, default 0 so a world that does not use a camera is unchanged in its file, its validation and its config hash. A world file carries
// them as schema version 4 (Phase 3.4; test_world_file_v4.cpp has the rest of that schema).

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "set_body_state_support.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "world/world_file.hpp"

namespace {

using spade::Capacities;
using spade::WorldDesc;
using spade::WorldSetDesc;

[[nodiscard]] WorldSetDesc floor_set() {
    spade::Result<WorldSetDesc> w = spade::test::floor_world_set(2);
    EXPECT_TRUE(w.has_value()) << w.error().context;
    return *w;
}

// The floor world built through the builder with these capacities.
[[nodiscard]] spade::Result<WorldDesc> build_with(uint32_t cameras, uint32_t camera_pixels, uint32_t keypoint_groups = 0,
                                                   uint32_t keypoints_per_group = 0) {
    Capacities caps;
    caps.bodies = 2;
    caps.force_elements = 4;
    caps.sensors = 2;
    caps.contacts = 1;
    caps.cameras = cameras;
    caps.camera_pixels = camera_pixels;
    caps.keypoint_groups = keypoint_groups;
    caps.keypoints_per_group = keypoints_per_group;
    return spade::WorldBuilder().name("floor").capacities(caps).plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).build();
}

}  // namespace

TEST(CameraCapacities, DefaultToZeroAndAWorldFileOfADefaultWorldHasNoCameraKeys) {
    const Capacities caps;
    EXPECT_EQ(caps.cameras, 0u);
    EXPECT_EQ(caps.camera_pixels, 0u);
    const WorldSetDesc set = floor_set();
    const spade::Result<std::string> text = spade::world_to_yaml(set.worlds[0].world);
    ASSERT_TRUE(text.has_value()) << text.error().context;
    EXPECT_EQ(text->find("camera"), std::string::npos) << "an existing world's file must not change";
}

TEST(CameraCapacities, CamerasNeedAPixelBudgetAndABudgetNeedsCameras) {
    EXPECT_TRUE(build_with(0, 0).has_value()) << "no camera: the default";
    EXPECT_TRUE(build_with(2, 1'228'800).has_value());
    const spade::Result<WorldDesc> no_budget = build_with(2, 0);
    ASSERT_FALSE(no_budget.has_value()) << "cameras with no pixel budget is an authoring mistake";
    EXPECT_NE(no_budget.error().context.find("camera_pixels"), std::string::npos) << no_budget.error().context;
    const spade::Result<WorldDesc> no_cameras = build_with(0, 4096);
    ASSERT_FALSE(no_cameras.has_value()) << "a pixel budget with no cameras is an authoring mistake";
    EXPECT_NE(no_cameras.error().context.find("cameras"), std::string::npos) << no_cameras.error().context;
}

TEST(CameraCapacities, AWorldFileCarriesACameraWorldAsVersion4) {
    // Phase 3.1 refused to write a camera world (no schema version carried the capacities); schema version 4 (Phase 3.4) carries them, so it round trips.
    const spade::Result<WorldDesc> world = build_with(2, 1'228'800);
    ASSERT_TRUE(world.has_value()) << world.error().context;
    const spade::Result<std::string> text = spade::world_to_yaml(*world);
    ASSERT_TRUE(text.has_value()) << text.error().context;
    EXPECT_NE(text->find("world_version: 4\n"), std::string::npos);
    const spade::Result<WorldDesc> loaded = spade::world_from_yaml(*text);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    EXPECT_EQ(loaded->capacities.cameras, 2u);
    EXPECT_EQ(loaded->capacities.camera_pixels, 1'228'800u);
}

TEST(CameraCapacities, TheConfigHashSeesThemOnlyWhenTheyAreUsed) {
    const WorldSetDesc base = floor_set();
    WorldSetDesc with_cameras = base;
    with_cameras.worlds[0].world.capacities.cameras = 1;
    with_cameras.worlds[0].world.capacities.camera_pixels = 4096;
    WorldSetDesc more_pixels = with_cameras;
    more_pixels.worlds[0].world.capacities.camera_pixels = 8192;
    EXPECT_NE(spade::config_hash(base), spade::config_hash(with_cameras)) << "a camera is world configuration";
    EXPECT_NE(spade::config_hash(with_cameras), spade::config_hash(more_pixels));
    EXPECT_EQ(spade::config_hash(base), spade::config_hash(floor_set())) << "and a world without one hashes as before";
}

TEST(CameraCapacities, TheKeypointCapacitiesDefaultToZeroAndComeAsAPair) {
    const Capacities caps;
    EXPECT_EQ(caps.keypoint_groups, 0u);
    EXPECT_EQ(caps.keypoints_per_group, 0u);
    EXPECT_TRUE(build_with(2, 1'228'800, 0, 0).has_value()) << "none: the default";
    EXPECT_TRUE(build_with(2, 1'228'800, 5, 4).has_value()) << "KAT's gates: G x 4";
    const spade::Result<WorldDesc> no_points = build_with(2, 1'228'800, 5, 0);
    ASSERT_FALSE(no_points.has_value()) << "groups with no points is an authoring mistake";
    EXPECT_NE(no_points.error().context.find("keypoints_per_group"), std::string::npos) << no_points.error().context;
    const spade::Result<WorldDesc> no_groups = build_with(2, 1'228'800, 0, 4);
    ASSERT_FALSE(no_groups.has_value()) << "points with no groups is an authoring mistake";
    EXPECT_NE(no_groups.error().context.find("keypoint_groups"), std::string::npos) << no_groups.error().context;
}

TEST(CameraCapacities, AWorldFileCarriesTheKeypointCapacitiesToo) {
    const spade::Result<WorldDesc> world = build_with(0, 0, 5, 4);
    ASSERT_TRUE(world.has_value()) << world.error().context;
    const spade::Result<std::string> text = spade::world_to_yaml(*world);
    ASSERT_TRUE(text.has_value()) << text.error().context;
    const spade::Result<WorldDesc> loaded = spade::world_from_yaml(*text);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    EXPECT_EQ(loaded->capacities.keypoint_groups, 5u);
    EXPECT_EQ(loaded->capacities.keypoints_per_group, 4u);
}

TEST(CameraCapacities, TheConfigHashSeesTheKeypointCapacitiesOnlyWhenTheyAreUsed) {
    const WorldSetDesc base = floor_set();
    WorldSetDesc with_keypoints = base;
    with_keypoints.worlds[0].world.capacities.keypoint_groups = 5;
    with_keypoints.worlds[0].world.capacities.keypoints_per_group = 4;
    WorldSetDesc more_groups = with_keypoints;
    more_groups.worlds[0].world.capacities.keypoint_groups = 6;
    EXPECT_NE(spade::config_hash(base), spade::config_hash(with_keypoints));
    EXPECT_NE(spade::config_hash(with_keypoints), spade::config_hash(more_groups));
    EXPECT_EQ(spade::config_hash(base), spade::config_hash(floor_set())) << "a world without them hashes as before";
}
