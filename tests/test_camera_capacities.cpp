// test_camera_capacities.cpp -- Capacities::cameras and ::camera_pixels (Phase 3.1 of rendering/camera-sensor-design, section 11 item 2): declared like the
// other capacities, default 0 so a world that does not use a camera is unchanged in its file, its validation and its config hash. A world file does not carry
// them yet (that is a schema version of its own, with the labels and materials of item 3.4): writing one is refused, never silently dropped.

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
[[nodiscard]] spade::Result<WorldDesc> build_with(uint32_t cameras, uint32_t camera_pixels) {
    Capacities caps;
    caps.bodies = 2;
    caps.force_elements = 4;
    caps.sensors = 2;
    caps.contacts = 1;
    caps.cameras = cameras;
    caps.camera_pixels = camera_pixels;
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

TEST(CameraCapacities, AWorldFileRefusesToWriteACameraWorldRatherThanDropItsCameras) {
    const spade::Result<WorldDesc> world = build_with(2, 1'228'800);
    ASSERT_TRUE(world.has_value()) << world.error().context;
    const spade::Result<std::string> text = spade::world_to_yaml(*world);
    ASSERT_FALSE(text.has_value()) << "silently writing a file that loses the cameras is the failure";
    EXPECT_NE(text.error().context.find("camera"), std::string::npos) << text.error().context;
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
