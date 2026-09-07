// test_render_velocity.cpp -- Plan A Task 10 (24th spec SL9c): DrawMode::
// velocity, closing the v1 RenderVelocity / Velocity.frag row of the SL7
// transfer register.
//
// The mode is only worth having if speed changes the pixels, and only safe to
// add if it changes nothing else. Both halves are asserted here.
//
// ON REUSE: the plan said to build on test_render_raster.cpp's scene and
// target construction. Those helpers live in that file's ANONYMOUS namespace
// and are unreachable from another TU, so the two this file needs are spelled
// again below -- deliberately minimal (one triangle, not a box) rather than
// copied wholesale, and noted here so nobody reads the duplication as an
// oversight.

#include <gtest/gtest.h>

#include <cstdint>
#include <span>
#include <vector>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::DrawMode;
using spade::render::kNoMaterial;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::PixelFormat;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

namespace {

constexpr uint32_t kWidth = 64;
constexpr uint32_t kHeight = 64;

[[nodiscard]] RenderTarget make_target(std::vector<uint8_t>& storage) {
    storage.assign(static_cast<size_t>(kWidth) * 4 * kHeight, 0xAAu);
    return RenderTarget{
        .pixels = std::span<uint8_t>(storage),
        .width = kWidth,
        .height = kHeight,
        .stride = kWidth * 4,
        .format = PixelFormat::bgrx8,
    };
}

// One triangle at z = +1, outward normal +Z, wound CCW as seen from a camera
// on the +Z side looking toward -Z (the renderer's canonical front face).
[[nodiscard]] MeshData single_triangle() {
    MeshData mesh;
    mesh.positions = {glm::vec3(1.0f, -1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f),
                      glm::vec3(-1.0f, 1.0f, 1.0f)};
    mesh.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 1, 2};
    return mesh;
}

// The moving thing is a DYNAMIC item, because speed belongs to a body: a
// static's speed is 0 and always will be.
[[nodiscard]] RenderScene scene_with_speed(float speed_mps) {
    RenderScene scene;
    scene.meshes.push_back(single_triangle());
    scene.materials = {Material{}};
    scene.dynamics.push_back(DrawItem{.mesh_index = 0,
                                      .local_to_world = glm::mat4(1.0f),
                                      .material_override = kNoMaterial,
                                      .speed_mps = speed_mps});
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    return scene;
}

[[nodiscard]] Camera front_camera() {
    Camera camera;  // identity orientation looks toward world -Z
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    return camera;
}

// Renders and returns the framebuffer bytes. Overlays off, so the comparison
// is about the geometry's shading and nothing else.
[[nodiscard]] std::vector<uint8_t> render_at_speed(float speed_mps, DrawMode mode,
                                                   float scale_mps = 20.0f) {
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage);
    const RenderScene scene = scene_with_speed(speed_mps);
    RenderOptions options;
    options.mode = mode;
    options.overlays = false;
    options.shadows = false;
    options.velocity_scale_mps = scale_mps;

    const spade::Result<void> result = spade::render::render(scene, front_camera(), options, target);
    if (!result) {
        ADD_FAILURE() << "render() failed: " << result.error().context;
        return {};
    }
    return storage;
}

}  // namespace

// The mode is only useful if speed changes the pixels.
TEST(VelocityDrawMode, FastAndSlowBodiesRenderDifferently) {
    const std::vector<uint8_t> slow = render_at_speed(0.0f, DrawMode::velocity);
    const std::vector<uint8_t> fast = render_at_speed(20.0f, DrawMode::velocity);
    ASSERT_FALSE(slow.empty());
    ASSERT_FALSE(fast.empty());
    EXPECT_NE(slow, fast);
}

TEST(VelocityDrawMode, IsDeterministic) {
    EXPECT_EQ(render_at_speed(7.5f, DrawMode::velocity), render_at_speed(7.5f, DrawMode::velocity));
}

// Clamped above the scale, never wrapped -- a fast outlier must read as "at
// least this fast". Wrapping would send 40 m/s back through the slow colours
// and paint a speeding body as though it were at rest, which is worse than
// uninformative: it is confidently wrong.
TEST(VelocityDrawMode, ClampsAboveTheScaleRatherThanWrapping) {
    const std::vector<uint8_t> at_scale = render_at_speed(20.0f, DrawMode::velocity, 20.0f);
    const std::vector<uint8_t> far_above = render_at_speed(200.0f, DrawMode::velocity, 20.0f);
    ASSERT_FALSE(at_scale.empty());
    EXPECT_EQ(at_scale, far_above) << "speeds above the scale must saturate, not wrap";
    EXPECT_NE(render_at_speed(0.0f, DrawMode::velocity, 20.0f), far_above);
}

// A non-positive scale would divide by zero. Inert-and-saturated, not NaN.
TEST(VelocityDrawMode, ADegenerateScaleDoesNotProduceGarbage) {
    const std::vector<uint8_t> zero_scale = render_at_speed(5.0f, DrawMode::velocity, 0.0f);
    ASSERT_FALSE(zero_scale.empty());
    EXPECT_EQ(zero_scale, render_at_speed(5.0f, DrawMode::velocity, 0.0f)) << "not deterministic";
}

// THE HALF THAT PROTECTS EVERY EXISTING RENDER GOLDEN. Adding an enumerator
// and a DrawItem field must not change what the other modes produce -- so a
// scene whose speed differs must render IDENTICALLY under shaded and under
// wireframe. If `speed_mps` ever leaks into another mode's output, this is
// what catches it; the committed render goldens elsewhere in the suite catch
// the pinned-pixel half.
TEST(VelocityDrawMode, DoesNotDisturbTheOtherDrawModes) {
    EXPECT_EQ(render_at_speed(0.0f, DrawMode::shaded), render_at_speed(20.0f, DrawMode::shaded))
        << "speed changed shaded output";
    EXPECT_EQ(render_at_speed(0.0f, DrawMode::wireframe), render_at_speed(20.0f, DrawMode::wireframe))
        << "speed changed wireframe output";
}

// The data path: BodyPose carries a velocity VECTOR and update_dynamics()
// reduces it to the magnitude DrawItem stores. Without this the mode would
// paint whatever a caller happened to leave in the field.
TEST(VelocityDrawMode, UpdateDynamicsCarriesSpeedFromTheBodyPose) {
    RenderScene scene;
    scene.meshes.push_back(single_triangle());
    scene.materials = {Material{}};

    const spade::render::BodyPose poses[] = {
        {.position = glm::vec3(0.0f), .mesh_index = 0, .velocity = glm::vec3(3.0f, 4.0f, 0.0f)},
        {.position = glm::vec3(2.0f, 0.0f, 0.0f), .mesh_index = 0, .velocity = glm::vec3(0.0f)},
    };
    spade::render::update_dynamics(scene, poses);

    ASSERT_EQ(scene.dynamics.size(), 2u);
    EXPECT_FLOAT_EQ(scene.dynamics[0].speed_mps, 5.0f) << "3-4-5, so the magnitude is exact";
    EXPECT_FLOAT_EQ(scene.dynamics[1].speed_mps, 0.0f);
}
