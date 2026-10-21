// The sensor draw modes on the CPU (N13; docs/design/rendering/plans/
// 2026-10-20-sensor-views-design.md): depth and normals draw the scene's
// geometry as data. Each test pins one rule of the design: planar depth
// through a stated range, black where no surface is, nothing of light,
// shadow, material or overlay, exact world normals interpolated
// perspective-correctly, the analytic ground, and CSG marched.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/segmentation.hpp"
#include "render/transfer_function.hpp"
#include "world/sdf.hpp"

namespace {

using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::DrawMode;
using spade::render::MeshData;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using Rgb = std::array<int, 3>;

constexpr uint32_t kSide = 64;
const float kTanHalf = std::tan(0.5f * Camera{}.fov_y_radians);

// A quad with corners a, b, c, d (counter-clockwise seen from the front) and
// a normal per corner.
[[nodiscard]] MeshData quad(const std::array<glm::vec3, 4>& p, const std::array<glm::vec3, 4>& n) {
    MeshData m;
    m.positions.assign(p.begin(), p.end());
    m.normals.assign(n.begin(), n.end());
    m.indices = {0, 1, 2, 0, 2, 3};
    return m;
}

[[nodiscard]] MeshData flat_quad(const std::array<glm::vec3, 4>& p, glm::vec3 normal) {
    return quad(p, {normal, normal, normal, normal});
}

// A wall facing +Z, x and y in [-half, half], at z.
[[nodiscard]] MeshData wall(float z, float half) {
    return flat_quad({glm::vec3(-half, -half, z), glm::vec3(half, -half, z), glm::vec3(half, half, z),
                      glm::vec3(-half, half, z)},
                     glm::vec3(0.0f, 0.0f, 1.0f));
}

[[nodiscard]] RenderScene scene_of(std::vector<MeshData> meshes) {
    RenderScene scene;
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-50.0f), .max = glm::vec3(50.0f)};
    scene.materials = {spade::render::Material{}};
    for (MeshData& m : meshes) {
        DrawItem item;
        item.mesh_index = static_cast<uint32_t>(scene.meshes.size());
        scene.meshes.push_back(std::move(m));
        scene.statics.push_back(item);
    }
    return scene;
}

struct Frame {
    std::vector<uint8_t> bgrx = std::vector<uint8_t>(kSide * kSide * 4u, 0u);
    [[nodiscard]] Rgb at(uint32_t x, uint32_t y) const {
        const std::size_t i = (static_cast<std::size_t>(y) * kSide + x) * 4u;
        return {bgrx[i + 2], bgrx[i + 1], bgrx[i]};
    }
};

[[nodiscard]] Frame draw(const RenderScene& scene, const RenderOptions& options, const Camera& camera = Camera{}) {
    Frame f;
    spade::render::RenderTarget target{.pixels = std::span<uint8_t>(f.bgrx), .width = kSide, .height = kSide,
                                       .stride = kSide * 4u};
    const auto drew = spade::render::render(scene, camera, options, target);
    EXPECT_TRUE(drew.has_value()) << drew.error().context;
    return f;
}

[[nodiscard]] RenderOptions sensor(DrawMode mode) {
    RenderOptions o;
    o.mode = mode;
    o.overlays = false;
    return o;
}

[[nodiscard]] Rgb bytes(glm::vec3 c) {
    return {spade::render::to_byte(c.r), spade::render::to_byte(c.g), spade::render::to_byte(c.b)};
}

// Depth's colour, as the design states it: view depth over the range, clamped.
[[nodiscard]] Rgb depth_colour(float z, const spade::render::DepthRange& r) {
    const float t = std::clamp((z - r.near_m) / (r.far_m - r.near_m), 0.0f, 1.0f);
    return bytes(spade::render::colour_map_at(r.colours, t));
}

[[nodiscard]] Rgb normal_colour(glm::vec3 n) { return bytes(0.5f * (glm::normalize(n) + 1.0f)); }

[[nodiscard]] int distance(const Rgb& a, const Rgb& b) {
    return std::max({std::abs(a[0] - b[0]), std::abs(a[1] - b[1]), std::abs(a[2] - b[2])});
}

// Whether `c` lies between colours `lo` and `hi` per channel, `slack` either side.
[[nodiscard]] bool between(const Rgb& c, const Rgb& lo, const Rgb& hi, int slack) {
    for (int k = 0; k < 3; ++k) {
        if (c[k] < std::min(lo[k], hi[k]) - slack || c[k] > std::max(lo[k], hi[k]) + slack) return false;
    }
    return true;
}

// The NDC y of a row's top and bottom edges (top row first).
[[nodiscard]] std::pair<float, float> row_edges(uint32_t y) {
    return {1.0f - 2.0f * static_cast<float>(y) / kSide, 1.0f - 2.0f * static_cast<float>(y + 1u) / kSide};
}

const Rgb kBlack{0, 0, 0};

// Planar depth, not radial range: a wall square to the camera is one depth
// across its whole width, the colour the range gives it, and where no
// surface is the frame is black (shaded would draw sky there).
TEST(SensorViews, AWallSquareToTheCameraIsOneDepthAcrossItsWidth) {
    const RenderScene scene = scene_of({wall(-3.0f, 1.5f)});
    RenderOptions options = sensor(DrawMode::depth);
    options.depth = spade::render::DepthRange{.near_m = 0.5f, .far_m = 5.5f};
    const Frame f = draw(scene, options);
    const Rgb want = depth_colour(3.0f, options.depth);
    uint32_t covered = 0;
    for (uint32_t y = 6; y < kSide - 6; ++y) {
        for (uint32_t x = 6; x < kSide - 6; ++x) {
            ASSERT_LE(distance(f.at(x, y), want), 1) << "at (" << x << ", " << y << ")";
            ++covered;
        }
    }
    EXPECT_GT(covered, kSide * kSide / 2u);
    EXPECT_EQ(f.at(0, 0), kBlack) << "no surface: black";
    EXPECT_EQ(f.at(kSide - 1, kSide - 1), kBlack);
    EXPECT_NE(draw(scene, sensor(DrawMode::shaded)).at(0, 0), kBlack) << "shaded draws sky there";
}

// Outside the range, depth clamps to its ends.
TEST(SensorViews, DepthClampsToTheEndsOfItsRange) {
    const RenderScene scene = scene_of({wall(-3.0f, 1.5f)});
    for (const auto& [range, end] : {std::pair{spade::render::DepthRange{.near_m = 0.5f, .far_m = 2.0f}, 1.0f},
                                     std::pair{spade::render::DepthRange{.near_m = 4.0f, .far_m = 8.0f}, 0.0f}}) {
        RenderOptions options = sensor(DrawMode::depth);
        options.depth = range;
        EXPECT_LE(distance(draw(scene, options).at(kSide / 2, kSide / 2),
                           bytes(spade::render::colour_map_at(range.colours, end))),
                  1)
            << range.near_m << " to " << range.far_m;
    }
}

// Data, not appearance: a sensor frame is the same whatever the material,
// the light, the shadows and the overlays.
TEST(SensorViews, ASensorFrameTakesNoMaterialLightShadowOrOverlay) {
    RenderScene plain = scene_of({wall(-3.0f, 1.5f), flat_quad({glm::vec3(-1, -1, -2), glm::vec3(0, -1, -2),
                                                                glm::vec3(0, 0, -2), glm::vec3(-1, 0, -2)},
                                                               glm::vec3(0, 0, 1))});
    RenderScene dressed = plain;
    dressed.materials = {spade::render::Material{.base_color = glm::vec4(0.9f, 0.2f, 0.1f, 1.0f), .shading = 0u}};
    dressed.lighting.sun_direction = glm::vec3(0.3f, -1.0f, -0.6f);
    dressed.lighting.ambient_color = glm::vec3(0.05f);
    dressed.lighting.sky_zenith = glm::vec3(0.9f, 0.1f, 0.1f);
    plain.statics[1].object_label = spade::render::intern_label(plain, "near");
    dressed.statics[1].object_label = spade::render::intern_label(dressed, "near");
    for (const DrawMode mode : {DrawMode::depth, DrawMode::normals, DrawMode::segmentation}) {
        SCOPED_TRACE(static_cast<int>(mode));
        RenderOptions bare = sensor(mode);
        bare.shadows = false;
        RenderOptions busy = sensor(mode);
        busy.shadows = true;
        busy.overlays = true;
        busy.spawn_markers = true;
        busy.ground_grid = true;
        busy.horizon_blend_strength = 1.0f;
        EXPECT_EQ(draw(plain, bare).bgrx, draw(dressed, busy).bgrx);
    }
}

// A tilted wall's pixels are its world normal, exactly.
TEST(SensorViews, NormalsOfATiltedWallAreItsWorldNormal) {
    RenderScene scene = scene_of({wall(0.0f, 1.5f)});
    const glm::mat4 turn = glm::rotate(glm::mat4(1.0f), 0.5f, glm::vec3(0.0f, 1.0f, 0.0f)) *
                           glm::rotate(glm::mat4(1.0f), -0.35f, glm::vec3(1.0f, 0.0f, 0.0f));
    scene.statics[0].local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -3.0f)) * turn;
    const Rgb want = normal_colour(glm::vec3(turn * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f)));
    const Frame f = draw(scene, sensor(DrawMode::normals));
    EXPECT_LE(distance(f.at(kSide / 2, kSide / 2), want), 1);
    EXPECT_LE(distance(f.at(kSide / 2 - 8, kSide / 2 + 5), want), 1);
    EXPECT_EQ(f.at(0, 0), kBlack);
}

// Normals interpolate perspective-correctly: along a floor whose normal turns
// from +Y at 1 m to +X at 41 m, a row's normal is the one at the floor's
// depth there. Interpolated in screen space, the far half of the floor would
// hold most of the turn.
TEST(SensorViews, NormalsInterpolatePerspectiveCorrectlyAlongTheFloor) {
    const glm::vec3 up(0.0f, 1.0f, 0.0f), side(1.0f, 0.0f, 0.0f);
    const RenderScene scene = scene_of({quad({glm::vec3(-20, -1, -1), glm::vec3(20, -1, -1), glm::vec3(20, -1, -41),
                                              glm::vec3(-20, -1, -41)},
                                             {up, up, side, side})});
    const Frame f = draw(scene, sensor(DrawMode::normals));
    // The floor y = -1 seen at NDC y (< 0) lies at view depth 1 / (-y tan).
    const auto colour_at = [&](float ndc_y) {
        const float z = 1.0f / (-ndc_y * kTanHalf);
        const float s = std::clamp((z - 1.0f) / 40.0f, 0.0f, 1.0f);
        return normal_colour((1.0f - s) * up + s * side);
    };
    uint32_t checked = 0;
    for (uint32_t y = kSide / 2 + 2; y < kSide; y += 3) {
        const auto [top, bottom] = row_edges(y);
        if (1.0f / (-bottom * kTanHalf) < 1.0f) continue;  // nearer than the floor's start
        EXPECT_TRUE(between(f.at(kSide / 2, y), colour_at(top), colour_at(bottom), 2))
            << "row " << y << ": got (" << f.at(kSide / 2, y)[0] << ", " << f.at(kSide / 2, y)[1] << ", "
            << f.at(kSide / 2, y)[2] << ")";
        ++checked;
    }
    EXPECT_GT(checked, 6u);
}

// The analytic ground is the world's surface: sensor frames draw it with
// its depth and its normal, past any ground mesh, and black above the horizon.
TEST(SensorViews, TheAnalyticGroundDrawsInSensorFrames) {
    RenderScene scene = scene_of({});
    scene.ground_planes.push_back(spade::render::GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = -1.0f});
    const Frame normals = draw(scene, sensor(DrawMode::normals));
    EXPECT_LE(distance(normals.at(kSide / 2, kSide - 2), normal_colour(glm::vec3(0.0f, 1.0f, 0.0f))), 1);
    EXPECT_EQ(normals.at(kSide / 2, 2), kBlack) << "above the horizon: no surface";

    RenderOptions options = sensor(DrawMode::depth);
    options.depth = spade::render::DepthRange{.near_m = 0.5f, .far_m = 6.0f};
    const Frame depth = draw(scene, options);
    for (uint32_t y = kSide / 2 + 6; y < kSide; y += 5) {
        const auto [top, bottom] = row_edges(y);
        EXPECT_TRUE(between(depth.at(kSide / 2, y), depth_colour(1.0f / (-top * kTanHalf), options.depth),
                            depth_colour(1.0f / (-bottom * kTanHalf), options.depth), 2))
            << "row " << y;
    }
    EXPECT_EQ(depth.at(kSide / 2, 2), kBlack);
}

// A CSG subtree is marched in sensor frames, as in shaded: a box whose front
// face is at 2.5 m, with its mesh emptied so only the march can draw it.
TEST(SensorViews, ACsgSubtreeIsMarchedInSensorFrames) {
    MeshData empty;
    RenderScene scene = scene_of({empty});
    spade::SdfProgram box;
    spade::SdfTransform at;
    at.world_to_local = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 3.0f));
    box.transforms.push_back(at);
    spade::SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    node.params = glm::vec4(0.5f, 0.5f, 0.5f, 0.0f);
    box.nodes.push_back(node);
    scene.csg_subtrees.push_back(spade::render::CsgSubtree{
        .program = box,
        .bounds = spade::render::Aabb{.min = glm::vec3(-0.6f, -0.6f, -3.6f), .max = glm::vec3(0.6f, 0.6f, -2.4f)},
        .draw_item = 0});
    RenderOptions options = sensor(DrawMode::depth);
    options.depth = spade::render::DepthRange{.near_m = 0.5f, .far_m = 4.5f};
    EXPECT_LE(distance(draw(scene, options).at(kSide / 2, kSide / 2), depth_colour(2.5f, options.depth)), 2);
    EXPECT_LE(distance(draw(scene, sensor(DrawMode::normals)).at(kSide / 2, kSide / 2),
                       normal_colour(glm::vec3(0.0f, 0.0f, 1.0f))),
              2);
    scene.statics[0].object_label = spade::render::intern_label(scene, "gate");
    const spade::render::Rgb8 gate = spade::render::segment_colour("gate");
    EXPECT_EQ(draw(scene, sensor(DrawMode::segmentation)).at(kSide / 2, kSide / 2), (Rgb{gate[0], gate[1], gate[2]}));
}

}  // namespace
