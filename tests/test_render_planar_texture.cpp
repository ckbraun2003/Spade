// The planar texture (render/planar_texture.hpp, SP-4 branch 2; docs/design/rendering/plans/2026-11-17-geo-terrain-design.md): an aerial image laid over the draw items that ask for it, mapped
// by world position, drawn by the CPU raster (the GL branch is the next).
//
//   PlanarTexture.Sampler*      bilinear between texel centres, clamped at the edge, row 0 at z_min
//   PlanarTexture.Refuses*      a malformed image or extent, and an item that asks for a texture the scene lacks, are refused by name
//   PlanarTexture.Frame*        a constant image equals the plain material's frame; a checker's colours land on their quadrants with north up; an oblique view is
//                               perspective-correct (each pixel samples the texture at its own world point); no item asking for it, no change
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "render/planar_texture.hpp"
#include "render/raster_cpu.hpp"
#include "render/raymarch.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render/terrain.hpp"
#include "world/terrain_input.hpp"

namespace {

using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::PixelFormat;
using spade::render::PlanarTexture;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

constexpr uint32_t kW = 160;
constexpr uint32_t kH = 120;

struct Rgb {
    int r, g, b;
};

// A 2 x 2 image over x, z in [-10, 10]: row 0 (north, z = -10..0) red then green, row 1 (south) blue then yellow.
[[nodiscard]] PlanarTexture quadrants() {
    PlanarTexture t;
    t.width = 2;
    t.height = 2;
    t.rgb = {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0};
    t.x_min = -10.0f;
    t.x_max = 10.0f;
    t.z_min = -10.0f;
    t.z_max = 10.0f;
    return t;
}

[[nodiscard]] PlanarTexture constant_image(uint8_t r, uint8_t g, uint8_t b, float half) {
    PlanarTexture t;
    t.width = 4;
    t.height = 4;
    for (int i = 0; i < 16; ++i) {
        t.rgb.push_back(r);
        t.rgb.push_back(g);
        t.rgb.push_back(b);
    }
    t.x_min = t.z_min = -half;
    t.x_max = t.z_max = half;
    return t;
}

// A checker of n x n texels over [-half, half]^2, two colours.
[[nodiscard]] PlanarTexture checker(uint32_t n, float half) {
    PlanarTexture t;
    t.width = t.height = n;
    for (uint32_t j = 0; j < n; ++j) {
        for (uint32_t i = 0; i < n; ++i) {
            const bool a = ((i + j) & 1u) == 0u;
            t.rgb.push_back(a ? 230 : 30);
            t.rgb.push_back(40);
            t.rgb.push_back(a ? 40 : 230);
        }
    }
    t.x_min = t.z_min = -half;
    t.x_max = t.z_max = half;
    return t;
}

// One quad at y = 0 over x, z in [-half, half], facing up, as the item `textured` or not, in a material of the given base colour.
[[nodiscard]] RenderScene quad_scene(float half, glm::vec4 base, bool textured) {
    RenderScene scene;
    scene.materials = {Material{base, 0u}};
    scene.bounds = {glm::vec3(-500.0f, -5.0f, -500.0f), glm::vec3(500.0f, 50.0f, 500.0f)};
    MeshData mesh;
    mesh.positions = {glm::vec3(-half, 0.0f, -half), glm::vec3(-half, 0.0f, half), glm::vec3(half, 0.0f, -half), glm::vec3(half, 0.0f, half)};
    mesh.normals.assign(4, glm::vec3(0.0f, 1.0f, 0.0f));
    mesh.indices = {0, 1, 2, 2, 1, 3};
    scene.meshes.push_back(std::move(mesh));
    DrawItem item{.mesh_index = 0, .local_to_world = glm::mat4(1.0f)};
    item.planar_textured = textured;
    scene.statics.push_back(item);
    return scene;
}

[[nodiscard]] Camera camera_over(float height) {
    Camera camera;
    camera.position = glm::vec3(0.0f, height, 0.0f);
    camera.orientation = glm::angleAxis(-1.5707963267948966f, glm::vec3(1.0f, 0.0f, 0.0f));  // straight down, north (z = -north) up the frame
    return camera;
}

[[nodiscard]] std::vector<Rgb> frame_of(const RenderScene& scene, const Camera& camera) {
    std::vector<uint8_t> storage(static_cast<size_t>(kW) * kH * 4u, 0u);
    RenderTarget target{.pixels = std::span<uint8_t>(storage), .width = kW, .height = kH, .stride = kW * 4u, .format = PixelFormat::bgrx8};
    RenderOptions options;
    options.overlays = false;
    options.horizon_blend_strength = 0.0f;  // the haze is position dependent and acts after the colour; these frames compare colours
    if (auto drew = spade::render::render(scene, camera, options, target); !drew) {
        ADD_FAILURE() << "render() failed: " << drew.error().context;
        return {};
    }
    std::vector<Rgb> out(static_cast<size_t>(kW) * kH);
    for (size_t p = 0; p < out.size(); ++p) out[p] = Rgb{storage[p * 4u + 2u], storage[p * 4u + 1u], storage[p * 4u]};
    return out;
}

}  // namespace

TEST(PlanarTexture, SamplerReturnsTheTexelAtItsCentreTheMeanBetweenAndClampsAtTheEdge) {
    const PlanarTexture t = quadrants();  // texel centres at +-5
    const auto near_colour = [](glm::vec3 got, glm::vec3 want, const char* what) {
        EXPECT_NEAR(got.x, want.x, 1e-5f) << what;
        EXPECT_NEAR(got.y, want.y, 1e-5f) << what;
        EXPECT_NEAR(got.z, want.z, 1e-5f) << what;
    };
    near_colour(spade::render::sample_planar_texture(t, -5.0f, -5.0f), glm::vec3(1, 0, 0), "NW centre: red");
    near_colour(spade::render::sample_planar_texture(t, 5.0f, -5.0f), glm::vec3(0, 1, 0), "NE centre: green (x east, columns west to east)");
    near_colour(spade::render::sample_planar_texture(t, -5.0f, 5.0f), glm::vec3(0, 0, 1), "SW centre: blue (z south, rows north to south)");
    near_colour(spade::render::sample_planar_texture(t, 5.0f, 5.0f), glm::vec3(1, 1, 0), "SE centre: yellow");
    near_colour(spade::render::sample_planar_texture(t, 0.0f, -5.0f), glm::vec3(0.5f, 0.5f, 0.0f), "between NW and NE: their mean");
    near_colour(spade::render::sample_planar_texture(t, 0.0f, 0.0f), glm::vec3(0.5f, 0.5f, 0.25f), "the middle: the mean of the four");
    near_colour(spade::render::sample_planar_texture(t, -50.0f, -50.0f), glm::vec3(1, 0, 0), "past the NW corner: clamped to its texel");
    near_colour(spade::render::sample_planar_texture(t, 50.0f, 50.0f), glm::vec3(1, 1, 0), "past the SE corner: clamped");
}

TEST(PlanarTexture, AMalformedImageOrExtentAndAnItemAskingForNoTextureAreRefusedByName) {
    EXPECT_TRUE(spade::render::validate_planar_texture(quadrants()).has_value());
    const auto refused = [](PlanarTexture t, const char* word) {
        const auto r = spade::render::validate_planar_texture(t);
        ASSERT_FALSE(r.has_value()) << word;
        EXPECT_NE(r.error().context.find(word), std::string::npos) << r.error().context;
    };
    PlanarTexture t = quadrants();
    t.width = 0;
    refused(t, "width");
    t = quadrants();
    t.rgb.pop_back();
    refused(t, "bytes");
    t = quadrants();
    t.x_max = t.x_min;
    refused(t, "extent");
    t = quadrants();
    t.z_min = std::numeric_limits<float>::quiet_NaN();
    refused(t, "extent");
    // render() refuses a scene whose item asks for a texture it does not have, and one with a malformed texture
    RenderScene scene = quad_scene(10.0f, glm::vec4(1.0f), true);
    std::vector<uint8_t> storage(static_cast<size_t>(kW) * kH * 4u, 0u);
    RenderTarget target{.pixels = std::span<uint8_t>(storage), .width = kW, .height = kH, .stride = kW * 4u, .format = PixelFormat::bgrx8};
    const auto no_texture = spade::render::render(scene, camera_over(20.0f), RenderOptions{}, target);
    ASSERT_FALSE(no_texture.has_value());
    EXPECT_NE(no_texture.error().context.find("planar"), std::string::npos) << no_texture.error().context;
    scene.planar_texture = quadrants();
    scene.planar_texture->rgb.pop_back();
    EXPECT_FALSE(spade::render::render(scene, camera_over(20.0f), RenderOptions{}, target).has_value());
}

TEST(PlanarTexture, AConstantImageDrawsTheSameFrameAsThatColourAsTheMaterial) {
    const RenderScene plain = quad_scene(10.0f, glm::vec4(0.8f, 0.5f, 0.2f, 1.0f), false);
    RenderScene textured = quad_scene(10.0f, glm::vec4(0.1f, 0.9f, 0.9f, 1.0f), true);  // the material's own colour is not drawn
    textured.planar_texture = constant_image(204, 128, 51, 10.0f);
    const Camera camera = camera_over(20.0f);
    const std::vector<Rgb> a = frame_of(plain, camera);
    const std::vector<Rgb> b = frame_of(textured, camera);
    ASSERT_EQ(a.size(), b.size());
    int worst = 0;
    for (size_t p = 0; p < a.size(); ++p) worst = std::max({worst, std::abs(a[p].r - b[p].r), std::abs(a[p].g - b[p].g), std::abs(a[p].b - b[p].b)});
    EXPECT_LE(worst, 1) << "within one level";
}

TEST(PlanarTexture, AFourColourCheckerPutsEachColourOnItsQuadrantWithNorthUp) {
    // Looking straight down with the camera's up toward -z (north), the image's top row is the frame's top: red top left, green top right, blue bottom left, yellow bottom right.
    RenderScene scene = quad_scene(10.0f, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f), true);
    scene.planar_texture = quadrants();
    const RenderScene grey = quad_scene(10.0f, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f), false);
    const Camera camera = camera_over(20.0f);
    const std::vector<Rgb> frame = frame_of(scene, camera);
    const std::vector<Rgb> light = frame_of(grey, camera);  // 0.5 x the lighting: the textured pixel is albedo x lighting = albedo x 2 x this
    ASSERT_EQ(frame.size(), static_cast<size_t>(kW) * kH);
    struct Probe {
        int x, y;
        float r, g, b;
        const char* what;
    };
    // the quad's quadrant centres (+-5, +-5) at 5.196 px a metre about the centre (80, 60)
    const Probe probes[] = {{54, 34, 1, 0, 0, "north-west: red"}, {106, 34, 0, 1, 0, "north-east: green"}, {54, 86, 0, 0, 1, "south-west: blue"}, {106, 86, 1, 1, 0, "south-east: yellow"}};
    for (const Probe& pr : probes) {
        const size_t i = static_cast<size_t>(pr.y) * kW + static_cast<size_t>(pr.x);
        const auto want = [&](float albedo, int lit) { return std::min(255, static_cast<int>(std::lround(albedo * 2.0f * lit))); };
        EXPECT_NEAR(frame[i].r, want(pr.r, light[i].r), 3.0f) << pr.what;
        EXPECT_NEAR(frame[i].g, want(pr.g, light[i].g), 3.0f) << pr.what;
        EXPECT_NEAR(frame[i].b, want(pr.b, light[i].b), 3.0f) << pr.what;
    }
}

TEST(PlanarTexture, AnObliqueViewSamplesTheTextureAtEachPixelsOwnWorldPointNotAnAffineBlend) {
    // One big quad (two triangles) seen at a grazing angle: the case where screen-space affine interpolation of the position visibly differs from the true one.
    const float half = 100.0f;
    RenderScene scene = quad_scene(half, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f), true);
    scene.planar_texture = checker(16, half);
    const RenderScene grey = quad_scene(half, glm::vec4(0.5f, 0.5f, 0.5f, 1.0f), false);
    Camera camera;
    camera.position = glm::vec3(0.0f, 3.0f, 60.0f);
    camera.orientation = glm::angleAxis(-0.18f, glm::vec3(1.0f, 0.0f, 0.0f));  // looking a little down, along -z
    const std::vector<Rgb> frame = frame_of(scene, camera);
    const std::vector<Rgb> light = frame_of(grey, camera);
    const spade::render::RayGrid grid = spade::render::ray_grid(camera, kW, kH);
    const PlanarTexture& texture = *scene.planar_texture;
    int checked = 0, bad = 0;
    for (uint32_t y = 48; y < kH; y += 4) {
        for (uint32_t x = 5; x < kW; x += 15) {
            const spade::render::CameraRay ray = spade::render::camera_ray(grid, x, y);
            if (!(ray.dir_world.y < -1e-4f)) continue;
            const float t = -grid.origin.y / ray.dir_world.y;
            const glm::vec3 hit = grid.origin + ray.dir_world * t;
            if (std::fabs(hit.x) > half - 1.0f || std::fabs(hit.z) > half - 1.0f) continue;
            const glm::vec3 albedo = spade::render::sample_planar_texture(texture, hit.x, hit.z);
            const size_t i = static_cast<size_t>(y) * kW + x;
            ++checked;
            const auto want = [&](float a, int lit) { return std::min(255, static_cast<int>(std::lround(a * 2.0f * lit))); };
            if (std::abs(frame[i].r - want(albedo.x, light[i].r)) > 4 || std::abs(frame[i].g - want(albedo.y, light[i].g)) > 4 || std::abs(frame[i].b - want(albedo.z, light[i].b)) > 4) ++bad;
        }
    }
    EXPECT_GT(checked, 40);
    EXPECT_EQ(bad, 0) << bad << " of " << checked << " pixels sampled the wrong world point";
}

TEST(PlanarTexture, ATextureNoItemAsksForChangesNothing) {
    const RenderScene without = quad_scene(10.0f, glm::vec4(0.8f, 0.5f, 0.2f, 1.0f), false);
    RenderScene with = quad_scene(10.0f, glm::vec4(0.8f, 0.5f, 0.2f, 1.0f), false);
    with.planar_texture = quadrants();  // present, but the item does not ask
    const Camera camera = camera_over(20.0f);
    const std::vector<Rgb> a = frame_of(without, camera);
    const std::vector<Rgb> b = frame_of(with, camera);
    ASSERT_EQ(a.size(), b.size());
    for (size_t p = 0; p < a.size(); ++p) {
        ASSERT_TRUE(a[p].r == b[p].r && a[p].g == b[p].g && a[p].b == b[p].b) << "pixel " << p;
    }
}

// attach_terrain with spec.textured (SP-4): the map's image goes into the scene at the map's extent (x = east, z = -north, so the image's top row is the north edge), the terrain's item is marked, and a frame
// shows the image's colours, north up. Without `textured` nothing changes: no texture in the scene, the item unmarked.
TEST(PlanarTexture, AttachTerrainPutsTheMapsImageAtItsExtentWhenAskedAndLeavesThePlainMapAloneOtherwise) {
    const auto make = [] {
        spade::world::TerrainInput map;
        map.nx = 20;
        map.ny = 20;
        map.extent = {-10.0, 10.0, -10.0, 10.0};  // east_min, east_max, north_min, north_max
        map.heights.assign(20u * 20u, 0.0f);
        map.image.width = 2;
        map.image.height = 2;
        map.image.rgb = {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0};  // rows north to south: red green / blue yellow
        return map;
    };
    {
        RenderScene plain;
        plain.materials = {Material{}};
        plain.bounds = {glm::vec3(-60.0f, -5.0f, -60.0f), glm::vec3(60.0f, 5.0f, 60.0f)};
        ASSERT_TRUE(spade::render::attach_terrain(plain, make()).has_value());
        EXPECT_FALSE(plain.planar_texture.has_value());
        EXPECT_FALSE(plain.statics.back().planar_textured);
    }
    RenderScene scene;
    scene.materials = {Material{}};
    scene.bounds = {glm::vec3(-60.0f, -5.0f, -60.0f), glm::vec3(60.0f, 5.0f, 60.0f)};
    spade::render::TerrainSpec spec;
    spec.textured = true;
    ASSERT_TRUE(spade::render::attach_terrain(scene, make(), spec).has_value());
    ASSERT_TRUE(scene.planar_texture.has_value());
    EXPECT_TRUE(scene.statics.back().planar_textured);
    EXPECT_EQ(scene.planar_texture->width, 2u);
    EXPECT_EQ(scene.planar_texture->rgb.size(), 12u);
    EXPECT_FLOAT_EQ(scene.planar_texture->x_min, -10.0f);
    EXPECT_FLOAT_EQ(scene.planar_texture->x_max, 10.0f);
    EXPECT_FLOAT_EQ(scene.planar_texture->z_min, -10.0f) << "z = -north: the north edge is the image's top row";
    EXPECT_FLOAT_EQ(scene.planar_texture->z_max, 10.0f);
    const std::vector<Rgb> frame = frame_of(scene, camera_over(20.0f));
    ASSERT_EQ(frame.size(), static_cast<size_t>(kW) * kH);
    const auto at = [&](int x, int y) { return frame[static_cast<size_t>(y) * kW + static_cast<size_t>(x)]; };
    EXPECT_TRUE(at(54, 34).r > 120 && at(54, 34).g < 40 && at(54, 34).b < 40) << "north-west: red";
    EXPECT_TRUE(at(106, 34).g > 120 && at(106, 34).r < 40 && at(106, 34).b < 40) << "north-east: green";
    EXPECT_TRUE(at(54, 86).b > 120 && at(54, 86).r < 40 && at(54, 86).g < 40) << "south-west: blue";
    EXPECT_TRUE(at(106, 86).r > 120 && at(106, 86).g > 120 && at(106, 86).b < 40) << "south-east: yellow";
}
