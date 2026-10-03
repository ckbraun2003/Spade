// Field layers on the CPU raster (render/field_layer.hpp): binning, the
// palette, and the drawn layer
// (docs/design/rendering/plans/2026-10-03-field-channel-plan.md, FC-1..FC-6).
//
// A layer is data, not appearance. So the drawing cases light the scene, turn
// the atmospheric term up and still expect each layer pixel to be exactly its
// bin's palette colour.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "render/field_layer.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace {

using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::FieldColourMap;
using spade::render::FieldLayer;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::PixelFormat;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

constexpr uint32_t kW = 160;
constexpr uint32_t kH = 120;

[[nodiscard]] std::vector<uint8_t> render_bgrx(const RenderScene& scene, const Camera& camera,
                                               const RenderOptions& options) {
    std::vector<uint8_t> storage(static_cast<size_t>(kW) * kH * 4u, 0xAAu);
    RenderTarget target{.pixels = std::span<uint8_t>(storage), .width = kW, .height = kH, .stride = kW * 4u,
                        .format = PixelFormat::bgrx8};
    if (auto drew = spade::render::render(scene, camera, options, target); !drew) {
        ADD_FAILURE() << "render() failed: " << drew.error().context;
        return {};
    }
    return storage;
}

struct Bgr {
    uint8_t b, g, r;
    friend bool operator==(const Bgr&, const Bgr&) = default;
};

[[nodiscard]] Bgr pixel_at(const std::vector<uint8_t>& px, uint32_t x, uint32_t y) {
    const size_t at = (static_cast<size_t>(y) * kW + x) * 4u;
    return Bgr{px[at], px[at + 1u], px[at + 2u]};
}

[[nodiscard]] Bgr to_bgr(const glm::vec3& c) {
    return Bgr{spade::render::to_byte(c.b), spade::render::to_byte(c.g), spade::render::to_byte(c.r)};
}

// A 2 m square layer at z = 0, facing +Z: two cells side by side, holding
// 0 and 10 over a fixed 0..10 range in 4 bins, so bin 0 on the left and bin 3
// on the right.
[[nodiscard]] FieldLayer two_cell_layer() {
    FieldLayer layer;
    layer.width = 2.0f;
    layer.height = 2.0f;
    layer.cells_u = 2;
    layer.cells_v = 1;
    layer.values = {0.0f, 10.0f};
    layer.colour_map.bins = 4;
    layer.colour_map.range_max = 10.0f;
    return layer;
}

// Every appearance term on: a lit sun, shadows asked for, and a strong
// atmospheric term. None of them may reach a layer pixel.
[[nodiscard]] RenderOptions appearance_on() {
    RenderOptions options;
    options.overlays = false;
    options.horizon_blend_strength = 1.0f;
    options.horizon_blend_onset = 0.5f;
    return options;
}

[[nodiscard]] RenderScene lit_scene() {
    RenderScene scene;
    scene.materials = {Material{}};
    scene.lighting.sun_direction = glm::normalize(glm::vec3(0.4f, 0.8f, 0.6f));
    return scene;
}

[[nodiscard]] Camera camera_at(glm::vec3 position, glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
    Camera camera;
    camera.position = position;
    camera.orientation = orientation;
    return camera;
}

// A square of `half` metres at depth z, facing +Z, in one unlit colour.
[[nodiscard]] MeshData square_at(float z, float half) {
    MeshData mesh;
    mesh.positions = {glm::vec3(-half, -half, z), glm::vec3(half, -half, z), glm::vec3(half, half, z),
                      glm::vec3(-half, half, z)};
    mesh.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 1, 2, 0, 2, 3};
    return mesh;
}

}  // namespace

// FC-5: binning is total and never divides by zero.
TEST(RenderField, BinningIsTotal) {
    FieldColourMap map;
    map.bins = 4;
    constexpr float kInf = std::numeric_limits<float>::infinity();
    EXPECT_EQ(spade::render::field_bin(map, std::nanf(""), 10.0f), 0u);
    EXPECT_EQ(spade::render::field_bin(map, kInf, 10.0f), 0u);
    EXPECT_EQ(spade::render::field_bin(map, -kInf, 10.0f), 0u);
    EXPECT_EQ(spade::render::field_bin(map, 5.0f, 0.0f), 0u) << "an empty range";
    EXPECT_EQ(spade::render::field_bin(map, 5.0f, -1.0f), 0u) << "an inverted range";
    EXPECT_EQ(spade::render::field_bin(map, 5.0f, kInf), 0u) << "a non-finite range";
    EXPECT_EQ(spade::render::field_bin(map, -3.0f, 10.0f), 0u) << "below the bottom";
    EXPECT_EQ(spade::render::field_bin(map, 2.5f, 10.0f), 1u);
    EXPECT_EQ(spade::render::field_bin(map, 7.4f, 10.0f), 2u);
    EXPECT_EQ(spade::render::field_bin(map, 9.99f, 10.0f), 3u);
    EXPECT_EQ(spade::render::field_bin(map, 25.0f, 10.0f), 3u) << "above the top saturates";

    map.range_min = 10.0f;
    EXPECT_EQ(spade::render::field_bin(map, 15.0f, 20.0f), 2u) << "the range starts at range_min";
}

TEST(RenderField, AutoRangeIsTheLargestFiniteValue) {
    FieldLayer layer;
    layer.cells_u = 3;
    layer.values = {1.0f, std::nanf(""), 7.0f};
    EXPECT_EQ(spade::render::resolved_range_max(layer), 7.0f);
    layer.colour_map.range_max = 12.0f;
    EXPECT_EQ(spade::render::resolved_range_max(layer), 12.0f) << "a fixed range wins";
}

TEST(RenderField, ViridisHitsItsEndStops) {
    const spade::render::FieldPalette viridis = spade::render::viridis_palette();
    EXPECT_EQ(spade::render::palette_colour(viridis, 0.0f), glm::vec3(68.0f, 1.0f, 84.0f) / 255.0f);
    EXPECT_EQ(spade::render::palette_colour(viridis, 1.0f), glm::vec3(253.0f, 231.0f, 37.0f) / 255.0f);
    EXPECT_EQ(spade::render::palette_colour(viridis, -2.0f), spade::render::palette_colour(viridis, 0.0f));
}

// FC-2 and the SR-17a exemption: under a lit sun and a strong atmospheric
// term, every layer pixel is exactly a bin colour, and each half shows its own.
TEST(RenderField, LayerPixelsAreExactlyTheirBinColours) {
    RenderScene scene = lit_scene();
    const RenderScene no_layer = scene;
    scene.field_layers = {two_cell_layer()};
    const Camera camera = camera_at(glm::vec3(0.0f, 0.0f, 3.0f));
    const std::vector<uint8_t> frame = render_bgrx(scene, camera, appearance_on());
    const std::vector<uint8_t> background = render_bgrx(no_layer, camera, appearance_on());
    ASSERT_EQ(frame.size(), background.size());
    ASSERT_FALSE(frame.empty());

    const FieldColourMap& map = scene.field_layers[0].colour_map;
    const Bgr low = to_bgr(spade::render::field_bin_colour(map, 0u));
    const Bgr high = to_bgr(spade::render::field_bin_colour(map, 3u));
    EXPECT_EQ(pixel_at(frame, 60, 60), low) << "the left cell, value 0";
    EXPECT_EQ(pixel_at(frame, 100, 60), high) << "the right cell, value 10";

    size_t layer_pixels = 0, other = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            if (pixel_at(frame, x, y) == pixel_at(background, x, y)) continue;
            ++layer_pixels;
            if (pixel_at(frame, x, y) != low && pixel_at(frame, x, y) != high) ++other;
        }
    }
    EXPECT_GT(layer_pixels, 1000u) << "the layer must cover its part of the frame";
    EXPECT_EQ(other, 0u) << "layer pixels that are not exactly a bin colour";
}

// FC-3: depth-tested against meshes, and visible from behind.
TEST(RenderField, LayerDepthTestsAgainstMeshesAndDrawsFromBothSides) {
    RenderScene scene = lit_scene();
    scene.materials = {Material{.base_color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), .shading = 1u}};
    scene.meshes = {square_at(1.0f, 0.3f), square_at(-1.0f, 3.0f)};  // one in front, one behind
    scene.statics = {DrawItem{.mesh_index = 0}, DrawItem{.mesh_index = 1}};
    scene.field_layers = {two_cell_layer()};
    RenderOptions options = appearance_on();
    options.horizon_blend_strength = 0.0f;  // keep the green exact

    const Bgr green{0u, 255u, 0u};
    const std::vector<uint8_t> front = render_bgrx(scene, camera_at(glm::vec3(0.0f, 0.0f, 3.0f)), options);
    ASSERT_FALSE(front.empty());
    EXPECT_EQ(pixel_at(front, kW / 2u, kH / 2u), green) << "the mesh in front hides the layer";
    const Bgr low = to_bgr(spade::render::field_bin_colour(scene.field_layers[0].colour_map, 0u));
    EXPECT_EQ(pixel_at(front, 60, 60), low) << "the layer hides the mesh behind it";

    // From -Z looking +Z, with no meshes: only the layer's back shows, and
    // the camera's right is world -X.
    const glm::quat turned = glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    RenderScene behind = scene;
    behind.statics = {};
    const std::vector<uint8_t> back = render_bgrx(behind, camera_at(glm::vec3(0.0f, 0.0f, -3.0f), turned), options);
    ASSERT_FALSE(back.empty());
    EXPECT_EQ(pixel_at(back, 100, 60), low) << "from behind, the low cell is on the right";
}

// FC-1 and L6: a malformed layer is refused before anything draws.
TEST(RenderField, AMalformedLayerIsRefused) {
    RenderScene scene = lit_scene();
    FieldLayer layer = two_cell_layer();
    layer.values.pop_back();
    scene.field_layers = {layer};
    std::vector<uint8_t> storage(static_cast<size_t>(kW) * kH * 4u, 0xAAu);
    RenderTarget target{.pixels = std::span<uint8_t>(storage), .width = kW, .height = kH, .stride = kW * 4u,
                        .format = PixelFormat::bgrx8};
    const spade::Result<void> drew = spade::render::render(scene, camera_at(glm::vec3(0.0f, 0.0f, 3.0f)),
                                                           appearance_on(), target);
    ASSERT_FALSE(drew.has_value());
    EXPECT_EQ(drew.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(storage[0], 0xAAu) << "nothing drew";
}
