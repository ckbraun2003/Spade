#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>
#include <unordered_set>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// Materials, directional lighting, sky gradient and the analytic ground
// (S7a Task R6, rulings SR-17/SR-18). This file never references
// SPADE_GOLDEN_DIR -- it asserts qualitative/geometric behaviour, not a
// committed sha256 (that is test_render_raster.cpp's RasterGolden.* suite,
// whose three hashes this task also regenerated) -- so it is NOT swept into
// test_m1b_bar.cpp's libm-transcendental scan (that scan protects committed
// golden hashes from cross-platform ULP drift; a plain std::tan/glm::angleAxis
// call here has no such hash to move) and freely uses ordinary trig to build
// test cameras.
//
// Sections:
//   1. Step 1: two materials render two distinct colours; a lambert face
//      toward the sun is brighter than one facing away and neither is pure
//      black; an unlit material ignores sun direction entirely; the sky
//      gradient fills the background (top row differs from the horizon row,
//      no background pixel is the old flat clear colour).
//   2. Added Step 1b: the tessellated-grid/analytic-ground seam is
//      byte-identical at world bounds, a pixel above the horizon is sky, and
//      the horizon is a hard, one-row-wide transition.
// ---------------------------------------------------------------------------

namespace {

using spade::Capacities;
using spade::Result;
using spade::SdfPose;
using spade::WorldBuilder;
using spade::WorldDesc;
using spade::render::Camera;
using spade::render::DrawMode;
using spade::render::GroundPlane;
using spade::render::NamedMesh;
using spade::render::PixelFormat;
using spade::render::render;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::scene_from_world;

// Mirrors test_render_raster.cpp's identical helper: storage is the caller's,
// target is a non-owning view over it (PA-1); pre-filled with a sentinel byte
// so "every byte was actually overwritten" is a real assertion.
[[nodiscard]] RenderTarget make_target(std::vector<uint8_t>& storage, uint32_t width, uint32_t height) {
    storage.assign(static_cast<size_t>(width) * 4 * height, 0xAAu);
    return RenderTarget{
        .pixels = std::span<uint8_t>(storage),
        .width = width,
        .height = height,
        .stride = width * 4,
        .format = PixelFormat::bgrx8,
    };
}

void render_or_fail(const RenderScene& scene, const Camera& camera, const RenderOptions& options,
                     RenderTarget& target) {
    const Result<void> result = render(scene, camera, options, target);
    if (!result) {
        ADD_FAILURE() << "render() failed: " << result.error().context;
    }
}

[[nodiscard]] Camera camera_looking_down_neg_z(glm::vec3 position) {
    Camera camera;
    camera.position = position;
    return camera;
}

struct Bgr {
    uint8_t b, g, r;
};

[[nodiscard]] Bgr pixel_at(const std::vector<uint8_t>& storage, uint32_t width, uint32_t x, uint32_t y) {
    const size_t idx = (static_cast<size_t>(y) * width + x) * 4;
    return Bgr{storage[idx], storage[idx + 1], storage[idx + 2]};
}

[[nodiscard]] bool operator==(const Bgr& a, const Bgr& b) { return a.b == b.b && a.g == b.g && a.r == b.r; }
[[nodiscard]] bool operator!=(const Bgr& a, const Bgr& b) { return !(a == b); }

// Luminance is only ever used to compare two RENDERED pixels for "brighter
// than", never to reconstruct a physical unit -- a plain, unweighted channel
// sum is enough for that ordering.
[[nodiscard]] uint32_t luma(const Bgr& c) {
    return static_cast<uint32_t>(c.b) + static_cast<uint32_t>(c.g) + static_cast<uint32_t>(c.r);
}

// A builder with the world-level requirements already satisfied (mirrors
// test_render_scene.cpp's own base_builder()).
[[nodiscard]] WorldBuilder base_builder() {
    WorldBuilder b;
    b.name("render-shading-test").capacities(Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    return b;
}

[[nodiscard]] WorldDesc build_or_fail(const WorldBuilder& b) {
    const Result<WorldDesc> world = b.build();
    if (!world) {
        ADD_FAILURE() << "builder failed: " << world.error().context;
        return WorldDesc{};
    }
    return *world;
}

[[nodiscard]] RenderScene scene_or_fail(const WorldDesc& world, std::span<const NamedMesh> resolved = {}) {
    Result<RenderScene> scene = scene_from_world(world, resolved);
    if (!scene) {
        ADD_FAILURE() << "scene_from_world failed: " << scene.error().context;
        return RenderScene{};
    }
    return std::move(*scene);
}

}  // namespace

// ===========================================================================
// 1. Step 1.
// ===========================================================================

TEST(RenderShading, TwoDifferentMaterialsRenderTwoDistinctColours) {
    // Two boxes side by side, same shape, same normals, different materials
    // -- isolates "does a material change actually change the rendered
    // colour" from any geometry/lighting-angle effect (both faces share the
    // SAME normal, so if lit, both get the SAME lighting term -- any colour
    // difference is attributable to base_color alone).
    WorldBuilder b = base_builder();
    b.material(spade::MaterialDesc{.name = "left", .base_color = {0.9f, 0.1f, 0.1f, 1.0f}})
        .material(spade::MaterialDesc{.name = "right", .base_color = {0.1f, 0.1f, 0.9f, 1.0f}});
    b.box(glm::vec3(0.8f), SdfPose{.position = {-1.5f, 0.0f, 0.0f}}).material_for_last_node(0);
    b.box(glm::vec3(0.8f), SdfPose{.position = {1.5f, 0.0f, 0.0f}}).material_for_last_node(1);
    b.union_();  // a well-formed SDF program must reduce to exactly one value
    const WorldDesc world = build_or_fail(b);
    const RenderScene scene = scene_or_fail(world);

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 6.0f));
    RenderOptions options;
    options.overlays = false;
    constexpr uint32_t kW = 160, kH = 120;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kW, kH);
    render_or_fail(scene, camera, options, target);

    const Bgr left = pixel_at(storage, kW, kW / 4, kH / 2);
    const Bgr right = pixel_at(storage, kW, (kW * 3) / 4, kH / 2);
    EXPECT_NE(left, right) << "two different materials must render two distinct colours";
}

TEST(RenderShading, LambertFaceTowardSunIsBrighterThanAwayAndNeitherIsPureBlack) {
    // A sphere lit from an angle OFF the camera's own view axis: the visible
    // (camera-facing) hemisphere still spans a range of N.L values, from
    // near-maximal (where the surface normal points close to the sun) down
    // to near-ambient-only (where it points nearly perpendicular to the sun)
    // -- scanning every visible, non-background pixel for min/max brightness
    // proves both halves of Step 1's requirement without having to hand-solve
    // which exact pixel each extreme lands on.
    WorldBuilder b = base_builder();
    b.lighting(spade::LightingDesc{
        .sun_direction = {0.0f, 1.0f, 0.0f},  // straight up -- off the camera's -Z view axis
        .sun_color = {1.0f, 1.0f, 1.0f},
        .sun_intensity = 1.0f,
        .ambient_color = {0.08f, 0.08f, 0.08f},
        .sky_zenith = {0.0f, 0.0f, 0.0f},
        .sky_horizon = {0.0f, 0.0f, 0.0f},
    });
    b.material(spade::MaterialDesc{.name = "sphere", .base_color = {1.0f, 1.0f, 1.0f, 1.0f}});
    b.sphere(1.5f);
    const WorldDesc world = build_or_fail(b);
    const RenderScene scene = scene_or_fail(world);

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
    RenderOptions options;
    options.overlays = false;
    constexpr uint32_t kW = 160, kH = 120;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kW, kH);
    render_or_fail(scene, camera, options, target);

    // Background is pure black here (sky_zenith/horizon both zeroed above),
    // so "not background" is exactly "luma > 0" -- a plain, unambiguous
    // partition that needs no separate reference render.
    uint32_t min_luma = 255u * 3u, max_luma = 0u;
    bool saw_sphere = false;
    std::unordered_set<uint32_t> distinct_colours;
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const Bgr c = pixel_at(storage, kW, x, y);
            const uint32_t l = luma(c);
            if (l == 0u) {
                continue;  // background
            }
            saw_sphere = true;
            min_luma = std::min(min_luma, l);
            max_luma = std::max(max_luma, l);
            distinct_colours.insert((static_cast<uint32_t>(c.b) << 16) | (static_cast<uint32_t>(c.g) << 8) |
                                     static_cast<uint32_t>(c.r));
        }
    }
    ASSERT_TRUE(saw_sphere) << "sanity: the sphere must actually be visible";
    EXPECT_GT(min_luma, 0u) << "even the dimmest lit pixel must not be pure black (Step 1's ambient floor)";
    EXPECT_GT(max_luma, min_luma)
        << "a face toward the sun must be brighter than one away from it -- got a perfectly flat sphere";

    // Review IMPORTANT 1: frame-wide min/max luma alone does not distinguish
    // genuine per-vertex Gouraud shading from a per-TRIANGLE-flat mutant
    // (colour every fan vertex from poly[0].normal alone) -- both still
    // produce a bright pole and a dim rim, just far fewer distinct shades in
    // between. Measured on this exact fixture: the real Gouraud path
    // produces several hundred distinct colours across the sphere's
    // kTessellationDefaults tessellation; a poly[0]-only flat mutant
    // collapses that to roughly one shade per triangle. 40 sits comfortably
    // below the real count and comfortably above what per-triangle-flat
    // shading can produce even generously, so it is a genuine, non-fragile
    // floor rather than a coin flip -- confirmed directly by mutating
    // draw_mesh_triangle_shaded to colour every fan vertex from poly[0]
    // .normal alone and rebuilding: this assertion fails (measured well
    // under 40 distinct colours) while the frame-wide min/max check above
    // stays green, reproducing exactly the blind spot this finding named.
    EXPECT_GT(distinct_colours.size(), 40u)
        << "too few distinct shaded colours (" << distinct_colours.size() << ") across the sphere -- this is what "
           "a per-triangle-flat shading regression looks like, not genuine per-vertex Gouraud";
}

TEST(RenderShading, UnlitMaterialIgnoresSunDirectionEntirely) {
    // Same geometry and material colour, rendered under two DIFFERENT
    // lighting setups (opposite sun directions, different sun colours) --
    // an unlit material must produce byte-identical output in both, because
    // "unlit" means base_color verbatim, no lighting term at all.
    const auto render_unlit_box = [&](glm::vec3 sun_direction, glm::vec3 sun_color) {
        WorldBuilder b = base_builder();
        b.lighting(spade::LightingDesc{.sun_direction = sun_direction, .sun_color = sun_color, .sun_intensity = 3.0f});
        b.material(spade::MaterialDesc{
            .name = "decal", .base_color = {0.6f, 0.3f, 0.8f, 1.0f}, .shading = spade::MaterialShading::unlit});
        b.box(glm::vec3(1.0f));
        const WorldDesc world = build_or_fail(b);
        const RenderScene scene = scene_or_fail(world);

        const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 0.0f, 5.0f));
        RenderOptions options;
        options.overlays = false;
        std::vector<uint8_t> storage;
        RenderTarget target = make_target(storage, 160, 120);
        render_or_fail(scene, camera, options, target);
        return storage;
    };

    const std::vector<uint8_t> lit_from_front = render_unlit_box({0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f});
    const std::vector<uint8_t> lit_from_behind = render_unlit_box({0.0f, 0.0f, -1.0f}, {0.2f, 0.9f, 0.1f});

    EXPECT_EQ(lit_from_front, lit_from_behind)
        << "an unlit material must ignore sun_direction (and sun_color) entirely";
}

TEST(RenderShading, SkyGradientFillsEveryBackgroundPixelTopDiffersFromHorizon) {
    const WorldDesc world = build_or_fail(base_builder());  // no SDF nodes at all -- pure background
    const RenderScene scene = scene_or_fail(world);

    const Camera camera = camera_looking_down_neg_z(glm::vec3(0.0f, 1000.0f, 1000.0f));  // looks at nothing
    RenderOptions options;
    options.overlays = false;
    constexpr uint32_t kW = 64, kH = 64;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kW, kH);
    render_or_fail(scene, camera, options, target);

    const Bgr top = pixel_at(storage, kW, kW / 2, 0);
    const Bgr horizon_row = pixel_at(storage, kW, kW / 2, kH - 1);
    EXPECT_NE(top, horizon_row) << "the sky gradient must actually vary from top to bottom";

    // render/raster_cpu.cpp's OLD flat background clear (removed at this
    // task) was BGR (16, 18, 20) -- literal, not a symbolic constant, since
    // that constant no longer exists in production; this is a regression
    // guard against ever reintroducing a flat background under this name.
    const Bgr old_flat_background{16, 18, 20};
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            ASSERT_NE(pixel_at(storage, kW, x, y), old_flat_background)
                << "no background pixel may be the old flat clear colour (" << x << "," << y << ")";
        }
    }
}

// ===========================================================================
// 2. Added Step 1b: the tessellated-grid/analytic-ground seam.
// ===========================================================================

namespace {

// Independent re-derivation of raster_cpu.cpp's own background ray
// reconstruction (background_ray_camera_space + the camera-to-world
// rotation), in double precision, via plain std::tan/quaternion-vector
// rotation -- this file is not golden-feeding (see the file header comment),
// so ordinary libm is fine here.
[[nodiscard]] glm::dvec3 background_ray_world_oracle(const Camera& camera, uint32_t width, uint32_t height,
                                                      uint32_t px, uint32_t py) {
    const double half_fov = static_cast<double>(camera.fov_y_radians) * 0.5;
    const double f = 1.0 / std::tan(half_fov);
    const double aspect = static_cast<double>(width) / static_cast<double>(height);
    const double x_ndc = 2.0 * (static_cast<double>(px) + 0.5) / static_cast<double>(width) - 1.0;
    const double y_ndc = 1.0 - 2.0 * (static_cast<double>(py) + 0.5) / static_cast<double>(height);
    const glm::dvec3 dir_cam(x_ndc * aspect / f, y_ndc / f, -1.0);
    const glm::dquat q(static_cast<double>(camera.orientation.w), static_cast<double>(camera.orientation.x),
                        static_cast<double>(camera.orientation.y), static_cast<double>(camera.orientation.z));
    return q * dir_cam;
}

// Independent re-derivation of the analytic ground pass's own SR-13-parity
// front-side ray/plane hit test (raster_cpu.cpp's draw_sky_and_ground_background).
[[nodiscard]] std::optional<double> ground_hit_t_oracle(glm::dvec3 origin, glm::dvec3 dir, glm::dvec3 normal,
                                                          double offset) {
    const double n_dot_o = glm::dot(normal, origin);
    if (!(n_dot_o > offset)) {
        return std::nullopt;
    }
    const double denom = glm::dot(normal, dir);
    if (!(denom < 0.0)) {
        return std::nullopt;
    }
    const double t = (offset - n_dot_o) / denom;
    return t > 0.0 ? std::optional<double>(t) : std::nullopt;
}

// Independent re-derivation of the sky gradient's own ELEVATION-based
// formula (draw_sky_and_ground_background; ruling SR-23, Task VQ-A): reuses
// this file's own background_ray_world_oracle() above for the per-pixel
// world-space ray -- the sky colour depends on both x AND y now (a camera's
// horizontal FOV makes even an unrotated ray's elevation vary slightly
// across a row, since sky_gradient_color() divides by the ray's FULL
// length, not merely its world-Y component), so unlike the old plain
// fraction-of-SCREEN-ROW formula this oracle needs the real ray, not just a
// row index.
[[nodiscard]] Bgr expected_sky_bgr_oracle(const RenderScene& scene, const Camera& camera, uint32_t width,
                                           uint32_t height, uint32_t x, uint32_t y) {
    const glm::dvec3 dir = background_ray_world_oracle(camera, width, height, x, y);
    const double len = std::sqrt(glm::dot(dir, dir));
    const double elevation = len > 0.0 ? dir.y / len : 1.0;
    const double horizon_fraction = std::clamp(1.0 - elevation, 0.0, 1.0);
    const float hf = static_cast<float>(horizon_fraction);
    // `zenith + (horizon-zenith)*hf`, not `zenith*(1-hf)+horizon*hf` -- see
    // render/scene.hpp's own sky_gradient_color() comment (Task VQ-A): only
    // this form is exact when sky_zenith == sky_horizon, and matches
    // production's own formula so this independent oracle cannot disagree
    // with it over a rounding half-boundary that the OTHER form could hit.
    const glm::vec3 sky = scene.lighting.sky_zenith + (scene.lighting.sky_horizon - scene.lighting.sky_zenith) * hf;
    const auto to_byte = [](float c) {
        return static_cast<uint8_t>(std::lround(std::clamp(c, 0.0f, 1.0f) * 255.0f));
    };
    return Bgr{to_byte(sky.b), to_byte(sky.g), to_byte(sky.r)};
}

// Independent re-derivation of shade_vertex_color()'s own formula
// (raster_cpu.cpp), for the ONE fixed material/normal a standalone ground

// Independent re-derivation of shade_vertex_color()'s own formula
// (raster_cpu.cpp), for the ONE fixed material/normal a standalone ground
// plane always shades with -- used to assert a scanned row/column is the
// EXACT ground colour (never a blend with sky), review IMPORTANT 2's fix:
// an infinite, single-material, constant-normal plane's colour is the same
// at every hit point, so this is a single, reusable expected value.
[[nodiscard]] Bgr expected_ground_bgr_oracle(const RenderScene& scene, const GroundPlane& ground) {
    const spade::render::Material& material = scene.materials.at(ground.material);
    const glm::vec3 base(material.base_color);
    glm::vec3 color = base;
    if (material.shading != 1u && material.shading != 2u) {  // not unlit/emissive -- lambert
        const float n_dot_l = std::max(glm::dot(ground.normal, scene.lighting.sun_direction), 0.0f);
        color = base * (scene.lighting.sun_color * (scene.lighting.sun_intensity * n_dot_l) + scene.lighting.ambient_color);
    }
    const auto to_byte = [](float c) {
        return static_cast<uint8_t>(std::lround(std::clamp(c, 0.0f, 1.0f) * 255.0f));
    };
    return Bgr{to_byte(color.b), to_byte(color.g), to_byte(color.r)};
}

}  // namespace

TEST(RenderShading, TessellatedAndAnalyticGroundAgreeAcrossTheHardHorizonSeam) {
    // A single ground plane, identity pose, offset 0 -- world_bounds_of()
    // (scene.cpp) sees no spawns and one SDF node (the plane, whose
    // transform ORIGIN is (0,0,0)), so it falls back to the origin-plus-
    // fixed-margin box; tessellate_plane fits its bounded grid to exactly
    // that box. The plane extends past it (SR-17): this world is exactly
    // the "ground plane extends past its bounds" case the brief names.
    // TWO materials, with the ground plane explicitly assigned the SECOND
    // (review MINOR 10): a single-material fixture cannot discriminate "the
    // analytic path resolved the wrong material index" from "it happened to
    // fall back to index 0, which is also the only material" -- index 0
    // stays defined but genuinely unused, so a bug that always resolves to
    // it (instead of honouring node_materials) would show up as the WRONG
    // base_color on the analytic (or tessellated) side, not a coincidental
    // match.
    WorldBuilder b = base_builder();
    b.material(spade::MaterialDesc{.name = "unused_default", .base_color = {0.1f, 0.9f, 0.1f, 1.0f}})
        .material(spade::MaterialDesc{.name = "ground", .base_color = {0.8f, 0.75f, 0.7f, 1.0f}});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).material_for_last_node(1);
    const WorldDesc world = build_or_fail(b);
    const RenderScene scene = scene_or_fail(world);
    ASSERT_EQ(scene.ground_planes.size(), 1u);
    const GroundPlane& ground = scene.ground_planes[0];
    EXPECT_NEAR(ground.normal.x, 0.0f, 1e-4f);
    EXPECT_NEAR(ground.normal.y, 1.0f, 1e-4f);
    EXPECT_NEAR(ground.normal.z, 0.0f, 1e-4f);
    EXPECT_NEAR(ground.offset, 0.0f, 1e-4f);
    ASSERT_EQ(ground.material, 1u) << "sanity: the ground plane must resolve to the material node_materials names";
    ASSERT_EQ(scene.statics.size(), 1u);
    EXPECT_EQ(scene.statics[0].material_override, 1u)
        << "the tessellated plane's own DrawItem must resolve to the SAME material index as the analytic entry";

    // Independent re-derivation of tessellate_plane's own (u,v) basis and
    // half-extent computation (tessellate.cpp), from the REAL scene.bounds
    // -- not a hardcoded number, so this stays correct even if world_bounds_
    // of()'s own margin/default constants ever change. n.y == 1 takes
    // tessellate_plane's "up = (1,0,0)" branch, giving u=(0,0,1), v=(1,0,0):
    // the tessellated quad spans WORLD X in [-half_v, half_v] and WORLD Z in
    // [-half_u, half_u].
    const glm::vec3 n(0.0f, 1.0f, 0.0f);
    const glm::vec3 up(1.0f, 0.0f, 0.0f);
    const glm::vec3 u = glm::normalize(glm::cross(up, n));
    const glm::vec3 v = glm::cross(n, u);
    const glm::vec3 center = (scene.bounds.min + scene.bounds.max) * 0.5f;
    float half_u = 0.0f, half_v = 0.0f;
    for (uint32_t corner = 0; corner < 8; ++corner) {
        const glm::vec3 c((corner & 1u) ? scene.bounds.max.x : scene.bounds.min.x,
                           (corner & 2u) ? scene.bounds.max.y : scene.bounds.min.y,
                           (corner & 4u) ? scene.bounds.max.z : scene.bounds.min.z);
        const glm::vec3 delta = c - center;  // center already lies ON the y=0 plane -- origin == center
        half_u = std::max(half_u, std::fabs(glm::dot(delta, u)));
        half_v = std::max(half_v, std::fabs(glm::dot(delta, v)));
    }
    ASSERT_GT(half_v, 0.5f) << "sanity: the tessellated grid must have a real, non-degenerate extent";

    constexpr uint32_t kWidth = 400, kHeight = 300;
    Camera camera;
    camera.position = glm::vec3(0.0f, 4.0f, 6.0f);
    // A moderate downward pitch about the camera's local X (ZERO yaw, ZERO
    // roll): with no yaw/roll, the ray's world Y-component -- and therefore
    // the ground/sky classification AND the ground hit point's world Z --
    // depend on SCREEN ROW alone, never on column (a pure-pitch rotation
    // only mixes the camera's local Y/Z axes, leaving X untouched). That is
    // what makes "sweep across one row, cross the tessellated grid's X-bound"
    // and "a pixel above the horizon is sky" both hold simultaneously for a
    // single, simple camera pose here.
    camera.orientation = glm::angleAxis(glm::radians(-18.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kWidth, kHeight);
    render_or_fail(scene, camera, options, target);

    // ---- Ground-row search: a row whose (row-constant) world Z lands
    // safely inside [-half_u, half_u], with two columns on that row -- one
    // safely inside [-half_v, half_v] (tessellated path), one safely outside
    // it (analytic-only path) -- both still hitting the plane.
    uint32_t seam_row = 0, col_inside = 0, col_outside = 0;
    bool found_row = false;
    for (uint32_t y = 0; y < kHeight && !found_row; ++y) {
        const glm::dvec3 origin(camera.position);
        const glm::dvec3 mid_dir = background_ray_world_oracle(camera, kWidth, kHeight, kWidth / 2, y);
        const std::optional<double> mid_t = ground_hit_t_oracle(origin, mid_dir, glm::dvec3(0, 1, 0), 0.0);
        if (!mid_t) {
            continue;  // this row is sky, not ground
        }
        const double z = origin.z + *mid_t * mid_dir.z;
        if (std::fabs(z) > static_cast<double>(half_u) - 0.4) {
            continue;  // too close to the Z-bound (or outside it) -- want comfortable margin
        }
        std::optional<uint32_t> in, out;
        for (uint32_t x = 0; x < kWidth; ++x) {
            const glm::dvec3 dir = background_ray_world_oracle(camera, kWidth, kHeight, x, y);
            const std::optional<double> t = ground_hit_t_oracle(origin, dir, glm::dvec3(0, 1, 0), 0.0);
            if (!t) {
                continue;
            }
            const double world_x = origin.x + *t * dir.x;
            if (!in && std::fabs(world_x) < static_cast<double>(half_v) - 0.4) {
                in = x;
            } else if (!out && std::fabs(world_x) > static_cast<double>(half_v) + 0.4) {
                out = x;
            }
        }
        if (in && out) {
            seam_row = y;
            col_inside = *in;
            col_outside = *out;
            found_row = true;
        }
    }
    ASSERT_TRUE(found_row) << "sanity: this camera/world must yield a row with both an inside-bounds and an "
                               "outside-bounds ground pixel -- adjust the fixture if this ever fires";

    // The load-bearing assertion: byte-identical in all four BGRX bytes.
    const size_t idx_in = (static_cast<size_t>(seam_row) * kWidth + col_inside) * 4;
    const size_t idx_out = (static_cast<size_t>(seam_row) * kWidth + col_outside) * 4;
    EXPECT_EQ(storage[idx_in], storage[idx_out]) << "B channel differs across the seam";
    EXPECT_EQ(storage[idx_in + 1], storage[idx_out + 1]) << "G channel differs across the seam";
    EXPECT_EQ(storage[idx_in + 2], storage[idx_out + 2]) << "R channel differs across the seam";
    EXPECT_EQ(storage[idx_in + 3], storage[idx_out + 3]) << "X channel differs across the seam";
    // Sanity: this must actually be the GROUND colour, not a background
    // coincidence -- distinct from row 0's sky colour.
    EXPECT_NE(pixel_at(storage, kWidth, col_inside, seam_row), pixel_at(storage, kWidth, kWidth / 2, 0));

    // A pixel above the horizon is sky: row 0, this camera's own pitch
    // (-18 deg, comfortably inside the default 60 deg vertical FOV's top
    // half) keeps the top of the frame above the horizon.
    const std::optional<double> top_hit =
        ground_hit_t_oracle(glm::dvec3(camera.position),
                             background_ray_world_oracle(camera, kWidth, kHeight, kWidth / 2, 0), glm::dvec3(0, 1, 0),
                             0.0);
    ASSERT_FALSE(top_hit) << "sanity: row 0 must be above the horizon for this camera pose";
    EXPECT_EQ(pixel_at(storage, kWidth, kWidth / 2, 0),
              expected_sky_bgr_oracle(scene, camera, kWidth, kHeight, kWidth / 2, 0));

    // Hard horizon (review IMPORTANT 2's fix -- the original version of this
    // check found `transition_row` as "the first row that differs from the
    // sky formula", then asserted the PREVIOUS row matches the sky formula:
    // that is the loop's own break condition restated at an index it
    // already passed, and cannot fail for ANY transition shape, gradual or
    // hard, real or synthetic -- confirmed by feeding the identical
    // assertion block a synthetic 11-row sky-to-ground blend, which also
    // "passed"). The fix scans the WHOLE column and requires every single
    // row to be EXACTLY one of two values -- the sky oracle's own per-row
    // colour before the transition, or the ground's own single fixed colour
    // at and after it -- with no third, in-between value anywhere. A
    // blended/gradient row of any width, anywhere in the column, fails this
    // (it matches neither exactly); only a genuinely hard, one-row-wide
    // transition passes.
    const Bgr expected_ground = expected_ground_bgr_oracle(scene, ground);
    uint32_t transition_row = kHeight;
    for (uint32_t y = 0; y < kHeight; ++y) {
        const Bgr actual = pixel_at(storage, kWidth, col_inside, y);
        const Bgr expected_sky = expected_sky_bgr_oracle(scene, camera, kWidth, kHeight, col_inside, y);
        if (actual == expected_sky) {
            ASSERT_EQ(transition_row, kHeight)
                << "row " << y << " is sky again after row " << transition_row << " was ground -- not a single hard transition";
            continue;
        }
        ASSERT_EQ(actual, expected_ground)
            << "row " << y << " is neither the exact sky colour nor the exact ground colour -- a blended/"
               "gradient row, which a hard horizon (SR-17, no fog) must never produce";
        if (transition_row == kHeight) {
            transition_row = y;
        }
    }
    ASSERT_LT(transition_row, kHeight) << "sanity: this column must transition from sky to ground somewhere";
    ASSERT_GT(transition_row, 0u) << "sanity: row 0 must still be sky (checked above)";
}

// ===========================================================================
// 3. Fix round 1 (review IMPORTANT 4, MINOR 11): SR-22's wireframe gating,
//    the analytic ground's own from-below cull, and nearest-of-several-planes.
// ===========================================================================

namespace {

[[nodiscard]] WorldDesc single_ground_plane_world() {
    WorldBuilder b = base_builder();
    b.material(spade::MaterialDesc{.name = "ground", .base_color = {0.8f, 0.75f, 0.7f, 1.0f}});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    return build_or_fail(b);
}

}  // namespace

TEST(RenderShading, WireframeModeNeverDrawsTheAnalyticGroundFromAboveOrBelow) {
    // Ruling SR-22 (review IMPORTANT 4): the analytic ground is gated to
    // DrawMode::shaded only. Before this fix, an unconditional analytic
    // ground made wireframe mode show a SOLID, LIT ground fill (up to 70% of
    // the frame, measured by the review) behind its own unlit, edges-only
    // geometry -- and, from below, produced an inverted seam (tessellated
    // wireframe edges draw, unculled, while the analytic fill correctly did
    // not, an inconsistency exactly like the one SR-17 exists to prevent in
    // shaded mode).
    const WorldDesc world = single_ground_plane_world();
    const RenderScene scene = scene_or_fail(world);
    ASSERT_EQ(scene.ground_planes.size(), 1u);
    // The colour the analytic ground WOULD shade with in DrawMode::shaded --
    // must never appear anywhere in a wireframe-rendered frame. Wireframe's
    // own tessellated-mesh edges use the material's RAW, unlit base_color
    // bytes (never this lit value, raster_cpu.cpp's own draw_mesh_item), so
    // this is a clean, unambiguous discriminator between "no analytic fill
    // at all" and "analytic fill leaked through".
    const Bgr lit_ground = expected_ground_bgr_oracle(scene, scene.ground_planes[0]);

    RenderOptions options;
    options.mode = DrawMode::wireframe;
    options.overlays = false;

    const auto assert_no_lit_ground_fill_anywhere = [&](const Camera& camera, uint32_t width, uint32_t height) {
        std::vector<uint8_t> storage;
        RenderTarget target = make_target(storage, width, height);
        render_or_fail(scene, camera, options, target);
        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                ASSERT_NE(pixel_at(storage, width, x, y), lit_ground)
                    << "the analytically-shaded ground colour must never appear in a wireframe frame (" << x << ","
                    << y << ")";
            }
        }
    };

    // From above: the SAME camera pose the seam test uses -- exactly the
    // pose/world combination that showed a solid analytic fill beyond the
    // tessellated grid's own bounds before this fix.
    {
        Camera camera;
        camera.position = glm::vec3(0.0f, 4.0f, 6.0f);
        camera.orientation = glm::angleAxis(glm::radians(-18.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        assert_no_lit_ground_fill_anywhere(camera, 200, 150);
    }
    // From below, looking up -- the case the review named directly.
    {
        Camera camera;
        camera.position = glm::vec3(0.0f, -5.0f, 0.0f);
        camera.orientation = glm::angleAxis(glm::radians(30.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        assert_no_lit_ground_fill_anywhere(camera, 200, 150);
    }
}

TEST(RenderShading, AnalyticGroundNeverDrawsWhenCameraIsAtOrBelowThePlane) {
    // Review MINOR 11(a): the analytic ground's own from-below cull (SR-13
    // parity) had no committed test -- the review verified it only by
    // probe. Every background pixel must be EXACTLY the sky gradient's own
    // per-row colour, matching the tessellated mesh's back-face cull from
    // below.
    const WorldDesc world = single_ground_plane_world();
    const RenderScene scene = scene_or_fail(world);
    ASSERT_EQ(scene.ground_planes.size(), 1u);

    Camera camera;
    camera.position = glm::vec3(0.0f, -5.0f, 0.0f);  // below the plane (offset 0)
    camera.orientation = glm::angleAxis(glm::radians(30.0f), glm::vec3(1.0f, 0.0f, 0.0f));  // pitched to look up
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    constexpr uint32_t kW = 200, kH = 150;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kW, kH);
    render_or_fail(scene, camera, options, target);

    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const Bgr expected = expected_sky_bgr_oracle(scene, camera, kW, kH, x, y);
            ASSERT_EQ(pixel_at(storage, kW, x, y), expected)
                << "(" << x << "," << y << ") should be pure sky -- the camera is below the plane";
        }
    }
}

TEST(RenderShading, AnalyticGroundPicksTheNearestOfSeveralPlanes) {
    // Review MINOR 11(b): "nearest hit among several planes" had no
    // committed test either. Two standalone ground planes, authored
    // FAR-then-NEAR (offset -5 first, offset 0 second) so a "first
    // successful hit wins" traversal-order bug would pick the WRONG (far,
    // red) material instead of the correct (near, blue) one that "smallest
    // t wins" must produce.
    WorldBuilder b = base_builder();
    b.material(spade::MaterialDesc{.name = "default"})
        .material(spade::MaterialDesc{.name = "far", .base_color = {0.9f, 0.1f, 0.1f, 1.0f}})
        .material(spade::MaterialDesc{.name = "near", .base_color = {0.1f, 0.1f, 0.9f, 1.0f}});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), -5.0f).material_for_last_node(1);
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).material_for_last_node(2);
    b.union_();
    const WorldDesc world = build_or_fail(b);
    const RenderScene scene = scene_or_fail(world);
    ASSERT_EQ(scene.ground_planes.size(), 2u);

    const GroundPlane* near_plane = nullptr;
    for (const GroundPlane& gp : scene.ground_planes) {
        if (gp.material == 2u) {
            near_plane = &gp;
        }
    }
    ASSERT_NE(near_plane, nullptr) << "sanity: the near plane (material 2) must exist";

    Camera camera;
    camera.position = glm::vec3(0.0f, 10.0f, 0.0f);
    camera.orientation = glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));  // straight down
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    constexpr uint32_t kW = 100, kH = 100;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kW, kH);
    render_or_fail(scene, camera, options, target);

    const Bgr expected_near = expected_ground_bgr_oracle(scene, *near_plane);
    EXPECT_EQ(pixel_at(storage, kW, kW / 2, kH / 2), expected_near)
        << "the NEARER plane (offset 0, material 2) must win, not the farther one (offset -5, material 1) that "
           "was authored first";
}

// ===========================================================================
// 4. Task VQ-A (rulings SR-33/SR-35): the fix round for the user's
//    verdict "the rendering is pretty horrible". Steps 1-3 of
//    task-VQ-A-brief.md -- each a regression test the OLD (pre-fix)
//    defaults/formula would genuinely have failed; each is verified below
//    to actually discriminate, not merely to pass.
// ===========================================================================

TEST(RenderShading, DefaultMaterialUnderWorstCaseLightingLeavesHeadroomNoChannelReachesByte255) {
    // Step 1: N.L = 1 (a surface normal aligned EXACTLY with the sun) is the
    // worst case any lambert material can face -- shade_vertex_color()'s own
    // formula there is base * (ambient_color + sun_color*sun_intensity).
    // Under the OLD default (pure white base_color, ambient 0.1, sun_color
    // (1,1,1), sun_intensity 1): 1.0 * (0.1 + 1.0) = 1.1, clipping to byte
    // 255 on every channel -- exactly the "blown out" verdict. This
    // constructs that worst case directly (a normal set to the scene's OWN
    // normalized sun_direction, so N.L is exactly 1 by construction, not
    // approximated by geometry) and asserts headroom remains under the
    // CURRENT default.
    const WorldDesc world = build_or_fail(base_builder());  // no .material()/.lighting() calls -- pure defaults
    const RenderScene scene = scene_or_fail(world);
    ASSERT_FALSE(scene.materials.empty());
    ASSERT_GT(glm::length(scene.lighting.sun_direction), 0.0f)
        << "sanity: scene_from_world must have a real, non-zero sun direction to normalize";

    const glm::vec3 worst_case_normal = glm::normalize(scene.lighting.sun_direction);  // N.L == 1 exactly
    const spade::render::ShadedColor shaded =
        spade::render::shade_vertex_color(scene.materials[0], scene.lighting, worst_case_normal);

    EXPECT_LT(spade::render::to_byte(shaded.combined.r), 255u)
        << "red channel clipped at the worst-case N.L=1 -- no exposure headroom left";
    EXPECT_LT(spade::render::to_byte(shaded.combined.g), 255u)
        << "green channel clipped at the worst-case N.L=1 -- no exposure headroom left";
    EXPECT_LT(spade::render::to_byte(shaded.combined.b), 255u)
        << "blue channel clipped at the worst-case N.L=1 -- no exposure headroom left";
}

TEST(RenderShading, TwoVerticalFacesAtDifferentAnglesToTheDefaultSunShadeDifferently) {
    // Step 2: under the OLD straight-overhead default sun_direction (0,1,0),
    // EVERY vertical face has N.L == 0 exactly -- all of them collapse to
    // the identical bare ambient floor, discriminating nothing about which
    // way a wall faces. The new off-axis default must break that
    // degeneracy: two vertical faces (normals +X and +Z) at DIFFERENT
    // angles to the sun must shade to genuinely different values, and each
    // must receive a real, non-zero sun contribution of its own.
    const WorldDesc world = build_or_fail(base_builder());
    const RenderScene scene = scene_or_fail(world);
    ASSERT_FALSE(scene.materials.empty());

    const glm::vec3 face_plus_x(1.0f, 0.0f, 0.0f);
    const glm::vec3 face_plus_z(0.0f, 0.0f, 1.0f);
    const spade::render::ShadedColor shaded_x =
        spade::render::shade_vertex_color(scene.materials[0], scene.lighting, face_plus_x);
    const spade::render::ShadedColor shaded_z =
        spade::render::shade_vertex_color(scene.materials[0], scene.lighting, face_plus_z);

    EXPECT_GT(shaded_x.sun.r + shaded_x.sun.g + shaded_x.sun.b, 1e-4f)
        << "the +X wall face must receive a real, non-zero sun contribution under the off-axis default";
    EXPECT_GT(shaded_z.sun.r + shaded_z.sun.g + shaded_z.sun.b, 1e-4f)
        << "the +Z wall face must receive a real, non-zero sun contribution under the off-axis default";

    const Bgr bgr_x{spade::render::to_byte(shaded_x.combined.b), spade::render::to_byte(shaded_x.combined.g),
                     spade::render::to_byte(shaded_x.combined.r)};
    const Bgr bgr_z{spade::render::to_byte(shaded_z.combined.b), spade::render::to_byte(shaded_z.combined.g),
                     spade::render::to_byte(shaded_z.combined.r)};
    EXPECT_NE(bgr_x, bgr_z) << "two vertical faces at different angles to the sun must shade to genuinely "
                                "different values -- under the OLD straight-overhead default both collapse to "
                                "the identical ambient floor";
}

TEST(RenderShading, SkyColourAtTheTrueHorizonMatchesAcrossDifferentCameraPitchesRulingSR23) {
    // Step 3 / ruling SR-23: the OLD sky gradient interpolated on a plain
    // fraction of SCREEN ROW (t = row/(height-1)), so the TRUE horizon (the
    // ray whose elevation is exactly zero) got a DIFFERENT colour under
    // different camera pitches, because it lands at a different screen row
    // each time. This picks two pitches whose own true-horizon row is
    // deliberately far apart on screen and asserts the ACTUAL rendered sky
    // colour at each camera's own true-horizon row agrees -- something the
    // row-based formula could not do (checked directly below: applied at
    // these SAME two rows, it disagrees by far more than this test's own
    // tolerance).
    const WorldDesc world = build_or_fail(base_builder());  // no SDF nodes -- pure background
    const RenderScene scene = scene_or_fail(world);

    constexpr uint32_t kW = 128, kH = 128;
    RenderOptions options;
    options.overlays = false;

    // For a PURE-PITCH camera (no yaw, no roll -- angleAxis about local X),
    // the ray's world-space Y component depends only on screen ROW and the
    // pitch angle, never on screen column (an X-axis rotation leaves the
    // camera-space X component untouched) -- so "the row whose world-Y is
    // closest to zero" is a column-independent property of the camera
    // alone. Found here by a plain scan through this file's own
    // background_ray_world_oracle(), never by re-deriving the rotation by
    // hand.
    const auto find_horizon_row = [&](const Camera& camera) {
        uint32_t best_row = 0;
        double best_abs_y = 1e18;
        for (uint32_t y = 0; y < kH; ++y) {
            const glm::dvec3 dir = background_ray_world_oracle(camera, kW, kH, kW / 2, y);
            const double abs_y = std::fabs(dir.y);
            if (abs_y < best_abs_y) {
                best_abs_y = abs_y;
                best_row = y;
            }
        }
        return best_row;
    };

    Camera camera_a;
    camera_a.orientation = glm::angleAxis(glm::radians(-30.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    Camera camera_b;
    camera_b.orientation = glm::angleAxis(glm::radians(20.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    const uint32_t row_a = find_horizon_row(camera_a);
    const uint32_t row_b = find_horizon_row(camera_b);
    ASSERT_GE(row_a > row_b ? row_a - row_b : row_b - row_a, 20u)
        << "sanity: these two pitches must put the true horizon at genuinely different screen rows, or this "
           "test cannot discriminate row-based from elevation-based interpolation -- row_a=" << row_a
        << " row_b=" << row_b;

    std::vector<uint8_t> storage_a, storage_b;
    RenderTarget target_a = make_target(storage_a, kW, kH);
    RenderTarget target_b = make_target(storage_b, kW, kH);
    render_or_fail(scene, camera_a, options, target_a);
    render_or_fail(scene, camera_b, options, target_b);

    const Bgr horizon_a = pixel_at(storage_a, kW, kW / 2, row_a);
    const Bgr horizon_b = pixel_at(storage_b, kW, kW / 2, row_b);

    // Tight but not bit-exact: each row is the CLOSEST integer pixel to the
    // true (continuous) zero-elevation ray, not that ray exactly, so a
    // residual sub-pixel elevation offset remains -- a few LSBs of headroom
    // absorbs that without weakening what this test actually proves (the
    // row-based formula's OWN error here is not a couple of LSBs; it is
    // checked separately below and is most of the zenith-to-horizon range).
    EXPECT_LE(std::abs(int(horizon_a.b) - int(horizon_b.b)), 3)
        << "B channel: true-horizon colour must agree across pitches";
    EXPECT_LE(std::abs(int(horizon_a.g) - int(horizon_b.g)), 3)
        << "G channel: true-horizon colour must agree across pitches";
    EXPECT_LE(std::abs(int(horizon_a.r) - int(horizon_b.r)), 3)
        << "R channel: true-horizon colour must agree across pitches";

    // Proof this test actually discriminates: the OLD row-based formula
    // (t = row/(height-1)) applied at these SAME two rows would have
    // reported two fractions far apart, hence two very different colours --
    // recomputed here independently, not asserted from prose.
    const auto old_row_fraction_bgr = [&](uint32_t row) {
        const float t = static_cast<float>(row) / static_cast<float>(kH - 1);
        const glm::vec3 c = scene.lighting.sky_zenith * (1.0f - t) + scene.lighting.sky_horizon * t;
        return Bgr{spade::render::to_byte(c.b), spade::render::to_byte(c.g), spade::render::to_byte(c.r)};
    };
    const Bgr old_a = old_row_fraction_bgr(row_a);
    const Bgr old_b = old_row_fraction_bgr(row_b);
    const int old_channel_spread =
        std::max({std::abs(int(old_a.b) - int(old_b.b)), std::abs(int(old_a.g) - int(old_b.g)),
                  std::abs(int(old_a.r) - int(old_b.r))});
    EXPECT_GT(old_channel_spread, 20)
        << "sanity: the OLD row-based formula must disagree substantially between these two rows, or this test "
           "does not actually discriminate row-based from elevation-based interpolation";
}

// ===========================================================================
// 4. SR-17a -- the atmospheric term (03-world-and-render.md section 16,
//    amended to ALL GEOMETRY AT RANGE by user ruling 2026-09-17).
//
// THE FIXTURE IS A SINGLE GROUND PLANE, AND IT IS CHOSEN BECAUSE IT EXERCISES
// ALL THREE CALL SITES IN ONE FRAME. scene_from_world() turns one plane node
// into BOTH a tessellated quad in `statics` (drawn by draw_mesh_item's Gouraud
// fill, bounded to scene.bounds) AND a GroundPlane in `ground_planes` (drawn
// analytically by the background pass, everywhere the quad is not). The SDF
// program carries the same plane, so DrawMode::raymarch hits it as geometry.
// One world, three code paths, no hand-built scene.
//
// ⚠ THESE TESTS DO NOT RE-ASSERT THE HARD HORIZON. That assertion lives in
// TessellatedAndAnalyticGroundAgreeAcrossTheHardHorizonSeam above and is
// DELIBERATELY LEFT WHOLE: it runs at the engine default strength 0, where
// horizon_blend() returns the surface colour exactly, so it still states
// SR-17 clause 5 as the degenerate case of SR-17a rather than being deleted by
// the supersession. Section 16.4 records why that matters -- that test's body
// also carries clause 3/clause 4's seam assertion, which the term does not
// touch, and rewriting it wholesale would have destroyed that as collateral.
// ===========================================================================

namespace {

// A small onset (metres) relative to this camera's own view distances: the
// term must be MEASURABLE on near geometry for the mesh-fill arm below to
// discriminate at all. With the shipped default of 45 m every pixel of this
// fixture sits in the fade's near-linear toe and the arm would read as "the
// term does nothing" when the truth is "this camera is too close to it".
constexpr float kProbeOnset = 8.0f;
constexpr float kProbeStrength = 0.8f;

// ⚠⚠ `shadows = false` IS A SELECTOR, NOT A SIMPLIFICATION, and leaving it at
// its default silently disarmed the mesh arm below. RenderOptions::shadows
// defaults to TRUE (target.hpp:63) and scene_from_world() builds a static
// shadow map for any scene with statics (scene.cpp:434) -- so `shadow` is
// non-null, and rasterizeTriangleGouraud's equal-colour fast path, whose
// condition REQUIRES `shadow == nullptr`, is never reached. MEASURED: with
// shadows on, deleting the fast path's `!atmo.active()` guard entirely changed
// nothing and the arm stayed green.
//
// ⭐ A TEST CANNOT EXERCISE A BRANCH WHOSE PRECONDITION ITS OWN DEFAULTS
// NEGATE, and nothing says so -- the frame renders, the pixels are right, and
// the arm passes for a reason that has nothing to do with what it is named
// for. Turning shadows off is what puts the guarded branch back in the path.
[[nodiscard]] RenderOptions ground_probe_options(float strength, float onset) {
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;  // the overlay grid is a separate mechanism (SR-22); keep it out of the measurement
    options.shadows = false;   // see above: this is what makes the Gouraud fast path reachable at all
    options.horizon_blend_strength = strength;
    options.horizon_blend_onset = onset;
    return options;
}

[[nodiscard]] size_t count_bytes_differing(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    EXPECT_EQ(a.size(), b.size());
    size_t n = 0;
    for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
        if (a[i] != b[i]) ++n;
    }
    return n;
}

// A downward-pitched camera whose frame spans a LARGE range of ground
// distances: the bottom rows land a few metres away (inside scene.bounds, so
// the tessellated quad draws them) and the upper rows recede to the horizon
// (outside the quad, so the analytic ground draws them). That spread is what
// makes a distance-keyed term's monotonicity observable at all.
[[nodiscard]] Camera ground_probe_camera() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 4.0f, 6.0f);
    camera.orientation = glm::angleAxis(glm::radians(-18.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    return camera;
}

}  // namespace

TEST(RenderShading, AtmosphericTermAtStrengthZeroIsBitIdenticalAndOnsetAloneDoesNothing) {
    // SR-17a's degenerate case, asserted as a REQUIREMENT rather than assumed
    // from the formula. horizon_blend() is written `base + (other - base) * k`
    // precisely so k == 0 returns `base` bit-for-bit; the algebraically-equal
    // `base * (1 - k) + other * k` would NOT, and this is the assertion that
    // would catch someone "simplifying" it.
    //
    // It also pins the second half of the contract: with strength 0, the ONSET
    // is inert. A non-zero onset that perturbed a single byte would mean the
    // term had a path that ignores its own off switch.
    // RenderScene::sdf is NON-OWNING, so the WorldDesc must outlive the scene --
    // binding it to a named local rather than passing a temporary.
    const WorldDesc world = single_ground_plane_world();
    const RenderScene scene = scene_or_fail(world);
    const Camera camera = ground_probe_camera();
    constexpr uint32_t kW = 240, kH = 180;

    std::vector<uint8_t> defaulted;
    RenderTarget t0 = make_target(defaulted, kW, kH);
    render_or_fail(scene, camera, ground_probe_options(0.0f, 45.0f), t0);

    std::vector<uint8_t> explicit_zero;
    RenderTarget t1 = make_target(explicit_zero, kW, kH);
    render_or_fail(scene, camera, ground_probe_options(0.0f, kProbeOnset), t1);

    EXPECT_EQ(count_bytes_differing(defaulted, explicit_zero), 0u)
        << "SR-17a at strength 0 must be EXACTLY the pre-term frame, and the onset must be inert there";
}

TEST(RenderShading, AtmosphericTermActuallyChangesPixelsOnTheMeshFillAndTheAnalyticGround) {
    // THE POSITIVE CONTROL, and section 16.4 requires it by name: a term that
    // is identically zero is monotone AND bounded and would pass every other
    // assertion in this section. Without this arm those assertions cannot tell
    // a working term from a dead one.
    //
    // ⚠⚠ THE MESH HALF IS THE ONE THAT CAN SILENTLY DIE. raster_cpu.cpp's
    // Gouraud fill has an equal-colour fast path that paints ONE colour across
    // a whole triangle, valid only while shading has no position dependence --
    // exactly the assumption this term breaks. A constant-normal surface (a
    // plane, a box face, a wall) takes that path, so a missing `!atmo.active()`
    // guard would drop the term across most real geometry while the analytic
    // ground kept working and the frame kept looking plausible.
    // RenderScene::sdf is NON-OWNING, so the WorldDesc must outlive the scene --
    // binding it to a named local rather than passing a temporary.
    const WorldDesc world = single_ground_plane_world();
    const RenderScene scene = scene_or_fail(world);
    ASSERT_EQ(scene.ground_planes.size(), 1u) << "fixture: the analytic ground must exist";
    ASSERT_EQ(scene.statics.size(), 1u) << "fixture: the tessellated quad must exist -- this is the mesh-fill arm's subject";
    const Camera camera = ground_probe_camera();
    constexpr uint32_t kW = 240, kH = 180;

    std::vector<uint8_t> off;
    RenderTarget t_off = make_target(off, kW, kH);
    render_or_fail(scene, camera, ground_probe_options(0.0f, kProbeOnset), t_off);

    std::vector<uint8_t> on;
    RenderTarget t_on = make_target(on, kW, kH);
    render_or_fail(scene, camera, ground_probe_options(kProbeStrength, kProbeOnset), t_on);

    const size_t differing = count_bytes_differing(off, on);
    EXPECT_GT(differing, 100u) << "SR-17a is wired but inert: turning the term on changed " << differing
                                << " bytes. Check AtmosphereContext::active() and the Gouraud fast-path guard.";

    // ⚠⚠ THE MESH ARM ISOLATES ITS SUBJECT BY DELETING THE OTHER ONE, and the
    // first version of this arm did not -- it asserted that the frame's BOTTOM
    // BAND moved, on the reasoning that near ground must be inside
    // scene.bounds and therefore tessellated. IT SURVIVED THE MUTATION THAT
    // DELETES THE FAST-PATH GUARD, measured, which is the only reason this
    // paragraph exists. The quad is bounded in world Z as well as X, so with
    // the camera at z=6 the nearest rows fall OUTSIDE it and are drawn by the
    // analytic background pass -- the band was measuring the very path the arm
    // was supposed to exclude. A geometric assumption about where a fixture
    // puts its pixels is not a selector for which CODE PATH drew them.
    //
    // Clearing `ground_planes` leaves the tessellated quad as the only thing
    // that can draw ground at all, so every ground pixel below is unambiguously
    // draw_mesh_item's. A flat quad has one normal, so `equal_combined` holds
    // and the Gouraud fast path is live -- which is precisely the condition the
    // `!atmo.active()` guard exists for.
    RenderScene mesh_only = scene_or_fail(world);
    ASSERT_EQ(mesh_only.statics.size(), 1u);
    mesh_only.ground_planes.clear();

    std::vector<uint8_t> mesh_off;
    RenderTarget t_mesh_off = make_target(mesh_off, kW, kH);
    render_or_fail(mesh_only, camera, ground_probe_options(0.0f, kProbeOnset), t_mesh_off);

    std::vector<uint8_t> mesh_on;
    RenderTarget t_mesh_on = make_target(mesh_on, kW, kH);
    render_or_fail(mesh_only, camera, ground_probe_options(kProbeStrength, kProbeOnset), t_mesh_on);

    // Sanity floor FIRST: the quad must actually cover pixels, or "the term
    // changed nothing" and "there was nothing to change" are the same result.
    std::vector<uint8_t> sky_only_storage;
    RenderTarget t_sky = make_target(sky_only_storage, kW, kH);
    RenderScene empty_ground = mesh_only;
    empty_ground.statics.clear();
    render_or_fail(empty_ground, camera, ground_probe_options(0.0f, kProbeOnset), t_sky);
    const size_t quad_pixels = count_bytes_differing(mesh_off, sky_only_storage);
    ASSERT_GT(quad_pixels, 400u) << "sanity floor: the tessellated quad covers only " << quad_pixels
                                  << " bytes of this frame -- the mesh arm below would assert nothing";

    const size_t mesh_changes = count_bytes_differing(mesh_off, mesh_on);
    EXPECT_GT(mesh_changes, 100u)
        << "SR-17a NEVER REACHED THE MESH FILL: with the analytic ground removed, turning the term on moved "
        << mesh_changes << " bytes of " << quad_pixels
        << " covered by the tessellated quad. The Gouraud equal-colour fast path paints one colour per "
           "triangle and is only valid while shading has no position dependence -- check the `!atmo.active()` "
           "term in raster_cpu.cpp's fast-path condition.";
}

TEST(RenderShading, AtmosphericTermIsBoundedByItsTwoEndpointColoursAndNeverOutOfGamut) {
    // The property SR-17 clause 5's clamp existed to protect, carried forward
    // rather than discarded with the clause. Every blended pixel must lie
    // between the colour it started at and the sky it is heading toward -- an
    // out-of-gamut extrapolation is exactly what the old hard clamp prevented,
    // and the supersession must not reintroduce it.
    // RenderScene::sdf is NON-OWNING, so the WorldDesc must outlive the scene --
    // binding it to a named local rather than passing a temporary.
    const WorldDesc world = single_ground_plane_world();
    const RenderScene scene = scene_or_fail(world);
    const GroundPlane& ground = scene.ground_planes[0];
    const Camera camera = ground_probe_camera();
    constexpr uint32_t kW = 240, kH = 180;

    std::vector<uint8_t> off;
    RenderTarget t_off = make_target(off, kW, kH);
    render_or_fail(scene, camera, ground_probe_options(0.0f, kProbeOnset), t_off);

    std::vector<uint8_t> on;
    RenderTarget t_on = make_target(on, kW, kH);
    render_or_fail(scene, camera, ground_probe_options(kProbeStrength, kProbeOnset), t_on);

    const Bgr unblended = expected_ground_bgr_oracle(scene, ground);
    size_t checked = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const Bgr before = pixel_at(off, kW, x, y);
            if (before != unblended) {
                continue;  // sky, or a pixel the analytic ground did not own -- not this arm's subject
            }
            const Bgr after = pixel_at(on, kW, x, y);
            const Bgr sky = expected_sky_bgr_oracle(scene, camera, kW, kH, x, y);
            ++checked;
            // Per channel, `after` must lie in the closed interval spanned by
            // `before` and `sky` -- 1 byte of slack for to_byte()'s own
            // round-to-nearest at an interval endpoint, never more.
            const auto within = [](int a, int p, int q) {
                const int lo = p < q ? p : q;
                const int hi = p < q ? q : p;
                return a >= lo - 1 && a <= hi + 1;
            };
            ASSERT_TRUE(within(after.b, before.b, sky.b))
                << "out of gamut at (" << x << "," << y << ") B: " << int(after.b) << " not between " << int(before.b)
                << " and " << int(sky.b);
            ASSERT_TRUE(within(after.g, before.g, sky.g))
                << "out of gamut at (" << x << "," << y << ") G: " << int(after.g) << " not between " << int(before.g)
                << " and " << int(sky.g);
            ASSERT_TRUE(within(after.r, before.r, sky.r))
                << "out of gamut at (" << x << "," << y << ") R: " << int(after.r) << " not between " << int(before.r)
                << " and " << int(sky.r);
        }
    }
    EXPECT_GT(checked, 100u) << "sanity floor: this fixture must present real ground pixels, or the loop above "
                                 "asserted nothing -- " << checked << " checked";
}

TEST(RenderShading, AtmosphericTermIsMonotoneInDistanceDownAGroundColumn) {
    // Section 16.4's monotonicity requirement. Scanned down ONE column of the
    // centre, from the horizon toward the camera: the ground recedes upward in
    // this frame, so as `y` increases the ground gets NEARER, the blend weight
    // must fall, and each pixel must sit no further from the unblended ground
    // colour than the pixel above it.
    //
    // ⚠ MEASURED AGAINST AN INDEPENDENT PER-PIXEL QUANTITY, NEVER AGAINST
    // ANOTHER PIXEL'S BYTES. Section 16.4 records why: the seam test above
    // compares two DIFFERENT pixels and its byte-equality was only ever valid
    // while shading had no position dependence. Re-using that shape here with
    // a looser tolerance would rebuild the same defect. This walks one column
    // and compares each pixel to a FIXED reference colour, so what it measures
    // is the weight's behaviour, not two pixels' agreement.
    // RenderScene::sdf is NON-OWNING, so the WorldDesc must outlive the scene --
    // binding it to a named local rather than passing a temporary.
    const WorldDesc world = single_ground_plane_world();
    const RenderScene scene = scene_or_fail(world);
    const GroundPlane& ground = scene.ground_planes[0];
    const Camera camera = ground_probe_camera();
    constexpr uint32_t kW = 240, kH = 180;
    const uint32_t col = kW / 2;

    std::vector<uint8_t> off;
    RenderTarget t_off = make_target(off, kW, kH);
    render_or_fail(scene, camera, ground_probe_options(0.0f, kProbeOnset), t_off);

    std::vector<uint8_t> on;
    RenderTarget t_on = make_target(on, kW, kH);
    render_or_fail(scene, camera, ground_probe_options(kProbeStrength, kProbeOnset), t_on);

    const Bgr unblended = expected_ground_bgr_oracle(scene, ground);
    const auto distance_from_ground = [&](const Bgr& c) {
        return std::abs(int(c.b) - int(unblended.b)) + std::abs(int(c.g) - int(unblended.g)) +
               std::abs(int(c.r) - int(unblended.r));
    };

    int previous = -1;
    int farthest = -1, nearest = -1;
    size_t samples = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        if (pixel_at(off, kW, col, y) != unblended) {
            continue;  // above the horizon, or not an analytic-ground pixel
        }
        const int d = distance_from_ground(pixel_at(on, kW, col, y));
        if (farthest < 0) farthest = d;  // first ground row in the column == the most distant
        nearest = d;                      // last one wins == the closest to the camera
        if (previous >= 0) {
            // Non-increasing, with 3 bytes of slack (one per channel) for
            // to_byte()'s round-to-nearest -- NOT a tolerance on the trend,
            // which must still fall.
            ASSERT_LE(d, previous + 3)
                << "row " << y << " is FURTHER from the ground colour (" << d << ") than the row above it ("
                << previous << ") -- the blend is not monotone in distance";
        }
        previous = d;
        ++samples;
    }
    ASSERT_GT(samples, 20u) << "sanity floor: only " << samples
                             << " ground rows sampled in this column -- the scan asserted almost nothing";
    // ⚠⚠ THE TREND MUST ACTUALLY FALL. Every ASSERT_LE above is satisfied by a
    // CONSTANT term -- non-increasing is not decreasing -- so without this line
    // the whole scan is compatible with a blend that ignores distance entirely.
    // (The first draft of this test ended `EXPECT_GT(..., -1)`, which cannot
    // fail for any input at all: a conclusion that cannot fail to print is a
    // decoration, and it had been written into the arm whose entire job is
    // being a positive control.)
    ASSERT_GE(farthest, 0);
    ASSERT_GE(nearest, 0);
    EXPECT_GT(farthest, nearest + 2)
        << "the most distant ground row is " << farthest << " from the unblended colour and the nearest is "
        << nearest << " -- the term is not varying with distance";
}

// ===========================================================================
// 5. Normals under a non-conformal transform (rendering 07-status debt).
//    transform_normal() takes the inverse-transpose, so a per-axis scale or
//    a shear leaves a normal perpendicular to its surface; a conformal
//    transform keeps the old mat3() result bit for bit, which is what keeps
//    every frame golden where it was.
// ===========================================================================

namespace {

// The outward normal of triangle (a, b, c) after `m` moves it: the cross
// product of the moved edges, flipped when `m` mirrors (a mirror reverses
// the winding, not the outward side).
[[nodiscard]] glm::vec3 moved_face_normal(const glm::mat4& m, glm::vec3 a, glm::vec3 b, glm::vec3 c) {
    const glm::vec3 ma(m * glm::vec4(a, 1.0f));
    const glm::vec3 mb(m * glm::vec4(b, 1.0f));
    const glm::vec3 mc(m * glm::vec4(c, 1.0f));
    const float det = glm::determinant(glm::mat3(m));
    return glm::normalize(glm::cross(mb - ma, mc - ma)) * (det < 0.0f ? -1.0f : 1.0f);
}

}  // namespace

TEST(RenderShading, TransformedNormalStaysPerpendicularToItsSurfaceUnderNonConformalTransforms) {
    // A tilted triangle in the plane x + z = 0, wound CCW about (1, 0, 1).
    const glm::vec3 a(-1.0f, -1.0f, 1.0f), b(1.0f, -1.0f, -1.0f), c(0.0f, 1.0f, 0.0f);
    const glm::vec3 local_normal = glm::normalize(glm::cross(b - a, c - a));

    glm::mat4 shear(1.0f);
    shear[1][0] = 0.6f;  // x += 0.6 y
    const glm::mat4 rotate_then_scale =
        glm::mat4_cast(glm::quat(0x1.ee8dd4p-1f, 0.0f, 0x1.0907dcp-2f, 0.0f)) *
        glm::mat4(glm::vec4(2.0f, 0.0f, 0.0f, 0.0f), glm::vec4(0.0f, 0.5f, 0.0f, 0.0f),
                  glm::vec4(0.0f, 0.0f, 1.0f, 0.0f), glm::vec4(0.3f, 0.2f, 0.1f, 1.0f));
    const struct {
        const char* name;
        glm::mat4 m;
    } cases[] = {
        {"per-axis scale (2, 1, 1)", glm::mat4(glm::vec4(2.0f, 0.0f, 0.0f, 0.0f), glm::vec4(0.0f, 1.0f, 0.0f, 0.0f),
                                               glm::vec4(0.0f, 0.0f, 1.0f, 0.0f), glm::vec4(0.0f, 0.0f, 0.0f, 1.0f))},
        {"rotation times per-axis scale, translated", rotate_then_scale},
        {"shear", shear},
        {"mirrored per-axis scale (-2, 1, 1)",
         glm::mat4(glm::vec4(-2.0f, 0.0f, 0.0f, 0.0f), glm::vec4(0.0f, 1.0f, 0.0f, 0.0f),
                   glm::vec4(0.0f, 0.0f, 1.0f, 0.0f), glm::vec4(0.0f, 0.0f, 0.0f, 1.0f))},
    };
    for (const auto& tc : cases) {
        const glm::vec3 expected = moved_face_normal(tc.m, a, b, c);
        const glm::vec3 got = spade::render::transform_normal(tc.m, local_normal);
        EXPECT_GT(glm::dot(got, expected), 0.99999f)
            << tc.name << ": transform_normal gave (" << got.x << ", " << got.y << ", " << got.z
            << ") but the moved surface faces (" << expected.x << ", " << expected.y << ", " << expected.z << ")";
    }
}

TEST(RenderShading, TransformedNormalUnderAConformalTransformIsBitIdenticalToTheRotatedNormal) {
    // The goldens' own poses: identity, the cylinder frame's quaternion spin,
    // that spin with a uniform scale, and a mirror. For each, the answer must
    // be exactly normalize(mat3(m) * n), the pre-inverse-transpose formula.
    const glm::quat spin(0x1.ee8dd4p-1f, 0.0f, 0x1.0907dcp-2f, 0.0f);
    glm::mat4 spun = glm::mat4_cast(spin);
    spun[3] = glm::vec4(1.5f, 0.3f, 0.5f, 1.0f);
    glm::mat4 spun_scaled = glm::mat4_cast(spin) * glm::mat4(glm::mat3(1.7f));
    spun_scaled[3] = glm::vec4(-0.4f, 0.0f, 2.0f, 1.0f);
    const glm::mat4 mirror(glm::vec4(-1.0f, 0.0f, 0.0f, 0.0f), glm::vec4(0.0f, 1.0f, 0.0f, 0.0f),
                           glm::vec4(0.0f, 0.0f, 1.0f, 0.0f), glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
    const glm::mat4 poses[] = {glm::mat4(1.0f), spun, spun_scaled, mirror};
    const glm::vec3 normals[] = {glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f),
                                 glm::normalize(glm::vec3(0.3f, -0.8f, 0.52f))};
    for (size_t p = 0; p < std::size(poses); ++p) {
        for (const glm::vec3& n : normals) {
            const glm::vec3 old_formula = glm::normalize(glm::mat3(poses[p]) * n);
            const glm::vec3 got = spade::render::transform_normal(poses[p], n);
            EXPECT_EQ(got.x, old_formula.x) << "pose " << p;
            EXPECT_EQ(got.y, old_formula.y) << "pose " << p;
            EXPECT_EQ(got.z, old_formula.z) << "pose " << p;
        }
    }
}
