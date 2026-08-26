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
    // S7a Task R7: this world's default lighting (LightingDesc's own
    // sun_direction{0,1,0}, straight up) is EXACTLY the grazing-incidence
    // case that makes a box's own vertical front face collapse onto the
    // same shadow-map footprint column as its own top face (both boxes'
    // visible faces are then self-shadowed to pure black) -- a known, real
    // limitation of a single fixed-bias shadow map at grazing N.L, not a
    // rendering defect this test is about. Disabled here so this test keeps
    // isolating "does a material change actually change the rendered
    // colour" from that unrelated concern, the same way the SR-11 tests
    // (test_render_raster.cpp) use an unlit material to isolate themselves
    // from lighting.
    options.shadows = false;
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
        // S7a Task R7: this test's claim is about shade_vertex_color()'s own
        // material term ("unlit" = base_color verbatim, no N.L/sun_color
        // involved) -- NOT about shadows, which are a SEPARATE, deliberately
        // sun-direction-dependent effect layered on top of whatever the
        // material produced (SR-24: the shadow term multiplies into the
        // per-pixel colour regardless of shading mode). Two opposite
        // sun_direction values legitimately build two DIFFERENT shadow maps
        // -- one of the box's own faces can end up self-shadowed for one
        // direction and not the other (basic shadow mapping's known
        // grazing-incidence limitation, not a defect), which would make
        // this comparison fail for a reason that has nothing to do with
        // what it is actually testing. Disabled here for the same reason
        // TwoDifferentMaterialsRenderTwoDistinctColours (above) disables it.
        options.shadows = false;
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

// Independent re-derivation of the sky gradient's own per-row formula
// (draw_sky_and_ground_background): zenith at row 0, horizon at the last row.
[[nodiscard]] Bgr expected_sky_bgr_oracle(const RenderScene& scene, uint32_t height, uint32_t y) {
    const float t = height > 1 ? static_cast<float>(y) / static_cast<float>(height - 1) : 0.0f;
    const glm::vec3 sky = scene.lighting.sky_zenith * (1.0f - t) + scene.lighting.sky_horizon * t;
    const auto to_byte = [](float c) {
        return static_cast<uint8_t>(std::lround(std::clamp(c, 0.0f, 1.0f) * 255.0f));
    };
    return Bgr{to_byte(sky.b), to_byte(sky.g), to_byte(sky.r)};
}

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
    EXPECT_EQ(pixel_at(storage, kWidth, kWidth / 2, 0), expected_sky_bgr_oracle(scene, kHeight, 0));

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
        const Bgr expected_sky = expected_sky_bgr_oracle(scene, kHeight, y);
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
        const Bgr expected = expected_sky_bgr_oracle(scene, kH, y);
        for (uint32_t x = 0; x < kW; ++x) {
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
