#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/shadow.hpp"
#include "render/target.hpp"

// ---------------------------------------------------------------------------
// build_static_shadow_map()/rasterize_shadow_casters()/sample_shadow() --
// S7a Task R7, the single-sun shadow map. Not golden-feeding (no
// SPADE_GOLDEN_DIR reference anywhere below), so this file is free to build
// camera orientations with glm::angleAxis -- its assertions are all
// qualitative (lit/shadowed classification, region comparisons), never a
// committed byte-exact digest, so the libm-portability hazard that rule
// exists for (raster_cpu.cpp's own tan32 comment) does not apply here.
//
// Sections:
//   1. Fixtures/helpers.
//   2. build_static_shadow_map/sample_shadow direct (non-render()) checks:
//      size, determinism, the degenerate-light-basis branch, SR-17's
//      outside-the-footprint rule.
//   3. Step 1: a box above a ground plane darkens the pixels beneath it,
//      relative to pixels far away, only when RenderOptions::shadows is set;
//      determinism of the shadowed frame itself.
//   4. Step 3: the static/dynamic split -- a DYNAMIC caster's shadow shows up
//      in render()'s output but never mutates the cached
//      RenderScene::static_shadow it was copied from.
//   5. SR-24: per-pixel, perspective-correct sampling -- a large mesh
//      receiver (the "ground grid" case the ruling itself names) must
//      recover the SAME shadow boundary the exact, non-interpolated analytic
//      ground pass does, not a screen-space-affine approximation of it.
// ---------------------------------------------------------------------------

namespace {

using spade::Result;
using spade::render::Aabb;
using spade::render::build_static_shadow_map;
using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::DrawMode;
using spade::render::GroundPlane;
using spade::render::kNoMaterial;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::PixelFormat;
using spade::render::render;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::sample_shadow;
using spade::render::ShadowMap;

// ===========================================================================
// 1. Fixtures/helpers.
// ===========================================================================

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

// A hand-built, CCW-outward-wound axis-aligned box -- the SAME face/corner
// convention as render/tessellate.cpp's tessellate_box() and
// test_render_raster.cpp's own make_box_mesh() (duplicated here rather than
// shared, matching this test corpus's own per-file-fixture convention).
[[nodiscard]] MeshData make_box_mesh(float half_extent, glm::vec3 center = glm::vec3(0.0f)) {
    MeshData mesh;
    const float h = half_extent;
    struct Face {
        glm::vec3 normal;
        glm::vec3 corners[4];
    };
    const Face faces[6] = {
        {{1.0f, 0.0f, 0.0f}, {{h, -h, -h}, {h, h, -h}, {h, h, h}, {h, -h, h}}},
        {{-1.0f, 0.0f, 0.0f}, {{-h, -h, h}, {-h, h, h}, {-h, h, -h}, {-h, -h, -h}}},
        {{0.0f, 1.0f, 0.0f}, {{-h, h, h}, {h, h, h}, {h, h, -h}, {-h, h, -h}}},
        {{0.0f, -1.0f, 0.0f}, {{-h, -h, -h}, {h, -h, -h}, {h, -h, h}, {-h, -h, h}}},
        {{0.0f, 0.0f, 1.0f}, {{h, -h, h}, {h, h, h}, {-h, h, h}, {-h, -h, h}}},
        {{0.0f, 0.0f, -1.0f}, {{-h, -h, -h}, {-h, h, -h}, {h, h, -h}, {h, -h, -h}}},
    };
    for (const Face& f : faces) {
        const uint32_t base = static_cast<uint32_t>(mesh.positions.size());
        for (const glm::vec3& c : f.corners) {
            mesh.positions.push_back(c + center);
            mesh.normals.push_back(f.normal);
        }
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    return mesh;
}

// A single, huge, flat (+Y normal) quad in the XZ plane at y=0, two
// triangles -- deliberately GIGANTIC (x spans +-200, z spans +50..-999,
// i.e. the near edge sits WELL IN FRONT of any camera used below, not just
// in front of z=0) so it covers the full visible frame and this test never
// has to reason about "did the mesh's own finite edge show through as sky".
// Same CCW-outward corner ordering as make_box_mesh's own +Y face.
[[nodiscard]] MeshData make_huge_ground_quad() {
    MeshData mesh;
    const glm::vec3 corners[4] = {
        {-200.0f, 0.0f, 50.0f},
        {200.0f, 0.0f, 50.0f},
        {200.0f, 0.0f, -999.0f},
        {-200.0f, 0.0f, -999.0f},
    };
    for (const glm::vec3& c : corners) {
        mesh.positions.push_back(c);
        mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
    }
    mesh.indices = {0, 1, 2, 0, 2, 3};
    return mesh;
}

[[nodiscard]] glm::mat4 translation(glm::vec3 t) {
    glm::mat4 m(1.0f);
    m[3] = glm::vec4(t, 1.0f);
    return m;
}

[[nodiscard]] std::array<uint8_t, 3> bgr_at(const std::vector<uint8_t>& storage, uint32_t width, uint32_t x,
                                             uint32_t y) {
    const size_t idx = (static_cast<size_t>(y) * width + x) * 4;
    return {storage[idx], storage[idx + 1], storage[idx + 2]};
}

[[nodiscard]] size_t count_pixels_differing(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    size_t count = 0;
    for (size_t idx = 0; idx + 4 <= a.size(); idx += 4) {
        if (a[idx] != b[idx] || a[idx + 1] != b[idx + 1] || a[idx + 2] != b[idx + 2]) {
            ++count;
        }
    }
    return count;
}

}  // namespace

// ===========================================================================
// 2. build_static_shadow_map()/sample_shadow() direct checks.
// ===========================================================================

TEST(BuildStaticShadowMap, SizeMatchesRequestAndDepthBufferIsSizeSquared) {
    RenderScene scene;
    scene.materials = {Material{}};
    scene.bounds = Aabb{.min = glm::vec3(-2.0f), .max = glm::vec3(2.0f)};
    scene.meshes.push_back(make_box_mesh(1.0f));
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});

    const Result<ShadowMap> map = build_static_shadow_map(scene, /*size=*/256);
    ASSERT_TRUE(map.has_value());
    EXPECT_EQ(map->size, 256u);
    EXPECT_EQ(map->depth.size(), static_cast<size_t>(256) * 256);
}

TEST(BuildStaticShadowMap, IsDeterministicAcrossTwoCalls) {
    RenderScene scene;
    scene.materials = {Material{}};
    scene.bounds = Aabb{.min = glm::vec3(-2.0f), .max = glm::vec3(2.0f)};
    scene.meshes.push_back(make_box_mesh(1.0f, glm::vec3(0.3f, 0.1f, -0.2f)));
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});

    const Result<ShadowMap> a = build_static_shadow_map(scene);
    const Result<ShadowMap> b = build_static_shadow_map(scene);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a->depth, b->depth);
    EXPECT_EQ(a->light_view_proj, b->light_view_proj);
}

// LightingDesc's own default sun_direction (world/builder.hpp) is exactly
// (0,1,0) -- straight up -- which is precisely the degenerate case
// shadow.cpp's build_light_basis() guards (cross(worldUp, forward) is
// undefined when forward IS worldUp). This scene never sets `lighting`
// explicitly, so it exercises that guard directly via RenderScene's own
// struct default (scene.hpp: sun_direction{-0.35,-0.86,-0.37}, ALSO close
// enough to vertical -- |dot| ~0.86, actually below the 0.999 guard
// threshold) -- use an explicit (0,1,0) sun to hit the guard for certain.
TEST(BuildStaticShadowMap, StraightUpSunDoesNotProduceAGarbageOrNanMap) {
    RenderScene scene;
    scene.materials = {Material{}};
    scene.lighting.sun_direction = glm::vec3(0.0f, 1.0f, 0.0f);
    scene.bounds = Aabb{.min = glm::vec3(-2.0f), .max = glm::vec3(2.0f)};
    scene.meshes.push_back(make_box_mesh(1.0f));
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});

    const Result<ShadowMap> map = build_static_shadow_map(scene, /*size=*/64);
    ASSERT_TRUE(map.has_value());
    bool any_occluder = false;
    for (float d : map->depth) {
        ASSERT_FALSE(std::isnan(d)) << "a degenerate light basis must never manufacture a NaN depth value";
        if (d != spade::render::kNoOccluder) {
            any_occluder = true;
        }
    }
    EXPECT_TRUE(any_occluder) << "the box is inside scene.bounds and should have rasterised at least one occluder texel";
}

TEST(SampleShadow, EmptyStaticsMapIsUnconditionallyLitEverywhereInsideBounds) {
    RenderScene scene;  // statics left empty -- nothing to cast a shadow at all
    scene.materials = {Material{}};
    scene.bounds = Aabb{.min = glm::vec3(-2.0f), .max = glm::vec3(2.0f)};

    const Result<ShadowMap> map = build_static_shadow_map(scene);
    ASSERT_TRUE(map.has_value());
    EXPECT_FLOAT_EQ(sample_shadow(*map, glm::vec3(0.0f, 0.0f, 0.0f)), 1.0f);
    EXPECT_FLOAT_EQ(sample_shadow(*map, glm::vec3(1.5f, 1.0f, -1.0f)), 1.0f);
}

// Ruling SR-17: the shadow map is fitted to scene.bounds and a point well
// outside that fitted footprint must read as unconditionally lit -- this
// program does not extend the shadow frustum to cover an infinite plane.
TEST(SampleShadow, PointOutsideTheOrthoFootprintIsAlwaysLit) {
    RenderScene scene;
    scene.materials = {Material{}};
    scene.lighting.sun_direction = glm::vec3(0.0f, 1.0f, 0.0f);
    scene.bounds = Aabb{.min = glm::vec3(-2.0f), .max = glm::vec3(2.0f)};
    // A box big enough that, if the frustum somehow extended past bounds,
    // this occluder would legitimately shadow the far-away point checked
    // below -- so a false "lit" here can only come from the footprint
    // clamp, not from the caster being too small to reach that far.
    scene.meshes.push_back(make_box_mesh(1.5f));
    scene.statics.push_back(DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});

    const Result<ShadowMap> map = build_static_shadow_map(scene);
    ASSERT_TRUE(map.has_value());
    EXPECT_FLOAT_EQ(sample_shadow(*map, glm::vec3(500.0f, 0.0f, 500.0f)), 1.0f);
}

// ===========================================================================
// 3. Step 1 (task-R7-brief.md): a box floating above a ground plane darkens
// the ground pixels directly beneath it relative to pixels far away, with
// shadows=true; equal with shadows=false. Determinism: same scene =>
// byte-identical shadowed frames.
//
// sun_direction = (0,1,0) (straight up) makes the shadow fall EXACTLY in the
// box's own XZ footprint -- geometrically the box would hide that exact
// ground patch from any camera at or above the box's own base height, so
// the camera here sits BELOW the box's underside (box spans y in [2,4];
// camera at y=0.5) and looks toward it: the line of sight to the ground
// point directly beneath the box never rises above y=0.5 on the way there,
// so it is never occluded by the box itself -- like looking under a
// floating table.
// ===========================================================================

namespace {

[[nodiscard]] RenderScene make_step1_scene() {
    RenderScene scene;
    scene.materials = {Material{.base_color = glm::vec4(0.5f, 0.5f, 0.55f, 1.0f), .shading = 1u}};  // unlit
    scene.lighting.sun_direction = glm::vec3(0.0f, 1.0f, 0.0f);
    scene.bounds = Aabb{.min = glm::vec3(-4.0f, -1.0f, -4.0f), .max = glm::vec3(4.0f, 5.0f, 4.0f)};
    scene.ground_planes.push_back(GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = 0.0f, .material = 0});
    scene.meshes.push_back(make_box_mesh(1.0f));
    scene.statics.push_back(
        DrawItem{.mesh_index = 0, .local_to_world = translation(glm::vec3(0.0f, 3.0f, 0.0f)), .material_override = kNoMaterial});
    const Result<ShadowMap> map = build_static_shadow_map(scene);
    if (!map) {
        ADD_FAILURE() << "build_static_shadow_map failed: " << map.error().context;
        return scene;
    }
    scene.static_shadow = *map;
    return scene;
}

[[nodiscard]] Camera step1_camera() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.5f, 10.0f);  // below the box's y=2 underside; identity orientation (-Z forward)
    return camera;
}

constexpr uint32_t kStep1Width = 320, kStep1Height = 240;

}  // namespace

TEST(ShadowStep1, ShadowsOnDarkensGroundBeneathCasterRelativeToShadowsOff) {
    const RenderScene scene = make_step1_scene();
    ASSERT_TRUE(scene.static_shadow.has_value());
    const Camera camera = step1_camera();

    RenderOptions with_shadows;
    with_shadows.mode = DrawMode::shaded;
    with_shadows.overlays = false;
    with_shadows.shadows = true;
    std::vector<uint8_t> storage_on;
    RenderTarget target_on = make_target(storage_on, kStep1Width, kStep1Height);
    render_or_fail(scene, camera, with_shadows, target_on);

    RenderOptions without_shadows = with_shadows;
    without_shadows.shadows = false;
    std::vector<uint8_t> storage_off;
    RenderTarget target_off = make_target(storage_off, kStep1Width, kStep1Height);
    render_or_fail(scene, camera, without_shadows, target_off);

    // With shadows off, sample_shadow() is never even consulted (render()'s
    // own gate) -- the two renders must therefore differ SOMEWHERE (the
    // shadow must actually show up), and every differing pixel must have
    // gone STRICTLY DARKER (never lighter, never a new colour) when shadows
    // are on -- consistent with SR-24's own "multiply into the per-pixel
    // colour" (never additive, never replacing with an unrelated colour).
    size_t darkened = 0;
    for (size_t idx = 0; idx + 4 <= storage_on.size(); idx += 4) {
        const bool differs =
            storage_on[idx] != storage_off[idx] || storage_on[idx + 1] != storage_off[idx + 1] ||
            storage_on[idx + 2] != storage_off[idx + 2];
        if (!differs) {
            continue;
        }
        ++darkened;
        EXPECT_LE(storage_on[idx], storage_off[idx]) << "pixel " << (idx / 4) << " B channel got lighter, not darker";
        EXPECT_LE(storage_on[idx + 1], storage_off[idx + 1])
            << "pixel " << (idx / 4) << " G channel got lighter, not darker";
        EXPECT_LE(storage_on[idx + 2], storage_off[idx + 2])
            << "pixel " << (idx / 4) << " R channel got lighter, not darker";
    }
    EXPECT_GT(darkened, 0u) << "the box's shadow must darken at least one ground pixel beneath it";

    // "far away" sanity floor: a ground region well off to the side, clear
    // of the box's own shadow footprint (box/shadow occupy world x,z in
    // roughly [-1,1]; this samples the far corners of the frame, which at
    // this camera's grazing angle correspond to ground well outside that
    // footprint) must be BYTE-IDENTICAL between the two renders -- shadows
    // do not spread beyond the caster's own footprint.
    for (uint32_t y : {kStep1Height / 2, kStep1Height - 1}) {
        for (uint32_t x : {0u, kStep1Width - 1}) {
            EXPECT_EQ(bgr_at(storage_on, kStep1Width, x, y), bgr_at(storage_off, kStep1Width, x, y))
                << "far-corner pixel (" << x << "," << y << ") should be unaffected by the box's shadow";
        }
    }
}

TEST(ShadowStep1, SameSceneRendersByteIdenticalShadowedFramesTwice) {
    const RenderScene scene = make_step1_scene();
    ASSERT_TRUE(scene.static_shadow.has_value());
    const Camera camera = step1_camera();
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    options.shadows = true;

    std::vector<uint8_t> storage_a, storage_b;
    RenderTarget target_a = make_target(storage_a, kStep1Width, kStep1Height);
    RenderTarget target_b = make_target(storage_b, kStep1Width, kStep1Height);
    render_or_fail(scene, camera, options, target_a);
    render_or_fail(scene, camera, options, target_b);

    EXPECT_EQ(storage_a, storage_b);
}

// ===========================================================================
// 4. Step 3: the static/dynamic split. A DYNAMIC caster's shadow must show
// up in render()'s output, but scene.static_shadow (built from `statics`
// alone, at scene-build time) must never be mutated by that render() call --
// render() folds dynamics into a per-frame COPY, never the cached original.
// ===========================================================================

TEST(ShadowStep3, DynamicCasterShadowsTheGroundButCachedStaticMapNeverGainsAnOccluder) {
    RenderScene scene;
    scene.materials = {Material{.base_color = glm::vec4(0.5f, 0.5f, 0.55f, 1.0f), .shading = 1u}};
    scene.lighting.sun_direction = glm::vec3(0.0f, 1.0f, 0.0f);
    scene.bounds = Aabb{.min = glm::vec3(-4.0f, -1.0f, -4.0f), .max = glm::vec3(4.0f, 5.0f, 4.0f)};
    scene.ground_planes.push_back(GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = 0.0f, .material = 0});
    // NOTE: no static geometry at all -- `statics` stays empty. The caster
    // below is DYNAMIC ONLY.
    scene.meshes.push_back(make_box_mesh(1.0f));
    scene.dynamics.push_back(DrawItem{
        .mesh_index = 0, .local_to_world = translation(glm::vec3(0.0f, 3.0f, 0.0f)), .material_override = kNoMaterial});

    const Result<ShadowMap> map = build_static_shadow_map(scene);
    ASSERT_TRUE(map.has_value());
    scene.static_shadow = *map;

    // The cached STATIC map was built while `statics` was empty -- it must
    // show no occluder at all at the point the dynamic box would otherwise
    // shadow.
    EXPECT_FLOAT_EQ(sample_shadow(*scene.static_shadow, glm::vec3(0.0f, 0.0f, 0.0f)), 1.0f)
        << "a shadow map built from empty `statics` must not already show the dynamic-only caster's shadow";

    const std::vector<float> depth_before_render = scene.static_shadow->depth;

    const Camera camera = step1_camera();
    RenderOptions with_shadows;
    with_shadows.overlays = false;
    with_shadows.shadows = true;
    std::vector<uint8_t> storage_on;
    RenderTarget target_on = make_target(storage_on, kStep1Width, kStep1Height);
    render_or_fail(scene, camera, with_shadows, target_on);

    RenderOptions without_shadows = with_shadows;
    without_shadows.shadows = false;
    std::vector<uint8_t> storage_off;
    RenderTarget target_off = make_target(storage_off, kStep1Width, kStep1Height);
    render_or_fail(scene, camera, without_shadows, target_off);

    EXPECT_GT(count_pixels_differing(storage_on, storage_off), 0u)
        << "render() must fold the DYNAMIC caster into a per-frame shadow map even though `statics` is empty";

    // The cached static map itself must be byte-for-byte unchanged by the
    // render() call above -- Step 3's own "only dynamics re-render per
    // frame, into a COPY" contract.
    EXPECT_EQ(scene.static_shadow->depth, depth_before_render)
        << "render() must never mutate the cached RenderScene::static_shadow it copies from";
}

// ===========================================================================
// 5. SR-24: per-pixel, perspective-correct sampling. Compares two scenes that
// should shade IDENTICALLY -- one where the ground is the EXACT, non-
// interpolated analytic background plane (SR-17's own ray/plane
// intersection, zero interpolation error by construction), the other where
// the SAME ground is a single HUGE tessellated mesh triangle pair (the
// "ground grid" case the ruling names) shaded via draw_mesh_triangle_shaded/
// rasterizeTriangleGouraud's own perspective-correct interpolation. A camera
// with a large depth range across that mesh (near ground close to the
// camera, far ground receding toward the horizon) is exactly the setup
// where a screen-space-AFFINE interpolation of world position (the bug
// SR-24 forbids) would recover a visibly WRONG world position -- and
// therefore misclassify lit/shadowed pixels the exact ground pass gets
// right by construction -- while a correct implementation produces the
// SAME classification either way.
// ===========================================================================

namespace {

// make_box_mesh (section 1, above) only takes a SCALAR half-extent (uniform
// cube) -- the wall below needs an anisotropic box. A tiny overload, local
// to this section, rather than complicating the scalar version every other
// test above uses. Same CCW-outward corner convention throughout.
[[nodiscard]] MeshData make_box_mesh(glm::vec3 half_extent) {
    MeshData mesh;
    const glm::vec3 h = half_extent;
    struct Face {
        glm::vec3 normal;
        glm::vec3 corners[4];
    };
    const Face faces[6] = {
        {{1.0f, 0.0f, 0.0f}, {{h.x, -h.y, -h.z}, {h.x, h.y, -h.z}, {h.x, h.y, h.z}, {h.x, -h.y, h.z}}},
        {{-1.0f, 0.0f, 0.0f}, {{-h.x, -h.y, h.z}, {-h.x, h.y, h.z}, {-h.x, h.y, -h.z}, {-h.x, -h.y, -h.z}}},
        {{0.0f, 1.0f, 0.0f}, {{-h.x, h.y, h.z}, {h.x, h.y, h.z}, {h.x, h.y, -h.z}, {-h.x, h.y, -h.z}}},
        {{0.0f, -1.0f, 0.0f}, {{-h.x, -h.y, -h.z}, {h.x, -h.y, -h.z}, {h.x, -h.y, h.z}, {-h.x, -h.y, h.z}}},
        {{0.0f, 0.0f, 1.0f}, {{h.x, -h.y, h.z}, {h.x, h.y, h.z}, {-h.x, h.y, h.z}, {-h.x, -h.y, h.z}}},
        {{0.0f, 0.0f, -1.0f}, {{-h.x, -h.y, -h.z}, {-h.x, h.y, -h.z}, {h.x, h.y, -h.z}, {h.x, -h.y, -h.z}}},
    };
    for (const Face& f : faces) {
        const uint32_t base = static_cast<uint32_t>(mesh.positions.size());
        for (const glm::vec3& c : f.corners) {
            mesh.positions.push_back(c);
            mesh.normals.push_back(f.normal);
        }
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    return mesh;
}

// A thin, tall wall spanning a wide Z range at x~0. With sun_direction =
// normalize(1,1,0) (light travels in direction -(1,1,0)/sqrt2, i.e. equal
// parts -X and -Y), a point on the wall at height h casts its shadow at
// world x = wall_x - h (independent of z) -- so the wall (height 1, base at
// x=0) casts a shadow occupying roughly x in [-1, 0] at every z along its
// length, a straight-line boundary at x=-1 that is easy to reason about
// independently of any camera/rasterizer arithmetic.
[[nodiscard]] MeshData make_wall_mesh() { return make_box_mesh(glm::vec3(0.05f, 0.5f, 45.0f)); }

[[nodiscard]] DrawItem make_wall_item(uint32_t mesh_index) {
    return DrawItem{.mesh_index = mesh_index,
                     .local_to_world = translation(glm::vec3(0.0f, 0.5f, -20.0f)),
                     .material_override = kNoMaterial};
}

}  // namespace

TEST(ShadowPerspectiveCorrectness, MeshReceiverMatchesExactAnalyticGroundAcrossALargeDepthRange) {
    const Material material{.base_color = glm::vec4(0.6f, 0.55f, 0.5f, 1.0f), .shading = 1u};  // unlit
    const glm::vec3 sun = glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f));
    const Aabb bounds{.min = glm::vec3(-6.0f, -1.0f, -45.0f), .max = glm::vec3(6.0f, 3.0f, 8.0f)};

    // Variant "Ground": the EXACT analytic background plane (zero
    // interpolation error) plus the wall caster.
    RenderScene ground_scene;
    ground_scene.materials = {material};
    ground_scene.lighting.sun_direction = sun;
    ground_scene.bounds = bounds;
    ground_scene.ground_planes.push_back(
        GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = 0.0f, .material = 0});
    ground_scene.meshes.push_back(make_wall_mesh());
    ground_scene.statics.push_back(make_wall_item(0));
    {
        const Result<ShadowMap> map = build_static_shadow_map(ground_scene);
        ASSERT_TRUE(map.has_value());
        ground_scene.static_shadow = *map;
    }

    // Variant "Mesh": the SAME wall, but the ground is now a single HUGE
    // tessellated quad (2 triangles spanning a near-to-far depth range of
    // roughly 1 to 999 world units) shaded via the Gouraud/perspective-
    // correct path -- no ground_planes at all.
    RenderScene mesh_scene;
    mesh_scene.materials = {material};
    mesh_scene.lighting.sun_direction = sun;
    mesh_scene.bounds = bounds;
    mesh_scene.meshes.push_back(make_wall_mesh());
    mesh_scene.meshes.push_back(make_huge_ground_quad());
    mesh_scene.statics.push_back(make_wall_item(0));
    mesh_scene.statics.push_back(DrawItem{.mesh_index = 1, .local_to_world = glm::mat4(1.0f), .material_override = kNoMaterial});
    {
        const Result<ShadowMap> map = build_static_shadow_map(mesh_scene);
        ASSERT_TRUE(map.has_value());
        mesh_scene.static_shadow = *map;
    }

    // A grazing, forward-looking, downward-pitched camera -- near ground
    // (close to the camera) and far ground (receding toward the horizon)
    // appear at very different depths in the SAME frame, exactly the
    // "unequal per-vertex invDepth across one large triangle" condition
    // that makes screen-space-affine interpolation of world position
    // visibly diverge from the perspective-correct answer.
    Camera camera;
    camera.position = glm::vec3(0.0f, 4.0f, 3.0f);
    camera.orientation = glm::angleAxis(glm::radians(-55.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    constexpr uint32_t kWidth = 320, kHeight = 240;
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.overlays = false;
    options.shadows = true;

    std::vector<uint8_t> storage_ground;
    RenderTarget target_ground = make_target(storage_ground, kWidth, kHeight);
    render_or_fail(ground_scene, camera, options, target_ground);

    std::vector<uint8_t> storage_mesh;
    RenderTarget target_mesh = make_target(storage_mesh, kWidth, kHeight);
    render_or_fail(mesh_scene, camera, options, target_mesh);

    // Sanity floor: the shadow mechanism must have actually engaged in
    // BOTH variants (otherwise this test would trivially "pass" by
    // comparing two unshadowed, featureless frames). A shadowed pixel
    // multiplies the unlit base_color by 0.0f, landing at pure black
    // (0,0,0) -- distinct from the lit ground colour and from the sky.
    const auto is_black = [](std::array<uint8_t, 3> c) { return c[0] == 0 && c[1] == 0 && c[2] == 0; };
    size_t black_ground = 0, black_mesh = 0;
    for (uint32_t y = 0; y < kHeight; ++y) {
        for (uint32_t x = 0; x < kWidth; ++x) {
            if (is_black(bgr_at(storage_ground, kWidth, x, y))) ++black_ground;
            if (is_black(bgr_at(storage_mesh, kWidth, x, y))) ++black_mesh;
        }
    }
    EXPECT_GT(black_ground, 0u) << "sanity: the analytic-ground variant must show SOME shadowed pixels";
    EXPECT_GT(black_mesh, 0u) << "sanity: the mesh-receiver variant must show SOME shadowed pixels too";

    // The two variants must agree almost everywhere: same wall, same
    // camera, same lighting/material, same shadow-map footprint (identical
    // bounds/sun_direction => identical light_view_proj) -- the ONLY
    // difference is HOW each recovers the ground's own world position per
    // pixel (exact ray/plane intersection vs. this task's own perspective-
    // correct Gouraud interpolation). A small fringe of mismatches right at
    // the shadow boundary line (sub-pixel differences between two genuinely
    // different formulas) is tolerated; a broad swath of mismatches is
    // exactly the signature a screen-space-affine bug would leave, since
    // its error grows with the interpolated triangle's own depth range,
    // not with distance from the boundary.
    const size_t mismatches = count_pixels_differing(storage_ground, storage_mesh);
    const size_t total = static_cast<size_t>(kWidth) * kHeight;
    EXPECT_LE(mismatches, total / 100)
        << mismatches << " of " << total
        << " pixels disagree between the exact analytic ground and the perspective-correct mesh receiver -- "
           "more than a boundary-line fringe suggests the mesh path is not perspective-correct (SR-24)";
}
