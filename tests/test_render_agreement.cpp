#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "render/agreement.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// The RS4 visual/physics agreement machinery's own tests. render/agreement.hpp
// is the measurement (compare_silhouettes, the Step 1c sky-collision guard,
// strip_to_ground_plane_only); this file proves the metric can tell a wrong
// scene from a right one, and that the Step 1c guard fires when it must.
//
// THE MATRIX THAT USED TO LIVE HERE MOVED OUT (restructure, 2026-10-01). Its
// thirty cases measured KAT's ten worlds at three bookmarks each, read from a
// sibling KAT checkout, and so did the bookmark-drift guard beside them. Spade's
// tests no longer read any other repository. Their bands are to be re-measured
// on worlds this repository owns -- docs/design/backlog.md, "Spade-owned
// content for render agreement bands". The removed cases, their bookmark
// fixture and agreement_bands.json are recoverable from git history for KAT.
//
// THE FLAT-SKY REQUIREMENT (render/agreement.hpp's header): every scene here
// has `lighting.sky_horizon` forced equal to `sky_zenith` before either
// render() call, because compare_silhouettes() decides "is this the sky" by
// one reference colour.
//
// SHADOWS AND OVERLAYS OFF ON THE FAST PATH: raymarch draws neither, so leaving
// them on would only add colour divergence that RS4 already permits.
// ---------------------------------------------------------------------------

namespace {

using spade::Capacities;
using spade::MaterialDesc;
using spade::MaterialShading;
using spade::Result;
using spade::WorldBuilder;
using spade::WorldDesc;
using spade::render::AgreementResult;
using spade::render::assert_no_material_matches_sky;
using spade::render::Camera;
using spade::render::compare_silhouettes;
using spade::render::DrawMode;
using spade::render::Lighting;
using spade::render::NamedMesh;
using spade::render::pack_sky_reference_bgrx;
using spade::render::PixelFormat;
using spade::render::render;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::scene_from_world;
using spade::render::strip_to_ground_plane_only;

// ---------------------------------------------------------------------------
// Small helpers, each mirroring an existing sibling test file's identical
// pattern (test_render_raymarch.cpp's make_target()/render_or_fail())
// rather than sharing a header -- this
// program's own established per-file-duplication convention for helpers this
// small.
// ---------------------------------------------------------------------------

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
    ASSERT_TRUE(result) << "render() failed: " << result.error().context;
}

[[nodiscard]] WorldBuilder base_builder() {
    WorldBuilder b;
    b.name("render-agreement-test")
        .capacities(Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
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

// Forces `scene.lighting`'s sky gradient flat (sky_horizon = sky_zenith) and
// returns the packed reference colour BOTH render() calls' misses will
// produce -- see this file's header comment and render/agreement.hpp's own,
// longer one for why this step is not optional.
[[nodiscard]] uint32_t flatten_sky_and_pack_reference(RenderScene& scene) {
    scene.lighting.sky_horizon = scene.lighting.sky_zenith;
    return pack_sky_reference_bgrx(scene.lighting);
}

// Matches the frame goldens' own resolution.
constexpr uint32_t kFrameWidth = 160, kFrameHeight = 120;

}  // namespace

// ===========================================================================
// 1. Step 1 -- guard the guard (lesson L8): a deliberately wrong scene must
//    report a LARGE disagreement_fraction, and an identical pair must report
//    ~0. Both are pinned as separate tests so a future change that breaks
//    EITHER direction (a metric that always reads near-zero, or one that
//    always reads large) fails loudly on its own.
//
//    THE INJECTED WRONGNESS IS DELIBERATELY LARGE AT THIS RESOLUTION (task
//    brief's own warning: "a 1.5x-scaled box at a camera distance where 1.5x
//    is sub-pixel would repeat the mistake with better numbers"): a 2 m cube
//    (half-extent 1 m) at 6 m from the camera subtends roughly 58% of the
//    frame's height at this file's 60 deg vertical FOV; scaled to half-extent
//    1.5 m it subtends roughly 87% -- a silhouette-diameter change of about
//    30 percentage points of frame height, thousands of pixels at even a
//    small resolution, not a fraction of one. MUTATION-VERIFIED (this task's
//    own report carries the transcript): temporarily setting BOTH scales to
//    1.0 (removing the injected wrongness) makes
//    MismatchedBoxScaleReportsALargeDisagreement's own EXPECT_GT fail, which
//    is what proves this assertion is measuring the injected mismatch and
//    not a fixed baseline the metric always reports regardless of input.
//
//    THIS GUARD EXERCISES THE METRIC AGAINST PURE SKY: a box against sky is
//    exactly the configuration where coverage discriminates. A world with an
//    infinite ground saturates most of the frame, so a band measured on one
//    must also prove its own detection surface (SR-30: re-render the
//    reference through strip_to_ground_plane_only() and check the band would
//    catch that). Whoever re-measures bands on Spade's own worlds owes that
//    probe per case; this guard alone does not provide it.
// ===========================================================================

namespace {

[[nodiscard]] WorldDesc box_world(float half_extent) {
    WorldBuilder b = base_builder();
    b.box(glm::vec3(half_extent));
    return build_or_fail(b);
}

// Camera at (0,0,6) looking down -Z at the origin (Camera's own default
// identity orientation, the SAME convention test_render_raymarch.cpp's
// SphereAnalytic fixture uses) -- no trig needed.
[[nodiscard]] Camera box_guard_camera() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 6.0f);
    return camera;
}

}  // namespace

TEST(AgreementGuard, MismatchedBoxScaleReportsALargeDisagreement) {
    // The "wrong" scene: raster draws a box scaled 1.5x...
    const WorldDesc wrong_world = box_world(1.5f);
    RenderScene wrong_scene = scene_or_fail(wrong_world);
    const uint32_t sky_ref = flatten_sky_and_pack_reference(wrong_scene);
    const Result<void> guard = assert_no_material_matches_sky(wrong_world.materials, wrong_scene.lighting, sky_ref);
    ASSERT_TRUE(guard) << "Step 1c guard: " << guard.error().context;

    // ...while raymarch sphere-traces the box at its TRUE 1.0x scale -- the
    // reference this fast render disagrees with.
    const WorldDesc right_world = box_world(1.0f);
    RenderScene right_scene = scene_or_fail(right_world);
    right_scene.lighting.sky_horizon = right_scene.lighting.sky_zenith;  // same flat sky, independently forced

    const Camera camera = box_guard_camera();

    RenderOptions fast_options;
    fast_options.mode = DrawMode::shaded;
    fast_options.overlays = false;
    fast_options.shadows = false;
    std::vector<uint8_t> fast_storage;
    RenderTarget fast_target = make_target(fast_storage, kFrameWidth, kFrameHeight);
    render_or_fail(wrong_scene, camera, fast_options, fast_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kFrameWidth, kFrameHeight);
    render_or_fail(right_scene, camera, raymarch_options, raymarch_target);

    const AgreementResult result = compare_silhouettes(fast_target, raymarch_target, sky_ref);
    std::cout << "[AgreementGuard.MismatchedBoxScale] disagreement_fraction=" << result.disagreement_fraction
              << " covered_a=" << result.covered_a << " covered_b=" << result.covered_b << "\n";

    ASSERT_GT(result.covered_a, 0u) << "sanity: the fast path must see the (larger) box";
    ASSERT_GT(result.covered_b, 0u) << "sanity: raymarch must see the (smaller, true-scale) box";
    EXPECT_GT(result.disagreement_fraction, 0.15)
        << "a 1.5x box-scale mismatch must register as a LARGE disagreement, not merely a nonzero one -- got "
        << (result.disagreement_fraction * 100.0) << "%";
}

TEST(AgreementGuard, IdenticalScenesReportNearZeroDisagreement) {
    const WorldDesc world = box_world(1.0f);
    RenderScene scene = scene_or_fail(world);
    const uint32_t sky_ref = flatten_sky_and_pack_reference(scene);
    const Result<void> guard = assert_no_material_matches_sky(world.materials, scene.lighting, sky_ref);
    ASSERT_TRUE(guard) << "Step 1c guard: " << guard.error().context;

    const Camera camera = box_guard_camera();

    RenderOptions fast_options;
    fast_options.mode = DrawMode::shaded;
    fast_options.overlays = false;
    fast_options.shadows = false;
    std::vector<uint8_t> fast_storage;
    RenderTarget fast_target = make_target(fast_storage, kFrameWidth, kFrameHeight);
    render_or_fail(scene, camera, fast_options, fast_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kFrameWidth, kFrameHeight);
    render_or_fail(scene, camera, raymarch_options, raymarch_target);

    const AgreementResult result = compare_silhouettes(fast_target, raymarch_target, sky_ref);
    std::cout << "[AgreementGuard.IdenticalScenes] disagreement_fraction=" << result.disagreement_fraction
              << " covered_a=" << result.covered_a << " covered_b=" << result.covered_b << "\n";

    EXPECT_LT(result.disagreement_fraction, 0.02)
        << "the SAME scene rendered by both paths must agree almost everywhere -- got "
        << (result.disagreement_fraction * 100.0) << "%";
}

// ===========================================================================
// 2. Step 1c -- the sky-reference-collision guard itself, in isolation
//    (reproduces the exact SR-27-era scenario: an UNLIT material 0 whose
//    base_color equals the sky reference colour verbatim) and its negative
//    control (an ordinary, non-colliding material palette must pass
//    silently). A THIRD test (fix round 1) proves the specific gap the
//    review found: a LAMBERT material whose authored base_color is nowhere
//    near the sky, but which SHADES into it at an achievable surface
//    orientation.
// ===========================================================================

TEST(AgreementGuard, Step1cFailsLoudlyWhenAMaterialMatchesTheSkyReference) {
    // shading = unlit, explicitly: shade_vertex_color() echoes base_color
    // VERBATIM for unlit (render/scene.hpp), which is what reproduces the
    // historical SR-27 scenario exactly -- LAMBERT (the MaterialDesc
    // default) would shade this same base_color through sun/ambient terms
    // and might not land on the sky reference at all, which is a different
    // (and separately covered, see the THIRD test below) case.
    const std::vector<MaterialDesc> materials = {
        MaterialDesc{.name = "default_matches_sky",
                     .base_color = glm::vec4(0.3f, 0.5f, 0.7f, 1.0f),
                     .shading = MaterialShading::unlit},
    };
    Lighting flat_sky;
    flat_sky.sky_zenith = glm::vec3(0.3f, 0.5f, 0.7f);
    flat_sky.sky_horizon = flat_sky.sky_zenith;
    const uint32_t sky_ref = pack_sky_reference_bgrx(flat_sky);

    const Result<void> result = assert_no_material_matches_sky(materials, flat_sky, sky_ref);
    ASSERT_FALSE(result) << "must fail loudly when a material equals the sky reference colour";
    EXPECT_NE(result.error().context.find("default_matches_sky"), std::string::npos)
        << "failure must name the offending material -- got: " << result.error().context;
    EXPECT_NE(result.error().context.find("index 0"), std::string::npos)
        << "failure must name the offending index -- got: " << result.error().context;
}

TEST(AgreementGuard, Step1cPassesWhenNoMaterialIsNearTheSkyReference) {
    // materials[0] left at the builder's own default (opaque white lambert)
    // against render/scene.hpp's own default Lighting (sun_color/ambient_color
    // both non-grey-preserving, sky_zenith (0.28, 0.42, 0.62) clearly
    // saturated/non-grey) -- a white material can only ever shade to a
    // GREY output (r == g == b) when sun_color/ambient_color are equal per
    // channel; this default Lighting's are NOT, but solving each channel's
    // own required N.L independently still shows they never coincide (a
    // fact the achievable-range sampling below confirms directly rather
    // than asserting from this comment alone).
    const std::vector<MaterialDesc> materials = {MaterialDesc{}};
    const Lighting default_lighting;
    const uint32_t sky_ref = pack_sky_reference_bgrx(default_lighting);

    const Result<void> result = assert_no_material_matches_sky(materials, default_lighting, sky_ref);
    EXPECT_TRUE(result) << "an ordinary, non-colliding material palette must not be flagged: "
                         << (result ? "" : result.error().context);
}

TEST(AgreementGuard, Step1cCatchesAShadedColourCollisionEvenWhenTheAuthoredBaseColourDoesNot) {
    // THE EXACT GAP THE REVIEW FOUND: base_color is opaque WHITE (1,1,1,1) --
    // nowhere near the sky's (0.3, 0.5, 0.7) by any base_color-only check --
    // but sun_color and ambient_color are both (0.15, 0.25, 0.35), so at
    // N.L=1 (a real, achievable surface orientation: any point whose
    // normal aligns with the sun) shade_vertex_color()'s own formula gives
    // combined = 1 * ((0.15,0.25,0.35)*1 + (0.15,0.25,0.35))
    //          = (0.3, 0.5, 0.7) -- EXACTLY the sky reference.
    // A base_color-only check (this function's ORIGINAL, pre-fix-round
    // behaviour) would have passed this material silently.
    const std::vector<MaterialDesc> materials = {
        MaterialDesc{.name = "white_but_shades_into_sky", .base_color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)},
    };
    Lighting lighting;
    lighting.sun_direction = glm::vec3(0.0f, 1.0f, 0.0f);  // any nonzero direction; only magnitudes matter here
    lighting.sun_color = glm::vec3(0.15f, 0.25f, 0.35f);
    lighting.sun_intensity = 1.0f;
    lighting.ambient_color = glm::vec3(0.15f, 0.25f, 0.35f);
    lighting.sky_zenith = glm::vec3(0.3f, 0.5f, 0.7f);
    lighting.sky_horizon = lighting.sky_zenith;
    const uint32_t sky_ref = pack_sky_reference_bgrx(lighting);

    // Sanity: the base_color-only check this function used to run would
    // have missed this -- confirm the raw base_color really is far from the
    // sky reference before proving the NEW check catches it anyway.
    ASSERT_GT(std::abs(static_cast<int>(spade::render::to_byte(materials[0].base_color.r)) -
                        static_cast<int>(spade::render::to_byte(lighting.sky_zenith.r))),
              8)
        << "sanity: base_color.r must clear the OLD base_color-only tolerance for this test to prove anything";

    const Result<void> result = assert_no_material_matches_sky(materials, lighting, sky_ref);
    ASSERT_FALSE(result) << "must catch a material whose SHADED colour (not its authored base_color) lands on the "
                             "sky reference at an achievable surface orientation";
    EXPECT_NE(result.error().context.find("white_but_shades_into_sky"), std::string::npos)
        << "failure must name the offending material -- got: " << result.error().context;
}

// ===========================================================================
// 3. SR-30's probe helper, on its own. strip_to_ground_plane_only() was only
//    ever exercised through the KAT-world matrix; these keep it tested while
//    the bands are rebuilt on Spade's own worlds.
// ===========================================================================

TEST(AgreementProbe, StripKeepsOnlyTheStandaloneGroundPlane) {
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).box(glm::vec3(0.5f)).union_().sphere(0.3f).union_();
    const WorldDesc world = build_or_fail(b);
    ASSERT_GT(world.sdf.nodes.size(), 1u) << "sanity: the source world must hold more than the plane";

    const Result<WorldDesc> bare = strip_to_ground_plane_only(world);
    ASSERT_TRUE(bare) << bare.error().context;
    ASSERT_EQ(bare->sdf.nodes.size(), 1u);
    EXPECT_EQ(bare->sdf.nodes[0].kind, static_cast<uint32_t>(spade::SdfPrim::plane));
    EXPECT_EQ(bare->sdf.nodes[0].op, static_cast<uint32_t>(spade::SdfOp::none));
    EXPECT_EQ(bare->materials.size(), world.materials.size()) << "everything but the nodes is copied verbatim";
}

TEST(AgreementProbe, StripRefusesAWorldWithNoGroundPlane) {
    const Result<WorldDesc> bare = strip_to_ground_plane_only(box_world(1.0f));
    ASSERT_FALSE(bare);
    EXPECT_EQ(bare.error().code, spade::Code::not_found);
}

TEST(AgreementProbe, StripRefusesToGuessBetweenTwoPlaneLeaves) {
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).plane(glm::vec3(1.0f, 0.0f, 0.0f), 4.0f).union_();
    const Result<WorldDesc> bare = strip_to_ground_plane_only(build_or_fail(b));
    ASSERT_FALSE(bare);
    EXPECT_EQ(bare.error().code, spade::Code::invalid_argument);
}
