#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "render/raster_cpu.hpp"
#include "render/raymarch.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// The exact SDF raymarch reference path (S7a Task R8). Step 1's own burdens
// of proof, plus the controller amendments' binding null-vs-empty split:
//
//   1. Raymarching hover-pad at 96x72 produces a non-empty silhouette whose
//      covered-pixel count is within 2% of the tessellated (DrawMode::shaded)
//      path's own count -- a loose smoke bound (task brief: "the real bound
//      is measured in R9").
//   2. A NULL scene.sdf (a hand-built RenderScene that never pointed it at a
//      program) and a non-null but EMPTY SdfProgram (validly zero nodes) are
//      DIFFERENT cases, per the controller amendments -- both must yield sky
//      only, never a fault. Tested separately so a fix for one case cannot
//      silently leave the other broken.
//   3. Determinism: byte-identical output across repeated renders of the
//      same scene/camera/target.
//
// FIX ROUND 1 additions (review found the algorithm correct, the constants
// and two behaviours wrong):
//   4. A bare-sphere fixture with a CLOSED-FORM exact silhouette -- section
//      1's own hover-pad fixture cannot see the constant table at all (96x72
//      makes one pixel ~0.096 m, far coarser than the original 2 cm epsilon
//      bug); this one can.
//   5. Frustum parity with raster_cpu.cpp: a camera fully enclosed by a
//      convex solid must see nothing (SR-13 back-face cull, mirrored here by
//      the "started inside solid" cull), and geometry beyond
//      Camera::far_plane must be invisible to both paths.
//   6. Per-leaf material resolution: a world whose default material
//      (node_materials' own fallback target) happens to match the sky
//      colour must not erase a real silhouette's coverage.
//
// This file never references SPADE_GOLDEN_DIR (no committed sha256 -- it
// asserts geometric/coverage behaviour, mirroring test_render_shading.cpp's
// own posture) and is NOT swept by test_m1b_bar.cpp's libm-transcendental
// scan for the identical reason: nothing here feeds a committed golden
// digest, so ordinary trig is fine for building TEST cameras and the exact
// analytic sphere oracle (production render/raymarch.cpp itself uses only
// sin32/cos32/std::sqrt via render/scene.hpp's shared tan32(), per its own
// header comment).
// ---------------------------------------------------------------------------

namespace {

using spade::Capacities;
using spade::LightingDesc;
using spade::MaterialDesc;
using spade::MaterialShading;
using spade::Result;
using spade::SdfPose;
using spade::WorldBuilder;
using spade::WorldDesc;
using spade::render::Camera;
using spade::render::DrawMode;
using spade::render::Lighting;
using spade::render::NamedMesh;
using spade::render::PixelFormat;
using spade::render::render;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::scene_from_world;
using spade::render::sky_gradient_color;

// Mirrors test_render_shading.cpp's identical helper: storage is the
// caller's, target is a non-owning view over it (PA-1); pre-filled with a
// sentinel byte so "every byte was actually overwritten" is a real
// assertion.
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

struct Bgr {
    uint8_t b, g, r, x;
};

[[nodiscard]] Bgr pixel_at(const std::vector<uint8_t>& storage, uint32_t width, uint32_t x, uint32_t y) {
    const size_t idx = (static_cast<size_t>(y) * width + x) * 4;
    return Bgr{storage[idx], storage[idx + 1], storage[idx + 2], storage[idx + 3]};
}

[[nodiscard]] bool operator==(const Bgr& a, const Bgr& b) {
    return a.b == b.b && a.g == b.g && a.r == b.r && a.x == b.x;
}

// Independent-in-spelling, but the SAME formula raster_cpu.cpp's own
// (private) to_byte uses -- reproduced here only to turn
// sky_gradient_color()'s float result into the byte triple a rendered frame
// actually stores, not to test the rounding itself.
[[nodiscard]] uint8_t to_byte(float channel) {
    const float clamped = std::clamp(channel, 0.0f, 1.0f);
    return static_cast<uint8_t>(std::lround(clamped * 255.0f));
}

[[nodiscard]] Bgr expected_sky_bgr(const Lighting& lighting, uint32_t height, uint32_t y) {
    const float t = height > 1 ? static_cast<float>(y) / static_cast<float>(height - 1) : 0.0f;
    const glm::vec3 sky = sky_gradient_color(lighting, t);
    return Bgr{to_byte(sky.b), to_byte(sky.g), to_byte(sky.r), 0xFFu};
}

// A world with no SDF nodes at all -- scene_from_world() still produces a
// non-null, but EMPTY (SdfProgram::empty() == true), RenderScene::sdf
// pointing at world.sdf. Distinct from the hand-built-RenderScene null case
// exercised separately below.
[[nodiscard]] WorldBuilder base_builder() {
    WorldBuilder b;
    b.name("render-raymarch-test").capacities(Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
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

// hover-pad's own shape (task brief, and test_render_csg.cpp's identical
// fixture): plane, cylinder, union. A pure-union program -- every leaf
// tessellates individually on the raster side, and marches directly on this
// side.
[[nodiscard]] WorldDesc hover_pad_world() {
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).cylinder(0.5f, 0.1f).union_();
    return build_or_fail(b);
}

// A camera well above and back from the pad, pitched down -- puts a real
// horizon in frame (some sky rows, some ground rows) and the pad itself
// somewhere in the ground region, so "covered vs sky" is a meaningful,
// non-degenerate split for both paths (mirrors test_render_shading.cpp's own
// TessellatedAndAnalyticGroundAgreeAcrossTheHardHorizonSeam camera pose).
[[nodiscard]] Camera hover_pad_camera() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 3.0f, 6.0f);
    camera.orientation = glm::angleAxis(glm::radians(-20.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    return camera;
}

}  // namespace

// ===========================================================================
// 1. Step 1: hover-pad coverage within 2% of the tessellated path's.
// ===========================================================================

TEST(RaymarchSmoke, HoverPadCoverageWithinTwoPercentOfTessellated) {
    const WorldDesc world = hover_pad_world();
    const RenderScene scene = scene_or_fail(world);
    ASSERT_NE(scene.sdf, nullptr) << "sanity: scene_from_world must point RenderScene::sdf at world.sdf";
    ASSERT_FALSE(scene.sdf->empty()) << "sanity: hover-pad has real SDF nodes";

    const Camera camera = hover_pad_camera();
    constexpr uint32_t kW = 96, kH = 72;

    RenderOptions shaded_options;
    shaded_options.mode = DrawMode::shaded;
    shaded_options.overlays = false;  // a fair silhouette comparison: raymarch draws no overlays at all
    std::vector<uint8_t> shaded_storage;
    RenderTarget shaded_target = make_target(shaded_storage, kW, kH);
    render_or_fail(scene, camera, shaded_options, shaded_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kW, kH);
    render_or_fail(scene, camera, raymarch_options, raymarch_target);

    uint32_t shaded_covered = 0, raymarch_covered = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        const Bgr expected_sky = expected_sky_bgr(scene.lighting, kH, y);
        for (uint32_t x = 0; x < kW; ++x) {
            if (!(pixel_at(shaded_storage, kW, x, y) == expected_sky)) {
                ++shaded_covered;
            }
            if (!(pixel_at(raymarch_storage, kW, x, y) == expected_sky)) {
                ++raymarch_covered;
            }
        }
    }

    ASSERT_GT(raymarch_covered, 0u) << "the raymarch pass must actually produce a non-empty silhouette";
    ASSERT_GT(shaded_covered, 0u) << "sanity: the tessellated reference must also see ground/pad, not pure sky";
    // Sanity: this fixture must be a genuinely non-degenerate split (neither
    // path may read as "the whole frame", which would make a 2% comparison
    // meaningless) -- both a real ground/sky boundary and headroom for the
    // pad to sit inside the ground region.
    ASSERT_LT(shaded_covered, kW * kH) << "sanity: this camera pose must also see some sky";

    const double diff = std::fabs(static_cast<double>(raymarch_covered) - static_cast<double>(shaded_covered));
    const double relative = diff / static_cast<double>(shaded_covered);
    EXPECT_LE(relative, 0.02) << "raymarch covered " << raymarch_covered << " px, tessellated covered "
                               << shaded_covered << " px (of " << (kW * kH) << ") -- " << (relative * 100.0)
                               << "% relative difference, over the 2% smoke bound";
}

// ===========================================================================
// 2. Null vs empty scene.sdf -- both must be sky-only, never a fault.
// ===========================================================================

TEST(RaymarchSmoke, EmptySdfProgramYieldsSkyOnlyNeverFaults) {
    // A world with capacities but NO SDF nodes at all: scene_from_world()
    // still points RenderScene::sdf at world.sdf (non-null), but that
    // program is validly EMPTY (SdfProgram::empty() == true) -- distinct
    // from the null-pointer case below.
    const WorldDesc world = build_or_fail(base_builder());
    const RenderScene scene = scene_or_fail(world);
    ASSERT_NE(scene.sdf, nullptr) << "sanity: this is the non-null-but-empty case, not the null case";
    ASSERT_TRUE(scene.sdf->empty()) << "sanity: a world with zero SDF nodes must produce an empty program";

    Camera camera;
    camera.position = glm::vec3(0.0f, 1000.0f, 1000.0f);  // looks at nothing, mirrors test_render_shading.cpp
    RenderOptions options;
    options.mode = DrawMode::raymarch;
    constexpr uint32_t kW = 64, kH = 64;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kW, kH);
    render_or_fail(scene, camera, options, target);

    for (uint32_t y = 0; y < kH; ++y) {
        const Bgr expected = expected_sky_bgr(scene.lighting, kH, y);
        for (uint32_t x = 0; x < kW; ++x) {
            ASSERT_EQ(pixel_at(storage, kW, x, y), expected)
                << "(" << x << "," << y << ") must be exactly the sky gradient -- an empty SDF program has nothing "
                                            "to hit";
        }
    }
}

TEST(RaymarchSmoke, NullSdfPointerYieldsSkyOnlyNeverFaults) {
    // A hand-built RenderScene -- RenderScene::sdf defaults to nullptr
    // (scene.hpp's own "non-owning, MAY BE NULL" contract), never populated
    // because this fixture never calls scene_from_world() at all. This is
    // the OTHER half of the null-vs-empty split the controller amendments
    // require: a null pointer is not merely "an empty program found a
    // different way" -- render_raymarch() must check for it explicitly
    // before ever dereferencing scene.sdf.
    RenderScene scene;
    ASSERT_EQ(scene.sdf, nullptr) << "sanity: this is the null-pointer case, not the empty-program case";
    scene.materials.push_back(spade::render::Material{});  // index 0 must exist for shading to be reachable at all
    scene.lighting.sky_zenith = glm::vec3(0.1f, 0.2f, 0.9f);
    scene.lighting.sky_horizon = glm::vec3(0.8f, 0.7f, 0.6f);

    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    RenderOptions options;
    options.mode = DrawMode::raymarch;
    constexpr uint32_t kW = 64, kH = 64;
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, kW, kH);
    render_or_fail(scene, camera, options, target);

    for (uint32_t y = 0; y < kH; ++y) {
        const Bgr expected = expected_sky_bgr(scene.lighting, kH, y);
        for (uint32_t x = 0; x < kW; ++x) {
            ASSERT_EQ(pixel_at(storage, kW, x, y), expected)
                << "(" << x << "," << y << ") must be exactly the sky gradient -- a null scene.sdf has nothing to "
                                            "hit and must never be dereferenced";
        }
    }
}

// ===========================================================================
// 3. Determinism: byte-identical across repeated renders.
// ===========================================================================

TEST(RaymarchSmoke, DeterministicAcrossRepeatedRenders) {
    const WorldDesc world = hover_pad_world();
    const RenderScene scene = scene_or_fail(world);
    const Camera camera = hover_pad_camera();
    RenderOptions options;
    options.mode = DrawMode::raymarch;
    constexpr uint32_t kW = 96, kH = 72;

    std::vector<uint8_t> first_storage;
    RenderTarget first_target = make_target(first_storage, kW, kH);
    render_or_fail(scene, camera, options, first_target);

    // A SECOND, independently-allocated (and differently pre-filled, via
    // make_target's own 0xAA sentinel each call) target -- deliberately not
    // the same buffer reused, so this proves render_raymarch() itself is a
    // pure function of its inputs, not merely "leaves stale bytes alone".
    std::vector<uint8_t> second_storage;
    RenderTarget second_target = make_target(second_storage, kW, kH);
    render_or_fail(scene, camera, options, second_target);

    EXPECT_EQ(first_storage, second_storage) << "render_raymarch() must be byte-identical across repeated calls "
                                                 "with the same scene/camera/target";
}

// ===========================================================================
// 4. Sphere analytic fixture (fix round 1). hover-pad's own coverage stat
//    cannot see the constant table (section 1's own header note) -- a bare
//    sphere has a CLOSED-FORM exact silhouette, so this fixture can.
// ===========================================================================

namespace {

[[nodiscard]] WorldDesc single_sphere_world(float radius) {
    WorldBuilder b = base_builder();
    b.sphere(radius);
    return build_or_fail(b);
}

// Exact pixel-space silhouette RADIUS of an on-axis sphere (radius `radius`,
// centered at the world origin) as seen by a Camera at world-space distance
// `distance` along +Z with NO rotation (looking straight down -Z at the
// sphere's own center, camera.hpp's own default orientation).
//
// Derivation: a point on the sphere's silhouette lies at half-angle
// theta = asin(radius/distance) from the view axis. A pinhole camera's
// perspective projection maps a ray at azimuth phi and half-angle theta to
// camera-space direction (sin(theta)*cos(phi), sin(theta)*sin(phi),
// -cos(theta)), which render/raymarch.cpp's own NDC formulas
// (x_ndc = (f/aspect)*dir.x/(-dir.z), y_ndc = f*dir.y/(-dir.z), f =
// 1/tan(half_fov_y)) turn into PIXEL offsets from the frame center
// (width/2, height/2) of (0.5*height*f*tan(theta)*cos(phi),
// -0.5*height*f*tan(theta)*sin(phi)) -- the aspect-ratio terms in x_ndc and
// the width/aspect=height identity cancel exactly, so as phi sweeps
// [0, 2*pi) this traces a PERFECT CIRCLE in actual screen pixels (not merely
// in NDC, where it would be an ellipse off a square aspect), of radius
// 0.5*height*f*tan(theta), regardless of aspect ratio.
[[nodiscard]] double exact_sphere_pixel_radius(float radius, float distance, float fov_y_radians, uint32_t height) {
    const double theta = std::asin(static_cast<double>(radius) / static_cast<double>(distance));
    const double half_fov = static_cast<double>(fov_y_radians) * 0.5;
    const double f = 1.0 / std::tan(half_fov);
    return 0.5 * static_cast<double>(height) * f * std::tan(theta);
}

[[nodiscard]] uint32_t exact_sphere_coverage(float radius, float distance, float fov_y_radians, uint32_t width,
                                              uint32_t height) {
    const double pixel_radius = exact_sphere_pixel_radius(radius, distance, fov_y_radians, height);
    const double radius_sq = pixel_radius * pixel_radius;
    const double cx = static_cast<double>(width) * 0.5;
    const double cy = static_cast<double>(height) * 0.5;
    uint32_t covered = 0;
    for (uint32_t y = 0; y < height; ++y) {
        const double dy = (static_cast<double>(y) + 0.5) - cy;
        for (uint32_t x = 0; x < width; ++x) {
            const double dx = (static_cast<double>(x) + 0.5) - cx;
            if (dx * dx + dy * dy <= radius_sq) {
                ++covered;
            }
        }
    }
    return covered;
}

}  // namespace

TEST(RaymarchSmoke, SphereAnalyticSilhouetteAgreesWithTheExactCircleWithinHalfAPercent) {
    constexpr float kRadius = 0.5f, kDistance = 1.2f;
    const WorldDesc world = single_sphere_world(kRadius);
    const RenderScene scene = scene_or_fail(world);
    ASSERT_NE(scene.sdf, nullptr);
    ASSERT_FALSE(scene.sdf->empty());

    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, kDistance);
    constexpr uint32_t kW = 480, kH = 270;

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kW, kH);
    render_or_fail(scene, camera, raymarch_options, raymarch_target);

    RenderOptions shaded_options;
    shaded_options.mode = DrawMode::shaded;
    shaded_options.overlays = false;
    std::vector<uint8_t> shaded_storage;
    RenderTarget shaded_target = make_target(shaded_storage, kW, kH);
    render_or_fail(scene, camera, shaded_options, shaded_target);

    const uint32_t exact_covered = exact_sphere_coverage(kRadius, kDistance, camera.fov_y_radians, kW, kH);
    ASSERT_GT(exact_covered, 0u) << "sanity: the analytic circle must actually cover pixels";

    uint32_t raymarch_covered = 0, shaded_covered = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        const Bgr expected_sky = expected_sky_bgr(scene.lighting, kH, y);
        for (uint32_t x = 0; x < kW; ++x) {
            if (!(pixel_at(raymarch_storage, kW, x, y) == expected_sky)) {
                ++raymarch_covered;
            }
            if (!(pixel_at(shaded_storage, kW, x, y) == expected_sky)) {
                ++shaded_covered;
            }
        }
    }

    // Sanity: the tessellated path's own (separate, tessellation-driven)
    // error against the true sphere must also stay small, or this fixture's
    // own analytic oracle is the thing that is wrong, not raymarch.
    const double shaded_relative =
        std::fabs(static_cast<double>(shaded_covered) - static_cast<double>(exact_covered)) / exact_covered;
    ASSERT_LT(shaded_relative, 0.05) << "sanity: tessellated-vs-exact error is implausibly large (" << shaded_covered
                                      << " vs exact " << exact_covered << ") -- check the analytic oracle first";

    const double raymarch_relative =
        std::fabs(static_cast<double>(raymarch_covered) - static_cast<double>(exact_covered)) / exact_covered;
    EXPECT_LE(raymarch_relative, 0.005)
        << "raymarch covered " << raymarch_covered << " px, EXACT analytic covered " << exact_covered << " px (of "
        << (kW * kH) << ") -- " << (raymarch_relative * 100.0)
        << "% relative difference against the TRUE geometry -- fix round 1's retuned constants must keep this well "
           "under 1%, not merely under hover-pad's original loose 2% (which could not see a 2 cm epsilon error at "
           "all)";
}

// ===========================================================================
// 5. Frustum parity with raster_cpu.cpp (fix round 1, review IMPORTANT):
//    near/far-plane bounds, and a camera embedded in a solid.
// ===========================================================================

TEST(RaymarchSmoke, CameraFullyInsideAConvexSolidSeesNothingLikeRastersBackFaceCull) {
    // A 4m box (half-extents 2m) centered at the origin, camera dead center
    // -- the exact fixture the review measured 4096/4096 false coverage on
    // before this fix (a ray starting inside a solid satisfied
    // d <= 0 <= epsilon at t=0).
    WorldBuilder b = base_builder();
    b.box(glm::vec3(2.0f));
    const WorldDesc world = build_or_fail(b);
    const RenderScene scene = scene_or_fail(world);

    Camera camera;
    camera.position = glm::vec3(0.0f);
    constexpr uint32_t kW = 64, kH = 64;

    RenderOptions shaded_options;
    shaded_options.mode = DrawMode::shaded;
    shaded_options.overlays = false;
    std::vector<uint8_t> shaded_storage;
    RenderTarget shaded_target = make_target(shaded_storage, kW, kH);
    render_or_fail(scene, camera, shaded_options, shaded_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kW, kH);
    render_or_fail(scene, camera, raymarch_options, raymarch_target);

    for (uint32_t y = 0; y < kH; ++y) {
        const Bgr expected_sky = expected_sky_bgr(scene.lighting, kH, y);
        for (uint32_t x = 0; x < kW; ++x) {
            ASSERT_EQ(pixel_at(shaded_storage, kW, x, y), expected_sky)
                << "sanity: raster's own SR-13 back-face cull must show nothing from inside a convex box";
            ASSERT_EQ(pixel_at(raymarch_storage, kW, x, y), expected_sky)
                << "(" << x << "," << y << ") a camera fully enclosed by a solid must see sky, not the solid's own "
                                            "inside surface";
        }
    }
}

TEST(RaymarchSmoke, GeometryBeyondFarPlaneIsInvisibleToBothPaths) {
    // The exact fixture the review measured 172/4096 false coverage on
    // before this fix (no far-plane escape at all).
    WorldBuilder b = base_builder();
    b.sphere(1.0f, SdfPose{.position = glm::vec3(0.0f, 0.0f, -50.0f)});
    const WorldDesc world = build_or_fail(b);
    const RenderScene scene = scene_or_fail(world);

    Camera camera;
    camera.position = glm::vec3(0.0f);
    camera.far_plane = 10.0f;  // the sphere at z=-50 sits well beyond this
    constexpr uint32_t kW = 64, kH = 64;

    RenderOptions shaded_options;
    shaded_options.mode = DrawMode::shaded;
    shaded_options.overlays = false;
    std::vector<uint8_t> shaded_storage;
    RenderTarget shaded_target = make_target(shaded_storage, kW, kH);
    render_or_fail(scene, camera, shaded_options, shaded_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kW, kH);
    render_or_fail(scene, camera, raymarch_options, raymarch_target);

    for (uint32_t y = 0; y < kH; ++y) {
        const Bgr expected_sky = expected_sky_bgr(scene.lighting, kH, y);
        for (uint32_t x = 0; x < kW; ++x) {
            ASSERT_EQ(pixel_at(shaded_storage, kW, x, y), expected_sky)
                << "sanity: raster's own far-plane clip must remove this sphere entirely";
            ASSERT_EQ(pixel_at(raymarch_storage, kW, x, y), expected_sky)
                << "(" << x << "," << y << ") geometry beyond far_plane must be invisible to the reference path "
                                            "too, matching raster's own clip";
        }
    }
}

// ===========================================================================
// 6. Per-leaf material resolution (fix round 1, review IMPORTANT --
//    SUPERSEDES this file's own original materials[0]-for-everything
//    ruling).
// ===========================================================================

TEST(RaymarchSmoke, MaterialMisresolutionAgainstSkyColourNoLongerErasesTheSilhouette) {
    // Constructed exactly as the review's own counterexample: material 0
    // (the builder's default slot, and node_materials' own empty-array
    // fallback target) is unlit and EQUAL to the (flattened, so every row
    // shares one constant expected colour) sky colour; the sphere is
    // authored at material index 1, a visibly distinct colour. Before this
    // fix, EVERY raymarched hit resolved to materials[0] regardless of which
    // primitive it actually hit -- painting the whole silhouette the exact
    // sky colour and erasing it from a colour-vs-sky classifier, even though
    // raster (which meshes per-node, so its own material lookup was never
    // wrong) shows the sphere plainly.
    constexpr glm::vec3 kFlatColor(0.3f, 0.5f, 0.7f);
    WorldBuilder b = base_builder();
    b.lighting(LightingDesc{
        .sky_zenith = kFlatColor,
        .sky_horizon = kFlatColor,  // flat sky -- every row's expected colour is the SAME constant
    });
    b.material(MaterialDesc{.name = "default_matches_sky",
                             .base_color = glm::vec4(kFlatColor, 1.0f),
                             .shading = MaterialShading::unlit});
    b.material(MaterialDesc{.name = "sphere", .base_color = {0.9f, 0.1f, 0.1f, 1.0f}});
    b.sphere(0.8f).material_for_last_node(1);
    const WorldDesc world = build_or_fail(b);
    const RenderScene scene = scene_or_fail(world);
    ASSERT_EQ(scene.materials.size(), 2u);

    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 3.0f);
    constexpr uint32_t kW = 96, kH = 72;

    RenderOptions shaded_options;
    shaded_options.mode = DrawMode::shaded;
    shaded_options.overlays = false;
    std::vector<uint8_t> shaded_storage;
    RenderTarget shaded_target = make_target(shaded_storage, kW, kH);
    render_or_fail(scene, camera, shaded_options, shaded_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kW, kH);
    render_or_fail(scene, camera, raymarch_options, raymarch_target);

    uint32_t shaded_covered = 0, raymarch_covered = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        const Bgr expected_sky = expected_sky_bgr(scene.lighting, kH, y);
        for (uint32_t x = 0; x < kW; ++x) {
            if (!(pixel_at(shaded_storage, kW, x, y) == expected_sky)) {
                ++shaded_covered;
            }
            if (!(pixel_at(raymarch_storage, kW, x, y) == expected_sky)) {
                ++raymarch_covered;
            }
        }
    }

    ASSERT_GT(shaded_covered, 0u) << "sanity: the raster path must see the sphere";
    EXPECT_GT(raymarch_covered, 0u)
        << "raymarch covered 0 px -- the WHOLE silhouette resolved to material 0 (which equals the sky colour by "
           "construction), the exact failure mode this test pins";

    const double relative =
        std::fabs(static_cast<double>(raymarch_covered) - static_cast<double>(shaded_covered)) / shaded_covered;
    EXPECT_LE(relative, 0.02) << "raymarch covered " << raymarch_covered << " px, tessellated covered "
                               << shaded_covered << " px -- " << (relative * 100.0) << "% relative difference";
}

// ===========================================================================
// 7. Frustum parity: camera-space DEPTH, not radial distance (fix round 2,
//    review IMPORTANT). `far_plane` is deliberately set to 10 m, not the
//    default 1000 m against a ~10 m world -- the same "fixture cannot
//    measure the constant it is meant to check" shape the review named for
//    the original epsilon defect, so this one is built to actually see it.
// ===========================================================================

TEST(RaymarchSmoke, GeometryWithinTheFrustumButBeyondTheOldRadialCapIsStillVisible) {
    // A sphere positioned at 0.9 of the frustum's own half-extents at depth
    // 9 m: an off-axis ray toward it has RADIAL distance ~13.1 m (beyond
    // far_plane=10) while its CAMERA-SPACE DEPTH is exactly 9 m (well within
    // it). Fix round 1's radial-t escape stopped such a ray at t=10, short
    // of the sphere; raster (which clips on depth, raster_cpu.cpp's ported
    // clipTriangleNearFar) draws it in full.
    constexpr float kDepth = 9.0f, kRadius = 0.6f, kFraction = 0.9f;
    Camera camera;
    camera.position = glm::vec3(0.0f);
    camera.far_plane = 10.0f;
    constexpr uint32_t kW = 320, kH = 180;  // 16:9, matching the review's own fixture

    const double half_fov = static_cast<double>(camera.fov_y_radians) * 0.5;
    const double half_extent_y = static_cast<double>(kDepth) * std::tan(half_fov);
    const double half_extent_x = half_extent_y * (static_cast<double>(kW) / static_cast<double>(kH));
    const glm::vec3 sphere_position(static_cast<float>(kFraction * half_extent_x),
                                     static_cast<float>(kFraction * half_extent_y), -kDepth);

    WorldBuilder b = base_builder();
    b.sphere(kRadius, SdfPose{.position = sphere_position});
    const WorldDesc world = build_or_fail(b);
    const RenderScene scene = scene_or_fail(world);

    RenderOptions shaded_options;
    shaded_options.mode = DrawMode::shaded;
    shaded_options.overlays = false;
    std::vector<uint8_t> shaded_storage;
    RenderTarget shaded_target = make_target(shaded_storage, kW, kH);
    render_or_fail(scene, camera, shaded_options, shaded_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kW, kH);
    render_or_fail(scene, camera, raymarch_options, raymarch_target);

    uint32_t shaded_covered = 0, raymarch_covered = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        const Bgr expected_sky = expected_sky_bgr(scene.lighting, kH, y);
        for (uint32_t x = 0; x < kW; ++x) {
            if (!(pixel_at(shaded_storage, kW, x, y) == expected_sky)) {
                ++shaded_covered;
            }
            if (!(pixel_at(raymarch_storage, kW, x, y) == expected_sky)) {
                ++raymarch_covered;
            }
        }
    }

    ASSERT_GT(shaded_covered, 0u)
        << "sanity: raster must see this sphere -- its depth (9 m) is inside far_plane (10 m)";
    EXPECT_GT(raymarch_covered, 0u)
        << "raymarch saw nothing -- the RADIAL distance to this sphere (~13.1 m) exceeds far_plane even though its "
           "CAMERA-SPACE DEPTH (9 m) does not; the escape must bound depth, not radial t";

    const double relative =
        std::fabs(static_cast<double>(raymarch_covered) - static_cast<double>(shaded_covered)) / shaded_covered;
    EXPECT_LE(relative, 0.05) << "raymarch covered " << raymarch_covered << " px, tessellated covered "
                               << shaded_covered << " px -- " << (relative * 100.0) << "% relative difference";
}

// ===========================================================================
// 8. "March past" the solid a ray starts inside (fix round 2, review MINOR).
//    Section 5's one-object fixture (CameraFullyInsideAConvexSolidSeesNothing
//    LikeRastersBackFaceCull, above) cannot distinguish "the whole ray is
//    abandoned" from "the enclosing solid is correctly skipped and there is
//    nothing else to find" -- both read as 100% sky. This one has a second
//    object.
// ===========================================================================

TEST(RaymarchSmoke, CameraInsideASolidWithAnotherObjectBehindItSeesTheSecondObject) {
    // A 10x10x0.6 m slab centered ON the camera (half-extents 5,5,0.3 -- the
    // camera sits dead in the middle of its own thin Z dimension) unioned
    // with an UNOCCLUDED sphere further along the view axis. Raster's SR-13
    // cull hides the slab itself (camera embedded in it) but still draws the
    // sphere behind it -- a completely separate, unoccluded piece of
    // geometry. Fix round 1's "started inside solid -> whole ray is a miss"
    // cull wrongly hid the sphere too.
    WorldBuilder b = base_builder();
    b.box(glm::vec3(5.0f, 5.0f, 0.3f));
    b.sphere(1.0f, SdfPose{.position = glm::vec3(0.0f, 0.0f, -4.0f)});
    b.union_();
    const WorldDesc world = build_or_fail(b);
    const RenderScene scene = scene_or_fail(world);

    Camera camera;
    camera.position = glm::vec3(0.0f);
    constexpr uint32_t kW = 128, kH = 128;

    RenderOptions shaded_options;
    shaded_options.mode = DrawMode::shaded;
    shaded_options.overlays = false;
    std::vector<uint8_t> shaded_storage;
    RenderTarget shaded_target = make_target(shaded_storage, kW, kH);
    render_or_fail(scene, camera, shaded_options, shaded_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kW, kH);
    render_or_fail(scene, camera, raymarch_options, raymarch_target);

    uint32_t shaded_covered = 0, raymarch_covered = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        const Bgr expected_sky = expected_sky_bgr(scene.lighting, kH, y);
        for (uint32_t x = 0; x < kW; ++x) {
            if (!(pixel_at(shaded_storage, kW, x, y) == expected_sky)) {
                ++shaded_covered;
            }
            if (!(pixel_at(raymarch_storage, kW, x, y) == expected_sky)) {
                ++raymarch_covered;
            }
        }
    }

    ASSERT_GT(shaded_covered, 0u) << "sanity: raster must see the sphere behind the slab it back-face-culls";
    EXPECT_GT(raymarch_covered, 0u)
        << "raymarch saw nothing -- the ray starting inside the slab must march PAST it and find the sphere, not "
           "abandon the whole ray";

    const double relative =
        std::fabs(static_cast<double>(raymarch_covered) - static_cast<double>(shaded_covered)) / shaded_covered;
    EXPECT_LE(relative, 0.05) << "raymarch covered " << raymarch_covered << " px, tessellated covered "
                               << shaded_covered << " px -- " << (relative * 100.0) << "% relative difference";
}
