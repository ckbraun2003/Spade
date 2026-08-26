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
// This file never references SPADE_GOLDEN_DIR (no committed sha256 -- it
// asserts geometric/coverage behaviour, mirroring test_render_shading.cpp's
// own posture) and is NOT swept by test_m1b_bar.cpp's libm-transcendental
// scan for the identical reason: nothing here feeds a committed golden
// digest, so ordinary trig is fine for building TEST cameras (production
// render/raymarch.cpp itself uses only sin32/cos32/std::sqrt, per its own
// header comment).
// ---------------------------------------------------------------------------

namespace {

using spade::Capacities;
using spade::Result;
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
