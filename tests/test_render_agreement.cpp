#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <tuple>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>

#include "core/error.hpp"
#include "core/fp32_math.hpp"
#include "render/agreement.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"
#include "world/world_file.hpp"

// ---------------------------------------------------------------------------
// The RS4 visual/physics agreement MATRIX (S7a Task R9). render/agreement.hpp
// is the pure, general-purpose measurement machinery; this file is the one
// place that says WHICH worlds and WHICH cameras get measured, and pins the
// per-world bands recorded in tests/golden/render/agreement_bands.json.
//
// GOLDEN-FEEDING (references SPADE_GOLDEN_DIR below, per
// test_m1b_bar.cpp's collect_golden_feeding_test_sources() -- see that
// file's own header comment): this file is therefore swept by
// BitPortability.NoLibmTranscendentalInEngineOrGoldenTestSource. The camera
// bookmark schema (content/scenes/*.kscene's camera_bookmarks: target +
// distance + yaw_radians + pitch_radians, an orbit camera -- mirrors
// editor/ui/viewport/camera_controller.cpp's identical model, NOT included
// from here since spade/ stays engine-agnostic and that file's own std::sin/
// std::cos would fail this scan anyway) is converted to a position+
// orientation using spade::math::sin32/cos32 for the yaw/pitch trig and
// Shepperd's rotation-matrix-to-quaternion method (cross/dot/normalize/sqrt
// only -- no trig at all) for the look-at orientation, never glm::angleAxis/
// glm::quatLookAt/std::sin/std::cos.
//
// TEN SHIPPED WORLDS, THREE BOOKMARKS EACH (task brief Step 2; controller
// amendments confirm content/worlds/*.world.yaml IS "the ten shipped
// worlds"): loaded from content/ (SPADE_CONTENT_DIR, tests/CMakeLists.txt's
// own comment explains why that live directory, not a second copy, is the
// right source), each against its OWN content/scenes/<name>.kscene's
// "default"/"top_down"/"family_third" bookmarks (CS5's own three required
// names, tools/tests/test_content_suite.py's
// test_scene_has_at_least_three_camera_bookmarks). 30 parameterized cases,
// each its own ctest entry (gtest_discover_tests) -- comfortably inside the
// 60 s per-case timeout at either resolution this task's brief costs out.
//
// BARE GEOMETRY ONLY (ruling SR-2): worlds are loaded straight off disk via
// world_from_yaml() + scene_from_world() with an EMPTY resolved-mesh span --
// no package instances, no prefab props. None of the ten shipped worlds
// authors props of its own (all are schema v1, confirmed empty `props` on
// load) or spawns a dynamic body in this harness (update_dynamics() is never
// called), so raymarch.hpp's "props/dynamics are structurally invisible to
// the reference" fact does not bite this matrix today -- stated here, not
// assumed, because Task C4 dressing these same worlds with prefab instances
// is exactly the case where it would start to, which is why SR-2 requires a
// re-measure rather than reusing these bands unchanged.
//
// THE FLAT-SKY REQUIREMENT (render/agreement.hpp's own header comment, load-
// bearing, restated here because this is the file that must actually honour
// it): every scene this file builds has its `lighting.sky_horizon` forced
// equal to `sky_zenith` immediately after scene_from_world() returns, BEFORE
// either render() call -- compare_silhouettes()'s single sky_reference_rgb
// parameter is only a correct "is this the sky" test against a flat sky, and
// both DrawMode::shaded and DrawMode::raymarch read `scene.lighting` from
// the SAME (already-flattened) RenderScene, so they agree on the flattened
// colour by construction, not by coincidence.
//
// STEP 1c, RUN FOR REAL (not only the synthetic regression below):
// assert_no_material_matches_sky() runs against every one of the ten worlds'
// OWN authored materials before either render() call, in every one of the 30
// cases -- not merely asserted once in isolation. All ten worlds are schema
// v1 (no materials: section), so every one resolves to the SAME single
// default material (name "default", opaque white) against the SAME default
// LightingDesc's sky_zenith (0.3, 0.5, 0.8) -- far apart by construction --
// but the assertion runs unconditionally regardless, so a future world that
// changes either is caught here, not by a silently-erased silhouette.
//
// SHADOWS EXPLICITLY OFF ON THE FAST PATH (RenderOptions::shadows = false,
// on top of overlays = false): raymarch never casts one (SR-25, out of
// scope by design), so leaving raster's own shadow ON would add a second,
// irrelevant source of colour divergence -- a shadow only ever DARKENS an
// already-hit pixel (never turns a hit into a miss or back), so it cannot
// change which pixels compare_silhouettes() calls "covered", but disabling
// it removes any chance of a shadow-darkened pixel coincidentally
// quantizing to the flat sky reference, which is a strictly-safer posture
// for a colour-based classifier for zero cost (RS4 already permits shading,
// shadows included, to differ between the paths).
// ---------------------------------------------------------------------------

namespace {

using spade::Capacities;
using spade::MaterialDesc;
using spade::Result;
using spade::WorldBuilder;
using spade::WorldDesc;
using spade::render::AgreementResult;
using spade::render::assert_no_material_matches_sky;
using spade::render::Camera;
using spade::render::compare_silhouettes;
using spade::render::DrawMode;
using spade::render::NamedMesh;
using spade::render::pack_sky_reference_bgrx;
using spade::render::PixelFormat;
using spade::render::render;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::scene_from_world;

using Json = nlohmann::json;

// ---------------------------------------------------------------------------
// Small helpers, each mirroring an existing sibling test file's identical
// pattern (test_render_raymarch.cpp's make_target()/render_or_fail(),
// test_world_file.cpp's read_file()) rather than sharing a header -- this
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

[[nodiscard]] bool read_file(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
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

// ---------------------------------------------------------------------------
// Orbit camera (content/scenes/*.kscene's camera_bookmarks schema: target +
// distance + yaw_radians + pitch_radians) -- reimplements
// editor/ui/viewport/camera_controller.cpp's identical model with sin32/
// cos32 in place of std::sin/std::cos (this file's own header comment: it is
// swept for libm transcendentals because it references SPADE_GOLDEN_DIR
// below). Shepperd's method needs no trig at all -- only cross/dot/
// normalize/sqrt, all IEEE-mandated.
// ---------------------------------------------------------------------------

struct OrbitBasis {
    glm::vec3 right, up, backward;
};

[[nodiscard]] OrbitBasis basis_from_direction(const glm::vec3& backward) {
    glm::vec3 world_up(0.0f, 1.0f, 0.0f);
    if (std::fabs(glm::dot(backward, world_up)) > 0.999f) {
        world_up = glm::vec3(0.0f, 0.0f, 1.0f);  // avoid a degenerate cross product looking straight up/down
    }
    const glm::vec3 right = glm::normalize(glm::cross(world_up, backward));
    const glm::vec3 up = glm::cross(backward, right);
    return OrbitBasis{right, up, backward};
}

// Shepperd's method, verbatim in structure from camera_controller.cpp's
// lookAtQuaternion() -- rotation-matrix columns [right, up, backward], no
// trig anywhere in this function.
[[nodiscard]] glm::quat look_at_quaternion(const glm::vec3& position, const glm::vec3& target) {
    const glm::vec3 backward = glm::normalize(position - target);
    const OrbitBasis basis = basis_from_direction(backward);

    const float r00 = basis.right.x, r01 = basis.up.x, r02 = basis.backward.x;
    const float r10 = basis.right.y, r11 = basis.up.y, r12 = basis.backward.y;
    const float r20 = basis.right.z, r21 = basis.up.z, r22 = basis.backward.z;

    const float trace = r00 + r11 + r22;
    float w, x, y, z;
    if (trace > 0.0f) {
        const float s = 0.5f / std::sqrt(trace + 1.0f);
        w = 0.25f / s;
        x = (r21 - r12) * s;
        y = (r02 - r20) * s;
        z = (r10 - r01) * s;
    } else if (r00 > r11 && r00 > r22) {
        const float s = 2.0f * std::sqrt(1.0f + r00 - r11 - r22);
        w = (r21 - r12) / s;
        x = 0.25f * s;
        y = (r01 + r10) / s;
        z = (r02 + r20) / s;
    } else if (r11 > r22) {
        const float s = 2.0f * std::sqrt(1.0f + r11 - r00 - r22);
        w = (r02 - r20) / s;
        x = (r01 + r10) / s;
        y = 0.25f * s;
        z = (r12 + r21) / s;
    } else {
        const float s = 2.0f * std::sqrt(1.0f + r22 - r00 - r11);
        w = (r10 - r01) / s;
        x = (r02 + r20) / s;
        y = (r12 + r21) / s;
        z = 0.25f * s;
    }
    return glm::quat(w, x, y, z);
}

[[nodiscard]] Camera camera_from_bookmark(const glm::vec3& target, double distance, double yaw_radians,
                                           double pitch_radians) {
    const float yaw = static_cast<float>(yaw_radians);
    const float pitch = static_cast<float>(pitch_radians);
    const float cy = spade::math::cos32(yaw), sy = spade::math::sin32(yaw);
    const float cp = spade::math::cos32(pitch), sp = spade::math::sin32(pitch);
    const glm::vec3 dir(cp * sy, sp, cp * cy);
    const glm::vec3 position = target + dir * static_cast<float>(distance);

    Camera camera;
    camera.position = position;
    camera.orientation = look_at_quaternion(position, target);
    return camera;
}

// ---------------------------------------------------------------------------
// content/ loaders (SPADE_CONTENT_DIR -- tests/CMakeLists.txt's own comment
// explains why this suite reads the live shipped directory rather than a
// second copy).
// ---------------------------------------------------------------------------

[[nodiscard]] std::filesystem::path content_dir() {
    return std::filesystem::path(SPADE_CONTENT_DIR);
}

[[nodiscard]] WorldDesc load_shipped_world(const std::string& world_name) {
    const std::filesystem::path path = content_dir() / "worlds" / (world_name + ".world.yaml");
    const Result<WorldDesc> world = spade::load_world_file(path);
    if (!world) {
        ADD_FAILURE() << "load_world_file(" << path << ") failed: " << world.error().context;
        return WorldDesc{};
    }
    return *world;
}

[[nodiscard]] Json load_shipped_scene_json(const std::string& world_name) {
    const std::filesystem::path path = content_dir() / "scenes" / (world_name + ".kscene");
    std::string text;
    if (!read_file(path.string(), text)) {
        ADD_FAILURE() << "could not read shipped scene document: " << path;
        return Json::object();
    }
    return Json::parse(text, /*cb=*/nullptr, /*allow_exceptions=*/false);
}

[[nodiscard]] Camera camera_from_scene_bookmark(const Json& scene_json, const std::string& bookmark_name) {
    const auto bookmarks_it = scene_json.find("camera_bookmarks");
    if (bookmarks_it == scene_json.end()) {
        ADD_FAILURE() << "scene document has no camera_bookmarks object";
        return Camera{};
    }
    const auto bookmark_it = bookmarks_it->find(bookmark_name);
    if (bookmark_it == bookmarks_it->end()) {
        ADD_FAILURE() << "scene document is missing the '" << bookmark_name << "' camera bookmark";
        return Camera{};
    }
    const Json& bookmark = *bookmark_it;
    const Json& target_arr = bookmark.at("target");
    const glm::vec3 target(target_arr.at(0).get<double>(), target_arr.at(1).get<double>(),
                            target_arr.at(2).get<double>());
    const double distance = bookmark.at("distance").get<double>();
    const double yaw = bookmark.at("yaw_radians").get<double>();
    const double pitch = bookmark.at("pitch_radians").get<double>();
    return camera_from_bookmark(target, distance, yaw, pitch);
}

// ---------------------------------------------------------------------------
// agreement_bands.json -- one pinned band per world (task brief Step 3: "set
// each world's threshold", singular per world, covering all three of its
// camera bookmarks). Loaded once, cached for the process lifetime -- 30
// parameterized cases sharing one small parsed document rather than
// re-parsing it 30 times.
// ---------------------------------------------------------------------------

[[nodiscard]] const Json& agreement_bands_json() {
    static const Json bands = [] {
        const std::filesystem::path path = std::filesystem::path(SPADE_GOLDEN_DIR) / "render" / "agreement_bands.json";
        std::string text;
        if (!read_file(path.string(), text)) {
            ADD_FAILURE() << "could not read agreement_bands.json at " << path;
            return Json::object();
        }
        const Json parsed = Json::parse(text, /*cb=*/nullptr, /*allow_exceptions=*/false);
        if (parsed.is_discarded()) {
            ADD_FAILURE() << "agreement_bands.json at " << path << " is not valid JSON";
            return Json::object();
        }
        return parsed;
    }();
    return bands;
}

[[nodiscard]] double band_for_world(const std::string& world_name) {
    const Json& bands = agreement_bands_json();
    const auto worlds_it = bands.find("worlds");
    if (worlds_it == bands.end()) {
        ADD_FAILURE() << "agreement_bands.json has no top-level 'worlds' object";
        return 0.0;
    }
    const auto world_it = worlds_it->find(world_name);
    if (world_it == worlds_it->end()) {
        ADD_FAILURE() << "agreement_bands.json has no pinned band for world '" << world_name << "'";
        return 0.0;
    }
    return world_it->at("band").get<double>();
}

[[nodiscard]] std::string sanitize_for_gtest_name(std::string s) {
    for (char& c : s) {
        if (c == '-') c = '_';
    }
    return s;
}

// The ten shipped worlds (controller amendments: "content/worlds/*.world.yaml
// IS the ten shipped worlds") and CS5's three required bookmark names,
// spelled once here rather than discovered by directory listing -- a
// silently-added eleventh world file would not silently join this matrix
// (and drift the corpus this task measured) without a deliberate edit here.
const std::vector<std::string> kShippedWorldNames = {
    "circuit-track", "figure-eight",  "gate-corridor", "hover-pad",     "pdel-site",
    "pfol-road",     "pint-box",      "pnav-canyon",   "povr-compound", "swarm-grid",
};
const std::vector<std::string> kBookmarkNames = {"default", "top_down", "family_third"};

// Matches Step 2's own "reduced resolution... 160x120 is recommended" --
// exactly the frame goldens' own resolution (task brief's Frame-time budget
// section).
constexpr uint32_t kMatrixWidth = 160, kMatrixHeight = 120;

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
// ===========================================================================

namespace {

[[nodiscard]] WorldDesc box_world(float half_extent) {
    WorldBuilder b = base_builder();
    b.box(glm::vec3(half_extent));
    return build_or_fail(b);
}

// Camera at (0,0,6) looking down -Z at the origin (Camera's own default
// identity orientation, the SAME convention test_render_raymarch.cpp's
// SphereAnalytic fixture uses) -- no trig needed, so this section stays
// entirely literal even though the file as a whole is libm-scanned.
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
    const Result<void> guard = assert_no_material_matches_sky(wrong_world.materials, sky_ref);
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
    RenderTarget fast_target = make_target(fast_storage, kMatrixWidth, kMatrixHeight);
    render_or_fail(wrong_scene, camera, fast_options, fast_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kMatrixWidth, kMatrixHeight);
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
    const Result<void> guard = assert_no_material_matches_sky(world.materials, sky_ref);
    ASSERT_TRUE(guard) << "Step 1c guard: " << guard.error().context;

    const Camera camera = box_guard_camera();

    RenderOptions fast_options;
    fast_options.mode = DrawMode::shaded;
    fast_options.overlays = false;
    fast_options.shadows = false;
    std::vector<uint8_t> fast_storage;
    RenderTarget fast_target = make_target(fast_storage, kMatrixWidth, kMatrixHeight);
    render_or_fail(scene, camera, fast_options, fast_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kMatrixWidth, kMatrixHeight);
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
//    (reproduces the exact SR-27-era scenario: material 0 quantizes to the
//    sky reference colour) and its negative control (an ordinary,
//    non-colliding material palette must pass silently).
// ===========================================================================

TEST(AgreementGuard, Step1cFailsLoudlyWhenAMaterialMatchesTheSkyReference) {
    const std::vector<MaterialDesc> materials = {
        MaterialDesc{.name = "default_matches_sky", .base_color = glm::vec4(0.3f, 0.5f, 0.7f, 1.0f)},
    };
    spade::render::Lighting flat_sky;
    flat_sky.sky_zenith = glm::vec3(0.3f, 0.5f, 0.7f);
    flat_sky.sky_horizon = flat_sky.sky_zenith;
    const uint32_t sky_ref = pack_sky_reference_bgrx(flat_sky);

    const Result<void> result = assert_no_material_matches_sky(materials, sky_ref);
    ASSERT_FALSE(result) << "must fail loudly when a material equals the sky reference colour";
    EXPECT_NE(result.error().context.find("default_matches_sky"), std::string::npos)
        << "failure must name the offending material -- got: " << result.error().context;
    EXPECT_NE(result.error().context.find("index 0"), std::string::npos)
        << "failure must name the offending index -- got: " << result.error().context;
}

TEST(AgreementGuard, Step1cPassesWhenNoMaterialIsNearTheSkyReference) {
    // materials[0] left at the builder's own default (opaque white) against
    // scene.hpp's own default Lighting sky_zenith (0.28, 0.42, 0.62) --
    // white and a mid-blue are nowhere near each other, so this must pass.
    const std::vector<MaterialDesc> materials = {MaterialDesc{}};
    const spade::render::Lighting default_lighting;
    const uint32_t sky_ref = pack_sky_reference_bgrx(default_lighting);

    const Result<void> result = assert_no_material_matches_sky(materials, sky_ref);
    EXPECT_TRUE(result) << "an ordinary, non-colliding material palette must not be flagged: "
                         << (result ? "" : result.error().context);
}

// ===========================================================================
// 3. The matrix: ten shipped worlds x three camera bookmarks each, at
//    160x120, DrawMode::shaded vs DrawMode::raymarch -- see this file's own
//    header comment for the bare-geometry/flat-sky/Step-1c posture every
//    case below follows.
// ===========================================================================

class AgreementMatrix : public ::testing::TestWithParam<std::tuple<std::string, std::string>> {};

TEST_P(AgreementMatrix, MeasuredDisagreementIsWithinItsWorldsPinnedBand) {
    const std::string world_name = std::get<0>(GetParam());
    const std::string camera_name = std::get<1>(GetParam());

    const WorldDesc world = load_shipped_world(world_name);
    ASSERT_FALSE(world.sdf.nodes.empty()) << "sanity: '" << world_name << "' must have real SDF geometry";

    const Json scene_json = load_shipped_scene_json(world_name);
    const Camera camera = camera_from_scene_bookmark(scene_json, camera_name);

    RenderScene scene = scene_or_fail(world);
    const uint32_t sky_ref = flatten_sky_and_pack_reference(scene);

    // Step 1c, run for real against this world's OWN authored materials --
    // fatal, because a collision here means every downstream pixel count is
    // meaningless, not merely off.
    const Result<void> guard = assert_no_material_matches_sky(world.materials, sky_ref);
    ASSERT_TRUE(guard) << "Step 1c guard tripped for '" << world_name << "': " << guard.error().context;

    RenderOptions fast_options;
    fast_options.mode = DrawMode::shaded;
    fast_options.overlays = false;
    fast_options.shadows = false;
    std::vector<uint8_t> fast_storage;
    RenderTarget fast_target = make_target(fast_storage, kMatrixWidth, kMatrixHeight);
    render_or_fail(scene, camera, fast_options, fast_target);

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> raymarch_storage;
    RenderTarget raymarch_target = make_target(raymarch_storage, kMatrixWidth, kMatrixHeight);
    render_or_fail(scene, camera, raymarch_options, raymarch_target);

    const AgreementResult result = compare_silhouettes(fast_target, raymarch_target, sky_ref);
    std::cout << "[AgreementMatrix] " << world_name << "/" << camera_name
              << ": disagreement_fraction=" << result.disagreement_fraction << " covered_a=" << result.covered_a
              << " covered_b=" << result.covered_b << " (of " << (kMatrixWidth * kMatrixHeight) << " px)\n";

    ASSERT_GT(result.covered_a, 0u) << "sanity: '" << world_name << "'/" << camera_name
                                     << " tessellated path must see SOME geometry, not pure sky";
    ASSERT_GT(result.covered_b, 0u) << "sanity: '" << world_name << "'/" << camera_name
                                     << " raymarch path must see SOME geometry, not pure sky";

    const double band = band_for_world(world_name);
    EXPECT_LE(result.disagreement_fraction, band)
        << "'" << world_name << "'/" << camera_name << " disagreement_fraction "
        << (result.disagreement_fraction * 100.0) << "% exceeds its pinned band " << (band * 100.0)
        << "% (agreement_bands.json)";
}

INSTANTIATE_TEST_SUITE_P(
    ShippedWorlds, AgreementMatrix,
    ::testing::Combine(::testing::ValuesIn(kShippedWorldNames), ::testing::ValuesIn(kBookmarkNames)),
    [](const ::testing::TestParamInfo<AgreementMatrix::ParamType>& info) {
        return sanitize_for_gtest_name(std::get<0>(info.param)) + "_" + std::get<1>(info.param);
    });
