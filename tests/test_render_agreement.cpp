#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <utility>
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
// per-(world, camera) bands recorded in
// tests/golden/render/agreement_bands.json.
//
// FIX ROUND 1 (review): the review proved, by construction (Probe A --
// re-rendering the reference against a world with EVERY node deleted except
// its ground plane), that 24 of the original 30 per-WORLD-banded cases
// passed their pinned band with the reference's entire geometry gone, 22
// bit-identical to the committed value. The root cause: every shipped world
// paints its infinite ground plane below the horizon regardless of what
// sits on it, so "covered" (not sky) saturates the WHOLE frame for
// `top_down` and most of it for `default` -- the only region an error can
// move is the sky area, and when that area is zero (or the raster path also
// saturates it), the per-pixel XOR is identically zero for ANY reference
// geometry whatsoever. A per-WORLD band (this world's *maximum* observed
// camera) additionally LAUNDERED a discriminating camera's tight signal onto
// a non-discriminating one's inflated threshold (circuit-track/
// family_third: 1.74% with the world deleted, invisible under the 2.5% band
// its own horizon-inflated `default` camera set). SR-30/SR-31 below are the
// fix. This file's own git history carries the pre-fix-round version for
// comparison; task-R9-report.md's fix-round-1 section is the full account.
//
// SR-30 -- EVERY CASE MUST PROVE ITS OWN DETECTION SURFACE: at measurement
// time (and again, LIVE, in every test run below -- "assert it in the test",
// not narrate it), each case's reference is re-rendered against
// render/agreement.hpp's strip_to_ground_plane_only(world) (every SDF node
// deleted except the standalone ground-plane leaf) and compared against the
// SAME real fast-path frame. `detects_total_deletion` records whether THAT
// probe's disagreement exceeds the case's own pinned band -- i.e. whether
// the band, as pinned, would actually catch a reference with zero real
// scene geometry. Bands moved from per-WORLD to per-(WORLD, CAMERA)
// (nothing else needed to change to stop the laundering: each camera's band
// now comes ONLY from its own measured value). A case that proves `false`
// is NOT deleted from the matrix -- it is still a real tripwire against a
// change that breaks horizon-deficit saturation itself, and it is a real
// product camera bookmark CK-2 will show -- it is labelled, not hidden, and
// excluded from ever being used to derive ANOTHER camera's band (per-camera
// banding already makes that structural, not a rule someone has to
// remember).
//
// SR-31 -- THE BANDS FILE IS STRUCTURALLY VERSIONED: `agreement_bands.json`
// keys its data under `measurements.<version>`, with a top-level
// `active_measurement_version` selecting which one is live. A re-measure
// (SR-2: Task C4, after dressing these worlds with prefab instances) adds a
// NEW version and bumps the selector -- there is no field left whose only
// valid edit is the in-place mutation the original file's own prose asked
// readers not to make.
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
// PARAMETERIZED AS A FLAT "world|camera" STRING, NOT ::testing::Combine's
// std::tuple<std::string,std::string> -- simpler to build and split, but
// (fix round 2, review IMPORTANT correction) this reparameterization is NOT
// what fixed ctest's display/selection of this suite's custom test names.
// Fix round 1 misdiagnosed that symptom as a tuple-specific comma-parsing
// bug in this box's CMake GoogleTest module and "fixed" it by switching to a
// flat string -- which changed nothing observable, because the real cause is
// neither box-specific nor tuple-specific: CMake's own
// GoogleTestAddTests.cmake (write_test_to_file(), read directly rather than
// guessed a second time) DELIBERATELY substitutes GetParam()'s printed value
// for a value-parameterized test's name suffix in the name CTest displays
// and matches (`ctest -N`/`-R`) -- documented, intentional behaviour, for
// ANY printable parameter type, tuple or plain string alike (a flat string
// registers exactly as `.../"world|camera"`, no better than a tuple's
// `.../("world", "camera")`). The actual fix is `NO_PRETTY_VALUES` on this
// binary's one `gtest_discover_tests()` call -- see spade/tests/
// CMakeLists.txt's own comment there for the CMake source lines that prove
// it, and task-R9-report.md's fix-round-2 note for how the round 1 mistake
// was caught. This file keeps the flat-string parameterization anyway
// because it is simpler to read and split than a tuple, not because it
// fixes anything on its own.
//
// BARE GEOMETRY ONLY (ruling SR-2): worlds are loaded straight off disk via
// load_world_file() + scene_from_world() with an EMPTY resolved-mesh span --
// no package instances, no resolved prop meshes. T8 (Task C4 fix, sweeping a
// stale comment this same file's neighbourhood carried): this paragraph used
// to say "none of the ten shipped worlds authors props (all are schema v1,
// confirmed empty `props` on load)" -- both halves of that were already
// false BEFORE Task C4 touched a single world (world_version has been 2,
// with an explicit `props: []`, since Task H2), and the FIRST half is now
// false in substance too: Task C4 (SR-54) placed real props in 9 of the ten
// shipped worlds. The reason this matrix still does not need those props
// resolved is different, and stronger, than "there are none" -- it is
// structural: this file's own scene_or_fail() passes an EMPTY resolved-mesh
// span to every scene_from_world() call, so EVERY prop's mesh_ref misses
// that span's linear scan and resolves to kNoMesh (render/scene.cpp:340-352)
// on BOTH the raster and raymarch paths alike (raymarch never even looks at
// `scene.statics` -- raymarch.hpp's own "props are structurally invisible to
// the reference" comment, a SEPARATE and additional reason on that side).
// Both paths therefore agree about every prop by construction, contributing
// zero disagreement regardless of how many a world carries -- Task C4's own
// report (task-C4-report.md) states this plainly: the agreement matrix does
// not validate props at all; C5's gallery is where they get checked, by eye.
// Every SDF prefab INSTANCE Task C4 added, by contrast, is real scene.sdf
// geometry and DOES change what this matrix measures, which is why SR-2
// required a re-measure (see agreement_bands.json's versioned
// `measurements.2`) rather than reusing measurement 1's bare-geometry bands
// unchanged. No shipped world spawns a dynamic body in this harness either
// (update_dynamics() is never called), so `scene.dynamics` stays structurally
// invisible to the reference for the same raymarch.hpp reason, unaffected by
// any of this.
//
// THE FLAT-SKY REQUIREMENT (render/agreement.hpp's own header comment, load-
// bearing, restated here because this is the file that must actually honour
// it): every scene this file builds has its `lighting.sky_horizon` forced
// equal to `sky_zenith` immediately after scene_from_world() returns, BEFORE
// either render() call -- compare_silhouettes()'s single sky_reference_rgb
// parameter is only a correct "is this the sky" test against a flat sky, and
// both DrawMode::shaded and DrawMode::raymarch (INCLUDING the SR-30 bare-
// ground probe's own raymarch render) read `scene.lighting` from an
// already-flattened RenderScene, so all three agree on the flattened colour
// by construction, not by coincidence.
//
// STEP 1c CHECKS THE SHADED RANGE, NOT THE AUTHORED base_color (fix round 1,
// review IMPORTANT -- render/agreement.hpp's own header comment has the full
// account): assert_no_material_matches_sky() now takes the scene's Lighting
// and samples the material's ACHIEVABLE shaded colour (via the real
// shade_vertex_color()) across every achievable N.L, because the classifier
// sees the SHADED output, not the raw base_color, and a material can clear a
// base_color-only tolerance check yet still shade to the sky colour at some
// real surface orientation. Runs against every one of the ten worlds' OWN
// authored materials before either render() call, in every one of the 30
// cases -- not merely asserted once in isolation.
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
// SR-30's probe: re-renders the reference against a world with every SDF
// node deleted except the standalone ground-plane leaf
// (render::strip_to_ground_plane_only(), a PRODUCTION function -- Task C4
// will call this SAME one against dressed worlds, SR-2) and compares that
// against the REAL fast-path frame already rendered for this case. Reuses
// the ALREADY-FLATTENED Lighting from the real comparison verbatim (not a
// second, independently-flattened copy) so the probe's sky reference is
// bit-identical to the real one, never a second source of drift.
// ---------------------------------------------------------------------------

[[nodiscard]] AgreementResult bare_ground_probe(const WorldDesc& real_world, const RenderTarget& fast_target,
                                                 const Camera& camera, const Lighting& flattened_lighting,
                                                 uint32_t sky_ref) {
    const Result<WorldDesc> bare_world = strip_to_ground_plane_only(real_world);
    if (!bare_world) {
        ADD_FAILURE() << "strip_to_ground_plane_only failed: " << bare_world.error().context;
        return AgreementResult{};
    }
    RenderScene bare_scene = scene_or_fail(*bare_world);
    bare_scene.lighting = flattened_lighting;

    RenderOptions raymarch_options;
    raymarch_options.mode = DrawMode::raymarch;
    std::vector<uint8_t> bare_storage;
    RenderTarget bare_target = make_target(bare_storage, fast_target.width, fast_target.height);
    render_or_fail(bare_scene, camera, raymarch_options, bare_target);

    return compare_silhouettes(fast_target, bare_target, sky_ref);
}

// ---------------------------------------------------------------------------
// agreement_bands.json -- SR-31: a VERSIONED list of measurements, with
// `active_measurement_version` selecting the live one, and SR-30: bands are
// keyed per (world, camera), not per world. Loaded once, cached for the
// process lifetime.
// ---------------------------------------------------------------------------

[[nodiscard]] const Json& empty_json_object() {
    static const Json empty = Json::object();
    return empty;
}

[[nodiscard]] const Json& agreement_bands_root() {
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

// SR-31: selects `measurements.<active_measurement_version>` -- a re-measure
// adds a NEW entry under `measurements` and bumps the selector, rather than
// editing a committed measurement in place.
[[nodiscard]] const Json& active_measurement() {
    const Json& root = agreement_bands_root();
    const auto version_it = root.find("active_measurement_version");
    if (version_it == root.end()) {
        ADD_FAILURE() << "agreement_bands.json has no 'active_measurement_version'";
        return empty_json_object();
    }
    const auto measurements_it = root.find("measurements");
    if (measurements_it == root.end()) {
        ADD_FAILURE() << "agreement_bands.json has no 'measurements' object";
        return empty_json_object();
    }
    const std::string version_key = std::to_string(version_it->get<int>());
    const auto measurement_it = measurements_it->find(version_key);
    if (measurement_it == measurements_it->end()) {
        ADD_FAILURE() << "agreement_bands.json's active_measurement_version (" << version_key
                       << ") has no matching entry under 'measurements'";
        return empty_json_object();
    }
    return *measurement_it;
}

[[nodiscard]] const Json& camera_entry(const std::string& world_name, const std::string& camera_name) {
    const Json& measurement = active_measurement();
    const auto worlds_it = measurement.find("worlds");
    if (worlds_it == measurement.end()) {
        ADD_FAILURE() << "active measurement has no 'worlds' object";
        return empty_json_object();
    }
    const auto world_it = worlds_it->find(world_name);
    if (world_it == worlds_it->end()) {
        ADD_FAILURE() << "active measurement has no entry for world '" << world_name << "'";
        return empty_json_object();
    }
    const auto cameras_it = world_it->find("cameras");
    if (cameras_it == world_it->end()) {
        ADD_FAILURE() << "world '" << world_name << "' has no 'cameras' object";
        return empty_json_object();
    }
    const auto camera_it = cameras_it->find(camera_name);
    if (camera_it == cameras_it->end()) {
        ADD_FAILURE() << "world '" << world_name << "' has no pinned camera entry '" << camera_name << "'";
        return empty_json_object();
    }
    return *camera_it;
}

[[nodiscard]] double band_for(const std::string& world_name, const std::string& camera_name) {
    const Json& entry = camera_entry(world_name, camera_name);
    if (!entry.contains("band")) {
        ADD_FAILURE() << "'" << world_name << "'/" << camera_name << " has no pinned 'band'";
        return 0.0;
    }
    return entry.at("band").get<double>();
}

[[nodiscard]] bool recorded_detects_total_deletion(const std::string& world_name, const std::string& camera_name) {
    const Json& entry = camera_entry(world_name, camera_name);
    if (!entry.contains("detects_total_deletion")) {
        ADD_FAILURE() << "'" << world_name << "'/" << camera_name << " has no recorded 'detects_total_deletion'";
        return false;
    }
    return entry.at("detects_total_deletion").get<bool>();
}

// Controller fix round 1, I-4 (a review process gap, not this task's own
// bug -- the reviewer's own words: "I told you the suite 'recomputes and
// asserts against the file'. That is true only for detects_total_deletion."):
// band_for() and recorded_detects_total_deletion() are the only two fields
// this suite ever checked LIVE against agreement_bands.json -- nothing
// asserted that the recorded 'disagreement_fraction'/'covered_a'/
// 'covered_b' (real OR probe) actually equal what this run measures. A
// recorded disagreement_fraction 1.5x too high yields a band 1.5x too wide
// (band_for() reads it straight from the same entry EXPECT_LE checks
// against) and the suite stays green regardless -- the band derivation
// itself was never a checked claim, only its OWN internal arithmetic
// (margin_note) was. These four helpers make every recorded number in a
// measurement a checked claim, the same sense detects_total_deletion
// already was, for both the real comparison and the SR-30 probe.
[[nodiscard]] double recorded_double(const std::string& world_name, const std::string& camera_name,
                                      const std::string& field) {
    const Json& entry = camera_entry(world_name, camera_name);
    if (!entry.contains(field)) {
        ADD_FAILURE() << "'" << world_name << "'/" << camera_name << " has no recorded '" << field << "'";
        return 0.0;
    }
    return entry.at(field).get<double>();
}

[[nodiscard]] uint32_t recorded_uint(const std::string& world_name, const std::string& camera_name,
                                      const std::string& field) {
    const Json& entry = camera_entry(world_name, camera_name);
    if (!entry.contains(field)) {
        ADD_FAILURE() << "'" << world_name << "'/" << camera_name << " has no recorded '" << field << "'";
        return 0;
    }
    return entry.at(field).get<uint32_t>();
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

// Flat "world|camera" keys, NOT a std::tuple<std::string,std::string> (fix
// round 1 -- this file's own header comment has the measured CMake/CTest
// parsing symptom the tuple form triggered).
[[nodiscard]] std::vector<std::string> matrix_case_keys() {
    std::vector<std::string> keys;
    keys.reserve(kShippedWorldNames.size() * kBookmarkNames.size());
    for (const std::string& world_name : kShippedWorldNames) {
        for (const std::string& camera_name : kBookmarkNames) {
            keys.push_back(world_name + "|" + camera_name);
        }
    }
    return keys;
}

[[nodiscard]] std::pair<std::string, std::string> split_case_key(const std::string& key) {
    const std::size_t sep = key.find('|');
    return {key.substr(0, sep), key.substr(sep + 1)};
}

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
//    THIS GUARD EXERCISES THE METRIC AGAINST PURE SKY, NOT THE MATRIX'S OWN
//    REGIME (review, fix round 1): a box against sky is exactly the
//    configuration where coverage discriminates -- none of the 30 matrix
//    cases look like this, since every shipped world saturates most or all
//    of the frame with its own infinite ground. This guard is still correct
//    and still required (it proves the METRIC can register a real
//    difference at all), but it does NOT, on its own, prove any given
//    MATRIX CASE can -- that is what section 3's SR-30 probe below is for.
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
    const Result<void> guard = assert_no_material_matches_sky(world.materials, scene.lighting, sky_ref);
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
// 3. The matrix: ten shipped worlds x three camera bookmarks each, at
//    160x120, DrawMode::shaded vs DrawMode::raymarch -- see this file's own
//    header comment for the bare-geometry/flat-sky/Step-1c posture every
//    case below follows, and SR-30 for the bare-ground detection-surface
//    probe every case now runs and checks LIVE.
// ===========================================================================

class AgreementMatrix : public ::testing::TestWithParam<std::string> {};

TEST_P(AgreementMatrix, MeasuredDisagreementIsWithinItsPinnedBandAndDetectionSurfaceMatchesTheRecordedClaim) {
    const auto [world_name, camera_name] = split_case_key(GetParam());

    const WorldDesc world = load_shipped_world(world_name);
    ASSERT_FALSE(world.sdf.nodes.empty()) << "sanity: '" << world_name << "' must have real SDF geometry";

    const Json scene_json = load_shipped_scene_json(world_name);
    const Camera camera = camera_from_scene_bookmark(scene_json, camera_name);

    RenderScene scene = scene_or_fail(world);
    const uint32_t sky_ref = flatten_sky_and_pack_reference(scene);

    // Step 1c, run for real against this world's OWN authored materials --
    // fatal, because a collision here means every downstream pixel count is
    // meaningless, not merely off.
    const Result<void> guard = assert_no_material_matches_sky(world.materials, scene.lighting, sky_ref);
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

    // I-4 (controller fix round 1): the recorded 'disagreement_fraction',
    // 'covered_a', and 'covered_b' are now themselves checked claims, not
    // merely the band the test derives from them -- see recorded_double()/
    // recorded_uint()'s own header comment for why this was missing and
    // what it would have let slip through unnoticed (a recorded
    // disagreement_fraction inflated relative to what this run actually
    // measures, silently widening this case's own band via band_for()
    // reading the SAME inflated number).
    EXPECT_NEAR(result.disagreement_fraction, recorded_double(world_name, camera_name, "disagreement_fraction"),
                1e-9)
        << "'" << world_name << "'/" << camera_name
        << "': live disagreement_fraction does not match agreement_bands.json's recorded value -- "
           "the recorded number is no longer a faithful measurement of this world/camera";
    EXPECT_EQ(result.covered_a, recorded_uint(world_name, camera_name, "covered_a"))
        << "'" << world_name << "'/" << camera_name << "': live covered_a does not match the recorded value";
    EXPECT_EQ(result.covered_b, recorded_uint(world_name, camera_name, "covered_b"))
        << "'" << world_name << "'/" << camera_name << "': live covered_b does not match the recorded value";

    const double band = band_for(world_name, camera_name);
    EXPECT_LE(result.disagreement_fraction, band)
        << "'" << world_name << "'/" << camera_name << " disagreement_fraction "
        << (result.disagreement_fraction * 100.0) << "% exceeds its pinned band " << (band * 100.0)
        << "% (agreement_bands.json)";

    // SR-30, checked LIVE, not narrated: re-render the reference with every
    // node except the ground plane deleted, and confirm the band's own
    // power to detect that (or documented lack of it) matches what
    // agreement_bands.json claims.
    const AgreementResult probe = bare_ground_probe(world, fast_target, camera, scene.lighting, sky_ref);
    std::cout << "[AgreementMatrix] " << world_name << "/" << camera_name
              << " SR-30 PROBE (bare ground): disagreement_fraction=" << probe.disagreement_fraction
              << " covered_a=" << probe.covered_a << " covered_b=" << probe.covered_b << "\n";

    const bool live_detects = probe.disagreement_fraction > band;
    const bool recorded_detects = recorded_detects_total_deletion(world_name, camera_name);

    // THE LINE fix round 2 adds (review: "PARTIALLY ADDRESSED -- the honesty
    // is in the data file and absent from the test surface"): every case
    // prints its own verdict, not just its raw numbers, so a human scanning
    // 30 green ctest lines (or CK-2) sees which ones constrain real scene
    // geometry without having to cross-reference agreement_bands.json at
    // all. Gated on `recorded_detects` (the committed claim just verified
    // above), not `live_detects` -- the printed verdict is a restatement of
    // what this run is CHECKING, the same source EXPECT_EQ below reads.
    std::cout << "[AgreementMatrix] " << world_name << "/" << camera_name << " band=" << (band * 100.0) << "% "
              << (recorded_detects
                      ? "VERDICT: DISCRIMINATING -- this case's band would catch its reference's entire scene "
                        "geometry being deleted.\n"
                      : "VERDICT: NON-DISCRIMINATING -- this case's band constrains NO scene geometry today (see "
                        "agreement_bands.json's non_discriminating_summary/worlds_with_no_discriminating_camera).\n");

    EXPECT_EQ(live_detects, recorded_detects)
        << "'" << world_name << "'/" << camera_name << "': live SR-30 probe "
        << (live_detects ? "DETECTS" : "does NOT detect") << " total geometry deletion (probe disagreement "
        << (probe.disagreement_fraction * 100.0) << "% vs band " << (band * 100.0)
        << "%), but agreement_bands.json's 'detects_total_deletion' claims "
        << (recorded_detects ? "true" : "false")
        << " -- SR-30 requires this claim be checked every run, not recorded once and trusted";
}

INSTANTIATE_TEST_SUITE_P(
    ShippedWorlds, AgreementMatrix, ::testing::ValuesIn(matrix_case_keys()),
    [](const ::testing::TestParamInfo<std::string>& info) {
        std::string name = info.param;
        for (char& c : name) {
            if (c == '-' || c == '|') c = '_';
        }
        return name;
    });
