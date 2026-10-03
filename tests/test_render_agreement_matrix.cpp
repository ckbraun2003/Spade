// AgreementMatrix: raster-vs-ray-march agreement bands on worlds this
// repository owns (RS4), from tests/golden/render/agreement_bands.json
// (docs/design/rendering/plans/2026-10-03-agreement-bands-plan.md).
//
// Each case is a golden world and a camera from the band file's active
// version. It renders the world with the CPU raster and the CPU ray-march
// over a flat sky, compares coverage only, and asserts the disagreement is
// within the pinned band. Then it re-runs the SR-30 probe live and asserts
// the band would catch the deletion of every object but the ground: a band
// that cannot fail is not a guard.
//
// Every run prints one line per case, "agreement: <world>/<camera>
// toolchain=<t> d=<v> d_probe=<v>", at full precision, so another toolchain's
// d can be compared bit for bit from its ctest log. With
// SPADE_AGREEMENT_MEASURE set, each case also writes its measured values as
// JSON, and its raster, ray-march and probe frames as PPMs, under
// <build>/tests/test-output/agreement/, for pinning and reviewing a new
// version. Neither changes a result: the band is asserted either way.

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <span>
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

namespace {

using Json = nlohmann::json;
using spade::Result;
using spade::WorldDesc;
using spade::render::AgreementResult;
using spade::render::Camera;
using spade::render::DrawMode;
using spade::render::PixelFormat;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

// Every member has a default initializer, so a designated initializer may
// omit any of them: gcc's -Wmissing-field-initializers flags an omitted
// member that has none, and the Docker leg builds with -Werror.
struct MatrixCase {
    std::string world{};
    std::string camera_name{};
    glm::vec3 position{0.0f};
    float yaw = 0.0f;
    float pitch = 0.0f;
    std::string probe{};   // "ground_only" or "empty"
    Json band{};           // a number once pinned, null before
    std::string error{};   // set when the band file could not be read: the case fails with it
};

[[nodiscard]] std::filesystem::path bands_path() {
    return std::filesystem::path(SPADE_GOLDEN_DIR) / "render" / "agreement_bands.json";
}

// Read at registration. A file that cannot be read becomes one case that
// fails with the reason, never zero cases that pass by absence.
[[nodiscard]] std::vector<MatrixCase> active_cases() {
    try {
        std::ifstream in(bands_path());
        const Json root = Json::parse(in);
        const std::string version = root.at("active_measurement_version").get<std::string>();
        std::vector<MatrixCase> cases;
        for (const Json& c : root.at("measurements").at(version).at("cases")) {
            const Json& cam = c.at("camera");
            const Json& pos = cam.at("position");
            cases.push_back(MatrixCase{
                .world = c.at("world").get<std::string>(),
                .camera_name = cam.at("name").get<std::string>(),
                .position = glm::vec3(pos.at(0).get<float>(), pos.at(1).get<float>(), pos.at(2).get<float>()),
                .yaw = cam.at("yaw_radians").get<float>(),
                .pitch = cam.at("pitch_radians").get<float>(),
                .probe = c.at("probe").get<std::string>(),
                .band = c.at("band"),
            });
        }
        if (cases.empty()) return {MatrixCase{.world = "unreadable", .error = "the active version has no cases"}};
        return cases;
    } catch (const std::exception& e) {
        return {MatrixCase{.world = "unreadable", .error = bands_path().string() + ": " + e.what()}};
    }
}

// The band file's camera convention: yaw about world +Y, then pitch about
// the camera's own X, through sin32/cos32 so every toolchain frames alike.
[[nodiscard]] Camera camera_of(const MatrixCase& c) {
    const glm::quat yaw(spade::math::cos32(0.5f * c.yaw), 0.0f, spade::math::sin32(0.5f * c.yaw), 0.0f);
    const glm::quat pitch(spade::math::cos32(0.5f * c.pitch), spade::math::sin32(0.5f * c.pitch), 0.0f, 0.0f);
    Camera camera;
    camera.position = c.position;
    camera.orientation = yaw * pitch;
    return camera;
}

constexpr uint32_t kWidth = 160, kHeight = 120;

struct Frame {
    std::vector<uint8_t> storage;
    RenderTarget target;
};

[[nodiscard]] Frame render_frame(const RenderScene& scene, const Camera& camera, DrawMode mode) {
    Frame f;
    f.storage.assign(static_cast<size_t>(kWidth) * kHeight * 4u, 0u);
    f.target = RenderTarget{.pixels = std::span<uint8_t>(f.storage), .width = kWidth, .height = kHeight,
                            .stride = kWidth * 4u, .format = PixelFormat::bgrx8};
    RenderOptions options;
    options.mode = mode;
    options.overlays = false;
    options.shadows = false;
    if (auto drew = spade::render::render(scene, camera, options, f.target); !drew) {
        ADD_FAILURE() << "render() failed: " << drew.error().context;
    }
    return f;
}

// The band rule signed off under TD-2: round_up(0.001, max(0.001, 1.5 d)).
// The epsilon keeps an exact multiple (1.5 x 0.016666... = 0.025) from
// rounding up a whole step.
[[nodiscard]] double band_rule(double d) {
    const double thousandths = std::ceil(1.5 * d * 1000.0 - 1e-9);
    return std::max(1.0, thousandths) / 1000.0;
}

[[nodiscard]] std::string toolchain() {
#if defined(_MSC_VER)
    std::string name = "msvc";
#elif defined(__GNUC__)
    std::string name = "gcc";
#else
    std::string name = "other";
#endif
#if defined(NDEBUG)
    return name + "-release";
#else
    return name + "-debug";
#endif
}

// A BGRX frame as a binary PPM, so a framing can be looked at, not only counted.
void write_ppm(const std::filesystem::path& path, const RenderTarget& frame) {
    std::ofstream out(path, std::ios::binary);
    out << "P6\n" << frame.width << ' ' << frame.height << "\n255\n";
    for (size_t i = 0; i + 3u < frame.pixels.size(); i += 4u) {
        const char rgb[3] = {static_cast<char>(frame.pixels[i + 2u]), static_cast<char>(frame.pixels[i + 1u]),
                             static_cast<char>(frame.pixels[i])};
        out.write(rgb, 3);
    }
    if (!out) std::fprintf(stderr, "SPADE_AGREEMENT_MEASURE: could not write %s\n", path.string().c_str());
}

void write_measurement_if_asked(const MatrixCase& c, const AgreementResult& real, const AgreementResult& probe,
                                const RenderTarget& raster, const RenderTarget& raymarch,
                                const RenderTarget& probe_raymarch) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)  // read-only check, as env_gate_is_set() in test_fp32_math.cpp
#endif
    const bool asked = std::getenv("SPADE_AGREEMENT_MEASURE") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    if (!asked) return;
    const double band = band_rule(real.disagreement_fraction);
    const Json out = {
        {"world", c.world},
        {"camera", c.camera_name},
        {"toolchain", toolchain()},
        {"d", real.disagreement_fraction},
        {"covered_raster", real.covered_a},
        {"covered_raymarch", real.covered_b},
        {"d_probe", probe.disagreement_fraction},
        {"probe", c.probe},
        {"band_by_rule", band},
        {"probe_over_band", probe.disagreement_fraction / band},
        {"detects_total_deletion", probe.disagreement_fraction > band},
    };
    const std::filesystem::path dir = std::filesystem::path(SPADE_TEST_OUTPUT_DIR) / "agreement";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::filesystem::path path = dir / (c.world + "." + c.camera_name + "." + toolchain() + ".json");
    std::ofstream file(path);
    file << out.dump(2) << "\n";
    if (ec || !file) {
        std::fprintf(stderr, "SPADE_AGREEMENT_MEASURE: could not write %s\n", path.string().c_str());
    }
    const std::string stem = c.world + "." + c.camera_name + ".";
    write_ppm(dir / (stem + "raster.ppm"), raster);
    write_ppm(dir / (stem + "raymarch.ppm"), raymarch);
    write_ppm(dir / (stem + "probe.ppm"), probe_raymarch);
}

class AgreementMatrix : public ::testing::TestWithParam<MatrixCase> {};

TEST_P(AgreementMatrix, RasterAgreesWithTheRayMarchWithinItsPinnedBand) {
    const MatrixCase& c = GetParam();
    ASSERT_TRUE(c.error.empty()) << c.error;

    const std::filesystem::path world_path =
        std::filesystem::path(SPADE_GOLDEN_DIR) / "worlds" / (c.world + ".world.yaml");
    const Result<WorldDesc> world = spade::load_world_file(world_path);
    ASSERT_TRUE(world) << world_path.string() << ": " << world.error().context;
    Result<RenderScene> scene = spade::render::scene_from_world(*world, {});
    ASSERT_TRUE(scene) << scene.error().context;

    // Flat sky first (agreement.hpp's precondition), then Step 1c.
    scene->lighting.sky_horizon = scene->lighting.sky_zenith;
    const uint32_t sky = spade::render::pack_sky_reference_bgrx(scene->lighting);
    const Result<void> step1c = spade::render::assert_no_material_matches_sky(world->materials, scene->lighting, sky);
    ASSERT_TRUE(step1c) << "Step 1c: " << step1c.error().context;

    const Camera camera = camera_of(c);
    const Frame raster = render_frame(*scene, camera, DrawMode::shaded);
    const Frame raymarch = render_frame(*scene, camera, DrawMode::raymarch);
    const AgreementResult real = spade::render::compare_silhouettes(raster.target, raymarch.target, sky);

    // SR-30's probe: the band file names it, and it must match what the
    // renderer found, so a world gaining or losing its ground is noticed.
    const bool has_ground = !scene->ground_planes.empty();
    ASSERT_EQ(c.probe, has_ground ? "ground_only" : "empty")
        << c.world << ": the band file's probe does not match whether the renderer found a ground plane";
    WorldDesc probe_world = *world;
    if (has_ground) {
        Result<WorldDesc> stripped = spade::render::strip_to_ground_plane_only(*world);
        ASSERT_TRUE(stripped) << stripped.error().context;
        probe_world = std::move(*stripped);
    } else {
        probe_world.sdf.nodes.clear();
        probe_world.sdf.node_materials.clear();
    }
    Result<RenderScene> probe_scene = spade::render::scene_from_world(probe_world, {});
    ASSERT_TRUE(probe_scene) << probe_scene.error().context;
    probe_scene->lighting.sky_horizon = probe_scene->lighting.sky_zenith;
    const Frame probe_raymarch = render_frame(*probe_scene, camera, DrawMode::raymarch);
    const AgreementResult probe = spade::render::compare_silhouettes(raster.target, probe_raymarch.target, sky);

    std::printf("agreement: %s/%s toolchain=%s d=%.17g d_probe=%.17g\n", c.world.c_str(), c.camera_name.c_str(),
                toolchain().c_str(), real.disagreement_fraction, probe.disagreement_fraction);
    write_measurement_if_asked(c, real, probe, raster.target, raymarch.target, probe_raymarch.target);
    ASSERT_GT(real.covered_a, 0u) << "the raster must see the world from this camera";
    ASSERT_GT(real.covered_b, 0u) << "the ray-march must see the world from this camera";

    ASSERT_TRUE(c.band.is_number())
        << c.world << "/" << c.camera_name << " has no pinned band; measure it with SPADE_AGREEMENT_MEASURE "
        << "(measured d = " << real.disagreement_fraction << ", d_probe = " << probe.disagreement_fraction << ")";
    const double band = c.band.get<double>();
    EXPECT_LE(real.disagreement_fraction, band)
        << c.world << "/" << c.camera_name << ": raster and ray-march disagree on "
        << real.disagreement_fraction * 100.0 << "% of pixels, above the pinned band " << band * 100.0 << "%";
    EXPECT_GT(probe.disagreement_fraction, band)
        << c.world << "/" << c.camera_name << ": the band would not catch the deletion of every object but the "
        << "ground (d_probe " << probe.disagreement_fraction << "), so it guards nothing (SR-30)";
}

INSTANTIATE_TEST_SUITE_P(Active, AgreementMatrix, ::testing::ValuesIn(active_cases()),
                         [](const ::testing::TestParamInfo<MatrixCase>& info) {
                             std::string name = info.param.world + "_" + info.param.camera_name;
                             std::replace_if(
                                 name.begin(), name.end(), [](char ch) { return !std::isalnum(static_cast<unsigned char>(ch)); },
                                 '_');
                             return name;
                         });

}  // namespace
