// test_path_text.cpp -- every error message that names a file names it in
// UTF-8, and no file loader throws while saying so. Kat's host treats every
// path across its C interfaces as UTF-8. On Windows path::string() converts to
// the ANSI code page instead, which garbles a path with é in it and THROWS for
// one with 中 in it -- out of functions that return Result.

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <string_view>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "render/gltf.hpp"
#include "scene/compose.hpp"
#include "scene/scene_file.hpp"
#include "state/snapshot.hpp"
#include "testing/scenario_file.hpp"
#include "world/world_file.hpp"

namespace {

// "spade_path_é_中" as UTF-8, spelled in bytes so the test does not depend on
// the source file's encoding.
constexpr std::string_view kDirUtf8 = "spade_path_\xC3\xA9_\xE4\xB8\xAD";

[[nodiscard]] std::filesystem::path from_utf8(std::string_view text) {
    return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(text.data()), text.size()));
}

// A directory whose name is outside every ANSI code page, made fresh.
[[nodiscard]] std::filesystem::path unicode_dir() {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / from_utf8(kDirUtf8);
    std::filesystem::create_directories(dir);
    return dir;
}

}  // namespace

TEST(PathText, IsUtf8AndNeverThrows) {
    const std::filesystem::path path = std::filesystem::path("root") / from_utf8(kDirUtf8) / "file.yaml";
    const std::string text = spade::path_text(path);
    EXPECT_NE(text.find(kDirUtf8), std::string::npos) << text;
}

// Each loader, given a missing file under that directory, refuses with
// io_error and names the path in UTF-8. A throw fails the test.
TEST(PathText, EveryFileLoaderNamesAMissingFileInUtf8) {
    const std::filesystem::path dir = unicode_dir();
    const auto check = [](const char* loader, const spade::Error& error) {
        EXPECT_EQ(error.code, spade::Code::io_error) << loader << ": " << error.context;
        EXPECT_NE(error.context.find(kDirUtf8), std::string::npos)
            << loader << " does not name the path in UTF-8: " << error.context;
    };
    try {
        const auto gltf = spade::render::load_gltf(dir / "missing.gltf");
        ASSERT_FALSE(gltf.has_value());
        check("render::load_gltf", gltf.error());
    } catch (const std::exception& e) {
        ADD_FAILURE() << "render::load_gltf threw: " << e.what();
    }
    try {
        const auto world = spade::load_world_file(dir / "missing.world.yaml");
        ASSERT_FALSE(world.has_value());
        check("load_world_file", world.error());
    } catch (const std::exception& e) {
        ADD_FAILURE() << "load_world_file threw: " << e.what();
    }
    try {
        const auto scene = spade::scene::load_scene_file(dir / "missing.scene.yaml");
        ASSERT_FALSE(scene.has_value());
        check("scene::load_scene_file", scene.error());
    } catch (const std::exception& e) {
        ADD_FAILURE() << "scene::load_scene_file threw: " << e.what();
    }
    try {
        const auto composed = spade::scene::compose_file(dir / "missing.scene.yaml");
        ASSERT_FALSE(composed.has_value());
        check("scene::compose_file", composed.error());
    } catch (const std::exception& e) {
        ADD_FAILURE() << "scene::compose_file threw: " << e.what();
    }
    try {
        const auto blob = spade::SnapshotBlob::read_file(dir / "missing.snapshot");
        ASSERT_FALSE(blob.has_value());
        check("SnapshotBlob::read_file", blob.error());
    } catch (const std::exception& e) {
        ADD_FAILURE() << "SnapshotBlob::read_file threw: " << e.what();
    }
    try {
        const auto scenario = spade::testing::scenario_from_yaml(dir / "missing.scenario.yaml");
        ASSERT_FALSE(scenario.has_value());
        check("testing::scenario_from_yaml", scenario.error());
    } catch (const std::exception& e) {
        ADD_FAILURE() << "testing::scenario_from_yaml threw: " << e.what();
    }
}
