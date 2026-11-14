// L73a: where the app finds its assets, and what a bare launch opens (sandbox/app_paths.hpp). A tester starts the
// editor from the Start menu, where the working directory is not the install folder.

#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>

#include "../sandbox/app_paths.hpp"

namespace {

namespace app = spade::sandbox::app;
namespace fs = std::filesystem;

// A folder of its own, removed after.
struct Dir {
    fs::path path;
    Dir() {
        static std::atomic<int> n{0};
        path = fs::temp_directory_path() / ("spade_app_paths_" + std::to_string(++n) + "_" +
                                            std::to_string(static_cast<long long>(fs::file_time_type::clock::now()
                                                                                      .time_since_epoch()
                                                                                      .count())));
        fs::create_directories(path);
    }
    ~Dir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    Dir(const Dir&) = delete;
    Dir& operator=(const Dir&) = delete;

    // <path>/<where>/scenes and /worlds: an assets root.
    fs::path assets(const std::string& where) const {
        const fs::path a = path / where;
        fs::create_directories(a / "scenes");
        fs::create_directories(a / "worlds");
        return a;
    }
};

TEST(AppPaths, AFolderWithScenesAndWorldsIsAnAssetsRootAndOneWithoutIsNot) {
    Dir d;
    EXPECT_TRUE(app::is_assets_root(d.assets("assets")));
    fs::create_directories(d.path / "half" / "scenes");
    EXPECT_FALSE(app::is_assets_root(d.path / "half")) << "scenes without worlds";
    EXPECT_FALSE(app::is_assets_root(d.path / "nowhere"));
}

// The install layout: assets/ beside the exe, and a working directory that is somewhere else entirely.
TEST(AppPaths, TheAssetsBesideTheExeAreFoundWhateverTheWorkingDirectory) {
    Dir d;
    const fs::path installed = d.assets("install/assets");
    fs::create_directories(d.path / "elsewhere");
    EXPECT_EQ(app::resolve_assets({}, d.path / "install", d.path / "elsewhere"), installed);
}

// A dev build has nothing beside the exe: the working directory's assets/ (the repository root) is used, so a dev run
// and the live smoke work as before.
TEST(AppPaths, WithNoAssetsBesideTheExeTheWorkingDirectorysAreUsed) {
    Dir d;
    fs::create_directories(d.path / "build" / "bin");
    const fs::path repo = d.assets("repo/assets");
    EXPECT_EQ(app::resolve_assets({}, d.path / "build" / "bin", d.path / "repo"), repo);
}

// The one beside the exe wins when both exist (an installed copy run from a folder that has its own assets/).
TEST(AppPaths, TheAssetsBesideTheExeWinOverTheWorkingDirectorys) {
    Dir d;
    const fs::path installed = d.assets("install/assets");
    d.assets("repo/assets");
    EXPECT_EQ(app::resolve_assets({}, d.path / "install", d.path / "repo"), installed);
}

// An assets/ beside the exe that is not an assets root (a stray folder) is not taken: it would give a window that
// cannot find a single scene.
TEST(AppPaths, AStrayAssetsFolderBesideTheExeIsNotTaken) {
    Dir d;
    fs::create_directories(d.path / "install" / "assets" / "fonts");
    const fs::path repo = d.assets("repo/assets");
    EXPECT_EQ(app::resolve_assets({}, d.path / "install", d.path / "repo"), repo);
}

// --assets overrides both, and is returned as given, even when it is no assets root, so the refusal that follows names
// the folder the person asked for rather than quietly using another.
TEST(AppPaths, AssetsOverridesBothAndIsNeverReplaced) {
    Dir d;
    d.assets("install/assets");
    d.assets("repo/assets");
    const fs::path mine = d.assets("mine");
    EXPECT_EQ(app::resolve_assets(mine, d.path / "install", d.path / "repo"), mine);
    const fs::path wrong = d.path / "not-assets";
    EXPECT_EQ(app::resolve_assets(wrong, d.path / "install", d.path / "repo"), wrong);
}

// Nothing found anywhere: the working directory's assets/, so the message says where it looked.
TEST(AppPaths, WithNoAssetsAnywhereTheWorkingDirectorysIsNamed) {
    Dir d;
    EXPECT_EQ(app::resolve_assets({}, d.path / "install", d.path / "repo"), d.path / "repo" / "assets");
}

TEST(AppPaths, TheDefaultSceneIsTheHoverSceneUnderTheAssets) {
    EXPECT_EQ(app::default_scene(fs::path("A") / "assets"), fs::path("A") / "assets" / "scenes" / "hover.scene.yaml");
}

// The font: where a build puts it (beside the exe) is used when it is there; the installer puts it under assets/.
TEST(AppPaths, TheFontsComeFromBesideTheExeThenFromTheAssets) {
    Dir d;
    const fs::path exe = d.path / "bin";
    const fs::path assets = d.assets("assets");
    fs::create_directories(assets / "fonts");
    std::ofstream(assets / "fonts" / "Inter-Regular.ttf") << "x";
    EXPECT_EQ(app::fonts_dir(exe, assets), assets / "fonts") << "nothing beside the exe";
    fs::create_directories(exe / "fonts");
    std::ofstream(exe / "fonts" / "Inter-Regular.ttf") << "x";
    EXPECT_EQ(app::fonts_dir(exe, assets), exe / "fonts") << "a build's own";
}

TEST(AppPaths, TheUserGuideIsTheIndexOfTheDocsFolderBesideTheExe) {
    EXPECT_EQ(app::user_guide_file(fs::path("X") / "Spade"),
              fs::path("X") / "Spade" / "docs" / "user-guide" / "index.html");
}

// THE BARE LAUNCH IS THE EDITOR (L73a). The drone sim box is --window or --scene drone; --headless and --live-smoke
// are their own; nothing else changes.
TEST(AppPaths, ABareLaunchOpensTheEditorAndTheDroneBoxIsAskedFor) {
    using app::Mode;
    //                              headless window scene  smoke
    EXPECT_EQ(app::default_mode(false, false, false, false), Mode::editor) << "no arguments";
    EXPECT_EQ(app::default_mode(false, true, false, false), Mode::drone_box) << "--window";
    EXPECT_EQ(app::default_mode(false, false, true, false), Mode::drone_box) << "--scene drone";
    EXPECT_EQ(app::default_mode(true, false, false, false), Mode::headless) << "--headless";
    EXPECT_EQ(app::default_mode(true, false, true, false), Mode::headless) << "--headless --scene drone";
    EXPECT_EQ(app::default_mode(false, false, false, true), Mode::live_smoke) << "--live-smoke";
}

}  // namespace
