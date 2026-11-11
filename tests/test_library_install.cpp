// test_library_install.cpp -- the installed library tree (L50, Kat's ask). `cmake --install --component spade_library` puts everything under
// assets/library, and nothing else from assets/, at <prefix>/share/spade/library, so a host that installs Spade loads its drones by path
// from there (crazyflie_class, fpv_5inch, phantom3_class). The tests install into a temp prefix of their own, from this build tree, and
// check the result: the same files byte for byte, nothing else, and a tree that loads (library.yaml, its contents, and every drone's
// model with its sources sidecar passing check_model_sources()). No engine code moves; no step changes.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#include "scene/library.hpp"
#include "scene/library_sources.hpp"
#include "scene/model_sources.hpp"

// Both are defined for this file alone by tests/CMakeLists.txt (set_source_files_properties): the build tree to install from and the cmake
// that configured it. gcc-check's compile of a new test file lacks them; the tests then fail by name rather than guess.
#ifndef SPADE_TEST_BUILD_DIR
#define SPADE_TEST_BUILD_DIR ""
#endif
#ifndef SPADE_TEST_CMAKE_COMMAND
#define SPADE_TEST_CMAKE_COMMAND ""
#endif

namespace {

namespace fs = std::filesystem;

[[nodiscard]] std::string read_bytes(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

[[nodiscard]] std::vector<std::string> relative_files(const fs::path& root) {
    std::vector<std::string> out;
    for (const fs::directory_entry& e : fs::recursive_directory_iterator(root)) {
        if (e.is_regular_file()) out.push_back(fs::relative(e.path(), root).generic_string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

// One test, one prefix: ctest runs each TEST() in its own process, in parallel, so a shared prefix would race.
class LibraryInstall : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_FALSE(std::string(SPADE_TEST_BUILD_DIR).empty()) << "SPADE_TEST_BUILD_DIR is not defined for this file (tests/CMakeLists.txt)";
        prefix_ = fs::path(SPADE_TEST_BUILD_DIR) / "library-install-test" / ::testing::UnitTest::GetInstance()->current_test_info()->name();
        std::error_code ec;
        fs::remove_all(prefix_, ec);
        fs::create_directories(prefix_);
        const fs::path log = prefix_.string() + ".log";
        std::string command = std::string("\"") + SPADE_TEST_CMAKE_COMMAND + "\" --install \"" + SPADE_TEST_BUILD_DIR + "\" --prefix \"" +
                              prefix_.generic_string() + "\" --component spade_library > \"" + log.generic_string() + "\" 2>&1";
#ifdef _WIN32
        command = "\"" + command + "\"";  // cmd /c strips one outer pair of quotes
#endif
        ASSERT_EQ(std::system(command.c_str()), 0) << "cmake --install failed: " << read_bytes(log);
        library_ = prefix_ / "share" / "spade" / "library";
    }
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(prefix_, ec);
        fs::remove(prefix_.string() + ".log", ec);
    }

    fs::path prefix_;
    fs::path library_;
};

}  // namespace

// Every file under assets/library is installed under share/spade/library, byte for byte, and no other file is there.
TEST_F(LibraryInstall, EveryFileUnderAssetsLibraryIsInstalledByteForByteAndNothingElse) {
    const fs::path source = fs::path("assets") / "library";
    ASSERT_TRUE(fs::is_directory(source)) << "run from the source root";
    ASSERT_TRUE(fs::is_directory(library_)) << "no " << library_.generic_string() << ": the install rule is missing";
    const std::vector<std::string> want = relative_files(source);
    const std::vector<std::string> have = relative_files(library_);
    EXPECT_EQ(have, want) << "the installed file list is not assets/library's";
    for (const std::string& rel : want) {
        const fs::path installed = library_ / fs::path(rel);
        if (!fs::is_regular_file(installed)) continue;  // the list comparison above already named it
        EXPECT_EQ(read_bytes(installed), read_bytes(source / fs::path(rel))) << rel << " differs from the source";
    }
}

// Nothing else from assets/ comes with it: the prefix holds share/spade/library and nothing beside it.
TEST_F(LibraryInstall, NothingElseFromAssetsIsInstalled) {
    std::set<std::string> top;
    for (const fs::directory_entry& e : fs::directory_iterator(prefix_)) top.insert(e.path().filename().generic_string());
    EXPECT_EQ(top, (std::set<std::string>{"share"}));
    std::set<std::string> share;
    if (fs::is_directory(prefix_ / "share")) {
        for (const fs::directory_entry& e : fs::directory_iterator(prefix_ / "share")) share.insert(e.path().filename().generic_string());
    }
    EXPECT_EQ(share, (std::set<std::string>{"spade"}));
    std::set<std::string> spade;
    if (fs::is_directory(prefix_ / "share" / "spade")) {
        for (const fs::directory_entry& e : fs::directory_iterator(prefix_ / "share" / "spade")) spade.insert(e.path().filename().generic_string());
    }
    EXPECT_EQ(spade, (std::set<std::string>{"library"}));
}

// The installed tree loads: the index and its contents, every drone's model by its listed path, and every drone's sources sidecar
// passes check_model_sources() against the installed registry. The three drones Kat's install-host loads by path are among them.
TEST_F(LibraryInstall, TheInstalledTreeLoadsAndEveryDronesSourcesCheck) {
    namespace sc = spade::scene;
    const spade::Result<sc::Library> library = sc::load_library(library_ / "library.yaml");
    ASSERT_TRUE(library.has_value()) << library.error().context;
    const auto contents = sc::check_library_contents(*library);
    EXPECT_TRUE(contents.has_value()) << (contents ? "" : contents.error().context);
    const spade::Result<sc::SourceRegistry> registry = sc::load_library_sources(*library);
    ASSERT_TRUE(registry.has_value()) << registry.error().context;

    std::set<std::string> drones;
    for (const sc::LibraryEntry& entry : library->entries) {
        if (entry.kind != sc::LibraryKind::model || !entry.file.starts_with("drones/")) continue;
        drones.insert(entry.file);
        const auto model = sc::load_library_model(*library, entry);
        EXPECT_TRUE(model.has_value()) << entry.file << ": " << (model ? "" : model.error().context);
        const std::string stem = entry.file.substr(0, entry.file.size() - std::string(".model.yaml").size());
        const auto sources = sc::load_model_sources(library->dir / fs::path(stem + ".sources.yaml"));
        ASSERT_TRUE(sources.has_value()) << entry.file << ": " << sources.error().context;
        const auto checked = sc::check_model_sources(*library, *registry, *sources);
        EXPECT_TRUE(checked.has_value()) << entry.file << ": " << (checked ? "" : checked.error().context);
    }
    for (const char* by_path : {"drones/crazyflie_class.model.yaml", "drones/fpv_5inch.model.yaml", "drones/phantom3_class.model.yaml"}) {
        EXPECT_TRUE(drones.contains(by_path)) << by_path << " is not a listed drone of the installed library";
    }
}
