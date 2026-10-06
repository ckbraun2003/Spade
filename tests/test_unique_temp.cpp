// Temporary paths that cannot collide (tests/unique_temp.hpp).
//
// Every realm's worktree, the main tree and a consumer's clone run this suite
// on one machine at once, sharing one temp folder. A path named only after
// its test was the same in every process, so one run's cleanup deleted
// another's files mid-test (the lead's gate 5, 2026-10-06). Each path now
// carries the process id and a per-process counter.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "unique_temp.hpp"

namespace {

namespace fs = std::filesystem;
using spade::test::TempDir;

}  // namespace

TEST(UniqueTemp, TwoTempDirsInOneTestGetDifferentFolders) {
    const TempDir a("twin");
    const TempDir b("twin");
    EXPECT_NE(a.path(), b.path());
    EXPECT_TRUE(fs::is_directory(a.path()));
    EXPECT_TRUE(fs::is_directory(b.path()));
}

TEST(UniqueTemp, APathNamesItsProcessAndTag) {
    const fs::path p = spade::test::unique_temp_path("probe.yaml");
    const std::string name = p.filename().string();
    EXPECT_NE(name.find("_" + std::to_string(spade::test::process_id()) + "_"), std::string::npos) << name;
    EXPECT_TRUE(name.ends_with("_probe.yaml")) << name;
    // temp_directory_path() ends in a separator on Windows; parent_path() does not.
    EXPECT_EQ((p.parent_path() / "x").lexically_normal(), (fs::temp_directory_path() / "x").lexically_normal());
    EXPECT_FALSE(fs::exists(p));  // a path is named, not created
    EXPECT_NE(spade::test::unique_temp_path("probe.yaml"), p);
}

TEST(UniqueTemp, AFolderGoesWithItsTempDir) {
    fs::path kept;
    {
        const TempDir dir("gone");
        kept = dir.path();
        std::ofstream(dir.path() / "file.txt") << "x";
        ASSERT_TRUE(fs::exists(kept / "file.txt"));
    }
    EXPECT_FALSE(fs::exists(kept));
}
