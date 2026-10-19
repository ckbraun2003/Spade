// The engine's build stamp (core/build_identity.hpp): its form, and, in a
// build from `git archive` (the Docker leg), that it names the archived commit.
// The stamp script's fallbacks themselves are StampIdentity.ArchiveFallback, a
// CMake-script test (tests/stamp_identity_test.cmake).

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>

#include "core/build_identity.hpp"

namespace {

namespace fs = std::filesystem;

[[nodiscard]] bool is_full_commit(std::string_view text) {
    if (text.size() != 40) return false;
    for (const char c : text) {
        if ((c < '0' || c > '9') && (c < 'a' || c > 'f')) return false;
    }
    return true;
}

}  // namespace

TEST(BuildIdentity, CommitIsFortyLowercaseHexOrUnknown) {
    const spade::BuildIdentity built = spade::build_identity();
    EXPECT_TRUE(is_full_commit(built.commit) || built.commit == "unknown") << "commit '" << built.commit << "'";
}

TEST(BuildIdentity, UnknownIsNeverDirty) {
    const spade::BuildIdentity built = spade::build_identity();
    if (built.commit == "unknown") {
        EXPECT_FALSE(built.dirty);
    }
}

// cmake/archive_commit.txt holds `$Format:%H$` in a checkout, and the commit in
// a tree `git archive` made (export-subst). There the build has no git to ask,
// so the stamp must have read the commit from it, clean.
TEST(BuildIdentity, AnArchiveBuildNamesTheArchivedCommit) {
    const fs::path file = fs::path(SPADE_TESTS_DIR).parent_path() / "cmake" / "archive_commit.txt";
    std::ifstream in(file, std::ios::binary);
    ASSERT_TRUE(in) << "cannot read " << file.string();
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r' || text.back() == ' ')) text.pop_back();
    if (text == "$Format:%H$") {
        GTEST_SKIP() << file.string() << " is unsubstituted, so these sources are a git checkout, not an archive: "
                     << "the stamp came from git. A build from `git archive` (the Docker leg) runs this check";
    }
    ASSERT_TRUE(is_full_commit(text)) << file.string() << " holds neither $Format:%H$ nor a commit: '" << text << "'";
    const spade::BuildIdentity built = spade::build_identity();
    EXPECT_EQ(built.commit, text);
    EXPECT_FALSE(built.dirty);
}
