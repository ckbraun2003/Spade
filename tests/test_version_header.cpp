// The generated version header (L73a; cmake/SpadeVersionGen.cmake): the string --version and About show.
#include <gtest/gtest.h>

#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

#include "spade_version.hpp"

namespace {

using spade::version::kDirty;
using spade::version::kGitSha;
using spade::version::kString;
using spade::version::kVersion;

TEST(VersionHeader, TheVersionIsThreeNumbersAndIsPackagingsVersionFile) {
    const std::string version(kVersion);
    int dots = 0;
    for (const char c : version) {
        if (c == '.') {
            ++dots;
        } else {
            EXPECT_TRUE(std::isdigit(static_cast<unsigned char>(c))) << version;
        }
    }
    EXPECT_EQ(dots, 2) << version;

    std::ifstream in(std::string(SPADE_REPO_DIR) + "/packaging/common/VERSION");
    ASSERT_TRUE(in.good()) << "packaging/common/VERSION is the product's version";
    std::string first_word;
    in >> first_word;
    EXPECT_EQ(first_word, version) << "the header was generated from the VERSION file";
}

TEST(VersionHeader, TheShaIsSevenHexDigitsOrUnknown) {
    const std::string sha(kGitSha);
    if (sha == "unknown") return;  // a tree with no git: the header says so, never a made-up SHA
    EXPECT_EQ(sha.size(), std::size_t{7}) << sha;
    for (const char c : sha) {
        EXPECT_TRUE((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')) << sha;
    }
}

TEST(VersionHeader, TheStringIsVersionPlusShaWithDirtyAppended) {
    const std::string expected = std::string(kVersion) + "+" + std::string(kGitSha) + (kDirty ? "-dirty" : "");
    EXPECT_EQ(std::string(kString), expected);
}

}  // namespace
