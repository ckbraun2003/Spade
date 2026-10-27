// ---------------------------------------------------------------------------
// test_temp_hygiene.cpp -- no fixed-name temp path outside tests/unique_temp.hpp.
//
// WHY. Every tree on the development box (the realms' worktrees, the lead's
// tree, Kat's clone) runs this suite at the same time, and they share one
// %TEMP%. A test that writes "spade_test_maximal.world.yaml" there collides
// with the same test in another tree, and the loser fails for no reason of
// its own: gate 5 lost two editor tests that way (2026-10-06).
// tests/unique_temp.hpp names temp paths per process, so it is the one place
// allowed to ask for the temp directory (with its own test; see kAllowed).
//
// HOW. Like BitPortability.NoLibmTranscendentalInEngineOrGoldenTestSource
// (test_m1b_bar.cpp), this reads the sources with their comments stripped and
// refuses the call anywhere else, under tests/ (tests/consumer included),
// engine/ and sandbox/. A product use would need its own argument: a fixed
// name in a shared temp directory is the same hazard outside a test.
// A scan that finds no sources fails (TD-5).
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

// Split so this file's own source text never contains the pattern.
const std::string kPattern = std::string("temp_directory") + "_path";

// The files allowed to call it, relative to tests/: the header that names
// every temp path, and its own test, which checks those names against the
// real temp directory and writes nothing there; and one product use with its
// own argument (the lead's grant, 2026-10-22): the editor's import staging
// (N15, ImportStaging), a folder per workspace under spade-editor-imports
// named by a random 64-bit token and removed with the workspace, so no two
// runs share a name.
constexpr const char* kAllowed[] = {"unique_temp.hpp", "test_unique_temp.cpp", "../sandbox/editor_workspace.hpp"};

// The text with // and /* */ comments blanked, keeping line breaks so line
// numbers survive. String literals are not parsed; a "//" inside one only
// hides the rest of that line from the scan, which can miss but never invent
// a hit.
[[nodiscard]] std::string strip_comments(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size();) {
        if (text.compare(i, 2, "//") == 0) {
            while (i < text.size() && text[i] != '\n') ++i;
        } else if (text.compare(i, 2, "/*") == 0) {
            i += 2;
            while (i < text.size() && text.compare(i, 2, "*/") != 0) {
                if (text[i] == '\n') out += '\n';
                ++i;
            }
            i += 2;
        } else {
            out += text[i++];
        }
    }
    return out;
}

[[nodiscard]] std::vector<fs::path> sources_under(const fs::path& root) {
    std::vector<fs::path> out;
    if (!fs::exists(root)) return out;
    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(root)) {
        if (!entry.is_regular_file()) continue;
        const std::string ext = entry.path().extension().string();
        if (ext == ".cpp" || ext == ".hpp" || ext == ".h") out.push_back(entry.path());
    }
    std::sort(out.begin(), out.end());
    return out;
}

}  // namespace

TEST(TestHygiene, NoTempDirectoryPathOutsideUniqueTemp) {
    const fs::path tests_dir(SPADE_TESTS_DIR);
    const fs::path roots[] = {tests_dir, fs::path(SPADE_ENGINE_DIR), tests_dir.parent_path() / "sandbox"};

    std::size_t scanned = 0;
    std::vector<std::string> hits;
    for (const fs::path& root : roots) {
        const std::vector<fs::path> files = sources_under(root);
        ASSERT_FALSE(files.empty()) << "no sources under " << root.string() << " -- is the scan root right?";
        for (const fs::path& file : files) {
            if (std::any_of(std::begin(kAllowed), std::end(kAllowed), [&](const char* allowed) {
                    return file.lexically_normal() == (tests_dir / allowed).lexically_normal();
                })) {
                continue;
            }
            std::ifstream in(file, std::ios::binary);
            std::stringstream buffer;
            buffer << in.rdbuf();
            const std::string text = strip_comments(buffer.str());
            ++scanned;
            std::size_t line = 1;
            std::size_t line_start = 0;
            for (std::size_t i = 0; i < text.size(); ++i) {
                if (text[i] == '\n') {
                    ++line;
                    line_start = i + 1;
                } else if (text.compare(i, kPattern.size(), kPattern) == 0) {
                    const std::size_t end = text.find('\n', i);
                    hits.push_back(file.lexically_relative(tests_dir.parent_path()).generic_string() + ":" +
                                   std::to_string(line) + ": " +
                                   text.substr(line_start, (end == std::string::npos ? text.size() : end) - line_start));
                    i += kPattern.size() - 1;
                }
            }
        }
    }

    ASSERT_GT(scanned, 0u);
    std::string report;
    for (const std::string& h : hits) report += "\n  " + h;
    EXPECT_TRUE(hits.empty()) << hits.size() << " fixed-name temp path(s) outside tests/unique_temp.hpp"
                              << ". Concurrent runs on this box share %TEMP%; name the path through "
                              << "tests/unique_temp.hpp instead:" << report;
}
