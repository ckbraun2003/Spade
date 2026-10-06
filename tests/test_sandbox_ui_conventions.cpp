// The sandbox UI's conventions (the editor design's §13, EDT-021: the user's
// direction of 2026-10-06 for a sleek, consolidated, Unity/Unreal-style UI).
//
// A source scan over sandbox/, in the manner of TD-3's canary: the calls §13
// does not use appear nowhere, and style is set only in ui_theme.hpp. Comments
// are skipped, so a comment may name a call to explain why it is not used.
// The UI font ships with its licence beside it.

#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace fs = std::filesystem;

[[nodiscard]] fs::path sandbox_dir() { return (fs::path(SPADE_TESTS_DIR) / ".." / "sandbox").lexically_normal(); }

struct Hit {
    std::string where;  // file:line
    std::string call;
};

// Every use of one of `calls` in the sandbox's sources, comments skipped,
// except in the files named in `allowed`.
[[nodiscard]] std::vector<Hit> uses(const std::vector<std::string_view>& calls,
                                    const std::vector<std::string_view>& allowed, std::size_t* files = nullptr) {
    std::vector<Hit> hits;
    std::size_t scanned = 0;
    for (const auto& entry : fs::recursive_directory_iterator(sandbox_dir())) {
        const fs::path& p = entry.path();
        if (!entry.is_regular_file() || (p.extension() != ".cpp" && p.extension() != ".hpp")) continue;
        ++scanned;
        const std::string name = p.filename().string();
        bool exempt = false;
        for (const std::string_view a : allowed) exempt = exempt || name == a;
        if (exempt) continue;
        std::ifstream in(p);
        std::string line;
        std::size_t number = 0;
        bool in_block_comment = false;
        while (std::getline(in, line)) {
            ++number;
            std::string code;
            for (std::size_t i = 0; i < line.size(); ++i) {
                if (in_block_comment) {
                    if (line.compare(i, 2, "*/") == 0) {
                        in_block_comment = false;
                        ++i;
                    }
                    continue;
                }
                if (line.compare(i, 2, "//") == 0) break;
                if (line.compare(i, 2, "/*") == 0) {
                    in_block_comment = true;
                    ++i;
                    continue;
                }
                code += line[i];
            }
            for (const std::string_view call : calls) {
                if (code.find(call) != std::string::npos) {
                    hits.push_back({name + ":" + std::to_string(number), std::string(call)});
                }
            }
        }
    }
    if (files != nullptr) *files = scanned;
    return hits;
}

[[nodiscard]] std::string list(const std::vector<Hit>& hits) {
    std::string out;
    for (const Hit& h : hits) out += "\n  " + h.where + "  " + h.call;
    return out;
}

}  // namespace

// §13: no boxes inside panels, and no separators for structure. Lists are
// Selectable rows in the panel itself, so there is no BeginChild at all.
TEST(UiConventions, NoBoxesOrSeparatorsInTheSandbox) {
    std::size_t files = 0;
    const std::vector<Hit> hits =
        uses({"ImGui::Separator", "ImGui::BeginChild", "ImGuiChildFlags_Border"}, {}, &files);
    EXPECT_GT(files, 10u) << "the scan found too few files under " << sandbox_dir();
    EXPECT_TRUE(hits.empty()) << "§13 does not use these:" << list(hits);
}

// §13: one theme. Colours and style variables are set in ui_theme.hpp alone,
// and status text goes through its status_text().
TEST(UiConventions, StyleIsSetOnlyInTheTheme) {
    const std::vector<Hit> hits = uses({"ImGui::PushStyleColor", "ImGui::PushStyleVar", "ImGui::StyleColors",
                                        "ImGui::TextColored", "GetStyle().Colors"},
                                       {"ui_theme.hpp"});
    EXPECT_TRUE(hits.empty()) << "style outside ui_theme.hpp:" << list(hits);
    EXPECT_TRUE(fs::exists(sandbox_dir() / "ui_theme.hpp"));
}

// The UI font (§13, the lead's conditions of 2026-10-06): OFL or Apache-2.0,
// its licence vendored beside it, one regular and one semibold weight.
TEST(UiConventions, TheUiFontShipsWithItsLicence) {
    const fs::path fonts = fs::path(SPADE_ASSETS_DIR) / "fonts";
    ASSERT_TRUE(fs::is_directory(fonts)) << fonts;
    for (const char* file : {"Inter-Regular.ttf", "Inter-SemiBold.ttf", "OFL.txt", "README.md"}) {
        EXPECT_TRUE(fs::exists(fonts / file)) << (fonts / file);
    }
    std::ifstream in(fonts / "OFL.txt");
    const std::string licence((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(licence.find("SIL OPEN FONT LICENSE"), std::string::npos);
    std::size_t ttf = 0;
    for (const auto& entry : fs::directory_iterator(fonts)) ttf += entry.path().extension() == ".ttf" ? 1u : 0u;
    EXPECT_EQ(ttf, 2u) << "one regular and at most one semibold weight";
}
