// ---------------------------------------------------------------------------
// test_installed_layout.cpp -- the editor's inputs live where the INSTALLER puts them (O167, 2026-11-23: the installer built from ba66a3e failed 23 of 24 because
// the live tour copied tools/flow_movie/.../x500_contact.model.yaml, which an install does not have).
//
// WHAT THE INSTALLED LAYOUT IS, read from its one source: packaging/Packaging.cmake's install(DIRECTORY ... DESTINATION assets) lists the folders of assets/ that
// an install carries. This test parses that text; nothing is hard-coded here, so a folder added to the installer is allowed here at once and a folder dropped from
// it fails here at once.
//
//   InstalledLayout.TheInstallerCarriesTheAssetFoldersTheEditorNeeds         library, scenes, worlds, fonts and models are in the install rule
//   InstalledLayout.NoEditorSourceReadsAPathTheInstallerDoesNotCarry         every `assets / "<folder>"` and `.parent_path() / "<folder>"` literal in sandbox/ names a
//                                                                            carried folder; no SPADE_REPO_DIR or flow_movie path; the dynamic joins are pinned by name
//   InstalledLayout.TheX500ContactModelHasOneCopyUnderAssetsModels           assets/models/x500_contact.model.yaml exists and the old copy under tools/ does not
//
// THE LIMIT, said where it matters: this scans LITERALS. A path assembled from a variable at run time is not seen, so the few such joins are listed below by file
// and pinned, and a new one fails the scan until it is listed. The end-to-end check is the installer test (scripts/package-test.ps1: the live smoke run from the
// install folder); this one makes the next source-tree-only path fail in the gate rather than in an installer.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

fs::path repo(const char* relative) { return fs::path(SPADE_REPO_DIR) / relative; }

std::string read_all(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// The asset folders install(DIRECTORY ... DESTINATION assets) carries.
std::set<std::string> installed_asset_folders() {
    const std::string text = read_all(repo("packaging/Packaging.cmake"));
    std::set<std::string> folders;
    const std::size_t start = text.find("install(DIRECTORY");
    if (start == std::string::npos) return folders;
    const std::size_t end = text.find("DESTINATION assets", start);
    if (end == std::string::npos) return folders;
    const std::string rule = text.substr(start, end - start);
    const std::regex folder(R"(/assets/([A-Za-z0-9_]+)\")");
    for (std::sregex_iterator it(rule.begin(), rule.end(), folder), done; it != done; ++it) folders.insert((*it)[1]);
    return folders;
}

struct SourceLine {
    std::string file;  // relative to sandbox/
    int number = 0;
    std::string text;
};

// Every non-comment line of the editor's own sources (sandbox/*.cpp, *.hpp, *.inc).
std::vector<SourceLine> editor_code_lines() {
    std::vector<SourceLine> out;
    for (const auto& entry : fs::directory_iterator(repo("sandbox"))) {
        const std::string ext = entry.path().extension().string();
        if (!entry.is_regular_file() || (ext != ".cpp" && ext != ".hpp" && ext != ".inc")) continue;
        std::istringstream in(read_all(entry.path()));
        std::string line;
        int n = 0;
        while (std::getline(in, line)) {
            ++n;
            const std::size_t first = line.find_first_not_of(" \t");
            if (first == std::string::npos) continue;
            if (line.compare(first, 2, "//") == 0 || line[first] == '*') continue;  // a comment line
            const std::size_t slashes = line.find("//");
            out.push_back({entry.path().filename().string(), n, slashes == std::string::npos ? line : line.substr(0, slashes)});
        }
    }
    return out;
}

TEST(InstalledLayout, TheInstallerCarriesTheAssetFoldersTheEditorNeeds) {
    const std::set<std::string> folders = installed_asset_folders();
    ASSERT_FALSE(folders.empty()) << "packaging/Packaging.cmake: no install(DIRECTORY ... DESTINATION assets) rule found";
    for (const char* needed : {"library", "scenes", "worlds", "fonts", "models"}) {
        EXPECT_TRUE(folders.count(needed) == 1) << "assets/" << needed << " is not in the installer's rule";
    }
}

TEST(InstalledLayout, NoEditorSourceReadsAPathTheInstallerDoesNotCarry) {
    const std::set<std::string> carried = installed_asset_folders();
    ASSERT_FALSE(carried.empty());
    const std::vector<SourceLine> lines = editor_code_lines();
    ASSERT_GT(lines.size(), 1000u) << "sandbox/ was not found or is empty";

    const std::regex assets_literal(R"(\bassets\w*(\(\))?\s*/\s*"([^"/]+))");
    const std::regex parent_literal(R"(parent_path\(\)\s*/\s*"([^"/]+))");
    const std::regex assets_dynamic(R"(\bassets\w*(\(\))?\s*/\s*[A-Za-z_(])");

    // THE DYNAMIC JOINS, pinned by file: a join of the assets root with a variable. live_tour.cpp's work copy joins `assets / dir` for dir in {"scenes", "worlds"}.
    const std::set<std::string> pinned_dynamic = {"live_tour.cpp:assets / dir"};
    std::set<std::string> seen_dynamic;

    for (const SourceLine& l : lines) {
        std::smatch m;
        if (std::regex_search(l.text, m, assets_literal)) {
            EXPECT_TRUE(carried.count(m[2]) == 1) << l.file << ":" << l.number << " joins the assets root with \"" << m[2] << "\", which the installer does not carry:\n  " << l.text;
        }
        if (std::regex_search(l.text, m, parent_literal)) {
            EXPECT_TRUE(carried.count(m[1]) == 1) << l.file << ":" << l.number << " joins a parent folder with \"" << m[1] << "\", which the installer does not carry:\n  " << l.text;
        }
        EXPECT_EQ(l.text.find("SPADE_REPO_DIR"), std::string::npos) << l.file << ":" << l.number << " reads the source tree: " << l.text;
        EXPECT_EQ(l.text.find("\"flow_movie\""), std::string::npos) << l.file << ":" << l.number << " reads tools/flow_movie, which an install does not have: " << l.text;
        if (std::regex_search(l.text, assets_dynamic) && !std::regex_search(l.text, assets_literal)) {
            seen_dynamic.insert(l.file + ":assets / dir");  // the only dynamic form pinned is `assets / dir`
            EXPECT_TRUE(l.text.find("assets / dir") != std::string::npos)
                << l.file << ":" << l.number << " joins the assets root with a run-time value; pin it in this test (and make sure the folder is carried):\n  " << l.text;
        }
    }
    for (const std::string& site : seen_dynamic) EXPECT_TRUE(pinned_dynamic.count(site) == 1) << site << " is a dynamic join this test does not pin";

    // The pinned loop's own literals are carried folders.
    const std::string tour = read_all(repo("sandbox/live_tour.cpp"));
    const std::regex loop(R"re(for \(const char\* dir : \{([^}]*)\})re");
    std::smatch lm;
    ASSERT_TRUE(std::regex_search(tour, lm, loop)) << "live_tour.cpp's work-copy loop moved: update this test's pin";
    const std::string names = lm[1];
    const std::regex quoted(R"re("([^"]+)")re");
    int count = 0;
    for (std::sregex_iterator it(names.begin(), names.end(), quoted), done; it != done; ++it, ++count) {
        EXPECT_TRUE(carried.count((*it)[1]) == 1) << "the work-copy loop reads assets/" << (*it)[1] << ", which the installer does not carry";
    }
    EXPECT_GE(count, 2);
}

TEST(InstalledLayout, TheX500ContactModelHasOneCopyUnderAssetsModels) {
    EXPECT_TRUE(fs::is_regular_file(repo("assets/models/x500_contact.model.yaml"))) << "assets/models/x500_contact.model.yaml is missing";
    EXPECT_FALSE(fs::exists(repo("tools/flow_movie/data/x500/x500_contact.model.yaml"))) << "a second copy of the contact model is left under tools/flow_movie";
}

}  // namespace
