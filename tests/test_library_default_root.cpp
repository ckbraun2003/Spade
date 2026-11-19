// ---------------------------------------------------------------------------
// test_library_default_root.cpp -- the default root on library version 3 (L88l): assets/library/{library,index}.yaml replace the version-2 library.yaml;
// tests/data/library_default_v2.yaml is that old text, kept as the reader's fixture.
//
//   LibraryDefaultRoot.TheV3RootReadsAsTheLibraryTheV2IndexGave   scene::load_library() of the committed library.yaml and library_from_yaml() of the old
//                                                  text: the same entries, in the same order, name, category, file, thumbnail and kind each
//   LibraryDefaultRoot.TheSetKeepsEveryIdAndFileOfTheV2Root         open_libraries() on the committed root and on a v2 root made of the old text (its
//                                                  files as empty placeholders): the old 38 are the root's first 38, id for id, file for file,
//                                                  thumbnail, name, category, kind
//   LibraryDefaultRoot.ALegacyReaderKeepsReadingVersion2           load_library() of the old text on disk still gives the version-2 library (a consumer's
//                                                  own library may be v2), and version 4 is still refused by name
//   LibraryDefaultRoot.LoadLibraryListsOnlyTheKindsItKnows         a v3 root with a model, a frame part and a world: the list is the model
// Every test writes its own folders under unique temp roots.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "scene/library.hpp"
#include "scene/library_set.hpp"
#include "unique_temp.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;

const fs::path kAssets = fs::path(SPADE_ASSETS_DIR) / "library";
const fs::path kFixture = fs::path(SPADE_REPO_DIR) / "tests" / "data" / "library_default_v2.yaml";

fs::path fresh(const std::string& name) {
    static std::vector<std::unique_ptr<spade::test::TempDir>> dirs;
    dirs.push_back(std::make_unique<spade::test::TempDir>("library_default_root_" + name));
    return dirs.back()->path();
}

std::string read_text(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void put(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

// The old index's text, LF only (a Windows checkout may have turned it CRLF).
std::string v2_text() {
    std::string text = read_text(kFixture);
    std::string out;
    for (char c : text) {
        if (c != '\r') out += c;
    }
    return out;
}

// A folder holding the old text as library.yaml and an empty file for every entry's file and thumbnail.
fs::path v2_root(const std::string& name) {
    const fs::path dir = fresh(name);
    const std::string text = v2_text();
    put(dir / "library.yaml", text);
    const auto parsed = sc::library_from_yaml(text);
    EXPECT_TRUE(parsed.has_value()) << (parsed.has_value() ? "" : parsed.error().context);
    if (!parsed) return dir;
    for (const sc::LibraryEntry& e : parsed->entries) {
        put(dir / e.file, "");
        if (!e.thumbnail.empty()) put(dir / e.thumbnail, "");
    }
    return dir;
}

TEST(LibraryDefaultRoot, TheV3RootReadsAsTheLibraryTheV2IndexGave) {
    const auto old = sc::library_from_yaml(v2_text());
    ASSERT_TRUE(old.has_value()) << old.error().context;
    ASSERT_EQ(old->entries.size(), 38u);
    const auto now = sc::load_library(kAssets / "library.yaml");
    ASSERT_TRUE(now.has_value()) << now.error().context;
    ASSERT_EQ(now->entries.size(), old->entries.size());
    for (std::size_t i = 0; i < old->entries.size(); ++i) {
        const sc::LibraryEntry& a = old->entries[i];
        const sc::LibraryEntry& b = now->entries[i];
        EXPECT_EQ(b.name, a.name) << "entry " << i;
        EXPECT_EQ(b.category, a.category) << a.name;
        EXPECT_EQ(b.file, a.file) << a.name;
        EXPECT_EQ(b.thumbnail, a.thumbnail) << a.name;
        EXPECT_EQ(b.kind, a.kind) << a.name;
    }
}

TEST(LibraryDefaultRoot, TheSetKeepsEveryIdAndFileOfTheV2Root) {
    const fs::path old_dir = v2_root("v2");
    const std::vector<sc::LibraryRootSpec> old_roots = {{"default", old_dir, false}};
    const auto old_set = sc::open_libraries(old_roots);
    ASSERT_TRUE(old_set.has_value()) << old_set.error().message;
    const std::vector<sc::LibraryRootSpec> new_roots = {{"default", kAssets, false}};
    const auto new_set = sc::open_libraries(new_roots);
    ASSERT_TRUE(new_set.has_value()) << new_set.error().message;
    ASSERT_EQ(old_set->roots.size(), 1u);
    EXPECT_EQ(old_set->roots[0].library_version, 2u);
    ASSERT_EQ(new_set->roots.size(), 1u);
    EXPECT_EQ(new_set->roots[0].library_version, 3u);
    // the 38 entries of the old file come first and unchanged; the root has grown after them (the X500 build's parts and drone, L88l)
    ASSERT_GE(new_set->entries.size(), old_set->entries.size());
    for (std::size_t i = 0; i < old_set->entries.size(); ++i) {
        const sc::SetEntry& a = old_set->entries[i];
        const sc::SetEntry& b = new_set->entries[i];
        EXPECT_EQ(b.id, a.id) << "entry " << i;
        EXPECT_EQ(b.kind, a.kind) << a.id;
        EXPECT_EQ(b.part_kind, a.part_kind) << a.id;
        EXPECT_EQ(b.name, a.name) << a.id;
        EXPECT_EQ(b.category, a.category) << a.id;
        EXPECT_EQ(b.rel_file, a.rel_file) << a.id;
        EXPECT_EQ(b.rel_thumbnail, a.rel_thumbnail) << a.id;
        EXPECT_TRUE(fs::is_regular_file(b.file)) << b.file.string();
    }
}

TEST(LibraryDefaultRoot, ALegacyReaderKeepsReadingVersion2) {
    const fs::path dir = v2_root("legacy");
    const auto library = sc::load_library(dir / "library.yaml");
    ASSERT_TRUE(library.has_value()) << library.error().context;
    const auto parsed = sc::library_from_yaml(v2_text());
    ASSERT_TRUE(parsed.has_value());
    ASSERT_EQ(library->entries.size(), parsed->entries.size());
    for (std::size_t i = 0; i < library->entries.size(); ++i) {
        EXPECT_EQ(library->entries[i].name, parsed->entries[i].name);
        EXPECT_EQ(library->entries[i].file, parsed->entries[i].file);
    }
    // a version after this build's is still refused by name
    std::string newer = v2_text();
    const std::size_t at = newer.find("\nlibrary_version: 2\n");  // the key line, not the header's comment naming it
    ASSERT_NE(at, std::string::npos);
    newer.replace(at, std::string("\nlibrary_version: 2\n").size(), "\nlibrary_version: 4\n");
    put(dir / "newer.yaml", newer);
    const auto refused = sc::load_library(dir / "newer.yaml");
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::schema_mismatch);
}

TEST(LibraryDefaultRoot, LoadLibraryListsOnlyTheKindsItKnows) {
    const fs::path dir = fresh("kinds") / "r";
    put(dir / "library.yaml", "library_version: 3\nname: r\ndepends: []\nindexes:\n  - index.yaml\n");
    put(dir / "index.yaml",
        "index_version: 1\nentries:\n"
        "  - id: model.quad\n    name: Quad\n    category: drones\n    file: drones/quad.model.yaml\n"
        "  - id: part.frame.plate\n    name: Plate\n    category: parts\n    file: parts/plate.frame.yaml\n"
        "  - id: world.pad\n    name: Pad\n    category: worlds\n    file: worlds/pad.world.yaml\n");
    put(dir / "drones/quad.model.yaml", "x\n");
    put(dir / "parts/plate.frame.yaml", "x\n");
    put(dir / "worlds/pad.world.yaml", "x\n");
    const auto library = sc::load_library(dir / "library.yaml");
    ASSERT_TRUE(library.has_value()) << library.error().context;
    ASSERT_EQ(library->entries.size(), 1u);
    EXPECT_EQ(library->entries[0].name, "Quad");
    EXPECT_EQ(library->entries[0].kind, sc::LibraryKind::model);
    EXPECT_EQ(library->dir, dir);
}

}  // namespace
