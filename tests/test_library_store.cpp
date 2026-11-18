// ---------------------------------------------------------------------------
// test_library_store.cpp -- entries whose file lives in the local store (L88b):
// `store: {sha256, file}` instead of `file`, at <store>/<sha256>/<file>. The shapes
// are the contract's section 3 (docs/design/core/plans/2026-11-17-spade-library-
// json-contract.md, "Store files"). Every test writes its own roots and its own
// store under unique temp folders.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "scene/library_query.hpp"
#include "scene/library_set.hpp"
#include "unique_temp.hpp"
#include "world/sha256.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;

fs::path fresh(const std::string& name) {
    static std::vector<std::unique_ptr<spade::test::TempDir>> dirs;
    dirs.push_back(std::make_unique<spade::test::TempDir>("library_store_" + name));
    return dirs.back()->path();
}

void put(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

const std::string kGlb = std::string("glTF\0\r\nbinary bytes", 19);
const std::string kSha = spade::sha256_hex(kGlb);

// A v3 root `r` with the given entries (one index).
fs::path root_with(const fs::path& dir, const std::string& entries) {
    put(dir / "library.yaml", "library_version: 3\nname: r\ndepends: []\nindexes:\n  - index.yaml\n");
    put(dir / "index.yaml", "index_version: 1\nentries:\n" + entries);
    return dir;
}

const std::string kMesh = "  - id: mesh.x500\n    name: x500 frame\n    category: Meshes\n"
                          "    store: {sha256: \"" + kSha + "\", file: x500_frame.glb}\n";

sc::LibraryResult<sc::LibrarySet> open(const fs::path& root, const fs::path& store = {}) {
    const std::vector<sc::LibraryRootSpec> roots = {{"r", root, false}};
    return sc::open_libraries(roots, store);
}

const sc::LibraryRefusal* issue(const sc::LibraryRefusal& r, const std::string& code) {
    for (const sc::LibraryRefusal& i : r.issues) {
        if (i.code == code) {
            return &i;
        }
    }
    return nullptr;
}

TEST(LibraryStore, AStoreEntryResolvesAgainstTheHostsStoreAndOpensWithoutReadingIt) {
    const fs::path base = fresh("resolve");
    const fs::path root = root_with(base / "r", kMesh);
    const fs::path store = base / "store";
    put(store / kSha / "x500_frame.glb", kGlb);
    const auto set = open(root, store);
    ASSERT_TRUE(set.has_value()) << set.error().message;
    EXPECT_EQ(set->store, store);
    ASSERT_EQ(set->entries.size(), 1u);
    const sc::SetEntry& e = set->entries[0];
    EXPECT_EQ(e.id, "r:mesh.x500");
    EXPECT_EQ(e.kind, "mesh");
    EXPECT_TRUE(e.in_store);
    EXPECT_EQ(e.store_sha256, kSha);
    EXPECT_EQ(e.store_file, "x500_frame.glb");
    EXPECT_TRUE(e.rel_file.empty());
    EXPECT_EQ(e.file, store / kSha / "x500_frame.glb");
}

TEST(LibraryStore, AnAbsentStoreOrAMissingFileDoesNotStopTheOpenAndIsAListWarning) {
    const fs::path base = fresh("missing");
    const fs::path root = root_with(base / "r", kMesh);
    // No store at all.
    const auto none = open(root);
    ASSERT_TRUE(none.has_value()) << none.error().message;
    EXPECT_TRUE(none->entries[0].file.empty());
    EXPECT_TRUE(none->entries[0].in_store);
    const auto page = sc::list_entries(*none, {});
    ASSERT_TRUE(page.has_value());
    ASSERT_EQ(page->rows.size(), 1u);
    bool warned = false;
    for (const sc::LibraryRefusal& w : page->warnings) {
        warned = warned || (w.code == "store_file_missing" && w.id == "r:mesh.x500" && w.field == "store");
    }
    EXPECT_TRUE(warned);
    ASSERT_TRUE(page->rows[0].peek.has_value());  // an empty peek, not a failed one
    EXPECT_FALSE(page->rows[0].peek->mass_kg.has_value());
    // A store that lacks the file: the same, and the warning names the path it looked at.
    const fs::path store = base / "store";
    fs::create_directories(store);
    const auto lacking = open(root, store);
    ASSERT_TRUE(lacking.has_value());
    const auto page2 = sc::list_entries(*lacking, {});
    ASSERT_TRUE(page2.has_value());
    ASSERT_FALSE(page2->warnings.empty());
    EXPECT_EQ(page2->warnings[0].code, "store_file_missing");
    EXPECT_EQ(page2->warnings[0].path, (store / kSha / "x500_frame.glb").string());
}

TEST(LibraryStore, TheHashUsesTheDeclaredShaAndIsTheSameWithOrWithoutTheStore) {
    const fs::path base = fresh("hash");
    const fs::path root = root_with(base / "r", kMesh);
    const fs::path store = base / "store";
    put(store / kSha / "x500_frame.glb", kGlb);
    const std::string expected =
        "sha256:" + spade::sha256_hex("store/" + kSha + "/x500_frame.glb\t" + kSha + "\n");
    const auto with = open(root, store);
    const auto without = open(root);
    ASSERT_TRUE(with.has_value() && without.has_value());
    const auto a = sc::entry_hash(*with, with->entries[0]);
    const auto b = sc::entry_hash(*without, without->entries[0]);
    ASSERT_TRUE(a.has_value()) << a.error().message;
    ASSERT_TRUE(b.has_value()) << b.error().message;
    EXPECT_EQ(*a, expected);
    EXPECT_EQ(*b, expected);
    // The file is not read: other bytes under the same name do not change the hash.
    put(store / kSha / "x500_frame.glb", "different bytes");
    EXPECT_EQ(*sc::entry_hash(*with, with->entries[0]), expected);
    // A present store file gives a normal list row, without a warning.
    put(store / kSha / "x500_frame.glb", kGlb);
    const auto page = sc::list_entries(*with, {});
    ASSERT_TRUE(page.has_value());
    EXPECT_TRUE(page->warnings.empty());
    EXPECT_EQ(page->rows[0].hash, expected);
}

TEST(LibraryStore, APresentStoreFileGivesItsPeekLikeARootFile) {
    const fs::path base = fresh("peek");
    const std::string motor = "part_version: 1\nmotor:\n  kv: 2000\n  mass: 0.041\n";
    const std::string sha = spade::sha256_hex(motor);
    const fs::path root = root_with(base / "r", "  - id: part.motor.stored\n    name: Stored motor\n    category: Motors\n"
                                                "    store: {sha256: \"" + sha + "\", file: stored.motor.yaml}\n");
    const fs::path store = base / "store";
    put(store / sha / "stored.motor.yaml", motor);
    const auto set = open(root, store);
    ASSERT_TRUE(set.has_value()) << set.error().message;
    EXPECT_EQ(set->entries[0].part_kind, "motor");
    const auto page = sc::list_entries(*set, {});
    ASSERT_TRUE(page.has_value());
    ASSERT_TRUE(page->rows[0].peek.has_value());
    ASSERT_TRUE(page->rows[0].peek->mass_kg.has_value());
    EXPECT_DOUBLE_EQ(*page->rows[0].peek->mass_kg, 0.041);
}

TEST(LibraryStore, StoreRulesNameTheirFieldAndARelativeStoreIsRefused) {
    const fs::path base = fresh("rules");
    auto breach = [&](const std::string& name, const std::string& entry) {
        const fs::path r = root_with(base / name, entry);
        const auto set = open(r, base / "store");
        EXPECT_FALSE(set.has_value()) << name;
        if (set.has_value()) {
            return std::string();
        }
        const sc::LibraryRefusal* i = issue(set.error(), "bad_index");
        EXPECT_NE(i, nullptr) << name;
        return i != nullptr ? i->field : std::string();
    };
    const std::string head = "  - id: mesh.x\n    name: x\n    category: Meshes\n";
    const std::string ok_sha = "\"" + kSha + "\"";
    EXPECT_EQ(breach("both", head + "    file: m.gltf\n    store: {sha256: " + ok_sha + ", file: x.glb}\n"), "entries[0].store");
    EXPECT_EQ(breach("upper", head + "    store: {sha256: \"" + std::string(64, 'A') + "\", file: x.glb}\n"),
              "entries[0].store.sha256");
    EXPECT_EQ(breach("short", head + "    store: {sha256: \"abc\", file: x.glb}\n"), "entries[0].store.sha256");
    EXPECT_EQ(breach("slash", head + "    store: {sha256: " + ok_sha + ", file: a/x.glb}\n"), "entries[0].store.file");
    EXPECT_EQ(breach("dots", head + "    store: {sha256: " + ok_sha + ", file: \"..\"}\n"), "entries[0].store.file");
    EXPECT_EQ(breach("back", head + "    store: {sha256: " + ok_sha + ", file: \"a\\\\x.glb\"}\n"), "entries[0].store.file");
    EXPECT_EQ(breach("kind", "  - id: part.motor.x\n    name: x\n    category: M\n    store: {sha256: " + ok_sha +
                                 ", file: x.glb}\n"),
              "entries[0].store.file");
    EXPECT_EQ(breach("extra", head + "    store: {sha256: " + ok_sha + ", file: x.glb, url: u}\n"), "entries[0].store.url");
    EXPECT_EQ(breach("nofile", head + "    store: {sha256: " + ok_sha + "}\n"), "entries[0].store.file");

    const fs::path r = root_with(base / "rel", kMesh);
    const auto rel = open(r, fs::path("relative/store"));
    ASSERT_FALSE(rel.has_value());
    EXPECT_NE(issue(rel.error(), "relative_path"), nullptr);
}

}  // namespace
