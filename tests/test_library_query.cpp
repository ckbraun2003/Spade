// ---------------------------------------------------------------------------
// test_library_query.cpp -- listing an opened library set (scene/library_query.hpp):
// filters and paging, the peek fields, and the computed entry hash. The shapes are
// the contract's (docs/design/core/plans/2026-11-17-spade-library-json-contract.md
// sections 3 and 4). Every test writes its own roots under unique temp folders;
// the last opens the repository's default library.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
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
    dirs.push_back(std::make_unique<spade::test::TempDir>("library_query_" + name));
    return dirs.back()->path();
}

void put(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

std::string slurp(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// A version 3 root named `name` with one index of `entries`.
fs::path v3_root(const fs::path& dir, const std::string& name, const std::string& depends,
                 const std::string& entries) {
    put(dir / "library.yaml",
        "library_version: 3\nname: " + name + "\ndepends: [" + depends + "]\nindexes:\n  - index.yaml\n");
    put(dir / "index.yaml", "index_version: 1\nentries:\n" + entries);
    return dir;
}

// A small set: `a` (v3) holds a drone, a motor, a frame with install points, a world, a mesh and an opaque entry;
// `b` (v3) holds a prop.
struct Fixture {
    fs::path base;
    sc::LibrarySet set;
};

Fixture make_fixture(const std::string& name) {
    Fixture f;
    f.base = fresh(name);
    const fs::path a = f.base / "a";
    const fs::path b = f.base / "b";
    v3_root(a, "a", "",
            "  - id: drone.quad\n    name: Quad\n    category: Drones\n    file: drones/quad.model.yaml\n"
            "    tags: [quad, five_inch]\n    files: [drones/quad.notes.txt]\n"
            "  - id: part.motor.fpv\n    name: FPV motor\n    category: Motors\n    file: motors/fpv.motor.yaml\n"
            "    tags: [fpv]\n"
            "  - id: part.motor.big\n    name: Big motor\n    category: Motors\n    file: motors/big.motor.yaml\n"
            "  - id: world.pad\n    name: Pad\n    category: Worlds\n    file: worlds/pad.world.yaml\n"
            "  - id: mesh.marker\n    name: Marker\n    category: Meshes\n    file: meshes/marker.gltf\n"
            "  - id: gates.corridor\n    name: Corridor gates\n    category: Gates\n    type: some.gate_set.v5\n"
            "    file: worlds/corridor.gates.json\n"
            "  - id: drone.mapped\n    name: Mapped\n    category: Drones\n    parts_map:\n"
            "      - {slot: frame, part: \"a:part.motor.fpv\", hash: null, placement: {pos: [0, 0, 0], quat: [1, 0, 0, 0]}, count: 1}\n");
    put(a / "drones/quad.model.yaml",
        "model_version: 1\nmodel:\n  name: \"quad\"\n  body: {mass: 0.6, inertia_diag: [1, 2, 3]}\n"
        "  rotors:\n    - {spin_dir: 1}\n    - {spin_dir: -1}\n    - {spin_dir: 1}\n    - {spin_dir: -1}\n");
    put(a / "drones/quad.notes.txt", "notes\n");
    put(a / "motors/fpv.motor.yaml", "part_version: 1\nmotor:\n  kv: 2000\n  mass: 0.032\n");
    put(a / "motors/big.motor.yaml",
        "part_version: 2\nmotor:\n  kv: 900\nprovides: [thrust.vertical]\nlimits: {current_max_a: 40}\n"
        "install_points:\n  - {name: hub, kind: prop_hub}\n  - {name: mount, kind: motor}\n  - {name: hub2, kind: prop_hub}\n");
    put(a / "worlds/pad.world.yaml", "world_version: 3\nname: pad\n");
    put(a / "meshes/marker.gltf", "{}\n");
    put(a / "worlds/corridor.gates.json", "{}\n");
    v3_root(b, "b", "a",
            "  - id: part.prop.five\n    name: Five inch\n    category: Props\n    file: props/five.prop.yaml\n"
            "    tags: [fpv, prop]\n    requires: [a:part.motor.fpv]\n");
    put(b / "props/five.prop.yaml", "part_version: 1\nprop:\n  diameter: 0.127\n");
    const std::vector<sc::LibraryRootSpec> roots = {{"a", a, false}, {"b", b, false}};
    auto set = sc::open_libraries(roots);
    EXPECT_TRUE(set.has_value()) << (set.has_value() ? "" : set.error().message);
    if (set.has_value()) {
        f.set = std::move(*set);
    }
    return f;
}

std::vector<std::string> ids(const sc::ListPage& page) {
    std::vector<std::string> out;
    for (const sc::ListedEntry& r : page.rows) {
        out.push_back(r.entry->id);
    }
    return out;
}

const sc::ListedEntry* row(const sc::ListPage& page, const std::string& id) {
    for (const sc::ListedEntry& r : page.rows) {
        if (r.entry->id == id) {
            return &r;
        }
    }
    return nullptr;
}

TEST(LibraryQuery, ListFollowsTheHostsRootOrderThenIndexOrderAndRootsFilter) {
    const Fixture f = make_fixture("order");
    const auto all = sc::list_entries(f.set, {});
    ASSERT_TRUE(all.has_value()) << all.error().message;
    EXPECT_EQ(all->total, 8u);
    EXPECT_EQ(ids(*all), (std::vector<std::string>{"a:drone.quad", "a:part.motor.fpv", "a:part.motor.big", "a:world.pad",
                                                   "a:mesh.marker", "a:gates.corridor", "a:drone.mapped",
                                                   "b:part.prop.five"}));
    sc::ListFilter only_b;
    only_b.roots = {"b"};
    const auto b = sc::list_entries(f.set, only_b);
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(ids(*b), std::vector<std::string>{"b:part.prop.five"});

    sc::ListFilter unknown;
    unknown.roots = {"nope"};
    const auto bad = sc::list_entries(f.set, unknown);
    ASSERT_FALSE(bad.has_value());
    EXPECT_EQ(bad.error().code, "root_not_loaded");
    EXPECT_EQ(bad.error().root, "nope");
}

TEST(LibraryQuery, KindPartKindTypeTextAndTagFiltersCombine) {
    const Fixture f = make_fixture("filters");
    auto list = [&](sc::ListFilter flt) {
        const auto r = sc::list_entries(f.set, flt);
        EXPECT_TRUE(r.has_value()) << (r.has_value() ? "" : r.error().message);
        return r.has_value() ? ids(*r) : std::vector<std::string>{};
    };
    sc::ListFilter k;
    k.kind = "part";
    EXPECT_EQ(list(k), (std::vector<std::string>{"a:part.motor.fpv", "a:part.motor.big", "b:part.prop.five"}));
    k.part_kind = "motor";
    EXPECT_EQ(list(k), (std::vector<std::string>{"a:part.motor.fpv", "a:part.motor.big"}));
    sc::ListFilter t;
    t.type = "some.gate_set.v5";
    EXPECT_EQ(list(t), std::vector<std::string>{"a:gates.corridor"});
    sc::ListFilter o;
    o.kind = "opaque";
    EXPECT_EQ(list(o), std::vector<std::string>{"a:gates.corridor"});
    sc::ListFilter text;
    text.text = "MOTOR";  // case-folded, over name, category and tags
    EXPECT_EQ(list(text), (std::vector<std::string>{"a:part.motor.fpv", "a:part.motor.big"}));
    text.text = "five";  // a tag on the quad, and the prop's name
    EXPECT_EQ(list(text), (std::vector<std::string>{"a:drone.quad", "b:part.prop.five"}));
    sc::ListFilter tags;
    tags.tags = {"fpv"};
    EXPECT_EQ(list(tags), (std::vector<std::string>{"a:part.motor.fpv", "b:part.prop.five"}));
    tags.tags = {"fpv", "prop"};
    EXPECT_EQ(list(tags), std::vector<std::string>{"b:part.prop.five"});

    sc::ListFilter bad;
    bad.kind = "widget";
    const auto r = sc::list_entries(f.set, bad);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, "invalid_request");
}

TEST(LibraryQuery, OffsetAndLimitPageTheFilteredListAndTotalIsBeforePaging) {
    const Fixture f = make_fixture("paging");
    sc::ListFilter flt;
    flt.offset = 2;
    flt.limit = 3;
    const auto page = sc::list_entries(f.set, flt);
    ASSERT_TRUE(page.has_value());
    EXPECT_EQ(page->total, 8u);
    EXPECT_EQ(page->offset, 2u);
    EXPECT_EQ(ids(*page), (std::vector<std::string>{"a:part.motor.big", "a:world.pad", "a:mesh.marker"}));
    flt.offset = 100;
    const auto past = sc::list_entries(f.set, flt);
    ASSERT_TRUE(past.has_value());
    EXPECT_TRUE(past->rows.empty());
    EXPECT_EQ(past->total, 8u);
}

TEST(LibraryQuery, TheHashIsSha256OverTheFileAndItsCompanionsAndChangesWithEither) {
    Fixture f = make_fixture("hash");
    const sc::SetEntry* quad = nullptr;
    for (const sc::SetEntry& e : f.set.entries) {
        if (e.id == "a:drone.quad") {
            quad = &e;
        }
    }
    ASSERT_NE(quad, nullptr);
    // The two files' lines, sorted by relative path bytewise.
    const std::string expected =
        "sha256:" + spade::sha256_hex(
                        "drones/quad.model.yaml\t" + spade::sha256_hex(slurp(quad->file)) + "\n" +
                        "drones/quad.notes.txt\t" + spade::sha256_hex("notes\n") + "\n");
    const auto h = sc::entry_hash(f.set, *quad);
    ASSERT_TRUE(h.has_value()) << h.error().message;
    EXPECT_EQ(*h, expected);
    EXPECT_EQ(h->size(), 7u + 64u);
    // The same bytes give the same hash.
    EXPECT_EQ(*sc::entry_hash(f.set, *quad), expected);
    // A companion changing changes it; so does the file.
    put(f.base / "a/drones/quad.notes.txt", "other notes\n");
    EXPECT_NE(*sc::entry_hash(f.set, *quad), expected);
    put(f.base / "a/drones/quad.notes.txt", "notes\n");
    EXPECT_EQ(*sc::entry_hash(f.set, *quad), expected);
    put(f.base / "a/drones/quad.model.yaml", slurp(quad->file) + "# edit\n");
    EXPECT_NE(*sc::entry_hash(f.set, *quad), expected);
    // A missing file is io_error naming the entry.
    fs::remove(f.base / "a/drones/quad.notes.txt");
    const auto gone = sc::entry_hash(f.set, *quad);
    ASSERT_FALSE(gone.has_value());
    EXPECT_EQ(gone.error().code, "io_error");
    EXPECT_EQ(gone.error().id, "a:drone.quad");
}

TEST(LibraryQuery, TextFilesHashWithCrLfReadAsLfAndEveryOtherFileByItsExactBytes) {
    const fs::path base = fresh("eol");
    const fs::path r = v3_root(base / "e", "e", "",
                               "  - id: drone.q\n    name: Q\n    category: Drones\n    file: drones/q.model.yaml\n"
                               "    files: [drones/q.notes.txt, drones/q.bin]\n");
    const std::string yaml_lf = "model_version: 1\nmodel:\n  name: q\n";
    const std::string yaml_crlf = "model_version: 1\r\nmodel:\r\n  name: q\r\n";
    put(r / "drones/q.model.yaml", yaml_lf);
    put(r / "drones/q.notes.txt", "a\nb\n");
    const std::string bin = std::string("x\r\ny\r\n");  // a binary companion that happens to hold CR LF
    put(r / "drones/q.bin", bin);
    const std::vector<sc::LibraryRootSpec> roots = {{"e", r, false}};
    const auto set = sc::open_libraries(roots);
    ASSERT_TRUE(set.has_value()) << set.error().message;
    const sc::SetEntry& q = set->entries.at(0);
    // The rule: .yaml, .yml, .json, .gltf, .md, .txt are hashed with CR LF read as LF; everything else exactly.
    const std::string expected =
        "sha256:" + spade::sha256_hex("drones/q.bin\t" + spade::sha256_hex(bin) + "\n" +
                                      "drones/q.model.yaml\t" + spade::sha256_hex(yaml_lf) + "\n" +
                                      "drones/q.notes.txt\t" + spade::sha256_hex("a\nb\n") + "\n");
    const auto lf = sc::entry_hash(*set, q);
    ASSERT_TRUE(lf.has_value()) << lf.error().message;
    EXPECT_EQ(*lf, expected);

    // A CRLF copy of the text files (a Windows checkout with autocrlf) hashes the same.
    put(r / "drones/q.model.yaml", yaml_crlf);
    put(r / "drones/q.notes.txt", "a\r\nb\r\n");
    EXPECT_EQ(*sc::entry_hash(*set, q), expected);

    // The binary companion is hashed as it is: dropping its CRs changes the hash.
    put(r / "drones/q.bin", "x\ny\n");
    EXPECT_NE(*sc::entry_hash(*set, q), expected);
    put(r / "drones/q.bin", bin);
    EXPECT_EQ(*sc::entry_hash(*set, q), expected);

    // Only CR LF pairs are folded: a lone CR is a different text.
    put(r / "drones/q.notes.txt", "a\rb\n");
    EXPECT_NE(*sc::entry_hash(*set, q), expected);
}

TEST(LibraryQuery, ACsvCompanionIsATextFileToo) {
    const fs::path base = fresh("csv");
    const fs::path r = v3_root(base / "e", "e", "",
                               "  - id: drone.q\n    name: Q\n    category: Drones\n    file: drones/q.model.yaml\n"
                               "    files: [data/t.csv]\n");
    put(r / "drones/q.model.yaml", "model_version: 1\n");
    put(r / "data/t.csv", "t,x\n1,2\n");
    const std::vector<sc::LibraryRootSpec> roots = {{"e", r, false}};
    const auto set = sc::open_libraries(roots);
    ASSERT_TRUE(set.has_value()) << set.error().message;
    const auto lf = sc::entry_hash(*set, set->entries.at(0));
    ASSERT_TRUE(lf.has_value());
    put(r / "data/t.csv", "t,x\r\n1,2\r\n");
    EXPECT_EQ(*sc::entry_hash(*set, set->entries.at(0)), *lf);
}

TEST(LibraryQuery, APartsMapIsPartOfTheHashAsItsCanonicalJson) {
    const Fixture f = make_fixture("partsmap");
    const sc::SetEntry* mapped = nullptr;
    for (const sc::SetEntry& e : f.set.entries) {
        if (e.id == "a:drone.mapped") {
            mapped = &e;
        }
    }
    ASSERT_NE(mapped, nullptr);
    // Keys sorted, no whitespace, null kept, numbers shortest.
    EXPECT_EQ(mapped->parts_map_json, R"([{"count":1,"hash":null,"part":"a:part.motor.fpv","placement":{"pos":[0,0,0],"quat":[1,0,0,0]},"slot":"frame"}])");
    const std::string expected =
        "sha256:" + spade::sha256_hex("parts_map\t" + spade::sha256_hex(mapped->parts_map_json) + "\n");
    const auto h = sc::entry_hash(f.set, *mapped);
    ASSERT_TRUE(h.has_value()) << h.error().message;
    EXPECT_EQ(*h, expected);
}

TEST(LibraryQuery, ThePeekReadsAPartsMassAndAModelsMassAndRotorsAndAWorldsVersion) {
    const Fixture f = make_fixture("peek");
    const auto page = sc::list_entries(f.set, {});
    ASSERT_TRUE(page.has_value());
    const sc::ListedEntry* fpv = row(*page, "a:part.motor.fpv");
    ASSERT_NE(fpv, nullptr);
    ASSERT_TRUE(fpv->peek.has_value());
    ASSERT_TRUE(fpv->peek->mass_kg.has_value());
    EXPECT_DOUBLE_EQ(*fpv->peek->mass_kg, 0.032);
    EXPECT_TRUE(fpv->peek->install_kinds.empty());

    const sc::ListedEntry* quad = row(*page, "a:drone.quad");
    ASSERT_NE(quad, nullptr);
    ASSERT_TRUE(quad->peek.has_value());
    EXPECT_DOUBLE_EQ(*quad->peek->mass_kg, 0.6);
    EXPECT_EQ(quad->peek->rotor_count, std::optional<uint32_t>(4));

    const sc::ListedEntry* pad = row(*page, "a:world.pad");
    ASSERT_NE(pad, nullptr);
    ASSERT_TRUE(pad->peek.has_value());
    EXPECT_EQ(pad->peek->world_version, std::optional<uint32_t>(3));
    EXPECT_FALSE(pad->peek->mass_kg.has_value());

    // part_version 2 keys: provides, limits, install kinds (sorted, de-duplicated).
    const sc::ListedEntry* big = row(*page, "a:part.motor.big");
    ASSERT_NE(big, nullptr);
    ASSERT_TRUE(big->peek.has_value());
    EXPECT_FALSE(big->peek->mass_kg.has_value());
    EXPECT_EQ(big->peek->provides, std::vector<std::string>{"thrust.vertical"});
    ASSERT_EQ(big->peek->limits.size(), 1u);
    EXPECT_DOUBLE_EQ(big->peek->limits.at("current_max_a"), 40.0);
    EXPECT_EQ(big->peek->install_kinds, (std::vector<std::string>{"motor", "prop_hub"}));

    // Meshes, opaque entries and a file-less drone have an empty peek, not a failed one.
    for (const char* id : {"a:mesh.marker", "a:gates.corridor", "a:drone.mapped"}) {
        const sc::ListedEntry* r = row(*page, id);
        ASSERT_NE(r, nullptr) << id;
        ASSERT_TRUE(r->peek.has_value()) << id;
        EXPECT_FALSE(r->peek->mass_kg.has_value()) << id;
        EXPECT_FALSE(r->peek->rotor_count.has_value()) << id;
    }
    EXPECT_TRUE(page->warnings.empty());
    for (const sc::ListedEntry& r : page->rows) {
        EXPECT_EQ(r.hash.rfind("sha256:", 0), 0u) << r.entry->id;
    }
}

TEST(LibraryQuery, AFileThatCannotBeParsedGivesNoPeekAndAWarningButStillLists) {
    Fixture f = make_fixture("broken");
    put(f.base / "a/motors/fpv.motor.yaml", "part_version: 1\nmotor: [unclosed\n");
    const auto page = sc::list_entries(f.set, {});
    ASSERT_TRUE(page.has_value());
    EXPECT_EQ(page->total, 8u);
    const sc::ListedEntry* r = row(*page, "a:part.motor.fpv");
    ASSERT_NE(r, nullptr);
    EXPECT_FALSE(r->peek.has_value());
    EXPECT_EQ(r->hash.rfind("sha256:", 0), 0u);  // the bytes were readable; only the parse failed
    int warned = 0;
    for (const sc::LibraryRefusal& w : page->warnings) {
        if (w.code == "peek_failed" && w.id == "a:part.motor.fpv") {
            ++warned;
            EXPECT_FALSE(w.path.empty());
        }
    }
    EXPECT_EQ(warned, 1);

    // A file that is gone: no hash, an io_error warning, no peek.
    fs::remove(f.base / "a/motors/fpv.motor.yaml");
    const auto gone = sc::list_entries(f.set, {});
    ASSERT_TRUE(gone.has_value());
    const sc::ListedEntry* g = row(*gone, "a:part.motor.fpv");
    ASSERT_NE(g, nullptr);
    EXPECT_TRUE(g->hash.empty());
    EXPECT_FALSE(g->peek.has_value());
    bool io = false;
    for (const sc::LibraryRefusal& w : gone->warnings) {
        io = io || (w.code == "io_error" && w.id == "a:part.motor.fpv");
    }
    EXPECT_TRUE(io);
}

TEST(LibraryQuery, TheRepositorysDefaultLibraryListsWithPeeksAndUniqueHashes) {
    const fs::path dir = fs::path(SPADE_ASSETS_DIR) / "library";
    const std::vector<sc::LibraryRootSpec> roots = {{"default", dir, false}};
    const auto set = sc::open_libraries(roots);
    ASSERT_TRUE(set.has_value()) << set.error().message;
    const auto all = sc::list_entries(*set, {});
    ASSERT_TRUE(all.has_value()) << all.error().message;
    EXPECT_EQ(all->total, set->entries.size());
    EXPECT_TRUE(all->warnings.empty());
    std::vector<std::string> hashes;
    for (const sc::ListedEntry& r : all->rows) {
        EXPECT_EQ(r.hash.size(), 7u + 64u) << r.entry->id;
        hashes.push_back(r.hash);
        EXPECT_TRUE(r.peek.has_value()) << r.entry->id;
    }
    std::sort(hashes.begin(), hashes.end());
    EXPECT_EQ(std::adjacent_find(hashes.begin(), hashes.end()), hashes.end());

    sc::ListFilter motors;
    motors.kind = "part";
    motors.part_kind = "motor";
    const auto m = sc::list_entries(*set, motors);
    ASSERT_TRUE(m.has_value());
    EXPECT_GE(m->rows.size(), 1u);
    for (const sc::ListedEntry& r : m->rows) {
        ASSERT_TRUE(r.peek.has_value() && r.peek->mass_kg.has_value()) << r.entry->id;
        EXPECT_GT(*r.peek->mass_kg, 0.0);
    }
    sc::ListFilter models;
    models.kind = "model";
    const auto d = sc::list_entries(*set, models);
    ASSERT_TRUE(d.has_value());
    EXPECT_GE(d->rows.size(), 1u);
    for (const sc::ListedEntry& r : d->rows) {
        ASSERT_TRUE(r.peek.has_value() && r.peek->rotor_count.has_value()) << r.entry->id;
    }
}

// A blocks-only part (part_version 2: install points, meshes, shapes) has its mass in mass_properties.mass, not in a motor/prop/esc/battery block: the peek reads it.
TEST(LibraryQuery, APeekReadsABlocksOnlyPartsMass) {
    const fs::path dir = fresh("blocks_peek");
    put(dir / "p.motor.yaml", "part_version: 2\nid: \"part.motor.x\"\nmass_properties: {mass: 0.055}\n");
    sc::SetEntry entry;
    entry.id = "default:part.motor.x";
    entry.kind = "part";
    entry.part_kind = "motor";
    entry.file = dir / "p.motor.yaml";
    std::string why;
    const auto peek = sc::peek_entry(entry, &why);
    ASSERT_TRUE(peek.has_value()) << why;
    ASSERT_TRUE(peek->mass_kg.has_value()) << "a version-2 part's mass is in mass_properties";
    EXPECT_DOUBLE_EQ(*peek->mass_kg, 0.055);
}

}  // namespace
