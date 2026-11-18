// ---------------------------------------------------------------------------
// test_library_find.cpp -- finding one entry of an opened set (scene/library_query.hpp,
// find_entry): the hash and the caller's pin, a part's record as JSON, a drone's parts
// map, and the refusals; and the parts_map rules the reader enforces
// (scene/library_set.hpp). The shapes are the contract's sections 2, 4 and 5
// (docs/design/core/plans/2026-11-17-spade-library-json-contract.md). Every test
// writes its own roots under unique temp folders.
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
    dirs.push_back(std::make_unique<spade::test::TempDir>("library_find_" + name));
    return dirs.back()->path();
}

void put(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

fs::path v3_root(const fs::path& dir, const std::string& name, const std::string& depends, const std::string& entries) {
    put(dir / "library.yaml",
        "library_version: 3\nname: " + name + "\ndepends: [" + depends + "]\nindexes:\n  - index.yaml\n");
    put(dir / "index.yaml", "index_version: 1\nentries:\n" + entries);
    return dir;
}

const std::string kHashA = "sha256:" + std::string(64, 'a');

// `a` holds a motor and a world; `b` (depends on a) holds a prop and a drone with a parts map.
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
            "  - id: part.motor.fpv\n    name: FPV motor\n    category: Motors\n    file: motors/fpv.motor.yaml\n"
            "  - id: world.pad\n    name: Pad\n    category: Worlds\n    file: worlds/pad.world.yaml\n");
    put(a / "motors/fpv.motor.yaml", "part_version: 1\nmotor:\n  kv: 2000\n  mass: 0.032\nsources:\n  - id: s1\n");
    put(a / "worlds/pad.world.yaml", "world_version: 3\nname: pad\n");
    v3_root(b, "b", "a",
            "  - id: part.prop.five\n    name: Five inch\n    category: Props\n    file: props/five.prop.yaml\n"
            "  - id: drone.quad\n    name: Quad\n    category: Drones\n    parts_map:\n"
            "      - slot: frame\n        part: a:part.motor.fpv\n        hash: \"" + kHashA + "\"\n"
            "        placement: {pos: [0, 0, 0], quat: [1, 0, 0, 0]}\n"
            "      - slot: motor_fl\n        part: a:part.motor.fpv\n        parent: frame\n        install_point: mount_fl\n"
            "        adjust: {pos: [0, 0, 0.002], yaw_deg: 5, pitch_deg: -10, roll_deg: 1.5}\n"
            "      - slot: prop_fl\n        part: b:part.prop.five\n        parent: motor_fl\n        install_point: prop_hub\n"
            "        count: 2\n");
    put(b / "props/five.prop.yaml", "part_version: 1\nprop:\n  diameter: 0.127\n");
    const std::vector<sc::LibraryRootSpec> roots = {{"a", a, false}, {"b", b, false}};
    auto set = sc::open_libraries(roots);
    EXPECT_TRUE(set.has_value()) << (set.has_value() ? "" : set.error().message);
    if (set.has_value()) {
        f.set = std::move(*set);
    }
    return f;
}

const sc::LibraryRefusal* issue(const sc::LibraryRefusal& r, const std::string& code) {
    for (const sc::LibraryRefusal& i : r.issues) {
        if (i.code == code) {
            return &i;
        }
    }
    return nullptr;
}

TEST(LibraryFind, FindReturnsTheEntryItsHashAndComparesACallersPin) {
    const Fixture f = make_fixture("pin");
    sc::FindOptions none;
    const auto r = sc::find_entry(f.set, "a:part.motor.fpv", none);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    ASSERT_NE(r->entry, nullptr);
    EXPECT_EQ(r->entry->id, "a:part.motor.fpv");
    EXPECT_EQ(r->hash, *sc::entry_hash(f.set, *r->entry));
    EXPECT_FALSE(r->hash_matches.has_value());  // no pin given
    EXPECT_TRUE(r->warnings.empty());
    EXPECT_TRUE(r->record_json.empty());

    sc::FindOptions pinned;
    pinned.expect_hash = r->hash;
    const auto same = sc::find_entry(f.set, "a:part.motor.fpv", pinned);
    ASSERT_TRUE(same.has_value());
    EXPECT_EQ(same->hash_matches, std::optional<bool>(true));
    EXPECT_TRUE(same->warnings.empty());

    pinned.expect_hash = kHashA;
    const auto differs = sc::find_entry(f.set, "a:part.motor.fpv", pinned);
    ASSERT_TRUE(differs.has_value());  // the entry still comes back
    EXPECT_EQ(differs->hash_matches, std::optional<bool>(false));
    ASSERT_EQ(differs->warnings.size(), 1u);
    EXPECT_EQ(differs->warnings[0].code, "hash_mismatch");
    EXPECT_EQ(differs->warnings[0].id, "a:part.motor.fpv");

    pinned.expect_hash = "abc";  // not a hash at all
    const auto bad = sc::find_entry(f.set, "a:part.motor.fpv", pinned);
    ASSERT_FALSE(bad.has_value());
    EXPECT_EQ(bad.error().code, "invalid_request");
}

TEST(LibraryFind, RefusalsNameTheRootThePathAndTheId) {
    Fixture f = make_fixture("refusals");
    const auto bare = sc::find_entry(f.set, "part.motor.fpv", {});
    ASSERT_FALSE(bare.has_value());
    EXPECT_EQ(bare.error().code, "bare_id");
    const auto closed = sc::find_entry(f.set, "kat:drone.x500", {});
    ASSERT_FALSE(closed.has_value());
    EXPECT_EQ(closed.error().code, "root_not_loaded");
    EXPECT_EQ(closed.error().root, "kat");
    EXPECT_EQ(closed.error().id, "kat:drone.x500");
    const auto absent = sc::find_entry(f.set, "a:part.motor.nothere", {});
    ASSERT_FALSE(absent.has_value());
    EXPECT_EQ(absent.error().code, "entry_not_found");
    EXPECT_EQ(absent.error().root, "a");
    EXPECT_EQ(absent.error().id, "a:part.motor.nothere");
    // An unreadable file is io_error for the hash, naming the entry.
    fs::remove(f.base / "a/worlds/pad.world.yaml");
    const auto gone = sc::find_entry(f.set, "a:world.pad", {});
    ASSERT_FALSE(gone.has_value());
    EXPECT_EQ(gone.error().code, "io_error");
    EXPECT_EQ(gone.error().id, "a:world.pad");
}

TEST(LibraryFind, AStoreEntryWhoseFileIsMissingIsRefusedStoreFileMissing) {
    const fs::path base = fresh("store");
    const std::string sha(64, 'b');
    const fs::path r = v3_root(base / "r", "r", "",
                               "  - id: mesh.x\n    name: x\n    category: Meshes\n"
                               "    store: {sha256: \"" + sha + "\", file: x.glb}\n");
    const std::vector<sc::LibraryRootSpec> roots = {{"r", r, false}};
    const fs::path store = base / "store";
    fs::create_directories(store);
    const auto set = sc::open_libraries(roots, store);
    ASSERT_TRUE(set.has_value()) << set.error().message;
    const auto found = sc::find_entry(*set, "r:mesh.x", {});
    ASSERT_FALSE(found.has_value());
    EXPECT_EQ(found.error().code, "store_file_missing");
    EXPECT_EQ(found.error().id, "r:mesh.x");
    EXPECT_EQ(found.error().path, (store / sha / "x.glb").string());
    put(store / sha / "x.glb", "bytes");
    const auto present = sc::find_entry(*set, "r:mesh.x", {});
    ASSERT_TRUE(present.has_value()) << present.error().message;
    EXPECT_EQ(present->entry->file, store / sha / "x.glb");
}

TEST(LibraryFind, ARecordIsAPartFilesOwnMappingAsCanonicalJsonOnlyWhenAsked) {
    const Fixture f = make_fixture("record");
    sc::FindOptions with;
    with.with_record = true;
    const auto motor = sc::find_entry(f.set, "a:part.motor.fpv", with);
    ASSERT_TRUE(motor.has_value()) << motor.error().message;
    EXPECT_EQ(motor->record_json,
              R"({"motor":{"kv":2000,"mass":0.032},"part_version":1,"sources":[{"id":"s1"}]})");
    // Not a part: no record.
    const auto world = sc::find_entry(f.set, "a:world.pad", with);
    ASSERT_TRUE(world.has_value());
    EXPECT_TRUE(world->record_json.empty());
    // A file-less drone: no record either.
    const auto drone = sc::find_entry(f.set, "b:drone.quad", with);
    ASSERT_TRUE(drone.has_value());
    EXPECT_TRUE(drone->record_json.empty());
}

TEST(LibraryFind, ADronesPartsMapIsStructuredSlotByInOrder) {
    const Fixture f = make_fixture("parts");
    const auto drone = sc::find_entry(f.set, "b:drone.quad", {});
    ASSERT_TRUE(drone.has_value()) << drone.error().message;
    const std::vector<sc::PartsMapSlot>& m = drone->entry->parts_map;
    ASSERT_EQ(m.size(), 3u);
    EXPECT_EQ(m[0].slot, "frame");
    EXPECT_EQ(m[0].part, "a:part.motor.fpv");
    EXPECT_EQ(m[0].hash, kHashA);
    EXPECT_TRUE(m[0].parent.empty());
    ASSERT_TRUE(m[0].placement.has_value());
    EXPECT_EQ(m[0].placement->quat, (std::array<double, 4>{1, 0, 0, 0}));
    EXPECT_FALSE(m[0].adjust.has_value());
    EXPECT_EQ(m[0].count, 1u);
    EXPECT_EQ(m[1].slot, "motor_fl");
    EXPECT_EQ(m[1].parent, "frame");
    EXPECT_EQ(m[1].install_point, "mount_fl");
    EXPECT_FALSE(m[1].placement.has_value());
    ASSERT_TRUE(m[1].adjust.has_value());
    EXPECT_DOUBLE_EQ(m[1].adjust->pos[2], 0.002);
    EXPECT_DOUBLE_EQ(m[1].adjust->yaw_deg, 5.0);
    EXPECT_DOUBLE_EQ(m[1].adjust->pitch_deg, -10.0);
    EXPECT_DOUBLE_EQ(m[1].adjust->roll_deg, 1.5);
    EXPECT_EQ(m[2].parent, "motor_fl");
    EXPECT_EQ(m[2].install_point, "prop_hub");
    EXPECT_FALSE(m[2].adjust.has_value());
    EXPECT_EQ(m[2].count, 2u);
}

TEST(LibraryFind, PartsMapRulesNameTheirFieldAndAFreeOrSlottedSlotIsExactlyOne) {
    const fs::path base = fresh("rules");
    // A root `r` with one motor and a drone whose parts_map is `map` (indented under `parts_map:`).
    auto breach = [&](const std::string& name, const std::string& map, const std::string& code = "bad_index") {
        const fs::path r = v3_root(base / name, "r", "",
                                   "  - id: part.motor.m\n    name: m\n    category: M\n    file: m.motor.yaml\n"
                                   "  - id: drone.d\n    name: d\n    category: D\n    parts_map:\n" + map);
        put(r / "m.motor.yaml", "part_version: 1\n");
        const std::vector<sc::LibraryRootSpec> roots = {{"r", r, false}};
        const auto set = sc::open_libraries(roots);
        EXPECT_FALSE(set.has_value()) << name;
        if (set.has_value()) {
            return std::string();
        }
        const sc::LibraryRefusal* i = issue(set.error(), code);
        EXPECT_NE(i, nullptr) << name;
        return i != nullptr ? i->field : std::string();
    };
    const std::string frame = "      - slot: frame\n        part: r:part.motor.m\n"
                              "        placement: {pos: [0, 0, 0], quat: [1, 0, 0, 0]}\n";
    auto slotted = [](const std::string& slot, const std::string& parent, const std::string& extra = "") {
        return "      - slot: " + slot + "\n        part: r:part.motor.m\n        parent: " + parent +
               "\n        install_point: p\n" + extra;
    };
    const std::string p = "entries[1].parts_map[";
    EXPECT_EQ(breach("dupslot", frame + slotted("frame", "frame")), p + "1].slot");
    EXPECT_EQ(breach("unknown", frame + "        bogus: 1\n"), p + "0].bogus");
    EXPECT_EQ(breach("both", frame + slotted("a", "frame", "        placement: {pos: [0, 0, 0], quat: [1, 0, 0, 0]}\n")),
              p + "1].placement");
    EXPECT_EQ(breach("neither", frame + "      - slot: a\n        part: r:part.motor.m\n"), p + "1]");
    EXPECT_EQ(breach("noparent", frame + slotted("a", "nope")), p + "1].parent");
    EXPECT_EQ(breach("cycle", "      - slot: a\n        part: r:part.motor.m\n        parent: b\n        install_point: p\n"
                              "      - slot: b\n        part: r:part.motor.m\n        parent: a\n        install_point: p\n"),
              p + "0].parent");
    EXPECT_EQ(breach("freeadjust", frame + "        adjust: {pos: [0, 0, 0], yaw_deg: 0, pitch_deg: 0, roll_deg: 0}\n"),
              p + "0].adjust");
    EXPECT_EQ(breach("hash", frame + "        hash: abc\n"), p + "0].hash");
    EXPECT_EQ(breach("quat", "      - slot: frame\n        part: r:part.motor.m\n"
                             "        placement: {pos: [0, 0, 0], quat: [2, 0, 0, 0]}\n"),
              p + "0].placement.quat");
    EXPECT_EQ(breach("count", frame + "        count: 0\n"), p + "0].count");
    EXPECT_EQ(breach("ipnoparent", "      - slot: a\n        part: r:part.motor.m\n        install_point: p\n"), p + "0]");
    EXPECT_EQ(breach("missing", "      - slot: frame\n        part: r:part.motor.none\n"
                                "        placement: {pos: [0, 0, 0], quat: [1, 0, 0, 0]}\n"),
              p + "0].part");
    EXPECT_EQ(breach("bare", "      - slot: frame\n        part: part.motor.m\n"
                             "        placement: {pos: [0, 0, 0], quat: [1, 0, 0, 0]}\n",
                     "bare_id"),
              p + "0].part");
    EXPECT_EQ(breach("foreign", "      - slot: frame\n        part: other:part.motor.m\n"
                                "        placement: {pos: [0, 0, 0], quat: [1, 0, 0, 0]}\n",
                     "foreign_root_reference"),
              p + "0].part");
}

TEST(LibraryFind, RequestProblemsAreTopLevelRefusalsOfOpenAndReadingProblemsAreOpenFailed) {
    const fs::path base = fresh("toplevel");
    const fs::path good = v3_root(base / "good", "good", "", "");
    // The request itself is wrong: refused before any file is read, with the first problem's code at the top and
    // every request problem in `issues`.
    const std::vector<sc::LibraryRootSpec> rel = {{"good", good, false}, {"rel", fs::path("relative/dir"), false}};
    const auto r1 = sc::open_libraries(rel);
    ASSERT_FALSE(r1.has_value());
    EXPECT_EQ(r1.error().code, "relative_path");
    EXPECT_EQ(r1.error().root, "rel");
    ASSERT_EQ(r1.error().issues.size(), 1u);
    EXPECT_EQ(r1.error().issues[0].code, "relative_path");
    const std::vector<sc::LibraryRootSpec> twice = {{"good", good, false}, {"good", good, false}};
    const auto r2 = sc::open_libraries(twice);
    ASSERT_FALSE(r2.has_value());
    EXPECT_EQ(r2.error().code, "duplicate_root");
    const auto r3 = sc::open_libraries(std::vector<sc::LibraryRootSpec>{{"good", good, false}}, fs::path("rel/store"));
    ASSERT_FALSE(r3.has_value());
    EXPECT_EQ(r3.error().code, "relative_path");
    // A problem in what is read is open_failed, with one issue per problem.
    const std::vector<sc::LibraryRootSpec> gone = {{"good", good, false}, {"gone", base / "gone", false}};
    const auto r4 = sc::open_libraries(gone);
    ASSERT_FALSE(r4.has_value());
    EXPECT_EQ(r4.error().code, "open_failed");
    EXPECT_NE(issue(r4.error(), "root_unavailable"), nullptr);
}

}  // namespace
