// ---------------------------------------------------------------------------
// test_library_set.cpp -- N library roots opened together (scene/library_set.hpp):
// the v3 index and entry layout, namespaced ids, aliases and the structured
// refusals. The shapes are the contract's (docs/design/core/plans/2026-11-17-
// spade-library-json-contract.md sections 2 and 3). Every test writes its own
// roots under the temp folder, so none depends on another or on the repository's
// assets, except the last, which opens the real default library.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "scene/library_set.hpp"
#include "unique_temp.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;

// A fresh, empty folder with a per-process unique name (tests/unique_temp.hpp),
// removed with everything in it when the test process ends.
fs::path fresh(const std::string& name) {
    static std::vector<std::unique_ptr<spade::test::TempDir>> dirs;
    dirs.push_back(std::make_unique<spade::test::TempDir>("library_set_" + name));
    return dirs.back()->path();
}

void put(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

// A version 2 root with one drone and one motor.
fs::path v2_root(const fs::path& dir) {
    put(dir / "library.yaml",
        "library_version: 2\n"
        "entries:\n"
        "  - name: \"Reference quad\"\n"
        "    category: \"Drones\"\n"
        "    file: \"drones/reference-quad.model.yaml\"\n"
        "  - name: \"FPV 2306\"\n"
        "    category: \"Motors\"\n"
        "    file: \"motors/fpv_2306.motor.yaml\"\n");
    put(dir / "drones/reference-quad.model.yaml", "x\n");
    put(dir / "motors/fpv_2306.motor.yaml", "x\n");
    return dir;
}

// A version 3 root named `name` that depends on `depends`, with one drone index.
fs::path v3_root(const fs::path& dir, const std::string& name, const std::string& depends,
                 const std::string& extra_root_keys, const std::string& entries) {
    put(dir / "library.yaml",
        "library_version: 3\nname: " + name + "\ndepends: [" + depends + "]\n" + extra_root_keys +
            "indexes:\n  - drones.yaml\n");
    put(dir / "drones.yaml", "index_version: 1\nentries:\n" + entries);
    return dir;
}

const std::string kDrone =
    "  - id: drone.x500\n"
    "    name: x500\n"
    "    category: Drones\n"
    "    file: drones/x500.model.yaml\n";

sc::LibraryResult<sc::LibrarySet> open(std::vector<sc::LibraryRootSpec> roots) {
    return sc::open_libraries(roots);
}

// The first issue of an open_failed refusal that has `code`, or null.
const sc::LibraryRefusal* issue(const sc::LibraryRefusal& r, const std::string& code) {
    for (const sc::LibraryRefusal& i : r.issues) {
        if (i.code == code) {
            return &i;
        }
    }
    return nullptr;
}

const sc::SetEntry* entry(const sc::LibrarySet& set, const std::string& id) {
    for (const sc::SetEntry& e : set.entries) {
        if (e.id == id) {
            return &e;
        }
    }
    return nullptr;
}

TEST(LibrarySet, AVersion2RootOpensUnderTheHostsNameWithDerivedIds) {
    const fs::path dir = v2_root(fresh("v2"));
    const auto set = open({{"default", dir, false}});
    ASSERT_TRUE(set.has_value()) << set.error().message;
    ASSERT_EQ(set->roots.size(), 1u);
    EXPECT_EQ(set->roots[0].name, "default");
    EXPECT_EQ(set->roots[0].library_version, 2u);
    EXPECT_EQ(set->roots[0].entry_count, 2u);
    ASSERT_EQ(set->entries.size(), 2u);
    // `-` and `.` in the stem become `_`; the kind is the extension's.
    EXPECT_EQ(set->entries[0].id, "default:model.reference_quad");
    EXPECT_EQ(set->entries[0].kind, "model");
    EXPECT_EQ(set->entries[0].name, "Reference quad");
    EXPECT_EQ(set->entries[0].category, "Drones");
    EXPECT_EQ(set->entries[0].file, dir / "drones/reference-quad.model.yaml");
    EXPECT_EQ(set->entries[1].id, "default:part.motor.fpv_2306");
    EXPECT_EQ(set->entries[1].kind, "part");
    EXPECT_EQ(set->entries[1].part_kind, "motor");
}

TEST(LibrarySet, AVersion3RootReadsItsIndexesAliasesAndEntryFields) {
    const fs::path base = fresh("v3");
    v2_root(base / "default");
    const fs::path kat = v3_root(base / "kat", "kat", "default",
                                 "aliases:\n  \"mesh:drone/x500-body\": kat:drone.x500\n",
                                 "  - id: drone.x500\n"
                                 "    name: x500\n"
                                 "    category: Drones\n"
                                 "    file: drones/x500.model.yaml\n"
                                 "    tags: [quad, five_inch]\n"
                                 "    thumbnail: thumbnails/x500.png\n"
                                 "    requires: [default:part.motor.fpv_2306]\n"
                                 "    files: [drones/x500.notes.txt]\n"
                                 "    generated_from: {recipe: gate-corridor, hash: \"sha256:abc\"}\n"
                                 "    parts_map: []\n");
    put(kat / "drones/x500.model.yaml", "x\n");
    put(kat / "thumbnails/x500.png", "x\n");
    put(kat / "drones/x500.notes.txt", "x\n");
    const auto set = open({{"default", base / "default", false}, {"kat", kat, true}});
    ASSERT_TRUE(set.has_value()) << set.error().message;
    ASSERT_EQ(set->roots.size(), 2u);
    EXPECT_EQ(set->roots[1].name, "kat");
    EXPECT_EQ(set->roots[1].library_version, 3u);
    EXPECT_EQ(set->roots[1].depends, std::vector<std::string>{"default"});
    EXPECT_TRUE(set->roots[1].writable);
    EXPECT_FALSE(set->roots[0].writable);
    const sc::SetEntry* e = entry(*set, "kat:drone.x500");
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->root, "kat");
    EXPECT_EQ(e->local_id, "drone.x500");
    EXPECT_EQ(e->kind, "drone");
    EXPECT_EQ(e->tags, (std::vector<std::string>{"quad", "five_inch"}));
    EXPECT_EQ(e->rel_thumbnail, "thumbnails/x500.png");
    EXPECT_EQ(e->thumbnail, kat / "thumbnails/x500.png");
    EXPECT_EQ(e->requires_ids, std::vector<std::string>{"default:part.motor.fpv_2306"});
    EXPECT_EQ(e->files, std::vector<std::string>{"drones/x500.notes.txt"});
    EXPECT_EQ(e->generated_recipe, "gate-corridor");
    EXPECT_EQ(e->generated_hash, "sha256:abc");
    EXPECT_TRUE(e->has_parts_map);
    EXPECT_EQ(set->aliases.at("mesh:drone/x500-body"), "kat:drone.x500");
}

TEST(LibrarySet, RootsAndEntriesKeepTheHostsOrder) {
    const fs::path base = fresh("order");
    v2_root(base / "a");
    v2_root(base / "b");
    const auto set = open({{"zeta", base / "b", false}, {"alpha", base / "a", false}});
    ASSERT_TRUE(set.has_value()) << set.error().message;
    EXPECT_EQ(set->roots[0].name, "zeta");
    EXPECT_EQ(set->roots[1].name, "alpha");
    ASSERT_EQ(set->entries.size(), 4u);
    EXPECT_EQ(set->entries[0].id, "zeta:model.reference_quad");
    EXPECT_EQ(set->entries[1].id, "zeta:part.motor.fpv_2306");
    EXPECT_EQ(set->entries[2].id, "alpha:model.reference_quad");
}

TEST(LibrarySet, ARefusalIsOpenFailedWithOneIssuePerProblemAndNothingIsOpenedPartly) {
    const fs::path base = fresh("collect");
    v2_root(base / "good");
    v2_root(base / "bad");
    put(base / "bad/library.yaml", "library_version: 9\nentries: []\n");
    const auto set = open({{"good", base / "good", false},
                           {"gone", base / "gone", false},
                           {"bad", base / "bad", false}});
    ASSERT_FALSE(set.has_value());
    EXPECT_EQ(set.error().code, "open_failed");
    const sc::LibraryRefusal* gone = issue(set.error(), "root_unavailable");
    ASSERT_NE(gone, nullptr);
    EXPECT_EQ(gone->root, "gone");
    EXPECT_EQ(gone->path, (base / "gone").string());
    const sc::LibraryRefusal* bad = issue(set.error(), "bad_index");
    ASSERT_NE(bad, nullptr);
    EXPECT_EQ(bad->root, "bad");
    EXPECT_EQ(set.error().issues.size(), 2u);  // the good root contributes none
}

TEST(LibrarySet, RootNamesAreCheckedAndDistinct) {
    const fs::path base = fresh("names");
    v2_root(base / "a");
    const auto twice = open({{"d", base / "a", false}, {"d", base / "a", false}});
    ASSERT_FALSE(twice.has_value());
    ASSERT_NE(issue(twice.error(), "duplicate_root"), nullptr);
    EXPECT_EQ(issue(twice.error(), "duplicate_root")->root, "d");
    for (const char* bad : {"", "Kat", "1kat", "k-at", "k at"}) {
        const auto r = open({{bad, base / "a", false}});
        ASSERT_FALSE(r.has_value()) << bad;
        EXPECT_NE(issue(r.error(), "invalid_request"), nullptr) << bad;
    }
}

TEST(LibrarySet, AVersion3NameThatIsNotTheHostsIsRefused) {
    const fs::path base = fresh("mismatch");
    const fs::path kat = v3_root(base / "kat", "kat", "", "", kDrone);
    put(kat / "drones/x500.model.yaml", "x\n");
    const auto set = open({{"other", kat, false}});
    ASSERT_FALSE(set.has_value());
    const sc::LibraryRefusal* r = issue(set.error(), "root_name_mismatch");
    ASSERT_NE(r, nullptr);
    EXPECT_EQ(r->root, "other");
    EXPECT_EQ(r->path, kat.string());
}

TEST(LibrarySet, DependenciesMustBeOpenedAndAcyclic) {
    const fs::path base = fresh("deps");
    const fs::path a = v3_root(base / "a", "a", "b", "", kDrone);
    const fs::path b = v3_root(base / "b", "b", "a", "", kDrone);
    put(a / "drones/x500.model.yaml", "x\n");
    put(b / "drones/x500.model.yaml", "x\n");
    const auto cycle = open({{"a", a, false}, {"b", b, false}});
    ASSERT_FALSE(cycle.has_value());
    EXPECT_NE(issue(cycle.error(), "dependency_cycle"), nullptr);

    const auto missing = open({{"a", a, false}});
    ASSERT_FALSE(missing.has_value());
    const sc::LibraryRefusal* m = issue(missing.error(), "dependency_missing");
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(m->root, "a");
    EXPECT_NE(m->message.find("'b'"), std::string::npos) << m->message;
}

TEST(LibrarySet, ARootMayNameOnlyItsOwnAndItsDependenciesRoots) {
    const fs::path base = fresh("foreign");
    // `default` names a kat id; `kat` names a default id but does not depend on it.
    const fs::path def = v3_root(base / "default", "default", "", "",
                                 kDrone + "    requires: [kat:drone.y]\n");
    const fs::path kat = v3_root(base / "kat", "kat", "", "",
                                 "  - id: drone.y\n    name: y\n    category: Drones\n"
                                 "    file: drones/y.model.yaml\n"
                                 "    requires: [default:drone.x500]\n");
    put(def / "drones/x500.model.yaml", "x\n");
    put(kat / "drones/y.model.yaml", "x\n");
    const auto set = open({{"default", def, false}, {"kat", kat, false}});
    ASSERT_FALSE(set.has_value());
    int foreign = 0;
    for (const sc::LibraryRefusal& i : set.error().issues) {
        foreign += i.code == "foreign_root_reference" ? 1 : 0;
    }
    EXPECT_EQ(foreign, 2);
    const sc::LibraryRefusal* r = issue(set.error(), "foreign_root_reference");
    ASSERT_NE(r, nullptr);
    EXPECT_FALSE(r->id.empty());
    EXPECT_FALSE(r->root.empty());
}

TEST(LibrarySet, RequiresAreFullIdsOfExistingEntries) {
    const fs::path base = fresh("requires");
    const fs::path r = v3_root(base / "r", "r", "", "", kDrone + "    requires: [part.motor.x]\n");
    put(r / "drones/x500.model.yaml", "x\n");
    const auto bare = open({{"r", r, false}});
    ASSERT_FALSE(bare.has_value());
    EXPECT_NE(issue(bare.error(), "bare_id"), nullptr);

    const fs::path s = v3_root(base / "s", "s", "", "", kDrone + "    requires: [s:part.motor.nothere]\n");
    put(s / "drones/x500.model.yaml", "x\n");
    const auto absent = open({{"s", s, false}});
    ASSERT_FALSE(absent.has_value());
    const sc::LibraryRefusal* i = issue(absent.error(), "bad_index");
    ASSERT_NE(i, nullptr);
    EXPECT_EQ(i->field, "entries[0].requires[0]");
}

TEST(LibrarySet, AliasesAreMergedAndAConflictOrAnIdKeyIsRefused) {
    const fs::path base = fresh("aliases");
    const fs::path a = v3_root(base / "a", "a", "", "aliases:\n  \"mesh:x\": a:drone.x500\n", kDrone);
    const fs::path b = v3_root(base / "b", "b", "", "aliases:\n  \"mesh:y\": b:drone.x500\n", kDrone);
    put(a / "drones/x500.model.yaml", "x\n");
    put(b / "drones/x500.model.yaml", "x\n");
    const auto both = open({{"a", a, false}, {"b", b, false}});
    ASSERT_TRUE(both.has_value()) << both.error().message;
    EXPECT_EQ(both->aliases.size(), 2u);
    EXPECT_EQ(both->aliases.at("mesh:y"), "b:drone.x500");

    // The same key in two roots.
    const fs::path c = v3_root(base / "c", "c", "", "aliases:\n  \"mesh:x\": c:drone.x500\n", kDrone);
    put(c / "drones/x500.model.yaml", "x\n");
    const auto clash = open({{"a", a, false}, {"c", c, false}});
    ASSERT_FALSE(clash.has_value());
    EXPECT_NE(issue(clash.error(), "alias_conflict"), nullptr);

    // A key that is also an id.
    const fs::path d = v3_root(base / "d", "d", "", "aliases:\n  \"d:drone.x500\": d:drone.x500\n", kDrone);
    put(d / "drones/x500.model.yaml", "x\n");
    const auto idkey = open({{"d", d, false}});
    ASSERT_FALSE(idkey.has_value());
    EXPECT_NE(issue(idkey.error(), "alias_conflict"), nullptr);

    // An alias to an entry that does not exist, and one to a bare id.
    const fs::path e = v3_root(base / "e", "e", "", "aliases:\n  \"mesh:z\": e:drone.nothere\n", kDrone);
    put(e / "drones/x500.model.yaml", "x\n");
    EXPECT_NE(issue(open({{"e", e, false}}).error(), "bad_index"), nullptr);
    const fs::path f = v3_root(base / "f", "f", "", "aliases:\n  \"mesh:z\": drone.x500\n", kDrone);
    put(f / "drones/x500.model.yaml", "x\n");
    EXPECT_NE(issue(open({{"f", f, false}}).error(), "bare_id"), nullptr);
}

TEST(LibrarySet, AnIndexIsStrictAndEveryBreachNamesItsField) {
    const fs::path base = fresh("strict");
    auto breach = [&](const std::string& name, const std::string& entries) {
        const fs::path r = v3_root(base / name, name, "", "", entries);
        put(r / "drones/x500.model.yaml", "x\n");
        const auto set = open({{name, r, false}});
        EXPECT_FALSE(set.has_value()) << name;
        if (set.has_value()) {
            return std::string();
        }
        const sc::LibraryRefusal* i = issue(set.error(), "bad_index");
        EXPECT_NE(i, nullptr) << name;
        return i != nullptr ? i->field : std::string();
    };
    EXPECT_EQ(breach("unknown", kDrone + "    mystery: 1\n"), "entries[0].mystery");
    EXPECT_EQ(breach("noname", "  - id: drone.x500\n    category: Drones\n    file: drones/x500.model.yaml\n"),
              "entries[0].name");
    EXPECT_EQ(breach("dupid", kDrone + "  - id: drone.x500\n    name: other\n    category: Drones\n"
                                       "    file: drones/x500.model.yaml\n"),
              "entries[1].id");
    EXPECT_EQ(breach("dupname", kDrone + "  - id: drone.y\n    name: X500\n    category: Drones\n"
                                         "    file: drones/x500.model.yaml\n"),
              "entries[1].name");
    EXPECT_EQ(breach("badid", "  - id: Drone-X\n    name: x\n    category: Drones\n    file: drones/x500.model.yaml\n"),
              "entries[0].id");
    EXPECT_EQ(breach("kind", "  - id: part.motor.x\n    name: x\n    category: Drones\n"
                             "    file: drones/x500.model.yaml\n"),
              "entries[0].file");
    EXPECT_EQ(breach("dots", "  - id: drone.x\n    name: x\n    category: Drones\n    file: ../x500.model.yaml\n"),
              "entries[0].file");
    EXPECT_EQ(breach("nofile", "  - id: drone.x\n    name: x\n    category: Drones\n    file: drones/none.model.yaml\n"),
              "entries[0].file");
    EXPECT_EQ(breach("nothumb", kDrone + "    thumbnail: thumbnails/none.png\n"), "entries[0].thumbnail");
    EXPECT_EQ(breach("nocompanion", kDrone + "    files: [drones/none.txt]\n"), "entries[0].files[0]");
    EXPECT_EQ(breach("tagcase", kDrone + "    tags: [Quad]\n"), "entries[0].tags[0]");
    EXPECT_EQ(breach("nofileoparts", "  - id: drone.x\n    name: x\n    category: Drones\n"), "entries[0].file");
}

TEST(LibrarySet, AnOpaqueEntryNeedsNoKnownKindAndKeepsItsType) {
    const fs::path base = fresh("opaque");
    const fs::path r = v3_root(base / "r", "r", "", "",
                               "  - id: gateset.corridor\n"
                               "    name: corridor gates\n"
                               "    category: Gates\n"
                               "    type: some.gate_set.v5\n"
                               "    file: worlds/corridor.gates.json\n");
    put(r / "worlds/corridor.gates.json", "{}\n");
    const auto set = open({{"r", r, false}});
    ASSERT_TRUE(set.has_value()) << set.error().message;
    const sc::SetEntry* e = entry(*set, "r:gateset.corridor");
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->kind, "opaque");
    EXPECT_EQ(e->type, "some.gate_set.v5");

    // An opaque entry may not borrow a Spade kind's name as its id prefix.
    const fs::path s = v3_root(base / "s", "s", "", "",
                               "  - id: world.x\n    name: x\n    category: Gates\n    type: t\n"
                               "    file: worlds/corridor.gates.json\n");
    put(s / "worlds/corridor.gates.json", "{}\n");
    const auto bad = open({{"s", s, false}});
    ASSERT_FALSE(bad.has_value());
    EXPECT_NE(issue(bad.error(), "bad_index"), nullptr);
}

TEST(LibrarySet, AVersion1Or2RootsKeepTheirOwnRulesAndAPathRuleBreachIsRefused) {
    const fs::path base = fresh("old");
    // A part entry in a version 1 index is refused, as scene/library.hpp refuses it.
    put(base / "library.yaml",
        "library_version: 1\nentries:\n  - name: m\n    category: c\n    file: motors/m.motor.yaml\n");
    put(base / "motors/m.motor.yaml", "x\n");
    const auto set = open({{"default", base, false}});
    ASSERT_FALSE(set.has_value());
    EXPECT_NE(issue(set.error(), "bad_index"), nullptr);

    // A version this build does not read.
    put(base / "library.yaml", "library_version: 9\nentries: []\n");
    const auto future = open({{"default", base, false}});
    ASSERT_FALSE(future.has_value());
    EXPECT_NE(issue(future.error(), "bad_index"), nullptr);
}

TEST(LibrarySet, TheRepositorysDefaultLibraryOpensWithDerivedIdsForEveryEntry) {
    const fs::path dir = fs::path(SPADE_ASSETS_DIR) / "library";
    const auto set = open({{"default", dir, false}});
    ASSERT_TRUE(set.has_value()) << set.error().message;
    EXPECT_EQ(set->roots[0].library_version, 2u);
    EXPECT_GE(set->entries.size(), 30u);
    // Every id is namespaced and unique, and every entry's file exists.
    std::vector<std::string> ids;
    for (const sc::SetEntry& e : set->entries) {
        EXPECT_EQ(e.id.rfind("default:", 0), 0u) << e.id;
        EXPECT_TRUE(fs::exists(e.file)) << e.file;
        ids.push_back(e.id);
    }
    std::sort(ids.begin(), ids.end());
    EXPECT_EQ(std::adjacent_find(ids.begin(), ids.end()), ids.end());
}

// L88l: a frame and a payload (avionics, camera, computer, radio, mast) are part kinds beside motor, prop, esc and battery.
TEST(LibrarySet, AFrameAndAPayloadArePartKinds) {
    const std::string index =
        "  - id: part.frame.x500_frame\n    name: x500 frame\n    category: Parts\n    file: parts/x500_frame.frame.yaml\n"
        "  - id: part.payload.pixhawk_6c\n    name: Pixhawk 6C\n    category: Parts\n    file: parts/pixhawk_6c.payload.yaml\n";
    const fs::path dir = v3_root(fresh("l88l_kinds"), "l88l", "", "", index);
    put(dir / "parts/x500_frame.frame.yaml", "x\n");
    put(dir / "parts/pixhawk_6c.payload.yaml", "x\n");
    const auto set = open({{"l88l", dir, false}});
    ASSERT_TRUE(set.has_value()) << set.error().message;
    const sc::SetEntry* frame = entry(*set, "l88l:part.frame.x500_frame");
    ASSERT_NE(frame, nullptr);
    EXPECT_EQ(frame->kind, "part");
    EXPECT_EQ(frame->part_kind, "frame");
    const sc::SetEntry* payload = entry(*set, "l88l:part.payload.pixhawk_6c");
    ASSERT_NE(payload, nullptr);
    EXPECT_EQ(payload->part_kind, "payload");

    // the file's suffix is the kind's: a payload in a .frame.yaml is refused by its file
    const fs::path bad = v3_root(fresh("l88l_kinds_bad"), "l88lbad", "", "",
                                 "  - id: part.payload.x\n    name: x\n    category: Parts\n    file: parts/x.frame.yaml\n");
    put(bad / "parts/x.frame.yaml", "x\n");
    const auto refused = open({{"l88lbad", bad, false}});
    ASSERT_FALSE(refused.has_value());
    const sc::LibraryRefusal* i = issue(refused.error(), "bad_index");
    ASSERT_NE(i, nullptr) << refused.error().message;
    EXPECT_EQ(i->field, "entries[0].file");
}

}  // namespace
