// ---------------------------------------------------------------------------
// test_library_resolve.cpp -- resolve_alias and hash_text (scene/library_query.hpp; L88l, the library contract's section 4).
//
//   LibraryResolve.AnAliasResolvesToItsFullIdAndTheRootOfThatId        an alias key is any non-empty string (it need not contain ':'),
//                                                  and the result is {id, root} with the root the id's own
//   LibraryResolve.ARootLimitsTheLookupToThatRootsOwnTable             `root` names the table to read: an alias written in another
//                                                  root is not found there, even though the set has it
//   LibraryResolve.TheRefusalsAreNamed                                 entry_not_found (field alias), root_not_loaded (field root),
//                                                  invalid_request for an empty alias
//   LibraryResolve.HashTextIsTheSha256OfTheBytesAsGiven                known vectors; no newline translation; UTF-8 bytes
// Every test writes its own roots under unique temp folders.
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

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;

fs::path fresh(const std::string& name) {
    static std::vector<std::unique_ptr<spade::test::TempDir>> dirs;
    dirs.push_back(std::make_unique<spade::test::TempDir>("library_resolve_" + name));
    return dirs.back()->path();
}

void put(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

// `a` holds a motor and the alias "mesh:drone/x500-body" for it; `b` (depends on a) holds a drone, the alias "gate-corridor" for it and
// the alias "legacy-motor" for a's motor.
sc::LibrarySet make_set(const std::string& name) {
    const fs::path base = fresh(name);
    const fs::path a = base / "a";
    const fs::path b = base / "b";
    put(a / "library.yaml",
        "library_version: 3\nname: a\ndepends: []\naliases:\n  \"mesh:drone/x500-body\": \"a:part.motor.fpv\"\nindexes:\n  - index.yaml\n");
    put(a / "index.yaml",
        "index_version: 1\nentries:\n  - id: part.motor.fpv\n    name: FPV motor\n    category: Motors\n    file: motors/fpv.motor.yaml\n");
    put(a / "motors/fpv.motor.yaml", "part_version: 1\nmotor:\n  kv: 2000\n  mass: 0.032\n");
    put(b / "library.yaml",
        "library_version: 3\nname: b\ndepends: [a]\naliases:\n  gate-corridor: \"b:drone.quad\"\n  legacy-motor: \"a:part.motor.fpv\"\nindexes:\n  - index.yaml\n");
    put(b / "index.yaml",
        "index_version: 1\nentries:\n  - id: drone.quad\n    name: Quad\n    category: Drones\n    file: drones/quad.model.yaml\n");
    put(b / "drones/quad.model.yaml", "x\n");
    const std::vector<sc::LibraryRootSpec> roots = {{"a", a, false}, {"b", b, false}};
    auto set = sc::open_libraries(roots);
    EXPECT_TRUE(set.has_value()) << (set.has_value() ? "" : set.error().message);
    return set.has_value() ? std::move(*set) : sc::LibrarySet{};
}

TEST(LibraryResolve, AnAliasResolvesToItsFullIdAndTheRootOfThatId) {
    const sc::LibrarySet set = make_set("plain");
    const auto colon = sc::resolve_alias(set, "mesh:drone/x500-body");
    ASSERT_TRUE(colon.has_value()) << colon.error().message;
    EXPECT_EQ(colon->id, "a:part.motor.fpv");
    EXPECT_EQ(colon->root, "a");
    const auto plain = sc::resolve_alias(set, "gate-corridor", "");
    ASSERT_TRUE(plain.has_value()) << plain.error().message;
    EXPECT_EQ(plain->id, "b:drone.quad");
    EXPECT_EQ(plain->root, "b");
    // an alias that names another root's entry: the result's root is that entry's
    const auto cross = sc::resolve_alias(set, "legacy-motor");
    ASSERT_TRUE(cross.has_value()) << cross.error().message;
    EXPECT_EQ(cross->id, "a:part.motor.fpv");
    EXPECT_EQ(cross->root, "a");
}

TEST(LibraryResolve, ARootLimitsTheLookupToThatRootsOwnTable) {
    const sc::LibrarySet set = make_set("root");
    const auto in_b = sc::resolve_alias(set, "gate-corridor", "b");
    ASSERT_TRUE(in_b.has_value()) << in_b.error().message;
    EXPECT_EQ(in_b->id, "b:drone.quad");
    const auto wrong = sc::resolve_alias(set, "gate-corridor", "a");
    ASSERT_FALSE(wrong.has_value());
    EXPECT_EQ(wrong.error().code, "entry_not_found");
    const auto own = sc::resolve_alias(set, "legacy-motor", "b");
    ASSERT_TRUE(own.has_value()) << "b's own table names an entry of a";
    EXPECT_EQ(own->id, "a:part.motor.fpv");
    EXPECT_EQ(own->root, "a");
    EXPECT_FALSE(sc::resolve_alias(set, "legacy-motor", "a").has_value());
}

TEST(LibraryResolve, TheRefusalsAreNamed) {
    const sc::LibrarySet set = make_set("refusals");
    const auto none = sc::resolve_alias(set, "no-such-alias");
    ASSERT_FALSE(none.has_value());
    EXPECT_EQ(none.error().code, "entry_not_found");
    EXPECT_EQ(none.error().field, "alias");
    EXPECT_FALSE(none.error().message.empty());
    const auto unopened = sc::resolve_alias(set, "gate-corridor", "zeta");
    ASSERT_FALSE(unopened.has_value());
    EXPECT_EQ(unopened.error().code, "root_not_loaded");
    EXPECT_EQ(unopened.error().root, "zeta");
    const auto empty = sc::resolve_alias(set, "");
    ASSERT_FALSE(empty.has_value());
    EXPECT_EQ(empty.error().code, "invalid_request");
    EXPECT_EQ(empty.error().field, "alias");
    // an id is not an alias: ids are found, not resolved
    const auto id = sc::resolve_alias(set, "a:part.motor.fpv");
    ASSERT_FALSE(id.has_value());
    EXPECT_EQ(id.error().code, "entry_not_found");
}

TEST(LibraryResolve, HashTextIsTheSha256OfTheBytesAsGiven) {
    EXPECT_EQ(sc::hash_text(""), "sha256:e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    EXPECT_EQ(sc::hash_text("abc"), "sha256:ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    // no newline translation: the bytes are hashed as given, so \n and \r\n differ
    EXPECT_NE(sc::hash_text("a\nb"), sc::hash_text("a\r\nb"));
    EXPECT_EQ(sc::hash_text("a\nb"), sc::hash_text(std::string("a\nb")));
    // UTF-8 bytes: U+00E9 is the two bytes C3 A9
    EXPECT_EQ(sc::hash_text("\xC3\xA9"), "sha256:4a99557e4033c3539de2eb65472017cad5f9557f7a0625a09f1c3f6e2ba69c4c");
}

}  // namespace
