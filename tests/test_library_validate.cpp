// ---------------------------------------------------------------------------
// test_library_validate.cpp -- validating a document's text as one of the library's
// file kinds (scene/library_validate.hpp). The shape is the contract's `validate`
// (docs/design/core/plans/2026-11-17-spade-library-json-contract.md section 4). The
// valid texts are the repository's own default-library files and one of its worlds.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "scene/library_set.hpp"
#include "scene/library_validate.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;

fs::path library_dir() {
    return fs::path(SPADE_ASSETS_DIR) / "library";
}

std::string slurp(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

sc::LibraryResult<sc::ValidationResult> validate(const std::string& as, const std::string& text,
                                                 const fs::path& base = {}) {
    sc::ValidateOptions o;
    o.as = as;
    o.base_dir = base;
    return sc::validate_text(text, o);
}

const sc::SetEntry* first_entry(const sc::LibrarySet& set, const std::string& kind, const std::string& part_kind = {}) {
    for (const sc::SetEntry& e : set.entries) {
        if (e.kind == kind && (part_kind.empty() || e.part_kind == part_kind)) {
            return &e;
        }
    }
    return nullptr;
}

TEST(LibraryValidate, EveryKindOfTheDefaultLibrarysTextValidates) {
    const std::vector<sc::LibraryRootSpec> roots = {{"default", library_dir(), false}};
    const auto set = sc::open_libraries(roots);
    ASSERT_TRUE(set.has_value()) << set.error().message;
    struct Case {
        const char* as;
        const char* kind;
        const char* part_kind;
    };
    for (const Case& c : {Case{"object", "object", ""}, Case{"part.motor", "part", "motor"},
                          Case{"part.prop", "part", "prop"}, Case{"part.esc", "part", "esc"},
                          Case{"part.battery", "part", "battery"}}) {
        const sc::SetEntry* e = first_entry(*set, c.kind, c.part_kind);
        ASSERT_NE(e, nullptr) << c.as;
        const auto r = validate(c.as, slurp(e->file));
        ASSERT_TRUE(r.has_value()) << c.as << ": " << r.error().message;
        EXPECT_TRUE(r->valid) << c.as << " " << e->id << ": " << (r->issues.empty() ? "" : r->issues[0].message);
        EXPECT_TRUE(r->issues.empty()) << c.as;
    }
    // A model, with its sidecars read from its own folder.
    const sc::SetEntry* model = first_entry(*set, "model");
    ASSERT_NE(model, nullptr);
    const auto m = validate("model", slurp(model->file), model->file.parent_path());
    ASSERT_TRUE(m.has_value()) << m.error().message;
    EXPECT_TRUE(m->valid) << (m->issues.empty() ? "" : m->issues[0].message);
    // The index (a version 2 library.yaml: the default root's old text, kept as a fixture; the root itself is version 3).
    const auto idx = validate("index", slurp(fs::path(SPADE_REPO_DIR) / "tests" / "data" / "library_default_v2.yaml"));
    ASSERT_TRUE(idx.has_value()) << idx.error().message;
    EXPECT_TRUE(idx->valid) << (idx->issues.empty() ? "" : idx->issues[0].message);
    // A world file of the repository.
    const std::string world_text = slurp(fs::path(SPADE_ASSETS_DIR) / "worlds" / "bounce_lane_0.world.yaml");
    ASSERT_FALSE(world_text.empty());
    const auto w = validate("world", world_text);
    ASSERT_TRUE(w.has_value()) << w.error().message;
    EXPECT_TRUE(w->valid) << (w->issues.empty() ? "" : w->issues[0].message);
}

TEST(LibraryValidate, ADocumentThatFailsIsAnAnsweredResultWithTheLoadersProblem) {
    const auto r = validate("part.motor", "part_version: 1\nmotor:\n  kv: abc\n");
    ASSERT_TRUE(r.has_value()) << r.error().message;  // not a refusal
    EXPECT_FALSE(r->valid);
    ASSERT_EQ(r->issues.size(), 1u);
    EXPECT_EQ(r->issues[0].code, "invalid_argument");
    EXPECT_EQ(r->issues[0].severity, "error");
    EXPECT_FALSE(r->issues[0].message.empty());
    // The loader's "(line N, column M)" becomes the position.
    ASSERT_TRUE(r->issues[0].line.has_value()) << r->issues[0].message;
    ASSERT_TRUE(r->issues[0].column.has_value());
    EXPECT_EQ(*r->issues[0].line, 3u);

    const auto future = validate("part.motor", "part_version: 9\nmotor: {}\n");
    ASSERT_TRUE(future.has_value());
    EXPECT_FALSE(future->valid);
    ASSERT_EQ(future->issues.size(), 1u);
    EXPECT_EQ(future->issues[0].code, "schema_mismatch");

    const auto empty = validate("object", "");
    ASSERT_TRUE(empty.has_value());
    EXPECT_FALSE(empty->valid);
    EXPECT_EQ(empty->issues.size(), 1u);

    const auto broken = validate("world", "world_version: [unclosed\n");
    ASSERT_TRUE(broken.has_value());
    EXPECT_FALSE(broken->valid);
}

TEST(LibraryValidate, ATextIsCheckedAgainstTheKindAskedFor) {
    const std::vector<sc::LibraryRootSpec> roots = {{"default", library_dir(), false}};
    const auto set = sc::open_libraries(roots);
    ASSERT_TRUE(set.has_value());
    const sc::SetEntry* motor = first_entry(*set, "part", "motor");
    ASSERT_NE(motor, nullptr);
    const std::string text = slurp(motor->file);
    const auto as_prop = validate("part.prop", text);
    ASSERT_TRUE(as_prop.has_value());
    EXPECT_FALSE(as_prop->valid);
    const auto as_object = validate("object", text);
    ASSERT_TRUE(as_object.has_value());
    EXPECT_FALSE(as_object->valid);
}

TEST(LibraryValidate, AModelWithMeshPartsNeedsItsBaseFolderForTheSidecars) {
    const fs::path file = library_dir() / "drones" / "basic_quad_6inch.model.yaml";
    const std::string text = slurp(file);
    ASSERT_FALSE(text.empty());
    const auto without = validate("model", text);
    ASSERT_TRUE(without.has_value());
    EXPECT_FALSE(without->valid);  // the loader refuses a mesh part with no base folder
    const auto with = validate("model", text, file.parent_path());
    ASSERT_TRUE(with.has_value());
    EXPECT_TRUE(with->valid) << (with->issues.empty() ? "" : with->issues[0].message);
}

TEST(LibraryValidate, MalformedRequestsAreRefusedByName) {
    const auto unknown = validate("widget", "x");
    ASSERT_FALSE(unknown.has_value());
    EXPECT_EQ(unknown.error().code, "invalid_request");
    EXPECT_EQ(unknown.error().field, "as");
    const auto none = validate("", "x");
    ASSERT_FALSE(none.has_value());
    EXPECT_EQ(none.error().code, "invalid_request");
    const auto rel = validate("model", "x", fs::path("relative/dir"));
    ASSERT_FALSE(rel.has_value());
    EXPECT_EQ(rel.error().code, "relative_path");
    const auto drone = validate("drone", "x");
    ASSERT_FALSE(drone.has_value());
    EXPECT_EQ(drone.error().code, "not_implemented");
}

}  // namespace
