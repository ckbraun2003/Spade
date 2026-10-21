// Spade's default library (N3 of the editor end-goal design,
// docs/design/interface/plans/2026-10-19-editor-end-goal-design.md §3): the
// index assets/library/library.yaml and its reader (scene/library.hpp), and
// the library's eight object files.
//
// What is pinned here:
//   - THE INDEX FORMAT (library_from_yaml): the version gate; unknown, missing
//     and duplicate keys at every level, blamed on "library schema v1", with
//     the dotted path and the line; names and files non-empty and unique
//     ignoring case; a file's path by the scene's relative-path rule plus no
//     '..' segment, its kind by its extension; an optional thumbnail by the
//     same path rule.
//   - ON DISK (load_library): dir is the index's directory; a missing file,
//     or a missing thumbnail an entry names, is refused naming the entry and
//     the path; each loader refuses the other kind by name.
//   - ACROSS ENTRIES (check_library_contents): every entry loads, and two
//     objects' visual mesh_refs that differ only in case are refused.
//   - THE OBJECT FILES, and their GENERATOR. Each committed
//     assets/library/<folder>/<name>.object.yaml is object_to_yaml() of the
//     ObjectFile built here from Interface's content table, byte for byte.
//     With SPADE_LIBRARY_WRITE set the same test WRITES them into the source
//     tree instead, and passes, saying so (tests/test_library_quad.cpp's
//     pattern). They are only ever written that way, never by hand. Each is
//     centred at its origin with one lambert material, library_<name>, and
//     no visual; the gate is one CSG root; no colour is the editor's
//     selection accent.
//   - THE COMMITTED INDEX: the reference quad (N4's model file) first, then
//     exactly the table's entries, in its order, and every one loads
//     (check_library_contents).
// The four primitives' colliders are compared with the editor's built-in
// ones (sandbox builtin_library()) in test_library_builtin.cpp, which builds
// only with the sandbox.
//
// LINE ENDINGS. object_to_yaml() writes LF, and the comparison reads the file
// in binary. assets/.gitattributes gives *.object.yaml and library.yaml
// `text eol=lf`, as it does the scene, world and model files, so a Windows
// checkout (core.autocrlf=true) and the leg's git archive both give LF bytes.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "render/csg_mesh.hpp"
#include "scene/library.hpp"
#include "scene/library_sources.hpp"
#include "scene/model_file.hpp"
#include "scene/object_file.hpp"
#include "scene/part_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_type.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

namespace {

namespace fs = std::filesystem;
using spade::MaterialDesc;
using spade::MaterialShading;
using spade::SdfNode;
using spade::SdfOp;
using spade::SdfPose;
using spade::SdfPrim;
using spade::SdfTransform;
using spade::scene::Library;
using spade::scene::LibraryEntry;
using spade::scene::LibraryKind;
using spade::scene::ModelFile;
using spade::scene::ObjectFile;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

// The refusal's context, or a marker when the call unexpectedly succeeded.
template <class T>
[[nodiscard]] std::string why(const spade::Result<T>& r) {
    return r ? std::string("<succeeded>") : r.error().context;
}

template <class T>
[[nodiscard]] int code_of(const spade::Result<T>& r) {
    return r ? -1 : static_cast<int>(r.error().code);
}

[[nodiscard]] constexpr int code(spade::Code c) { return static_cast<int>(c); }

// `text` with its one occurrence of `from` replaced by `to`; an anchor that is
// absent or repeated fails the calling test instead of mutating the wrong place.
[[nodiscard]] std::string with(const std::string& text, const std::string& from, const std::string& to) {
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << "anchor not found: " << from;
    EXPECT_EQ(text.find(from, at + 1), std::string::npos) << "anchor not unique: " << from;
    if (at == std::string::npos) return text;
    std::string out = text;
    out.replace(at, from.size(), to);
    return out;
}

void write_text(const fs::path& path, std::string_view text) {
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
}

[[nodiscard]] bool read_bytes(const fs::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

// The committed library, in the SOURCE tree: the generator writes here.
[[nodiscard]] fs::path library_dir() {
    return fs::path(SPADE_ASSETS_DIR) / "library";
}

// SPADE_LIBRARY_WRITE set: write the library's object files instead of
// comparing them. A read-only null check before any thread starts, as
// env_gate_is_set() in test_fp32_math.cpp.
[[nodiscard]] bool library_write_asked() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_LIBRARY_WRITE") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// The line holding byte `at` of `text` (or "<end of file>").
[[nodiscard]] std::string line_at(const std::string& text, std::size_t at) {
    if (at >= text.size()) return "<end of file>";
    std::size_t from = 0;
    if (at > 0) {
        const std::size_t newline = text.rfind('\n', at - 1);
        if (newline != std::string::npos) from = newline + 1;
    }
    const std::size_t end = text.find('\n', at);
    return text.substr(from, (end == std::string::npos ? text.size() : end) - from);
}

// Why the committed bytes are not the generator's, and what to do about it.
[[nodiscard]] std::string mismatch(const fs::path& path, const std::string& committed, const std::string& generated) {
    std::size_t i = 0;
    while (i < committed.size() && i < generated.size() && committed[i] == generated[i]) ++i;
    std::size_t line = 1;
    for (std::size_t j = 0; j < i; ++j) {
        if (committed[j] == '\n') ++line;
    }
    std::string out = spade::path_text(path) + " (" + std::to_string(committed.size()) +
                      " bytes) is not the generator's text (" + std::to_string(generated.size()) +
                      " bytes): first difference at byte " + std::to_string(i) + ", line " + std::to_string(line) +
                      ".\n  committed: " + line_at(committed, i) + "\n  generated: " + line_at(generated, i) + "\n";
    if (committed.find('\r') != std::string::npos) {
        out += "The committed file holds CR bytes: a checkout converted its line endings. assets/.gitattributes "
               "must give *.object.yaml `text eol=lf` (check with `git check-attr -a` on the path).\n";
    }
    out += "The file is generated from this test's content table; never edit it by hand. To move it: (1) name "
           "the cause (TD-1): the table or the object-file format; (2) regenerate: set SPADE_LIBRARY_WRITE=1 and "
           "run ctest -R LibraryObjects.TheCommittedFilesAreTheGeneratorsText; (3) it is final only once a fresh "
           "(-NoSeed) Docker gcc leg reproduces it (TD-12).";
    return out;
}

// ---------------------------------------------------------------------------
// INTERFACE'S CONTENT TABLE (N3): the library's eight objects, in index order.
// Metres, each centred at its origin (one identity transform, the identity
// pose); base_color RGB with alpha 1, lambert. The four primitives are the
// editor's built-in ones (sandbox builtin_library(): unit size, as the builder
// makes them; test_library_builtin.cpp compares them).
// ---------------------------------------------------------------------------

[[nodiscard]] SdfNode prim(SdfPrim kind, glm::vec4 params) {
    SdfNode n{};
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(SdfOp::none);
    n.transform = 0;
    n.params = params;
    return n;
}

[[nodiscard]] SdfNode op(SdfOp which) {
    SdfNode n{};
    n.op = static_cast<uint32_t>(which);
    return n;
}

struct ObjectRow {
    std::string_view name;      // the index's
    std::string_view category;  // Interface's
    std::string_view file;      // relative to library.yaml
    std::vector<SdfNode> nodes;  // the collider, on one identity transform
    glm::vec3 rgb;
};

[[nodiscard]] std::vector<ObjectRow> content_table() {
    return {
        {"Box", "primitives", "primitives/box.object.yaml", {prim(SdfPrim::box, {0.5f, 0.5f, 0.5f, 0.0f})},
         {0.62f, 0.64f, 0.67f}},
        {"Sphere", "primitives", "primitives/sphere.object.yaml", {prim(SdfPrim::sphere, {0.5f, 0.0f, 0.0f, 0.0f})},
         {0.93f, 0.55f, 0.20f}},
        // Axis +Y: radius 0.5, half-height 0.5.
        {"Cylinder", "primitives", "primitives/cylinder.object.yaml",
         {prim(SdfPrim::cylinder, {0.5f, 0.5f, 0.0f, 0.0f})}, {0.36f, 0.70f, 0.42f}},
        // Axis +Y: radius 0.25, half-length 0.25.
        {"Capsule", "primitives", "primitives/capsule.object.yaml",
         {prim(SdfPrim::capsule, {0.25f, 0.25f, 0.0f, 0.0f})}, {0.58f, 0.44f, 0.80f}},
        // A frame in the XY plane, flown through along Z: the outer box minus
        // the opening, one CSG root (one drawable). The opening is deeper than
        // the frame, so it cuts clean through.
        {"Gate", "obstacles", "obstacles/gate.object.yaml",
         {prim(SdfPrim::box, {0.85f, 0.85f, 0.05f, 0.0f}), prim(SdfPrim::box, {0.75f, 0.75f, 0.1f, 0.0f}),
          op(SdfOp::subtract)},
         {0.95f, 0.45f, 0.10f}},
        {"Floor tile", "floors", "floors/floor_tile.object.yaml", {prim(SdfPrim::box, {1.0f, 0.025f, 1.0f, 0.0f})},
         {0.45f, 0.47f, 0.50f}},
        // Axis +Y: radius 0.05, half-height 1.
        {"Pole", "obstacles", "obstacles/pole.object.yaml", {prim(SdfPrim::cylinder, {0.05f, 1.0f, 0.0f, 0.0f})},
         {0.90f, 0.90f, 0.88f}},
        {"Wall", "obstacles", "obstacles/wall.object.yaml", {prim(SdfPrim::box, {1.5f, 1.0f, 0.1f, 0.0f})},
         {0.70f, 0.66f, 0.60f}},
    };
}

// The editor's selection accent (sandbox/ui_theme.hpp's kAccent): no library
// colour may be it, or a placed object would read as selected.
const glm::vec3 kSelectionAccent{0.29f, 0.56f, 0.89f};

// The name as a token: lowercased, spaces as '_' ("Floor tile" -> "floor_tile").
[[nodiscard]] std::string token_of(std::string_view name) {
    std::string out;
    for (const char c : name) {
        out += (c == ' ') ? '_' : (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
    }
    return out;
}

// THE GENERATOR: a row as its object file. The object is named by the token,
// at the identity pose; its collider is the row's nodes on one identity
// transform, every node naming the object's one material, library_<token>:
// the row's colour, alpha 1, lambert. No visual: collider-only, as the
// editor's built-in primitives are.
[[nodiscard]] ObjectFile generated_object(const ObjectRow& row) {
    const std::string token = token_of(row.name);
    const std::string material = "library_" + token;
    ObjectFile f;
    f.object.name = token;
    f.object.collider.transforms = {SdfTransform{}};
    f.object.collider.nodes = row.nodes;
    f.object.collider_materials.assign(row.nodes.size(), material);
    f.materials = {MaterialDesc{material, glm::vec4(row.rgb, 1.0f), MaterialShading::lambert}};
    return f;
}

// THE INDEX the committed library.yaml must read as, in order: the reference
// quad first (N4's model file, tests/test_library_quad.cpp pins it), then the
// content table's objects.
struct IndexRow {
    std::string_view name;
    std::string_view category;
    std::string_view file;
    LibraryKind kind;
};

[[nodiscard]] std::vector<IndexRow> expected_index() {
    std::vector<IndexRow> out;
    out.push_back({"Reference quad", "drones", "drones/reference_quad.model.yaml", LibraryKind::model});
    for (const ObjectRow& row : content_table()) {
        out.push_back({row.name, row.category, row.file, LibraryKind::object});
    }
    return out;
}

// ---------------------------------------------------------------------------
// Fixtures for the format and the disk
// ---------------------------------------------------------------------------

// An index of two entries, one of each kind, the first with a thumbnail.
constexpr std::string_view kTwoEntries = R"YAML(library_version: 1
entries:
  - name: "Box"
    category: "primitives"
    file: "primitives/box.object.yaml"
    thumbnail: "thumbnails/box.png"
  - name: "Quad"
    category: "drones"
    file: "drones/quad.model.yaml"
)YAML";

[[nodiscard]] std::string two_entries() {
    return std::string(kTwoEntries);
}

// A model the model file accepts.
[[nodiscard]] ModelFile tiny_model() {
    ModelFile file;
    file.model.name = "quad";
    file.model.body.mass = 1.5f;
    file.model.proxy_radius = 0.2f;
    return file;
}

[[nodiscard]] std::string text_of(const ObjectFile& object) {
    const spade::Result<std::string> text = spade::scene::object_to_yaml(object);
    EXPECT_TRUE(text.has_value()) << why(text);
    return text.value_or("");
}

[[nodiscard]] std::string text_of(const ModelFile& model) {
    const spade::Result<std::string> text = spade::scene::model_to_yaml(model);
    EXPECT_TRUE(text.has_value()) << why(text);
    return text.value_or("");
}

// A folder holding kTwoEntries' index and the files it names.
void lay_out_two_entries(const fs::path& dir) {
    write_text(dir / "library.yaml", kTwoEntries);
    write_text(dir / "primitives" / "box.object.yaml", text_of(generated_object(content_table()[0])));
    write_text(dir / "drones" / "quad.model.yaml", text_of(tiny_model()));
    write_text(dir / "thumbnails" / "box.png", "not decoded here");
}

// An object row with a visual: a mesh_ref, drawn in its own material.
[[nodiscard]] ObjectFile with_visual(ObjectFile object, std::string mesh_ref) {
    object.object.visual = {std::move(mesh_ref), object.materials[0].name};
    return object;
}

}  // namespace

// ===========================================================================
// The index format (library_from_yaml)
// ===========================================================================

TEST(Library, TheIndexReadsAsTheFormatSpellsIt) {
    const spade::Result<Library> library = spade::scene::library_from_yaml(two_entries());
    ASSERT_OK(library);
    EXPECT_TRUE(library->dir.empty()) << "the index alone: no disk";
    ASSERT_EQ(library->entries.size(), 2u);
    const LibraryEntry& box = library->entries[0];
    EXPECT_EQ(box.name, "Box");
    EXPECT_EQ(box.category, "primitives");
    EXPECT_EQ(box.file, "primitives/box.object.yaml");
    EXPECT_EQ(box.thumbnail, "thumbnails/box.png");
    EXPECT_EQ(box.kind, LibraryKind::object);
    const LibraryEntry& quad = library->entries[1];
    EXPECT_EQ(quad.name, "Quad");
    EXPECT_EQ(quad.category, "drones");
    EXPECT_EQ(quad.file, "drones/quad.model.yaml");
    EXPECT_TRUE(quad.thumbnail.empty()) << "no thumbnail key is none";
    EXPECT_EQ(quad.kind, LibraryKind::model);

    // An empty library is a library; comments and plain scalars read as YAML.
    const spade::Result<Library> empty = spade::scene::library_from_yaml("library_version: 1\nentries: []\n");
    ASSERT_OK(empty);
    EXPECT_TRUE(empty->entries.empty());
    const spade::Result<Library> plain = spade::scene::library_from_yaml(
        "# a note\nlibrary_version: 1\nentries:\n  - {name: Pole, category: obstacles, file: pole.object.yaml}\n");
    ASSERT_OK(plain);
    ASSERT_EQ(plain->entries.size(), 1u);
    EXPECT_EQ(plain->entries[0].file, "pole.object.yaml");
}

// The library plan's AnOlderBuildRefusesVersion2: an older build runs this
// same gate against its own newest version, so the newest-plus-one case here
// is what a version 1 build said of a version 2 index.
TEST(Library, AnotherLibraryVersionIsASchemaMismatch) {
    const spade::Result<Library> newer =
        spade::scene::library_from_yaml(with(two_entries(), "library_version: 1\n", "library_version: 3\n"));
    EXPECT_EQ(code_of(newer), code(spade::Code::schema_mismatch));
    EXPECT_EQ(why(newer), "library index is schema version 3, this build reads versions 1 to 2 (line 1, column 18)");

    // The version is read first: a newer file's unknown keys say "upgrade",
    // not "unknown key".
    const spade::Result<Library> newer_keys = spade::scene::library_from_yaml(
        with(two_entries(), "library_version: 1\n", "library_version: 3\nicons: \"icons/\"\n"));
    EXPECT_EQ(code_of(newer_keys), code(spade::Code::schema_mismatch)) << why(newer_keys);

    const spade::Result<Library> zero = spade::scene::library_from_yaml("library_version: 0\nentries: []\n");
    EXPECT_EQ(code_of(zero), code(spade::Code::schema_mismatch));
}

TEST(Library, LibraryVersion1StillReads) {
    const spade::Result<Library> v1 = spade::scene::library_from_yaml(two_entries());
    ASSERT_OK(v1);
    ASSERT_EQ(v1->entries.size(), 2u);
    EXPECT_EQ(v1->entries[0].kind, LibraryKind::object);
    EXPECT_EQ(v1->entries[1].kind, LibraryKind::model);
    // A version 1 index has no part kinds.
    const spade::Result<Library> part = spade::scene::library_from_yaml(
        with(two_entries(), "drones/quad.model.yaml", "parts/m2312.motor.yaml"));
    EXPECT_EQ(code_of(part), code(spade::Code::invalid_argument));
    EXPECT_TRUE(why(part).starts_with("entries[1].file 'parts/m2312.motor.yaml' is a part file, which needs "
                                      "library_version 2"))
        << why(part);
}

TEST(Library, LibraryVersion2NamesThePartKinds) {
    EXPECT_EQ(spade::scene::kLibraryVersion, 2u);
    const spade::Result<Library> v2 = spade::scene::library_from_yaml(R"YAML(library_version: 2
entries:
  - {name: Quad, category: drones, file: drones/quad.model.yaml}
  - {name: Box, category: primitives, file: primitives/box.object.yaml}
  - {name: Motor, category: motors, file: parts/m2312.motor.yaml}
  - {name: Prop, category: props, file: parts/p10x5.prop.yaml}
  - {name: ESC, category: escs, file: parts/esc45.esc.yaml}
  - {name: Pack, category: batteries, file: parts/p4s.battery.yaml}
)YAML");
    ASSERT_OK(v2);
    ASSERT_EQ(v2->entries.size(), 6u);
    const LibraryKind kinds[] = {LibraryKind::model, LibraryKind::object, LibraryKind::motor,
                                 LibraryKind::prop,  LibraryKind::esc,    LibraryKind::battery};
    for (std::size_t i = 0; i < 6; ++i) EXPECT_EQ(v2->entries[i].kind, kinds[i]) << v2->entries[i].file;

    // A version 2 index's unknown keys are blamed on version 2.
    const spade::Result<Library> extra = spade::scene::library_from_yaml(
        "library_version: 2\nentries:\n  - {name: M, category: motors, file: m.motor.yaml, maker: x}\n");
    EXPECT_EQ(code_of(extra), code(spade::Code::invalid_argument));
    EXPECT_TRUE(why(extra).starts_with("unknown key 'maker' in entries[0] -- library schema v2 does not define it"))
        << why(extra);
}

TEST(Library, UnknownMissingAndDuplicateKeysAreRefusedAtEveryLevel) {
    struct Case {
        std::string what;
        std::string text;
        std::string expected;  // the whole context
    };
    const std::string base = two_entries();
    const std::vector<Case> cases = {
        {"an unknown top-level key", with(base, "entries:\n", "icons: \"icons/\"\nentries:\n"),
         "unknown key 'icons' in library index -- library schema v1 does not define it (line 2, column 1)"},
        {"no library_version", with(base, "library_version: 1\n", ""),
         "missing required key 'library_version' in library index (line 1, column 1)"},
        {"no entries", "library_version: 1\n", "missing required key 'entries' in library index (line 1, column 1)"},
        {"a duplicate top-level key", with(base, "entries:\n", "library_version: 1\nentries:\n"),
         "duplicate key 'library_version' in library index (line 2, column 1)"},
        {"entries not a sequence", "library_version: 1\nentries: {}\n", "entries must be a sequence (line 2, column 10)"},
        {"a top level that is not a mapping", "- library_version: 1\n",
         "a library index's top level must be a mapping with a 'library_version' key (line 1, column 1)"},
        {"an unknown entry key", with(base, "    thumbnail: \"thumbnails/box.png\"\n", "    icon: \"box.png\"\n"),
         "unknown key 'icon' in entries[0] -- library schema v1 does not define it (line 6, column 5)"},
        {"an entry with no name", with(base, "  - name: \"Quad\"\n    category: \"drones\"\n", "  - category: \"drones\"\n"),
         "missing required key 'name' in entries[1] (line 7, column 5)"},
        {"an entry with no category", with(base, "    category: \"drones\"\n", ""),
         "missing required key 'category' in entries[1] (line 7, column 5)"},
        {"an entry with no file", with(base, "    file: \"drones/quad.model.yaml\"\n", ""),
         "missing required key 'file' in entries[1] (line 7, column 5)"},
        {"a duplicate entry key", with(base, "    category: \"drones\"\n", "    category: \"drones\"\n    category: \"x\"\n"),
         "duplicate key 'category' in entries[1] (line 9, column 5)"},
        {"an entry that is not a mapping", "library_version: 1\nentries:\n  - \"Box\"\n",
         "entries[0] must be a mapping (line 3, column 5)"},
        {"a name that is not a scalar", with(base, "  - name: \"Quad\"\n", "  - name: [\"Quad\"]\n"),
         "entries[1].name must be a scalar (line 7, column 11)"},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        const spade::Result<Library> r = spade::scene::library_from_yaml(c.text);
        EXPECT_EQ(code_of(r), code(spade::Code::invalid_argument));
        EXPECT_EQ(why(r), c.expected);
    }

    // A YAML syntax error is refused, not thrown.
    const spade::Result<Library> syntax = spade::scene::library_from_yaml("library_version: [1\n");
    EXPECT_EQ(code_of(syntax), code(spade::Code::invalid_argument));
}

TEST(Library, NamesAndCategoriesAreNonEmptyAndNamesAreUniqueIgnoringCase) {
    const std::string base = two_entries();
    const spade::Result<Library> empty_name =
        spade::scene::library_from_yaml(with(base, "name: \"Quad\"", "name: \"\""));
    EXPECT_EQ(code_of(empty_name), code(spade::Code::invalid_argument));
    EXPECT_EQ(why(empty_name), "entries[1].name must not be empty (line 7, column 11)");

    const spade::Result<Library> empty_category =
        spade::scene::library_from_yaml(with(base, "category: \"drones\"", "category: \"\""));
    EXPECT_EQ(why(empty_category), "entries[1].category must not be empty (line 8, column 15)");

    // "box" reads as "Box" in the panel.
    const spade::Result<Library> twin = spade::scene::library_from_yaml(with(base, "name: \"Quad\"", "name: \"box\""));
    EXPECT_EQ(code_of(twin), code(spade::Code::invalid_argument));
    EXPECT_EQ(why(twin),
              "entries[1].name 'box' is entries[0]'s name 'Box' ignoring case; names are unique ignoring case "
              "(line 7, column 11)");
}

TEST(Library, AFileFollowsTheScenesPathRuleAndStaysInTheLibrary) {
    const std::string base = two_entries();
    const auto refused = [&](const std::string& file, const std::string& problem) {
        SCOPED_TRACE(file);
        const spade::Result<Library> r = spade::scene::library_from_yaml(
            with(base, "file: \"drones/quad.model.yaml\"", "file: '" + file + "'"));
        EXPECT_EQ(code_of(r), code(spade::Code::invalid_argument));
        EXPECT_EQ(why(r), "entries[1].file '" + file + "' " + problem + " (line 9, column 11)");
    };
    // The scene's rule (relative_path_problem, TD-9), word for word.
    refused("drones\\quad.model.yaml", "must use forward slashes");
    refused("/srv/library/drones/quad.model.yaml", "must be relative to the library index file");
    refused("C:/library/drones/quad.model.yaml", "must be relative to the library index file");
    // And the library's own: an entry stays inside the library's folder.
    refused("../drones/quad.model.yaml",
            "must not have a '..' segment: an entry stays inside the library's folder");
    refused("drones/../../quad.model.yaml",
            "must not have a '..' segment: an entry stays inside the library's folder");

    const spade::Result<Library> empty =
        spade::scene::library_from_yaml(with(base, "file: \"drones/quad.model.yaml\"", "file: \"\""));
    EXPECT_EQ(why(empty), "entries[1].file must not be empty (line 9, column 11)");

    // A '..' inside a segment is a name, not the parent.
    const spade::Result<Library> dots = spade::scene::library_from_yaml(
        with(base, "file: \"drones/quad.model.yaml\"", "file: \"drones/quad..v2.model.yaml\""));
    ASSERT_OK(dots);
    EXPECT_EQ(dots->entries[1].file, "drones/quad..v2.model.yaml");
}

TEST(Library, TheExtensionDecidesTheKindAndAnyOtherIsRefused) {
    const std::string base = two_entries();
    for (const std::string file : {"drones/quad.yaml", "drones/quad.MODEL.YAML", "drones/quad.scene.yaml",
                                   "drones/quad.model.yml", "drones/quad.spmesh"}) {
        SCOPED_TRACE(file);
        const spade::Result<Library> r = spade::scene::library_from_yaml(
            with(base, "file: \"drones/quad.model.yaml\"", "file: \"" + file + "\""));
        EXPECT_EQ(code_of(r), code(spade::Code::invalid_argument));
        EXPECT_EQ(why(r), "entries[1].file '" + file +
                              "' must end in .model.yaml (a model file) or .object.yaml (an object file) (line 9, "
                              "column 11)");
    }
    const spade::Result<Library> library = spade::scene::library_from_yaml(base);
    ASSERT_OK(library);
    EXPECT_EQ(library->entries[0].kind, LibraryKind::object);
    EXPECT_EQ(library->entries[1].kind, LibraryKind::model);
}

TEST(Library, FilesAreUniqueIgnoringCase) {
    // One file on Windows, two on Linux: refused on both.
    const spade::Result<Library> twin = spade::scene::library_from_yaml(
        with(two_entries(), "file: \"drones/quad.model.yaml\"", "file: \"Primitives/Box.object.yaml\""));
    EXPECT_EQ(code_of(twin), code(spade::Code::invalid_argument));
    EXPECT_EQ(why(twin),
              "entries[1].file 'Primitives/Box.object.yaml' is entries[0]'s file 'primitives/box.object.yaml' "
              "ignoring case, and on Windows they are one file (line 9, column 11)");
}

TEST(Library, AThumbnailIsOptionalAndFollowsTheFilesPathRule) {
    const std::string base = two_entries();
    const spade::Result<Library> empty =
        spade::scene::library_from_yaml(with(base, "thumbnail: \"thumbnails/box.png\"", "thumbnail: \"\""));
    EXPECT_EQ(code_of(empty), code(spade::Code::invalid_argument));
    EXPECT_EQ(why(empty),
              "entries[0].thumbnail must not be empty: leave the key out for an entry with none (line 6, column 16)");
    const spade::Result<Library> backslash =
        spade::scene::library_from_yaml(with(base, "thumbnail: \"thumbnails/box.png\"", "thumbnail: 'thumbnails\\box.png'"));
    EXPECT_EQ(why(backslash),
              "entries[0].thumbnail 'thumbnails\\box.png' must use forward slashes (line 6, column 16)");
    const spade::Result<Library> outside =
        spade::scene::library_from_yaml(with(base, "thumbnail: \"thumbnails/box.png\"", "thumbnail: \"../box.png\""));
    EXPECT_EQ(why(outside),
              "entries[0].thumbnail '../box.png' must not have a '..' segment: an entry stays inside the library's "
              "folder (line 6, column 16)");
}

// ===========================================================================
// On disk (load_library, load_library_model, load_library_object)
// ===========================================================================

TEST(LibraryLoad, TheDirectoryIsTheIndexsAndEveryFileItNamesMustExist) {
    const spade::test::TempDir dir("library-load");
    lay_out_two_entries(dir.path());
    const fs::path index = dir.path() / "library.yaml";
    const spade::Result<Library> library = spade::scene::load_library(index);
    ASSERT_OK(library);
    EXPECT_EQ(library->dir, dir.path());
    ASSERT_EQ(library->entries.size(), 2u);

    // A missing file, and a missing thumbnail the entry names, by name and path.
    const fs::path absent_file = dir.path() / "absent_file.yaml";
    write_text(absent_file, with(two_entries(), "drones/quad.model.yaml", "drones/absent.model.yaml"));
    const spade::Result<Library> no_file = spade::scene::load_library(absent_file);
    EXPECT_EQ(code_of(no_file), code(spade::Code::io_error));
    EXPECT_EQ(why(no_file), spade::path_text(absent_file) +
                                ": library entry 'Quad': file 'drones/absent.model.yaml' is missing (" +
                                spade::path_text(dir.path() / fs::path("drones/absent.model.yaml")) + ")");
    const fs::path absent_thumbnail = dir.path() / "absent_thumbnail.yaml";
    write_text(absent_thumbnail, with(two_entries(), "thumbnails/box.png", "thumbnails/bxo.png"));
    const spade::Result<Library> no_thumbnail = spade::scene::load_library(absent_thumbnail);
    EXPECT_EQ(code_of(no_thumbnail), code(spade::Code::io_error));
    EXPECT_EQ(why(no_thumbnail), spade::path_text(absent_thumbnail) +
                                     ": library entry 'Box': thumbnail 'thumbnails/bxo.png' is missing (" +
                                     spade::path_text(dir.path() / fs::path("thumbnails/bxo.png")) + ")");
    // A folder is not a file.
    const fs::path a_folder = dir.path() / "a_folder.yaml";
    fs::create_directories(dir.path() / "folder.object.yaml");
    write_text(a_folder, with(two_entries(), "primitives/box.object.yaml", "folder.object.yaml"));
    EXPECT_EQ(code_of(spade::scene::load_library(a_folder)), code(spade::Code::io_error));

    // No index, and a bad one, by its path.
    const fs::path absent = dir.path() / "absent.yaml";
    const spade::Result<Library> no_index = spade::scene::load_library(absent);
    EXPECT_EQ(code_of(no_index), code(spade::Code::io_error));
    EXPECT_EQ(why(no_index), "cannot open library index '" + spade::path_text(absent) + "'");
    const fs::path newer = dir.path() / "newer.yaml";
    write_text(newer, with(two_entries(), "library_version: 1\n", "library_version: 2\n"));
    const spade::Result<Library> version = spade::scene::load_library(newer);
    EXPECT_EQ(code_of(version), code(spade::Code::schema_mismatch));
    EXPECT_TRUE(why(version).starts_with(spade::path_text(newer) + ": library index is schema version 2"))
        << why(version);
}

TEST(LibraryLoad, EachKindLoadsWithItsOwnLoaderAndTheOtherRefusesItByName) {
    const spade::test::TempDir dir("library-kinds");
    lay_out_two_entries(dir.path());
    const spade::Result<Library> library = spade::scene::load_library(dir.path() / "library.yaml");
    ASSERT_OK(library);
    const LibraryEntry& box = library->entries[0];
    const LibraryEntry& quad = library->entries[1];

    const spade::Result<ObjectFile> object = spade::scene::load_library_object(*library, box);
    ASSERT_OK(object);
    EXPECT_EQ(object->object.name, "box");
    const spade::Result<ModelFile> model = spade::scene::load_library_model(*library, quad);
    ASSERT_OK(model);
    EXPECT_EQ(model->model.name, "quad");

    const spade::Result<ModelFile> box_as_model = spade::scene::load_library_model(*library, box);
    EXPECT_EQ(code_of(box_as_model), code(spade::Code::invalid_argument));
    EXPECT_EQ(why(box_as_model), "library entry 'Box' is an object, not a model: load it with load_library_object");
    const spade::Result<ObjectFile> quad_as_object = spade::scene::load_library_object(*library, quad);
    EXPECT_EQ(code_of(quad_as_object), code(spade::Code::invalid_argument));
    EXPECT_EQ(why(quad_as_object), "library entry 'Quad' is a model, not an object: load it with load_library_model");

    // A loader's refusal keeps its code, after the entry's name.
    const fs::path box_file = dir.path() / "primitives" / "box.object.yaml";
    std::string box_text;
    ASSERT_TRUE(read_bytes(box_file, box_text));
    write_text(box_file, with(box_text, "\nobject_version: 1\n", "\nobject_version: 2\n"));
    const spade::Result<ObjectFile> newer = spade::scene::load_library_object(*library, box);
    EXPECT_EQ(code_of(newer), code(spade::Code::schema_mismatch));
    EXPECT_TRUE(why(newer).starts_with("library entry 'Box': " + spade::path_text(dir.path() / fs::path(box.file)) +
                                       ": object file is schema version 2"))
        << why(newer);
}

TEST(LibraryContents, EveryEntryLoadsAndTwoRefsThatDifferOnlyInCaseAreRefused) {
    const spade::test::TempDir dir("library-contents");
    const std::vector<ObjectRow> table = content_table();
    write_text(dir.path() / "props" / "ball.object.yaml", text_of(with_visual(generated_object(table[1]), "props/ball")));
    write_text(dir.path() / "props" / "ball_again.object.yaml",
               text_of(with_visual(generated_object(table[0]), "props/ball")));
    write_text(dir.path() / "props" / "big_ball.object.yaml",
               text_of(with_visual(generated_object(table[2]), "props/Ball")));
    write_text(dir.path() / "drones" / "quad.model.yaml", text_of(tiny_model()));
    Library library;
    library.dir = dir.path();
    library.entries = {
        LibraryEntry{"Quad", "drones", "drones/quad.model.yaml", "", LibraryKind::model},
        LibraryEntry{"Ball", "props", "props/ball.object.yaml", "", LibraryKind::object},
        LibraryEntry{"Ball again", "props", "props/ball_again.object.yaml", "", LibraryKind::object},
    };
    // The same ref twice is one mesh.
    EXPECT_OK(spade::scene::check_library_contents(library));

    library.entries.push_back(LibraryEntry{"Big ball", "props", "props/big_ball.object.yaml", "", LibraryKind::object});
    const spade::Result<void> clash = spade::scene::check_library_contents(library);
    EXPECT_EQ(code_of(clash), code(spade::Code::invalid_argument));
    EXPECT_EQ(why(clash),
              "library entries 'Ball' and 'Big ball': visual mesh_refs 'props/ball' and 'props/Ball' differ only in "
              "case, and on Windows they name one file");

    // Every entry is loaded: a broken model is refused with its own code.
    library.entries.pop_back();
    write_text(dir.path() / "drones" / "quad.model.yaml", "model_version: 1\nmodel: {}\n");
    const spade::Result<void> broken = spade::scene::check_library_contents(library);
    EXPECT_EQ(code_of(broken), code(spade::Code::invalid_argument));
    EXPECT_TRUE(why(broken).starts_with("library entry 'Quad': ")) << why(broken);
}

namespace {

// A motor part citing `kv_source` for its kv and assuming the rest (L2's part
// files).
[[nodiscard]] std::string a_motor_text(std::string_view kv_source) {
    spade::scene::MotorPart m;
    m.block.kv = 960.0;
    m.block.resistance = 0.1;
    m.block.no_load_current = 0.5;
    m.block.current_max = 20.0;
    m.block.mass = 0.055;
    for (const char* field : {"kv", "resistance", "no_load_current", "no_load_voltage", "current_max", "pole_pairs",
                              "inductance", "rotor_inertia", "mass"}) {
        m.sources.push_back({field, std::string_view(field) == "kv"
                                        ? spade::scene::SourceRef{std::string(kv_source),
                                                                  spade::scene::Grade::measured, ""}
                                        : spade::scene::SourceRef{"", spade::scene::Grade::assumed, "typical"}});
    }
    const spade::Result<std::string> text = spade::scene::motor_to_yaml(m);
    EXPECT_TRUE(text.has_value()) << text.error().context;
    return text.has_value() ? *text : std::string{};
}

constexpr std::string_view kLibrarySources = R"YAML(sources_version: 1
sources:
  - {id: uiuc_propdb, citation: "UIUC", retrieved: 2026-10-21, license: facts_cite, license_evidence: "facts"}
  - {id: apc_data_files, citation: "APC", retrieved: 2026-10-21, license: not_usable, license_evidence: "no"}
)YAML";

}  // namespace

TEST(LibraryLoad, APartEntryLoadsAgainstTheLibrarysSources) {
    const spade::test::TempDir dir("library-parts");
    write_text(dir.path() / "sources.yaml", kLibrarySources);
    write_text(dir.path() / "parts" / "m2312.motor.yaml", a_motor_text("uiuc_propdb"));
    write_text(dir.path() / "library.yaml", "library_version: 2\nentries:\n"
                                            "  - {name: M2312, category: motors, file: parts/m2312.motor.yaml}\n");
    const spade::Result<Library> library = spade::scene::load_library(dir.path() / "library.yaml");
    ASSERT_OK(library);
    const LibraryEntry& motor = library->entries[0];
    EXPECT_EQ(motor.kind, LibraryKind::motor);

    const spade::Result<spade::scene::SourceRegistry> registry = spade::scene::load_library_sources(*library);
    ASSERT_OK(registry);
    const spade::Result<spade::scene::PartFile> part = spade::scene::load_library_part(*library, motor, *registry);
    ASSERT_OK(part);
    ASSERT_TRUE(std::holds_alternative<spade::scene::MotorPart>(*part));
    EXPECT_EQ(std::get<spade::scene::MotorPart>(*part).block.kv, 960.0);
    EXPECT_OK(spade::scene::check_library_contents(*library));

    // Each loader refuses the other kinds by name.
    EXPECT_EQ(why(spade::scene::load_library_model(*library, motor)),
              "library entry 'M2312' is a motor part, not a model: load it with load_library_part");
    EXPECT_EQ(why(spade::scene::load_library_object(*library, motor)),
              "library entry 'M2312' is a motor part, not an object: load it with load_library_part");

    // A part citing a source the registry rules out is refused, by entry,
    // file, field and source.
    write_text(dir.path() / "parts" / "m2312.motor.yaml", a_motor_text("apc_data_files"));
    const spade::Result<void> apc = spade::scene::check_library_contents(*library);
    EXPECT_EQ(code_of(apc), code(spade::Code::invalid_argument));
    EXPECT_TRUE(why(apc).starts_with("library entry 'M2312': " +
                                     spade::path_text(dir.path() / "parts" / "m2312.motor.yaml") + ": motor.kv"))
        << why(apc);
    EXPECT_NE(why(apc).find("'apc_data_files'"), std::string::npos) << why(apc);

    // A library with parts needs its sources registry.
    fs::remove(dir.path() / "sources.yaml");
    const spade::Result<void> no_sources = spade::scene::check_library_contents(*library);
    EXPECT_EQ(code_of(no_sources), code(spade::Code::io_error));
    EXPECT_NE(why(no_sources).find("sources.yaml"), std::string::npos) << why(no_sources);
}

// ===========================================================================
// The object files, and their generator
// ===========================================================================

// The golden, and with SPADE_LIBRARY_WRITE set, its generator.
TEST(LibraryObjects, TheCommittedFilesAreTheGeneratorsText) {
    const bool write = library_write_asked();
    std::size_t written = 0;
    for (const ObjectRow& row : content_table()) {
        SCOPED_TRACE(std::string(row.name));
        const spade::Result<std::string> text = spade::scene::object_to_yaml(generated_object(row));
        ASSERT_OK(text);
        const fs::path path = library_dir() / fs::path(std::string(row.file));

        if (write) {
            std::error_code ec;
            fs::create_directories(path.parent_path(), ec);
            ASSERT_FALSE(ec) << spade::path_text(path.parent_path()) << ": " << ec.message();
            {
                std::ofstream out(path, std::ios::binary | std::ios::trunc);
                out.write(text->data(), static_cast<std::streamsize>(text->size()));
                out.close();
                ASSERT_TRUE(out) << "SPADE_LIBRARY_WRITE: could not write " << spade::path_text(path);
            }
            std::string back;
            ASSERT_TRUE(read_bytes(path, back)) << spade::path_text(path);
            ASSERT_TRUE(back == *text) << "SPADE_LIBRARY_WRITE: " << spade::path_text(path)
                                       << " does not read back as written";
            std::printf("SPADE_LIBRARY_WRITE: wrote %zu bytes to %s.\n", text->size(), spade::path_text(path).c_str());
            ++written;
            continue;
        }

        std::string committed;
        ASSERT_TRUE(read_bytes(path, committed))
            << spade::path_text(path) << " is missing (TD-5). Generate it: set SPADE_LIBRARY_WRITE=1 and run ctest -R "
            << "LibraryObjects.TheCommittedFilesAreTheGeneratorsText.";
        EXPECT_TRUE(committed == *text) << mismatch(path, committed, *text);
    }
    if (write) {
        std::printf("SPADE_LIBRARY_WRITE: wrote %zu object files. Review the diff, name its cause (TD-1), and have a "
                    "fresh Docker gcc leg reproduce them (TD-12).\n",
                    written);
        RecordProperty("written", static_cast<int>(written));
        SUCCEED() << "wrote " << written << " object files";
    }
}

// Each file loads back as the table says: centred at its origin, one lambert
// material named library_<name> in the row's colour, every node naming it,
// and no visual. Saved unedited, it writes back its own bytes.
TEST(LibraryObjects, EachIsCentredWithOneLambertMaterialAndNoVisual) {
    for (const ObjectRow& row : content_table()) {
        SCOPED_TRACE(std::string(row.name));
        const fs::path path = library_dir() / fs::path(std::string(row.file));
        const spade::Result<ObjectFile> loaded = spade::scene::load_object_file(path);
        ASSERT_OK(loaded);
        const std::string token = token_of(row.name);
        const std::string material = "library_" + token;
        EXPECT_EQ(loaded->object.name, token);

        // Centred at its origin: the identity pose, and one identity transform
        // every node uses (each primitive is centred on its own frame,
        // world/sdf.hpp's packing).
        const SdfPose identity{};
        EXPECT_EQ(std::memcmp(&loaded->object.pose, &identity, sizeof identity), 0);
        ASSERT_EQ(loaded->object.collider.transforms.size(), 1u);
        const SdfTransform unit{};
        EXPECT_EQ(std::memcmp(&loaded->object.collider.transforms[0], &unit, sizeof unit), 0);
        ASSERT_EQ(loaded->object.collider.nodes.size(), row.nodes.size());
        EXPECT_EQ(std::memcmp(loaded->object.collider.nodes.data(), row.nodes.data(), row.nodes.size() * sizeof(SdfNode)),
                  0);
        for (const SdfNode& n : loaded->object.collider.nodes) EXPECT_EQ(n.transform, 0u);

        // Collider-only, in its one material.
        EXPECT_TRUE(loaded->object.visual.mesh_ref.empty());
        EXPECT_TRUE(loaded->object.visual.material.empty());
        EXPECT_EQ(loaded->object.collider_materials, std::vector<std::string>(row.nodes.size(), material));
        ASSERT_EQ(loaded->materials.size(), 1u);
        const MaterialDesc& m = loaded->materials[0];
        EXPECT_EQ(m.name, material);
        EXPECT_EQ(m.shading, MaterialShading::lambert);
        EXPECT_EQ(m.base_color, glm::vec4(row.rgb, 1.0f));
        EXPECT_NE(glm::vec3(m.base_color), kSelectionAccent) << "a placed object would read as selected";

        const spade::Result<std::string> again = spade::scene::object_to_yaml(*loaded);
        ASSERT_OK(again);
        std::string committed;
        ASSERT_TRUE(read_bytes(path, committed));
        EXPECT_TRUE(*again == committed) << mismatch(path, committed, *again);
    }
}

// The gate is one drawable: its collider is one CSG root (the outer box minus
// the opening), never a primitive drawn alone, and the opening cuts clean
// through the frame.
TEST(LibraryObjects, TheGateIsOneCsgRootWithAnOpeningThroughZ) {
    const fs::path path = library_dir() / "obstacles" / "gate.object.yaml";
    const spade::Result<ObjectFile> gate = spade::scene::load_object_file(path);
    ASSERT_OK(gate);
    const spade::Result<spade::render::SubtreeSplit> split = spade::render::split_program(gate->object.collider);
    ASSERT_OK(split);
    EXPECT_EQ(split->csg_roots, std::vector<uint32_t>{2u});
    EXPECT_TRUE(split->union_primitive_nodes.empty());

    const std::vector<SdfNode>& nodes = gate->object.collider.nodes;
    ASSERT_EQ(nodes.size(), 3u);
    EXPECT_EQ(nodes[2].op, static_cast<uint32_t>(SdfOp::subtract));
    const glm::vec4 outer = nodes[0].params;
    const glm::vec4 opening = nodes[1].params;
    EXPECT_LT(opening.x, outer.x);
    EXPECT_LT(opening.y, outer.y);
    EXPECT_GT(opening.z, outer.z) << "deeper than the frame, so it cuts through";
    // Through the middle it is open; in the frame it is solid.
    EXPECT_GT(spade::eval(gate->object.collider, glm::vec3(0.0f)), 0.0f);
    EXPECT_LT(spade::eval(gate->object.collider, glm::vec3(0.8f, 0.0f, 0.0f)), 0.0f);
}

// ===========================================================================
// The committed index
// ===========================================================================

TEST(LibraryIndex, TheCommittedIndexIsTheTablesEntriesInOrderAndEveryOneLoads) {
    const spade::Result<Library> library = spade::scene::load_library(library_dir() / "library.yaml");
    ASSERT_OK(library);
    EXPECT_EQ(library->dir, library_dir());
    const std::vector<IndexRow> expected = expected_index();
    ASSERT_EQ(library->entries.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        SCOPED_TRACE("entry " + std::to_string(i));
        const LibraryEntry& e = library->entries[i];
        EXPECT_EQ(e.name, expected[i].name);
        EXPECT_EQ(e.category, expected[i].category);
        EXPECT_EQ(e.file, expected[i].file);
        EXPECT_EQ(e.kind, expected[i].kind);
        EXPECT_TRUE(e.thumbnail.empty()) << "thumbnails come later, rendered by Rendering";
    }
    EXPECT_OK(spade::scene::check_library_contents(*library));

    // The drone is N4's model file, with its chain (LibraryQuad pins its
    // bytes and identities).
    const spade::Result<ModelFile> quad = spade::scene::load_library_model(*library, library->entries[0]);
    ASSERT_OK(quad);
    EXPECT_TRUE(quad->model.propulsion.has_value()) << "a library drone carries its chain";

    // Each object entry is the generator's object.
    for (const ObjectRow& row : content_table()) {
        SCOPED_TRACE(std::string(row.name));
        const LibraryEntry* entry = nullptr;
        for (const LibraryEntry& e : library->entries) {
            if (e.name == row.name) entry = &e;
        }
        ASSERT_NE(entry, nullptr);
        const spade::Result<ObjectFile> object = spade::scene::load_library_object(*library, *entry);
        ASSERT_OK(object);
        EXPECT_EQ(text_of(*object), text_of(generated_object(row)));
    }
}
