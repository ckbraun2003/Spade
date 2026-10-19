// The object file (scene/object_file.hpp): one placeable object -- a prop, an
// obstacle, a gate, a floor tile, a pole, a wall or a primitive -- and the
// materials it brings that a world does not ship; and add_object(), the one
// way the editor (AddAsset) and Kat's builder place one (TD-9).
//
// What is pinned here:
//   - CANONICAL TEXT. A gate -- a CSG collider of three nodes over two
//     transforms, a material per node, a visual and two materials, at a pose
//     that is not the identity -- survives text, file and text again byte for
//     byte, and field for field; a ball with one material matches a pinned
//     text, which the gcc leg reproduces (MSVC = gcc).
//   - ONE ENCODING (TD-9). The object block is the scene's per-asset block,
//     key for key, two spaces shallower; materials are the scene's encoding.
//   - ONE VALIDATOR: the scene's per-asset rules with the scene's messages,
//     the palette's material rules with the scene's messages, unique names,
//     and no unused material; a world material need not be listed.
//   - THE VERSION GATE, and unknown, missing and duplicate keys refused at
//     every level, blamed on "object schema v1".
//   - add_object(): the asset appended under the given name at the given
//     pose, its materials merged (an identical one reused, a different one
//     refused naming every field with both values), a taken name refused,
//     the scene unchanged on every refusal, and a world material it names
//     resolved by compose().

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "scene/compose.hpp"
#include "scene/object_file.hpp"
#include "scene/scene_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_type.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

namespace {

using spade::MaterialDesc;
using spade::MaterialShading;
using spade::SdfNode;
using spade::SdfOp;
using spade::SdfPose;
using spade::SdfPrim;
using spade::SdfTransform;
using spade::scene::ObjectFile;
using spade::scene::SceneAsset;
using spade::scene::SceneDesc;
using spade::scene::SceneVehicle;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

[[nodiscard]] bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

// The refusal's context, or a marker when the call unexpectedly succeeded.
template <class T>
[[nodiscard]] std::string why(const spade::Result<T>& r) {
    return r ? std::string("<succeeded>") : r.error().context;
}

[[nodiscard]] std::string first_difference(const std::string& expected, const std::string& actual) {
    std::size_t i = 0;
    while (i < expected.size() && i < actual.size() && expected[i] == actual[i]) ++i;
    if (i == expected.size() && i == actual.size()) return "identical";
    std::size_t line = 1;
    for (std::size_t j = 0; j < i; ++j) {
        if (expected[j] == '\n') ++line;
    }
    return "first difference at byte " + std::to_string(i) + ", line " + std::to_string(line);
}

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

void write_text(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
}

// A rotation of +90 degrees about +Y, (w, x, y, z) = (cos 45, 0, sin 45, 0), as
// an exact float constant: no libm here (TD-3).
const glm::quat kYawPlus90(0.707106769f, 0.0f, 0.707106769f, 0.0f);

[[nodiscard]] SdfNode prim(SdfPrim kind, uint32_t transform, glm::vec4 params) {
    SdfNode n{};
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(SdfOp::none);
    n.transform = transform;
    n.params = params;
    return n;
}

[[nodiscard]] SdfNode op(SdfOp which) {
    SdfNode n{};
    n.op = static_cast<uint32_t>(which);
    return n;
}

// A translation by `offset`, pre-inverted as SdfTransform stores it.
[[nodiscard]] SdfTransform moved_by(glm::vec3 offset) {
    SdfTransform t{};
    t.world_to_local[3] = glm::vec4(-offset, 1.0f);
    return t;
}

[[nodiscard]] SdfPose place_at(glm::vec3 position, glm::quat rotation, float scale) {
    SdfPose p;
    p.position = position;
    p.rotation = rotation;
    p.scale = scale;
    return p;
}

[[nodiscard]] MaterialDesc red() {
    return MaterialDesc{"red", {0.8f, 0.1f, 0.1f, 1.0f}, MaterialShading::lambert};
}

[[nodiscard]] MaterialDesc blue() {
    return MaterialDesc{"blue", {0.1f, 0.2f, 0.9f, 1.0f}, MaterialShading::unlit};
}

[[nodiscard]] MaterialDesc white() {
    return MaterialDesc{"white", {1.0f, 1.0f, 1.0f, 1.0f}, MaterialShading::lambert};
}

// ---------------------------------------------------------------------------
// The gate: a CSG collider (a box and a sphere joined by a union, over two
// transforms), a material per node, a visual and two materials, at a pose
// that is not the identity. "blue" is named by a collider node alone.
// ---------------------------------------------------------------------------
[[nodiscard]] ObjectFile gate_object() {
    ObjectFile f;
    f.object.name = "gate";
    f.object.pose = place_at({1.0f, 2.0f, -3.0f}, kYawPlus90, 1.25f);
    f.object.collider.transforms = {SdfTransform{}, moved_by({0.5f, 0.0f, 0.0f})};
    f.object.collider.nodes = {prim(SdfPrim::box, 0, {0.1f, 0.9f, 0.1f, 0.0f}),
                               prim(SdfPrim::sphere, 1, {0.3f, 0.0f, 0.0f, 0.0f}), op(SdfOp::union_)};
    f.object.collider_materials = {"red", "blue", "red"};
    f.object.visual = {"gates/frame_2000", "red"};
    f.materials = {red(), blue()};
    return f;
}

// The ball of the pinned text: one sphere, half a metre up, and one material,
// named by its node and by its visual.
[[nodiscard]] ObjectFile ball_object() {
    ObjectFile f;
    f.object.name = "ball";
    f.object.pose.position = {0.0f, 0.5f, 0.0f};
    f.object.collider.transforms = {SdfTransform{}};
    f.object.collider.nodes = {prim(SdfPrim::sphere, 0, {0.5f, 0.0f, 0.0f, 0.0f})};
    f.object.collider_materials = {"red"};
    f.object.visual = {"props/ball", "red"};
    f.materials = {red()};
    return f;
}

// THE PINNED CANONICAL TEXT of ball_object(). Written by hand from the format
// (object_file.hpp): the scene's per-asset block two spaces shallower, the
// scene's material encoding, every float as %.9g of its binary32 (a Python
// port, not the C++). The gcc leg must reproduce it byte for byte.
constexpr std::string_view kPinnedText = R"YAML(# =============================================================================
# Spade object file -- schema version 1.
#
# WHAT THIS IS: one placeable object -- a prop, an obstacle, a gate, a floor
# tile, a pole, a wall or a primitive -- what Spade's object library holds, and
# what the editor and Kat's builder place in a scene with add_object().
#
# GENERATED by spade::scene::object_to_yaml(). Hand editing is supported: every
# field is validated on load, and re-saving normalizes the file back to this
# canonical form (fixed key order, fixed float spelling, these comments).
#
# UNITS are SI -- metres and radians -- in a right-handed, Y-up frame. FLOATS
# carry 9 significant digits, so every value survives a load and a save bit
# for bit. Quaternions are [w, x, y, z].
#
# NAMES, NEVER INDICES: a visual and a collider's nodes name their materials,
# each one listed under `materials` or a world's.
# =============================================================================

object_version: 1

# The object: a scene file's per-asset block, key for key. `pose` is the
# object's as the file holds it; add_object() places it at the pose it is
# given instead. `collider.sdf` is the world file's SDF encoding in the
# object's frame, with node_materials by name; no nodes means no collision.
# An empty visual means none.
object:
  name: "ball"
  pose: {position: [0, 0.5, 0], orientation: [1, 0, 0, 0], scale: 1}
  collider:
    sdf:
      transforms:
        # [0]
        - world_to_local: [1, 0, 0, 0,
                           0, 1, 0, 0,
                           0, 0, 1, 0,
                           0, 0, 0, 1]
          scale: 1
      nodes:
        - {prim: sphere, transform: 0, params: [0.5, 0, 0, 0]}
      node_materials: ["red"]
  visual: {mesh_ref: "props/ball", material: "red"}

# The materials the object names that a world does not ship, in the scene
# file's encoding. Placing the object adds each to the scene's materials, or
# reuses an identical one of the same name.
materials:
  - {name: "red", base_color: [0.800000012, 0.100000001, 0.100000001, 1], shading: "lambert"}
)YAML";

// Field for field, bit for bit where a value is POD.
void expect_same_asset(const SceneAsset& expected, const SceneAsset& actual) {
    EXPECT_EQ(actual.name, expected.name);
    EXPECT_EQ(std::memcmp(&actual.pose.position, &expected.pose.position, sizeof expected.pose.position), 0);
    EXPECT_EQ(std::memcmp(&actual.pose.rotation, &expected.pose.rotation, sizeof expected.pose.rotation), 0);
    EXPECT_EQ(std::memcmp(&actual.pose.scale, &expected.pose.scale, sizeof expected.pose.scale), 0);
    ASSERT_EQ(actual.collider.transforms.size(), expected.collider.transforms.size());
    ASSERT_EQ(actual.collider.nodes.size(), expected.collider.nodes.size());
    if (!expected.collider.transforms.empty()) {
        EXPECT_EQ(std::memcmp(actual.collider.transforms.data(), expected.collider.transforms.data(),
                              expected.collider.transforms.size() * sizeof(SdfTransform)),
                  0);
    }
    if (!expected.collider.nodes.empty()) {
        EXPECT_EQ(std::memcmp(actual.collider.nodes.data(), expected.collider.nodes.data(),
                              expected.collider.nodes.size() * sizeof(SdfNode)),
                  0);
    }
    EXPECT_EQ(actual.collider.node_materials, expected.collider.node_materials);
    EXPECT_EQ(actual.collider_materials, expected.collider_materials);
    EXPECT_EQ(actual.visual.mesh_ref, expected.visual.mesh_ref);
    EXPECT_EQ(actual.visual.material, expected.visual.material);
}

void expect_same_materials(const std::vector<MaterialDesc>& expected, const std::vector<MaterialDesc>& actual) {
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        SCOPED_TRACE("material " + std::to_string(i));
        EXPECT_EQ(actual[i].name, expected[i].name);
        EXPECT_EQ(std::memcmp(&actual[i].base_color, &expected[i].base_color, sizeof expected[i].base_color), 0);
        EXPECT_EQ(actual[i].shading, expected[i].shading);
    }
}

// A scene with one asset of `asset`, for the scene's verdict on it.
[[nodiscard]] SceneDesc scene_of(const SceneAsset& asset) {
    SceneDesc s;
    s.name = "s";
    s.world.file = "w.world.yaml";
    s.assets = {asset};
    return s;
}

// A scene to place objects in: a material, an asset, a model and a vehicle,
// so a name can be taken by an asset or by a vehicle.
[[nodiscard]] SceneDesc stage() {
    SceneDesc s;
    s.name = "stage";
    s.world.file = "ground.world.yaml";
    s.materials = {white()};
    SceneAsset marker;
    marker.name = "marker";
    marker.visual = {"decor/marker", "white"};
    s.assets = {marker};
    spade::vehicles::ModelType spotter{};
    spotter.name = "spotter";
    spotter.body.mass = 2.5f;
    spotter.proxy_radius = 0.3f;
    s.models = {spotter};
    SceneVehicle quad;
    quad.name = "quad_1";
    quad.model = "spotter";
    s.vehicles = {quad};
    return s;
}

// The scene's canonical text, the whole of it: what "unchanged" is compared by.
[[nodiscard]] std::string text_of(const SceneDesc& scene) {
    const spade::Result<std::string> text = spade::scene::scene_to_yaml(scene);
    EXPECT_TRUE(text.has_value()) << why(text);
    return text.value_or("<the scene does not validate>");
}

}  // namespace

// ===========================================================================
// Canonical text
// ===========================================================================

TEST(ObjectFile, AGateRoundTripsThroughTextAndFileByteForByte) {
    const ObjectFile gate = gate_object();
    const spade::Result<std::string> text = spade::scene::object_to_yaml(gate);
    ASSERT_OK(text);

    // Text, then file, then text: the same bytes.
    const spade::test::TempDir dir("object-round-trip");
    const std::filesystem::path path = dir.path() / "gate.object.yaml";
    write_text(path, *text);
    const spade::Result<ObjectFile> loaded = spade::scene::load_object_file(path);
    ASSERT_OK(loaded);
    const spade::Result<std::string> again = spade::scene::object_to_yaml(*loaded);
    ASSERT_OK(again);
    EXPECT_TRUE(*again == *text) << first_difference(*text, *again);

    // And what was read is what was written, field for field.
    {
        SCOPED_TRACE("loaded");
        expect_same_asset(gate.object, loaded->object);
        expect_same_materials(gate.materials, loaded->materials);
    }
    const spade::Result<ObjectFile> via_text = spade::scene::object_from_yaml(*text);
    ASSERT_OK(via_text);
    {
        SCOPED_TRACE("from text");
        expect_same_asset(gate.object, via_text->object);
        expect_same_materials(gate.materials, via_text->materials);
    }
}

TEST(ObjectFile, NoMaterialsRoundTripWithNoMaterialsKey) {
    // Its names are a world's materials now, which need not be listed.
    ObjectFile bare = gate_object();
    bare.materials.clear();

    const spade::Result<std::string> text = spade::scene::object_to_yaml(bare);
    ASSERT_OK(text);
    EXPECT_FALSE(contains(*text, "\nmaterials:")) << *text;
    EXPECT_FALSE(contains(*text, "# The materials")) << "no block, and no comment for one: " << *text;
    const spade::Result<ObjectFile> back = spade::scene::object_from_yaml(*text);
    ASSERT_OK(back);
    EXPECT_TRUE(back->materials.empty());
    expect_same_asset(bare.object, back->object);
    const spade::Result<std::string> again = spade::scene::object_to_yaml(*back);
    ASSERT_OK(again);
    EXPECT_TRUE(*again == *text) << first_difference(*text, *again);

    // The same object with its materials is the same text plus the block.
    const spade::Result<std::string> dressed = spade::scene::object_to_yaml(gate_object());
    ASSERT_OK(dressed);
    EXPECT_TRUE(dressed->starts_with(*text)) << "the materials block comes last";

    // An empty list reads as none, and is written as none.
    const spade::Result<ObjectFile> empty_list = spade::scene::object_from_yaml(*text + "materials: []\n");
    ASSERT_OK(empty_list);
    EXPECT_TRUE(empty_list->materials.empty());
    const spade::Result<std::string> normalized = spade::scene::object_to_yaml(*empty_list);
    ASSERT_OK(normalized);
    EXPECT_TRUE(*normalized == *text) << first_difference(*text, *normalized);
}

TEST(ObjectFile, TheCanonicalTextIsPinned) {
    const ObjectFile ball = ball_object();
    const spade::Result<std::string> text = spade::scene::object_to_yaml(ball);
    ASSERT_OK(text);
    const std::string pinned(kPinnedText);
    EXPECT_TRUE(*text == pinned) << first_difference(pinned, *text) << "\n" << *text;

    // The pinned text reads back to the same object, every bit.
    const spade::Result<ObjectFile> back = spade::scene::object_from_yaml(pinned);
    ASSERT_OK(back);
    expect_same_asset(ball.object, back->object);
    expect_same_materials(ball.materials, back->materials);
}

TEST(ObjectFile, HandWrittenTextNormalizesToTheCanonicalForm) {
    // No comments, flow mappings with their keys in another order, plain
    // scalars, a '+', and other spellings of the same values.
    const std::string pinned(kPinnedText);
    std::string hand = pinned.substr(pinned.find("object_version: 1"));
    hand = with(hand, "object_version: 1", "object_version: +1");
    hand = with(hand, "  pose: {position: [0, 0.5, 0], orientation: [1, 0, 0, 0], scale: 1}\n",
                "  pose: {scale: 1.0, orientation: [1.0, 0, 0, 0], position: [0, 5e-1, 0]}\n");
    hand = with(hand, "  visual: {mesh_ref: \"props/ball\", material: \"red\"}\n",
                "  visual: {material: red, mesh_ref: props/ball}\n");
    hand = with(hand, "  - {name: \"red\", base_color: [0.800000012, 0.100000001, 0.100000001, 1], shading: \"lambert\"}\n",
                "  - {shading: lambert, base_color: [0.8, 0.1, 0.1, 1], name: red}\n");
    ASSERT_NE(hand, pinned);
    const spade::Result<ObjectFile> read = spade::scene::object_from_yaml(hand);
    ASSERT_OK(read);
    const spade::Result<std::string> text = spade::scene::object_to_yaml(*read);
    ASSERT_OK(text);
    EXPECT_TRUE(*text == pinned) << first_difference(pinned, *text);
}

TEST(ObjectFile, TheObjectBlockIsTheScenesPerAssetBlock) {
    // TD-9: the scene's `assets:` item, as a mapping two spaces shallower, is
    // the object file's `object:` block, byte for byte.
    const ObjectFile gate = gate_object();
    const spade::Result<std::string> scene_text = spade::scene::scene_to_yaml(scene_of(gate.object));
    ASSERT_OK(scene_text);
    const std::size_t item = scene_text->find("\nassets:\n  - name: ");
    ASSERT_NE(item, std::string::npos) << *scene_text;
    const std::size_t item_begin = item + std::string_view("\nassets:\n").size();
    const std::size_t item_end = scene_text->find("\n\n", item_begin);
    ASSERT_NE(item_end, std::string::npos);
    std::string shallower;
    std::size_t at = item_begin;
    while (at <= item_end) {
        const std::size_t eol = scene_text->find('\n', at);
        std::string line = scene_text->substr(at, eol - at);
        if (at == item_begin) {
            ASSERT_TRUE(line.starts_with("  - ")) << line;
            line = "  " + line.substr(4);
        } else {
            ASSERT_TRUE(line.starts_with("  ")) << line;
            line = line.substr(2);
        }
        shallower += line + "\n";
        at = eol + 1;
    }

    const spade::Result<std::string> object_text = spade::scene::object_to_yaml(gate);
    ASSERT_OK(object_text);
    const std::size_t block = object_text->find("\nobject:\n");
    ASSERT_NE(block, std::string::npos) << *object_text;
    const std::size_t block_begin = block + std::string_view("\nobject:\n").size();
    const std::size_t block_end = object_text->find("\n\n", block_begin);
    ASSERT_NE(block_end, std::string::npos) << "the materials block follows, after a blank line";
    const std::string object_block = object_text->substr(block_begin, block_end + 1 - block_begin);
    EXPECT_TRUE(object_block == shallower) << first_difference(shallower, object_block) << "\n"
                                           << object_block << "\nvs\n"
                                           << shallower;

    // And each material is the scene's spelling of it.
    SceneDesc with_materials = scene_of(gate.object);
    with_materials.materials = gate.materials;
    const spade::Result<std::string> materials_text = spade::scene::scene_to_yaml(with_materials);
    ASSERT_OK(materials_text);
    const std::size_t list = materials_text->find("\nmaterials:\n");
    ASSERT_NE(list, std::string::npos) << *materials_text;
    const std::size_t list_end = materials_text->find("\n\n", list + 1);
    ASSERT_NE(list_end, std::string::npos);
    const std::string scene_list = materials_text->substr(list, list_end + 1 - list);
    EXPECT_TRUE(object_text->ends_with(scene_list)) << scene_list << "\nvs\n" << *object_text;
}

// ===========================================================================
// The version gate and the keys
// ===========================================================================

TEST(ObjectFile, AnotherObjectVersionIsASchemaMismatch) {
    for (const char* version : {"2", "0"}) {
        SCOPED_TRACE(version);
        const spade::Result<ObjectFile> r = spade::scene::object_from_yaml(
            with(std::string(kPinnedText), "\nobject_version: 1\n", std::string("\nobject_version: ") + version + "\n"));
        ASSERT_FALSE(r);
        EXPECT_EQ(r.error().code, spade::Code::schema_mismatch);
        EXPECT_TRUE(contains(r.error().context, std::string("object file is schema version ") + version +
                                                    ", this build reads version 1 only"))
            << r.error().context;
    }
    // A version-2 file's keys are not reported as unknown: the version comes first.
    const spade::Result<ObjectFile> newer = spade::scene::object_from_yaml(
        with(with(std::string(kPinnedText), "\nobject_version: 1\n", "\nobject_version: 2\n"), "\nmaterials:\n",
             "\nmesh_collider: {file: \"m.spmesh\"}\nmaterials:\n"));
    ASSERT_FALSE(newer);
    EXPECT_EQ(newer.error().code, spade::Code::schema_mismatch);

    const spade::Result<ObjectFile> word = spade::scene::object_from_yaml(
        with(std::string(kPinnedText), "\nobject_version: 1\n", "\nobject_version: one\n"));
    ASSERT_FALSE(word);
    EXPECT_EQ(word.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(word.error().context, "object_version: 'one' is not a non-negative decimal integer"))
        << word.error().context;

    const spade::Result<ObjectFile> list = spade::scene::object_from_yaml("[1, 2]\n");
    EXPECT_TRUE(contains(why(list), "an object file's top level must be a mapping with an 'object_version' key"))
        << why(list);
    const spade::Result<ObjectFile> no_version = spade::scene::object_from_yaml("object: {}\n");
    EXPECT_TRUE(contains(why(no_version), "missing required key 'object_version' in object file")) << why(no_version);
}

TEST(ObjectFile, UnknownMissingAndDuplicateKeysAreRefusedAtEveryLevel) {
    const std::string base(kPinnedText);
    ASSERT_OK(spade::scene::object_from_yaml(base));
    struct Case {
        std::string what;
        std::string from;
        std::string to;
        std::string needle;
    };
    const std::string pose = "\n  pose: {position: [0, 0.5, 0], orientation: [1, 0, 0, 0], scale: 1}\n";
    const std::string material =
        "  - {name: \"red\", base_color: [0.800000012, 0.100000001, 0.100000001, 1], shading: \"lambert\"}\n";
    const std::vector<Case> cases = {
        // The top level.
        {"a misspelt object", "\nobject:\n  name: \"ball\"\n", "\nobject_block:\n  name: \"ball\"\n",
         "unknown key 'object_block' in object file -- object schema v1 does not define it (line "},
        {"an unknown top-level key", "\nobject_version: 1\n", "\nobject_version: 1\nbogus: 1\n",
         "unknown key 'bogus' in object file -- object schema v1 does not define it (line "},
        {"a duplicate top-level key", "\nobject_version: 1\n", "\nobject_version: 1\nobject_version: 1\n",
         "duplicate key 'object_version' in object file"},
        // The object block: the scene's key set, blamed on the object's schema.
        {"an unknown key in the object", "\n  name: \"ball\"\n", "\n  name: \"ball\"\n  bogus: 1\n",
         "unknown key 'bogus' in object -- object schema v1 does not define it (line "},
        {"a missing key in the object", pose, "\n", "missing required key 'pose' in object (line "},
        {"a duplicate key in the object", "\n  name: \"ball\"\n", "\n  name: \"ball\"\n  name: \"ball\"\n",
         "duplicate key 'name' in object"},
        {"an unknown key in the pose", "scale: 1}\n", "scale: 1, bogus: 1}\n",
         "unknown key 'bogus' in object.pose -- object schema v1 does not define it (line "},
        {"a missing key in the pose", ", scale: 1}\n", "}\n", "missing required key 'scale' in object.pose (line "},
        {"an unknown key in the collider", "\n    sdf:\n", "\n    bogus: 1\n    sdf:\n",
         "unknown key 'bogus' in object.collider -- object schema v1 does not define it (line "},
        {"an unknown key in the collider's sdf", "\n      node_materials: [\"red\"]\n",
         "\n      node_materials: [\"red\"]\n      bogus: 1\n",
         "unknown key 'bogus' in object.collider.sdf -- object schema v1 does not define it (line "},
        {"a missing key in the collider's sdf", "\n      node_materials: [\"red\"]\n", "\n",
         "missing required key 'node_materials' in object.collider.sdf (line "},
        {"an unknown key in a node", "{prim: sphere, ", "{bogus: 1, prim: sphere, ",
         "unknown key 'bogus' in object.collider.sdf.nodes[0] -- object schema v1 does not define it (line "},
        {"an unknown key in the visual", "material: \"red\"}\n", "material: \"red\", bogus: 1}\n",
         "unknown key 'bogus' in object.visual -- object schema v1 does not define it (line "},
        // The materials: the scene's encoding, blamed on the object's schema.
        {"an unknown key in a material", "shading: \"lambert\"}", "shading: \"lambert\", bogus: 1}",
         "unknown key 'bogus' in materials[0] -- object schema v1 does not define it (line "},
        {"a missing key in a material", ", shading: \"lambert\"}", "}",
         "missing required key 'shading' in materials[0] (line "},
        {"a duplicate key in a material", "shading: \"lambert\"}", "shading: \"lambert\", shading: \"lambert\"}",
         "duplicate key 'shading' in materials[0]"},
        {"materials that are not a sequence", "\nmaterials:\n" + material, "\nmaterials: 3\n",
         "materials must be a sequence (line "},
        // Values, by their dotted paths.
        {"a float that is not a number", "scale: 1}\n", "scale: big}\n",
         "object.pose.scale: 'big' is not a decimal number (line "},
        {"a non-finite colour", "base_color: [0.800000012, ", "base_color: [nan, ",
         "materials[0].base_color[0]: 'nan' is not finite"},
        {"an unknown shading", "shading: \"lambert\"}", "shading: \"glossy\"}",
         "unknown material shading 'glossy' in materials[0] (line "},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        const spade::Result<ObjectFile> r = spade::scene::object_from_yaml(with(base, c.from, c.to));
        ASSERT_FALSE(r) << "accepted";
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
        EXPECT_TRUE(contains(r.error().context, c.needle)) << r.error().context;
        EXPECT_FALSE(contains(r.error().context, "scene schema")) << "an object file blames its own schema";
    }

    // The object key is required, and materials is not.
    const std::size_t object_at = base.find("\n# The object:");
    const std::size_t materials_at = base.find("\n# The materials");
    ASSERT_NE(object_at, std::string::npos);
    ASSERT_NE(materials_at, std::string::npos);
    const spade::Result<ObjectFile> missing =
        spade::scene::object_from_yaml(base.substr(0, object_at) + base.substr(materials_at));
    EXPECT_TRUE(contains(why(missing), "missing required key 'object' in object file")) << why(missing);
    const spade::Result<ObjectFile> no_materials = spade::scene::object_from_yaml(base.substr(0, materials_at));
    ASSERT_OK(no_materials);
    EXPECT_TRUE(no_materials->materials.empty()) << "\"red\" is a world's material now";

    // An object that is not a mapping.
    const spade::Result<ObjectFile> scalar = spade::scene::object_from_yaml(
        base.substr(0, object_at) + "\nobject: 3\n" + base.substr(materials_at));
    EXPECT_TRUE(contains(why(scalar), "object must be a mapping (line ")) << why(scalar);
}

// ===========================================================================
// One validator
// ===========================================================================

TEST(ObjectFile, ABadMaterialIsRefusedWithThePalettesMessage) {
    // The palette's rules, the one function (validate_material), as the scene
    // runs it: the scene's message, with the object file's label.
    struct Case {
        std::string what;
        std::function<void(MaterialDesc&)> edit;
        std::string message;
    };
    const std::vector<Case> cases = {
        {"an empty name", [](MaterialDesc& m) { m.name.clear(); }, "object file material 0 has an empty name"},
        {"a non-finite colour", [](MaterialDesc& m) { m.base_color.y = std::numeric_limits<float>::quiet_NaN(); },
         "object file material 0 has a non-finite base_color"},
        {"an unknown shading", [](MaterialDesc& m) { m.shading = static_cast<MaterialShading>(7); },
         "object file material 0 has an unknown shading value"},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        ObjectFile file = ball_object();
        c.edit(file.materials[0]);
        const spade::Result<void> validated = spade::scene::validate_object_file(file);
        ASSERT_FALSE(validated);
        EXPECT_EQ(validated.error().code, spade::Code::invalid_argument);
        EXPECT_EQ(validated.error().context, c.message);
        const spade::Result<std::string> written = spade::scene::object_to_yaml(file);
        ASSERT_FALSE(written);
        EXPECT_EQ(written.error().context, c.message) << "the writer validates first";

        SceneDesc scene = scene_of(file.object);
        scene.materials = file.materials;
        const spade::Result<void> in_scene = spade::scene::validate_scene(scene);
        ASSERT_FALSE(in_scene) << "the scene refuses it";
        EXPECT_EQ(with(in_scene.error().context, "scene material 0", "object file material 0"), c.message);
    }

    // Through the reader: an empty name.
    const spade::Result<ObjectFile> read =
        spade::scene::object_from_yaml(with(std::string(kPinnedText), "{name: \"red\", ", "{name: \"\", "));
    ASSERT_FALSE(read);
    EXPECT_EQ(read.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(read.error().context, "object file material 0 has an empty name");
}

TEST(ObjectFile, ADuplicateMaterialNameIsRefused) {
    ObjectFile file = ball_object();
    file.materials.push_back(red());
    const spade::Result<void> validated = spade::scene::validate_object_file(file);
    ASSERT_FALSE(validated);
    EXPECT_EQ(validated.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(validated.error().context, "duplicate material name 'red'");
    EXPECT_EQ(why(spade::scene::object_to_yaml(file)), "duplicate material name 'red'");

    // A different definition under the same name is refused alike.
    file.materials[1].shading = MaterialShading::emissive;
    EXPECT_EQ(why(spade::scene::validate_object_file(file)), "duplicate material name 'red'");

    // Through the reader.
    const std::string line =
        "  - {name: \"red\", base_color: [0.800000012, 0.100000001, 0.100000001, 1], shading: \"lambert\"}\n";
    const spade::Result<ObjectFile> read =
        spade::scene::object_from_yaml(with(std::string(kPinnedText), line, line + line));
    EXPECT_EQ(why(read), "duplicate material name 'red'");
}

TEST(ObjectFile, AnUnusedMaterialIsRefusedByName) {
    ObjectFile file = ball_object();
    file.materials.push_back(MaterialDesc{"green", {0.1f, 0.8f, 0.1f, 1.0f}, MaterialShading::lambert});
    const std::string message =
        "material 'green' is listed, but the object names it nowhere (in collider_materials or "
        "visual.material); an object file lists only the materials its object uses";
    const spade::Result<void> validated = spade::scene::validate_object_file(file);
    ASSERT_FALSE(validated);
    EXPECT_EQ(validated.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(validated.error().context, message);
    EXPECT_EQ(why(spade::scene::object_to_yaml(file)), message);

    const spade::Result<ObjectFile> read = spade::scene::object_from_yaml(
        std::string(kPinnedText) + "  - {name: \"green\", base_color: [0.1, 0.8, 0.1, 1], shading: \"lambert\"}\n");
    ASSERT_FALSE(read);
    EXPECT_EQ(read.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(read.error().context, message);

    // Named by a collider node alone, or by the visual alone, a material is used.
    EXPECT_OK(spade::scene::validate_object_file(gate_object())) << "\"blue\" is named by a node alone";
    ObjectFile visual_only = ball_object();
    visual_only.object.collider_materials.clear();
    EXPECT_OK(spade::scene::validate_object_file(visual_only));
    ObjectFile node_only = ball_object();
    node_only.object.visual = {};
    EXPECT_OK(spade::scene::validate_object_file(node_only));
}

TEST(ObjectFile, AWorldMaterialNeedNotBeListed) {
    // A name the object uses and does not list is a world's, resolved at
    // compose (SCN-004): the file is valid without it.
    ObjectFile file = gate_object();
    file.materials = {blue()};  // "red" is the world's
    ASSERT_OK(spade::scene::validate_object_file(file));
    const spade::Result<std::string> text = spade::scene::object_to_yaml(file);
    ASSERT_OK(text);
    const spade::Result<ObjectFile> back = spade::scene::object_from_yaml(*text);
    ASSERT_OK(back);
    expect_same_materials(file.materials, back->materials);
}

TEST(ObjectFile, ABadAssetIsRefusedWithTheScenesMessage) {
    // The per-asset rules are the scene's, in one function (TD-9): the same
    // message, word for word, except where it names the schema.
    struct Case {
        std::string what;
        std::function<void(SceneAsset&)> edit;
        bool names_the_schema;  // the scene's message names "scene schema v1"
    };
    const std::vector<Case> cases = {
        {"no collider and no visual",
         [](SceneAsset& a) {
             a.collider = {};
             a.collider_materials.clear();
             a.visual = {};
         },
         false},
        {"a non-unit orientation", [](SceneAsset& a) { a.pose.rotation = glm::quat(0.5f, 0.0f, 0.0f, 0.0f); }, false},
        {"a scale of 0", [](SceneAsset& a) { a.pose.scale = 0.0f; }, false},
        {"a malformed collider", [](SceneAsset& a) { a.collider.nodes[0].transform = 7; }, false},
        {"two node materials for one node", [](SceneAsset& a) { a.collider_materials.push_back("red"); }, false},
        {"a visual with a mesh and no material", [](SceneAsset& a) { a.visual.material.clear(); }, false},
        {"a node with padding the text cannot carry", [](SceneAsset& a) { a.collider.nodes[0]._pad = 1; }, true},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        ObjectFile file = ball_object();
        c.edit(file.object);
        const spade::Result<void> in_scene = spade::scene::validate_scene(scene_of(file.object));
        ASSERT_FALSE(in_scene) << "the scene refuses it";
        const spade::Result<void> in_file = spade::scene::validate_object_file(file);
        ASSERT_FALSE(in_file);
        EXPECT_EQ(in_file.error().code, in_scene.error().code);
        std::string expected = in_scene.error().context;
        if (c.names_the_schema) {
            expected = with(expected, "scene schema v1", "object schema v1");
        }
        EXPECT_EQ(in_file.error().context, expected);
        const spade::Result<std::string> written = spade::scene::object_to_yaml(file);
        ASSERT_FALSE(written);
        EXPECT_EQ(written.error().context, expected) << "the writer validates first";
    }

    // An object with no name: the scene's rule, the object's label.
    ObjectFile unnamed = ball_object();
    unnamed.object.name.clear();
    EXPECT_EQ(why(spade::scene::validate_scene(scene_of(unnamed.object))), "scene asset 0 has no name");
    EXPECT_EQ(why(spade::scene::validate_object_file(unnamed)), "object has no name");
    const spade::Result<ObjectFile> read =
        spade::scene::object_from_yaml(with(std::string(kPinnedText), "  name: \"ball\"\n", "  name: \"\"\n"));
    EXPECT_EQ(why(read), "object has no name");
}

// ===========================================================================
// load_object_file
// ===========================================================================

TEST(ObjectFile, LoadObjectFileNamesThePath) {
    const spade::test::TempDir dir("object-load");
    const std::filesystem::path absent = dir.path() / "absent.object.yaml";
    const spade::Result<ObjectFile> missing = spade::scene::load_object_file(absent);
    ASSERT_FALSE(missing);
    EXPECT_EQ(missing.error().code, spade::Code::io_error);
    EXPECT_TRUE(contains(why(missing), "cannot open object file '" + spade::path_text(absent) + "'")) << why(missing);

    // A parse diagnostic comes back prefixed with the path.
    const std::filesystem::path bad = dir.path() / "bad.object.yaml";
    write_text(bad, with(std::string(kPinnedText), "scale: 1}\n", "scale: big}\n"));
    const spade::Result<ObjectFile> parsed = spade::scene::load_object_file(bad);
    ASSERT_FALSE(parsed);
    EXPECT_EQ(parsed.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(parsed.error().context.starts_with(spade::path_text(bad) + ": object.pose.scale: 'big'"))
        << parsed.error().context;

    // A YAML syntax error too, and a version gate keeps its code.
    write_text(bad, "object_version: [1\n");
    const spade::Result<ObjectFile> syntax = spade::scene::load_object_file(bad);
    ASSERT_FALSE(syntax);
    EXPECT_EQ(syntax.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(syntax.error().context.starts_with(spade::path_text(bad) + ": ")) << syntax.error().context;
    write_text(bad, with(std::string(kPinnedText), "\nobject_version: 1\n", "\nobject_version: 2\n"));
    const spade::Result<ObjectFile> newer = spade::scene::load_object_file(bad);
    ASSERT_FALSE(newer);
    EXPECT_EQ(newer.error().code, spade::Code::schema_mismatch);
    EXPECT_TRUE(newer.error().context.starts_with(spade::path_text(bad) + ": object file is schema version 2"))
        << newer.error().context;

    // And a good one loads.
    const std::filesystem::path good = dir.path() / "ball.object.yaml";
    write_text(good, std::string(kPinnedText));
    const spade::Result<ObjectFile> loaded = spade::scene::load_object_file(good);
    ASSERT_OK(loaded);
    EXPECT_EQ(loaded->object.name, "ball");
}

// ===========================================================================
// add_object(): the one way to place an object (TD-9)
// ===========================================================================

TEST(AddObject, AppendsTheObjectUnderItsNameAtItsPoseWithItsMaterials) {
    SceneDesc scene = stage();
    ASSERT_OK(spade::scene::validate_scene(scene));
    const ObjectFile gate = gate_object();
    const SdfPose at = place_at({4.0f, 0.0f, -10.0f}, kYawPlus90, 2.0f);
    ASSERT_OK(spade::scene::add_object(scene, gate, "gate_1", at));

    ASSERT_EQ(scene.assets.size(), 2u);
    EXPECT_EQ(scene.assets[0].name, "marker") << "appended, never inserted";
    SceneAsset expected = gate.object;
    expected.name = "gate_1";
    expected.pose = at;
    expect_same_asset(expected, scene.assets[1]);
    expect_same_materials({white(), red(), blue()}, scene.materials);
    EXPECT_EQ(scene.vehicles.size(), 1u);
    EXPECT_EQ(scene.models.size(), 1u);
    EXPECT_OK(spade::scene::validate_scene(scene));
    EXPECT_OK(spade::scene::scene_to_yaml(scene));
}

TEST(AddObject, ASecondPlacementReusesIdenticalMaterials) {
    SceneDesc scene = stage();
    const ObjectFile gate = gate_object();
    ASSERT_OK(spade::scene::add_object(scene, gate, "gate_1", SdfPose{}));
    ASSERT_OK(spade::scene::add_object(scene, gate, "gate_2", place_at({0.0f, 0.0f, -5.0f}, kYawPlus90, 1.0f)));
    ASSERT_EQ(scene.assets.size(), 3u);
    EXPECT_EQ(scene.assets[1].name, "gate_1");
    EXPECT_EQ(scene.assets[2].name, "gate_2");
    expect_same_materials({white(), red(), blue()}, scene.materials);

    // An object that brings a material the scene already has, identical, from
    // another object: reused too.
    ASSERT_OK(spade::scene::add_object(scene, ball_object(), "ball_1", SdfPose{}));
    EXPECT_EQ(scene.assets.size(), 4u);
    expect_same_materials({white(), red(), blue()}, scene.materials);
}

TEST(AddObject, AConflictingMaterialIsRefusedNamingEveryFieldAndTheSceneIsUnchanged) {
    SceneDesc scene = stage();
    scene.materials.push_back(MaterialDesc{"red", {0.9f, 0.1f, 0.1f, 1.0f}, MaterialShading::unlit});
    const std::string before = text_of(scene);

    const spade::Result<void> r = spade::scene::add_object(scene, gate_object(), "gate_1", SdfPose{});
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(r.error().context,
              "material 'red': base_color differs (0.8 0.1 0.1 1 here, 0.9 0.1 0.1 1 in the scene), "
              "shading differs (lambert here, unlit in the scene)");
    EXPECT_EQ(text_of(scene), before) << "all or nothing: not even \"blue\" was added";
    EXPECT_EQ(scene.materials.size(), 2u);
    EXPECT_EQ(scene.assets.size(), 1u);

    // Every conflicting material is named, in the file's order.
    scene.materials.push_back(MaterialDesc{"blue", {0.1f, 0.2f, 0.9f, 1.0f}, MaterialShading::emissive});
    const std::string before_both = text_of(scene);
    const spade::Result<void> both = spade::scene::add_object(scene, gate_object(), "gate_1", SdfPose{});
    ASSERT_FALSE(both);
    EXPECT_EQ(both.error().context,
              "material 'red': base_color differs (0.8 0.1 0.1 1 here, 0.9 0.1 0.1 1 in the scene), "
              "shading differs (lambert here, unlit in the scene); "
              "material 'blue': shading differs (unlit here, emissive in the scene)");
    EXPECT_EQ(text_of(scene), before_both);

    // IDENTICAL is bit for bit: a -0 where the object has +0 differs.
    SceneDesc dark = stage();
    dark.materials.push_back(MaterialDesc{"black", {-0.0f, 0.0f, 0.0f, 1.0f}, MaterialShading::lambert});
    ObjectFile puck = ball_object();
    puck.materials = {MaterialDesc{"black", {0.0f, 0.0f, 0.0f, 1.0f}, MaterialShading::lambert}};
    puck.object.collider_materials = {"black"};
    puck.object.visual.material = "black";
    const std::string before_dark = text_of(dark);
    const spade::Result<void> zero = spade::scene::add_object(dark, puck, "puck_1", SdfPose{});
    ASSERT_FALSE(zero);
    EXPECT_EQ(zero.error().context, "material 'black': base_color differs (0 0 0 1 here, -0 0 0 1 in the scene)");
    EXPECT_EQ(text_of(dark), before_dark);
}

TEST(AddObject, ATakenNameIsRefusedAndTheSceneIsUnchanged) {
    SceneDesc scene = stage();
    const std::string before = text_of(scene);

    const spade::Result<void> asset = spade::scene::add_object(scene, gate_object(), "marker", SdfPose{});
    ASSERT_FALSE(asset);
    EXPECT_EQ(asset.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(asset.error().context,
              "name 'marker' is taken by an asset in the scene; names are unique across assets and vehicles "
              "together, and add_object never renames");
    EXPECT_EQ(text_of(scene), before);

    const spade::Result<void> vehicle = spade::scene::add_object(scene, gate_object(), "quad_1", SdfPose{});
    ASSERT_FALSE(vehicle);
    EXPECT_EQ(vehicle.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(vehicle.error().context,
              "name 'quad_1' is taken by a vehicle in the scene; names are unique across assets and vehicles "
              "together, and add_object never renames");
    EXPECT_EQ(text_of(scene), before);

    // A model's or a material's name is another namespace: free for an object.
    ASSERT_OK(spade::scene::add_object(scene, gate_object(), "spotter", SdfPose{}));
    ASSERT_OK(spade::scene::add_object(scene, gate_object(), "white", SdfPose{}));
    EXPECT_EQ(scene.assets.size(), 3u);
}

TEST(AddObject, AnInvalidObjectNameOrPoseIsRefusedAndTheSceneIsUnchanged) {
    SceneDesc scene = stage();
    const std::string before = text_of(scene);
    const auto refused = [&](const ObjectFile& object, std::string_view name, const SdfPose& pose,
                             const std::string& message) {
        const spade::Result<void> r = spade::scene::add_object(scene, object, name, pose);
        ASSERT_FALSE(r);
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
        EXPECT_EQ(r.error().context, message);
        EXPECT_EQ(text_of(scene), before);
    };

    {
        SCOPED_TRACE("an object the object file refuses");
        ObjectFile unused = ball_object();
        unused.materials.push_back(MaterialDesc{"green", {0.1f, 0.8f, 0.1f, 1.0f}, MaterialShading::lambert});
        refused(unused, "ball_1", SdfPose{}, why(spade::scene::validate_object_file(unused)));
    }
    {
        SCOPED_TRACE("no name");
        refused(gate_object(), "", SdfPose{}, "scene asset 1 has no name");
    }
    {
        SCOPED_TRACE("a scale of 0");
        refused(gate_object(), "gate_1", place_at({0.0f, 0.0f, 0.0f}, glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 0.0f),
                "asset 'gate_1': pose scale must be > 0 (uniform scale only)");
    }
    {
        SCOPED_TRACE("a non-unit orientation");
        refused(gate_object(), "gate_1", place_at({0.0f, 0.0f, 0.0f}, glm::quat(0.5f, 0.0f, 0.0f, 0.0f), 1.0f),
                "asset 'gate_1': pose orientation must be a unit quaternion");
    }
    {
        SCOPED_TRACE("a non-finite position");
        refused(gate_object(), "gate_1",
                place_at({std::numeric_limits<float>::infinity(), 0.0f, 0.0f}, glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                         1.0f),
                "asset 'gate_1': pose must be finite");
    }
}

TEST(AddObject, AWorldMaterialTheObjectNamesIsResolvedWhenTheSceneComposes) {
    // A pole of the world's "steel", which its file does not list.
    ObjectFile pole;
    pole.object.name = "pole";
    pole.object.collider.transforms = {SdfTransform{}};
    pole.object.collider.nodes = {prim(SdfPrim::capsule, 0, {0.05f, 1.0f, 0.0f, 0.0f})};
    pole.object.collider_materials = {"steel"};
    pole.object.visual = {"props/pole", "steel"};
    ASSERT_OK(spade::scene::validate_object_file(pole));

    const auto ground = [] {
        spade::WorldBuilder b;
        b.name("ground")
            .environment(spade::Environment{})
            .capacities(spade::Capacities{1, 2, 3, 4})
            .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
        return b;
    };
    const spade::Result<spade::WorldDesc> world =
        ground().material(MaterialDesc{"steel", {0.6f, 0.6f, 0.65f, 1.0f}, MaterialShading::lambert}).build();
    ASSERT_OK(world);
    const spade::Result<uint64_t> hash = spade::scene::world_hash(*world);
    ASSERT_OK(hash);

    SceneDesc scene;
    scene.name = "poles";
    scene.world.file = "ground.world.yaml";
    scene.world.hash = *hash;
    ASSERT_OK(spade::scene::add_object(scene, pole, "pole_1", place_at({2.0f, 1.0f, 0.0f}, kYawPlus90, 1.0f)));
    EXPECT_TRUE(scene.materials.empty()) << "nothing to merge: the world brings it";

    const spade::Result<spade::scene::ComposedScene> composed = spade::scene::compose(scene, *world);
    ASSERT_OK(composed);
    const std::vector<MaterialDesc>& palette = composed->world.materials;
    const auto steel = std::find_if(palette.begin(), palette.end(), [](const MaterialDesc& m) { return m.name == "steel"; });
    ASSERT_NE(steel, palette.end());
    const uint32_t steel_index = static_cast<uint32_t>(steel - palette.begin());
    ASSERT_EQ(composed->world.props.size(), 1u);
    EXPECT_EQ(composed->world.props[0].material, steel_index) << "the visual resolved to the world's steel";
    const std::vector<uint32_t>& node_materials = composed->world.sdf.node_materials;
    EXPECT_NE(std::find(node_materials.begin(), node_materials.end(), steel_index), node_materials.end())
        << "the collider's node resolved to the world's steel";

    // Over a world without it, compose() refuses it by name: add_object() never
    // checked it, because it cannot see the world.
    const spade::Result<spade::WorldDesc> bare = ground().build();
    ASSERT_OK(bare);
    scene.world.hash = spade::scene::world_hash(*bare).value_or(0);
    const spade::Result<spade::scene::ComposedScene> unresolved = spade::scene::compose(scene, *bare);
    ASSERT_FALSE(unresolved);
    EXPECT_EQ(unresolved.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(unresolved.error().context, "steel")) << unresolved.error().context;
}
