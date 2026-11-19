// ---------------------------------------------------------------------------
// test_part_stand_in.cpp -- spade_part_stand_in (L88l; tools/part_stand_in): the stand-in boxes a part with store meshes carries, so a box shows the
// drone's layout where the (local, never published) part store is absent. The rule it serves, in the library contract: a part with `meshes` draws
// them when the store has them, else its `shapes`; physics and compile use the meshes only. The tool is compiled into spade_tests from the tool's
// own sources.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "scene/part_blocks.hpp"
#include "unique_temp.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"

#include "../tools/part_stand_in/stand_in.hpp"

namespace {

namespace fs = std::filesystem;
namespace si = spade::stand_in;

const std::string kShaA(64, 'a');
const std::string kShaB(64, 'b');

fs::path fresh(const std::string& name) {
    static std::vector<std::unique_ptr<spade::test::TempDir>> dirs;
    dirs.push_back(std::make_unique<spade::test::TempDir>("part_stand_in_" + name));
    return dirs.back()->path();
}

// A cube of side `side` metres from the origin, saved as <store>/<sha>/<file>.
void save_cube(const fs::path& store, const std::string& sha, const std::string& file, double side) {
    static constexpr uint32_t kTriangles[12][3] = {{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
                                                   {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};
    spade::MeshInput in;
    in.unit = spade::MeshUnit::metre;
    for (uint32_t v = 0; v < 8; ++v) {
        in.positions.push_back(glm::dvec3(static_cast<double>(v & 1u), static_cast<double>((v >> 1) & 1u), static_cast<double>((v >> 2) & 1u)) * side);
    }
    for (const auto& t : kTriangles) in.indices.insert(in.indices.end(), {t[0], t[1], t[2]});
    auto mesh = spade::make_mesh(in, "cube");
    ASSERT_TRUE(mesh.has_value()) << mesh.error().context;
    fs::create_directories(store / sha);
    auto saved = spade::save_mesh_file(store / sha / file, **mesh);
    ASSERT_TRUE(saved.has_value()) << saved.error().context;
}

std::string base64(const std::vector<uint8_t>& bytes) {
    static const char* kDigits = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (std::size_t i = 0; i < bytes.size(); i += 3) {
        const uint32_t b0 = bytes[i], b1 = i + 1 < bytes.size() ? bytes[i + 1] : 0u, b2 = i + 2 < bytes.size() ? bytes[i + 2] : 0u;
        const uint32_t n = (b0 << 16) | (b1 << 8) | b2;
        out += kDigits[(n >> 18) & 63u];
        out += kDigits[(n >> 12) & 63u];
        out += i + 1 < bytes.size() ? kDigits[(n >> 6) & 63u] : '=';
        out += i + 2 < bytes.size() ? kDigits[n & 63u] : '=';
    }
    return out;
}

// A glTF with two one-triangle primitives, the first in material "m/prop" (x 0..0.2, y 0..0.01, z 0..0.2), the second in "m/bell" (y -0.03..-0.01).
fs::path write_two_material_gltf(const fs::path& file) {
    const std::array<float, 18> positions = {0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.01f, 0.2f,
                                             0.0f, -0.03f, 0.0f, 0.05f, -0.03f, 0.0f, 0.0f, -0.01f, 0.05f};
    std::vector<uint8_t> bytes(reinterpret_cast<const uint8_t*>(positions.data()), reinterpret_cast<const uint8_t*>(positions.data()) + 72);
    for (int prim = 0; prim < 2; ++prim) {
        for (const uint16_t i : {uint16_t{0}, uint16_t{1}, uint16_t{2}}) {
            bytes.push_back(static_cast<uint8_t>(i & 0xffu));
            bytes.push_back(static_cast<uint8_t>(i >> 8));
        }
    }
    const std::string json =
        "{\"asset\":{\"version\":\"2.0\"},"
        "\"buffers\":[{\"byteLength\":84,\"uri\":\"data:application/octet-stream;base64," + base64(bytes) + "\"}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36},{\"buffer\":0,\"byteOffset\":36,\"byteLength\":36},"
        "{\"buffer\":0,\"byteOffset\":72,\"byteLength\":6},{\"buffer\":0,\"byteOffset\":78,\"byteLength\":6}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"},{\"bufferView\":1,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"},"
        "{\"bufferView\":2,\"componentType\":5123,\"count\":3,\"type\":\"SCALAR\"},{\"bufferView\":3,\"componentType\":5123,\"count\":3,\"type\":\"SCALAR\"}],"
        "\"materials\":[{\"name\":\"m/prop\"},{\"name\":\"m/bell\"}],"
        "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"indices\":2,\"material\":0},{\"attributes\":{\"POSITION\":1},\"indices\":3,\"material\":1}]}]}";
    std::ofstream out(file, std::ios::binary);
    out << json;
    return file;
}

// A blocks-only part with the given store meshes (sha, file), a source, and a mass citation.
std::string part_text(const std::vector<std::pair<std::string, std::string>>& meshes, bool with_source = true) {
    std::string t = "part_version: 2\nid: \"part.payload.t\"\nmass_properties: {mass: 0.1}\n";
    if (!meshes.empty()) {
        t += "meshes:\n";
        for (const auto& [sha, file] : meshes) t += "  - {store: {sha256: \"" + sha + "\", file: \"" + file + "\"}}\n";
    }
    if (with_source) t += "source: {id: \"s\", url: \"https://x.org/a\", license: \"none found\", retrieved: \"2026-11-19\"}\n";
    t += "sources:\n  mass_properties.mass: {grade: assumed, note: \"a stand-in mass\"}\n";
    return t;
}

TEST(PartStandIn, AnSpmeshGivesItsBoundingBox) {
    const fs::path store = fresh("spmesh");
    save_cube(store, kShaA, "a.spmesh", 0.1);
    const auto boxes = si::boxes_of_file(store / kShaA / "a.spmesh", {});
    ASSERT_TRUE(boxes.has_value()) << boxes.error();
    ASSERT_EQ(boxes->size(), 1u);
    for (int a = 0; a < 3; ++a) {
        EXPECT_NEAR((*boxes)[0].size[a], 0.1, 1e-6);
        EXPECT_NEAR((*boxes)[0].center[a], 0.05, 1e-6);
    }
}

TEST(PartStandIn, AGltfGivesOneBoxPerMaterialAndSkipsTheOnesNamed) {
    const fs::path dir = fresh("gltf");
    const fs::path file = write_two_material_gltf(dir / "r.gltf");
    const auto both = si::boxes_of_file(file, {});
    ASSERT_TRUE(both.has_value()) << both.error();
    EXPECT_EQ(both->size(), 2u);
    si::Options skip;
    skip.skip_materials = {"bell"};
    const auto prop = si::boxes_of_file(file, skip);
    ASSERT_TRUE(prop.has_value()) << prop.error();
    ASSERT_EQ(prop->size(), 1u);
    EXPECT_NEAR((*prop)[0].size.x, 0.2, 1e-6);
    EXPECT_NEAR((*prop)[0].size.y, 0.01, 1e-6);
    EXPECT_NEAR((*prop)[0].size.z, 0.2, 1e-6);
    EXPECT_NEAR((*prop)[0].center.y, 0.005, 1e-6);
}

TEST(PartStandIn, ADocumentOfAnotherKindIsRefusedByName) {
    const fs::path dir = fresh("kind");
    { std::ofstream(dir / "x.txt") << "x"; }
    const auto r = si::boxes_of_file(dir / "x.txt", {});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().find("x.txt"), std::string::npos) << r.error();
}

TEST(PartStandIn, APartGetsOneBoxPerObjectOverTheLimitAndAValidCitedShapesBlock) {
    const fs::path store = fresh("part");
    save_cube(store, kShaA, "a.spmesh", 0.1);    // 1000 cm3
    save_cube(store, kShaB, "b.spmesh", 0.005);  // 0.125 cm3, under 0.2 cm3
    const std::string text = part_text({{kShaA, "a.spmesh"}, {kShaB, "b.spmesh"}});
    const auto out = si::add_stand_in(text, store, {});
    ASSERT_TRUE(out.has_value()) << out.error();
    EXPECT_EQ(out->boxes, 1u);
    EXPECT_EQ(out->skipped, 1u);
    EXPECT_NE(out->text.find(si::kMarker), std::string::npos);
    EXPECT_EQ(out->text.compare(0, text.size(), text), 0) << "the part's own text is kept as it was";
    const auto blocks = spade::scene::read_part_blocks(out->text);
    ASSERT_TRUE(blocks.has_value()) << blocks.error().context;
    ASSERT_EQ(blocks->shapes.size(), 1u);
    EXPECT_EQ(blocks->shapes[0].kind, spade::scene::PartShape::Kind::box);
    EXPECT_NEAR(blocks->shapes[0].size[0], 0.1, 1e-5);
    EXPECT_NEAR(blocks->shapes[0].pos.x, 0.05, 1e-5);
    EXPECT_EQ(blocks->meshes.size(), 2u) << "the meshes stay: the boxes are the fallback";
    // The citation: grade derived, the part's own source, a note that says it is generated.
    EXPECT_NE(out->text.find("shapes: {source: \"s\", grade: derived"), std::string::npos) << out->text;
    EXPECT_NE(out->text.find("tools/part_stand_in"), std::string::npos);
}

TEST(PartStandIn, RunningItAgainGivesTheSameTextAndAPartWithNoMeshesIsLeftAlone) {
    const fs::path store = fresh("again");
    save_cube(store, kShaA, "a.spmesh", 0.1);
    const std::string text = part_text({{kShaA, "a.spmesh"}});
    const auto once = si::add_stand_in(text, store, {});
    ASSERT_TRUE(once.has_value()) << once.error();
    const auto twice = si::add_stand_in(once->text, store, {});
    ASSERT_TRUE(twice.has_value()) << twice.error();
    EXPECT_EQ(once->text, twice->text);
    const std::string bare = part_text({});
    const auto none = si::add_stand_in(bare, store, {});
    ASSERT_TRUE(none.has_value()) << none.error();
    EXPECT_EQ(none->boxes, 0u);
    EXPECT_EQ(none->text, bare);
}

TEST(PartStandIn, WhenEveryObjectIsUnderTheLimitTheLargestIsKept) {
    const fs::path store = fresh("small");
    save_cube(store, kShaA, "a.spmesh", 0.004);
    save_cube(store, kShaB, "b.spmesh", 0.005);
    const auto out = si::add_stand_in(part_text({{kShaA, "a.spmesh"}, {kShaB, "b.spmesh"}}), store, {});
    ASSERT_TRUE(out.has_value()) << out.error();
    EXPECT_EQ(out->boxes, 1u);
    const auto blocks = spade::scene::read_part_blocks(out->text);
    ASSERT_TRUE(blocks.has_value()) << blocks.error().context;
    ASSERT_EQ(blocks->shapes.size(), 1u);
    EXPECT_NEAR(blocks->shapes[0].size[0], 0.005, 1e-6);
}

TEST(PartStandIn, ARefusalNamesTheProblemAndWritesNothing) {
    const fs::path store = fresh("refuse");
    save_cube(store, kShaA, "a.spmesh", 0.1);
    const auto missing = si::add_stand_in(part_text({{kShaB, "gone.spmesh"}}), store, {});
    ASSERT_FALSE(missing.has_value());
    EXPECT_NE(missing.error().find("gone.spmesh"), std::string::npos) << missing.error();
    const auto no_source = si::add_stand_in(part_text({{kShaA, "a.spmesh"}}, false), store, {});
    ASSERT_FALSE(no_source.has_value());
    EXPECT_NE(no_source.error().find("source"), std::string::npos) << no_source.error();
    const auto not_a_part = si::add_stand_in("this: [is not a part", store, {});
    EXPECT_FALSE(not_a_part.has_value());
}

}  // namespace
