#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "part_import_fixtures.hpp"
#include "unique_temp.hpp"
#include "world/mesh_import.hpp"

// The collecting mode of the mesh readers (world/mesh_import.hpp; L88, spade_part_import's --inspect): every object of a file with its mesh
// or its own gate refusal, where the plain reader stops at the first bad one. The plain readers are the reference: the same objects for a
// file that passes, the same error for one that does not.

namespace {

namespace fs = std::filesystem;
using namespace spade;
using namespace spade::partimporttest;

[[nodiscard]] std::span<const std::byte> as_bytes(const std::string& s) {
    return std::as_bytes(std::span<const char>(s));
}

// Two objects in one OBJ: `a_faces` and `b_faces` triangles of a cube each (a cube has 12; fewer is an open mesh).
[[nodiscard]] std::string two_object_obj(std::size_t a_faces, std::size_t b_faces) {
    std::string t;
    uint32_t base = 0;
    for (const auto& [name, faces] : {std::pair<const char*, std::size_t>{"first", a_faces}, {"second", b_faces}}) {
        t += std::string("o ") + name + "\n";
        for (uint32_t v = 0; v < 8; ++v) {
            const glm::vec3 c = corner(v, 1.0f);
            t += "v " + std::to_string(c.x + static_cast<float>(base)) + " " + std::to_string(c.y) + " " + std::to_string(c.z) + "\n";
        }
        for (std::size_t i = 0; i < faces; ++i) {
            t += "f " + std::to_string(kCube[i][0] + 1 + base) + " " + std::to_string(kCube[i][1] + 1 + base) + " " +
                 std::to_string(kCube[i][2] + 1 + base) + "\n";
        }
        base += 8;
    }
    return t;
}

struct Files {
    test::TempDir tmp{"mesh_import_objects"};
    [[nodiscard]] fs::path put(const std::string& name, std::string_view bytes) const {
        const fs::path p = tmp.path() / name;
        write_file(p, bytes);
        return p;
    }
};

TEST(MeshImportObjects, AnStlThatPassesIsTheSameObjectAsThePlainReaders) {
    Files f;
    const fs::path p = f.put("cube.stl", cube_stl(2.0f));
    const auto plain = read_stl(p, MeshUnit::millimetre);
    const auto all = read_stl_objects(p, MeshUnit::millimetre);
    ASSERT_TRUE(plain.has_value());
    ASSERT_TRUE(all.has_value()) << all.error().context;
    ASSERT_EQ(all->size(), 1u);
    EXPECT_EQ((*all)[0].name, (*plain)[0].name);
    EXPECT_EQ((*all)[0].unit, MeshUnit::millimetre);
    ASSERT_TRUE((*all)[0].mesh.has_value());
    EXPECT_EQ((*(*all)[0].mesh)->content_hash(), (*plain)[0].mesh->content_hash());
}

TEST(MeshImportObjects, AnOpenStlIsOneRefusedOutcomeWithThePlainReadersError) {
    Files f;
    const fs::path p = f.put("open.stl", cube_stl(1.0f, 11));
    const auto plain = read_stl(p, MeshUnit::metre);
    ASSERT_FALSE(plain.has_value());
    const auto all = read_stl_objects(p, MeshUnit::metre);
    ASSERT_TRUE(all.has_value()) << all.error().context;
    ASSERT_EQ(all->size(), 1u);
    EXPECT_EQ((*all)[0].name, "open");
    ASSERT_FALSE((*all)[0].mesh.has_value());
    EXPECT_EQ((*all)[0].mesh.error(), plain.error());
    ASSERT_TRUE((*all)[0].kind.has_value());
    EXPECT_EQ(*(*all)[0].kind, MeshRefusal::not_closed);
}

TEST(MeshImportObjects, AnObjWithOneOpenObjectReportsBothInOrder) {
    Files f;
    const fs::path p = f.put("two.obj", two_object_obj(12, 11));
    const auto plain = read_obj(p, MeshUnit::metre);
    ASSERT_FALSE(plain.has_value());
    const auto all = read_obj_objects(p, MeshUnit::metre);
    ASSERT_TRUE(all.has_value()) << all.error().context;
    ASSERT_EQ(all->size(), 2u);
    EXPECT_EQ((*all)[0].name, "first");
    EXPECT_TRUE((*all)[0].mesh.has_value());
    EXPECT_EQ((*all)[1].name, "second");
    ASSERT_FALSE((*all)[1].mesh.has_value());
    EXPECT_EQ((*all)[1].mesh.error(), plain.error());
    EXPECT_FALSE((*all)[0].kind.has_value());
    ASSERT_TRUE((*all)[1].kind.has_value());
    EXPECT_EQ(*(*all)[1].kind, MeshRefusal::not_closed);
    // the other way round: the plain reader's error is the FIRST refusal
    const fs::path q = f.put("two2.obj", two_object_obj(11, 12));
    const auto plain2 = read_obj(q, MeshUnit::metre);
    const auto all2 = read_obj_objects(q, MeshUnit::metre);
    ASSERT_TRUE(all2.has_value());
    ASSERT_EQ(all2->size(), 2u);
    ASSERT_FALSE((*all2)[0].mesh.has_value());
    EXPECT_EQ((*all2)[0].mesh.error(), plain2.error());
    EXPECT_TRUE((*all2)[1].mesh.has_value());
    ASSERT_TRUE((*all2)[0].kind.has_value());
    EXPECT_EQ(*(*all2)[0].kind, MeshRefusal::not_closed);
    EXPECT_FALSE((*all2)[1].kind.has_value());
}

TEST(MeshImportObjects, A3mfWithTheMiddleObjectOpenReportsAllThree) {
    Files f;
    const fs::path p = f.put("three.3mf", three_mf({cube_object(1, "a", 10.0), cube_object(2, "b", 10.0, 40.0, 11), cube_object(3, "c", 10.0, 80.0)}));
    const auto plain = read_3mf(p);
    ASSERT_FALSE(plain.has_value());
    const auto all = read_3mf_objects(p);
    ASSERT_TRUE(all.has_value()) << all.error().context;
    ASSERT_EQ(all->size(), 3u);
    EXPECT_EQ((*all)[0].name, "a");
    EXPECT_EQ((*all)[1].name, "b");
    EXPECT_EQ((*all)[2].name, "c");
    EXPECT_TRUE((*all)[0].mesh.has_value());
    ASSERT_FALSE((*all)[1].mesh.has_value());
    EXPECT_EQ((*all)[1].mesh.error(), plain.error());
    EXPECT_TRUE((*all)[2].mesh.has_value());
    EXPECT_EQ((*all)[0].unit, MeshUnit::millimetre);
    EXPECT_FALSE((*all)[0].kind.has_value());
    ASSERT_TRUE((*all)[1].kind.has_value());
    EXPECT_EQ(*(*all)[1].kind, MeshRefusal::not_closed);
    EXPECT_FALSE((*all)[2].kind.has_value());
}

TEST(MeshImportObjects, A3mfThatPassesGivesThePlainReadersObjects) {
    Files f;
    const fs::path p = f.put("two.3mf", three_mf({cube_object(1, "a", 10.0), cube_object(2, "b", 20.0, 40.0)}));
    const auto plain = read_3mf(p);
    const auto all = read_3mf_objects(p);
    ASSERT_TRUE(plain.has_value());
    ASSERT_TRUE(all.has_value()) << all.error().context;
    ASSERT_EQ(all->size(), plain->size());
    for (std::size_t i = 0; i < plain->size(); ++i) {
        EXPECT_EQ((*all)[i].name, (*plain)[i].name);
        EXPECT_EQ((*all)[i].unit, (*plain)[i].unit);
        ASSERT_TRUE((*all)[i].mesh.has_value());
        EXPECT_EQ((*(*all)[i].mesh)->content_hash(), (*plain)[i].mesh->content_hash());
    }
}

TEST(MeshImportObjects, AGlbThatPassesOrIsOpenIsReportedLikeTheOthers) {
    const std::string good = cube_glb(1.0f, "frame");
    const auto plain_good = read_gltf_bytes(as_bytes(good), fs::path());
    const auto all_good = read_gltf_bytes_objects(as_bytes(good), fs::path());
    ASSERT_TRUE(plain_good.has_value());
    ASSERT_TRUE(all_good.has_value()) << all_good.error().context;
    ASSERT_EQ(all_good->size(), 1u);
    EXPECT_EQ((*all_good)[0].name, (*plain_good)[0].name);
    EXPECT_EQ((*all_good)[0].unit, MeshUnit::metre);
    ASSERT_TRUE((*all_good)[0].mesh.has_value());
    EXPECT_EQ((*(*all_good)[0].mesh)->content_hash(), (*plain_good)[0].mesh->content_hash());

    const std::string open = cube_glb(1.0f, "frame", 11);
    const auto plain_open = read_gltf_bytes(as_bytes(open), fs::path());
    ASSERT_FALSE(plain_open.has_value());
    const auto all_open = read_gltf_bytes_objects(as_bytes(open), fs::path());
    ASSERT_TRUE(all_open.has_value()) << all_open.error().context;
    ASSERT_EQ(all_open->size(), 1u);
    EXPECT_EQ((*all_open)[0].name, "frame");
    ASSERT_FALSE((*all_open)[0].mesh.has_value());
    EXPECT_EQ((*all_open)[0].mesh.error(), plain_open.error());
    EXPECT_FALSE((*all_good)[0].kind.has_value());
    ASSERT_TRUE((*all_open)[0].kind.has_value());
    EXPECT_EQ(*(*all_open)[0].kind, MeshRefusal::not_closed);
}

// A cube whose faces are all turned round: closed and consistent, but inside out.
[[nodiscard]] std::string inside_out_cube_obj(const char* name) {
    std::string t = std::string("o ") + name + "\n";
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::vec3 c = corner(v, 1.0f);
        t += "v " + std::to_string(c.x) + " " + std::to_string(c.y) + " " + std::to_string(c.z) + "\n";
    }
    for (const auto& tri : kCube) {
        t += "f " + std::to_string(tri[0] + 1) + " " + std::to_string(tri[2] + 1) + " " + std::to_string(tri[1] + 1) + "\n";
    }
    return t;
}

TEST(MeshImportObjects, EachObjectNamesItsOwnKindWhateverItIs) {
    Files f;
    const fs::path p = f.put("turned.obj", inside_out_cube_obj("turned"));
    const auto all = read_obj_objects(p, MeshUnit::metre);
    ASSERT_TRUE(all.has_value()) << all.error().context;
    ASSERT_EQ(all->size(), 1u);
    ASSERT_FALSE((*all)[0].mesh.has_value());
    ASSERT_TRUE((*all)[0].kind.has_value());
    EXPECT_EQ(*(*all)[0].kind, MeshRefusal::inside_out);
    // the plain reader still gives the bare Error, the same bytes
    const auto plain = read_obj(p, MeshUnit::metre);
    ASSERT_FALSE(plain.has_value());
    EXPECT_EQ((*all)[0].mesh.error(), plain.error());
}

TEST(MeshImportObjects, AProblemWithTheFileIsStillOneErrorTheSameAsThePlainReaders) {
    Files f;
    const std::string junk = "this is not a mesh file at all";
    {
        const fs::path p = f.put("junk.3mf", junk);
        const auto plain = read_3mf(p);
        ASSERT_FALSE(plain.has_value());
        const auto objects = read_3mf_objects(p);
        ASSERT_FALSE(objects.has_value());
        EXPECT_EQ(objects.error(), plain.error());
    }
    {
        const fs::path p = f.put("junk.stl", junk);
        const auto plain = read_stl(p, MeshUnit::metre);
        const auto objects = read_stl_objects(p, MeshUnit::metre);
        ASSERT_FALSE(plain.has_value());
        ASSERT_FALSE(objects.has_value());
        EXPECT_EQ(objects.error(), plain.error());
    }
    {
        const auto plain = read_obj(f.tmp.path() / "absent.obj", MeshUnit::metre);
        const auto objects = read_obj_objects(f.tmp.path() / "absent.obj", MeshUnit::metre);
        ASSERT_FALSE(plain.has_value());
        ASSERT_FALSE(objects.has_value());
        EXPECT_EQ(objects.error(), plain.error());
    }
    {
        const auto plain = read_gltf_bytes(as_bytes(junk), fs::path());
        const auto objects = read_gltf_bytes_objects(as_bytes(junk), fs::path());
        ASSERT_FALSE(plain.has_value());
        ASSERT_FALSE(objects.has_value());
        EXPECT_EQ(objects.error(), plain.error());
    }
    // a bad unit is the file-level problem the plain reader names
    const fs::path p = f.put("cube.stl", cube_stl(1.0f));
    const auto plain_unit = read_stl(p, MeshUnit::unspecified);
    const auto objects_unit = read_stl_objects(p, MeshUnit::unspecified);
    ASSERT_FALSE(plain_unit.has_value());
    ASSERT_FALSE(objects_unit.has_value());
    EXPECT_EQ(objects_unit.error(), plain_unit.error());
}

// ---- colour: the colour a file gives an object (sRGB 0..255), beside the mesh ------------------------------------------------------

[[nodiscard]] std::string material_factor(const char* rgba) {
    return std::string(R"({"pbrMetallicRoughness":{"baseColorFactor":[)") + rgba + "]}}";
}

[[nodiscard]] std::optional<SurfaceColour> glb_colour(const std::string& materials, int a, int b, std::size_t split) {
    const std::string glb = cube_glb_two_materials(materials, a, b, split);
    const auto all = read_gltf_bytes_objects(as_bytes(glb), fs::path());
    EXPECT_TRUE(all.has_value()) << (all.has_value() ? "" : all.error().context);
    if (!all.has_value() || all->size() != 1) {
        ADD_FAILURE() << "one object expected";
        return std::nullopt;
    }
    EXPECT_TRUE((*all)[0].mesh.has_value()) << "the colour never refuses a mesh";
    return (*all)[0].colour;
}

TEST(MeshImportColour, AGlbMaterialFactorIsLinearAndComesOutAsSrgbRoundedToTheNearestInteger) {
    // 0.1 -> 89.04 (rounds down), 0.5 -> 187.5 + 0.02 (rounds up), 0.2158605 is the linear value of sRGB 128
    const auto c = glb_colour("[" + material_factor("0.1,0.5,0.2158605,1.0") + "]", 0, 0, 6);
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->rgb, (std::array<uint8_t, 3>{89, 188, 128}));
    EXPECT_EQ(c->colours, 1u);
    const auto ends = glb_colour("[" + material_factor("0.0,1.0,0.0031308,1.0") + "]", 0, 0, 6);
    ASSERT_TRUE(ends.has_value());
    EXPECT_EQ(ends->rgb[0], 0);
    EXPECT_EQ(ends->rgb[1], 255);
    EXPECT_EQ(ends->rgb[2], 10);  // the linear segment's end: 12.92 * 0.0031308 * 255 = 10.31
}

TEST(MeshImportColour, AnObjectWithTwoMaterialsTakesTheLargerAreaAndCountsBothAndATieTakesTheLowestRgb) {
    const std::string two = "[" + material_factor("1,0,0,1") + "," + material_factor("0,0,1,1") + "]";
    const auto red_most = glb_colour(two, 0, 1, 8);
    ASSERT_TRUE(red_most.has_value());
    EXPECT_EQ(red_most->rgb, (std::array<uint8_t, 3>{255, 0, 0}));
    EXPECT_EQ(red_most->colours, 2u);
    const auto blue_most = glb_colour(two, 0, 1, 4);
    ASSERT_TRUE(blue_most.has_value());
    EXPECT_EQ(blue_most->rgb, (std::array<uint8_t, 3>{0, 0, 255}));
    EXPECT_EQ(blue_most->colours, 2u);
    const auto tie = glb_colour(two, 0, 1, 6);
    ASSERT_TRUE(tie.has_value());
    EXPECT_EQ(tie->rgb, (std::array<uint8_t, 3>{0, 0, 255})) << "a tie goes to the lowest (r, g, b)";
}

TEST(MeshImportColour, AMaterialWithNoFactorIsWhiteNoMaterialIsNoneAndAnUncolouredPrimitiveCountsForNothing) {
    const auto white = glb_colour("[{}]", 0, 0, 6);
    ASSERT_TRUE(white.has_value());
    EXPECT_EQ(white->rgb, (std::array<uint8_t, 3>{255, 255, 255}));
    EXPECT_FALSE(glb_colour("[{}]", -1, -1, 6).has_value());
    EXPECT_FALSE(glb_colour("", -1, -1, 6).has_value());
    const auto half = glb_colour("[" + material_factor("1,0,0,1") + "]", 0, -1, 4);
    ASSERT_TRUE(half.has_value());
    EXPECT_EQ(half->rgb, (std::array<uint8_t, 3>{255, 0, 0})) << "the uncoloured 8 triangles do not outweigh the coloured 4";
    EXPECT_EQ(half->colours, 1u);
}

TEST(MeshImportColour, ABadGlbMaterialGivesNoColourAndDoesNotRefuseTheMesh) {
    EXPECT_FALSE(glb_colour(R"([{"pbrMetallicRoughness":{"baseColorFactor":"red"}}])", 0, 0, 6).has_value());
    EXPECT_FALSE(glb_colour(R"([{"pbrMetallicRoughness":{"baseColorFactor":[1,0]}}])", 0, 0, 6).has_value());
    EXPECT_FALSE(glb_colour("[{}]", 5, 5, 6).has_value()) << "a material index past the array";
}

[[nodiscard]] std::string painted_cube_object(uint32_t id, std::string_view name, std::string_view object_attributes, const std::vector<int>& p1) {
    std::string s = "<object id=\"" + std::to_string(id) + "\" name=\"" + std::string(name) + "\" " + std::string(object_attributes) +
                    "><mesh><vertices>";
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::vec3 c = corner(v);
        s += "<vertex x=\"" + std::to_string(c.x) + "\" y=\"" + std::to_string(c.y) + "\" z=\"" + std::to_string(c.z) + "\"/>";
    }
    s += "</vertices><triangles>";
    for (std::size_t i = 0; i < 12; ++i) {
        s += "<triangle v1=\"" + std::to_string(kCube[i][0]) + "\" v2=\"" + std::to_string(kCube[i][1]) + "\" v3=\"" +
             std::to_string(kCube[i][2]) + "\"" + (p1.empty() ? "" : " p1=\"" + std::to_string(p1[i]) + "\"") + "/>";
    }
    return s + "</triangles></mesh></object>";
}

[[nodiscard]] std::optional<SurfaceColour> three_mf_colour(const std::string& resources, const std::string& root, const std::string& object) {
    Files f;
    const fs::path p = f.put("c.3mf", three_mf({object}, root, resources));
    const auto all = read_3mf_objects(p);
    EXPECT_TRUE(all.has_value()) << (all.has_value() ? "" : all.error().context);
    if (!all.has_value() || all->size() != 1) {
        ADD_FAILURE() << "one object expected";
        return std::nullopt;
    }
    EXPECT_TRUE((*all)[0].mesh.has_value());
    return (*all)[0].colour;
}

TEST(MeshImportColour, A3mfObjectTakesItsBasematerialsColourWithTheAlphaIgnored) {
    const std::string bases = R"(<basematerials id="10"><base name="a" displaycolor="#112233FF"/><base name="b" displaycolor="#FF8000"/></basematerials>)";
    const auto c = three_mf_colour(bases, R"(unit="millimeter")", painted_cube_object(1, "a", R"(pid="10" pindex="1")", {}));
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->rgb, (std::array<uint8_t, 3>{255, 128, 0}));
    EXPECT_EQ(c->colours, 1u);
    EXPECT_FALSE(three_mf_colour(bases, R"(unit="millimeter")", painted_cube_object(1, "a", "", {})).has_value());
}

TEST(MeshImportColour, A3mfObjectWithPerTriangleColoursTakesTheLargerAreaAndCountsBoth) {
    const std::string bases = R"(<basematerials id="10"><base name="a" displaycolor="#112233"/><base name="b" displaycolor="#445566"/></basematerials>)";
    const std::vector<int> mostly_a = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1};
    const auto c = three_mf_colour(bases, R"(unit="millimeter")", painted_cube_object(1, "a", R"(pid="10")", mostly_a));
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->rgb, (std::array<uint8_t, 3>{17, 34, 51}));
    EXPECT_EQ(c->colours, 2u);
    const std::vector<int> mostly_b = {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
    const auto d = three_mf_colour(bases, R"(unit="millimeter")", painted_cube_object(1, "a", R"(pid="10")", mostly_b));
    ASSERT_TRUE(d.has_value());
    EXPECT_EQ(d->rgb, (std::array<uint8_t, 3>{68, 85, 102}));
    EXPECT_EQ(d->colours, 2u);
}

TEST(MeshImportColour, A3mfColorgroupColourIsReadToo) {
    const std::string group = R"(<m:colorgroup id="11"><m:color color="#AABBCCDD"/></m:colorgroup>)";
    const auto c = three_mf_colour(group, R"(unit="millimeter" xmlns:m="http://schemas.microsoft.com/3dmanufacturing/material/2015/02")",
                                   painted_cube_object(1, "a", R"(pid="11" pindex="0")", {}));
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->rgb, (std::array<uint8_t, 3>{170, 187, 204}));
}

TEST(MeshImportColour, StlAndObjCarryNoColour) {
    Files f;
    const auto stl = read_stl_objects(f.put("cube.stl", cube_stl(1.0f)), MeshUnit::metre);
    ASSERT_TRUE(stl.has_value());
    ASSERT_EQ(stl->size(), 1u);
    EXPECT_TRUE((*stl)[0].mesh.has_value());
    EXPECT_FALSE((*stl)[0].colour.has_value());
    const auto obj = read_obj_objects(f.put("cube.obj", cube_obj()), MeshUnit::metre);
    ASSERT_TRUE(obj.has_value());
    ASSERT_EQ(obj->size(), 1u);
    EXPECT_TRUE((*obj)[0].mesh.has_value());
    EXPECT_FALSE((*obj)[0].colour.has_value());
}

}  // namespace
