#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
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

}  // namespace
