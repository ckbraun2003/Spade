// render/mesh_resolver.hpp and world/mesh_ref.hpp (N11): a world's visual
// meshes found on disk, never silently missing. The ref rule's cases are the
// lead's and Core's (2026-10-20); each sidecar is saved into a fresh temp
// folder by Core's own save_mesh_file().

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "render/mesh_resolver.hpp"
#include "render/scene.hpp"
#include "unique_temp.hpp"
#include "world/builder.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"
#include "world/mesh_ref.hpp"

namespace {

namespace fs = std::filesystem;
using spade::MeshInput;
using spade::render::NamedMesh;
using spade::render::ResolvedMeshes;
using spade::test::TempDir;

// A cube from `lo` with side `side` (test_mesh_file.cpp's fixture): 12 triangles.
[[nodiscard]] MeshInput cube(double side) {
    static constexpr uint32_t kTriangles[12][3] = {
        {0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
        {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5},
    };
    MeshInput m;
    m.unit = spade::MeshUnit::metre;
    for (uint32_t v = 0; v < 8; ++v) {
        m.positions.push_back(side * glm::dvec3{static_cast<double>(v & 1u), static_cast<double>((v >> 1) & 1u),
                                                static_cast<double>((v >> 2) & 1u)});
    }
    for (const auto& t : kTriangles) m.indices.insert(m.indices.end(), {t[0], t[1], t[2]});
    return m;
}

// A tetrahedron: 4 triangles.
[[nodiscard]] MeshInput tetra() {
    MeshInput m;
    m.unit = spade::MeshUnit::metre;
    m.positions = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    m.indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    return m;
}

void save(const fs::path& file, const MeshInput& input) {
    fs::create_directories(file.parent_path());
    auto mesh = spade::make_mesh(input, "fixture");
    ASSERT_TRUE(mesh.has_value()) << mesh.error().context;
    ASSERT_TRUE(spade::save_mesh_file(file, **mesh).has_value());
}

// A world with one prop per ref, in order.
[[nodiscard]] spade::WorldDesc world_with_props(const std::vector<std::string>& refs) {
    spade::WorldBuilder b;
    b.name("props").capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    b.material(spade::MaterialDesc{.name = "default"});
    for (const std::string& ref : refs) b.prop(ref, spade::SdfPose{}, 0);
    auto world = b.build();
    EXPECT_TRUE(world.has_value()) << world.error().context;
    return world ? *world : spade::WorldDesc{};
}

[[nodiscard]] std::size_t triangles(const NamedMesh& named) { return named.mesh.indices.size() / 3u; }

TEST(MeshRef, ARelativePathOfPlainSegmentsIsValid) {
    // "console" and "com10" are not device names, only like them.
    for (const char* ref : {"ball", "props/ball", "gates/race/arch_2", "a-b_c/D9", "console", "props/com10"}) {
        EXPECT_TRUE(spade::valid_mesh_ref(ref)) << ref << ": " << spade::mesh_ref_problem(ref);
    }
}

TEST(MeshRef, AnythingThatCouldLeaveItsRootIsRefusedByName) {
    for (const char* ref : {"", "../x", "a/../b", "./x", "a//b", "/x", "x/", "C:x", "mesh:props/tower", "a\\b",
                            "ball.spmesh", "a\tb"}) {
        EXPECT_FALSE(spade::valid_mesh_ref(ref)) << "\"" << ref << "\" must be refused";
        EXPECT_FALSE(spade::mesh_ref_problem(ref).empty()) << ref;
    }
}

// Core's portability rule: a mesh_ref is a file name in a library used on
// Windows and Linux alike, so each segment is ASCII [A-Za-z0-9_-] and never
// a name Windows reserves for a device (the lead's), in any case.
TEST(MeshRef, AnythingNotAPortableFileNameIsRefusedByName) {
    for (const char* ref : {"a b", "a*b", "props/b\xC3\xA4llchen", "a|b", "a?b", "a<b", "nul", "props/con", "COM1",
                            "lpt9/x", "Aux", "prn"}) {
        EXPECT_FALSE(spade::valid_mesh_ref(ref)) << "\"" << ref << "\" must be refused";
        EXPECT_FALSE(spade::mesh_ref_problem(ref).empty()) << ref;
    }
}

TEST(MeshResolver, ASidecarResolvesFromItsRootIntoFlatGeometry) {
    const TempDir dir("mesh_resolver");
    save(dir.path() / "props" / "ball.spmesh", cube(0.5));
    const spade::WorldDesc world = world_with_props({"props/ball"});
    const fs::path roots[] = {dir.path()};
    const ResolvedMeshes r = spade::render::resolve_meshes(world, roots);
    ASSERT_EQ(r.meshes.size(), 1u);
    EXPECT_EQ(r.meshes[0].ref, "props/ball");
    EXPECT_FALSE(r.meshes[0].stand_in);
    EXPECT_EQ(triangles(r.meshes[0]), 12u);
    EXPECT_EQ(r.meshes[0].mesh.positions.size(), 36u) << "flat: three vertices a triangle";
    EXPECT_EQ(r.meshes[0].mesh.normals.size(), 36u);
    EXPECT_TRUE(r.meshes[0].mesh.submesh_first_index.empty()) << "one submesh, drawn in the prop's material";
    EXPECT_TRUE(r.unresolved.empty());
    EXPECT_TRUE(r.from_fallback.empty());
}

// The lead's condition: the first root wins, and a ref found only in a later
// one -- the library, for the editor -- is listed, for the Save to copy in.
TEST(MeshResolver, TheFirstRootWinsAndALibraryOnlyRefIsReported) {
    const TempDir scene("mesh_resolver_scene"), library("mesh_resolver_library");
    save(scene.path() / "a.spmesh", cube(1.0));
    save(library.path() / "a.spmesh", tetra());
    save(library.path() / "props" / "b.spmesh", tetra());
    const spade::WorldDesc world = world_with_props({"a", "props/b", "a"});
    const fs::path roots[] = {scene.path(), library.path()};
    const ResolvedMeshes r = spade::render::resolve_meshes(world, roots);
    ASSERT_EQ(r.meshes.size(), 2u) << "each distinct ref once";
    EXPECT_EQ(r.meshes[0].ref, "a");
    EXPECT_EQ(triangles(r.meshes[0]), 12u) << "the scene's own cube, not the library's tetrahedron";
    EXPECT_EQ(r.meshes[1].ref, "props/b");
    EXPECT_EQ(triangles(r.meshes[1]), 4u);
    ASSERT_EQ(r.from_fallback.size(), 1u);
    EXPECT_EQ(r.from_fallback[0].ref, "props/b");
    EXPECT_EQ(r.from_fallback[0].root, 1u);
    EXPECT_EQ(r.from_fallback[0].file, library.path() / "props" / "b.spmesh");
    EXPECT_TRUE(r.unresolved.empty());
}

TEST(MeshResolver, AMissingRefStandsInAndIsNamedWithWhereItLooked) {
    const TempDir scene("mesh_resolver_scene"), library("mesh_resolver_library");
    const spade::WorldDesc world = world_with_props({"nowhere"});
    const fs::path roots[] = {scene.path(), library.path()};
    const ResolvedMeshes r = spade::render::resolve_meshes(world, roots);
    ASSERT_EQ(r.meshes.size(), 1u);
    EXPECT_TRUE(r.meshes[0].stand_in);
    EXPECT_EQ(r.meshes[0].ref, "nowhere");
    EXPECT_GT(triangles(r.meshes[0]), 0u) << "a visible stand-in, never nothing";
    ASSERT_EQ(r.unresolved.size(), 1u);
    EXPECT_EQ(r.unresolved[0].ref, "nowhere");
    EXPECT_NE(r.unresolved[0].why.find("nowhere.spmesh"), std::string::npos) << r.unresolved[0].why;
    EXPECT_NE(r.unresolved[0].why.find(library.path().filename().string()), std::string::npos) << r.unresolved[0].why;
}

TEST(MeshResolver, ACorruptSidecarIsNamedWithTheReadersRefusal) {
    const TempDir dir("mesh_resolver");
    std::ofstream(dir.path() / "bad.spmesh", std::ios::binary) << "not a mesh";
    const spade::WorldDesc world = world_with_props({"bad"});
    const fs::path roots[] = {dir.path()};
    const ResolvedMeshes r = spade::render::resolve_meshes(world, roots);
    ASSERT_EQ(r.unresolved.size(), 1u);
    EXPECT_NE(r.unresolved[0].why.find("mesh file"), std::string::npos) << r.unresolved[0].why;
    ASSERT_EQ(r.meshes.size(), 1u);
    EXPECT_TRUE(r.meshes[0].stand_in);
}

// A ref that could leave its root is never looked up: a sidecar where
// "../escape" would lead is not read.
TEST(MeshResolver, APathLikeRefIsNeverLookedUp) {
    const TempDir outer("mesh_resolver_outer");
    save(outer.path() / "escape.spmesh", cube(1.0));
    fs::create_directories(outer.path() / "root");
    const spade::WorldDesc world = world_with_props({"../escape"});
    const fs::path roots[] = {outer.path() / "root"};
    const ResolvedMeshes r = spade::render::resolve_meshes(world, roots);
    ASSERT_EQ(r.unresolved.size(), 1u);
    EXPECT_EQ(r.unresolved[0].why.find("escape.spmesh"), std::string::npos) << "never looked up: " << r.unresolved[0].why;
    EXPECT_NE(r.unresolved[0].why.find(std::string(spade::mesh_ref_problem("../escape"))), std::string::npos)
        << r.unresolved[0].why;
    ASSERT_EQ(r.meshes.size(), 1u);
    EXPECT_TRUE(r.meshes[0].stand_in);
}

// scene_from_world() never draws a prop blank: an unresolved one is refused,
// naming it; a stand-in draws in its own material, not the prop's.
TEST(MeshResolver, SceneFromWorldRefusesAnUnresolvedPropAndDrawsAStandInInItsOwnMaterial) {
    const spade::WorldDesc world = world_with_props({"props/ball"});
    auto refused = spade::render::scene_from_world(world, {});
    ASSERT_FALSE(refused.has_value()) << "an unresolved prop must be refused, never drawn blank";
    EXPECT_NE(refused.error().context.find("props/ball"), std::string::npos) << refused.error().context;

    const TempDir dir("mesh_resolver");
    const fs::path roots[] = {dir.path()};
    const ResolvedMeshes r = spade::render::resolve_meshes(world, roots);
    ASSERT_EQ(r.meshes.size(), 1u);
    auto scene = spade::render::scene_from_world(world, r.meshes);
    ASSERT_TRUE(scene.has_value()) << scene.error().context;
    ASSERT_EQ(scene->statics.size(), 1u);
    const auto& item = scene->statics[0];
    EXPECT_EQ(item.mesh_index, 0u);
    EXPECT_EQ(item.material_override, spade::render::kNoMaterial) << "the stand-in keeps its own colour";
    const auto& mesh = scene->meshes[item.mesh_index];
    ASSERT_FALSE(mesh.submesh_material.empty());
    const auto& material = scene->materials[mesh.submesh_material[0]];
    EXPECT_GT(material.base_color.r, 0.9f);
    EXPECT_LT(material.base_color.g, 0.1f);
    EXPECT_GT(material.base_color.b, 0.9f);
    EXPECT_EQ(material.shading, 1u) << "unlit, so no light makes it pass for a real mesh";
}

}  // namespace
