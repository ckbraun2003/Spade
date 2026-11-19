// ---------------------------------------------------------------------------
// test_library_part_meshes.cpp -- part_meshes(set, id) (scene/library_query.hpp; L88l): a part entry's objects, read from the store.
//
//   LibraryPartMeshes.APartsObjectsComeBackInItsRecordsOrderRebuiltFromTheStore   two cubes (10 mm and 20 mm) listed visual-then-solid:
//                                                  the volumes are 1e-6 and 8e-6 m3, the order and the role are the record's
//   LibraryPartMeshes.APartWithOnlyShapesHasNoObjects      an empty list, with no store at all
//   LibraryPartMeshes.TheRefusalsAreNamed                  no store (store_file_missing, field store), a file the store lacks (the path
//                                                  names it), a file that is no sidecar (invalid_entry), an id the set does not have,
//                                                  a bare id
// Every test writes its own root and store under unique temp folders.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "scene/library_query.hpp"
#include "scene/library_set.hpp"
#include "unique_temp.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;

const std::string kShaA(64, 'a');
const std::string kShaB(64, 'b');

fs::path fresh(const std::string& name) {
    static std::vector<std::unique_ptr<spade::test::TempDir>> dirs;
    dirs.push_back(std::make_unique<spade::test::TempDir>("library_part_meshes_" + name));
    return dirs.back()->path();
}

void put(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << text;
}

// A cube of side `side` metres from the origin.
void save_cube(const fs::path& file, double side) {
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
    fs::create_directories(file.parent_path());
    auto saved = spade::save_mesh_file(file, **mesh);
    ASSERT_TRUE(saved.has_value()) << saved.error().context;
}

const std::string kTwoMeshes = std::string("id: \"x500_frame\"\nmeshes:\n  - {store: {sha256: \"") + kShaA +
                               "\", file: \"small.spmesh\"}, role: visual}\n  - {store: {sha256: \"" + kShaB + "\", file: \"big.spmesh\"}}\n";

// A v3 root `r` holding a frame part (`record`) and a motor, opened with `store` (empty: none).
sc::LibrarySet open_set(const std::string& name, const std::string& record, const fs::path& store) {
    const fs::path root = fresh(name) / "r";
    put(root / "library.yaml", "library_version: 3\nname: r\ndepends: []\nindexes:\n  - index.yaml\n");
    put(root / "index.yaml",
        "index_version: 1\nentries:\n"
        "  - id: part.frame.x500_frame\n    name: x500 frame\n    category: Parts\n    file: parts/x500_frame.frame.yaml\n"
        "  - id: part.motor.fpv\n    name: FPV motor\n    category: Parts\n    file: parts/fpv.motor.yaml\n");
    put(root / "parts/x500_frame.frame.yaml", record);
    put(root / "parts/fpv.motor.yaml", "part_version: 1\nmotor:\n  kv: 2000\n  mass: 0.032\n");
    const std::vector<sc::LibraryRootSpec> roots = {{"r", root, false}};
    auto set = sc::open_libraries(roots, store);
    EXPECT_TRUE(set.has_value()) << (set.has_value() ? "" : set.error().message);
    return set.has_value() ? std::move(*set) : sc::LibrarySet{};
}

TEST(LibraryPartMeshes, APartsObjectsComeBackInItsRecordsOrderRebuiltFromTheStore) {
    const fs::path store = fresh("store");
    save_cube(store / kShaA / "small.spmesh", 0.01);
    save_cube(store / kShaB / "big.spmesh", 0.02);
    const sc::LibrarySet set = open_set("ok", kTwoMeshes, store);
    const auto got = sc::part_meshes(set, "r:part.frame.x500_frame");
    ASSERT_TRUE(got.has_value()) << got.error().message;
    ASSERT_EQ(got->size(), 2u);
    EXPECT_EQ((*got)[0].file, "small.spmesh");
    EXPECT_EQ((*got)[0].sha256, kShaA);
    EXPECT_TRUE((*got)[0].visual);
    EXPECT_NEAR((*got)[0].mesh->mass_properties().volume, 1e-6, 1e-12);
    EXPECT_EQ((*got)[1].file, "big.spmesh");
    EXPECT_FALSE((*got)[1].visual);
    EXPECT_NEAR((*got)[1].mesh->mass_properties().volume, 8e-6, 1e-12);
}

TEST(LibraryPartMeshes, APartWithOnlyShapesHasNoObjects) {
    const sc::LibrarySet set =
        open_set("shapes", "id: \"x500_frame\"\nshapes:\n  - {kind: box, size: [0.133, 0.045, 0.0335]}\n", {});
    const auto got = sc::part_meshes(set, "r:part.frame.x500_frame");
    ASSERT_TRUE(got.has_value()) << got.error().message;
    EXPECT_TRUE(got->empty());
}

TEST(LibraryPartMeshes, TheRefusalsAreNamed) {
    // no store was given
    const sc::LibrarySet no_store = open_set("nostore", kTwoMeshes, {});
    const auto none = sc::part_meshes(no_store, "r:part.frame.x500_frame");
    ASSERT_FALSE(none.has_value());
    EXPECT_EQ(none.error().code, "store_file_missing");
    EXPECT_EQ(none.error().field, "store");
    EXPECT_EQ(none.error().id, "r:part.frame.x500_frame");

    // the store lacks the second file: the path names it
    const fs::path store = fresh("store2");
    save_cube(store / kShaA / "small.spmesh", 0.01);
    const sc::LibrarySet half = open_set("half", kTwoMeshes, store);
    const auto missing = sc::part_meshes(half, "r:part.frame.x500_frame");
    ASSERT_FALSE(missing.has_value());
    EXPECT_EQ(missing.error().code, "store_file_missing");
    EXPECT_NE(missing.error().path.find("big.spmesh"), std::string::npos) << missing.error().path;

    // a file that is no sidecar
    put(store / kShaB / "big.spmesh", "not a mesh");
    const auto bad = sc::part_meshes(half, "r:part.frame.x500_frame");
    ASSERT_FALSE(bad.has_value());
    EXPECT_EQ(bad.error().code, "invalid_entry");
    EXPECT_NE(bad.error().path.find("big.spmesh"), std::string::npos) << bad.error().path;

    // an id the set does not have, and one without its root
    const auto absent = sc::part_meshes(half, "r:part.frame.nope");
    ASSERT_FALSE(absent.has_value());
    EXPECT_EQ(absent.error().code, "entry_not_found");
    const auto bare = sc::part_meshes(half, "part.frame.x500_frame");
    ASSERT_FALSE(bare.has_value());
    EXPECT_EQ(bare.error().code, "bare_id");
}

// A drawing-only object may be a glTF (the kit's rotors are presentation glTFs, not closed solids: no TriangleMesh): it comes back with no mesh once the store has the file,
// for the renderer to load; a SOLID object must be an .spmesh, since physics and compile need its volume.
TEST(LibraryPartMeshes, ADrawingOnlyGltfIsAnObjectWithNoMeshAndASolidThatIsNotAnSpmeshIsRefused) {
    const fs::path store = fresh("gltf_store");
    put(store / kShaA / "rotor.gltf", "{}");  // present, and not read here
    const std::string drawn = std::string("id: \"x500_frame\"\nmeshes:\n  - {store: {sha256: \"") + kShaA + "\", file: \"rotor.gltf\"}, role: visual}\n";
    const auto got = sc::part_meshes(open_set("gltf", drawn, store), "r:part.frame.x500_frame");
    ASSERT_TRUE(got.has_value()) << got.error().message << ' ' << got.error().path;
    ASSERT_EQ(got->size(), 1u);
    EXPECT_EQ((*got)[0].file, "rotor.gltf");
    EXPECT_TRUE((*got)[0].visual);
    EXPECT_EQ((*got)[0].mesh, nullptr);
    // the file must still be in the store
    const auto absent = sc::part_meshes(open_set("gltf_absent", drawn, fresh("gltf_empty_store")), "r:part.frame.x500_frame");
    ASSERT_FALSE(absent.has_value());
    EXPECT_EQ(absent.error().code, "store_file_missing");
    // the same file as a solid is refused
    const std::string solid = std::string("id: \"x500_frame\"\nmeshes:\n  - {store: {sha256: \"") + kShaA + "\", file: \"rotor.gltf\"}}\n";
    const auto refused = sc::part_meshes(open_set("gltf_solid", solid, store), "r:part.frame.x500_frame");
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, "invalid_entry");
    EXPECT_NE(refused.error().path.find("rotor.gltf"), std::string::npos) << refused.error().path;
}

}  // namespace
