// tools/flow_movie/build_parts.hpp -- the real-parts drone of the web clips (the lead, 2026-11-18, L88l: the X500 frame plus a flight
// controller, GPS mast, battery, companion computer, camera and radio), PRESENTATION ONLY, drawn over the held body.
//
// Core measures the install points and writes a plain table (tools/flow_movie/data/x500_build/parts.yaml): one list `parts:`, each
// entry `name`, `kind`, `mass_kg` (Physics' business: read and kept, not drawn), `pose: {position: [x, y, z] m, orientation: [w, x, y, z]}`
// in the X500 body frame (y up, +X forward, the origin the model's centre of mass, the frame of KAT's x500.gltf), and ONE shape: `objects`
// (a list of `store_ref: {sha256, file}` with an optional `offset: {position, orientation}`; the file is <store>/<sha256>/<file>, a
// .spmesh or a .gltf/.glb), `box: [sx, sy, sz]` m, or `cylinder: {radius, length}` m along the part's local y, centred. These tests are
// against small hand-made tables until Core's lands, so only the source of the poses changes.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../tools/flow_movie/build_parts.hpp"
#include "render/mesh_resolver.hpp"
#include "render/scene.hpp"
#include "unique_temp.hpp"
#include "world/builder.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"

namespace {

namespace fs = std::filesystem;

const std::string kSha(64, 'a');

std::string table(const std::string& entry) { return "parts:\n" + entry; }

const std::string kBoxEntry =
    "  - name: battery\n    kind: battery\n    mass_kg: 0.4\n    pose: {position: [0.0, 0.05, 0.0], orientation: [1, 0, 0, 0]}\n"
    "    box: [0.10, 0.03, 0.04]\n";

fs::path fresh_store(const char* name) {
    const fs::path root = spade::test::unique_temp_dir(std::string("build_parts_") + name);  // TestHygiene: temp paths come from unique_temp.hpp
    fs::create_directories(root / kSha);
    return root;
}

void save_tetra(const fs::path& file) {
    spade::MeshInput m;
    m.unit = spade::MeshUnit::metre;
    m.positions = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    m.indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    auto mesh = spade::make_mesh(m, "fixture");
    ASSERT_TRUE(mesh.has_value()) << mesh.error().context;
    ASSERT_TRUE(spade::save_mesh_file(file, **mesh).has_value());
}

}  // namespace

TEST(FlowMovieBuildParts, ATableOfEveryShapeIsRead) {
    const std::string text = std::string("parts:\n") +
        "  - name: autopilot\n    kind: controller\n    mass_kg: 0.038\n"
        "    pose: {position: [0.01, 0.04, -0.02], orientation: [0.7071068, 0, 0.7071068, 0]}\n"
        "    objects:\n"
        "      - store_ref: {sha256: " + kSha + ", file: board.spmesh}\n"
        "      - store_ref: {sha256: " + kSha + ", file: lid.glb}\n"
        "        offset: {position: [0, 0.005, 0], orientation: [1, 0, 0, 0]}\n" +
        kBoxEntry +
        "  - name: gps_mast\n    kind: mast\n    mass_kg: 0.02\n    pose: {position: [0, 0.1, -0.1], orientation: [1, 0, 0, 0]}\n"
        "    cylinder: {radius: 0.006, length: 0.12}\n";
    const auto parts = flow_movie::parse_build_parts(text);
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    ASSERT_EQ(parts->size(), 3u);
    const flow_movie::BuildPart& pilot = (*parts)[0];
    EXPECT_EQ(pilot.name, "autopilot");
    EXPECT_EQ(pilot.kind, "controller");
    EXPECT_DOUBLE_EQ(pilot.mass_kg, 0.038);
    EXPECT_NEAR(pilot.position.x, 0.01f, 1e-6f);
    EXPECT_NEAR(pilot.position.z, -0.02f, 1e-6f);
    EXPECT_NEAR(pilot.orientation.w, 0.7071068f, 1e-5f);
    EXPECT_NEAR(pilot.orientation.y, 0.7071068f, 1e-5f);
    ASSERT_EQ(pilot.objects.size(), 2u);
    EXPECT_EQ(pilot.objects[0].sha256, kSha);
    EXPECT_EQ(pilot.objects[0].file, "board.spmesh");
    EXPECT_EQ(pilot.objects[0].offset_position, glm::vec3(0.0f)) << "no offset: the part's own pose";
    EXPECT_EQ(pilot.objects[1].file, "lid.glb");
    EXPECT_NEAR(pilot.objects[1].offset_position.y, 0.005f, 1e-7f);
    EXPECT_EQ(pilot.shape, flow_movie::BuildShape::objects);
    EXPECT_EQ((*parts)[1].shape, flow_movie::BuildShape::box);
    EXPECT_EQ((*parts)[1].box, glm::vec3(0.10f, 0.03f, 0.04f));
    EXPECT_EQ((*parts)[2].shape, flow_movie::BuildShape::cylinder);
    EXPECT_FLOAT_EQ((*parts)[2].radius, 0.006f);
    EXPECT_FLOAT_EQ((*parts)[2].length, 0.12f);
}

TEST(FlowMovieBuildParts, ATableThatCannotBeDrawnIsRefusedNamingWhy) {
    const std::string pose = "    pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}\n";
    const std::string head = "  - name: p\n    kind: battery\n    mass_kg: 0.1\n";
    struct Bad {
        std::string text;
        std::string says;
    };
    const std::vector<Bad> bad = {
        {"", "parts"},
        {"parts: []\n", "no parts"},
        {"nonsense: 1\n", "parts"},
        {table(head + pose), "shape"},                                                          // no shape
        {table(head + pose + "    box: [1, 1, 1]\n    cylinder: {radius: 1, length: 1}\n"), "one shape"},  // two shapes
        {table(head + "    box: [1, 1, 1]\n"), "pose"},
        {table("  - name: p\n    kind: toaster\n    mass_kg: 0.1\n" + pose + "    box: [1, 1, 1]\n"), "toaster"},  // unknown kind
        {table(head + pose + "    box: [1, 1, 1]\n    colour: red\n"), "colour"},                                    // unknown key
        {table(head + pose + "    box: [1, 0, 1]\n"), "box"},                                                       // a flat box
        {table(head + pose + "    box: [1, 1]\n"), "box"},
        {table(head + pose + "    cylinder: {radius: 0, length: 1}\n"), "cylinder"},
        {table(head + "    pose: {position: [0, 0], orientation: [1, 0, 0, 0]}\n    box: [1, 1, 1]\n"), "position"},
        {table(head + "    pose: {position: [0, 0, 0], orientation: [2, 0, 0, 0]}\n    box: [1, 1, 1]\n"), "unit"},  // not a unit quaternion
        {table(head + "    pose: {position: [0, .nan, 0], orientation: [1, 0, 0, 0]}\n    box: [1, 1, 1]\n"), "finite"},
        {table(head + pose + "    objects:\n      - store_ref: {sha256: abc, file: a.spmesh}\n"), "sha256"},
        {table(head + pose + "    objects:\n      - store_ref: {sha256: " + kSha + ", file: ../a.spmesh}\n"), "file"},
        {table(head + pose + "    objects:\n      - store_ref: {sha256: " + kSha + ", file: a.stl}\n"), "extension"},
        {table(head + pose + "    objects: []\n"), "objects"},
        {table(head + pose + "    box: [1, 1, 1]\n" + head + pose + "    box: [1, 1, 1]\n"), "twice"},  // a name used twice
    };
    for (const Bad& b : bad) {
        const auto r = flow_movie::parse_build_parts(b.text);
        ASSERT_FALSE(r.has_value()) << b.text;
        EXPECT_NE(r.error().context.find("build parts"), std::string::npos) << r.error().context;
        EXPECT_NE(r.error().context.find(b.says), std::string::npos) << "wanted '" << b.says << "' in: " << r.error().context;
    }
}

TEST(FlowMovieBuildParts, EveryKindHasItsOwnColour) {
    std::vector<glm::vec4> seen;
    for (const char* kind : {"frame", "controller", "gps", "battery", "computer", "camera", "radio", "mast"}) {
        const auto m = flow_movie::kind_material(kind);
        ASSERT_TRUE(m.has_value()) << kind;
        EXPECT_EQ(std::count(seen.begin(), seen.end(), m->base_color), 0) << kind << " shares a colour";
        seen.push_back(m->base_color);
    }
    EXPECT_FALSE(flow_movie::kind_material("toaster").has_value());
}

TEST(FlowMovieBuildParts, ABoxAndACylinderAreClosedFlatShadedMeshesAboutTheirCentre) {
    const spade::render::MeshData box = flow_movie::box_mesh(glm::vec3(0.10f, 0.03f, 0.04f));
    EXPECT_EQ(box.indices.size(), 36u) << "12 triangles";
    EXPECT_EQ(box.normals.size(), box.positions.size());
    glm::vec3 lo(1e9f), hi(-1e9f);
    for (const glm::vec3& p : box.positions) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
    }
    EXPECT_NEAR(lo.x, -0.05f, 1e-6f);
    EXPECT_NEAR(hi.x, 0.05f, 1e-6f);
    EXPECT_NEAR(lo.y, -0.015f, 1e-6f);
    EXPECT_NEAR(hi.z, 0.02f, 1e-6f);
    for (const glm::vec3& n : box.normals) EXPECT_NEAR(glm::length(n), 1.0f, 1e-5f);

    const spade::render::MeshData cyl = flow_movie::cylinder_mesh(0.006f, 0.12f);
    ASSERT_FALSE(cyl.indices.empty());
    EXPECT_EQ(cyl.normals.size(), cyl.positions.size());
    lo = glm::vec3(1e9f);
    hi = glm::vec3(-1e9f);
    for (const glm::vec3& p : cyl.positions) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
        EXPECT_LE(std::hypot(p.x, p.z), 0.006f + 1e-6f);
    }
    EXPECT_NEAR(lo.y, -0.06f, 1e-6f) << "along the part's local y, centred";
    EXPECT_NEAR(hi.y, 0.06f, 1e-6f);
    EXPECT_NEAR(hi.x, 0.006f, 1e-4f);
    for (const glm::vec3& n : cyl.normals) EXPECT_NEAR(glm::length(n), 1.0f, 1e-4f);
}

TEST(FlowMovieBuildParts, APartIsDrawnAtTheBodysPoseTimesItsOwnAndItsKindsColour) {
    const auto parts = flow_movie::parse_build_parts(table(kBoxEntry));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    spade::render::RenderScene scene;
    scene.materials.push_back(spade::render::Material{});  // something already in the palette
    const glm::vec3 body(1.0f, 2.0f, 3.0f);
    const glm::quat turn = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const auto added = flow_movie::add_build_parts(scene, *parts, fs::path("unused"), body, turn);
    ASSERT_TRUE(added.has_value()) << added.error().context;
    ASSERT_EQ(scene.statics.size(), 1u);
    const glm::vec4 centre = scene.statics.front().local_to_world * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    EXPECT_NEAR(centre.x, 1.0f, 1e-5f) << "the part sits 0.05 up in the body's frame; a turn about y leaves up where it is";
    EXPECT_NEAR(centre.y, 2.05f, 1e-5f);
    EXPECT_NEAR(centre.z, 3.0f, 1e-5f);
    const glm::vec4 x = scene.statics.front().local_to_world * glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    EXPECT_NEAR(x.z, 2.0f, 1e-5f) << "the body's quarter turn takes the part's +x to -z";
    EXPECT_FALSE(scene.statics.front().flow_wall) << "presentation only";
    ASSERT_EQ(scene.materials.size(), 2u);
    EXPECT_EQ(scene.materials.back().base_color, flow_movie::kind_material("battery")->base_color);
    for (const uint32_t m : scene.meshes.back().submesh_material) EXPECT_EQ(m, 1u);
}

TEST(FlowMovieBuildParts, AStoreObjectIsReadFromItsHashFolderAndDrawnAtThePartsPoseTimesItsOffset) {
    const fs::path store = fresh_store("object");
    save_tetra(store / kSha / "board.spmesh");
    const auto parts = flow_movie::parse_build_parts(table(
        "  - name: autopilot\n    kind: controller\n    mass_kg: 0.038\n"
        "    pose: {position: [0.1, 0.0, 0.0], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: board.spmesh}\n"
        "        offset: {position: [0, 0.02, 0], orientation: [1, 0, 0, 0]}\n"));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    spade::render::RenderScene scene;
    const auto added = flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0));
    ASSERT_TRUE(added.has_value()) << added.error().context;
    ASSERT_EQ(scene.statics.size(), 1u);
    ASSERT_EQ(scene.meshes.size(), 1u);
    EXPECT_FALSE(scene.meshes.front().positions.empty());
    const glm::vec4 origin = scene.statics.front().local_to_world * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    EXPECT_NEAR(origin.x, 0.1f, 1e-6f);
    EXPECT_NEAR(origin.y, 0.02f, 1e-6f) << "the part's pose, then the object's offset in the part's frame";
    EXPECT_EQ(scene.materials.back().base_color, flow_movie::kind_material("controller")->base_color) << ".spmesh: coloured by kind";
    fs::remove_all(store);
}

TEST(FlowMovieBuildParts, APartWhoseObjectIsMissingIsRefusedNamingThePathAndTheSceneIsLeftAsItWas) {
    const fs::path store = fresh_store("missing");
    save_tetra(store / kSha / "board.spmesh");
    const auto parts = flow_movie::parse_build_parts(table(
        kBoxEntry +
        "  - name: autopilot\n    kind: controller\n    mass_kg: 0.038\n"
        "    pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: board.spmesh}\n"
        "      - store_ref: {sha256: " + kSha + ", file: gone.spmesh}\n"));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    spade::render::RenderScene scene;
    scene.materials.push_back(spade::render::Material{});
    const auto added = flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0));
    ASSERT_FALSE(added.has_value());
    EXPECT_NE(added.error().context.find("gone.spmesh"), std::string::npos) << added.error().context;
    EXPECT_NE(added.error().context.find("autopilot"), std::string::npos) << "the part is named: " << added.error().context;
    EXPECT_TRUE(scene.statics.empty()) << "all or nothing: the box that came first is not left behind";
    EXPECT_TRUE(scene.meshes.empty());
    EXPECT_EQ(scene.materials.size(), 1u);
    fs::remove_all(store);
}
