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
#include <array>
#include <filesystem>
#include <cstring>
#include <fstream>
#include <vector>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../tools/flow_movie/build_parts.hpp"
#include "render/mesh_resolver.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
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

// REAL MATERIALS (the lead, 2026-11-18: role colours, a red arm, a green Pi and a blue controller, "read as a toy"; Cameron asked for
// realistic): a small set of NAMED materials (carbon fibre near-black, bare and anodised aluminium, PCB green, matte black and white
// plastic, black rubber, gold), set on a part (`material:`) or on one of its objects (`objects: - material:`, which wins), and keyed on
// the glTF's own material NAMES for the frame and the rotors (carbon_plates, arms, metal, stator, bell, prop ...). The renderer has the
// lambert, unlit and emissive models only: so these are colours, not a sheen (a specular term is an engine change, sized to the lead).
TEST(FlowMovieBuildParts, TheRealMaterialsAreNamedAndLookLikeTheirThings) {
    for (const char* name : {"carbon_fibre", "aluminium_bare", "aluminium_anodised", "pcb_green", "plastic_black", "plastic_white", "rubber_black", "gold"}) {
        EXPECT_TRUE(flow_movie::real_material(name).has_value()) << name;
    }
    EXPECT_FALSE(flow_movie::real_material("chrome").has_value());
    const auto rgb = [](const char* name) { return glm::vec3(flow_movie::real_material(name)->base_color); };
    const glm::vec3 carbon = rgb("carbon_fibre");
    EXPECT_LT(std::max({carbon.x, carbon.y, carbon.z}), 0.07f) << "carbon fibre is near black";
    const glm::vec3 bare = rgb("aluminium_bare");
    EXPECT_GT(std::min({bare.x, bare.y, bare.z}), 0.5f) << "bare aluminium is light";
    EXPECT_LT(std::max({bare.x, bare.y, bare.z}) - std::min({bare.x, bare.y, bare.z}), 0.1f) << "and nearly grey";
    const glm::vec3 anodised = rgb("aluminium_anodised");
    EXPECT_LT(anodised.x + anodised.y + anodised.z, bare.x + bare.y + bare.z) << "anodised is darker than bare";
    const glm::vec3 pcb = rgb("pcb_green");
    EXPECT_GT(pcb.y, pcb.x);
    EXPECT_GT(pcb.y, pcb.z);
    EXPECT_LT(pcb.y, 0.5f) << "a dark PCB green, not a toy green";
    EXPECT_GT(std::min({rgb("plastic_white").x, rgb("plastic_white").y, rgb("plastic_white").z}), 0.8f);
    const glm::vec3 gold = rgb("gold");
    EXPECT_GT(gold.x, gold.y);
    EXPECT_GT(gold.y, gold.z);
    std::vector<glm::vec4> seen;
    for (const char* name : {"carbon_fibre", "aluminium_bare", "aluminium_anodised", "pcb_green", "plastic_black", "plastic_white", "rubber_black", "gold"}) {
        const glm::vec4 c = flow_movie::real_material(name)->base_color;
        EXPECT_EQ(std::count(seen.begin(), seen.end(), c), 0) << name << " shares a colour";
        seen.push_back(c);
        EXPECT_EQ(flow_movie::real_material(name)->shading, 0u) << "lambert";
    }
}

TEST(FlowMovieBuildParts, AGltfsMaterialNamesMapToRealMaterialsByTheirLastWord) {
    EXPECT_EQ(flow_movie::real_material_for_gltf("drone/x500-body/carbon_plates"), "carbon_fibre");
    EXPECT_EQ(flow_movie::real_material_for_gltf("drone/x500-body/arms"), "carbon_fibre");
    EXPECT_EQ(flow_movie::real_material_for_gltf("drone/x500-body/metal"), "aluminium_bare");
    EXPECT_EQ(flow_movie::real_material_for_gltf("drone/x500-body/stator"), "aluminium_anodised");
    EXPECT_EQ(flow_movie::real_material_for_gltf("drone/x500-rotor-front-cw/bell"), "aluminium_anodised");
    EXPECT_EQ(flow_movie::real_material_for_gltf("drone/x500-rotor-front-cw/prop"), "plastic_black");
    EXPECT_EQ(flow_movie::real_material_for_gltf("drone/x500-body/landing_rubber"), "rubber_black");
    EXPECT_EQ(flow_movie::real_material_for_gltf("drone/x500-body/autopilot"), "plastic_black");
    EXPECT_EQ(flow_movie::real_material_for_gltf("arms"), "carbon_fibre") << "no folders: the name itself";
    EXPECT_FALSE(flow_movie::real_material_for_gltf("drone/x500-body/landing_foam").has_value()) << "not mapped: the file's own colour stays";
    EXPECT_FALSE(flow_movie::real_material_for_gltf("").has_value());
}

TEST(FlowMovieBuildParts, AGltfsMaterialNamesAreReadFromItsJsonInOrder) {
    const spade::test::TempDir dir("gltf_names");
    const fs::path file = dir.path() / "m.gltf";
    {
        std::ofstream out(file, std::ios::binary);
        out << R"({"asset":{"version":"2.0"},"materials":[{"name":"drone/x500-body/arms"},{"pbrMetallicRoughness":{}},{"name":"b"}]})";
    }
    const auto names = flow_movie::gltf_material_names(file);
    ASSERT_TRUE(names.has_value()) << names.error().context;
    EXPECT_EQ(*names, (std::vector<std::string>{"drone/x500-body/arms", "", "b"})) << "an unnamed material is an empty name, in place";
    {
        std::ofstream out(dir.path() / "none.gltf", std::ios::binary);
        out << R"({"asset":{"version":"2.0"}})";
    }
    const auto none = flow_movie::gltf_material_names(dir.path() / "none.gltf");
    ASSERT_TRUE(none.has_value());
    EXPECT_TRUE(none->empty());
    EXPECT_FALSE(flow_movie::gltf_material_names(dir.path() / "missing.gltf").has_value());
}

TEST(FlowMovieBuildParts, AMaterialOnAPartOrAnObjectIsReadAndAnUnknownOneIsRefused) {
    const std::string pose = "    pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}\n";
    const auto parts = flow_movie::parse_build_parts(table(
        "  - name: pi\n    kind: computer\n    mass_kg: 0.04\n    material: pcb_green\n" + pose +
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: board.spmesh}\n"
        "      - store_ref: {sha256: " + kSha + ", file: usb.spmesh}\n        material: aluminium_bare\n"
        "  - name: battery\n    kind: battery\n    mass_kg: 0.4\n" + pose + "    box: [0.1, 0.03, 0.04]\n"));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    EXPECT_EQ((*parts)[0].material, "pcb_green");
    EXPECT_EQ((*parts)[0].objects[0].material, "");
    EXPECT_EQ((*parts)[0].objects[1].material, "aluminium_bare");
    EXPECT_EQ((*parts)[1].material, "") << "no material: the kind's colour";
    for (const std::string& text : {
             table("  - name: p\n    kind: battery\n    mass_kg: 0.1\n    material: chrome\n" + pose + "    box: [1, 1, 1]\n"),
             table("  - name: p\n    kind: battery\n    mass_kg: 0.1\n" + pose + "    objects:\n      - store_ref: {sha256: " + kSha +
                   ", file: a.spmesh}\n        material: chrome\n")}) {
        const auto r = flow_movie::parse_build_parts(text);
        ASSERT_FALSE(r.has_value()) << text;
        EXPECT_NE(r.error().context.find("chrome"), std::string::npos) << r.error().context;
    }
}

TEST(FlowMovieBuildParts, APartsOwnMaterialReplacesItsKindsColourAndAnObjectsOwnWinsOverTheParts) {
    const fs::path store = fresh_store("material");
    save_tetra(store / kSha / "board.spmesh");
    save_tetra(store / kSha / "usb.spmesh");
    const auto parts = flow_movie::parse_build_parts(table(
        "  - name: pi\n    kind: computer\n    mass_kg: 0.04\n    material: pcb_green\n"
        "    pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: board.spmesh}\n"
        "      - store_ref: {sha256: " + kSha + ", file: usb.spmesh}\n        material: aluminium_bare\n" + kBoxEntry));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    spade::render::RenderScene scene;
    const auto added = flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0));
    ASSERT_TRUE(added.has_value()) << added.error().context;
    ASSERT_EQ(scene.materials.size(), 3u);
    EXPECT_EQ(scene.materials[0].base_color, flow_movie::real_material("pcb_green")->base_color) << "the part's material, not the computer kind's";
    EXPECT_EQ(scene.materials[1].base_color, flow_movie::real_material("aluminium_bare")->base_color) << "the object's own";
    EXPECT_EQ(scene.materials[2].base_color, flow_movie::kind_material("battery")->base_color) << "no material: the kind's colour";
    fs::remove_all(store);
}

// THE PROPELLERS (the lead, 2026-11-18): at 3282 rpm a real camera sees a blur, not a frozen blade. A part of kind `rotor` (`rotor: i`,
// the index of the omega in the logged body's rotor speeds, and `spin: 1 or -1`) is drawn as its blade SPREAD over the frame's shutter:
// `samples` copies at azimuths spread (centred on the blade's own pose) over omega x shutter radians, in the spin's direction, each
// see-through at a low alpha, so the blade's shape reads inside a soft disc. Without a speed it is one frozen opaque blade.
TEST(FlowMovieBuildParts, ABladeBlurIsSamplesAzimuthsSpreadSymmetricallyOverTheShutterInTheSpinsDirection) {
    const double omega = 343.667;
    const double shutter = 1.0 / 60.0;
    const auto up = flow_movie::prop_blur_angles(omega, +1, shutter, 24u);
    const auto down = flow_movie::prop_blur_angles(omega, -1, shutter, 24u);
    ASSERT_EQ(up.size(), 24u);
    ASSERT_EQ(down.size(), 24u);
    double sum = 0.0;
    for (std::size_t i = 0; i < up.size(); ++i) {
        sum += up[i];
        EXPECT_NEAR(up[i], -down[i], 1e-6) << "the opposite spin mirrors it";
        if (i > 0) {
            EXPECT_GT(up[i], up[i - 1]) << "a +1 spin advances";
        }
    }
    EXPECT_NEAR(sum, 0.0, 1e-4) << "centred on the blade's own pose";
    const double step = omega * shutter / 24.0;
    EXPECT_NEAR(up[1] - up[0], step, 1e-6) << "the sweep divided by the samples";
    EXPECT_NEAR(up.back() - up.front(), omega * shutter * 23.0 / 24.0, 1e-6);
    for (const double a : flow_movie::prop_blur_angles(0.0, +1, shutter, 24u)) EXPECT_EQ(a, 0.0) << "not turning: no spread";
    EXPECT_EQ(flow_movie::prop_blur_angles(omega, +1, shutter, 1u).size(), 1u);
    EXPECT_TRUE(flow_movie::prop_blur_angles(omega, +1, shutter, 0u).empty());
}

TEST(FlowMovieBuildParts, ARotorTableNeedsItsIndexAndSpinAndNoOtherPartMayHaveThem) {
    const std::string pose = "    pose: {position: [0.174, 0.0614, 0.174], orientation: [1, 0, 0, 0]}\n";
    const std::string obj = "    objects:\n      - store_ref: {sha256: " + kSha + ", file: prop.glb}\n";
    const auto good = flow_movie::parse_build_parts(table("  - name: rotor_fr\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 2\n    spin: -1\n" + pose + obj));
    ASSERT_TRUE(good.has_value()) << good.error().context;
    EXPECT_EQ((*good)[0].kind, "rotor");
    EXPECT_EQ((*good)[0].rotor, 2);
    EXPECT_EQ((*good)[0].spin, -1);
    for (const auto& [text, says] : std::vector<std::pair<std::string, std::string>>{
             {table("  - name: r\n    kind: rotor\n    mass_kg: 0.02\n    spin: 1\n" + pose + obj), "rotor"},
             {table("  - name: r\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n" + pose + obj), "spin"},
             {table("  - name: r\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 2\n" + pose + obj), "spin"},
             {table("  - name: r\n    kind: rotor\n    mass_kg: 0.02\n    rotor: -1\n    spin: 1\n" + pose + obj), "rotor"},
             {table("  - name: r\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n" + pose + "    box: [1, 1, 1]\n"), "objects"},
             {table("  - name: b\n    kind: battery\n    mass_kg: 0.1\n    rotor: 0\n    spin: 1\n" + pose + "    box: [1, 1, 1]\n"), "rotor"}}) {
        const auto r = flow_movie::parse_build_parts(text);
        ASSERT_FALSE(r.has_value()) << text;
        EXPECT_NE(r.error().context.find(says), std::string::npos) << "wanted '" << says << "' in: " << r.error().context;
    }
}

TEST(FlowMovieBuildParts, ASpinningRotorIsManySeeThroughCopiesOfOneMeshAndAStoppedOneIsOneOpaqueBlade) {
    const fs::path store = fresh_store("rotor");
    save_tetra(store / kSha / "prop.spmesh");
    const auto parts = flow_movie::parse_build_parts(table(
        "  - name: rotor_fl\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 1\n    spin: 1\n"
        "    pose: {position: [0.174, 0.0614, 0.174], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: prop.spmesh}\n"));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    flow_movie::BuildDraw draw;
    draw.blur.samples = 8u;
    draw.blur.alpha = 0.2f;
    const std::array<double, 4> omega{0.0, 343.667, 343.667, 343.667};
    draw.omega = omega;
    spade::render::RenderScene scene;
    const auto added = flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0), draw);
    ASSERT_TRUE(added.has_value()) << added.error().context;
    ASSERT_EQ(scene.statics.size(), 8u);
    ASSERT_EQ(scene.meshes.size(), 1u) << "one mesh, many items: not eight copies of the blade's triangles";
    const auto angles = flow_movie::prop_blur_angles(343.667, +1, draw.blur.shutter_s, 8u);
    for (std::size_t i = 0; i < 8u; ++i) {
        EXPECT_TRUE(scene.statics[i].see_through) << i;
        EXPECT_FALSE(scene.statics[i].flow_wall) << i;
        const glm::vec4 at = scene.statics[i].local_to_world * glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        EXPECT_NEAR(at.x, 0.174 + std::cos(angles[i]), 1e-4) << "turned about the hub's y by the azimuth, " << i;
        EXPECT_NEAR(at.z, 0.174 - std::sin(angles[i]), 1e-4) << i;
        EXPECT_NEAR(at.y, 0.0614, 1e-6) << i;
    }
    ASSERT_FALSE(scene.materials.empty());
    EXPECT_NEAR(scene.materials.back().base_color.a, 0.2f, 1e-6f) << "the blade's material at the blur's alpha";

    spade::render::RenderScene stopped;
    const std::array<double, 4> none{0.0, 0.0, 0.0, 0.0};
    draw.omega = none;
    ASSERT_TRUE(flow_movie::add_build_parts(stopped, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0), draw).has_value());
    ASSERT_EQ(stopped.statics.size(), 1u);
    EXPECT_FALSE(stopped.statics.front().see_through) << "a stopped rotor is a solid blade";
    spade::render::RenderScene no_speed;
    ASSERT_TRUE(flow_movie::add_build_parts(no_speed, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0)).has_value());
    ASSERT_EQ(no_speed.statics.size(), 1u) << "no speeds given at all: one frozen blade";
    fs::remove_all(store);
}

// THE DRAWN PROP IS THE SIMULATED PROP (the lead, 2026-11-18): KAT's rotor glTFs draw a 0.1468 m prop, the real X500 V2 flies 10 inch (R
// 0.127 m) and the model's rotors carry that radius, so the drawing's blade tips must be where the cloud's rings are. A rotor part's
// `radius:` (m, optional) scales the `prop` primitive of its glTF radially about the hub (x and z, not the blade's thickness), leaving the
// motor bell and everything else as drawn; the primitive is found by its glTF material name (the last word `prop`).
TEST(FlowMovieBuildParts, ARotorsRadiusIsReadOnlyForARotorAndMustBeAboveZero) {
    const std::string pose = "    pose: {position: [0.174, 0.0614, 0.174], orientation: [1, 0, 0, 0]}\n";
    const std::string obj = "    objects:\n      - store_ref: {sha256: " + kSha + ", file: prop.glb}\n";
    const auto good = flow_movie::parse_build_parts(table("  - name: r\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n    radius: 0.127\n" + pose + obj));
    ASSERT_TRUE(good.has_value()) << good.error().context;
    EXPECT_FLOAT_EQ((*good)[0].radius_m, 0.127f);
    const auto none = flow_movie::parse_build_parts(table("  - name: r\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n" + pose + obj));
    ASSERT_TRUE(none.has_value());
    EXPECT_FLOAT_EQ((*none)[0].radius_m, 0.0f) << "absent: the file's own size";
    for (const std::string& text : {
             table("  - name: r\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n    radius: 0\n" + pose + obj),
             table("  - name: r\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n    radius: 2\n" + pose + obj),
             table("  - name: b\n    kind: battery\n    mass_kg: 0.1\n    radius: 0.1\n" + pose + "    box: [1, 1, 1]\n")}) {
        const auto r = flow_movie::parse_build_parts(text);
        ASSERT_FALSE(r.has_value()) << text;
        EXPECT_NE(r.error().context.find("radius"), std::string::npos) << r.error().context;
    }
}

TEST(FlowMovieBuildParts, ThePropPrimitiveIsScaledRadiallyAboutTheHubAndTheBellIsNot) {
    spade::render::MeshData m;
    // submesh 0: a "prop" with tips at radius 0.1468 along x and z and a vertex off the y axis at height 0.0094
    m.positions = {{0.1468f, 0.0f, 0.0f}, {-0.1468f, 0.0094f, 0.0f}, {0.0f, 0.0094f, 0.022f}, {0.0f, -0.016f, -0.1468f}};
    m.indices = {0, 1, 2, 1, 2, 3};
    // submesh 1: a "bell" of radius 0.0295 hanging below
    m.positions.insert(m.positions.end(), {{0.0295f, -0.03f, 0.0f}, {-0.0295f, -0.03f, 0.0f}, {0.0f, -0.011f, 0.0295f}});
    m.indices.insert(m.indices.end(), {4, 5, 6});
    m.normals.assign(m.positions.size(), glm::vec3(0.0f, 1.0f, 0.0f));
    m.submesh_first_index = {0u, 6u};
    m.submesh_index_count = {6u, 3u};
    m.submesh_material = {0u, 1u};
    const spade::render::MeshData before = m;
    const auto done = flow_movie::scale_prop_radially(m, {"drone/x500-rotor-front-ccw/prop", "drone/x500-rotor-front-ccw/bell"}, 0.127f);
    ASSERT_TRUE(done.has_value()) << done.error().context;
    const float factor = 0.127f / 0.1468f;
    for (std::size_t i = 0; i < 4u; ++i) {
        EXPECT_NEAR(m.positions[i].x, before.positions[i].x * factor, 1e-6f) << i;
        EXPECT_NEAR(m.positions[i].z, before.positions[i].z * factor, 1e-6f) << i;
        EXPECT_EQ(m.positions[i].y, before.positions[i].y) << "the blade's thickness is not scaled, " << i;
    }
    EXPECT_NEAR(std::hypot(m.positions[0].x, m.positions[0].z), 0.127f, 1e-6f) << "the blade tip is at the model's radius";
    for (std::size_t i = 4u; i < 7u; ++i) EXPECT_EQ(m.positions[i], before.positions[i]) << "the bell is as drawn, " << i;
    EXPECT_EQ(m.indices, before.indices);
    EXPECT_EQ(m.normals, before.normals) << "a radial scale of a flat blade: normals are left (the blade is thin and facing y)";

    spade::render::MeshData none = before;
    const auto refused = flow_movie::scale_prop_radially(none, {"a/bell", "a/stator"}, 0.127f);
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("prop"), std::string::npos) << refused.error().context;
    EXPECT_EQ(none.positions, before.positions) << "left as it was";
    EXPECT_FALSE(flow_movie::scale_prop_radially(none, {"a/prop", "a/bell"}, 0.0f).has_value());
}

// --prop-blur alpha=0.4,samples=32,shutter=0.0167: the blade blur's three numbers on the command line (the first still's blur was not
// visible: its alpha and its samples are tuned by looking, so they are options, not constants). Empty: the defaults.
TEST(FlowMovieBuildParts, ThePropBlurOptionSetsAlphaSamplesAndShutterAndRefusesTheRest) {
    const auto defaults = flow_movie::parse_prop_blur("");
    ASSERT_TRUE(defaults.has_value()) << defaults.error().context;
    EXPECT_EQ(defaults->samples, flow_movie::PropBlur{}.samples);
    EXPECT_FLOAT_EQ(defaults->alpha, flow_movie::PropBlur{}.alpha);
    const auto set = flow_movie::parse_prop_blur("alpha=0.4,samples=32,shutter=0.02");
    ASSERT_TRUE(set.has_value()) << set.error().context;
    EXPECT_FLOAT_EQ(set->alpha, 0.4f);
    EXPECT_EQ(set->samples, 32u);
    EXPECT_DOUBLE_EQ(set->shutter_s, 0.02);
    for (const char* bad : {"alpha=0", "alpha=1.5", "samples=0", "samples=500", "samples=2.5", "shutter=0", "shutter=1", "colour=red", "alpha", "alpha=x", "alpha=0.4,"}) {
        const auto r = flow_movie::parse_prop_blur(bad);
        ASSERT_FALSE(r.has_value()) << bad;
        EXPECT_NE(r.error().context.find("prop blur"), std::string::npos) << bad << ": " << r.error().context;
    }
}

// CORE'S RECORD KEYS AND hide_materials (the lead, 2026-11-18): `mass_grade`, `mass_source` and `pose_grade` are records for Physics and the
// notes: accepted and ignored. `hide_materials: [..]` on an entry names glTF materials (by their last word) whose primitives are NOT drawn:
// the X500 frame mesh carries a built-in Pixhawk-4-class `autopilot` block where Core puts the real 6C, its rubber dampers kept. Unknown
// keys are still refused.
TEST(FlowMovieBuildParts, CoresRecordKeysAreAcceptedAndHideMaterialsIsRead) {
    const std::string pose = "    pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}\n";
    const auto parts = flow_movie::parse_build_parts(table(
        "  - name: frame\n    kind: frame\n    mass_kg: 1.2\n    mass_grade: assumed\n    mass_source: a catalogue\n    pose_grade: measured\n"
        "    hide_materials: [autopilot, antenna_holder]\n" + pose +
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: body.gltf}\n"));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    EXPECT_EQ((*parts)[0].hide_materials, (std::vector<std::string>{"autopilot", "antenna_holder"}));
    EXPECT_EQ((*parts)[0].name, "frame");
    for (const std::string& text : {
             table("  - name: p\n    kind: battery\n    mass_kg: 0.1\n    hide_materials: autopilot\n" + pose + "    box: [1, 1, 1]\n"),
             table("  - name: p\n    kind: battery\n    mass_kg: 0.1\n    hide_materials: [1]\n" + pose + "    box: [1, 1, 1]\n"),
             table("  - name: p\n    kind: battery\n    mass_kg: 0.1\n    pose_grades: x\n" + pose + "    box: [1, 1, 1]\n")}) {
        const auto r = flow_movie::parse_build_parts(text);
        ASSERT_FALSE(r.has_value()) << text;
        EXPECT_NE(r.error().context.find("build parts"), std::string::npos) << r.error().context;
    }
}

TEST(FlowMovieBuildParts, HiddenMaterialsLeaveTheirPrimitivesOutByTheirLastWordAndNothingElseMoves) {
    spade::render::MeshData m;
    for (int sub = 0; sub < 3; ++sub) {
        const float x = static_cast<float>(sub);
        m.positions.insert(m.positions.end(), {{x, 0.0f, 0.0f}, {x + 0.5f, 0.0f, 0.0f}, {x, 0.5f, 0.0f}});
        m.normals.insert(m.normals.end(), 3u, glm::vec3(0.0f, 0.0f, 1.0f));
        const auto base = static_cast<uint32_t>(3 * sub);
        m.indices.insert(m.indices.end(), {base, base + 1u, base + 2u});
        m.submesh_first_index.push_back(static_cast<uint32_t>(3 * sub));
        m.submesh_index_count.push_back(3u);
        m.submesh_material.push_back(static_cast<uint32_t>(sub));
    }
    const std::vector<std::string> names{"a/arms", "a/autopilot", "a/autopilot_rubber"};
    const auto done = flow_movie::hide_materials(m, names, {"autopilot"});
    ASSERT_TRUE(done.has_value()) << done.error().context;
    ASSERT_EQ(m.submesh_first_index.size(), 2u) << "the autopilot is gone, not the rubber (the last word must match whole)";
    EXPECT_EQ(m.submesh_material, (std::vector<uint32_t>{0u, 2u})) << "the others keep their palette entries";
    EXPECT_EQ(m.submesh_first_index, (std::vector<uint32_t>{0u, 3u})) << "contiguous again";
    EXPECT_EQ(m.submesh_index_count, (std::vector<uint32_t>{3u, 3u}));
    EXPECT_EQ(m.indices.size(), 6u);
    for (const uint32_t i : m.indices) EXPECT_LT(i, m.positions.size());
    for (const uint32_t i : m.indices) EXPECT_NE(m.positions[i].x, 1.0f) << "no vertex of the hidden primitive is drawn (it sat at x 1)";

    spade::render::MeshData before = m;
    const auto missing = flow_movie::hide_materials(m, names, {"toaster"});
    ASSERT_FALSE(missing.has_value());
    EXPECT_NE(missing.error().context.find("toaster"), std::string::npos) << missing.error().context;
    EXPECT_EQ(m.indices, before.indices) << "left as it was";
    EXPECT_TRUE(flow_movie::hide_materials(m, names, {}).has_value()) << "nothing to hide: nothing to do";
}

// CORE'S MEASURED TABLE (tools/flow_movie/data/x500_build/parts.yaml, core/x500-pose-table) as it is written, three things the loader had to learn:
// (1) `kind: mass_only`, a mass carrier that is not a drawing of anything: accepted, and nothing drawn for it; (2) its material names (`pcb`,
// `aluminium`, `carbon`, `lipo_wrap`) beside the loader's own: the same presets, the shorter names; (3) ONE top-level `hide_materials: [..]`
// (a glTF material's whole path or its last word) applying to the table's frame, since the frame is the part whose mesh carries the block.
TEST(FlowMovieBuildParts, ACoreMassOnlyPartIsAcceptedAndNothingIsDrawnForIt) {
    const std::string entry =
        "  - name: electronics_misc\n    kind: mass_only\n    mass_kg: 0.175\n    material: plastic_black\n"
        "    pose: {position: [0, 0.0113, 0], orientation: [1, 0, 0, 0]}\n    box: [0.07, 0.016, 0.07]\n";
    const auto parts = flow_movie::parse_build_parts(table(entry));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    ASSERT_EQ(parts->size(), 1u);
    EXPECT_EQ((*parts)[0].kind, "mass_only");
    spade::render::RenderScene scene;
    const auto drawn = flow_movie::add_build_parts(scene, *parts, fresh_store("mass_only"), glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    ASSERT_TRUE(drawn.has_value()) << drawn.error().context;
    EXPECT_TRUE(scene.meshes.empty());
    EXPECT_TRUE(scene.statics.empty());
    EXPECT_TRUE(scene.materials.empty());
}

TEST(FlowMovieBuildParts, CoresMaterialNamesAreTheLoadersPresetsUnderShorterNames) {
    const auto same = [](const char* core, const char* ours) {
        const auto a = flow_movie::real_material(core);
        const auto b = flow_movie::real_material(ours);
        ASSERT_TRUE(a.has_value()) << core;
        ASSERT_TRUE(b.has_value()) << ours;
        EXPECT_EQ(a->base_color, b->base_color) << core << " is " << ours;
    };
    same("pcb", "pcb_green");
    same("aluminium", "aluminium_bare");
    same("carbon", "carbon_fibre");
    const auto wrap = flow_movie::real_material("lipo_wrap");
    ASSERT_TRUE(wrap.has_value());
    for (const char* other : {"carbon_fibre", "plastic_black", "pcb_green", "rubber_black"}) {
        EXPECT_NE(wrap->base_color, flow_movie::real_material(other)->base_color) << "a battery wrap is its own colour, not " << other;
    }
    EXPECT_GT(wrap->base_color.b, wrap->base_color.r) << "a blue wrap";
    EXPECT_FALSE(flow_movie::real_material("unobtainium").has_value());
}

TEST(FlowMovieBuildParts, ATopLevelHideMaterialsAppliesToTheFrameByWholePathOrLastWord) {
    const std::string pose = "    pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}\n";
    const std::string text = "hide_materials: [drone/x500-body/autopilot]\nparts:\n"
                             "  - name: x500_frame\n    kind: frame\n    mass_kg: 1.0\n" + pose + "    box: [1, 1, 1]\n"
                             "  - name: battery\n    kind: battery\n    mass_kg: 0.4\n" + pose + "    box: [1, 1, 1]\n";
    const auto parts = flow_movie::parse_build_parts(text);
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    EXPECT_EQ((*parts)[0].hide_materials, (std::vector<std::string>{"drone/x500-body/autopilot"})) << "the frame carries it";
    EXPECT_TRUE((*parts)[1].hide_materials.empty()) << "no other part does";
    EXPECT_FALSE(flow_movie::parse_build_parts("hide_materials: autopilot\nparts:\n" + text.substr(text.find("  - name"))).has_value()) << "a list";
    EXPECT_FALSE(flow_movie::parse_build_parts("hide_material: [x]\nparts:\n" + text.substr(text.find("  - name"))).has_value()) << "other top keys are still refused";

    spade::render::MeshData m;
    for (int sub = 0; sub < 2; ++sub) {
        const float x = static_cast<float>(sub);
        m.positions.insert(m.positions.end(), {{x, 0.0f, 0.0f}, {x + 0.5f, 0.0f, 0.0f}, {x, 0.5f, 0.0f}});
        m.normals.insert(m.normals.end(), 3u, glm::vec3(0.0f, 0.0f, 1.0f));
        const auto base = static_cast<uint32_t>(3 * sub);
        m.indices.insert(m.indices.end(), {base, base + 1u, base + 2u});
        m.submesh_first_index.push_back(base);
        m.submesh_index_count.push_back(3u);
        m.submesh_material.push_back(static_cast<uint32_t>(sub));
    }
    const std::vector<std::string> names{"drone/x500-body/arms", "drone/x500-body/autopilot"};
    spade::render::MeshData by_path = m;
    ASSERT_TRUE(flow_movie::hide_materials(by_path, names, {"drone/x500-body/autopilot"}).has_value());
    EXPECT_EQ(by_path.submesh_material, (std::vector<uint32_t>{0u})) << "the whole path hides it";
    spade::render::MeshData wrong_path = m;
    EXPECT_FALSE(flow_movie::hide_materials(wrong_path, names, {"other/x500-body/autopilot"}).has_value()) << "a different path matches nothing";
    EXPECT_EQ(wrong_path.submesh_material.size(), 2u) << "left as it was";
}

// THE SEAM THE FIRST X500 STILLS FELL THROUGH (the lead, 2026-11-18: the spread blade was not visible in the frame): a rotor drawn from a
// REAL glTF (a prop primitive and a bell, two materials named drone/<file>/prop and /bell, one mesh, as KAT's rotor files are) through the
// whole path (names, real materials, the radial scale, the blur copies) and then RASTERISED, not only counted. A black blade over a pale sky:
// a turning rotor must darken a DISC of pixels, many times what the same blade frozen does; a frozen one is a solid blade; the blur's alpha
// is the blade's, the bell stays opaque.
namespace {

std::string base64_of(const std::vector<uint8_t>& bytes) {
    static const char* kDigits = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (std::size_t i = 0; i < bytes.size(); i += 3u) {
        const uint32_t a = bytes[i];
        const uint32_t b = i + 1u < bytes.size() ? bytes[i + 1u] : 0u;
        const uint32_t c = i + 2u < bytes.size() ? bytes[i + 2u] : 0u;
        const uint32_t v = (a << 16) | (b << 8) | c;
        out += kDigits[(v >> 18) & 63u];
        out += kDigits[(v >> 12) & 63u];
        out += i + 1u < bytes.size() ? kDigits[(v >> 6) & 63u] : '=';
        out += i + 2u < bytes.size() ? kDigits[v & 63u] : '=';
    }
    return out;
}

// A two-primitive rotor glTF: a flat blade along +x (chord 0.02, from 0.02 to `reach`, both windings) on material 0 "t/prop", and a small
// tetrahedron bell on material 1 "t/bell", both black-ish (the real materials replace them by name).
void save_rotor_gltf(const fs::path& file, float reach) {
    const std::vector<float> positions{0.02f, 0.0f, -0.01f, reach, 0.0f, -0.01f, reach, 0.0f, 0.01f, 0.02f, 0.0f, 0.01f,
                                       0.0f,  0.0f, 0.0f,   0.02f, 0.0f, 0.0f,  0.0f,  0.02f, 0.0f, 0.0f,  0.0f, 0.02f};
    std::vector<float> normals(24, 0.0f);
    for (std::size_t i = 1; i < 24; i += 3u) normals[i] = 1.0f;
    const std::vector<uint16_t> indices{0, 1, 2, 0, 2, 3, 0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 5, 7, 4, 7, 6, 5, 6, 7};
    std::vector<uint8_t> bytes(positions.size() * 4u + normals.size() * 4u + indices.size() * 2u);
    std::memcpy(bytes.data(), positions.data(), positions.size() * 4u);
    std::memcpy(bytes.data() + positions.size() * 4u, normals.data(), normals.size() * 4u);
    std::memcpy(bytes.data() + positions.size() * 4u + normals.size() * 4u, indices.data(), indices.size() * 2u);
    std::ofstream out(file, std::ios::binary);
    out << R"({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"mesh":0}],)"
        << R"("meshes":[{"primitives":[{"attributes":{"POSITION":0,"NORMAL":1},"indices":2,"material":0,"mode":4},)"
        << R"({"attributes":{"POSITION":0,"NORMAL":1},"indices":3,"material":1,"mode":4}]}],)"
        << R"("materials":[{"name":"t/prop","pbrMetallicRoughness":{"baseColorFactor":[0.05,0.05,0.06,1]}},)"
        << R"({"name":"t/bell","pbrMetallicRoughness":{"baseColorFactor":[0.2,0.2,0.2,1]}}],)"
        << R"("accessors":[{"bufferView":0,"componentType":5126,"count":8,"type":"VEC3","min":[0,0,-0.01],"max":[)" << reach
        << R"(,0.02,0.02]},{"bufferView":1,"componentType":5126,"count":8,"type":"VEC3"},)"
        << R"({"bufferView":2,"componentType":5123,"count":12,"type":"SCALAR"},)"
        << R"({"bufferView":2,"byteOffset":24,"componentType":5123,"count":12,"type":"SCALAR"}],)"
        << R"("bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":96},{"buffer":0,"byteOffset":96,"byteLength":96},)"
        << R"({"buffer":0,"byteOffset":192,"byteLength":48}],)"
        << R"("buffers":[{"byteLength":240,"uri":"data:application/octet-stream;base64,)" << base64_of(bytes) << R"("}]})";
}

// The count of dark pixels (the black blade over a pale sky) of a rotor part rasterised from 20 degrees above its plane.
uint32_t dark_pixels(const std::string& rotor_entry, const flow_movie::BuildDraw& draw, const fs::path& store) {
    const auto parts = flow_movie::parse_build_parts("parts:\n" + rotor_entry);
    EXPECT_TRUE(parts.has_value()) << (parts ? "" : parts.error().context);
    if (!parts) return 0u;
    spade::render::RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.9f);
    scene.lighting.sky_horizon = glm::vec3(0.9f);
    const auto added = flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), draw);
    EXPECT_TRUE(added.has_value()) << (added ? "" : added.error().context);
    if (!added) return 0u;
    constexpr uint32_t kW = 160, kH = 120;
    spade::render::Camera camera;
    const glm::vec3 eye(0.0f, 0.22f, 0.60f);
    const glm::mat4 view = glm::lookAt(eye, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    camera.position = eye;
    camera.orientation = glm::quat_cast(glm::mat3(glm::inverse(view)));
    std::vector<uint8_t> px(static_cast<std::size_t>(kW) * kH * 4u, 0xABu);
    spade::render::RenderTarget target{.pixels = std::span<uint8_t>(px), .width = kW, .height = kH, .stride = kW * 4u};
    spade::render::RenderOptions options;
    options.overlays = false;
    const auto rendered = spade::render::render(scene, camera, options, target);
    EXPECT_TRUE(rendered.has_value()) << (rendered ? "" : rendered.error().context);
    uint32_t dark = 0;
    for (std::size_t i = 0; i < px.size(); i += 4u) {
        if (static_cast<int>(px[i]) + px[i + 1] + px[i + 2] < 3 * 110) ++dark;
    }
    return dark;
}

}  // namespace

TEST(FlowMovieBuildParts, ATurningRotorFromARealGltfDarkensADiscAndAFrozenOneOnlyItsBlade) {
    const fs::path store = fresh_store("blades");
    save_rotor_gltf(store / kSha / "rotor.gltf", 0.127f);
    const std::string entry =
        "  - name: rotor_fr\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n    radius: 0.127\n"
        "    pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: rotor.gltf}\n";
    flow_movie::BuildDraw turning;
    turning.blur.samples = 48u;
    turning.blur.alpha = 0.9f;
    const std::array<double, 1> omega{343.667};
    turning.omega = omega;
    flow_movie::BuildDraw frozen;  // no speeds: one solid blade
    const uint32_t blur_px = dark_pixels(entry, turning, store);
    const uint32_t solid_px = dark_pixels(entry, frozen, store);
    EXPECT_GT(solid_px, 8u) << "the frozen blade itself is there";
    EXPECT_GT(blur_px, 4u * solid_px) << "the spread blade covers a disc, many times one blade's pixels: " << blur_px << " vs " << solid_px;
    fs::remove_all(store);
}

