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
#include "x500_model.hpp"

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

// THE TABLE IS IN THE DESIGN FRAME, NOT THE BODY'S (the lead, 2026-11-19, L104a): the kit table's origin is the bottom plate's lower face and its axes are
// the design axes, while the body the sim flies is turned to its principal axes (q_bd, design_to_principal) and sits at the centre of mass (com_offset, from the
// design origin, in the design frame). A part at table point p is therefore drawn at body_position + body_orient * q_bd * (p - com_offset), turned by
// body_orient * q_bd * its own orientation (sim/design_frame.hpp's R_wd = q_wb * q_bd). Hand-computed on quarter turns: q_bd 90 degrees about z takes
// (x, y, z) to (-y, x, z), the body 90 degrees about x takes (x, y, z) to (x, -z, y).
TEST(FlowMovieBuildParts, APartAtADesignFramePointLandsWhereTheDesignFramesRigidBodyIdentitiesPutIt) {
    const std::string entry =
        "  - name: battery\n    kind: battery\n    mass_kg: 0.4\n    pose: {position: [1, 2, 3], orientation: [1, 0, 0, 0]}\n"
        "    box: [0.10, 0.03, 0.04]\n";
    const auto parts = flow_movie::parse_build_parts(table(entry));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    constexpr float kHalfSqrt2 = 0.70710678118f;
    flow_movie::BuildDraw draw;
    draw.design_to_principal = glm::quat(kHalfSqrt2, 0.0f, 0.0f, kHalfSqrt2);  // w, x, y, z: 90 degrees about z
    draw.com_offset = glm::vec3(0.1f, 0.2f, 0.3f);
    const glm::quat body_orient(kHalfSqrt2, kHalfSqrt2, 0.0f, 0.0f);          // 90 degrees about x
    spade::render::RenderScene scene;
    const auto added = flow_movie::add_build_parts(scene, *parts, fs::path("unused"), glm::vec3(10.0f, 20.0f, 30.0f), body_orient, draw);
    ASSERT_TRUE(added.has_value()) << added.error().context;
    ASSERT_EQ(scene.statics.size(), 1u);
    // p - c = (0.9, 1.8, 2.7); q_bd: (-1.8, 0.9, 2.7); the body: (-1.8, -2.7, 0.9); plus the body's position
    const glm::vec4 centre = scene.statics.front().local_to_world * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    EXPECT_NEAR(centre.x, 8.2f, 1e-4f);
    EXPECT_NEAR(centre.y, 17.3f, 1e-4f);
    EXPECT_NEAR(centre.z, 30.9f, 1e-4f);
    // the part's own +x axis: q_bd takes it to +y, the body's turn takes +y to +z
    const glm::vec4 x = scene.statics.front().local_to_world * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_NEAR(x.x, 0.0f, 1e-4f);
    EXPECT_NEAR(x.y, 0.0f, 1e-4f);
    EXPECT_NEAR(x.z, 1.0f, 1e-4f);
}

// The frame comes from the model file the run flies (BuildDraw::design_to_principal and com_offset, read by body_frame_of): the committed X500 model's.
// The centre of mass is pinned to the kit's own (spade::testing::x500::kKitCom, the number the model file is generated from, tests/x500_model.hpp), not to
// a copy of it: a copy went stale when the kit model landed (gate 40, 8d026c5c: two branches each passed alone), and the next model change must not do that
// silently. The file's digits are the kit's rounded to float, so a float tolerance holds them together.
TEST(FlowMovieBuildParts, TheBodyFrameIsReadFromTheModelFileTheRunFlies) {
    const auto frame = flow_movie::body_frame_of(fs::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500/x500.model.yaml");
    ASSERT_TRUE(frame.has_value()) << frame.error().context;
    const glm::dvec3 kit = spade::testing::x500::kKitCom;
    EXPECT_NEAR(static_cast<double>(frame->com_offset.x), kit.x, 1e-7);
    EXPECT_NEAR(static_cast<double>(frame->com_offset.y), kit.y, 1e-7);
    EXPECT_NEAR(static_cast<double>(frame->com_offset.z), kit.z, 1e-7);
    EXPECT_NEAR(glm::length(frame->design_to_principal), 1.0f, 1e-6f) << "a unit quaternion";
    EXPECT_FALSE(flow_movie::body_frame_of(fs::path("no/such/model.yaml")).has_value());
}

// THE PRESSURE FILM'S WALLS (O124): the surface-pressure colouring reads only draw items flagged flow_wall (RND-8), and the parts drawn here are the solids the
// sim flew, so they are walls (--surface-pressure asks BuildDraw::flow_walls); the blade spread of a turning rotor is see-through and is no wall (an actuator
// line is no surface the solver saw). Without the ask no item is a wall, as before.
TEST(FlowMovieBuildParts, TheDrawnPartsAreFlowWallsOnlyWhenAskedAndABladeBlurNever) {
    const auto parts = flow_movie::parse_build_parts(table(kBoxEntry));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    spade::render::RenderScene plain;
    ASSERT_TRUE(flow_movie::add_build_parts(plain, *parts, fs::path("unused"), glm::vec3(0.0f), glm::quat(1, 0, 0, 0)).has_value());
    ASSERT_FALSE(plain.statics.empty());
    for (const auto& item : plain.statics) EXPECT_FALSE(item.flow_wall) << "no ask, no wall";

    flow_movie::BuildDraw draw;
    draw.flow_walls = true;
    spade::render::RenderScene walls;
    walls.statics.emplace_back();  // something drawn before the parts: not theirs to flag
    ASSERT_TRUE(flow_movie::add_build_parts(walls, *parts, fs::path("unused"), glm::vec3(0.0f), glm::quat(1, 0, 0, 0), draw).has_value());
    ASSERT_GE(walls.statics.size(), 2u);
    EXPECT_FALSE(walls.statics.front().flow_wall) << "the item that was already there";
    for (std::size_t i = 1; i < walls.statics.size(); ++i) EXPECT_TRUE(walls.statics[i].flow_wall) << "part item " << i;

    spade::render::RenderScene scene;
    scene.statics.resize(3);
    scene.statics[1].see_through = true;
    flow_movie::mark_flow_walls(scene, 1u);
    EXPECT_FALSE(scene.statics[0].flow_wall) << "before `first`";
    EXPECT_FALSE(scene.statics[1].flow_wall) << "a see-through blade blur is no wall";
    EXPECT_TRUE(scene.statics[2].flow_wall);
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
    // The kit's motor mounts and cans are BLACK anodised aluminium: beside the carbon they read as black metal, not as grey parts (the kit stills,
    // 2026-11-18: at 0.11 to 0.15 the mounts lit to a pale grey L at every arm end), and a little lighter than the carbon so the metal still shows.
    EXPECT_LT(std::max({anodised.x, anodised.y, anodised.z}), 0.07f) << "black anodised reads black";
    EXPECT_GT(anodised.x + anodised.y + anodised.z, carbon.x + carbon.y + carbon.z) << "and a little lighter than the carbon";
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
    draw.blur.sampled = true;
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

// THE REAL BLADES (--prop-blur mode=blades, O128): the kit's own blade mesh, ONE opaque copy a rotor, turned about the hub's +Y by s psi, psi the simulation's blade
// azimuth (LiveRun::blade_azimuth, posed_parts' R_y(s psi)). The actuator line puts blade b at phi = s psi + 2 pi b / B, d = ref cos phi + (axis x ref) sin phi with the
// thrust axis +Y and ref the part's own +X, so blade 0's direction is (cos phi, 0, -sin phi): the drawn blade and the line coincide, for both hands, at psi 0 and after.
TEST(FlowMovieBuildParts, ABladesModeRotorIsOneOpaqueBladeAtTheSimulationsAzimuthWhereTheLinePutsBladeZero) {
    const fs::path store = fresh_store("blades");
    save_tetra(store / kSha / "prop.spmesh");
    const auto parts = flow_movie::parse_build_parts(table(
        "  - name: rotor_fl\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n"
        "    pose: {position: [0.174, 0.0614, 0.174], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: prop.spmesh}\n"
        "  - name: rotor_fr\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 1\n    spin: -1\n"
        "    pose: {position: [-0.174, 0.0614, 0.174], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: prop.spmesh}\n"));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    const std::array<double, 2> omega{567.0, 567.0};
    const glm::vec3 hub[2] = {glm::vec3(0.174f, 0.0614f, 0.174f), glm::vec3(-0.174f, 0.0614f, 0.174f)};
    const int spin[2] = {1, -1};
    for (const double psi0 : {0.0, 1.5707963267948966, 2.5}) {
        flow_movie::BuildDraw draw;
        draw.blur.blades = true;
        draw.omega = omega;
        const std::array<double, 2> psi{psi0, psi0};
        draw.psi = psi;
        spade::render::RenderScene scene;
        const auto added = flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0), draw);
        ASSERT_TRUE(added.has_value()) << added.error().context;
        ASSERT_EQ(scene.statics.size(), 2u) << "one blade a rotor: no copies, no disc";
        for (std::size_t k = 0; k < 2u; ++k) {
            EXPECT_FALSE(scene.statics[k].see_through) << "a real blade is opaque, " << k;
            const double phi = static_cast<double>(spin[k]) * psi0;  // the line's blade 0: phi = s psi
            const glm::vec4 at = scene.statics[k].local_to_world * glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
            EXPECT_NEAR(at.x - hub[k].x, std::cos(phi), 1e-5) << "rotor " << k << " psi " << psi0;
            EXPECT_NEAR(at.z - hub[k].z, -std::sin(phi), 1e-5) << "rotor " << k << " psi " << psi0;
            EXPECT_NEAR(at.y, hub[k].y, 1e-6) << k;
        }
    }
    fs::remove_all(store);
}

// A blades-mode rotor with no blade azimuth for it is refused, naming it: drawing azimuth 0 would claim an angle the simulation did not have.
TEST(FlowMovieBuildParts, ABladesModeRotorWithNoAzimuthIsRefusedNamingIt) {
    const fs::path store = fresh_store("blades_refused");
    save_tetra(store / kSha / "prop.spmesh");
    const auto parts = flow_movie::parse_build_parts(table(
        "  - name: rotor_fl\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n"
        "    pose: {position: [0.174, 0.0614, 0.174], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: prop.spmesh}\n"));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    flow_movie::BuildDraw draw;
    draw.blur.blades = true;
    const std::array<double, 1> omega{567.0};
    draw.omega = omega;
    spade::render::RenderScene scene;
    const auto added = flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0), draw);
    ASSERT_FALSE(added.has_value());
    EXPECT_NE(added.error().context.find("blade azimuth"), std::string::npos) << added.error().context;
    EXPECT_NE(added.error().context.find("rotor_fl"), std::string::npos) << added.error().context;
    fs::remove_all(store);
}

// The blades read the air on THEIR OWN fixed range: with the pressure film's walls on, a blades-mode blade is a flow wall carrying BuildDraw::blade_flow_gain (the
// colouring's range over the blades' own), and every other part keeps gain 1. A blade that is not a wall in the flow (no --surface-pressure) is still drawn.
TEST(FlowMovieBuildParts, TheBladesAreFlowWallsWithTheirOwnRangeGainAndTheOtherPartsKeepGainOne) {
    const fs::path store = fresh_store("blade_gain");
    save_tetra(store / kSha / "prop.spmesh");
    const auto parts = flow_movie::parse_build_parts(table(
        std::string(kBoxEntry) +
        "  - name: rotor_fl\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n"
        "    pose: {position: [0.174, 0.0614, 0.174], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: prop.spmesh}\n"));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    const std::array<double, 1> omega{567.0};
    const std::array<double, 1> psi{0.3};
    flow_movie::BuildDraw draw;
    draw.blur.blades = true;
    draw.omega = omega;
    draw.psi = psi;
    draw.blade_flow_gain = 0.25f;
    draw.flow_walls = true;
    spade::render::RenderScene scene;
    ASSERT_TRUE(flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0), draw).has_value());
    ASSERT_GE(scene.statics.size(), 2u);
    const auto& blade = scene.statics.back();
    EXPECT_TRUE(blade.flow_wall);
    EXPECT_FLOAT_EQ(blade.flow_gain, 0.25f);
    for (std::size_t i = 0; i + 1 < scene.statics.size(); ++i) {
        EXPECT_TRUE(scene.statics[i].flow_wall) << i;
        EXPECT_FLOAT_EQ(scene.statics[i].flow_gain, 1.0f) << "a part that is no blade, " << i;
    }
    draw.flow_walls = false;
    spade::render::RenderScene plain;
    ASSERT_TRUE(flow_movie::add_build_parts(plain, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0), draw).has_value());
    for (const auto& item : plain.statics) EXPECT_FALSE(item.flow_wall) << "no ask, no wall";
    fs::remove_all(store);
}

// --prop-blur mode=blades is a third mode beside the disc and the sampled copies; the others are as they were, and the note says what is drawn.
TEST(FlowMovieBuildParts, ThePropBlurBladesModeIsParsedAndSaysWhatItDraws) {
    const auto blades = flow_movie::parse_prop_blur("mode=blades");
    ASSERT_TRUE(blades.has_value()) << blades.error().context;
    EXPECT_TRUE(blades->blades);
    EXPECT_FALSE(blades->sampled);
    const auto sampled = flow_movie::parse_prop_blur("mode=sampled");
    ASSERT_TRUE(sampled.has_value());
    EXPECT_FALSE(sampled->blades);
    EXPECT_TRUE(sampled->sampled);
    const auto plain = flow_movie::parse_prop_blur("");
    ASSERT_TRUE(plain.has_value());
    EXPECT_FALSE(plain->blades);
    const auto bad = flow_movie::parse_prop_blur("mode=propellers");
    ASSERT_FALSE(bad.has_value());
    EXPECT_NE(bad.error().context.find("blades"), std::string::npos) << "the refusal lists the modes: " << bad.error().context;
    const std::string note = flow_movie::prop_blur_note(*blades);
    EXPECT_NE(note.find("real blade"), std::string::npos) << note;
    EXPECT_NE(note.find("simulation's rotor angle"), std::string::npos) << note;
    EXPECT_EQ(note.find("copies"), std::string::npos) << note;
    EXPECT_EQ(note.find("disc"), std::string::npos) << note;
}

// The blades' azimuths ride in the frame's live record, after everything the draw already reads: [7 body][a speed a rotor][the lines' thrust, when lines][a psi a rotor].
// They are the LAST `rotors` values, and a record too short to hold them has none (the plain 11- and 12-value records draw as before).
TEST(FlowMovieBuildParts, TheBladeAzimuthsAreTheLastRotorsValuesOfALongEnoughLiveRecord) {
    const std::vector<double> plain{0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 567.0, 567.0, 567.0, 567.0};
    EXPECT_TRUE(flow_movie::rotor_azimuths_of_live(plain, 4u).empty());
    std::vector<double> lines = plain;
    lines.push_back(16.0);  // the lines' thrust
    EXPECT_TRUE(flow_movie::rotor_azimuths_of_live(lines, 4u).empty()) << "the thrust is not an azimuth";
    std::vector<double> with = lines;
    for (const double a : {0.5, 1.5, 2.5, 3.5}) with.push_back(a);
    const auto psi = flow_movie::rotor_azimuths_of_live(with, 4u);
    ASSERT_EQ(psi.size(), 4u);
    EXPECT_DOUBLE_EQ(psi[0], 0.5);
    EXPECT_DOUBLE_EQ(psi[3], 3.5);
    EXPECT_EQ(flow_movie::rotor_speeds_of_live(with, 4u).size(), 4u) << "the speeds are still the four after the body";
    EXPECT_DOUBLE_EQ(flow_movie::rotor_speeds_of_live(with, 4u)[3], 567.0);
    std::vector<double> no_thrust = plain;
    for (const double a : {0.5, 1.5, 2.5, 3.5}) no_thrust.push_back(a);
    EXPECT_EQ(flow_movie::rotor_azimuths_of_live(no_thrust, 4u).size(), 4u) << "a record with no lines' thrust";
}

// The frame's live record is the body (7: position, orientation), then one speed a rotor, then, since L106c, the lines' thrust in all (the harness's 12th
// value). The thrust is LAST and is not a speed: a draw that counts the rotors from the record's length would spin a fifth rotor at 13 rad/s. The speeds are
// read by the TABLE's rotor count (rotor_speeds_of_live), so a 12-value record draws the same frame as its 11-value record (the lead, 2026-11-19).
TEST(FlowMovieBuildParts, TheLinesThrustAppendedToTheLiveRecordIsNotARotorSpeedAndDrawsTheSameFrame) {
    const std::vector<double> eleven{0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 567.0, 567.0, 567.0, 567.0};
    std::vector<double> twelve = eleven;
    twelve.push_back(13.012);  // the thrust, N
    const auto a = flow_movie::rotor_speeds_of_live(eleven, 4u);
    const auto b = flow_movie::rotor_speeds_of_live(twelve, 4u);
    ASSERT_EQ(a.size(), 4u);
    ASSERT_EQ(b.size(), 4u) << "four rotors: the 12th value is the thrust, not a fifth speed";
    for (std::size_t i = 0; i < 4u; ++i) EXPECT_EQ(a[i], b[i]) << i;
    EXPECT_EQ(b[3], 567.0);
    EXPECT_TRUE(flow_movie::rotor_speeds_of_live(std::vector<double>{0.1, 0.2, 0.3}, 4u).empty()) << "no body, no speeds";
    EXPECT_EQ(flow_movie::rotor_speeds_of_live(std::vector<double>{0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 5.0}, 4u).size(), 1u) << "a short record gives what it has";
    EXPECT_EQ(flow_movie::rotor_speeds_of_live(twelve, 0u).size(), 0u);

    const fs::path store = fresh_store("thrust_last");
    save_tetra(store / kSha / "prop.spmesh");
    std::string text;
    for (int r = 0; r < 4; ++r) {
        text += "  - name: rotor_" + std::to_string(r) + "\n    kind: rotor\n    mass_kg: 0.02\n    rotor: " + std::to_string(r) + "\n    spin: 1\n"
                "    pose: {position: [0.17, 0.06, 0.17], orientation: [1, 0, 0, 0]}\n"
                "    objects:\n      - store_ref: {sha256: " + kSha + ", file: prop.spmesh}\n";
    }
    const auto parts = flow_movie::parse_build_parts(table(text));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    EXPECT_EQ(flow_movie::rotor_count_of(*parts), 4u);
    flow_movie::BuildDraw d11, d12;
    d11.blur.sampled = d12.blur.sampled = true;
    d11.blur.samples = d12.blur.samples = 8u;
    d11.blur.alpha = d12.blur.alpha = 0.2f;
    d11.omega = flow_movie::rotor_speeds_of_live(eleven, flow_movie::rotor_count_of(*parts));
    d12.omega = flow_movie::rotor_speeds_of_live(twelve, flow_movie::rotor_count_of(*parts));
    spade::render::RenderScene s11, s12;
    ASSERT_TRUE(flow_movie::add_build_parts(s11, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0), d11).has_value());
    ASSERT_TRUE(flow_movie::add_build_parts(s12, *parts, store, glm::vec3(0.0f), glm::quat(1, 0, 0, 0), d12).has_value());
    ASSERT_EQ(s11.statics.size(), s12.statics.size());
    for (std::size_t i = 0; i < s11.statics.size(); ++i) {
        EXPECT_EQ(s11.statics[i].see_through, s12.statics[i].see_through) << i;
        EXPECT_EQ(s11.statics[i].local_to_world, s12.statics[i].local_to_world) << i;
    }
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
// whole path (names, real materials, the radial scale, the blur copies) and then RASTERISED, not only counted. A blade over a pale sky:
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
        if (static_cast<int>(px[i]) + px[i + 1] + px[i + 2] < 3 * 190) ++dark;  // under the pale sky (230): the black blade or the grey haze
    }
    return dark;
}

// The count of LIT pixels of a rotor part over a BLACK sky (the first stills' background): the blur's tint, not the blade's black, shows there.
uint32_t lit_pixels(const std::string& rotor_entry, const flow_movie::BuildDraw& draw, const fs::path& store) {
    const auto parts = flow_movie::parse_build_parts("parts:\n" + rotor_entry);
    EXPECT_TRUE(parts.has_value()) << (parts ? "" : parts.error().context);
    if (!parts) return 0u;
    spade::render::RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
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
    uint32_t lit = 0;
    for (std::size_t i = 0; i < px.size(); i += 4u) {
        if (static_cast<int>(px[i]) + px[i + 1] + px[i + 2] > 3 * 40) ++lit;
    }
    return lit;
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
    turning.blur.sampled = true;
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

// WHY THE SPREAD BLADE DID NOT SHOW (the first X500 stills; the CPU raster test above passes, so the drawing is right): the blur copies were
// the prop's own BLACK (a real propeller's colour) over a frame whose background is black or dark blue cloud: a black disc on black is
// nothing. A blurred propeller is a smoky, lit haze: the blur copies take a TINT, a pale grey (PropBlur::tint, --prop-blur tint=..), the
// frozen blade stays black. Over a BLACK sky the turning rotor must light a disc of pixels where a frozen black blade lights almost none.
TEST(FlowMovieBuildParts, TheBlurTintIsAPaleGreyOptionAndTheSpreadBladeShowsOverABlackSky) {
    const flow_movie::PropBlur defaults;
    EXPECT_GE(defaults.tint, 0.3f);
    EXPECT_LE(defaults.tint, 0.7f);
    const auto set = flow_movie::parse_prop_blur("tint=0.6,alpha=0.3");
    ASSERT_TRUE(set.has_value()) << set.error().context;
    EXPECT_FLOAT_EQ(set->tint, 0.6f);
    EXPECT_FLOAT_EQ(set->alpha, 0.3f);
    for (const char* bad : {"tint=0", "tint=1.5", "tint=x"}) {
        const auto r = flow_movie::parse_prop_blur(bad);
        ASSERT_FALSE(r.has_value()) << bad;
        EXPECT_NE(r.error().context.find("prop blur"), std::string::npos) << r.error().context;
    }

    const fs::path store = fresh_store("tint");
    save_rotor_gltf(store / kSha / "rotor.gltf", 0.127f);
    const std::string entry =
        "  - name: rotor_fr\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n    radius: 0.127\n"
        "    pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: rotor.gltf}\n";
    const auto parts = flow_movie::parse_build_parts("parts:\n" + entry);
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    flow_movie::BuildDraw turning;
    turning.blur.sampled = true;
    const std::array<double, 1> omega{343.667};
    turning.omega = omega;
    spade::render::RenderScene scene;
    ASSERT_TRUE(flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), turning).has_value());
    float brightest = 0.0f;
    for (const auto& m : scene.materials) brightest = std::max(brightest, m.base_color.r);
    EXPECT_NEAR(brightest, turning.blur.tint, 0.08f) << "the blur copies' colour is the tint, not the propeller's black";

    const uint32_t spread = lit_pixels(entry, turning, store);
    const uint32_t frozen = lit_pixels(entry, flow_movie::BuildDraw{}, store);
    EXPECT_GT(spread, 150u) << "the blur is a disc of light over a black sky";
    EXPECT_GT(spread, 4u * (frozen + 10u)) << "and a frozen black blade is nearly nothing there: " << spread << " vs " << frozen;
    fs::remove_all(store);
}

// THE TURNING PROPELLER AS A DISC (the lead, 2026-11-18: the 48-copy blur read as a white daisy from above; a real prop seen over a shutter is a
// smooth translucent disc). The time average of a blade turning is a disc whose coverage at radius r is the blade's SOLIDITY there,
// sigma(r) = B c(r) / (2 pi r), from the prop part's own chord table (blade.c_over_r at blade.r_over_r, B = prop.blades): the same UIUC data
// the physics reads. Drawn as concentric rings, each one flat colour at alpha = sigma(r) x the blade's opacity: no angular structure at all.
namespace {

flow_movie::BladeChord uiuc_10x5() {
    flow_movie::BladeChord c;
    c.blades = 2u;
    c.r_over_r = {0.15f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f, 0.55f, 0.6f, 0.65f, 0.7f, 0.75f, 0.8f, 0.85f, 0.9f, 0.95f, 1.0f};
    c.c_over_r = {0.13f, 0.149f, 0.173f, 0.189f, 0.197f, 0.201f, 0.2f, 0.194f, 0.186f, 0.174f, 0.16f, 0.145f, 0.128f, 0.112f, 0.096f, 0.081f, 0.061f, 0.041f};
    c.hub_cutout = 0.15f;
    return c;
}

constexpr double kTwoPi = 6.283185307179586;

// A rotor part's rasterised luminance (sum of the channels over three) from straight above, over a BLACK sky, 240 x 240, the disc's hub at the
// centre; the pixel radius of the blade tip (R = 0.127 m) is returned beside it.
struct TopView {
    std::vector<float> luma;
    float tip_px = 0.0f;
};

TopView top_view(const std::string& rotor_entry, const flow_movie::BuildDraw& draw, const fs::path& store) {
    TopView view;
    const auto parts = flow_movie::parse_build_parts("parts:\n" + rotor_entry);
    EXPECT_TRUE(parts.has_value()) << (parts ? "" : parts.error().context);
    if (!parts) return view;
    spade::render::RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    const auto added = flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), draw);
    EXPECT_TRUE(added.has_value()) << (added ? "" : added.error().context);
    if (!added) return view;
    constexpr uint32_t kN = 240;
    spade::render::Camera camera;
    const glm::vec3 eye(0.0f, 0.35f, 0.0f);
    const glm::mat4 look = glm::lookAt(eye, glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
    camera.position = eye;
    camera.orientation = glm::quat_cast(glm::mat3(glm::inverse(look)));
    std::vector<uint8_t> px(static_cast<std::size_t>(kN) * kN * 4u, 0xABu);
    spade::render::RenderTarget target{.pixels = std::span<uint8_t>(px), .width = kN, .height = kN, .stride = kN * 4u};
    spade::render::RenderOptions options;
    options.overlays = false;
    const auto rendered = spade::render::render(scene, camera, options, target);
    EXPECT_TRUE(rendered.has_value()) << (rendered ? "" : rendered.error().context);
    if (!rendered) return view;
    view.luma.resize(static_cast<std::size_t>(kN) * kN);
    for (std::size_t i = 0; i < view.luma.size(); ++i) {
        view.luma[i] = (static_cast<float>(px[4u * i]) + px[4u * i + 1u] + px[4u * i + 2u]) / 3.0f;
    }
    const double f = 1.0 / std::tan(static_cast<double>(camera.fov_y_radians) * 0.5);
    view.tip_px = static_cast<float>(f * 0.127 / 0.35 * (kN * 0.5));
    return view;
}

// The mean luminance of the 3 x 3 pixels at `frac` of the tip radius, azimuth `az` (radians) round the hub.
float luma_at(const TopView& v, float frac, double az) {
    constexpr int kN = 240;
    const double r = static_cast<double>(frac) * static_cast<double>(v.tip_px);
    const int cx = static_cast<int>(std::lround(kN * 0.5 + r * std::cos(az)));
    const int cy = static_cast<int>(std::lround(kN * 0.5 + r * std::sin(az)));
    float sum = 0.0f;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) sum += v.luma[static_cast<std::size_t>(cy + dy) * kN + static_cast<std::size_t>(cx + dx)];
    }
    return sum / 9.0f;
}

}  // namespace

TEST(FlowMovieBuildParts, TheSolidityOfADiscIsTheBladeCoverageAtThatRadiusFromTheChordTable) {
    const flow_movie::BladeChord chord = uiuc_10x5();
    const auto sigma = [](double b, double c, double r) { return static_cast<float>(b * c / (kTwoPi * r)); };
    EXPECT_NEAR(flow_movie::blade_solidity(chord, 0.5f), sigma(2.0, 0.194, 0.5), 1e-4f);
    EXPECT_NEAR(flow_movie::blade_solidity(chord, 0.75f), sigma(2.0, 0.128, 0.75), 1e-4f);
    EXPECT_NEAR(flow_movie::blade_solidity(chord, 0.95f), sigma(2.0, 0.061, 0.95), 1e-4f);
    EXPECT_NEAR(flow_movie::blade_solidity(chord, 0.525f), sigma(2.0, 0.5 * (0.194 + 0.186), 0.525), 1e-4f) << "c/R is linear between the stations";
    EXPECT_FLOAT_EQ(flow_movie::blade_solidity(chord, 0.05f), 0.0f) << "inside the hub";
    EXPECT_FLOAT_EQ(flow_movie::blade_solidity(chord, 1.2f), 0.0f) << "past the tip";
    flow_movie::BladeChord three = chord;
    three.blades = 3u;
    EXPECT_NEAR(flow_movie::blade_solidity(three, 0.5f), 1.5f * flow_movie::blade_solidity(chord, 0.5f), 1e-5f) << "B blades";
}

TEST(FlowMovieBuildParts, TheChordTableIsReadFromThePropPartFileTheX500Uses) {
    const auto chord = flow_movie::read_blade_chord(fs::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500/parts/x500_10in_apc10x5.prop.yaml");
    ASSERT_TRUE(chord.has_value()) << chord.error().context;
    EXPECT_EQ(chord->blades, 2u);
    ASSERT_EQ(chord->r_over_r.size(), 18u);
    ASSERT_EQ(chord->c_over_r.size(), 18u);
    EXPECT_FLOAT_EQ(chord->hub_cutout, 0.15f);
    EXPECT_FLOAT_EQ(chord->c_over_r[7], 0.194f);
    EXPECT_FALSE(flow_movie::read_blade_chord(fs::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500_build/parts.yaml").has_value()) << "a table is not a prop part";
    EXPECT_FALSE(flow_movie::read_blade_chord(fs::path(SPADE_REPO_DIR) / "no/such/prop.yaml").has_value());
}

namespace {

// A ring's radial extent (as a fraction of the radius) from its triangles' vertices.
struct RingExtent {
    float lo = 1e9f, hi = -1e9f;
};
RingExtent ring_extent(const spade::render::MeshData& mesh, std::size_t ring, float radius) {
    RingExtent e;
    const uint32_t first = mesh.submesh_first_index[ring];
    for (uint32_t i = first; i < first + mesh.submesh_index_count[ring]; ++i) {
        const glm::vec3 v = mesh.positions[mesh.indices[i]];
        const float r = std::hypot(v.x, v.z) / radius;
        e.lo = std::min(e.lo, r);
        e.hi = std::max(e.hi, r);
    }
    return e;
}

}  // namespace

TEST(FlowMovieBuildParts, TheDiscMeshIsContiguousRingsWhoseAlphaFollowsTheSolidityContinuouslyWithNoAngularStructure) {
    const flow_movie::BladeChord chord = uiuc_10x5();
    constexpr uint32_t kSegments = 64;
    const float radius = 0.127f, plane_y = 0.011f, tint = 0.5f, max_step = 0.02f;
    for (const float opacity : {1.0f, 6.0f}) {
        const spade::render::MeshData mesh = flow_movie::rotor_disc_mesh(chord, radius, plane_y, opacity, tint, max_step, kSegments);
        const std::size_t rings = mesh.source_materials.size();
        ASSERT_EQ(mesh.submesh_index_count.size(), rings) << "one material, one alpha, per ring";
        EXPECT_GE(rings, 20u) << "the rings are as many as the alpha's change needs: opacity " << opacity;
        EXPECT_LE(rings, 400u);
        EXPECT_NEAR(ring_extent(mesh, 0u, radius).lo, chord.hub_cutout, 1e-4f) << "from the hub cutout";
        EXPECT_NEAR(ring_extent(mesh, rings - 1u, radius).hi, 1.0f, 1e-4f) << "to the tip";
        for (std::size_t i = 0; i < rings; ++i) {
            const RingExtent e = ring_extent(mesh, i, radius);
            if (i + 1u < rings) {
                EXPECT_NEAR(ring_extent(mesh, i + 1u, radius).lo, e.hi, 1e-4f) << "no gap and no overlap at ring " << i;
                // CONTINUOUS: the alpha of one ring and the next differ by no more than the step asked for (no step at the edge)
                EXPECT_LE(std::fabs(mesh.source_materials[i + 1u].base_color.a - mesh.source_materials[i].base_color.a), 1.5f * max_step)
                    << "a step at the edge of ring " << i << " (opacity " << opacity << ")";
            }
            const float mid = 0.5f * (e.lo + e.hi);
            EXPECT_NEAR(mesh.source_materials[i].base_color.a, std::min(0.5f, std::min(1.0f, flow_movie::blade_solidity(chord, mid) * opacity)), 2e-3f) << "ring " << i;
            EXPECT_NEAR(mesh.source_materials[i].base_color.r, tint, 1e-5f) << "the tint, not a blade's black";
            EXPECT_EQ(mesh.source_materials[i].shading, 1u) << "unlit: the drone's key light is not a light on a drawn average (at 2.5 times it turned the disc roots white)";
            EXPECT_EQ(mesh.submesh_index_count[i], kSegments * 2u * 3u * 2u) << "two triangles a segment, both windings";
        }
        // three radii of the first ring's neighbourhood: flat at the blade plane, evenly spaced round (no angular structure)
        for (const std::size_t ring : {std::size_t{0}, rings / 2u, rings - 1u}) {
            const uint32_t first = mesh.submesh_first_index[ring];
            for (uint32_t i = first; i < first + mesh.submesh_index_count[ring]; ++i) {
                const glm::vec3 v = mesh.positions[mesh.indices[i]];
                EXPECT_FLOAT_EQ(v.y, plane_y);
                const double steps = std::atan2(static_cast<double>(v.z), static_cast<double>(v.x)) / kTwoPi * kSegments;
                EXPECT_NEAR(steps, std::round(steps), 2e-3) << "ring " << ring;
            }
        }
        EXPECT_GT(mesh.source_materials.front().base_color.a, mesh.source_materials.back().base_color.a) << "denser at the root than at the tip";
    }
}

// THE DISC NEVER COVERS MORE THAN HALF (the lead, 2026-11-18, L100: under a near-opaque root the black anodised mounts read as white arms, which
// misstates the hardware): alpha = min(current, 0.5) with the ring's colour unchanged, so the disc still reads and the parts under it show through.
TEST(FlowMovieBuildParts, TheDiscIsNeverMoreThanHalfOpaqueSoThePartsUnderItShowThrough) {
    const flow_movie::BladeChord chord = uiuc_10x5();
    for (const float opacity : {1.0f, 5.0f, 10.0f}) {
        const spade::render::MeshData mesh = flow_movie::rotor_disc_mesh(chord, 0.127f, 0.0f, opacity, 0.65f, 0.01f, 32u);
        float most = 0.0f;
        for (const auto& material : mesh.source_materials) {
            EXPECT_LE(material.base_color.a, 0.5f + 1e-6f) << "opacity " << opacity;
            EXPECT_FLOAT_EQ(material.base_color.r, 0.65f) << "the ring's colour is not changed by the cap";
            most = std::max(most, material.base_color.a);
        }
        if (opacity >= 5.0f) EXPECT_NEAR(most, 0.5f, 1e-6f) << "the root reaches the cap at a gain of " << opacity;
        else EXPECT_LT(most, 0.5f) << "the physical average (1) is under the cap everywhere";
    }
}

TEST(FlowMovieBuildParts, ATurningRotorWithAChordTableIsOneSeeThroughDiscNotACopyPerSample) {
    const fs::path store = fresh_store("disc");
    save_rotor_gltf(store / kSha / "rotor.gltf", 0.127f);
    const std::string entry =
        "  - name: rotor_fr\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n    radius: 0.127\n"
        "    pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: rotor.gltf}\n";
    const auto parts = flow_movie::parse_build_parts("parts:\n" + entry);
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    const std::array<double, 1> omega{343.667};
    flow_movie::BuildDraw disc;  // the default blur is the disc
    disc.omega = omega;
    EXPECT_FALSE(disc.blur.sampled);
    {  // a turning rotor with no chord table: refused, saying what is missing (never the sampled blur by stealth)
        spade::render::RenderScene scene;
        const auto refused = flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), disc);
        ASSERT_FALSE(refused.has_value());
        EXPECT_NE(refused.error().context.find("chord"), std::string::npos) << refused.error().context;
        EXPECT_NE(refused.error().context.find("--lines"), std::string::npos) << refused.error().context;
        EXPECT_TRUE(scene.statics.empty()) << "left as it was";
    }
    disc.chord = uiuc_10x5();
    spade::render::RenderScene scene;
    const auto added = flow_movie::add_build_parts(scene, *parts, store, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), disc);
    ASSERT_TRUE(added.has_value()) << added.error().context;
    std::size_t see_through = 0, solid = 0;
    for (const auto& item : scene.statics) (item.see_through ? see_through : solid) += 1u;
    EXPECT_EQ(see_through, 1u) << "one disc";
    EXPECT_EQ(solid, 1u) << "the bell stays solid; the blade primitive is the disc now";
    EXPECT_NE(flow_movie::parse_prop_blur("mode=sampled").value().sampled, flow_movie::parse_prop_blur("mode=disc").value().sampled);
    EXPECT_FALSE(flow_movie::parse_prop_blur("mode=smear").has_value());
    EXPECT_FALSE(flow_movie::parse_prop_blur("opacity=0").has_value());
    EXPECT_GT(flow_movie::PropBlur{}.opacity, 1.0f) << "the look default is a gain: the physical average (1) is nearly invisible over the cloud";
    // the opacity is a GAIN on the solidity (a real prop over one shutter is faint; the look may want it stronger), alpha capped at 1 in the disc
    ASSERT_TRUE(flow_movie::parse_prop_blur("opacity=3").has_value());
    EXPECT_FLOAT_EQ(flow_movie::parse_prop_blur("opacity=3").value().opacity, 3.0f);
    EXPECT_FALSE(flow_movie::parse_prop_blur("opacity=11").has_value());
    fs::remove_all(store);
}

TEST(FlowMovieBuildParts, TheDiscRastersWithNoAngularStructureAndBrighterWhereTheBladeIsDenser) {
    const fs::path store = fresh_store("disc_raster");
    save_rotor_gltf(store / kSha / "rotor.gltf", 0.127f);
    const std::string entry =
        "  - name: rotor_fr\n    kind: rotor\n    mass_kg: 0.02\n    rotor: 0\n    spin: 1\n    radius: 0.127\n"
        "    pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}\n"
        "    objects:\n      - store_ref: {sha256: " + kSha + ", file: rotor.gltf}\n";
    flow_movie::BuildDraw disc;
    const std::array<double, 1> omega{343.667};
    disc.omega = omega;
    disc.chord = uiuc_10x5();
    disc.blur.opacity = 1.0f;
    const TopView view = top_view(entry, disc, store);
    ASSERT_FALSE(view.luma.empty());
    const float inner = 0.35f, middle = 0.6f, outer = 0.9f;  // fractions of the tip radius
    double means[3] = {0.0, 0.0, 0.0};
    const float fractions[3] = {inner, middle, outer};
    for (int k = 0; k < 3; ++k) {
        float lo = 1e9f, hi = -1e9f;
        for (int a = 0; a < 16; ++a) {
            const float l = luma_at(view, fractions[k], kTwoPi * a / 16.0 + 0.1);
            lo = std::min(lo, l);
            hi = std::max(hi, l);
            means[k] += l / 16.0;
        }
        EXPECT_LE(hi - lo, 0.10f * static_cast<float>(means[k]) + 2.0f) << "no angular structure at " << fractions[k] << " of the radius: " << lo << ".." << hi;
        EXPECT_GT(means[k], 2.0) << "the disc is there at " << fractions[k];
    }
    EXPECT_GT(means[0], 1.15 * means[1]) << "denser (brighter) nearer the root";
    EXPECT_GT(means[1], 1.15 * means[2]) << "and denser in the middle than near the tip";
    fs::remove_all(store);
}

// THE 6C'S PCB SEEN THROUGH ITS VENTS (the lead, 2026-11-18): the case's top has 17 real slots and the PCB below shows in them, but the renderer has no
// occlusion, so a flat bright pcb green reads as outside light. The interior is drawn in shadow: a near-black green preset, chosen.
TEST(FlowMovieBuildParts, ThePcbInShadowIsANearBlackGreenBelowThePcbGreen) {
    const auto shadow = flow_movie::real_material("pcb_in_shadow");
    ASSERT_TRUE(shadow.has_value());
    const auto green = flow_movie::real_material("pcb_green");
    ASSERT_TRUE(green.has_value());
    EXPECT_LT(shadow->base_color.g, 0.5f * green->base_color.g);
    EXPECT_LT(shadow->base_color.r, 0.03f);
    EXPECT_LT(shadow->base_color.g, 0.06f);
    EXPECT_GT(shadow->base_color.g, shadow->base_color.r) << "still green";
    std::ifstream in(fs::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500_build/parts.yaml", std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto table = flow_movie::parse_build_parts(text);
    ASSERT_TRUE(table.has_value()) << table.error().context;
    bool seen = false;
    for (const auto& part : *table) {
        if (part.name != "pixhawk_6c") continue;
        for (const auto& object : part.objects) {
            if (object.file == "PCB-PIXHAWK6C-F1.spmesh") {
                seen = true;
                EXPECT_EQ(object.material, "pcb_in_shadow");
            }
        }
    }
    EXPECT_TRUE(seen) << "the 6C's PCB object is in the table";
}

// THE --lines FILE TO THE DISC (the lead: main.cpp's chord wiring was exercised only by stills): the chord a draw gets is a pure choice now,
// and the disc it makes follows THAT file's chord table.
TEST(FlowMovieBuildParts, TheChordADrawGetsComesFromTheLinesFileOnlyForADiscWithParts) {
    const fs::path lines = fs::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500/parts/x500_10in_apc10x5.prop.yaml";
    flow_movie::PropBlur disc;
    const auto got = flow_movie::chord_for_draw(disc, true, lines);
    ASSERT_TRUE(got.has_value()) << got.error().context;
    ASSERT_TRUE(got->has_value());
    EXPECT_EQ((*got)->blades, 2u);
    EXPECT_FLOAT_EQ((*got)->c_over_r[7], 0.194f);
    flow_movie::PropBlur sampled;
    sampled.sampled = true;
    const auto none_for_sampled = flow_movie::chord_for_draw(sampled, true, lines);
    ASSERT_TRUE(none_for_sampled.has_value());
    EXPECT_FALSE(none_for_sampled->has_value()) << "the sampled blur needs no chord";
    const auto none_without_parts = flow_movie::chord_for_draw(disc, false, lines);
    ASSERT_TRUE(none_without_parts.has_value());
    EXPECT_FALSE(none_without_parts->has_value());
    const auto none_without_lines = flow_movie::chord_for_draw(disc, true, fs::path());
    ASSERT_TRUE(none_without_lines.has_value());
    EXPECT_FALSE(none_without_lines->has_value()) << "no --lines: no chord (a turning rotor is then refused, naming --lines)";
    const auto unreadable = flow_movie::chord_for_draw(disc, true, fs::path(SPADE_REPO_DIR) / "no/such.prop.yaml");
    ASSERT_FALSE(unreadable.has_value());
    EXPECT_NE(unreadable.error().context.find("--lines"), std::string::npos) << unreadable.error().context;

    // the disc of that file's chord: its ring alphas are that file's solidity (the real table's, not the test's)
    const flow_movie::BladeChord chord = **got;
    const spade::render::MeshData mesh = flow_movie::rotor_disc_mesh(chord, 0.127f, 0.0f, 1.0f, 0.5f, 0.02f, 32u);
    for (const std::size_t ring : {std::size_t{1}, mesh.source_materials.size() / 2u, mesh.source_materials.size() - 2u}) {
        const RingExtent e = ring_extent(mesh, ring, 0.127f);
        EXPECT_NEAR(mesh.source_materials[ring].base_color.a, flow_movie::blade_solidity(chord, 0.5f * (e.lo + e.hi)), 2e-3f) << ring;
    }
}

// THE CLIP RECORD NAMES WHAT THE PICTURE DOES NOT SHOW AS IT IS (the lead, 2026-11-18): the rotor discs are drawn at a gain, so the record says so.
TEST(FlowMovieBuildParts, TheRecordSaysHowTheRotorDiscsAreDrawn) {
    flow_movie::PropBlur gain;  // the defaults: an UNLIT disc at a gain of 5, never more than half opaque (the lead, 2026-11-18, the kit stills)
    EXPECT_FLOAT_EQ(gain.opacity, 5.0f);
    EXPECT_EQ(flow_movie::prop_blur_note(gain),
              "rotor discs drawn unlit at 5 times the blades' time-averaged coverage, no more than half opaque, so they show over the air and the parts under them show through");
    flow_movie::PropBlur exact;
    exact.opacity = 1.0f;
    EXPECT_EQ(flow_movie::prop_blur_note(exact), "rotor discs drawn unlit at the blades' time-averaged coverage, no more than half opaque");
    flow_movie::PropBlur sampled;
    sampled.sampled = true;
    sampled.samples = 24u;
    const std::string copies = flow_movie::prop_blur_note(sampled);
    EXPECT_NE(copies.find("24 copies"), std::string::npos) << copies;
    EXPECT_EQ(copies.find("times"), std::string::npos) << "no gain is claimed for the sampled blur: " << copies;
}

// CORE'S KIT TABLE (653fcdf9, L88s) has four `kind: motor` parts, the kit's motors, with their own material name (aluminium_anodised). The loader
// refused them as an unknown kind (flow_movie: "part 'motor_front_right': unknown kind 'motor'"), so no frame of the X500 could be drawn.
TEST(FlowMovieBuildParts, ACoreMotorIsAKnownKindWithAColourAndItsOwnMaterialName) {
    const std::string entry =
        "  - name: motor_front_right\n    kind: motor\n    mass_kg: 0.055\n    material: aluminium_anodised\n"
        "    pose: {position: [0.1774, 0.0, 0.1774], orientation: [1, 0, 0, 0]}\n    box: [0.029, 0.04, 0.029]\n";
    const auto parts = flow_movie::parse_build_parts(table(entry));
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    ASSERT_EQ(parts->size(), 1u);
    EXPECT_EQ((*parts)[0].kind, "motor");
    EXPECT_TRUE(flow_movie::kind_material("motor").has_value()) << "a colour for a motor that names no material";
    EXPECT_TRUE(flow_movie::real_material("aluminium_anodised").has_value());
}

TEST(FlowMovieBuildParts, TheWholeX500TableParsesAndHasItsFourMotors) {
    std::ifstream in(fs::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500_build/parts.yaml", std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto parts = flow_movie::parse_build_parts(text);
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    std::size_t motors = 0, rotors = 0;
    for (const auto& part : *parts) {
        motors += part.kind == "motor" ? 1u : 0u;
        rotors += part.kind == "rotor" ? 1u : 0u;
    }
    EXPECT_EQ(motors, 4u);
    EXPECT_EQ(rotors, 4u);
}

// THE DRONE'S LIGHT (the lead, 2026-11-18, the kit stills: "a soft rim or key light so the carbon plates and arm tubes read against the black,
// without lifting the background"; a presentation choice, named in the clip record). The air is not lit (a volume), and the web scene's sky is
// black, so the scene's one sun and its ambient light only the drone: --drone-light key=K,ambient=A sets them to K and A times the standard.
TEST(FlowMovieBuildParts, TheDroneLightIsAKeyAndAFillStrengthAndNothingElse) {
    const auto none = flow_movie::parse_drone_light("");
    ASSERT_TRUE(none.has_value()) << none.error().context;
    EXPECT_FLOAT_EQ(none->key, 1.0f);
    EXPECT_FLOAT_EQ(none->ambient, 1.0f);
    EXPECT_FALSE(none->on()) << "the standard light is not a choice to name";
    const auto got = flow_movie::parse_drone_light("key=2.5,ambient=1.5");
    ASSERT_TRUE(got.has_value()) << got.error().context;
    EXPECT_FLOAT_EQ(got->key, 2.5f);
    EXPECT_FLOAT_EQ(got->ambient, 1.5f);
    EXPECT_TRUE(got->on());
    const auto key_only = flow_movie::parse_drone_light("key=3");
    ASSERT_TRUE(key_only.has_value());
    EXPECT_FLOAT_EQ(key_only->ambient, 1.0f) << "what is not named keeps the standard";
    for (const char* bad : {"key=0", "key=21", "ambient=-1", "key=abc", "key", "glow=2", "key=2,,ambient=1", "key=", "key=inf"}) {
        const auto refused = flow_movie::parse_drone_light(bad);
        ASSERT_FALSE(refused.has_value()) << bad;
        EXPECT_NE(refused.error().context.find("drone light"), std::string::npos) << bad << ": " << refused.error().context;
    }
}

TEST(FlowMovieBuildParts, TheDroneLightSetsTheLightingToAMultipleOfTheStandardNotAFurtherMultiple) {
    const spade::render::Lighting standard;
    spade::render::RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    flow_movie::apply_drone_light(scene, flow_movie::DroneLight{.key = 2.5f, .ambient = 1.5f});
    EXPECT_FLOAT_EQ(scene.lighting.sun_intensity, standard.sun_intensity * 2.5f);
    EXPECT_FLOAT_EQ(scene.lighting.ambient_color.x, standard.ambient_color.x * 1.5f);
    EXPECT_FLOAT_EQ(scene.lighting.ambient_color.y, standard.ambient_color.y * 1.5f);
    EXPECT_FLOAT_EQ(scene.lighting.ambient_color.z, standard.ambient_color.z * 1.5f);
    EXPECT_EQ(scene.lighting.sun_direction, standard.sun_direction) << "the key's direction is not changed";
    EXPECT_EQ(scene.lighting.sky_zenith, glm::vec3(0.0f)) << "the background stays black";
    EXPECT_EQ(scene.lighting.sky_horizon, glm::vec3(0.0f));
    flow_movie::apply_drone_light(scene, flow_movie::DroneLight{.key = 2.5f, .ambient = 1.5f});
    EXPECT_FLOAT_EQ(scene.lighting.sun_intensity, standard.sun_intensity * 2.5f) << "every frame's scene is decorated: the same call twice is the same light";
    spade::render::RenderScene plain;
    const spade::render::Lighting before = plain.lighting;
    flow_movie::apply_drone_light(plain, flow_movie::DroneLight{});
    EXPECT_FLOAT_EQ(plain.lighting.sun_intensity, before.sun_intensity);
    EXPECT_EQ(plain.lighting.ambient_color, before.ambient_color) << "the standard light changes nothing";
}

TEST(FlowMovieBuildParts, TheRecordSaysHowTheDroneIsLit) {
    EXPECT_EQ(flow_movie::drone_light_note(flow_movie::DroneLight{}), "");
    EXPECT_EQ(flow_movie::drone_light_note(flow_movie::DroneLight{.key = 2.5f, .ambient = 1.5f}),
              "the drone is drawn under a key light at 2.5 times and a fill light at 1.5 times the standard lighting so its dark carbon reads against the black; the air is not lit by them");
}


// L106's dry run: S1 (the streamline view) seeds its lines at the rotors from the held body's pose, which a view's decorate() records as it draws. A web
// film that draws the real parts (--build-parts) REPLACES the case film's decorate(), and the replacement never recorded the pose, so S1 refused with
// "the streamline view ran before a frame's held body was read" on a frame whose log was whole. The pose is now read from the frame's log by one function
// the draw paths call before any decorate(), so no replacement can forget it. The log is the held frame's own: x, y, z, then w, x, y, z, then the four
// rotors' speeds and the lines' thrust (Physics' 10-inch X500 frame 200).
TEST(FlowMovieBuildParts, TheHeldPoseIsReadFromAFramesLogWhateverDecorateDoes) {
    const std::vector<double> live = {0.0009, -0.0048, -0.0016, 1.0, 0.0, 0.0, 0.0, 568.2145, 568.2145, 568.2145, 568.2145, 16.0371};
    const flow_movie::HeldPose pose = flow_movie::held_pose_from_live(live);
    ASSERT_TRUE(pose.known);
    EXPECT_FLOAT_EQ(pose.pos.x, 0.0009f);
    EXPECT_FLOAT_EQ(pose.pos.y, -0.0048f);
    EXPECT_FLOAT_EQ(pose.pos.z, -0.0016f);
    EXPECT_FLOAT_EQ(pose.orient.w, 1.0f);
    EXPECT_FLOAT_EQ(pose.orient.x, 0.0f);
    // fewer than 7 doubles is no held body: not known, and the pose is the default
    const flow_movie::HeldPose none = flow_movie::held_pose_from_live(std::vector<double>{1.0, 2.0, 3.0});
    EXPECT_FALSE(none.known);
    EXPECT_FALSE(flow_movie::held_pose_from_live(std::vector<double>{}).known);
    // exactly 7 is enough: the body alone, no rotor speeds
    EXPECT_TRUE(flow_movie::held_pose_from_live(std::vector<double>{0, 0, 0, 1, 0, 0, 0}).known);
}

// THE OpenFrame-5F's GENERATED PROPS: the drawn blade and the actuator line coincide (O138). The kit's rotor entries take the library's generated three-blade meshes
// (assets/library/drones/fpv_5inch/prop_ccw.spmesh and prop_cw.spmesh, UIUC chord and twist), the disc in the mesh's own XZ plane, thrust +Y. The line puts blade 0 at
// phi = s psi, direction (cos phi, 0, -sin phi), so at psi 0 blade 0 must lie along the mesh's +X, and the other two 120 degrees round (azimuth phi = atan2(-z, x)): this pins
// the MESHES' geometry for both hands, which the transform test above (a tetrahedron's +X) cannot. Also the disc is flat (the thickness a few mm) and the tips reach the
// simulated radius 0.0635.
TEST(FlowMovieBuildParts, TheGeneratedFpvPropMeshesHaveBladeZeroAlongPlusXForBothHands) {
    for (const char* name : {"prop_ccw.spmesh", "prop_cw.spmesh"}) {
        const fs::path file = fs::path(SPADE_ASSETS_DIR) / "library/drones/fpv_5inch" / name;
        auto loaded = spade::load_mesh_file(file);
        ASSERT_TRUE(loaded.has_value()) << name << ": " << (loaded ? "" : loaded.error().context);
        const spade::render::MeshData mesh = spade::render::flat_mesh(*loaded->mesh);
        ASSERT_FALSE(mesh.positions.empty()) << name;
        double r_max = 0.0, y_lo = 1e9, y_hi = -1e9;
        for (const glm::vec3& p : mesh.positions) {
            r_max = std::max(r_max, std::hypot(static_cast<double>(p.x), static_cast<double>(p.z)));
            y_lo = std::min(y_lo, static_cast<double>(p.y));
            y_hi = std::max(y_hi, static_cast<double>(p.y));
        }
        EXPECT_NEAR(r_max, 0.0635, 1.0e-3) << name << ": the tips reach the simulated rotor radius";
        EXPECT_LT(y_hi - y_lo, 0.012) << name << ": a flat disc, thrust along +Y";
        // the azimuth of each tip vertex (radius above 95% of the reach), folded to [0, 2 pi)
        std::vector<double> tips;
        for (const glm::vec3& p : mesh.positions) {
            if (std::hypot(static_cast<double>(p.x), static_cast<double>(p.z)) < 0.95 * r_max) continue;
            double a = std::atan2(-static_cast<double>(p.z), static_cast<double>(p.x));
            if (a < 0.0) a += kTwoPi;
            tips.push_back(a);
        }
        ASSERT_FALSE(tips.empty()) << name;
        for (int b = 0; b < 3; ++b) {
            const double want = kTwoPi * static_cast<double>(b) / 3.0;
            double nearest = 1e9;
            for (double a : tips) {
                double d = std::fabs(a - want);
                d = std::min(d, kTwoPi - d);
                nearest = std::min(nearest, d);
            }
            EXPECT_LT(nearest, 0.12) << name << ": a tip within 7 degrees of blade " << b << "'s azimuth " << want << " rad";
        }
    }
}

// THE LOOK SENTENCE NAMES THE DRONE'S OWN BLADES (Spade lead, from the posted kit film's caption, 2026-11-21): the clip record's "Look:" sentence for --prop-blur mode=blades said "the rotors' real blades"
// for every drone, but the kit's blades are the generated three-blade stand-in. prop_blur_note takes the words for the blades (the default stays the X500's "the rotors' real blades").
TEST(FlowMovieBuildParts, ThePropBlurNoteNamesTheBladesTheDroneDraws) {
    const auto blades = flow_movie::parse_prop_blur("mode=blades");
    ASSERT_TRUE(blades.has_value());
    EXPECT_EQ(flow_movie::prop_blur_note(*blades), "the rotors' real blades drawn opaque at the simulation's rotor angle, with no blur");
    const std::string kit = flow_movie::prop_blur_note(*blades, "the stand-in's generated blades");
    EXPECT_EQ(kit, "the stand-in's generated blades drawn opaque at the simulation's rotor angle, with no blur");
    EXPECT_EQ(kit.find("real"), std::string::npos) << kit;
    // the words are for the blades mode only: the disc and the sampled blur say what they always said
    const auto disc = flow_movie::parse_prop_blur("opacity=0.5");
    ASSERT_TRUE(disc.has_value());
    EXPECT_EQ(flow_movie::prop_blur_note(*disc, "the stand-in's generated blades").find("generated"), std::string::npos);
}
