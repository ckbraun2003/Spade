// A vehicle drawn from its parts (sandbox/vehicle_draw.hpp's
// append_vehicle_parts): N12, the end-goal design §2(a). The editor draws a
// model with parts through render_parts, posed where the vehicle is, so the
// drone on screen is the drone that flies. Display-free.
//
// Rendering's guidance (2026-10-19): bind_parts() writes statics, so the
// editor uploads once per composition and, each frame, binds into a copy of
// the base scene. That holds only while every frame binds the SAME meshes;
// a CSG-root piece marches at the pose it was uploaded with, so a model with
// one is uploaded every frame (has_csg_pieces()).

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "reference_airframe.hpp"
#include "render/scene.hpp"
#include "render_parts/render_parts.hpp"
#include "scene/model_file.hpp"
#include "sim/design_frame.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/part_pose.hpp"
#include "world/sdf.hpp"

#include "../sandbox/vehicle_draw.hpp"

namespace vh = spade::vehicles;
namespace rp = spade::render_parts;
using spade::sandbox::append_vehicle_parts;
using spade::sandbox::has_csg_pieces;

namespace {

// The reference quad, compiled with its parts.
void reference_quad(vh::ModelType& out) {
    const auto compiled = vh::compile_airframe(vh::fixtures::reference_quad());
    ASSERT_TRUE(compiled.has_value()) << compiled.error().context;
    out = compiled->model;
    ASSERT_FALSE(out.parts.empty());
}

// A scene with a three-material palette, as the editor's has.
spade::render::RenderScene palette_scene() {
    spade::render::RenderScene s;
    s.materials.resize(3);
    return s;
}

rp::PartsLook look() {
    rp::PartsLook l;
    l.body = 0;
    l.rotor = 1;
    l.porous = 2;
    return l;
}

std::size_t piece_count(const rp::PreparedParts& prepared) {
    std::size_t n = 0;
    for (const rp::PreparedPart& p : prepared.parts) n += p.pieces.size();
    return n;
}

}  // namespace

TEST(SandboxVehicleParts, TheReferenceQuadDrawsOneItemPerPiece) {
    vh::ModelType model;
    ASSERT_NO_FATAL_FAILURE(reference_quad(model));
    const auto prepared = rp::prepare_parts(model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    spade::render::RenderScene scene = palette_scene();
    const spade::Result<void> bound = append_vehicle_parts(scene, *prepared, model, glm::vec3(0.0f, 0.5f, 0.0f),
                                                           glm::quat(1.0f, 0.0f, 0.0f, 0.0f), look());
    ASSERT_TRUE(bound.has_value()) << bound.error().context;
    EXPECT_EQ(scene.statics.size(), piece_count(*prepared));
    EXPECT_EQ(scene.meshes.size(), piece_count(*prepared));
}

// Every item sits at the vehicle's pose times where it sits on a vehicle at
// the origin: the parts move with the vehicle, rigidly.
TEST(SandboxVehicleParts, ThePartsSitAtTheVehiclesPose) {
    vh::ModelType model;
    ASSERT_NO_FATAL_FAILURE(reference_quad(model));
    const auto prepared = rp::prepare_parts(model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    spade::render::RenderScene home = palette_scene();
    spade::render::RenderScene moved = palette_scene();
    const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::vec3 pos(1.5f, 0.75f, -2.0f);
    const glm::quat orient = glm::angleAxis(0.6f, glm::normalize(glm::vec3(0.2f, 1.0f, 0.1f)));
    ASSERT_TRUE(append_vehicle_parts(home, *prepared, model, glm::vec3(0.0f), identity, look()).has_value());
    ASSERT_TRUE(append_vehicle_parts(moved, *prepared, model, pos, orient, look()).has_value());
    ASSERT_EQ(home.statics.size(), moved.statics.size());
    const glm::mat4 pose = glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(orient);
    for (std::size_t i = 0; i < home.statics.size(); ++i) {
        const glm::mat4 want = pose * home.statics[i].local_to_world;
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) {
                EXPECT_NEAR(moved.statics[i].local_to_world[c][r], want[c][r], 1e-5f) << "item " << i;
            }
        }
    }
}

// The pose given is the vehicle's DESIGN frame (a scene's start pose, a run's
// state_of()), and the parts are posed by the body it makes: the centre of
// mass sits com_offset from the design origin (the reference quad's 6.4 mm
// below it). So drawing at a design pose draws what the engine's own path
// draws for to_body_state() of it, the rule the sim spawns a vehicle by.
TEST(SandboxVehicleParts, TheDesignPoseIsPosedAsTheBodyItMakes) {
    vh::ModelType model;
    ASSERT_NO_FATAL_FAILURE(reference_quad(model));
    ASSERT_GT(glm::length(model.com_offset), 1e-3f) << "the premise: a centre of mass off the design origin";
    const auto prepared = rp::prepare_parts(model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    const glm::vec3 pos(1.5f, 0.75f, -2.0f);
    const glm::quat orient = glm::angleAxis(0.6f, glm::normalize(glm::vec3(0.2f, 1.0f, 0.1f)));

    spade::render::RenderScene drawn = palette_scene();
    ASSERT_TRUE(append_vehicle_parts(drawn, *prepared, model, pos, orient, look()).has_value());

    const spade::FrameState design{pos, orient, glm::vec3(0.0f), glm::vec3(0.0f)};
    const spade::FrameState b = spade::to_body_state(design, glm::normalize(model.design_to_principal), model.com_offset);
    spade::BodyState body{};
    body.pos = b.pos;
    body.orient = b.orient;
    const std::vector<float> still(model.rotors.size(), 0.0f);
    const auto posed = vh::posed_parts(model, body, still, still, vh::PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    spade::render::RenderScene want = palette_scene();
    ASSERT_TRUE(rp::bind_parts(*prepared, *posed, look(), want).has_value());

    ASSERT_EQ(drawn.statics.size(), want.statics.size());
    for (std::size_t i = 0; i < want.statics.size(); ++i) {
        const glm::vec3 at(drawn.statics[i].local_to_world[3]);
        const glm::vec3 should(want.statics[i].local_to_world[3]);
        EXPECT_LT(glm::length(at - should), 1e-5f)
            << "item " << i << " is " << glm::length(at - should) * 1000.0f << " mm from where the body puts it";
    }
}

// Rendering's precondition for uploading once: two frames at different poses
// bind the same meshes, count and content, so only the transforms change.
TEST(SandboxVehicleParts, TwoFramesBindTheSameMeshes) {
    vh::ModelType model;
    ASSERT_NO_FATAL_FAILURE(reference_quad(model));
    const auto prepared = rp::prepare_parts(model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    const spade::render::RenderScene base = palette_scene();
    spade::render::RenderScene a = base;
    spade::render::RenderScene b = base;
    ASSERT_TRUE(append_vehicle_parts(a, *prepared, model, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), look())
                    .has_value());
    ASSERT_TRUE(append_vehicle_parts(b, *prepared, model, glm::vec3(3.0f, 1.0f, 0.0f),
                                     glm::angleAxis(1.0f, glm::vec3(0.0f, 1.0f, 0.0f)), look())
                    .has_value());
    ASSERT_EQ(a.meshes.size(), b.meshes.size());
    for (std::size_t i = 0; i < a.meshes.size(); ++i) {
        EXPECT_EQ(a.meshes[i].positions.size(), b.meshes[i].positions.size()) << "mesh " << i;
        EXPECT_EQ(a.meshes[i].indices, b.meshes[i].indices) << "mesh " << i;
    }
}

TEST(SandboxVehicleParts, ARefusedBindAddsNothing) {
    vh::ModelType model;
    ASSERT_NO_FATAL_FAILURE(reference_quad(model));
    const auto prepared = rp::prepare_parts(model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    spade::render::RenderScene scene = palette_scene();
    rp::PartsLook bad = look();
    bad.body = 7;  // not in the palette
    const spade::Result<void> bound =
        append_vehicle_parts(scene, *prepared, model, glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), bad);
    ASSERT_FALSE(bound.has_value());
    EXPECT_TRUE(scene.statics.empty());
    EXPECT_TRUE(scene.meshes.empty());
}

// Rendering: the reference quad has no CSG roots, so it uploads once; a part
// whose shape is a CSG root (a box minus a box) has one, so its vehicle is
// uploaded every frame.
TEST(SandboxVehicleParts, OnlyACsgRootPartNeedsAnUploadEveryFrame) {
    vh::ModelType model;
    ASSERT_NO_FATAL_FAILURE(reference_quad(model));
    const auto prepared = rp::prepare_parts(model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    EXPECT_FALSE(has_csg_pieces(*prepared));

    rp::PreparedParts carved = *prepared;
    carved.parts.front().pieces.front().is_csg = true;
    EXPECT_TRUE(has_csg_pieces(carved));
}

// The props spin (L7c2, Core's posing rule): given its rotors' omegas and
// blade azimuths, a vehicle's parts are bound as posed_parts() poses them at
// that omega and psi, and a turned prop sits elsewhere than a still one. The
// library's Phantom, whose props are rotor parts (L7c1).
TEST(SandboxVehicleParts, AVehiclesPropsArePosedByTheirAzimuth) {
    const spade::Result<spade::scene::ModelFile> file = spade::scene::load_model_file(
        std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "phantom3_class.model.yaml");
    ASSERT_TRUE(file.has_value()) << file.error().context;
    const vh::ModelType& model = file->model;
    ASSERT_FALSE(model.rotors.empty());
    const auto prepared = rp::prepare_parts(model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    const spade::BodyState body = spade::sandbox::body_at(
        model, glm::vec3(1.5f, 0.75f, -2.0f), glm::angleAxis(0.6f, glm::normalize(glm::vec3(0.2f, 1.0f, 0.1f))));
    const std::vector<float> omega(model.rotors.size(), 600.0f);
    std::vector<float> psi(model.rotors.size());
    for (std::size_t k = 0; k < psi.size(); ++k) psi[k] = 0.4f + 0.9f * static_cast<float>(k);

    spade::render::RenderScene drawn = palette_scene();
    ASSERT_TRUE(append_vehicle_parts(drawn, *prepared, model, body, omega, psi, look()).has_value());
    const auto posed = vh::posed_parts(model, body, omega, psi, vh::PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    spade::render::RenderScene want = palette_scene();
    ASSERT_TRUE(rp::bind_parts(*prepared, *posed, look(), want).has_value());
    ASSERT_EQ(drawn.statics.size(), want.statics.size());
    for (std::size_t i = 0; i < want.statics.size(); ++i) {
        EXPECT_TRUE(drawn.statics[i].local_to_world == want.statics[i].local_to_world) << "item " << i;
    }

    spade::render::RenderScene still = palette_scene();
    ASSERT_TRUE(append_vehicle_parts(still, *prepared, model, body, look()).has_value());
    ASSERT_EQ(still.statics.size(), drawn.statics.size());
    std::size_t turned = 0;
    for (std::size_t i = 0; i < still.statics.size(); ++i) {
        if (!(still.statics[i].local_to_world == drawn.statics[i].local_to_world)) ++turned;
    }
    EXPECT_GT(turned, 0u) << "no prop turned with its azimuth";

    spade::render::RenderScene empty = palette_scene();
    ASSERT_TRUE(append_vehicle_parts(empty, *prepared, model, body, {}, {}, look()).has_value());
    ASSERT_EQ(empty.statics.size(), still.statics.size());
    for (std::size_t i = 0; i < still.statics.size(); ++i) {
        EXPECT_TRUE(empty.statics[i].local_to_world == still.statics[i].local_to_world) << "empty spans pose it still";
    }
}

// What a still has to show for its props to be seen turned (the lead,
// 2026-10-27): a rotor counts when its azimuth is `at_least` or more from the
// authored pose, a prop of n blades repeating every 2 pi / n.
TEST(SandboxVehicleParts, APropIsTurnedWhenItsAzimuthIsFarFromTheAuthoredPoseModuloItsBlades) {
    constexpr float kDeg = 0.017453292519943295f;
    constexpr float kPi = 3.14159265358979f;
    const std::vector<float> psi = {0.0f, kPi, 0.5f, 2.0f, 3.0f};
    // 0 and pi are a two-blade prop's authored pose; 0.5 rad is 28.6 degrees
    // from it; 2.0 rad is 65 degrees; 3.0 rad is 8 degrees short of pi.
    EXPECT_EQ(spade::sandbox::props_turned(psi, 2, 25.0f * kDeg), 2u);
    EXPECT_EQ(spade::sandbox::props_turned(psi, 2, 30.0f * kDeg), 1u);
    EXPECT_EQ(spade::sandbox::props_turned(psi, 1, 25.0f * kDeg), 4u) << "one blade: pi is half a turn away";
    EXPECT_EQ(spade::sandbox::props_turned({}, 2, 25.0f * kDeg), 0u);
}

namespace {
// ---- the CSG upload gate -------------------------------------------------------------------------------------------------------

// The reference quad with one CSG part (a box minus a sphere) of the given half size on its body, so a vehicle of it has a CSG subtree.
vh::ModelType quad_with_ring(float half) {
    vh::ModelType model;
    reference_quad(model);
    spade::SdfProgram ring;
    ring.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode box{};
    box.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    box.params = glm::vec4(half, 0.03f, half, 0.0f);
    spade::SdfNode ball{};
    ball.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    ball.params = glm::vec4(0.045f, 0.0f, 0.0f, 0.0f);
    spade::SdfNode subtract{};
    subtract.op = static_cast<uint32_t>(spade::SdfOp::subtract);
    ring.nodes = {box, ball, subtract};
    model.parts = {vh::PartDesc{.name = "ring",
                                .role = "test",
                                .attach = vh::PartAttach::body,
                                .rotor = 0,
                                .mount_pos = glm::vec3(0.02f, 0.01f, -0.03f),
                                .mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                .shape = std::move(ring),
                                .surface = vh::PartSurface{.kind = vh::PartSurface::Kind::wall, .inv_permeability = 0.0f, .inertial_coeff = 0.0f},
                                .min_feature = 0.001f}};
    return model;
}

// A scene as the editor's draw_items() makes one each frame: the base palette, then each vehicle's parts bound where it is.
spade::render::RenderScene drawn(const std::vector<std::pair<vh::ModelType, glm::vec3>>& vehicles) {
    spade::render::RenderScene scene = palette_scene();
    for (const auto& [model, pos] : vehicles) {
        const auto prepared = rp::prepare_parts(model);
        EXPECT_TRUE(prepared.has_value()) << (prepared.has_value() ? "" : prepared.error().context);
        if (!prepared.has_value()) continue;
        EXPECT_TRUE(append_vehicle_parts(scene, *prepared, model, pos, glm::quat(1.0f, 0.0f, 0.0f, 0.0f), look()).has_value());
    }
    return scene;
}

}  // namespace

TEST(SandboxCsgUploadGate, AnUnchangedCsgSceneUploadsOnceOverManyFrames) {
    const vh::ModelType quad = quad_with_ring(0.06f);
    spade::sandbox::CsgUploadGate gate;
    int uploads = 0;
    for (int frame = 0; frame < 120; ++frame) {
        const spade::render::RenderScene scene = drawn({{quad, glm::vec3(0.0f)}});
        ASSERT_FALSE(scene.csg_subtrees.empty());
        if (gate.needs_upload(scene)) {
            gate.note_uploaded(scene);
            ++uploads;
        }
    }
    EXPECT_EQ(uploads, 1) << "the first frame uploads, the 119 that follow are the same scene";
}

TEST(SandboxCsgUploadGate, AMovedCsgVehicleUploadsAndSettlesAgain) {
    const vh::ModelType quad = quad_with_ring(0.06f);
    spade::sandbox::CsgUploadGate gate;
    gate.note_uploaded(drawn({{quad, glm::vec3(0.0f)}}));
    EXPECT_FALSE(gate.needs_upload(drawn({{quad, glm::vec3(0.0f)}})));
    const spade::render::RenderScene moved = drawn({{quad, glm::vec3(0.5f, 0.0f, 0.0f)}});
    EXPECT_TRUE(gate.needs_upload(moved)) << "a CSG piece marches at the pose it was uploaded with";
    EXPECT_EQ(gate.changed(moved), (std::vector<std::size_t>{0}));
    gate.note_uploaded(moved);
    EXPECT_FALSE(gate.needs_upload(drawn({{quad, glm::vec3(0.5f, 0.0f, 0.0f)}}))) << "it settles where it stopped";
}

TEST(SandboxCsgUploadGate, AnEditToOneCsgModelChangesThatModelsSubtreeOnly) {
    const vh::ModelType a = quad_with_ring(0.06f);
    const vh::ModelType b = quad_with_ring(0.06f);
    spade::sandbox::CsgUploadGate gate;
    gate.note_uploaded(drawn({{a, glm::vec3(0.0f)}, {b, glm::vec3(1.0f, 0.0f, 0.0f)}}));
    EXPECT_FALSE(gate.needs_upload(drawn({{a, glm::vec3(0.0f)}, {b, glm::vec3(1.0f, 0.0f, 0.0f)}})));
    const vh::ModelType edited = quad_with_ring(0.08f);  // the second vehicle's ring is edited
    const spade::render::RenderScene scene = drawn({{a, glm::vec3(0.0f)}, {edited, glm::vec3(1.0f, 0.0f, 0.0f)}});
    ASSERT_EQ(scene.csg_subtrees.size(), 2u);
    EXPECT_TRUE(gate.needs_upload(scene));
    EXPECT_EQ(gate.changed(scene), (std::vector<std::size_t>{1})) << "the first model's subtree is untouched";
}

TEST(SandboxCsgUploadGate, ASceneWithoutCsgNeverNeedsAnUploadAfterTheFirstAndACountChangeDoes) {
    vh::ModelType plain;
    reference_quad(plain);
    plain.parts.clear();
    spade::sandbox::CsgUploadGate gate;
    EXPECT_TRUE(gate.needs_upload(drawn({{plain, glm::vec3(0.0f)}}))) << "nothing noted yet";
    gate.note_uploaded(drawn({{plain, glm::vec3(0.0f)}}));
    for (int frame = 0; frame < 10; ++frame) {
        EXPECT_FALSE(gate.needs_upload(drawn({{plain, glm::vec3(0.1f * static_cast<float>(frame), 0.0f, 0.0f)}})));
    }
    const vh::ModelType ring = quad_with_ring(0.06f);
    EXPECT_TRUE(gate.needs_upload(drawn({{ring, glm::vec3(0.0f)}}))) << "a subtree that was not there";
    gate.note_uploaded(drawn({{ring, glm::vec3(0.0f)}}));
    EXPECT_TRUE(gate.needs_upload(drawn({{plain, glm::vec3(0.0f)}}))) << "a subtree that went";
}

