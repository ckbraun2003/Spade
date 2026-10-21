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
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "reference_airframe.hpp"
#include "render/scene.hpp"
#include "render_parts/render_parts.hpp"
#include "vehicles/airframe_compile.hpp"
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
