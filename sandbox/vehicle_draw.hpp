// How a vehicle of a composed scene draws: the scene files' sessions and the
// editor draw it the same way. Display-free.
//
// A vehicle whose model has parts draws them (append_vehicle_parts(), N12):
// render_parts posed where the vehicle is, so the drone on screen is the
// drone that flies. Otherwise a vehicle with rotors draws as a drone: a hull,
// an arm out to each hub, and a disc at each hub, from its model's rotor
// layout (drone_view.hpp's part sizes). The front arm and rotor take the nose
// colour. A vehicle without rotors is a ball of the given diameter, the
// contact proxy it collides as.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "drone_view.hpp"
#include "core/error.hpp"
#include "render/scene.hpp"
#include "render_parts/render_parts.hpp"
#include "sim/design_frame.hpp"
#include "state/layout.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/part_pose.hpp"

namespace spade::sandbox {

// The unit meshes (unit_meshes.hpp's box, sphere and cylinder) and the
// materials a vehicle draws with, as indices into the render scene.
struct VehicleLook {
    uint32_t box_mesh = 0;
    uint32_t sphere_mesh = 0;
    uint32_t disc_mesh = 0;
    uint32_t body_material = 0;   // the hull and the arms
    uint32_t nose_material = 0;   // the front arm and rotor
    uint32_t rotor_material = 0;
};

// Appends the draw items of one vehicle at `pos` and `orient` (world frame).
inline void append_vehicle_items(std::vector<render::DrawItem>& out, const VehicleLook& look,
                                 const vehicles::ModelType& model, glm::vec3 pos, glm::quat orient,
                                 float ball_diameter, uint32_t ball_material) {
    const glm::mat4 pose = glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(orient);
    if (model.rotors.empty()) {
        render::DrawItem item;
        item.mesh_index = look.sphere_mesh;
        item.local_to_world = glm::scale(pose, glm::vec3(ball_diameter));
        item.material_override = ball_material;
        out.push_back(item);
        return;
    }
    render::DrawItem hull;
    hull.mesh_index = look.box_mesh;
    hull.local_to_world = glm::scale(pose, 2.0f * kDroneBodyHalf);
    hull.material_override = look.body_material;
    out.push_back(hull);
    for (std::size_t k = 0; k < model.rotors.size(); ++k) {
        const vehicles::RotorDesc& rotor = model.rotors[k];
        // The arm runs flat from the hull's centre to the hub; the disc sits on
        // it. The arm mesh is a unit box along +X; turn it about +Y onto this
        // arm (rotating +X by a about +Y gives (cos a, 0, -sin a)).
        const glm::vec3 flat(rotor.local_pos.x, 0.0f, rotor.local_pos.z);
        const float reach = glm::length(flat);
        const float a = std::atan2(-flat.z, flat.x);
        render::DrawItem arm;
        arm.mesh_index = look.box_mesh;
        arm.local_to_world = glm::scale(pose * glm::translate(glm::mat4(1.0f), 0.5f * flat) *
                                            glm::rotate(glm::mat4(1.0f), a, glm::vec3(0.0f, 1.0f, 0.0f)),
                                        glm::vec3(reach, 2.0f * kDroneArmHalfSection, 2.0f * kDroneArmHalfSection));
        arm.material_override = k == 0 ? look.nose_material : look.body_material;
        out.push_back(arm);
        render::DrawItem disc;
        disc.mesh_index = look.disc_mesh;
        disc.local_to_world =
            glm::scale(pose * glm::translate(glm::mat4(1.0f), rotor.local_pos + glm::vec3(0.0f, kDroneRotorLift, 0.0f)),
                       glm::vec3(2.0f * rotor.radius, 2.0f * kDroneRotorHalfThickness, 2.0f * rotor.radius));
        disc.material_override = k == 0 ? look.nose_material : look.rotor_material;
        out.push_back(disc);
    }
}

// The body row (principal axes, at the centre of mass) of a vehicle whose
// DESIGN frame is at (pos, orient): to_body_state(), the rule the sim spawns
// a vehicle by. The centre of mass sits com_offset from the design origin.
[[nodiscard]] inline BodyState body_at(const vehicles::ModelType& model, glm::vec3 pos, glm::quat orient) {
    const FrameState b = to_body_state(FrameState{pos, orient, glm::vec3(0.0f), glm::vec3(0.0f)},
                                       glm::normalize(model.design_to_principal), model.com_offset);
    BodyState body{};
    body.pos = b.pos;
    body.orient = b.orient;
    return body;
}

// Appends `model`'s parts, prepared once by render_parts::prepare_parts(),
// posed by `body` (a body row: a LiveRun's body(), or body_at()) and still:
// the rotor spans are zero, so a rotor part shows no spin. bind_parts()
// writes statics (a CsgSubtree indexes them), so a caller binds each frame
// into a copy of its base scene. All or nothing: whatever posed_parts() or
// bind_parts() refuse, nothing is appended.
[[nodiscard]] inline Result<void> append_vehicle_parts(render::RenderScene& scene,
                                                       const render_parts::PreparedParts& prepared,
                                                       const vehicles::ModelType& model, const BodyState& body,
                                                       const render_parts::PartsLook& look) {
    const std::vector<float> still(model.rotors.size(), 0.0f);
    const Result<vehicles::PosedParts> posed =
        vehicles::posed_parts(model, body, still, still, vehicles::PartFilter::all);
    if (!posed) return std::unexpected(posed.error());
    return render_parts::bind_parts(prepared, *posed, look, scene);
}

// The same, posed at the vehicle's DESIGN frame (pos, orient): a scene's
// start pose, a run's state_of().
[[nodiscard]] inline Result<void> append_vehicle_parts(render::RenderScene& scene,
                                                       const render_parts::PreparedParts& prepared,
                                                       const vehicles::ModelType& model, glm::vec3 pos,
                                                       glm::quat orient, const render_parts::PartsLook& look) {
    return append_vehicle_parts(scene, prepared, model, body_at(model, pos, orient), look);
}

// Whether any piece is a CSG root. Such a piece marches at the pose it was
// uploaded with (Rendering, 2026-10-19), so a GPU caller uploads a scene that
// holds one every frame, until renderers move CSG subtrees.
[[nodiscard]] inline bool has_csg_pieces(const render_parts::PreparedParts& prepared) {
    for (const render_parts::PreparedPart& part : prepared.parts) {
        for (const render_parts::PartPiece& piece : part.pieces) {
            if (piece.is_csg) return true;
        }
    }
    return false;
}

}  // namespace spade::sandbox
