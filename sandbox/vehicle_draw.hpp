// How a vehicle of a composed scene draws: the scene files' sessions and the
// editor draw it the same way. Display-free.
//
// A vehicle with rotors draws as a drone: a hull, an arm out to each hub, and
// a disc at each hub, from its model's rotor layout (drone_view.hpp's part
// sizes). The front arm and rotor take the nose colour. A vehicle without
// rotors is a ball of the given diameter, the contact proxy it collides as.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "drone_view.hpp"
#include "render/scene.hpp"
#include "vehicles/model_type.hpp"

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

}  // namespace spade::sandbox
