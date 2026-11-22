#pragma once

// x500_contact_hull.hpp -- the X500's feet as a contact hull, in one place (O140 phase 1, item 4: docs/design/physics/plans/2026-11-21-ground-contact.md; O150 video
// #2). tests/test_x500_ground_contact.cpp builds the model with it in memory; tests/test_x500_contact_model.cpp writes it into
// tools/flow_movie/data/x500/x500_contact.model.yaml. Two copies of these numbers would be two places to be wrong.
//
// THE FEET. Four rubber pads (the build table's JIAO-EVA objects, measured in the design frame: centres (+-0.0675, -0.2058, +-0.12) m, 93 x 19 x 19 mm), three points
// a pad (both ends and the middle, so a pad cannot rest on a point pair), a skin radius of 9.5 mm: twelve points, the lowest of the drone at y = -0.2153 m in the design
// frame. Friction 0.6 and restitution 0.1 are ASSUMED (EVA foam on a hard floor). 16 sweeps (kMaxSweeps): the default 4 leaves a level 3.1 m/s drop spinning at 0.7 to
// 0.8 rad/s from an unconverged sequential-impulse solve over twelve points (test_x500_ground_contact.cpp's DiagnosticSpinAfterALevelDrop: 0.017 rad/s at 16).

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>

#include "physics/rigid_contact.hpp"
#include "vehicles/model_type.hpp"

namespace spade::test::x500 {

inline constexpr float kSkin = 0.0095f;
inline constexpr uint32_t kSweeps = 16;
inline constexpr float kFriction = 0.6f;
inline constexpr float kRestitution = 0.1f;

// The twelve points in the DESIGN frame: three along each pad's length (both ends of the 19 mm-section bar, and the middle).
[[nodiscard]] inline std::vector<glm::vec3> pad_points_design() {
    std::vector<glm::vec3> out;
    for (const float sx : {1.0f, -1.0f}) {
        for (const float sz : {1.0f, -1.0f}) {
            for (const float dx : {0.0305f, 0.0675f, 0.1045f}) out.emplace_back(sx * dx, -0.2058f, sz * 0.12f);
        }
    }
    return out;
}

// `model` standing on its feet: its `shape` the twelve points in the BODY (principal) frame, p_body = design_to_principal * (p_design - com_offset), with the skin, the
// friction and restitution given, and 16 sweeps. Nothing else of the model changes.
[[nodiscard]] inline vehicles::ModelType with_feet(vehicles::ModelType model, float friction = kFriction, float restitution = kRestitution) {
    std::vector<glm::vec3> body_points;
    for (const glm::vec3& p : pad_points_design()) body_points.push_back(model.design_to_principal * (p - model.com_offset));
    physics::ShapeDesc shape = physics::hull_shape(body_points, kSkin);
    shape.friction = friction;
    shape.restitution = restitution;
    shape.iterations = kSweeps;
    model.shape = shape;
    return model;
}

}  // namespace spade::test::x500
