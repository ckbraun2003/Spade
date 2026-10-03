#pragma once

#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"

// ===========================================================================
// composite_inertia.hpp -- one rigid body from an airframe's parts
// (docs/design/physics/plans/2026-10-03-drone-builder-physics.md section 6).
//
// A HOST UTILITY, NOT A STEP FUNCTION. The builder calls it once per airframe;
// its outputs become the body's mass, inertia and the model type's
// design-to-body rotation.
//
// THE METHOD. All sums run in double, in part declaration order:
//
//   M = sum m_i        c = sum m_i p_i / M
//   I = sum (R_i I_i R_i^T + m_i (|d_i|^2 E - d_i d_i^T)),   d_i = p_i - c
//
// The engine stores a diagonal body-frame inertia, so I is diagonalized by a
// cyclic Jacobi method in double. It uses only + - * / and sqrt, which IEEE
// 754 rounds correctly, so the result is the same on every conforming
// platform (TD-3). A matrix that is already diagonal takes no rotation, so a
// symmetric airframe keeps its design axes EXACTLY and its rotation is the
// identity. Each principal axis takes the label of the design axis it lies
// nearest, signed so the frame is right-handed. Each output is rounded to
// float once, at the end.
//
// THE ROTATION STAYS INTERNAL. design_to_body maps design-frame vectors to
// body-frame vectors. The model type uses it to put every mount in the body
// frame; spawn and the vehicle-state read convert at the edge (spec section
// 6). A flight package never sees it.
// ===========================================================================

namespace spade::vehicles {

enum class PartShape {
    point,     // a point mass; `size` unused
    sphere,    // solid sphere; size.x = radius
    box,       // solid box; size = full extents along local x, y, z
    cylinder,  // solid cylinder along local +Y; size.x = radius, size.y = length
    tube,      // thin-walled tube along local +Y (an arm); size.x = radius, size.y = length
    tensor,    // an explicit inertia tensor about the part's own centre of mass, local frame
};

// One part, in the airframe's design frame (DBP-03: every part has a definite
// mount pose).
struct PartInertia {
    double mass = 0.0;                            // kg, > 0
    glm::dvec3 position{0.0};                     // centre of mass, design frame, m
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};   // local -> design; normalized here
    PartShape shape = PartShape::point;
    glm::dvec3 size{0.0};                         // m; meaning per shape, above
    glm::dmat3 tensor{0.0};                       // kg m^2, local frame; PartShape::tensor only
};

struct CompositeInertia {
    float mass = 0.0f;                            // kg
    glm::vec3 center_of_mass{0.0f};               // design frame, m
    glm::vec3 principal_moments{0.0f};            // kg m^2, about the body axes
    glm::quat design_to_body{1.0f, 0.0f, 0.0f, 0.0f};
    glm::dmat3 inertia_design{0.0};               // kg m^2 about c, design frame, double, for inspection
};

// One problem with one part: its index in the span, the field at fault and a
// message that reads after "part <index> ".
struct PartIssue {
    std::size_t index = 0;
    std::string field;
    std::string message;
};

// Every problem with every part, in part order then field order; empty when
// every part is valid. The same checks composite_inertia() applies, from the
// same function, so the two cannot disagree. The composite's own check (a
// principal moment that is not positive) needs the parts summed, so only
// composite_inertia() reports it.
[[nodiscard]] std::vector<PartIssue> composite_inertia_issues(std::span<const PartInertia> parts);

// The inertia of one primitive about its own centre of mass, in its local
// frame. PartShape::tensor returns `tensor` unchanged.
[[nodiscard]] glm::dmat3 part_inertia_local(const PartInertia& part) noexcept;

// The composite. invalid_argument, naming the part, for: no parts; a mass not
// positive and finite; a non-finite position or orientation, or a zero
// orientation; a size not positive and finite where the shape reads it; an
// explicit tensor that is not symmetric, not positive semidefinite, or breaks
// the triangle inequality (I1 + I2 >= I3). The composite is checked the same
// way, and a principal moment that is not positive is refused, because the
// engine stores its inverse.
[[nodiscard]] Result<CompositeInertia> composite_inertia(std::span<const PartInertia> parts);

}  // namespace spade::vehicles
