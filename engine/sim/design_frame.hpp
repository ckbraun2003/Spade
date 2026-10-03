#pragma once

// ---------------------------------------------------------------------------
// sim/design_frame.hpp -- a vehicle's design frame (the drone-builder physics
// requirements DBP-44..47; Core's drone-builder plan, Task B).
//
// Kat's flight packages and part files see only the DESIGN frame: the flight
// controller's axes, with its origin wherever the airframe was drawn. The engine
// integrates a body in its PRINCIPAL frame, at its centre of mass, because its
// inertia is stored diagonal there. A model type carries the two quantities that
// relate them:
//   q_bd  design_to_principal -- maps design-frame vectors to body-frame ones;
//   c     com_offset          -- the centre of mass from the design origin,
//                                in the design frame, metres.
// Only the model type, Simulation::spawn() and Simulation::vehicle_state() see
// them (DBP-45). These two functions are the whole conversion, so the spawn and
// the read cannot disagree.
//
// THE RIGID-BODY IDENTITIES (DBP-44), with R_wd the design-to-world rotation:
//   q_wb  = q_wd * conj(q_bd)
//   p_com = p_d + R_wd * c
//   w_b   = R_bd * w_d                    (body rates from design rates)
//   v_com = v_d + w_w x (R_wd * c),       with w_w = R_wd * w_d
// and their inverses for the read.
//
// THE IDENTITY FRAME IS UNTOUCHED, BIT FOR BIT. Every model built before the
// design frame existed has q_bd exactly (1, 0, 0, 0) and c exactly zero; both
// functions then return their input as given, so those vehicles spawn and read
// exactly as before and no golden moves.
// ---------------------------------------------------------------------------

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

namespace spade {

// A rigid body's state referred to one frame's origin and axes: its position
// and velocity are the world-frame motion of that origin, `orient` maps the
// frame's axes to the world's, and `omega` is the angular rate in the frame's
// own axes. vehicle_state() returns it for the DESIGN frame.
struct FrameState {
    glm::vec3 pos{0.0f};
    glm::quat orient{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 vel{0.0f};
    glm::vec3 omega{0.0f};
};

// True when q_bd is exactly the identity and c exactly zero: the frames coincide.
[[nodiscard]] bool is_identity_frame(const glm::quat& q_bd, const glm::vec3& c) noexcept;

// Design-frame state -> the body's (principal axes, centre of mass). q_bd must
// be unit (register_model() stores it normalized). The design orientation is
// normalized before use, as spawn() would normalize it.
[[nodiscard]] FrameState to_body_state(const FrameState& design, const glm::quat& q_bd, const glm::vec3& c) noexcept;

// The inverse: the body's state -> the design frame's.
[[nodiscard]] FrameState to_design_state(const FrameState& body, const glm::quat& q_bd, const glm::vec3& c) noexcept;

}  // namespace spade
