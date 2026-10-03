// sim/design_frame.cpp -- see design_frame.hpp. Compiled into spade_sim, under
// spade_fp_strict, so no product here is contracted into an FMA.
#include "sim/design_frame.hpp"

#include <glm/geometric.hpp>

namespace spade {

bool is_identity_frame(const glm::quat& q_bd, const glm::vec3& c) noexcept {
    return q_bd.w == 1.0f && q_bd.x == 0.0f && q_bd.y == 0.0f && q_bd.z == 0.0f && c == glm::vec3(0.0f);
}

FrameState to_body_state(const FrameState& design, const glm::quat& q_bd, const glm::vec3& c) noexcept {
    if (is_identity_frame(q_bd, c)) {
        return design;
    }
    const glm::quat q_wd = glm::normalize(design.orient);
    const glm::vec3 c_w = q_wd * c;                 // R_wd * c
    const glm::vec3 omega_w = q_wd * design.omega;  // R_wd * w_d
    FrameState body;
    body.orient = q_wd * glm::conjugate(q_bd);
    body.pos = design.pos + c_w;
    body.vel = design.vel + glm::cross(omega_w, c_w);
    body.omega = q_bd * design.omega;  // R_bd * w_d
    return body;
}

FrameState to_design_state(const FrameState& body, const glm::quat& q_bd, const glm::vec3& c) noexcept {
    if (is_identity_frame(q_bd, c)) {
        return body;
    }
    const glm::quat q_wb = glm::normalize(body.orient);
    const glm::quat q_wd = q_wb * q_bd;
    const glm::vec3 c_w = q_wd * c;               // R_wd * c
    const glm::vec3 omega_w = q_wb * body.omega;  // R_wb * w_b
    FrameState design;
    design.orient = q_wd;
    design.pos = body.pos - c_w;
    design.vel = body.vel - glm::cross(omega_w, c_w);
    design.omega = glm::conjugate(q_bd) * body.omega;  // R_bd^T * w_b
    return design;
}

}  // namespace spade
