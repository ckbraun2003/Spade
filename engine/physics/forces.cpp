#include "physics/forces.hpp"

#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/integrator.hpp"

namespace spade::physics {

const char* check_drag_law(uint32_t mode, float area, const glm::vec3& coeffs) noexcept {
    if (mode != drag_mode::quadratic && mode != drag_mode::componentwise) return "unknown drag mode";
    if (!std::isfinite(area) || !(area >= 0.0f)) return "area must be finite and >= 0";
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(coeffs[i]) || !(coeffs[i] >= 0.0f)) return "coeffs must be finite and >= 0";
    }
    return nullptr;
}

void apply_drag(std::span<BodyState> bodies, std::span<const DragBodyRow> elems, const Medium& medium,
                 const WorldParams& params, float h) noexcept {
    // Pure per-substep force accumulation -- see forces.hpp's doc comment on
    // why `h` is nonetheless part of the signature.
    (void)h;

    for (const DragBodyRow& elem : elems) {
        if (elem.enabled == 0u) continue;

        // Precondition (documented in forces.hpp, not runtime-checked, same
        // posture as integrate_bodies()'s mass/orient preconditions):
        // elem.body_slot indexes `bodies` directly.
        BodyState& body = bodies[elem.body_slot];
        if ((body.flags & body_flags::active) == 0u) continue;

        // engine design D6 / world/medium.hpp: every aerodynamic force reads
        // the air through this one seam, sampled at the body's own position.
        const MediumSample sample = medium.sample(params, body.pos);
        const glm::vec3 v_rel = body.vel - sample.wind;  // world frame

        glm::vec3 F_world;  // what force_acc (WORLD frame) accumulates
        glm::vec3 F_body;   // what the torque cross product below uses

        if (elem.mode == drag_mode::componentwise) {
            // PER-AXIS QUADRATIC DRAG, F_i = -c_i * |v_i| * v_i on each BODY
            // axis. Unlike the isotropic law, each body axis carries its own
            // coefficient and one axis's speed never scales another axis's
            // force.
            //
            // The frames follow layout.hpp's accumulator convention:
            //   1. rotate v_rel into the body frame;
            //   2. apply the per-axis law there, giving F_body;
            //   3. rotate F_body back to world for force_acc.
            // F_body is KEPT rather than discarded, because the torque term
            // below needs it directly (torque_acc is body frame).
            //
            // v_rel is velocity relative to the air, vel - medium.wind, the
            // same relative-velocity notion the quadratic law uses, through
            // the one Medium seam. The sign is the explicit minus above: drag
            // opposes the relative velocity on every axis.
            const glm::vec3 v_rel_body = glm::conjugate(body.orient) * v_rel;
            F_body = glm::vec3(-elem.coeffs.x * std::fabs(v_rel_body.x) * v_rel_body.x,
                                -elem.coeffs.y * std::fabs(v_rel_body.y) * v_rel_body.y,
                                -elem.coeffs.z * std::fabs(v_rel_body.z) * v_rel_body.z);
            F_world = body.orient * F_body;
        } else {
            // Quadratic (coordinator resolution): F = -1/2 * rho * Cd * A *
            // |v_rel| * v_rel, WORLD frame. coeffs.x carries Cd
            // (dimensionless); y/z are unused -- see drag_mode::quadratic.
            const float speed = glm::length(v_rel);
            F_world = (-0.5f * sample.density * elem.coeffs.x * elem.area * speed) * v_rel;
            F_body = glm::conjugate(body.orient) * F_world;
        }

        body.force_acc += F_world;
        body.torque_acc += glm::cross(elem.local_pos, F_body);
    }
}

}  // namespace spade::physics
