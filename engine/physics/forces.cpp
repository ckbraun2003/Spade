#include "physics/forces.hpp"

#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/integrator.hpp"

namespace spade::physics {

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
            // KAT'S EXISTING DRAG CONVENTION -- configs/physics.yaml
            // (worktree root), keys `drag_coeff` / `drag_mode`:
            //
            //   drag_mode: componentwise # ... 'componentwise' (F_i=-cd*|v_i|*v_i)
            //   drag_coeff: 0.0280 # drag coefficient (kg/m for quadratic/componentwise)
            //
            // and the file's own header comment on those keys: "componentwise
            // quadratic drag, F_i = -cd*|v_i|*v_i per BODY axis (matches the
            // sim's per-axis VelocityDragCoefficients)" (S6 hygiene: this
            // header comment used to drop the minus sign the `drag_mode`
            // line above and this file's own sign convention both carry --
            // reconciled in configs/physics.yaml so the file no longer
            // states its own formula two ways). Cross-checked against
            // the two engines that actually implement it:
            //   * controller/dynamics/quadrotor.py, _drag_force_jit(),
            //     "componentwise" branch: rotate v into the body frame,
            //     apply drag_coeff*|v_i|*v_i per axis, rotate the RESULT back
            //     to world (that file wants a world-frame acceleration to
            //     subtract; see its "MEASURED 2026-07-29" doc comment for the
            //     physical justification of the per-axis form).
            //   * interface/src/physics/quadrotor.cpp, drag_accel():
            //     `k * fb` where `fb_i = |vb_i|*vb_i`, and the caller does
            //     `... - drag_accel(...)` -- i.e. this positive-signed
            //     per-axis quantity is SUBTRACTED, which is exactly the
            //     yaml's explicit minus sign spelled out as an addition here.
            //
            // Spade's own accumulator convention (layout.hpp: force_acc
            // world-frame, torque_acc body-frame) makes this the SAME
            // rotate-there-rotate-back round trip kat's integrators perform
            // (v_rel into body frame, apply the per-axis law, F_body back
            // out to world) -- but Spade additionally KEEPS the body-frame
            // F_body around afterwards, because the torque term below needs
            // it directly. kat discards it once it has F_world; Spade cannot,
            // since torque_acc is body-frame by layout.hpp's ruling.
            //
            // GENERALIZATION BEYOND KAT: kat's drag has no wind model (its
            // world has none), so its `v` is the raw world/body velocity.
            // Spade generalizes to v_rel = vel - medium.wind (per this task's
            // brief, so quadratic and componentwise share one relative-
            // velocity notion through the Medium seam) -- everything else
            // about the law, including its sign, is unchanged from kat's.
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
