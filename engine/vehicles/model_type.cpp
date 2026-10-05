#include "vehicles/model_type.hpp"

#include <string>
#include <utility>

#include <glm/geometric.hpp>

#include "core/validate.hpp"

// ---------------------------------------------------------------------------
// ModelType::validate() -- the model-type layer's whole runtime cost, paid
// once per registration. See model_type.hpp for what a ModelType is and why
// this function, rather than the passes, owns the checking.
// ---------------------------------------------------------------------------

namespace spade::vehicles {
namespace {

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

// A rotation that can be normalized: finite and of non-zero length. The spawn
// path normalizes it, so this is exactly the precondition that makes the
// passes' unit-quaternion requirement hold from the first substep.
[[nodiscard]] bool orientable(const glm::quat& q) noexcept {
    return finite(q) && glm::dot(q, q) > 0.0f;
}

[[nodiscard]] std::string rotor_prefix(std::size_t index) {
    return "rotor " + std::to_string(index) + ": ";
}

}  // namespace

Result<void> ModelType::validate() const {
    std::vector<ModelIssue> all = issues();
    if (all.empty()) return {};
    return std::unexpected(invalid(std::move(all.front().message)));
}

// THE COLLECT-ALL RULE. Every check runs, in the order validate() has always
// used, and each problem is listed once. A check that reads a value an earlier
// check refused, or would divide by or normalize it, is skipped after that
// refusal, so nothing listed is only an echo of an earlier problem and nothing
// reads out of range. Today no check depends on another: each reads only its
// own field, and the element loops walk the vectors themselves, so an
// over-long element list is listed once and its elements are still checked.
std::vector<ModelIssue> ModelType::issues() const {
    using E = ModelIssue::Element;
    std::vector<ModelIssue> out;
    const auto add = [&out](E element, std::size_t index, const char* field, std::string message) {
        out.push_back(ModelIssue{element, index, field, std::move(message)});
    };

    if (name.empty()) add(E::model, 0, "name", "model type: name must not be empty");
    const std::string at = "model type '" + name + "': ";

    // --- version and design frame (DBE-005, DBP-44) --------------------------
    if (version == 0) add(E::model, 0, "version", at + "version must be >= 1");
    if (!orientable(design_to_principal)) {
        add(E::model, 0, "design_to_principal", at + "design_to_principal must be finite with non-zero length");
    }
    if (!finite(com_offset)) add(E::model, 0, "com_offset", at + "com_offset must be finite");

    // --- body template ------------------------------------------------------
    if (!(body.mass > 0.0f) || !finite(body.mass)) {
        // physics/integrator.cpp DIVIDES by mass (step 1) and does not guard
        // it: a zero mass is an infinity that step 4's arithmetic turns into a
        // NaN one substep later. Simulation::spawn refuses it too; refusing it
        // HERE means a bad model is rejected at registration instead of at the
        // first instance.
        add(E::model, 0, "body.mass", at + "body.mass must be finite and > 0");
    }
    if (!finite(body.inertia_diag) || !(body.inertia_diag.x > 0.0f) || !(body.inertia_diag.y > 0.0f) ||
        !(body.inertia_diag.z > 0.0f)) {
        // Componentwise > 0, NOT >= 0. See BodyTemplate: a zero moment would
        // be inverted into the integrator's "rotation-locked axis", which is a
        // different statement from "this vehicle has no inertia about Y".
        add(E::model, 0, "body.inertia_diag", at + "body.inertia_diag must be finite and componentwise > 0");
    }
    if (!(proxy_radius >= 0.0f) || !finite(proxy_radius)) {
        add(E::model, 0, "proxy_radius", at + "proxy_radius must be finite and >= 0");
    }

    // --- element and sensor counts -----------------------------------------
    if (rotors.size() > kMaxModelRotors) {
        add(E::model, 0, "rotors", at + "declares more than " + std::to_string(kMaxModelRotors) + " rotors");
    }
    if (drag_bodies.size() > kMaxModelDragBodies) {
        add(E::model, 0, "drag_bodies",
            at + "declares more than " + std::to_string(kMaxModelDragBodies) + " drag bodies");
    }
    if (imu_mounts.size() > kMaxModelImuMounts) {
        add(E::model, 0, "imu_mounts",
            at + "declares more than " + std::to_string(kMaxModelImuMounts) + " IMU mounts");
    }

    // --- rotors -------------------------------------------------------------
    for (std::size_t i = 0; i < rotors.size(); ++i) {
        const RotorDesc& rotor = rotors[i];
        const std::string where = at + rotor_prefix(i);

        if (!finite(rotor.local_pos)) add(E::rotor, i, "local_pos", where + "local_pos must be finite");
        if (!orientable(rotor.local_orient)) {
            add(E::rotor, i, "local_orient", where + "local_orient must be finite and have non-zero length");
        }
        // A DIRECTION, not a scale (rotor.hpp): 2 would double the yaw torque
        // without doubling the thrust, which is not a rotor.
        if (rotor.spin_dir != 1.0f && rotor.spin_dir != -1.0f && rotor.spin_dir != 0.0f) {
            add(E::rotor, i, "spin_dir", where + "spin_dir must be exactly +1, -1 or 0");
        }
        if (!(rotor.tau >= 0.0f) || !finite(rotor.tau)) {
            // <= 0 legitimately means "no lag" (rotor.hpp section 5), so 0 is
            // admitted; a NEGATIVE time constant is a typo, not a model.
            add(E::rotor, i, "tau", where + "tau must be finite and >= 0");
        }
        // -------------------------------------------------------------------
        // THE ONE THAT FAILS QUIETLY WITHOUT THIS CHECK, and the reason
        // rotor.hpp's apply_rotors() preconditions name Task 18 by number:
        // radius == 0 makes v_h == 0 (no disc area, so rotor_inflow_factor()
        // returns its neutral 1) AND leaves no image source (so
        // rotor_ground_factor() returns 1 as well). The result is a rotor
        // producing exactly its static thrust in climb, in descent and on the
        // ground -- a plausible-looking vehicle with both aero corrections
        // silently switched off, and nothing anywhere reporting a problem.
        // -------------------------------------------------------------------
        if (!(rotor.radius > 0.0f) || !finite(rotor.radius)) {
            add(E::rotor, i, "radius",
                where + "radius must be finite and > 0 (radius == 0 silently disables BOTH the "
                        "momentum-theory inflow correction and ground effect -- see "
                        "vehicles/rotor.hpp's apply_rotors preconditions)");
        }
        if (!(rotor.thrust_coeff > 0.0f) || !finite(rotor.thrust_coeff)) {
            add(E::rotor, i, "thrust_coeff", where + "thrust_coeff must be finite and > 0");
        }
        if (!(rotor.torque_coeff >= 0.0f) || !finite(rotor.torque_coeff)) {
            // 0 is admitted: a rotor whose reaction torque is carried
            // elsewhere (a contra-rotating pair modelled as one disc) is a
            // legitimate model. Negative is not -- Q is a non-negative
            // magnitude and `spin_dir` is what carries the sign (rotor.hpp
            // section 6).
            add(E::rotor, i, "torque_coeff", where + "torque_coeff must be finite and >= 0");
        }
    }

    // --- drag bodies --------------------------------------------------------
    for (std::size_t i = 0; i < drag_bodies.size(); ++i) {
        const DragBodyDesc& drag = drag_bodies[i];
        const std::string where = at + "drag body " + std::to_string(i) + ": ";

        const physics::DragLawCheck law = physics::check_drag_law(drag.mode, drag.area, drag.coeffs);
        if (law.mode != nullptr) add(E::drag_body, i, "mode", where + law.mode);
        if (law.area != nullptr) add(E::drag_body, i, "area", where + law.area);
        if (law.coeffs != nullptr) add(E::drag_body, i, "coeffs", where + law.coeffs);
        if (!finite(drag.local_pos)) add(E::drag_body, i, "local_pos", where + "local_pos must be finite");
        if (!orientable(drag.local_orient)) {
            add(E::drag_body, i, "local_orient", where + "local_orient must be finite and have non-zero length");
        }
    }

    // --- IMU mounts ---------------------------------------------------------
    for (std::size_t i = 0; i < imu_mounts.size(); ++i) {
        const ImuMountDesc& imu = imu_mounts[i];
        const std::string where = at + "imu mount " + std::to_string(i) + ": ";

        if (imu.rate_divider == 0) add(E::imu_mount, i, "rate_divider", where + "rate_divider must be >= 1");
        if (!finite(imu.mount_pos)) add(E::imu_mount, i, "mount_pos", where + "mount_pos must be finite");
        if (!orientable(imu.mount_orient)) {
            add(E::imu_mount, i, "mount_orient", where + "mount_orient must be finite and have non-zero length");
        }
        if (!(imu.sigma_a >= 0.0f) || !(imu.sigma_g >= 0.0f) || !(imu.sigma_ba >= 0.0f) ||
            !(imu.sigma_bg >= 0.0f) || !finite(imu.sigma_a) || !finite(imu.sigma_g) ||
            !finite(imu.sigma_ba) || !finite(imu.sigma_bg)) {
            add(E::imu_mount, i, "sigma", where + "every sigma must be finite and >= 0");
        }
    }
    return out;
}

glm::quat canonical_design_rotation(const glm::quat& q) noexcept {
    if (q.w == 1.0f && q.x == 0.0f && q.y == 0.0f && q.z == 0.0f) {
        return q;
    }
    glm::quat n = glm::normalize(q);
    const bool flip = n.w < 0.0f ||
                      (n.w == 0.0f && (n.x < 0.0f || (n.x == 0.0f && (n.y < 0.0f || (n.y == 0.0f && n.z < 0.0f)))));
    if (flip) {
        n = -n;
    }
    return n;
}

}  // namespace spade::vehicles
