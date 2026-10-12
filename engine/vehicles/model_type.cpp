#include "vehicles/model_type.hpp"

#include <algorithm>
#include <cstdint>
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

// A rotation that can be normalized: finite and of non-zero length. The spawn
// path normalizes it, so this is exactly the precondition that makes the
// passes' unit-quaternion requirement hold from the first substep.
[[nodiscard]] bool orientable(const glm::quat& q) noexcept {
    return finite(q) && glm::dot(q, q) > 0.0f;
}

[[nodiscard]] std::string rotor_prefix(std::size_t index) {
    return "rotor " + std::to_string(index) + ": ";
}

// Whether a part's shape is bounded (CORE-7; the lead's answer of 2026-10-12:
// an unbounded part is refused, since it would fill half of space). A postfix
// walk beside SdfProgram's own stack machine: a plane and a heightfield are
// unbounded, every other primitive is bounded; union and smooth_union are
// bounded when both operands are, intersect when either is, and subtract
// ("a minus b") when its left operand `a` is.
//
// PRECONDITION: shape.validate() passed, so the program is non-empty, every
// operator has two operands, and the stack never holds more than kMaxSdfDepth
// values, as eval()'s does.
[[nodiscard]] bool bounded(const SdfProgram& shape) noexcept {
    bool stack[kMaxSdfDepth] = {};
    uint32_t sp = 0;
    for (const SdfNode& node : shape.nodes) {
        if (node.op == static_cast<uint32_t>(SdfOp::none)) {
            const auto kind = static_cast<SdfPrim>(node.kind);
            stack[sp++] = kind != SdfPrim::plane && kind != SdfPrim::heightfield;
            continue;
        }
        const bool b = stack[--sp];
        const bool a = stack[--sp];
        const auto which = static_cast<SdfOp>(node.op);
        stack[sp++] = which == SdfOp::intersect ? (a || b) : which == SdfOp::subtract ? a : (a && b);
    }
    return sp == 1 && stack[0];
}

}  // namespace

Result<void> ModelType::validate() const {
    std::vector<ModelIssue> all = issues();
    if (all.empty()) return {};
    ModelIssue& first = all.front();
    return std::unexpected(Error{first.code, std::move(first.message)});
}

// THE COLLECT-ALL RULE. Every check runs, in the order validate() has always
// used, and each problem is listed once. A check that reads a value an earlier
// check refused, or would divide by or normalize it, is skipped after that
// refusal, so nothing listed is only an echo of an earlier problem and nothing
// reads out of range. Outside the parts no check depends on another: each
// reads only its own field, and the element loops walk the vectors
// themselves, so an over-long element list is listed once and its elements
// are still checked. A part's checks skip after a refusal three times: the
// rotor index after an unknown attach, the bounded walk after a shape
// SdfProgram::validate() refuses, and the wall's zero rule after a
// coefficient that is negative or non-finite.
std::vector<ModelIssue> ModelType::issues() const {
    using E = ModelIssue::Element;
    std::vector<ModelIssue> out;
    const auto add = [&out](E element, std::size_t index, const char* field, std::string message,
                            Code code = Code::invalid_argument) {
        out.push_back(ModelIssue{
            .element = element, .index = index, .field = field, .message = std::move(message), .code = code});
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

    // --- parts (CORE-7), in PartDesc's field order --------------------------
    // Each message names the part by index and name, then the field.
    for (std::size_t i = 0; i < parts.size(); ++i) {
        const PartDesc& part = parts[i];
        const std::string where = at + "part " + std::to_string(i) + " '" + part.name + "': ";
        const auto refuse = [&](const char* field, const std::string& why, Code code = Code::invalid_argument) {
            add(E::part, i, field, where + why, code);
        };

        // name: non-empty, and unique. The later of two equal names is listed.
        if (part.name.empty()) {
            refuse("name", "name must not be empty");
        } else {
            for (std::size_t j = 0; j < i; ++j) {
                if (parts[j].name == part.name) {
                    refuse("name",
                           "name repeats part " + std::to_string(j) + "'s; a part's name is unique in its model");
                    break;
                }
            }
        }

        // attach, then rotor, which means something only for a known attach.
        const bool on_body = part.attach == PartAttach::body;
        const bool on_rotor = part.attach == PartAttach::rotor;
        if (!on_body && !on_rotor) {
            refuse("attach", "attach must be body (0) or rotor (1), found " +
                                 std::to_string(static_cast<uint32_t>(part.attach)));
        } else if (on_rotor && part.rotor >= rotors.size()) {
            refuse("rotor", "rotor " + std::to_string(part.rotor) + " is out of range: the model has " +
                                std::to_string(rotors.size()) + " rotors");
        } else if (on_body && part.rotor != 0) {
            refuse("rotor", "rotor must be 0 on a part attached to the body, found " + std::to_string(part.rotor));
        }

        // The mount: the body frame, or a rotor part's hub frame.
        if (!finite(part.mount_pos)) refuse("mount_pos", "mount_pos must be finite");
        if (!orientable(part.mount_orient)) {
            refuse("mount_orient", "mount_orient must be finite and have non-zero length");
        }

        // The geometry: exactly one of shape and mesh (CORE-6), refused at
        // `shape` either way. A mesh came from make_mesh(), so it is valid as
        // it stands. A shape is valid (a depth past kMaxSdfDepth keeps
        // capacity_exceeded, as validate_scene() does), free of render data,
        // and bounded; those checks read the shape alone, so they run
        // whenever one is set.
        const bool has_shape = !part.shape.empty();
        const bool has_mesh = part.mesh != nullptr;
        if (has_shape && has_mesh) {
            refuse("shape", "both shape and mesh are set: a part's geometry is exactly one of them");
        } else if (!has_shape && !has_mesh) {
            refuse("shape", "neither shape nor mesh is set: a part is a solid, given by exactly one of them");
        }
        if (has_shape) {
            if (const Result<uint32_t> valid = part.shape.validate(); !valid) {
                refuse("shape", "shape: " + valid.error().context, valid.error().code);
            } else {
                if (!part.shape.node_materials.empty()) {
                    refuse("shape", "shape must not carry node_materials: they are render data, and what the fluid "
                                    "sees is the part's surface");
                }
                if (!bounded(part.shape)) {
                    refuse("shape", "shape is unbounded: a plane or a heightfield fills half of space, so bound it "
                                    "inside an intersect or cut it from a bounded shape");
                }
            }
        }

        // The surface: a known kind, and coefficients finite and >= 0, both 0
        // on a wall (Physics, 2026-10-12).
        const bool wall = part.surface.kind == PartSurface::Kind::wall;
        if (!wall && part.surface.kind != PartSurface::Kind::porous) {
            refuse("surface", "surface.kind must be wall (0) or porous (1), found " +
                                  std::to_string(static_cast<uint32_t>(part.surface.kind)));
        }
        const std::pair<const char*, float> coefficients[] = {{"inv_permeability", part.surface.inv_permeability},
                                                              {"inertial_coeff", part.surface.inertial_coeff}};
        for (const auto& [label, value] : coefficients) {
            if (!(value >= 0.0f) || !finite(value)) {
                refuse("surface", std::string("surface.") + label + " must be finite and >= 0");
            } else if (wall && value != 0.0f) {
                refuse("surface", std::string("surface.") + label + " must be 0 on a wall; only a porous part "
                                                                     "takes coefficients");
            }
        }

        // The smallest feature: declared, by the author or the compile.
        if (!(part.min_feature > 0.0f) || !finite(part.min_feature)) {
            refuse("min_feature", "min_feature must be finite and > 0 (m): the smallest feature the fluid grid "
                                  "must resolve (CORE-8 D5)");
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

float model_reach(const ModelType& model) noexcept {
    float reach = model.proxy_radius;
    for (const RotorDesc& r : model.rotors) reach = std::max(reach, glm::length(r.local_pos) + r.radius);
    return reach;
}

}  // namespace spade::vehicles
