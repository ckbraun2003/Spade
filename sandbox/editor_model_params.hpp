// The inspector's model parameters. Display-free (SL15b).
//
// Every ModelType parameter as a path -- the part (the model itself, a rotor,
// a drag body, an IMU mount), its index, and the field -- with its kind, so
// the inspector draws a widget per row without knowing the struct (EDT-013).
// The field names are the ones ModelType::issues() reports, so a problem
// lands on its own row (EDT-014); issues() names the four IMU noise sigmas
// together as "sigma". The model's name is its identity, not a parameter.
//
// There is no path that adds or removes a part: a model with another
// structure is a new model, and new models come from Kat's builder (the
// user's D2).
//
// docs/design/interface/plans/2026-10-05-editor-design.md §4;
// 2026-10-05-editor-plan.md Task 3.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <variant>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "vehicles/model_type.hpp"

namespace spade::sandbox::editor {

enum class ParamKind { scalar, count, vec3, quat, text };

struct ModelParam {
    vehicles::ModelIssue::Element element;
    std::string_view field;
    ParamKind kind;
};

using ParamValue = std::variant<float, uint32_t, glm::vec3, glm::quat, std::string>;

// The inspector's table, in each struct's order.
[[nodiscard]] inline std::span<const ModelParam> model_params() {
    using E = vehicles::ModelIssue::Element;
    static constexpr std::array<ModelParam, 27> kTable = {{
        {E::model, "param_schema_id", ParamKind::count},
        {E::model, "visual_ref", ParamKind::text},
        {E::model, "version", ParamKind::count},
        {E::model, "design_to_principal", ParamKind::quat},
        {E::model, "com_offset", ParamKind::vec3},
        {E::model, "body.mass", ParamKind::scalar},
        {E::model, "body.inertia_diag", ParamKind::vec3},
        {E::model, "proxy_radius", ParamKind::scalar},
        {E::rotor, "local_pos", ParamKind::vec3},
        {E::rotor, "local_orient", ParamKind::quat},
        {E::rotor, "spin_dir", ParamKind::scalar},
        {E::rotor, "tau", ParamKind::scalar},
        {E::rotor, "radius", ParamKind::scalar},
        {E::rotor, "thrust_coeff", ParamKind::scalar},
        {E::rotor, "torque_coeff", ParamKind::scalar},
        {E::drag_body, "mode", ParamKind::count},
        {E::drag_body, "area", ParamKind::scalar},
        {E::drag_body, "coeffs", ParamKind::vec3},
        {E::drag_body, "local_pos", ParamKind::vec3},
        {E::drag_body, "local_orient", ParamKind::quat},
        {E::imu_mount, "mount_pos", ParamKind::vec3},
        {E::imu_mount, "mount_orient", ParamKind::quat},
        {E::imu_mount, "rate_divider", ParamKind::count},
        {E::imu_mount, "sigma_a", ParamKind::scalar},
        {E::imu_mount, "sigma_g", ParamKind::scalar},
        {E::imu_mount, "sigma_ba", ParamKind::scalar},
        {E::imu_mount, "sigma_bg", ParamKind::scalar},
    }};
    return kTable;
}

// A part's name in messages: "the model", "rotor 2", "drag body 0", "IMU mount 1".
[[nodiscard]] inline std::string part_name(vehicles::ModelIssue::Element e, std::size_t index) {
    using E = vehicles::ModelIssue::Element;
    switch (e) {
        case E::model: return "the model";
        case E::rotor: return "rotor " + std::to_string(index);
        case E::drag_body: return "drag body " + std::to_string(index);
        case E::imu_mount: return "IMU mount " + std::to_string(index);
    }
    return "part " + std::to_string(index);
}

namespace detail {

// The field a path names, as a pointer of the one type it holds; every other
// pointer is null. `index` must already be in range.
struct FieldRef {
    float* scalar = nullptr;
    uint32_t* count = nullptr;
    glm::vec3* vec3 = nullptr;
    glm::quat* quat = nullptr;
    std::string* text = nullptr;
};

[[nodiscard]] inline FieldRef field_of(vehicles::ModelType& m, vehicles::ModelIssue::Element e, std::size_t i,
                                       std::string_view f) {
    using E = vehicles::ModelIssue::Element;
    FieldRef r;
    if (e == E::model) {
        if (f == "param_schema_id") r.count = &m.param_schema_id;
        else if (f == "visual_ref") r.text = &m.visual_ref;
        else if (f == "version") r.count = &m.version;
        else if (f == "design_to_principal") r.quat = &m.design_to_principal;
        else if (f == "com_offset") r.vec3 = &m.com_offset;
        else if (f == "body.mass") r.scalar = &m.body.mass;
        else if (f == "body.inertia_diag") r.vec3 = &m.body.inertia_diag;
        else if (f == "proxy_radius") r.scalar = &m.proxy_radius;
    } else if (e == E::rotor) {
        vehicles::RotorDesc& p = m.rotors[i];
        if (f == "local_pos") r.vec3 = &p.local_pos;
        else if (f == "local_orient") r.quat = &p.local_orient;
        else if (f == "spin_dir") r.scalar = &p.spin_dir;
        else if (f == "tau") r.scalar = &p.tau;
        else if (f == "radius") r.scalar = &p.radius;
        else if (f == "thrust_coeff") r.scalar = &p.thrust_coeff;
        else if (f == "torque_coeff") r.scalar = &p.torque_coeff;
    } else if (e == E::drag_body) {
        vehicles::DragBodyDesc& p = m.drag_bodies[i];
        if (f == "mode") r.count = &p.mode;
        else if (f == "area") r.scalar = &p.area;
        else if (f == "coeffs") r.vec3 = &p.coeffs;
        else if (f == "local_pos") r.vec3 = &p.local_pos;
        else if (f == "local_orient") r.quat = &p.local_orient;
    } else if (e == E::imu_mount) {
        vehicles::ImuMountDesc& p = m.imu_mounts[i];
        if (f == "mount_pos") r.vec3 = &p.mount_pos;
        else if (f == "mount_orient") r.quat = &p.mount_orient;
        else if (f == "rate_divider") r.count = &p.rate_divider;
        else if (f == "sigma_a") r.scalar = &p.sigma_a;
        else if (f == "sigma_g") r.scalar = &p.sigma_g;
        else if (f == "sigma_ba") r.scalar = &p.sigma_ba;
        else if (f == "sigma_bg") r.scalar = &p.sigma_bg;
    }
    return r;
}

[[nodiscard]] inline std::size_t part_count(const vehicles::ModelType& m, vehicles::ModelIssue::Element e) {
    using E = vehicles::ModelIssue::Element;
    switch (e) {
        case E::model: return 1;
        case E::rotor: return m.rotors.size();
        case E::drag_body: return m.drag_bodies.size();
        case E::imu_mount: return m.imu_mounts.size();
    }
    return 0;
}

[[nodiscard]] inline std::string_view parts_word(vehicles::ModelIssue::Element e) {
    using E = vehicles::ModelIssue::Element;
    switch (e) {
        case E::model: return "models";
        case E::rotor: return "rotors";
        case E::drag_body: return "drag bodies";
        case E::imu_mount: return "IMU mounts";
    }
    return "parts";
}

// Checks the index, then finds the field; the errors say which and why.
[[nodiscard]] inline Result<FieldRef> locate(vehicles::ModelType& m, vehicles::ModelIssue::Element e,
                                             std::size_t index, std::string_view field) {
    const std::size_t n = part_count(m, e);
    if (index >= n) {
        return std::unexpected(Error{Code::invalid_argument,
                                     part_name(e, index) + " of model '" + m.name + "': the model has " +
                                         std::to_string(n) + " " + std::string(parts_word(e)) +
                                         ", and the editor never adds a part; a model with more is a new model, "
                                         "built in Kat"});
    }
    FieldRef r = field_of(m, e, index, field);
    if (!r.scalar && !r.count && !r.vec3 && !r.quat && !r.text) {
        return std::unexpected(Error{Code::invalid_argument, part_name(e, index) + " has no parameter '" +
                                                                 std::string(field) +
                                                                 "'; the inspector's table lists the ones it has"});
    }
    return r;
}

}  // namespace detail

[[nodiscard]] inline Result<ParamValue> get_model_param(const vehicles::ModelType& m, vehicles::ModelIssue::Element e,
                                                        std::size_t index, std::string_view field) {
    vehicles::ModelType copy = m;  // field_of() hands out pointers; read through a copy, never the caller's
    const Result<detail::FieldRef> r = detail::locate(copy, e, index, field);
    if (!r) return std::unexpected(r.error());
    if (r->scalar) return ParamValue{*r->scalar};
    if (r->count) return ParamValue{*r->count};
    if (r->vec3) return ParamValue{*r->vec3};
    if (r->quat) return ParamValue{*r->quat};
    return ParamValue{*r->text};
}

// Sets one parameter in place. Refuses an index past the part count, an
// unknown field, or a value of another kind; the model is unchanged then.
[[nodiscard]] inline Result<void> set_model_param(vehicles::ModelType& m, vehicles::ModelIssue::Element e,
                                                  std::size_t index, std::string_view field, const ParamValue& v) {
    const Result<detail::FieldRef> r = detail::locate(m, e, index, field);
    if (!r) return std::unexpected(r.error());
    const bool ok = (r->scalar && std::holds_alternative<float>(v)) || (r->count && std::holds_alternative<uint32_t>(v)) ||
                    (r->vec3 && std::holds_alternative<glm::vec3>(v)) || (r->quat && std::holds_alternative<glm::quat>(v)) ||
                    (r->text && std::holds_alternative<std::string>(v));
    if (!ok) {
        return std::unexpected(Error{Code::invalid_argument, part_name(e, index) + " " + std::string(field) +
                                                                 ": the value is of another kind than the parameter"});
    }
    if (r->scalar) *r->scalar = std::get<float>(v);
    if (r->count) *r->count = std::get<uint32_t>(v);
    if (r->vec3) *r->vec3 = std::get<glm::vec3>(v);
    if (r->quat) *r->quat = std::get<glm::quat>(v);
    if (r->text) *r->text = std::get<std::string>(v);
    return {};
}

// Every problem in a model, as the inspector shows them: "rotor 2 radius: ...".
[[nodiscard]] inline std::string describe_issues(const vehicles::ModelType& m) {
    std::string out;
    for (const vehicles::ModelIssue& i : m.issues()) {
        out += (out.empty() ? "" : "; ") + part_name(i.element, i.index) + " " + i.field + ": " + i.message;
    }
    return out;
}

}  // namespace spade::sandbox::editor
