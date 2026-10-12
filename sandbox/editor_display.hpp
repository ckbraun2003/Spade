// How the inspector shows a model's parameters. Display-free (SL15b).
//
// A parameter's row reads as a person would name it, with its unit in the
// label, so the number in the field is never cut off (the lead's review of
// the first stills, Task 10b). The internals a person rarely sets -- the
// parameter schema, the version, the visual's path -- are Advanced, folded
// (§13's progressive disclosure). A value that rounds to zero at the shown
// precision shows as 0: "-0.0" reads as a sign that means something.

#pragma once

#include <cmath>
#include <cstdlib>
#include <string>
#include <string_view>

#include "vehicles/model_type.hpp"

namespace spade::sandbox::editor {

struct ParamDisplay {
    std::string label;
    bool advanced = false;
};

[[nodiscard]] inline ParamDisplay param_display(vehicles::ModelIssue::Element element, std::string_view field) {
    using E = vehicles::ModelIssue::Element;
    struct Row {
        E element;
        std::string_view field;
        std::string_view label;
        bool advanced;
    };
    static constexpr Row kRows[] = {
        {E::model, "param_schema_id", "Parameter schema", true},
        {E::model, "visual_ref", "Visual", true},
        {E::model, "version", "Version", true},
        {E::model, "design_to_principal", "Principal axes", false},
        {E::model, "com_offset", "Centre of mass (m)", false},
        {E::model, "body.mass", "Mass (kg)", false},
        {E::model, "body.inertia_diag", "Inertia (kg\xC2\xB7m\xC2\xB2)", false},
        {E::model, "proxy_radius", "Contact radius (m)", false},
        {E::rotor, "local_pos", "Hub (m)", false},
        {E::rotor, "local_orient", "Orientation", false},
        {E::rotor, "spin_dir", "Spin (+1 or -1)", false},
        {E::rotor, "tau", "Spin-up time (s)", false},
        {E::rotor, "radius", "Radius (m)", false},
        {E::rotor, "thrust_coeff", "Thrust coefficient (N\xC2\xB7s\xC2\xB2)", false},
        {E::rotor, "torque_coeff", "Torque coefficient (N\xC2\xB7m\xC2\xB7s\xC2\xB2)", false},
        {E::drag_body, "mode", "Drag law", false},
        {E::drag_body, "area", "Area (m\xC2\xB2)", false},
        {E::drag_body, "coeffs", "Coefficients", false},
        {E::drag_body, "local_pos", "Position (m)", false},
        {E::drag_body, "local_orient", "Orientation", false},
        {E::imu_mount, "mount_pos", "Position (m)", false},
        {E::imu_mount, "mount_orient", "Orientation", false},
        {E::imu_mount, "rate_divider", "Sample every (substeps)", false},
        {E::imu_mount, "sigma_a", "Accel noise (m/s\xC2\xB2)", false},
        {E::imu_mount, "sigma_g", "Gyro noise (rad/s)", false},
        {E::imu_mount, "sigma_ba", "Accel bias walk (m/s\xC2\xB2)", false},
        {E::imu_mount, "sigma_bg", "Gyro bias walk (rad/s)", false},
    };
    for (const Row& r : kRows) {
        if (r.element == element && r.field == field) return ParamDisplay{std::string(r.label), r.advanced};
    }
    return ParamDisplay{std::string(field), false};  // a field the table does not name yet shows as itself
}

// `v` as it should show at `decimals` places: 0 when it would print as zero,
// so never "-0.0". NaN shows as it is.
[[nodiscard]] inline float shown(float v, int decimals) {
    if (std::isnan(v)) return v;
    const float half_step = 0.5f * std::pow(10.0f, -static_cast<float>(decimals));
    return std::abs(v) < half_step ? 0.0f : v;
}

// The decimals a printf format shows ("%.2f m" -> 2); 6 when it does not say,
// which is printf's own default.
[[nodiscard]] inline int decimals_of(std::string_view format) {
    const std::size_t dot = format.find("%.");
    if (dot == std::string_view::npos || dot + 2 >= format.size()) return 6;
    int n = 0;
    std::size_t i = dot + 2;
    for (; i < format.size() && format[i] >= '0' && format[i] <= '9'; ++i) n = 10 * n + (format[i] - '0');
    return i == dot + 2 ? 6 : n;
}

}  // namespace spade::sandbox::editor
