#include "vehicles/rotor_wake.hpp"

#include <cmath>

#include "vehicles/rotor.hpp"

namespace spade::vehicles {

namespace {

// Below this fraction of v_i the far-wake velocity F + 2 v_i n has no usable
// direction. That happens exactly at the windmill-brake boundary x = -2 with no
// crosswind, where the wake is not convected at all. The disc normal stands in
// for it there.
constexpr float kWakeAxisFloor = 1.0e-6f;

bool finite_vec(const glm::vec3& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace

glm::vec3 rotor_wake_velocity(const RotorWakeInput& in, glm::vec3 point) noexcept {
    const glm::vec3 none(0.0f);

    // Disc loading, through the same helper apply_rotors() uses. It already
    // returns 0 for a non-positive or non-finite thrust, density or radius.
    // The isfinite catches an omega large enough that k_T w^2 overflows.
    const float thrust = in.thrust_coeff * in.omega * in.omega;
    const float v_hover = rotor_hover_induced_velocity(thrust, in.density, in.radius);
    if (!(v_hover > 0.0f) || !std::isfinite(v_hover)) {
        return none;
    }
    const float R = in.radius;
    const float R2 = R * R;
    if (!(R2 > 0.0f)) {
        return none;  // R^2 underflowed; the profile below would divide by it
    }

    const float axis_len = glm::length(in.thrust_axis_world);
    if (!(axis_len > 0.0f) || !std::isfinite(axis_len)) {
        return none;
    }
    const glm::vec3 n = -(in.thrust_axis_world / axis_len);  // the direction the disc pushes air

    const glm::vec3& F = in.freestream;
    if (!finite_vec(F) || !std::isfinite(glm::dot(F, F))) {
        return none;
    }
    const glm::vec3 d = point - in.hub_world;
    if (!finite_vec(d) || !std::isfinite(glm::dot(d, d))) {
        return none;
    }

    // The induced velocity RotorElement's closure develops at the disc.
    // rotor.hpp's v_axial is the rotor's air-relative climb rate,
    // axis . (0 - F), which is F . n.
    const float v_axial = glm::dot(F, n);
    const float v_i = v_hover * rotor_inflow_factor(v_axial, v_hover);
    if (!(v_i > 0.0f)) {
        return none;
    }

    // Wake axis: along the far-wake air velocity (header, model step 2).
    const glm::vec3 far_wake = F + n * (2.0f * v_i);
    const float far_len = glm::length(far_wake);
    const glm::vec3 w = far_len > kWakeAxisFloor * v_i ? far_wake / far_len : n;

    // Station: downstream distance s along the axis, distance r from it.
    const float s = glm::dot(d, w);
    const float r = glm::length(d - w * s);

    // Axial profile u(s) = v_i (1 + s / q), q = sqrt(s^2 + R^2), in the form
    // that does not cancel on each side of the disc (header, model step 1).
    const float q = std::sqrt(s * s + R2);
    const float build = s >= 0.0f ? 1.0f + s / q : R2 / (q * (q - s));
    const float u = v_i * build;
    if (!(u > 0.0f)) {
        return none;  // far enough upstream that the profile underflowed
    }

    // Contraction by continuity along the axis (model step 3), falling back to
    // the still-air ratio where the flow along the axis is not one-signed.
    const float f_w = glm::dot(F, w);
    const float n_w = glm::dot(n, w);
    const float u_disc = f_w + v_i * n_w;
    const float u_here = f_w + u * n_w;
    const float area_ratio = (u_disc > 0.0f && u_here > 0.0f) ? u_disc / u_here : v_i / u;
    const float r_tube = R * std::sqrt(area_ratio);

    // Smooth tube edge (model step 4). An x large enough to overflow x^16
    // gives an edge of exactly 0, which is the right limit. A point on the
    // axis is inside the tube even if r_tube underflowed to 0, so it never
    // forms 0 / 0.
    const float x = r > 0.0f ? r / r_tube : 0.0f;
    const float x2 = x * x;
    const float x4 = x2 * x2;
    const float x8 = x4 * x4;
    const float edge = 1.0f / (1.0f + x8 * x8);

    return n * (u * edge);
}

}  // namespace spade::vehicles
