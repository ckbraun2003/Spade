#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "physics/airflow/blade.hpp"
#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// THE ACTUATOR LINE (P4b, L61; the design note .superpowers/sdd/2026-11-10-wake/p4-actuator-line-design.md section 2). Each
// blade is a row of elements from the hub's cut-out to the tip; each element feels the air it sits in, takes the load of its
// section (blade.hpp's section_load()), and gives the air the opposite force over a Gaussian kernel, on the face increments a
// fluid step adds (the discs' du, dv, dw). One rotor, one call a step; the CPU's, and the GPU twin's reference (P4c).
//
// THE FRAME. The rotor has its thrust axis `axis` (unit; the direction the rotor pushes the DRONE: +Y for a quad, opposite the
// ActuatorDisc's axis), an azimuth zero `ref` (unit, perpendicular to the axis) and a spin s = +-1. Blade b of B lies at azimuth
// phi = s psi + 2 pi b / B about the axis, psi P11's blade angle (grows with omega >= 0 whatever the hand): its direction
// d = ref cos phi + (axis x ref) sin phi, and it moves along m = s (axis x d), at omega r. The air's speed through the rotor is
// ua = -(u . axis), positive along the way the rotor pushes the air; the section's speed through the air is wt = omega r - u . m.
// The air takes -axial axis + tangent m per metre of span (blade.hpp: the force on the blade is axial along the axis and tangent
// against m).
//
// THE ELEMENTS. Between the first station's r and the radius, ceil((R - r0) / dx) of them a blade, at the midpoints of equal spans
// dr; chord and pitch interpolated linearly between stations. Where an element is, and the air it feels, is evaluated at
// n_sub azimuths across the step, psi + omega h (k + 1/2) / n_sub, k = 0 .. n_sub - 1 (n_sub 1: the mid-step azimuth), each at 1/n_sub of
// the force: the design note's choice between a strict tip rule and sub-stepping the force, MEASURED, not assumed. All read the grid as
// the step begins.
//
// THE KERNEL. Separable Gaussian, eps = 2 dx: per axis g(d) = exp(-(d / eps)^2) for |d| <= 3 eps (6 cells), 0 beyond, exp by
// fp32_math's exp32. The air an element feels is the kernel-weighted mean of the faces' velocities, component by component,
// normalised by its own weights' sum S_c (so a uniform flow is returned to float rounding, ~1e-6 relative). The force is spread with the same weights over the
// same S_c, so the impulse is conserved exactly: the increments sum to h F / (rho dx^3) a component, to rounding. (An unnormalised
// kernel would drop 4.4e-4 of it; the sum is cheaper than the error.) The summation order is fixed: the sample's faces in index
// order, a face's elements in element order (sub-azimuth, blade, element), a gather by face so the device needs no atomics.
//
// REFUSED (invalid_argument), before anything is added: no blade or no polar, an invalid blade; omega not finite or negative; a spin
// not +-1; an axis or ref not unit length (1e-3) or not perpendicular (1e-3); psi, centre or h not finite; n_sub 0; rho or h not
// positive; and a kernel that leaves the grid: every element's reach (centre +- (R + 3 eps) on each axis) must lie inside [0, n dx].
//
// No libm: sin32, cos32, exp32 and sqrt. Subnormals are flushed as the discs' spread flushes them. The R/dx >= 20 rule is the case's
// (Interface's resolve()); this refuses only what it cannot compute.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct LineRotor {
    glm::vec3 centre{0.0f};         // m, in the grid's frame (its minimum corner at 0)
    glm::vec3 axis{0.0f, 1.0f, 0.0f};  // unit: the thrust's direction
    glm::vec3 ref{1.0f, 0.0f, 0.0f};   // unit, perpendicular to the axis: azimuth 0
    float spin = 1.0f;              // +1 or -1
    float omega = 0.0f;             // rad/s, >= 0
    float psi = 0.0f;               // rad, the blade angle at the step's start
    const BladeShape* blade = nullptr;
    const PolarTable* polar = nullptr;
};

// One element of one blade at one sub-azimuth.
struct LineElement {
    glm::vec3 pos{0.0f};     // grid frame
    glm::vec3 motion{0.0f};  // m, unit
    float r = 0.0f;          // m from the axis
    float dr = 0.0f;         // m of span it stands for
    float chord = 0.0f;
    float sin_beta = 0.0f;
    float cos_beta = 1.0f;
};

// The kernel's reach beyond a rotor's radius: 3 eps = 6 dx.
[[nodiscard]] inline float line_margin(float dx) noexcept { return 6.0f * dx; }

[[nodiscard]] Result<void> check_line(const LineRotor& rotor, const GridShape& shape, float rho, float h, uint32_t n_sub);

// The elements, in order sub-azimuth, blade, element, for a rotor already checked (refused-by-check inputs are not rechecked).
[[nodiscard]] std::vector<LineElement> line_elements(const LineRotor& rotor, float dx, float h, uint32_t n_sub);

// The air an element at `pos` feels: the kernel mean of the faces' velocities (THE KERNEL). `pos` must have its reach in the grid.
[[nodiscard]] glm::vec3 line_sample(const MacGrid& grid, glm::vec3 pos) noexcept;

struct LineReport {
    float thrust = 0.0f;  // N along the axis, all blades, the mean over the sub-azimuths
    float torque = 0.0f;  // N m about the axis against the blades' motion
    float power = 0.0f;   // W, torque * omega
};

// Adds the step's increments to du, dv and dw (the velocity layouts) and reports the rotor's load. `grid` is the air as the step
// begins; the spans are as long as its faces.
[[nodiscard]] Result<LineReport> spread_line(const LineRotor& rotor, const MacGrid& grid, float rho, float h, uint32_t n_sub,
                                             std::span<float> du, std::span<float> dv, std::span<float> dw);

}  // namespace spade::physics::airflow
