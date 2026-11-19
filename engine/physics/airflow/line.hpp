#pragma once

#include <array>
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
    // The kernel's width eps in cells (the Gaussian's e-folding length): 2 is the line's own and, held by Interface, every run's; a
    // diagnostic knob for a sweep that holds the PHYSICAL width fixed across resolutions. The support stays 6 cells (a wider eps is
    // cut earlier than 3 eps). Refused outside [1, 3].
    float eps_cells = 2.0f;
    // THE TIP/SMEARING CORRECTION (below), L72k's A1: ON by default since the flip commit (the polar fits of B1 are made with it
    // on). Off is every bit as the line was before A1.
    bool tip_correction = true;
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

// THE STENCIL (the device twin packs these, P4c). One axis of one component's stencil: the faces lo .. lo + n - 1 and their weights
// exp(-((x_i - pos) / eps)^2); n is at most kLineTaps (the 6-cell support, whatever eps_cells), sum is the weights' sum.
inline constexpr int kLineTaps = 13;

struct LineAxisTaps {
    int lo = 0;
    int n = 0;
    float sum = 0.0f;
    std::array<float, kLineTaps> w{};
};

// The stencil of component c (0 u, 1 v, 2 w) at a point: its three axes' taps, and sum, the product of the axes' sums. The weights are
// computed under the caller's flush scope (spread_line's own, when it computes them).
struct LineStencil {
    std::array<LineAxisTaps, 3> ax;
    float sum = 0.0f;
};

[[nodiscard]] LineStencil line_stencil(const GridShape& shape, uint32_t component, glm::vec3 pos, float eps_cells) noexcept;

// The kernel's reach beyond a rotor's radius: 3 eps = 6 dx.
[[nodiscard]] inline float line_margin(float dx) noexcept { return 6.0f * dx; }

[[nodiscard]] Result<void> check_line(const LineRotor& rotor, const GridShape& shape, float rho, float h, uint32_t n_sub);

// ACTUATOR LINES NEED A SUBGRID MODEL (L105a, 2026-11-19). check_line_damping(sgs_cs) refuses a line run whose fluid has FluidConfig::sgs_cs not above 0, by
// name: "actuator lines need a subgrid model (sgs_cs above 0)". Why: one x500 kit rotor at 5 mm and 0.18 tip cells a step grows a grid-scale
// instability on the rotor's axis from about step 4500 (the extreme v 17 -> 200 m/s, the downflow and the box's momentum fall away from the impulse put
// in) in the lean mode (nu 0, Cs 0) AND with the air's own nu 1.4607e-5 and Cs 0 (from about step 5500: molecular viscosity damps nothing at the
// cell); with Cs 0.17 (the film's) the same case is steady to step 8032 (record docs/design/physics/plans/2026-11-19-x500-kit-remeasure.md). A
// refusal, not a default: a default would change the physics without saying so. GpuFluid::create() calls it, before the device is touched, whenever
// it is given a LineSetup: lines are only ever stepped by the device. The CPU's spread_line() is called by tests, outside any fluid config.
[[nodiscard]] Result<void> check_line_damping(float sgs_cs);

// The elements, in order sub-azimuth, blade, element, for a rotor already checked (refused-by-check inputs are not rechecked).
[[nodiscard]] std::vector<LineElement> line_elements(const LineRotor& rotor, float dx, float h, uint32_t n_sub);

// The air an element at `pos` feels: the kernel mean of the faces' velocities (THE KERNEL). `pos` must have its reach in the grid.
[[nodiscard]] glm::vec3 line_sample(const MacGrid& grid, glm::vec3 pos, float eps_cells = 2.0f) noexcept;

// ---------------------------------------------------------------------------
// THE TIP/SMEARING CORRECTION (L72k, A1; .superpowers/sdd/2026-11-10-wake/a1-tip-design.md). Smearing the blade force over a Gaussian
// kernel gives the TRAILED vorticity a viscous (Lamb-Oseen) core, so an element feels less trailed-vortex downwash than a lifting
// line would: its angle of attack is too high at the tip. Meyer Forsting, Pirrung and Ramos-Garcia (Wind Energy Science 4, 369, 2019,
// eqs 14-23; the fast variant 5, 349, 2020) add the missing part: at element s, du_s = sum over trailed filaments v of f * u~, u~ the
// Biot-Savart velocity of the UNSMEARED filament, f = exp(-x^2 / eps^2) (eps this kernel's e-folding length, the same form as the
// paper's) with x the element's distance from the filament perpendicular to it. A filament is shed between each pair of elements (and
// at the root and the tip) with the circulation step along the blade, s (Gamma_(v-1) - Gamma_v) along its path, s the spin; Gamma =
// L' / (rho W), L' the section's lift (axial cos(phi) + tangent sin(phi)), from the CORRECTED air: a damped fixed point. DEVIATION
// (stated): Gamma is first filtered along the span by this kernel's own Gaussian (zero beyond the ends), the circulation the smeared
// field itself carries: the unfiltered one has element-to-element modes the fixed point amplifies without bound.
// DEVIATION from the paper, stated: its near-wake model (Pirrung's indicial functions) is replaced by DIRECT QUADRATURE of the
// helical filaments: each follows the air back from the blade by the rotation `wake_angle` (pi / 2, the paper's near-wake length),
// at its own radius, sinking along -axis by (max(ua, 0) / omega) per radian, as `segments` straight chords (graded: short at the
// blade, theta_j = wake (j / n)^2), each by the closed form of
// the straight filament (f is constant along a chord, so the quadrature error is the polyline's only); a hover wake is steady, so no
// time history is kept. Filaments farther than 3 eps (radially) from the element are skipped (f < 1.3e-4). Not in: the shed/bound
// part (the paper calls it negligible).
// ---------------------------------------------------------------------------
struct LineTipParams {
    int segments = 12;       // straight chords per filament over the near wake
    int iterations = 6;      // damped fixed-point passes
    float relax = 0.0f;      // the damping; <= 0: min(0.5, 1 / (1 + pi c_max / (4 eps))), set by the chord against the core
    float wake_angle = 0.5f * 3.14159265358979f;  // rad of blade rotation the filament follows
};

// f * (the Biot-Savart velocity of a straight filament from a to b of unit circulation, at p); eps_m the core's e-folding length in m
// (<= 0: no core factor, the plain law). Zero where p lies on the filament's line.
[[nodiscard]] glm::vec3 line_segment_induction(glm::vec3 p, glm::vec3 a, glm::vec3 b, float eps_m) noexcept;

// What line_tip_correct() builds once for a rotor and its blade groups: the chords' angles and their cosines and sines, the Gaussian
// that filters the circulation along the span (its half-width in elements, `reach`, and its sum), and the damping. The device twin
// (spike/airflow/line_chain.hpp) is handed these very floats, so the two cannot differ in them.
struct LineTipTables {
    std::size_t per = 0;  // elements a blade (a group)
    int reach = 0;
    float eps_m = 0.0f;
    float kern_sum = 0.0f;
    float relax = 0.0f;
    std::vector<float> th, ct, st;  // segments + 1 entries: the chord boundaries' angle, cos, sin
    std::vector<float> kern;        // reach + 1 entries
};
[[nodiscard]] LineTipTables line_tip_tables(const LineRotor& rotor, std::span<const LineElement> elements, float dx,
                                            const LineTipParams& params = {});

// Corrects `air` (each element's sampled air, in line_elements' order) in place; returns the last pass's largest change (m/s).
// A rotor that does not turn is left as it is.
[[nodiscard]] float line_tip_correct(const LineRotor& rotor, std::span<const LineElement> elements, float dx, float rho,
                                     std::span<glm::vec3> air, const LineTipParams& params = {});

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
