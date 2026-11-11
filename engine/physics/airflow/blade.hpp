#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "core/error.hpp"

// ---------------------------------------------------------------------------
// THE BLADE'S SECTION LOAD AND THE GRID-FREE BLADE-ELEMENT-MOMENTUM REFERENCE (P4a, L61; the design note
// .superpowers/sdd/2026-11-10-wake/p4-actuator-line-design.md). What the actuator line (P4b) takes at each element and what
// validation (V1) compares it with, before a grid is involved.
//
// THE ELEMENT. A blade section at radius r sees the air at (ua, wt): ua the air's speed THROUGH the rotor, positive along
// the direction opposite the thrust (the inflow: a hovering rotor's induced flow), wt the speed of the section THROUGH the
// air along its motion, omega r less the air's own swirl along it. |W|^2 = ua^2 + wt^2; the inflow angle phi has
// cos phi = wt / |W| and sin phi = ua / |W|; the blade's pitch angle beta (from the rotor plane; the UIUC geometry's beta)
// makes alpha = beta - phi, whose sine and cosine come out algebraically (sin alpha = sin beta cos phi - cos beta sin phi,
// cos alpha = cos beta cos phi + sin beta sin phi). No angle is ever formed: fp32_math has no atan, and the polar is
// tabulated on the angle's DIAMOND ANGLE (below), so the same lookup runs on the device.
//
//   per unit span, q = 1/2 rho |W|^2 c:   lift q Cl, drag q Cd,
//   axial   = lift cos phi - drag sin phi    the force on the blade along the thrust,
//   tangent = lift sin phi + drag cos phi    the force on the blade against its motion.
// The air takes the same two forces the other way (the line spreads them); the torque the blade feels is r * tangent.
//
// THE DIAMOND ANGLE of (c, s), a unit direction: s / (|c| + |s|) for c >= 0, and sign(s) (2 - |s| / (|c| + |s|)) for
// c < 0. Monotone in the angle over (-pi, pi), -2 to 2, from +-0 at 0: a polar is a table over it, found by a binary search and
// read by linear interpolation, with the ends held. Its jump at +-pi is the angle's.
//
// THE POLAR. alpha_deg increasing, cl and cd the same length, at least 2 points, every value finite. The estimated polar
// (design note section 1, written before any comparison) is what a station without a measured one takes: attached for alpha in
// [alpha_0 - 14, alpha_0 + 14] degrees (-17 to 11), C_l = 5.5 (alpha - alpha_0) in radians (C_l 1.34 at 11 degrees), C_d = 0.03 + 0.02 C_l^2; blended
// linearly in alpha over 14 degrees more on each side to the flat plate, C_l = 0.9 sin 2 alpha, C_d = 0.03 + 1.77 sin^2 alpha,
// which holds from there around the whole circle. Labelled "estimated" wherever it shows.
//
// THE REFERENCE, the grid-free blade-element-momentum solve with Prandtl's tip loss, is tests/bemt_reference.hpp: Prandtl's
// factor needs acos, which is libm and barred under engine/. It takes these sections and this polar.
//
// No libm: the tables are built on the host with fp32_math's sin32 and cos32, and `std::sqrt` is the CPU's.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

// The diamond angle of the unit direction (c, s). NaN in, NaN out.
[[nodiscard]] float diamond_angle(float c, float s) noexcept;

struct PolarTable {
    std::vector<float> key;  // diamond angles, strictly increasing
    std::vector<float> cl;
    std::vector<float> cd;
};

// A polar from alpha in degrees. Refused (invalid_argument): fewer than 2 points, lengths that differ, a value not finite,
// alpha not strictly increasing, or alpha outside [-180, 180].
[[nodiscard]] Result<PolarTable> make_polar(std::span<const float> alpha_deg, std::span<const float> cl,
                                            std::span<const float> cd);

// The estimated polar's two free parameters: where lift is zero (-3 degrees) and a scale on all drag (1). A calibration moves only
// these (design note section 1; Test/Docs' V1: once, to the static point).
struct EstimatedPolarShape {
    float alpha0_deg = -3.0f;
    float cd_scale = 1.0f;
};

// The estimated polar above, every degree through +-20 and every 5 outside, to +-180.
[[nodiscard]] PolarTable estimated_polar(const EstimatedPolarShape& shape = {});

struct SectionCoefficients {
    float cl = 0.0f;
    float cd = 0.0f;
};

// The coefficients at the angle whose cosine and sine are (cos_alpha, sin_alpha).
[[nodiscard]] SectionCoefficients polar_at(const PolarTable& polar, float cos_alpha, float sin_alpha) noexcept;

struct SectionLoad {
    float axial = 0.0f;    // N per metre of span, on the blade along the thrust
    float tangent = 0.0f;  // N per metre of span, on the blade against its motion
};

// One section's load (THE ELEMENT above): chord c (m), the pitch angle's sine and cosine, the air's ua and wt (m/s), rho.
// Zero where |W| is 0.
[[nodiscard]] SectionLoad section_load(const PolarTable& polar, float chord, float sin_beta, float cos_beta, float ua,
                                       float wt, float rho) noexcept;

// A blade's stations, hub to tip.
struct BladeStation {
    float r = 0.0f;         // m from the axis
    float chord = 0.0f;     // m
    float beta_deg = 0.0f;  // pitch angle from the rotor plane, degrees
};

struct BladeShape {
    uint32_t blades = 0;
    float radius = 0.0f;  // m, the tip
    std::vector<BladeStation> stations;  // r strictly increasing; the first is the hub's cut-out
};

// Refused (invalid_argument), naming the field: no blades; a radius not positive and finite; fewer than 2 stations; r not
// strictly increasing, below 0 or past the radius; a chord not positive; a pitch not finite.
[[nodiscard]] Result<void> check_blade(const BladeShape& blade);

}  // namespace spade::physics::airflow
