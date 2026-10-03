#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "core/error.hpp"

// ===========================================================================
// propeller.hpp -- the drone builder's propeller coefficient tier, as pure
// functions (docs/design/physics/plans/2026-10-03-drone-builder-physics.md
// section 5).
//
// THE MODEL. Thrust and torque from measured coefficients, by advance ratio,
// in the UIUC convention (n in rev/s, D in m):
//
//   n = omega / (2 pi)          J = V_axial / (n D)
//   T = C_T(J) rho n^2 D^4      Q = C_Q(J) rho n^2 D^5
//
// With n = 0 both are 0. V_axial is the axial inflow speed the rotor pass
// already computes. Ground effect multiplies thrust in the rotor pass, as it
// does today; it is not in these functions.
//
// TABLES. C_T and C_Q are float tables on one UNIFORM J grid, linearly
// interpolated: the lookup is index arithmetic, which keeps it identical on
// both backends. propeller_resample_table() puts authored [J, value] points on
// that grid, in double, rounding each value once. Outside [j_min, j_max] the
// end value holds and propulsion_flags::out_of_table is set. That includes
// descent (J < 0), where most measured tables stop: this tier does not model
// the vortex-ring or windmill-brake states that today's momentum-theory rotor
// does.
//
// FLOAT AND DOUBLE, ONE SOURCE, as in vehicles/motor.hpp.
//
// TOTAL (DBP-02); tests/test_propeller.cpp pins every degenerate input.
// ===========================================================================

namespace spade::vehicles {

// A propeller's coefficient tables on one uniform J grid. `ct` and `cq` have
// the same length, at least 2; their grid runs from j_min to j_max.
struct PropellerTable {
    float j_min = 0.0f;
    float j_max = 0.0f;
    std::vector<float> ct;
    std::vector<float> cq;
};

// Resample authored [J, C_T] and [J, C_Q] points onto one uniform grid of
// `count` points spanning the wider of the two authored ranges. Each set must
// hold at least 2 finite points with J strictly increasing. invalid_argument
// otherwise, or for count < 2.
[[nodiscard]] Result<PropellerTable> propeller_resample_table(std::span<const std::array<double, 2>> ct_points,
                                                              std::span<const std::array<double, 2>> cq_points,
                                                              uint32_t count);

// J = 2 pi V_axial / (omega D). omega <= 0, D <= 0 or any non-finite input -> 0;
// the thrust and torque are then 0 through n, not through J.
[[nodiscard]] float propeller_advance_ratio(float v_axial, float omega, float diameter) noexcept;
[[nodiscard]] double propeller_advance_ratio(double v_axial, double omega, double diameter) noexcept;

// A coefficient at J from a uniform-grid table over [j_min, j_max]. Outside the
// range the end value holds and propulsion_flags::out_of_table is set. An
// empty table, a non-finite J, or j_max <= j_min -> 0 (J flagged when it is
// not finite).
[[nodiscard]] float propeller_coefficient(std::span<const float> table, float j_min, float j_max, float j,
                                          uint32_t& flags) noexcept;
[[nodiscard]] double propeller_coefficient(std::span<const float> table, double j_min, double j_max, double j,
                                           uint32_t& flags) noexcept;

// C_T rho n^2 D^4, N, with n = omega / (2 pi). Any non-finite input -> 0.
[[nodiscard]] float propeller_thrust(float ct, float density, float omega, float diameter) noexcept;
[[nodiscard]] double propeller_thrust(double ct, double density, double omega, double diameter) noexcept;

// C_Q rho n^2 D^5, N m. Any non-finite input -> 0.
[[nodiscard]] float propeller_torque(float cq, float density, float omega, float diameter) noexcept;
[[nodiscard]] double propeller_torque(double cq, double density, double omega, double diameter) noexcept;

}  // namespace spade::vehicles
