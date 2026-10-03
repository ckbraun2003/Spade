#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "core/error.hpp"

// ===========================================================================
// battery.hpp -- the drone builder's battery and its shared bus, as pure
// functions (docs/design/physics/plans/2026-10-03-drone-builder-physics.md
// section 4).
//
// THE MODEL. A Thevenin pack: an open-circuit source, a series resistance R0
// and one RC branch (R1, C1) for slow sag. S cells in series, P in parallel:
//
//   V_oc = S OCV(SoC)              OCV a per-cell table on a uniform SoC grid
//   R0   = S R_cell / P            battery_pack_resistance()
//   V_t  = V_oc - V_1 - R0 I_b     battery_terminal_voltage()
//   V_1 <- V_1 beta + R1 I_b (1 - beta),  beta = exp(-h / (R1 C1))
//   SoC <- SoC - I_b h / (3600 capacity_Ah)
//
// THE BUS. Every motor on a vehicle shares the pack. The ESC is an
// average-value converter, so motor k draws d_k I_k from the battery, and for
// the motors not at a current limit the bus voltage has a closed form
// (bus_solve()). The pack's current rating and its cutoff voltage clamp
// through ONE common duty scale for the whole vehicle, found by a bisection
// with a fixed 24 iterations, so both backends will run the same operations.
//
// FLOAT AND DOUBLE, ONE SOURCE, as in vehicles/motor.hpp: the double
// overloads serve the steady-state solver and Kat's fit, the float ones the
// step (module-API stage 4). OCV and propeller tables are float data in both,
// so the solver reads exactly the values a stepped run reads.
//
// TOTAL (DBP-02); tests/test_battery.cpp pins every degenerate input.
// ===========================================================================

namespace spade::vehicles {

// --- the pack --------------------------------------------------------------

// S R_cell / P, ohm. Zero counts or a non-positive or non-finite resistance -> 0.
[[nodiscard]] float battery_pack_resistance(uint32_t cells_series, uint32_t cells_parallel,
                                            float cell_resistance) noexcept;
[[nodiscard]] double battery_pack_resistance(uint32_t cells_series, uint32_t cells_parallel,
                                             double cell_resistance) noexcept;

// Per-cell open-circuit voltage at a state of charge, from a table on a
// uniform SoC grid over [0, 1], linearly interpolated. SoC is held to [0, 1]
// (non-finite -> 0). An empty table -> 0; a one-point table -> that point.
[[nodiscard]] float battery_ocv(float soc, std::span<const float> table) noexcept;
[[nodiscard]] double battery_ocv(double soc, std::span<const float> table) noexcept;

// Resample authored [SoC, V] points onto a uniform grid of `count` points over
// [0, 1], in double, each rounded to float once. Outside the authored range
// the end value holds. invalid_argument for: count < 2, fewer than 2 points,
// a non-finite value, a SoC outside [0, 1] or not strictly increasing, or a
// non-positive voltage.
[[nodiscard]] Result<std::vector<float>> battery_resample_ocv(std::span<const std::array<double, 2>> points,
                                                              uint32_t count);

// beta = exp(-h / (R1 C1)), float only: it feeds state, so it comes from
// core/fp32_math (TD-3). R1 C1 <= 0 or non-finite -> 0 (no memory: V_1 follows
// R1 I_b at once); h <= 0 or non-finite -> 1 (no time passes).
[[nodiscard]] float battery_rc_beta(float h, float r1, float c1) noexcept;

// V_1 beta + R1 I_b (1 - beta), V. Any non-finite input -> 0.
[[nodiscard]] float battery_polarization_step(float v1, float current, float r1, float beta) noexcept;
[[nodiscard]] double battery_polarization_step(double v1, double current, double r1, double beta) noexcept;

// SoC - I_b h / (3600 capacity_Ah), held to [0, 1]. Reaching 0 under load sets
// propulsion_flags::battery_empty. A non-positive or non-finite capacity or h
// leaves SoC unchanged (held to [0, 1]); a non-finite SoC reads as 0.
[[nodiscard]] float battery_soc_step(float soc, float current, float h, float capacity_ah,
                                     uint32_t& flags) noexcept;
[[nodiscard]] double battery_soc_step(double soc, double current, double h, double capacity_ah,
                                      uint32_t& flags) noexcept;

// V_oc - V_1 - R0 I_b, V. Any non-finite input -> 0.
[[nodiscard]] float battery_terminal_voltage(float v_oc, float v1, float r0, float current) noexcept;
[[nodiscard]] double battery_terminal_voltage(double v_oc, double v1, double r0, double current) noexcept;

// --- the bus ---------------------------------------------------------------

// One motor on the bus, as the bus solve sees it. `duty` is already clamped
// (esc_clamp_duty); `back_emf` is omega/Kv at the start of the substep;
// `r_eff` is motor_effective_resistance() there. A motor with r_eff <= 0 or a
// non-finite field draws nothing.
template <typename T>
struct BusMotorT {
    T duty = T(0);
    T back_emf = T(0);       // V
    T r_eff = T(0);          // ohm
    T current_min = T(0);    // A: 0, or -current_max with active braking
    T current_max = T(0);    // A
};
using BusMotor = BusMotorT<float>;
using BusMotorD = BusMotorT<double>;

template <typename T>
struct BusResultT {
    T bus_voltage = T(0);      // V, the pack's terminal voltage
    T battery_current = T(0);  // A, sum of d_k I_k
    T duty_scale = T(1);       // the common scale the battery clamps applied, in [0, 1]
    uint32_t flags = 0;        // current_limited, battery_current_limited, battery_cutoff
};
using BusResult = BusResultT<float>;
using BusResultD = BusResultT<double>;

// The most motors one bus solve takes; a larger span is refused with a zero
// result. The limit is what lets the solve track its clamped motors without
// allocating.
inline constexpr std::size_t kMaxBusMotors = 64;

// Solve the shared bus. `v_source` is V_oc - V_1; `r_series` is R0 (the step)
// or R0 + R1 (the steady state, where V_1 = R1 I_b). Writes each motor's
// current, after its clamp, into `currents`, which must be as long as
// `motors`. `battery_current_max` and `cutoff_voltage` are the pack's limits;
// pass a non-finite or non-positive value for "none".
//
// For the motors not at a limit:
//   V = (v_source + R0 sum(d_i e_i / R_i) - R0 sum(d_j I_j)) / (1 + R0 sum(d_i^2 / R_i))
// with i over free motors and j over clamped ones, both in declaration order.
// A motor whose current leaves [current_min, current_max] is clamped, moves to
// the fixed sum, and the solve repeats, at most once per motor. A clamped
// motor stays clamped within one solve.
//
// If the pack's current exceeds its rating, or V falls below cutoff, every
// duty is scaled by one common factor in [0, 1], found by 24 bisection steps:
// the largest factor that meets both limits. At factor 0 no current flows;
// if V is still below cutoff there, the battery is simply flat.
[[nodiscard]] BusResult bus_solve(float v_source, float r_series, float battery_current_max, float cutoff_voltage,
                                  std::span<const BusMotor> motors, std::span<float> currents) noexcept;
[[nodiscard]] BusResultD bus_solve(double v_source, double r_series, double battery_current_max,
                                   double cutoff_voltage, std::span<const BusMotorD> motors,
                                   std::span<double> currents) noexcept;

// The same solve with a 4-in-1 ESC's total-current rating (DBP-26): the sum of
// the motor-side currents, sum |I_k|, is held at `esc_current_total_max`
// through the SAME common duty scale and the same 24-step bisection, and sets
// propulsion_flags::esc_total_limited. A non-finite or non-positive value
// means "none". The six-argument overloads above are this with "none".
[[nodiscard]] BusResult bus_solve(float v_source, float r_series, float battery_current_max, float cutoff_voltage,
                                  float esc_current_total_max, std::span<const BusMotor> motors,
                                  std::span<float> currents) noexcept;
[[nodiscard]] BusResultD bus_solve(double v_source, double r_series, double battery_current_max,
                                   double cutoff_voltage, double esc_current_total_max,
                                   std::span<const BusMotorD> motors, std::span<double> currents) noexcept;

}  // namespace spade::vehicles
