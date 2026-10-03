#pragma once

#include <cstdint>

// ===========================================================================
// motor.hpp -- the drone builder's motor and ESC, as pure functions
// (docs/design/physics/plans/2026-10-03-drone-builder-physics.md section 3).
//
// THE MODEL. An average-value brushless DC motor behind an ESC. The ESC
// applies a duty d in [0, 1] to the bus voltage. All quantities are SI:
//
//   Kv   speed constant, rad/(s V); motor_kv_si() converts a datasheet's rpm/V
//   Kt   torque constant, N m/A, = 1/Kv; the back-EMF constant Ke equals Kt
//   R    winding resistance, line to line, plus the ESC's on-resistance, ohm
//   I0   no-load current, A: friction and iron loss, as a torque Kt I0
//   J_r  polar inertia of the motor bell plus the propeller, kg m^2
//
//   I   = (d V_bus - omega/Kv) / R_eff(omega)         motor_current()
//   Q_m = Kt (I - I0), never turning a stopped shaft  motor_torque()
//   J_r d(omega)/dt = Q_m - Q_load
//
// PHASE INDUCTANCE (DBP-18). At speed the winding's reactance X = p omega L
// is no longer small next to R (p pole pairs, L the phase inductance). With
// the drive in phase with the back-EMF, only the in-phase current makes
// torque, which is the resistive model with R replaced by
//
//   R_eff(omega) = R + X^2 / R                        motor_effective_resistance()
//
// Pass L = 0 (or p = 0) for the resistive model.
//
// THE SHAFT UPDATE (DBP-13). Hold the load torque and R_eff at their
// start-of-substep values. The electrical part is then linear in omega, with
// time constant tau_m = J_r R_eff Kv^2, and its exact solution is
//
//   omega_inf = Kv (d V_bus - R_eff (I0 + Q_load Kv))   motor_speed_target()
//   omega    += (omega_inf - omega) * alpha             motor_speed_step()
//   alpha     = 1 - exp(-h / tau_m)                     motor_alpha()
//
// alpha is the rotor lag's own factor (vehicles/rotor.hpp section 5), computed
// with core/fp32_math, so no libm transcendental reaches state (TD-3). When
// the current sits at a limit the torque is constant, and the update is the
// linear one in motor_speed_step_limited().
//
// FLOAT AND DOUBLE. Every algebraic function has a float overload (the step's,
// at module-API stage 4) and a double overload (the steady-state solver's,
// and Kat's fit's). Both come from one template, compiled in motor.cpp under
// spade_vehicles' strict floating-point flags, so they are the same
// operations in two precisions. A builder figure computed in double therefore
// differs from a stepped run by fp32 rounding only. The settling test in
// tests/test_propulsion_steady.cpp bounds that difference (DBP-52).
//
// TOTAL (DBP-02). Every function returns a finite, defined value for zero,
// negative, non-finite and overflowing inputs; each case is pinned by
// tests/test_motor.cpp.
// ===========================================================================

namespace spade::vehicles {

// The ESC's duty clamp (DBP-10): a command outside [0, 1] is held at the
// nearer end, and a non-finite one reads as 0. Either sets
// propulsion_flags::duty_clamped.
[[nodiscard]] float esc_clamp_duty(float duty, uint32_t& flags) noexcept;
[[nodiscard]] double esc_clamp_duty(double duty, uint32_t& flags) noexcept;

// rpm/V (datasheet) -> rad/(s V). Non-finite or negative input -> 0.
[[nodiscard]] float motor_kv_si(float kv_rpm_per_volt) noexcept;
[[nodiscard]] double motor_kv_si(double kv_rpm_per_volt) noexcept;

// R + (p omega L)^2 / R, ohm. R <= 0 or non-finite -> 0, which every reader
// treats as "no motor". A non-finite p, L or omega drops the inductance term
// (-> R). A result that overflows gives the largest finite value: the motor
// then carries no current.
[[nodiscard]] float motor_effective_resistance(float resistance, float pole_pairs, float inductance,
                                               float omega) noexcept;
[[nodiscard]] double motor_effective_resistance(double resistance, double pole_pairs, double inductance,
                                                double omega) noexcept;

// (d V_bus - omega/Kv) / R_eff, A, UNCLAMPED: the caller clamps through
// motor_clamp_current(), so the bus solve can see which motors hit a limit.
// Kv <= 0 or R_eff <= 0 or any non-finite input -> 0.
[[nodiscard]] float motor_current(float duty, float v_bus, float omega, float kv, float r_eff) noexcept;
[[nodiscard]] double motor_current(double duty, double v_bus, double omega, double kv, double r_eff) noexcept;

// Clamp a current to [i_lo, i_max]; sets propulsion_flags::current_limited in
// `flags` when it clamps. A non-finite current reads as 0.
[[nodiscard]] float motor_clamp_current(float current, float i_lo, float i_max, uint32_t& flags) noexcept;
[[nodiscard]] double motor_clamp_current(double current, double i_lo, double i_max, uint32_t& flags) noexcept;

// Kt (I - I0), N m, with Kt = 1/Kv. At omega <= 0 a negative torque is
// returned as 0: friction holds a stopped shaft, it never turns it backwards.
// Kv <= 0 or any non-finite input -> 0.
[[nodiscard]] float motor_torque(float current, float no_load_current, float kv, float omega) noexcept;
[[nodiscard]] double motor_torque(double current, double no_load_current, double kv, double omega) noexcept;

// tau_m = J_r R_eff Kv^2, s. Any non-positive or non-finite input -> 0, which
// motor_alpha() reads as instant tracking.
[[nodiscard]] float motor_time_constant(float inertia, float r_eff, float kv) noexcept;
[[nodiscard]] double motor_time_constant(double inertia, double r_eff, double kv) noexcept;

// alpha = 1 - exp(-h/tau), float only: it feeds state, so it comes from
// core/fp32_math (TD-3). It is vehicles::rotor_lag_alpha(), whose totality it
// inherits: h <= 0 -> 0 (nothing moves), tau <= 0 -> 1 (instant tracking).
[[nodiscard]] float motor_alpha(float h, float tau) noexcept;

// omega_inf = Kv (d V_bus - R_eff (I0 + Q_load Kv)), rad/s, the speed the
// motor settles at under a held load. It may be negative (the load wins); the
// step clamps omega, not this. Kv <= 0 or any non-finite input -> 0.
[[nodiscard]] float motor_speed_target(float duty, float v_bus, float kv, float r_eff, float no_load_current,
                                       float load_torque) noexcept;
[[nodiscard]] double motor_speed_target(double duty, double v_bus, double kv, double r_eff,
                                        double no_load_current, double load_torque) noexcept;

// omega + (omega_inf - omega) alpha, clamped at 0 (the ESC is one-directional;
// sets propulsion_flags::speed_clamped). A non-finite result reads as 0.
[[nodiscard]] float motor_speed_step(float omega, float omega_inf, float alpha, uint32_t& flags) noexcept;
[[nodiscard]] double motor_speed_step(double omega, double omega_inf, double alpha, uint32_t& flags) noexcept;

// The current-limited branch: omega + h (Q_m - Q_load) / J_r, clamped at 0
// with the same flag. J_r <= 0, h <= 0 or any non-finite input -> omega
// unchanged (0 if omega itself is not finite).
[[nodiscard]] float motor_speed_step_limited(float omega, float drive_torque, float load_torque, float inertia,
                                             float h, uint32_t& flags) noexcept;
[[nodiscard]] double motor_speed_step_limited(double omega, double drive_torque, double load_torque,
                                              double inertia, double h, uint32_t& flags) noexcept;

}  // namespace spade::vehicles
