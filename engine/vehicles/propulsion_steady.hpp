#pragma once

#include <cstdint>
#include <vector>

#include "core/error.hpp"
#include "vehicles/propeller.hpp"

// ===========================================================================
// propulsion_steady.hpp -- one motor, propeller and battery chain at
// equilibrium (docs/design/physics/plans/2026-10-03-drone-builder-physics.md
// section 7). A host function in double, never in the step.
//
// IT IS BUILT FROM THE STEP'S OWN FUNCTIONS, in their double overloads:
// motor_effective_resistance, motor_current, motor_torque (motor.hpp);
// battery_ocv and bus_solve (battery.hpp); propeller_advance_ratio,
// propeller_coefficient, propeller_thrust and propeller_torque
// (propeller.hpp). So a builder figure and a stepped run differ by fp32
// rounding only; the settling test in tests/test_propulsion_steady.cpp bounds
// that difference (DBP-52).
//
// FORWARD MODE (steady_state_at_duty). With `motors_on_bus` identical motors
// at one duty, find the shaft speed where the motor torque balances the
// propeller's: Q_m(omega) = Q_load(omega). Motor torque falls with omega and,
// for measured propellers, load torque rises, so the root is unique. It is
// found by bisection on [0, Kv d V_oc] with a fixed 100 iterations, so the
// answer is deterministic. A balance with no sign change on that bracket
// (a table whose torque falls with speed) sets propulsion_flags::no_bracket.
//
// INVERSE MODE (steady_state_for_thrust). Find the duty that gives a target
// thrust per motor (hover is weight / motors_on_bus), by bisection on [0, 1]
// with a fixed 60 iterations. A target above full-duty thrust returns full
// duty with propulsion_flags::unreachable.
//
// THE BATTERY IN STEADY STATE. The polarization voltage settles at R1 I_b, so
// R1 adds to the series resistance; the state of charge is an input and does
// not move.
// ===========================================================================

namespace spade::vehicles {

// One chain, in SI units, as the builder's part blocks compile it (spec
// section 14.2). Battery fields are per cell unless they say pack.
struct PropulsionChain {
    // Motor and ESC.
    double kv = 0.0;                       // rad/(s V); motor_kv_si() converts rpm/V
    double resistance = 0.0;               // ohm: winding, line to line, plus ESC on-resistance
    double no_load_current = 0.0;          // A
    double current_min = 0.0;              // A: 0, or -current_max with active braking
    double current_max = 0.0;              // A: the smaller of the motor's and the ESC's ratings
    double esc_current_total_max = 0.0;    // A: a 4-in-1 ESC's total over its channels (DBP-26); <= 0 for none
    double pole_pairs = 0.0;               // count; 0 drops the inductance term
    double inductance = 0.0;               // H, line to line; 0 drops the inductance term
    // Propeller.
    double diameter = 0.0;                 // m
    PropellerTable table;
    // Battery.
    uint32_t cells_series = 0;
    uint32_t cells_parallel = 0;
    double cell_resistance = 0.0;          // ohm
    double cell_cutoff_voltage = 0.0;      // V; <= 0 for none
    double pack_polarization_resistance = 0.0;  // ohm, R1 of the pack; 0 for none
    double pack_current_max = 0.0;         // A (c_rating x capacity); <= 0 for none
    std::vector<float> ocv_table;          // V per cell on a uniform SoC grid (battery_resample_ocv)
    // The bus.
    uint32_t motors_on_bus = 1;            // identical motors sharing the pack, 1 to kMaxBusMotors
};

// One equilibrium. Per-motor quantities unless they say pack.
struct SteadyPoint {
    double duty = 0.0;               // the clamped command
    double duty_scale = 1.0;         // the common scale the pack's limits applied
    double omega = 0.0;              // rad/s
    double advance_ratio = 0.0;      // J
    double thrust = 0.0;             // N
    double torque = 0.0;             // N m, the propeller's (equal to the motor's at the root)
    double current = 0.0;            // A, motor side
    double bus_voltage = 0.0;        // V, the pack's terminal voltage
    double pack_current = 0.0;       // A
    double power_electrical = 0.0;   // W, battery side, this motor's share
    double power_shaft = 0.0;        // W
    double efficiency = 0.0;         // shaft / electrical; 0 when no power flows
    uint32_t flags = 0;              // propulsion_flags
};

// An invalid chain (Kv, resistance or diameter not positive, an empty table,
// no cells, motors_on_bus outside 1..kMaxBusMotors) or non-finite inputs
// return a zero point with propulsion_flags::no_bracket.
[[nodiscard]] SteadyPoint steady_state_at_duty(const PropulsionChain& chain, double duty, double density,
                                               double v_axial, double soc) noexcept;

[[nodiscard]] SteadyPoint steady_state_for_thrust(const PropulsionChain& chain, double thrust, double density,
                                                  double v_axial, double soc) noexcept;

// The chain's small-signal time constant at the equilibrium for `duty`, s:
// J_r / -(d balance / d omega), where balance = Q_motor - Q_load with the bus
// solved at each omega. The slope is a central difference in double with a
// fixed step of 1e-4 of the equilibrium speed. It is what a first-order lag
// on shaft speed needs to match the chain for small changes about that point;
// the airframe compile fits the momentum-variant rotor's tau with it. 0 when
// the chain is invalid, the equilibrium is at rest or has no bracket,
// rotor_inertia is not positive, or the slope is not negative.
[[nodiscard]] double steady_state_time_constant(const PropulsionChain& chain, double duty, double density,
                                                double v_axial, double soc, double rotor_inertia) noexcept;

// ---------------------------------------------------------------------------
// THE THROTTLE MAP: a duty to a rotor's commanded speed, for a live run that
// drives the momentum-variant rotor row (vehicles/rotor.hpp) from a throttle.
//
//   omega_cmd = steady_state_at_duty(chain, duty, density, v_axial = 0, soc).omega
//
// rounded to float once. v_axial = 0 is exact for a held drone in still air,
// and it is the operating point the airframe compile fits the rotor at (hover
// in still air), so at the fit's hover duty the map gives the fit's hover
// speed. The row then lags omega toward omega_cmd on its fitted tau.
// ---------------------------------------------------------------------------

// One duty's command.
struct DutyCommand {
    float omega_cmd = 0.0f;  // rad/s: the equilibrium's speed, rounded once from double
    uint32_t flags = 0;      // propulsion_flags of the equilibrium
};

// The throttle map above. REFUSED by name (invalid_argument), checked in this
// order before any solve: a duty that is not finite and in [0, 1] (it is not
// clamped: an out-of-range duty is the caller's bug, not the ESC's), a density
// that is not finite and positive, a soc that is not finite and in [0, 1].
// Then refused when the equilibrium sets propulsion_flags::no_bracket (an
// invalid chain, as steady_state_at_duty() lists it, or a torque balance with
// no sign change) or propulsion_flags::unreachable.
//
// AT REST IS NOT AN ERROR. Duty 0, and a duty below the motor's breakaway
// (the drive current at rest cannot beat the no-load current, so the balance
// at omega = 0 is not positive), give omega_cmd 0 without no_bracket.
//
// THE OTHER FLAGS ARE OPERATING POINTS, NOT ERRORS. current_limited (a motor
// held at its current rating) and battery_current_limited (the pack held at
// its rating), like battery_cutoff, esc_total_limited and out_of_table, come
// back in `flags` with a valid omega_cmd: the chain really settles there. The
// caller ANNOUNCES them (a live run tells the pilot the motors or the pack
// are at their limit); it does not refuse them.
[[nodiscard]] Result<DutyCommand> rotor_omega_for_duty(const PropulsionChain& chain, double duty, double density,
                                                       double soc);

// THE ARM RULE: the duty a flight controller commands for a throttle.
//
//   disarmed   0
//   armed      idle + (1 - idle) throttle
//
// `idle` is the case's spin-when-armed duty, the flight controller's own
// setting (like ArduPilot's MOT_SPIN_ARM): explicit, with no default here.
// Zero throttle is exactly `idle` and full throttle exactly duty 1, for every
// idle in [0, 1), and rounding is monotonic, so the result never leaves
// [idle, 1] and always passes rotor_omega_for_duty()'s duty check.
//
// DISARM IS omega_cmd 0. A disarmed rotor is commanded duty 0, so
// rotor_omega_for_duty() gives omega_cmd 0 and omega spins down on the rotor
// row's fitted tau, as a real rotor coasts down. A rotor's `enabled`
// (RotorRow) is NEVER used to disarm: a disabled row is inert, and would drop
// its thrust and reaction torque in one substep.
//
// REFUSED by name (invalid_argument), whatever `armed` is, so a bad input is
// caught while disarmed too: a throttle that is not finite and in [0, 1],
// then an idle that is not finite with 0 <= idle < 1.
[[nodiscard]] Result<double> armed_duty(bool armed, double throttle, double idle);

}  // namespace spade::vehicles
