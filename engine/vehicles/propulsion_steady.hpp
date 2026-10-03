#pragma once

#include <cstdint>
#include <vector>

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

}  // namespace spade::vehicles
