#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "core/error.hpp"
#include "vehicles/propulsion_steady.hpp"

// ===========================================================================
// fc_link/autopilot_map.hpp -- what a flight controller's motor outputs mean to Spade's rotors (O140 phase 1; docs/design/physics/plans/2026-11-21-fc-physics.md
// section 3). Host-side, double and float arithmetic, never in the step. It lives in the link (tools/fc_link), not the engine: a table named for PX4's and
// ArduPilot's Quad X is autopilot domain (L7); it uses only the engine's public rotor_omega_for_duty().
//
// THE ORDER. PX4's x500 (4001_gz_x500, CA_ROTOR0..3) and ArduPilot's Quad X number their motors 1 front-right counter-clockwise, 2 rear-left
// counter-clockwise, 3 front-left clockwise, 4 rear-right clockwise. Spade's X500 and OpenFrame-5F models number their rotors 0 front-right (+1), 1
// front-left (-1), 2 rear-left (+1), 3 rear-right (-1), with spin_dir +1 counter-clockwise seen from above. So autopilot motor i drives Spade rotor
// kAutopilotQuadXToSpade[i], and the spins agree with no sign change. A model that orders its rotors otherwise is refused where the link is built, not
// here: the test pins the table to the X500 and OpenFrame-5F model files' geometry. (PX4 v1.17.0's airframe has a trapezoid, +-0.13 / 0.20 / 0.22 m, where
// `main` had +-0.174; the corners and the KM signs, which are all this table uses, are the same.)
//
// THE VALUE. A motor output is a duty, 0 to 1, of the propulsion chain (battery, ESC, motor, propeller): the pack's sag and the motor's limits are in the
// loop, which is what a motor model is for. rotor_omega_for_duty() turns a duty into the speed the chain settles at; this header turns the autopilot's
// own number into that duty. A disarmed output is zero, which is duty 0 (the rotor spins down on its time constant), not an idle speed.
//
// PX4's output range on the wire is not stated in the pages the design record read (phase 1's link test pins it from a SITL run); the proposal here is
// [0, 1], with a 1e-3 slack at each end for float noise, and a value outside that slack refused by name.
// ===========================================================================

namespace spade::fc_link {

inline constexpr std::size_t kAutopilotQuadXMotors = 4;

// autopilot motor index -> Spade rotor index (the X500 and OpenFrame-5F models' order).
inline constexpr std::array<uint32_t, kAutopilotQuadXMotors> kAutopilotQuadXToSpade{0u, 2u, 1u, 3u};

// A PX4 motor output (normalised, 0 to 1) to the chain's duty. Not finite, below -1e-3 or above 1 + 1e-3: invalid_argument naming the value. Inside the
// slack the result is clamped to [0, 1].
[[nodiscard]] Result<double> px4_output_to_duty(float output);

// An ArduPilot servo pulse (microseconds) to the chain's duty: (pwm - pwm_min) / (pwm_max - pwm_min), 0 at or below pwm_min (0 is "no pulse", a
// disarmed output), 1 at or above pwm_max. pwm_max <= pwm_min: invalid_argument.
[[nodiscard]] Result<double> ardupilot_pwm_to_duty(uint16_t pwm, uint16_t pwm_min = 1000, uint16_t pwm_max = 2000);

// Four duties in the AUTOPILOT's motor order to four rotor speed commands in SPADE's rotor order, through the chain: out[kAutopilotQuadXToSpade[i]] =
// rotor_omega_for_duty(chain, duty[i], density, soc).omega_cmd. A duty outside [0, 1] or a refused chain is rotor_omega_for_duty()'s refusal, with the
// autopilot motor index in the message.
[[nodiscard]] Result<std::array<float, kAutopilotQuadXMotors>> autopilot_duties_to_rotor_commands(
    const vehicles::PropulsionChain& chain, std::span<const double> duties, double density, double soc);

}  // namespace spade::fc_link
