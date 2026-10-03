#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// propulsion_flags -- the over-limit and out-of-range bits the drone builder's
// motor, battery and propeller functions report (DBP-05).
//
// A limit never damages a part: it clamps the quantity and sets one of these
// bits. The pure functions return them; the stateful rows (module-API stage 4)
// will store them in registered state so a viewer can read them (DBP-06).
// ---------------------------------------------------------------------------

namespace spade::vehicles::propulsion_flags {

inline constexpr uint32_t duty_clamped = 1u << 0;              // command outside [0, 1] or not finite
inline constexpr uint32_t current_limited = 1u << 1;           // motor current held at its limit
inline constexpr uint32_t speed_clamped = 1u << 2;             // shaft speed held at 0 (one-directional ESC)
inline constexpr uint32_t out_of_table = 1u << 3;              // advance ratio outside the propeller table
inline constexpr uint32_t battery_current_limited = 1u << 4;   // pack current held at its rating
inline constexpr uint32_t battery_cutoff = 1u << 5;            // terminal voltage held at cutoff
inline constexpr uint32_t battery_empty = 1u << 6;             // state of charge held at 0
inline constexpr uint32_t unreachable = 1u << 7;               // solver: target above full-duty thrust
inline constexpr uint32_t no_bracket = 1u << 8;                // solver: torque balance has no sign change

}  // namespace spade::vehicles::propulsion_flags
