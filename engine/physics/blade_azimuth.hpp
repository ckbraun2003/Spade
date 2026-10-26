#pragma once

#include <cstddef>
#include <span>

#include "core/validate.hpp"  // finite
#include "vehicles/rotor.hpp"

namespace spade::physics {

// ---------------------------------------------------------------------------
// THE BLADE AZIMUTH psi (P11, docs/design/physics/plans/2026-10-12-airflow-
// drone-demonstration-plan.md): one float per rotor, in radians, in
// [0, 2 pi). posed_parts() turns a rotor's parts by R_y(s_k psi_k)
// (vehicles/part_pose.cpp), so psi grows with omega >= 0 whatever the hand and
// the hand is s_k's. P4's actuator line will put blade b at
// s_k psi + 2 pi b / B about the hub's +Y, so a drawn blade and the line
// coincide.
//
// ONE RULE (TD-9), each step, after the rotors' own step:
//     psi <- psi + omega h, then psi -= kAzimuthTwoPi while psi >= it,
// with omega the row's omega AFTER the step -- the value its forces used
// (vehicles/rotor.cpp lags omega, then takes thrust and torque from it). A
// disabled rotor holds its psi: its forces used nothing, and its row keeps a
// stale omega. + - * only (TD-3: no fmod); the wrap keeps psi inside sin32's
// 1-ulp domain, which part_pose.hpp leaves to the caller.
//
// HELD ON THE HOST, outside RotorRow and registered state (every existing
// golden holds), nothing read back: a plain Simulation (the editor's Run) and
// LiveRun call the same function. It starts at 0, the authored pose. No
// allocation, no clock.
//
// REFUSED, psi held and false returned (the caller flags it): psi outside
// [0, 2 pi) or not finite, omega negative or not finite, h negative or not
// finite, or omega h above kAzimuthMaxStep. The bound keeps the wrap's loop
// to at most 11 subtractions, so no input can spin it.
// ---------------------------------------------------------------------------

// The float nearest 2 pi (0x40C90FDB), pinned.
inline constexpr float kAzimuthTwoPi = 6.28318548f;

// About 10 turns in one step, far past any rotor (40,000 rpm at 2 ms is 8.4 rad).
inline constexpr float kAzimuthMaxStep = 64.0f;

[[nodiscard]] inline bool advance_blade_azimuth(float& psi, float omega, float h) noexcept {
    if (!finite(psi) || !(psi >= 0.0f) || !(psi < kAzimuthTwoPi)) return false;
    if (!finite(omega) || !(omega >= 0.0f) || !finite(h) || !(h >= 0.0f)) return false;
    const float step = omega * h;
    if (!finite(step) || !(step <= kAzimuthMaxStep)) return false;
    float next = psi + step;
    while (next >= kAzimuthTwoPi) next -= kAzimuthTwoPi;
    psi = next;
    return true;
}

// Every rotor, in row order: psi[k] advances by rows[k]'s omega unless the row
// is disabled. A refused rotor holds its psi and the others still advance;
// false if any was refused, or if the spans differ in length (nothing moves).
[[nodiscard]] inline bool advance_blade_azimuths(std::span<float> psi, std::span<const vehicles::RotorRow> rows,
                                                 float h) noexcept {
    if (psi.size() != rows.size()) return false;
    bool ok = true;
    for (std::size_t k = 0; k < rows.size(); ++k) {
        if (rows[k].enabled == 0u) continue;
        if (!advance_blade_azimuth(psi[k], rows[k].omega, h)) ok = false;
    }
    return ok;
}

}  // namespace spade::physics
