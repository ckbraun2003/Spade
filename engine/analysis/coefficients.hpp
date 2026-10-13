// The analysis's coefficients (Q6; DBC-030, DBC-031): a rotor's thrust,
// torque and power as coefficients in either convention, and the surface
// pressure coefficient, each value with the definition it was computed by.
// Pure functions, computed in double; display-free.
//
//   rotorcraft (hover; Caradonna-Tung's data): A = pi R^2, tip speed Omega R
//     C_T = T / (rho A (Omega R)^2)   C_Q = Q / (rho A (Omega R)^2 R)
//     C_P = P / (rho A (Omega R)^3)   FM  = C_T^(3/2) / (sqrt(2) C_P), hover only
//   propeller (under a wind; UIUC's data): n = Omega / 2 pi, D = 2 R
//     C_T = T / (rho n^2 D^4)   C_Q = Q / (rho n^2 D^5)   C_P = P / (rho n^3 D^5)
//     J = V / (n D)             eta = J C_T / C_P = T V / P, under a wind only
// with P = Q Omega. C_p = (p - p_ref) / (1/2 rho V_ref^2), V_ref the tip
// speed (rotorcraft) or the freestream (propeller).
//
// Owned by Interface (INT-6).

#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "analysis/case.hpp"
#include "core/error.hpp"

namespace spade::analysis {

enum class Convention : uint32_t { rotorcraft = 0, propeller = 1 };

// Rotorcraft in hover, propeller under a wind (Q6).
[[nodiscard]] Convention default_convention(FlightKind kind) noexcept;

// One rotor's measured loads.
struct RotorLoads {
    float thrust = 0.0f;  // N, along the rotor's axis
    float torque = 0.0f;  // N m, the shaft's
    float omega = 0.0f;   // rad/s
    float radius = 0.0f;  // m
};

struct Coefficient {
    std::string_view symbol;      // "C_T", "C_Q", "C_P", "FM", "J", "eta"
    std::string_view definition;  // how `value` was computed, for the reader to check (DBC-031)
    double value = 0.0;
};

// The rotor's coefficients in `convention`, `freestream` the wind's speed past
// the drone (m/s, 0 in still air). The figure of merit only in still air, the
// efficiency only under a wind. Refused: a stopped rotor, a radius or density
// not above 0, a freestream below 0.
[[nodiscard]] Result<std::vector<Coefficient>> rotor_coefficients(Convention convention, const RotorLoads& loads,
                                                                  float density, float freestream);

// C_p of `pressure` against `reference` (Pa). Refused without a reference speed.
[[nodiscard]] Result<double> pressure_coefficient(float pressure, float reference, float density,
                                                  float reference_speed);

// C_p's reference speed: the tip speed Omega R (rotorcraft) or the freestream (propeller).
[[nodiscard]] float reference_speed(Convention convention, float omega, float radius, float freestream) noexcept;

}  // namespace spade::analysis
