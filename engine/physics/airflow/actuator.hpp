#pragma once

#include <cstdint>
#include <span>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// AN ACTUATOR DISC (spec §3.1): one step's impulse thrust * h along `axis`,
// carried by 48 points and spread with Peskin's 4-point kernel onto the
// velocity increments a fluid step adds. A gather, so the GPU twin needs no
// atomics.
//
// THE SWIRL (the demonstration plan's P1): `torque` goes to the air as a
// tangential force at each point, torque / (48 r) on its ring of radius r,
// along axis x (point - centre). That is constant circulation, the ideal
// rotor under the disc's uniform thrust loading, and the points' sum of
// r |f| is the torque exactly. The kernel keeps first moments, so the air's
// angular momentum about the centre gains h * torque along `axis` and the
// swirl adds no net force. A torque of 0 spreads exactly what the thrust-only
// disc did, bit for bit.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct ActuatorDisc {
    glm::vec3 centre{0.0f};             // metres, in the grid's frame
    glm::vec3 axis{0.0f, -1.0f, 0.0f};  // unit: the direction the disc pushes the air
    float radius = 0.0f;                // metres
    float thrust = 0.0f;                // newtons: the force the air takes along `axis`
    // newton-metres: the torque the air takes about `axis`, by the right-hand
    // rule. The air swirls with the blades, so a rotor spinning s (+1 or -1)
    // about its thrust axis -axis gives the air -s Q here (the coupler's rule).
    float torque = 0.0f;
};

inline constexpr uint32_t kDiscRings = 3;
inline constexpr uint32_t kDiscAngles = 16;
inline constexpr uint32_t kDiscPoints = kDiscRings * kDiscAngles;

// Peskin's 4-point kernel, in cells.
[[nodiscard]] float peskin4(float r) noexcept;

// Refused (invalid_argument): a radius not positive and finite; a thrust or a
// torque not finite; an axis not of unit length; a kernel support (radius +
// 2 dx about the centre) that leaves the grid.
[[nodiscard]] Result<void> check_disc(const ActuatorDisc& disc, const GridShape& shape);

// Adds one step's increments for a checked disc.
void spread_actuator(const ActuatorDisc& disc, const GridShape& shape, float rho, float h, std::span<float> du,
                     std::span<float> dv, std::span<float> dw) noexcept;

}  // namespace spade::physics::airflow
