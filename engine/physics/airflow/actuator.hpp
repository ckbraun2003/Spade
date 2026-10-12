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
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct ActuatorDisc {
    glm::vec3 centre{0.0f};             // metres, in the grid's frame
    glm::vec3 axis{0.0f, -1.0f, 0.0f};  // unit: the direction the disc pushes the air
    float radius = 0.0f;                // metres
    float thrust = 0.0f;                // newtons: the force the air takes along `axis`
};

inline constexpr uint32_t kDiscRings = 3;
inline constexpr uint32_t kDiscAngles = 16;
inline constexpr uint32_t kDiscPoints = kDiscRings * kDiscAngles;

// Peskin's 4-point kernel, in cells.
[[nodiscard]] float peskin4(float r) noexcept;

// Refused (invalid_argument): a radius not positive and finite; a thrust not
// finite; an axis not of unit length; a kernel support (radius + 2 dx about
// the centre) that leaves the grid.
[[nodiscard]] Result<void> check_disc(const ActuatorDisc& disc, const GridShape& shape);

// Adds one step's increments for a checked disc.
void spread_actuator(const ActuatorDisc& disc, const GridShape& shape, float rho, float h, std::span<float> du,
                     std::span<float> dv, std::span<float> dw) noexcept;

}  // namespace spade::physics::airflow
