#pragma once

#include <span>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// ROTORS TO DISCS (P7a; the lead, 2026-10-13: the one rule, TD-9). A rotor's
// actuator disc in the airflow grid's frame, from its hub and thrust axis in
// the body frame, the body's pose in the world, and the grid's place in the
// world (world-aligned, its minimum corner at grid_origin_world):
//
//   centre  com + R_body hub - grid_origin_world
//   axis    -(R_body (orient_body +Y)): the disc pushes the air against the
//           rotor's thrust
//   thrust  the rotor's thrust, N
//   torque  -spin_dir Q: the air turns with the blades (actuator.hpp's rule)
//
// One-way and disc-only: the discs act on the air and nothing here reads the
// air back. Full P7 (actuator lines, the solid sampler) extends it. A body's
// rotor rows reach it through vehicles::rotor_discs(), which forms each
// rotor's thrust and torque as apply_rotors() does.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct DiscRotor {
    glm::vec3 hub_body{0.0f};                       // m, body frame, from the COM
    glm::quat orient_body{1.0f, 0.0f, 0.0f, 0.0f};  // local -> body; the thrust axis is orient_body * +Y
    float radius = 0.0f;                            // m
    float spin_dir = 0.0f;                          // +1 / -1 about the thrust axis; 0: no swirl
    float thrust = 0.0f;                            // N, along the thrust axis
    float torque = 0.0f;                            // N m, the shaft's drag torque magnitude Q, >= 0
};

struct DiscBody {
    glm::vec3 com_world{0.0f};                       // m
    glm::quat orient_world{1.0f, 0.0f, 0.0f, 0.0f};  // body -> world
};

// One rotor's disc. Refused (invalid_argument): a body pose or a rotor field
// that is not finite; an orientation, the body's or the rotor's, whose squared
// length is not within 1e-5 of 1 (check_disc()'s tolerance on the axis it
// turns); a spin other than -1, 0 or +1; a negative torque magnitude; and
// whatever check_disc() refuses against `grid` (the radius; the kernel's
// support leaving the grid).
[[nodiscard]] Result<ActuatorDisc> disc_from_rotor(const DiscRotor& rotor, const DiscBody& body,
                                                   glm::vec3 grid_origin_world, const GridShape& grid);

// out[i] = disc_from_rotor(rotors[i], ...), refused as it refuses, naming
// "rotor i", or when out.size() != rotors.size(). Allocation-free on success.
[[nodiscard]] Result<void> discs_from_rotors(std::span<const DiscRotor> rotors, const DiscBody& body,
                                             glm::vec3 grid_origin_world, const GridShape& grid,
                                             std::span<ActuatorDisc> out);

}  // namespace spade::physics::airflow
