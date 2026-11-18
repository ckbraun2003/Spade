#pragma once

#include <array>
#include <cstdint>

#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// THE DOMAIN FACES' VELOCITY CONDITIONS (spec §2). Each fluid step a domain
// face is one of three kinds:
//   * fixed_velocity: a wall, or an open face the ambient flows in through.
//     Its normal velocity is prescribed; the pressure is Neumann there.
//   * pressure_outlet: an open face the ambient does not flow in through
//     (still air included). p = 0 on the face; the normal velocity is the
//     projection's, and apply_velocity_boundaries() leaves it alone. Flow into
//     the box through it is taken, as the air a rotor draws in must come in, and
//     the advection reads the ambient past the face (advect.cpp). (L90; until
//     then the face took a copy of its neighbour and refused inflow, once a
//     step, which wiped the entrainment and made a box hold a smaller share of
//     an impulse the smaller h was.)
//   * periodic: the last face copies the first.
// Tangential velocities have no stored boundary value: the viscous step and
// the sampler supply them (viscous.hpp, advect.hpp).
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

enum class FaceKind : uint8_t { fixed_velocity, pressure_outlet, periodic };

[[nodiscard]] FaceKind face_kind(const DomainBc& bc, Side side) noexcept;
[[nodiscard]] std::array<FaceKind, 6> face_kinds(const DomainBc& bc) noexcept;
[[nodiscard]] float fixed_normal_velocity(const DomainBc& bc, Side side) noexcept;

// Writes every domain face's normal velocity, axis by axis, x then y then z.
void apply_velocity_boundaries(MacGrid& g, const DomainBc& bc) noexcept;

}  // namespace spade::physics::airflow
