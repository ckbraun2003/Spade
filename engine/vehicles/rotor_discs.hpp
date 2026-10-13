#pragma once

#include <span>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "state/layout.hpp"
#include "vehicles/rotor.hpp"

// ---------------------------------------------------------------------------
// A BODY'S ROTOR ROWS AS AIRFLOW DISCS (P7a). The adapter a live run calls
// each fluid step: rows[i] becomes out[i] through physics/airflow/
// rotor_discs.hpp's rule, with
//
//   thrust  k_T w^2 and the torque magnitude k_Q w^2, from one w^2, as
//           apply_rotors() forms them (rotor.cpp, step 3): the static values,
//           exact in hover, where the inflow correction is 1, and not
//           inflow-corrected in climb or descent (P7a is one-way and
//           disc-only; full P7 takes the force pass's own values);
//   pose    the body row's pos and orient (COM and principal axes) and the
//           row's local_pos and local_orient, whose thrust axis is
//           kRotorLocalThrustAxis (+Y), the rule's;
//   inert   a disabled row (enabled == 0) is a zero-thrust, zero-torque disc
//           in its place, so indices stay aligned.
//
// `rows` are one body's (the caller selects by body_slot), and `body` is that
// body's row. Refused as discs_from_rotors() refuses, naming "rotor i".
// Allocation-free on success.
// ---------------------------------------------------------------------------
namespace spade::vehicles {

[[nodiscard]] Result<void> rotor_discs(std::span<const RotorRow> rows, const BodyState& body,
                                       glm::vec3 grid_origin_world, const physics::airflow::GridShape& grid,
                                       std::span<physics::airflow::ActuatorDisc> out);

}  // namespace spade::vehicles
