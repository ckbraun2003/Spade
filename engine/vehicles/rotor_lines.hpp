#pragma once

#include <span>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "physics/airflow/blade.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "state/layout.hpp"
#include "vehicles/rotor.hpp"

// ---------------------------------------------------------------------------
// A BODY'S ROTOR ROWS AS ACTUATOR LINES (L86i). The adapter a live run calls each fluid step in place of rotor_discs(): rows[i]
// becomes out[i] through physics/airflow/rotor_lines.hpp's rule, with
//
//   pose    the body row's pos and orient and the row's local_pos and local_orient (rotor_discs()'s, so a line is where its disc
//           was);
//   speed   the row's omega, the shaft's speed after the step; the blades' own thrust and torque come from the line, not from
//           k_T w^2 (the row's thrust_coeff and torque_coeff are the chain's disc model and are not read);
//   azimuth psi[i], the blade azimuth (physics/blade_azimuth.hpp), one per row;
//   blade   one BladeShape and one PolarTable for every rotor (a quad flies one propeller; GpuFluid's LineSetup holds one polar);
//   inert   a disabled row (enabled == 0) is a line with omega 0: its blades stand in the air, as they do on the drone.
//
// `rows` are one body's, and `body` is that body's row. Every out[i].blade and .polar point at `blade` and `polar`, which must
// outlive the lines. Refused (invalid_argument), naming "rotor i": a count other than rows.size() in `psi` or `out`; a row whose
// radius is not the blade's (1e-4 m: the model's rotor is this propeller); and whatever line_from_rotor() refuses.
// Allocation-free on success.
// ---------------------------------------------------------------------------
namespace spade::vehicles {

[[nodiscard]] Result<void> rotor_lines(std::span<const RotorRow> rows, const BodyState& body, std::span<const float> psi,
                                       glm::vec3 grid_origin_world, const physics::airflow::GridShape& grid,
                                       const physics::airflow::BladeShape& blade, const physics::airflow::PolarTable& polar,
                                       std::span<physics::airflow::LineRotor> out, float eps_cells = 2.0f);

}  // namespace spade::vehicles
