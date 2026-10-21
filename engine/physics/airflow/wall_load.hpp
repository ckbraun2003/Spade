#pragma once

#include <glm/glm.hpp>

#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/regions.hpp"
#include "physics/airflow/solids.hpp"

// ---------------------------------------------------------------------------
// THE WALL LOAD (P3; the solids design, section 6). The pressure force and
// moment the air puts on the solids, for display: one-way, and in D4 they
// replace the body's drag element (spec §2), so nothing is counted twice.
//
// On a cell with an open face, the divergence theorem gives the solid surface
// inside it the area vector A_c = sum over its faces of theta_f dx^2 n_f, n_f
// out of the cell; along axis a that is dx^2 (theta_hi - theta_lo). Then
//
//   F = -sum_c p_c A_c,    M = sum_c (x_c - about) x (-p_c A_c),
//
// with p_c the cell's own pressure (Pa) and x_c its centre, each summed in
// the fixed tile order (reduce.hpp). A full fluid cell gives A_c = 0. Beside
// a sheet the closed face gives each side -/+ dx^2 e, so the two sides sum to
// the pressure jump across it times dx^2. A cell with no open face (labelled
// solid) is skipped: its pressure is never read.
//
// The load on all the solids together; per-part loads wait for a second body
// (D4). It reads the pressure as the step left it, a cut cell's own value up
// to half a cell off the wall (Test/Docs' sampling term).
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct WallLoad {
    glm::vec3 force{0.0f};   // N, on the solids, along the grid's axes
    glm::vec3 moment{0.0f};  // N m, about the point given
};

// `about` is in the grid's frame (the world shifted by the grid's minimum
// corner). No solids, or regions not labelled for this grid: zero.
[[nodiscard]] WallLoad wall_load(const MacGrid& g, const SolidField& f, const FluidRegions& r,
                                 glm::vec3 about) noexcept;

}  // namespace spade::physics::airflow
