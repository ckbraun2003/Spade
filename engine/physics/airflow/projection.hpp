#pragma once

#include <cstdint>
#include <vector>

#include "physics/airflow/multigrid.hpp"

// ---------------------------------------------------------------------------
// THE PROJECTION (spec §1.5, step 6). Solves (dx^2-scaled) sum (p_nb - p) =
// (rho dx^2 / h) div(u*) by n_v V-cycles from the grid's own p, then subtracts
// (h / (rho dx)) (p_right - p_left) from every face that is not fixed. The
// face kinds come from `bc` on every call.
//
// A CLOSED DOMAIN (no pressure outlet: every face a wall, or every axis
// periodic) fixes p only up to a constant. Then the mean of the right side is
// removed before the cycles, so the system is solvable, and the mean of p
// after each cycle, so p never drifts: each mean is the fixed-tile fp32 sum
// (reduce.hpp, spec §4.5), so it is repeatable run to run and the GPU twin
// sums in the same order.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct ProjectionScratch {
    Multigrid mg;
    std::vector<float> div;
};

// The hierarchy follows the domain's periodic axes (make_multigrid).
[[nodiscard]] ProjectionScratch make_projection_scratch(const GridShape& shape, const DomainBc& bc);

void project(MacGrid& g, const DomainBc& bc, float rho, float h, uint32_t n_v, ProjectionScratch& s) noexcept;

}  // namespace spade::physics::airflow
