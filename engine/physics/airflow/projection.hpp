#pragma once

#include <cstdint>
#include <vector>

#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pcg.hpp"
#include "physics/airflow/regions.hpp"
#include "physics/airflow/solids.hpp"

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
    PcgScratch pcg;  // sized by set_solids(); empty without solids
};

// The hierarchy follows the domain's periodic axes (make_multigrid).
[[nodiscard]] ProjectionScratch make_projection_scratch(const GridShape& shape, const DomainBc& bc);

struct ProjectionReport {
    float residual = 0.0f;  // with solids: max|r| / max|b| after the solve (pcg.hpp); 0 on plan A's path
};

// WITH SOLIDS (solids->any, the solids design, section 3): the right side is
// (rho dx^2 / h) times the aperture divergence of theta u* + (1 - theta)
// u_wall, 0 in solid cells; each closed region's mean is removed; n_v
// iterations of solve_pcg() from the grid's p, the regions' gauge and solid
// mask with it; then every face with theta > 0 subtracts the pressure
// gradient as plan A does, and every face with theta = 0 takes the wall's
// velocity. The multigrid's weights are set by set_solids() (fluid_step.hpp),
// the regions' status by check_regions(). Without solids: plan A, bit for bit.
ProjectionReport project(MacGrid& g, const DomainBc& bc, float rho, float h, uint32_t n_v, ProjectionScratch& s,
                         const SolidField* solids = nullptr, FluidRegions* regions = nullptr) noexcept;

}  // namespace spade::physics::airflow
