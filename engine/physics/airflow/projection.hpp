#pragma once

#include <array>
#include <cstdint>
#include <span>
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

// The projection's host floats, each the CPU's own expression, computed in
// one place (TD-9): project() and project_solids() read b_coef and g_coef
// here, and the GPU projection's upload (plan B, B2:
// spike/airflow/projection_chain.hpp) writes all four into airflow_params, so
// a change here reaches both. inv_dx is divergence()'s 1 / dx and inv_count
// remove_mean()'s host reciprocal, the same expressions as theirs.
struct ProjectionFloats {
    float b_coef = 0.0f;     // rho dx^2 / h: the right side's scale (airflow_params word 24)
    float g_coef = 0.0f;     // h / (rho dx): the gradient's scale (word 25)
    float inv_dx = 0.0f;     // 1 / dx (word 5)
    float inv_count = 0.0f;  // 1 / cells, rounded once from double (word 26)
};

[[nodiscard]] ProjectionFloats projection_floats(const GridShape& shape, float rho, float h) noexcept;

struct ProjectionReport {
    float residual = 0.0f;    // max|r| / max|b| after the solve (pcg.hpp with solids; plan A's after its cycles)
    uint32_t iterations = 0;  // with solids: the CG iterations taken (solve_pcg_to()); plan A: its n_v cycles
    bool stalled = false;     // with solids: the CG stopped above the mark by its stall rule (pcg.hpp); plan A: false
};

// WITH SOLIDS (solids->any, the solids design, section 3): the right side is
// (rho dx^2 / h) times the aperture divergence of theta u* + (1 - theta)
// u_wall, 0 in solid cells; each closed region's mean is removed; n_v
// iterations of solve_pcg() from the grid's p, the regions' gauge and solid
// mask with it; then every face with theta > 0 subtracts the pressure
// gradient as plan A does, and every face with theta = 0 takes the wall's
// velocity. The multigrid's weights are set by set_solids() (fluid_step.hpp),
// the regions' status by check_regions(). Without solids: plan A, bit for bit.
// Solids without their regions: the grid untouched and the residual infinite.
// With solids, n_v is the CG's floor and cg_cap its cap (solve_pcg_to(), to
// kResidualAnnounce).
ProjectionReport project(MacGrid& g, const DomainBc& bc, float rho, float h, uint32_t n_v, ProjectionScratch& s,
                         const SolidField* solids = nullptr, FluidRegions* regions = nullptr,
                         uint32_t cg_cap = kSolidsCgCap) noexcept;

// The solids' projection's first and last stages, as project() runs them
// with solids, so the GPU twins (plan B's B5: airflow_solid_divergence and
// airflow_solid_gradient) are held to the CPU's own code.
//
// The right side before the region means come off: per cell,
// b_coef * (((du + dv) + dw) * inv_dx) of the aperture flux
// theta u + (1 - theta) u_wall, and 0 in a cell of no region. `b` holds the
// grid's cells.
void solid_divergence(const MacGrid& g, const SolidField& f, const FluidRegions& regions,
                      const ProjectionFloats& floats, std::span<float> b) noexcept;

// From the grid's p: every face with theta > 0 that is not fixed subtracts
// g_coef (p_hi - p_lo) as plan A does; every face with theta = 0 takes the
// wall's velocity; a fixed-velocity domain face is left as it is.
void subtract_gradient_with_apertures(MacGrid& g, const std::array<FaceKind, 6>& kinds, float g_coef,
                                      const SolidField& f) noexcept;

}  // namespace spade::physics::airflow
