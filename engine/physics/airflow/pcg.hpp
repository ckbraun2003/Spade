#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "physics/airflow/multigrid.hpp"

// ---------------------------------------------------------------------------
// THE PRESSURE SOLVE WITH SOLIDS (spec 1.5 as amended by the lead, 2026-10-13;
// the solids design, section 3): conjugate gradients on levels[0]'s weighted
// operator, preconditioned by one symmetric V(2,2) cycle (precondition()).
// McAdams, Sifakis and Teran (2010), "A parallel multigrid Poisson solver for
// fluids simulation on large grids". A fixed iteration count, nothing stopping
// on a residual. The dot products are reduce.hpp's fixed-tile sums, so the GPU
// twin sums them in the same order. The gauge removes each closed region's
// mean from r, z and x (regions.hpp); cells marked solid hold 0 in z and x.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct PcgGauge {
    void (*remove)(void* ctx, std::span<float> cells) noexcept = nullptr;  // null: no closed region
    void* ctx = nullptr;
    const uint8_t* solid = nullptr;  // per cell, nonzero: in no fluid region; null: none
};

struct PcgScratch {
    std::vector<float> x;  // the unknown: the caller's initial guess in, the solution out
    std::vector<float> b;  // the right side
    std::vector<float> r, z, d, q;
};

[[nodiscard]] PcgScratch make_pcg_scratch(std::size_t cells);

// `iterations` CG iterations on mg.levels[0]'s operator from s.x. Returns the
// final relative residual max|b - A x| / max|b| over the cells not marked
// solid (0 when b is 0 there); a max is order-free, so both backends report
// the same value. An exact zero in a step's denominator (a solved system)
// ends the iterations early, the only early exit.
float solve_pcg(Multigrid& mg, PcgScratch& s, uint32_t iterations, const PcgGauge& gauge) noexcept;

}  // namespace spade::physics::airflow
