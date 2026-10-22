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
// The bytes make_pcg_scratch() allocates: six floats per cell.
[[nodiscard]] inline uint64_t pcg_bytes(uint64_t cells) noexcept { return 6u * cells * sizeof(float); }

// A pressure solve leaving more than this relative residual is announced (the
// lead's condition, spec 1.5 as amended): ten times the warm residual the
// count above leaves (1.2e-6), held at the floor of 1e-4. The solids' CG runs
// on to it (solve_pcg_to()).
inline constexpr float kResidualAnnounce = 1.0e-4f;

// The most CG iterations a solids step takes (the lead's decision 1b,
// 2026-10-21): past its floor (kSolidsCgIterations) the solve runs on to
// kResidualAnnounce and stops here, announced. A sphere centred on a cell
// centre needs 24 at 48^3 (measured 2026-10-21); corner-centred, 8.
inline constexpr uint32_t kSolidsCgCap = 32;

struct PcgResult {
    float residual = 0.0f;    // max|b - A x| / max|b| over the cells not marked solid
    uint32_t iterations = 0;  // the iterations taken
};

// At least `floor` CG iterations on mg.levels[0]'s operator from s.x, then on
// until the relative residual max|b - A x| / max|b| over the cells not marked
// solid (0 when b is 0 there) is at most `mark`, or `cap` iterations (at
// least `floor`) are done. The residual is measured after the floor's
// iterations and after each one past it, so a solve that meets the mark at
// its floor takes exactly the floor's path, bit for bit. A max is order-free
// and the dots take the fixed tile order, so the count is the same on every
// build. An exact zero in a step's denominator (a solved system) ends the
// iterations early.
[[nodiscard]] PcgResult solve_pcg_to(Multigrid& mg, PcgScratch& s, uint32_t floor, uint32_t cap, float mark,
                                     const PcgGauge& gauge) noexcept;

// Exactly `iterations` CG iterations (solve_pcg_to() with its cap at its
// floor); returns the final relative residual.
float solve_pcg(Multigrid& mg, PcgScratch& s, uint32_t iterations, const PcgGauge& gauge) noexcept;

}  // namespace spade::physics::airflow
