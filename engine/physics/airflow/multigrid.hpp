#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "physics/airflow/pressure.hpp"

// ---------------------------------------------------------------------------
// GEOMETRIC MULTIGRID for the pressure equation (spec §1.5): V(2,2) cycles of
// red-black Gauss-Seidel, averaging restriction, cell-centred trilinear
// prolongation, and a coarsest level solved by 32 sweeps. A fixed number of
// cycles per projection; nothing stops on a residual.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

inline constexpr uint32_t kPreSweeps = 2;
inline constexpr uint32_t kPostSweeps = 2;
inline constexpr uint32_t kCoarsestSweeps = 32;

struct Multigrid {
    std::vector<PoissonLevel> levels;  // [0] the finest
};

// The hierarchy for an nx x ny x nz grid. Face kinds are set by set_kinds().
// Coarsening halves every axis of more than one cell while each is even and at
// least 4; a periodic axis must also halve to an even size (n a multiple of
// 4), so no level holds an odd periodic ring, whose red-black colouring would
// make a sweep depend on its order (the final review's I2, 2026-10-12).
// KNOWN LIMIT (the final review's M4): an axis of one cell takes no part, and
// its outlet faces' Dirichlet ghost term keeps the finest level's weight on
// the coarser levels, so a one-cell slab open on that axis gets a weaker
// coarse correction (slower convergence, not a wrong answer).
[[nodiscard]] Multigrid make_multigrid(uint32_t nx, uint32_t ny, uint32_t nz, std::array<bool, 3> periodic = {});
// The cells of every level make_multigrid() would build, in 64 bits, with no
// allocation (each level holds p, b and r: 3 floats per cell).
[[nodiscard]] uint64_t multigrid_cells(uint32_t nx, uint32_t ny, uint32_t nz,
                                       std::array<bool, 3> periodic = {}) noexcept;
void set_kinds(Multigrid& mg, const std::array<FaceKind, 6>& kind) noexcept;

// coarse.b = (sum of the children's residuals) * (4 / children).
void restrict_residual(const PoissonLevel& fine, PoissonLevel& coarse) noexcept;
// fine.p += the trilinear interpolation of coarse.p.
void prolong_add(const PoissonLevel& coarse, PoissonLevel& fine) noexcept;

// One V(2,2) cycle on levels[0]: its p is the initial guess, its b is set.
void v_cycle(Multigrid& mg) noexcept;

}  // namespace spade::physics::airflow
