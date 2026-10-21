#pragma once

#include <array>
#include <cstdint>
#include <span>
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
// The faces of every level make_multigrid() would build, in 64 bits, with no
// allocation (set_weights() holds a weight per face on each level).
[[nodiscard]] uint64_t multigrid_faces(uint32_t nx, uint32_t ny, uint32_t nz,
                                       std::array<bool, 3> periodic = {}) noexcept;
void set_kinds(Multigrid& mg, const std::array<FaceKind, 6>& kind) noexcept;

// The face weights of every level (the solids design, section 3; a host call:
// it allocates). Level 0 copies `fine`, the MAC grid's u, v and w apertures.
// Each coarser face is the mean of the faces of the level above on its own
// plane that it covers (2 x 2, or 2 or 1 across an axis that is not
// coarsened), summed with the second transverse axis outer. A coarse face
// covers only every second fine plane: a sheet on another plane is invisible
// to it (finding 2), which is why the solids path solves by CG (pcg.hpp).
void set_weights(Multigrid& mg, const std::array<std::span<const float>, 3>& fine);
// Every level back to plan A's stencil.
void clear_weights(Multigrid& mg) noexcept;

// coarse.b = (sum of the children's residuals) * (4 / children).
void restrict_residual(const PoissonLevel& fine, PoissonLevel& coarse) noexcept;
// fine.p += the trilinear interpolation of coarse.p.
void prolong_add(const PoissonLevel& coarse, PoissonLevel& fine) noexcept;

// One V(2,2) cycle on levels[0]: its p is the initial guess, its b is set.
void v_cycle(Multigrid& mg) noexcept;

// The coarsest level's sweeps each way in the symmetric cycle.
inline constexpr uint32_t kCoarsestSymSweeps = 16;

// coarse.b = (4 / children) * P^T fine.r, with P prolong_add()'s trilinear
// interpolation: the restriction is the interpolation's transpose (full
// weighting), so the cycle below is symmetric. Gathered per coarse cell from
// the fine cells within one of its children, in (k, j, i) order.
void restrict_full_weighting(const PoissonLevel& fine, PoissonLevel& coarse) noexcept;

// The symmetric V(2,2) cycle, CG's preconditioner (spec 1.5 as amended
// 2026-10-13): from p = 0 on every level, red then black twice, restrict by
// full weighting, recurse, prolong, black then red twice; the coarsest level
// kCoarsestSymSweeps sweeps red-first and as many black-first. levels[0].b is
// the input, levels[0].p the output, and every level's p, b and r are
// scratch. A symmetric operator (design test 10).
void precondition(Multigrid& mg) noexcept;

}  // namespace spade::physics::airflow
