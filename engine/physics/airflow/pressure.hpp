#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/worker_pool.hpp"

// ---------------------------------------------------------------------------
// THE PRESSURE EQUATION (spec §1.5, "Project"), on one grid level, scaled by
// dx^2: for each cell, the sum over its six faces of (p_nb - p) = b. A face to
// another cell (or across a periodic axis of size > 1) contributes p_nb - p; a
// pressure outlet contributes -2 p (ghost -p: p = 0 on the face); a
// fixed-velocity face or a periodic axis of size 1 contributes nothing. The
// faces are visited -x, +x, -y, +y, -z, +z, and the GPU twin keeps that order.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct PoissonLevel {
    uint32_t nx = 0;
    uint32_t ny = 0;
    uint32_t nz = 0;
    std::array<FaceKind, 6> kind{};
    std::vector<float> p;  // the unknown
    std::vector<float> b;  // the right side
    std::vector<float> r;  // the residual b - (sum - diag p), after residual()
    // Face weights (the solids design, section 3): w[a] holds the faces normal
    // to axis a, indexed as a MAC grid of this level's shape indexes u, v and
    // w. Each multiplies its neighbour term and its part of the diagonal. On a
    // periodic axis the last face plane mirrors the first. Empty: every weight
    // 1, and plan A's stencil and fast paths run, bit for bit.
    std::array<std::vector<float>, 3> w;
};

[[nodiscard]] PoissonLevel make_level(uint32_t nx, uint32_t ny, uint32_t nz, const std::array<FaceKind, 6>& kind);

// The discrete divergence of every cell, 1/s:
// ((u_{i+1} - u_i) + (v_{j+1} - v_j) + (w_{k+1} - w_k)) * (1 / dx).
void divergence(const MacGrid& g, std::span<float> out) noexcept;

// One Gauss-Seidel update of cell (i, j, k): p = (sum - b) / diag.
void smooth_one_cell(PoissonLevel& level, uint32_t i, uint32_t j, uint32_t k) noexcept;


// The cells a level needs before its smoothing and residual go to a pool's
// threads (L18 slice 2): below it a barrier costs more than the planes it
// shares out. Only speed: the bits are the serial pass's either way.
inline constexpr std::size_t kThreadedLevelCells = 8192;

// `sweeps` red-black sweeps: every cell with (i + j + k) even, then every odd
// one; black_first runs the odd colour first (the symmetric cycle's
// post-smoothing, Task 4). With a pool, and a level of kThreadedLevelCells or
// more, each colour pass is split by k plane across its threads, a barrier
// between the colours: a cell of one colour reads only the other colour (a
// wrap joins cell 0 and cell n - 1 of an even n, and an axis of one cell
// has none), so the bits are the same at any thread count.
void smooth_red_black(PoissonLevel& level, uint32_t sweeps, bool black_first = false,
                      WorkerPool* pool = nullptr) noexcept;

// out = sum - diag x, the level's operator applied to `x` (both sized like p).
void apply_operator(const PoissonLevel& level, std::span<const float> x, std::span<float> out) noexcept;

// level.r = b - (sum - diag p) for every cell; with a pool, split by k plane
// as smooth_red_black() splits its passes (each cell reads p and writes r).
void residual(PoissonLevel& level, WorkerPool* pool = nullptr) noexcept;

}  // namespace spade::physics::airflow
