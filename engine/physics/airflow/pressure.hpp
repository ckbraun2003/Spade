#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/mac_grid.hpp"

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
};

[[nodiscard]] PoissonLevel make_level(uint32_t nx, uint32_t ny, uint32_t nz, const std::array<FaceKind, 6>& kind);

// The discrete divergence of every cell, 1/s:
// ((u_{i+1} - u_i) + (v_{j+1} - v_j) + (w_{k+1} - w_k)) * (1 / dx).
void divergence(const MacGrid& g, std::span<float> out) noexcept;

// One Gauss-Seidel update of cell (i, j, k): p = (sum - b) / diag.
void smooth_one_cell(PoissonLevel& level, uint32_t i, uint32_t j, uint32_t k) noexcept;

// `sweeps` red-black sweeps: every cell with (i + j + k) even, then every odd one.
void smooth_red_black(PoissonLevel& level, uint32_t sweeps) noexcept;

// level.r = b - (sum - diag p) for every cell.
void residual(PoissonLevel& level) noexcept;

}  // namespace spade::physics::airflow
