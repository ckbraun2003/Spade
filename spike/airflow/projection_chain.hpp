#pragma once

// ---------------------------------------------------------------------------
// projection_chain.hpp (plan B, task B2) -- the GPU projection without
// solids, for the chain runner: the multigrid's levels packed as the
// kernels read them (engine/shaders/mg_levels.slang), the words of
// airflow_params the projection reads, and project()'s order as a chain of
// dispatches. The CPU twins, and the authority for the order, are
// engine/physics/airflow/projection.cpp's project() and multigrid.cpp's
// cycle(): the divergence, n_v V(2,2) cycles from the grid's own p, then the
// gradient subtracted from every face that is not fixed.
//
// A CLOSED DOMAIN (no pressure outlet) also removes b's mean before the
// cycles and p's after each one (reduce.cpp's remove_mean()): airflow_tile_sum
// (Physics' B1) sums the field in the CPU's fixed tile order into
// airflow_scalars[kMeanSlot], and airflow_remove_mean subtracts sum *
// inv_count from every cell. The chain then needs airflow_partials, one float
// per tile (tile_sum_tiles()), and airflow_scalars bound.
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "compute/vulkan/dispatch_chain.hpp"
#include "core/error.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/projection.hpp"

namespace spade::airflow_gpu {

// airflow_params' projection words (Physics' layout, 2026-10-22).
inline constexpr uint32_t kParamsInvDx = 5;        // divergence()'s 1 / dx
inline constexpr uint32_t kParamsLevelCount = 17;
inline constexpr uint32_t kParamsFaceKinds = 18;   // 18-23, mac_grid.hpp's Side order, FaceKind's values
inline constexpr uint32_t kParamsBCoef = 24;       // rho dx^2 / h
inline constexpr uint32_t kParamsGCoef = 25;       // h / (rho dx)
inline constexpr uint32_t kParamsInvCount = 26;    // remove_mean()'s 1 / cells
inline constexpr uint32_t kParamsLevelTable = 32;  // level l at 32 + 12 l
inline constexpr uint32_t kParamsLevelStride = 12;

// The multigrid's levels as the kernels read them. Level 0 is the grid's own
// arrays (airflow_pressure, airflow_rhs, airflow_residual); levels 1..L are
// packed in airflow_multigrid, each p, b and r in turn; a weighted level's
// w[0..2] are packed in airflow_mg_weights, in level order.
struct MgLayout {
    // {nx, ny, nz, off_p, off_b, off_r, off_w0, off_w1, off_w2, weighted, 0, 0}, offsets in floats
    std::vector<std::array<uint32_t, kParamsLevelStride>> levels;
    uint64_t multigrid_floats = 0;  // airflow_multigrid's size
    std::vector<float> weights;     // airflow_mg_weights' contents
};

// Refused (capacity_exceeded): an offset past 32 bits.
[[nodiscard]] Result<MgLayout> mg_layout(const physics::airflow::Multigrid& mg);

// The words airflow_params needs to hold the projection's: through the level table.
[[nodiscard]] std::size_t projection_params_words(const MgLayout& layout) noexcept;

// Writes the projection's words into `params`, which holds
// projection_params_words(): word 5, the level count, the face kinds, the
// three coefficients and the level table. The floats are the CPU's own
// (physics::airflow::projection_floats()).
void write_projection_params(std::span<uint32_t> params, const MgLayout& layout,
                             const std::array<physics::airflow::FaceKind, 6>& kinds,
                             const physics::airflow::ProjectionFloats& floats);

// remove_mean()'s airflow_scalars slot (Physics, 2026-10-22; 1-7 are B5's).
inline constexpr uint32_t kMeanSlot = 0;

// airflow_tile_sum's tiles over the grid, reduce.hpp's kReduceTile on a side:
// airflow_partials' floats.
[[nodiscard]] uint32_t tile_sum_tiles(const physics::airflow::GridShape& shape) noexcept;

// The projection kernels' pipelines in the runner (RunnerDesc::kernels).
// tile_sum and remove_mean are dispatched for a closed domain only.
struct ProjectionPipelines {
    uint32_t divergence = 0;
    uint32_t smooth = 0;
    uint32_t residual = 0;
    uint32_t restrict_residual = 0;
    uint32_t prolong = 0;
    uint32_t gradient = 0;
    uint32_t tile_sum = 0;
    uint32_t remove_mean = 0;
};

// Each projection kernel's push: four words, the count in word 2.
struct ProjectionPush {
    uint32_t a = 0;  // level, or component
    uint32_t b = 0;  // colour
    uint32_t count = 0;
    uint32_t d = 0;
};

// One dispatch of `pipeline` over push.count invocations.
[[nodiscard]] compute::ChainCommand projection_dispatch(uint32_t pipeline, const ProjectionPush& push,
                                                        uint32_t workgroup_size);

// `sweeps` red-black sweeps on level `level` (smooth_red_black()'s order).
void append_sweeps(std::vector<compute::ChainCommand>& out, const MgLayout& layout, const ProjectionPipelines& p,
                   uint32_t level, uint32_t sweeps, uint32_t workgroup_size);

// One V(2,2) cycle from level `level` down (cycle()'s order).
void append_v_cycle(std::vector<compute::ChainCommand>& out, const MgLayout& layout, const ProjectionPipelines& p,
                    uint32_t level, uint32_t workgroup_size);

// project()'s chain without solids: the divergence, n_v V(2,2) cycles from
// airflow_pressure's p, the gradient subtracted from u, v and w; on a closed
// domain, b's mean removed before the cycles and p's after each one.
[[nodiscard]] std::vector<compute::ChainCommand> projection_chain(
    const MgLayout& layout, const physics::airflow::GridShape& shape,
    const std::array<physics::airflow::FaceKind, 6>& kinds, uint32_t n_v, const ProjectionPipelines& p,
    uint32_t workgroup_size);

}  // namespace spade::airflow_gpu
