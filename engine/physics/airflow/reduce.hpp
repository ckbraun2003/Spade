#pragma once

#include <cstdint>
#include <span>

// ---------------------------------------------------------------------------
// THE FIXED-ORDER REDUCTION (spec §4.5 as amended 2026-10-12; Core's stage 7
// design §5.5). Every reduction that feeds stepping sums the same way on both
// backends: the cells are cut into tiles of kReduceTile^3 (fewer at the far
// edges), each tile is summed in (k, j, i) order from 0, and the tile sums are
// added in (tk, tj, ti) order from 0, all in fp32, with no fp64. The order is
// fixed in elements, independent of any workgroup size, so the GPU twin gives
// the same bits for every workgroup size and differs from this CPU reference
// only by each operation's rounding, which is banded (TD-14).
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

inline constexpr uint32_t kReduceTile = 4;

// The sum of an nx x ny x nz cell array (i innermost), in the order above.
[[nodiscard]] float fixed_tile_sum(std::span<const float> x, uint32_t nx, uint32_t ny, uint32_t nz) noexcept;

// The sum of a[i] * b[i] in fixed_tile_sum()'s order, each product rounded to
// fp32 first: CG's dot products (pcg.hpp), so the GPU twin sums them alike.
[[nodiscard]] float fixed_tile_dot(std::span<const float> a, std::span<const float> b, uint32_t nx, uint32_t ny,
                                   uint32_t nz) noexcept;

// x -= the fixed-tile sum times the host's reciprocal of the cell count,
// subtracted in array order: a closed domain's pressure gauge (Task 5).
void remove_mean(std::span<float> x, uint32_t nx, uint32_t ny, uint32_t nz) noexcept;

}  // namespace spade::physics::airflow
