#pragma once

#include <algorithm>
#include <cstddef>
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

// The sum of cell(i, j, k, idx) over an nx x ny x nz grid (i innermost, idx
// the cell's array index), in the order above, each tile and the total from
// T(0). T is float, or a glm vector summed per component in the same order.
// Every fixed-order sum is this one (TD-9).
template <class T, class Cell>
[[nodiscard]] T fixed_tile_sum_of(uint32_t nx, uint32_t ny, uint32_t nz, Cell cell) noexcept {
    T total = T(0);
    for (uint32_t tk = 0; tk < nz; tk += kReduceTile) {
        const uint32_t k_end = std::min(tk + kReduceTile, nz);
        for (uint32_t tj = 0; tj < ny; tj += kReduceTile) {
            const uint32_t j_end = std::min(tj + kReduceTile, ny);
            for (uint32_t ti = 0; ti < nx; ti += kReduceTile) {
                const uint32_t i_end = std::min(ti + kReduceTile, nx);
                T tile = T(0);
                for (uint32_t k = tk; k < k_end; ++k) {
                    for (uint32_t j = tj; j < j_end; ++j) {
                        for (uint32_t i = ti; i < i_end; ++i) {
                            tile += cell(i, j, k, i + std::size_t{nx} * (j + std::size_t{ny} * k));
                        }
                    }
                }
                total += tile;
            }
        }
    }
    return total;
}

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
