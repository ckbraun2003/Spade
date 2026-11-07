// grid_kernels.hpp -- the CPU twins of the domain-neutral grid kernels Core owns (module API stage 7, Task B3;
// docs/design/core/plans/2026-11-06-module-api-stage7-plan.md; design 7.5 and the answers doc, item 4).
//
// Two kernels, neither knowing what the grid holds: an integer-shift copy (re-centring a moving window) and trilinear
// interpolation at points (the point provider's read). The device kernels are engine/shaders/kernels/grid_shift.slang and
// grid_points.slang; each twin here evaluates in the device's order, so the parity bands are the fp32 expression order and
// nothing wider.
//
// LAYOUT. A grid of nx*ny*nz cells, x fastest: cell (x, y, z) is word (z*ny + y)*nx + x; a field of `comps` floats per
// cell is comps consecutive floats. A cell-centred field: cell (i, j, k) holds the value at origin + (i + 0.5, j + 0.5,
// k + 0.5) * cell.
//
// RED STUBS (stage 7 Task B3): grid_shift() writes nothing, grid_points() writes zeros; the lerp is the device's form.
#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "core/error.hpp"

namespace spade::world {

struct GridDims {
    uint32_t nx = 0;
    uint32_t ny = 0;
    uint32_t nz = 0;
    [[nodiscard]] constexpr uint64_t cells() const noexcept { return uint64_t{nx} * ny * nz; }
};

// dst(x, y, z) = src(x + shift[0], y + shift[1], z + shift[2]) where that cell is inside the grid, `fill` where it is not.
// Words move as words: no float conversion, so every bit pattern (NaN payloads, -0, denormals) arrives as it left. Nothing
// wraps. src and dst are each dims.cells() words (invalid_argument, naming both sizes, otherwise) and must not overlap.
[[nodiscard]] inline Result<void> grid_shift(std::span<const uint32_t> /*src*/, std::span<uint32_t> /*dst*/,
                                             GridDims /*dims*/, std::array<int32_t, 3> /*shift*/,
                                             uint32_t /*fill*/) {
    return {};
}

// The device's lerp: a + (b - a) * t, three separately rounded fp32 operations (no fused multiply-add).
[[nodiscard]] constexpr float lerp32(float a, float b, float t) noexcept { return a + (b - a) * t; }

// Where a cell-centred field sits in space: its origin corner and the reciprocal of the cell edge (the reciprocal is
// formed on the host and multiplied, on both backends).
struct GridFrame {
    std::array<float, 3> origin{0.0f, 0.0f, 0.0f};
    float inv_cell = 1.0f;
};

// Trilinear interpolation of a cell-centred field at `points` (three floats each), `comps` floats per point written to
// `out`. A point outside the cell centres' span reads the border (the index clamps to [0, n-1]); a non-finite coordinate
// gives quiet NaN in every component. The order, per point: g = (p - origin) * inv_cell - 0.5 per axis, clamp, i0 =
// min(uint(g), n - 2) (0 when n = 1), t = g - i0, then lerp32 along x for the four (y, z) corners, along y for two, along
// z for one. Sizes are checked (invalid_argument, naming them): field = cells*comps floats, points a multiple of 3,
// out = points/3*comps, comps 1..4.
[[nodiscard]] inline Result<void> grid_points(std::span<const float> /*field*/, GridDims /*dims*/, uint32_t /*comps*/,
                                              const GridFrame& /*frame*/, std::span<const float> /*points*/,
                                              std::span<float> out) {
    for (float& v : out) v = 0.0f;
    return {};
}

}  // namespace spade::world
