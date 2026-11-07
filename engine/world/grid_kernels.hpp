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
// WHAT THE BORDER CLAMP IS NOT. A point outside the cell centres reads the border value because the index has to land
// somewhere; that is not a boundary condition. A point outside the domain's box (which should read the ambient wind) and
// a point inside a solid (which reads a blend of solid-cell values, not the flow there) are the CALLER's to detect, from
// the occupancy it already has, before it takes this kernel's value (lead and Physics, 2026-11-07).
//
// EXACTNESS AT A CENTRE. A point exactly at a cell centre has t = 0 and reads that cell's value exactly. At the last centre
// of an axis (g = n - 1: i0 = n - 2, t = 1) the lerp gives a + (b - a), which is b exactly whenever b - a is exact (b within
// a factor of two of a, by Sterbenz) and otherwise b to within an ulp of the larger operand: the form is the device's
// (three separately rounded operations), not a promise of an exact endpoint.
#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <span>
#include <string>

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
[[nodiscard]] inline Result<void> grid_shift(std::span<const uint32_t> src, std::span<uint32_t> dst, GridDims dims,
                                             std::array<int32_t, 3> shift, uint32_t fill) {
    const uint64_t cells = dims.cells();
    if (src.size() != cells || dst.size() != cells) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "grid_shift: src has " + std::to_string(src.size()) + " words and dst " +
                                         std::to_string(dst.size()) + "; a " + std::to_string(dims.nx) + "x" +
                                         std::to_string(dims.ny) + "x" + std::to_string(dims.nz) + " grid has " +
                                         std::to_string(cells)});
    }
    for (uint32_t z = 0; z < dims.nz; ++z) {
        for (uint32_t y = 0; y < dims.ny; ++y) {
            for (uint32_t x = 0; x < dims.nx; ++x) {
                const int64_t sx = int64_t{x} + shift[0];
                const int64_t sy = int64_t{y} + shift[1];
                const int64_t sz = int64_t{z} + shift[2];
                const bool inside = sx >= 0 && sy >= 0 && sz >= 0 && sx < dims.nx && sy < dims.ny && sz < dims.nz;
                const uint64_t to = (uint64_t{z} * dims.ny + y) * dims.nx + x;
                dst[to] = inside ? src[(static_cast<uint64_t>(sz) * dims.ny + static_cast<uint64_t>(sy)) * dims.nx +
                                       static_cast<uint64_t>(sx)]
                                 : fill;
            }
        }
    }
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
namespace detail {
[[nodiscard]] inline bool finite32(float v) noexcept { return (std::bit_cast<uint32_t>(v) & 0x7F800000u) != 0x7F800000u; }
}  // namespace detail

[[nodiscard]] inline Result<void> grid_points(std::span<const float> field, GridDims dims, uint32_t comps,
                                              const GridFrame& frame, std::span<const float> points,
                                              std::span<float> out) {
    const uint64_t cells = dims.cells();
    if (comps < 1 || comps > 4) {
        return std::unexpected(Error{Code::invalid_argument, "grid_points: comps is " + std::to_string(comps) + "; 1 to 4"});
    }
    if (cells == 0 || field.size() != cells * comps) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "grid_points: the field has " + std::to_string(field.size()) + " floats; a " +
                                         std::to_string(dims.nx) + "x" + std::to_string(dims.ny) + "x" +
                                         std::to_string(dims.nz) + " grid of " + std::to_string(comps) + " has " +
                                         std::to_string(cells * comps)});
    }
    if (points.size() % 3 != 0) {
        return std::unexpected(Error{Code::invalid_argument, "grid_points: " + std::to_string(points.size()) +
                                                                 " point floats; three per point"});
    }
    const uint64_t count = points.size() / 3;
    if (out.size() != count * comps) {
        return std::unexpected(Error{Code::invalid_argument, "grid_points: out has " + std::to_string(out.size()) +
                                                                 " floats; " + std::to_string(count) + " points of " +
                                                                 std::to_string(comps) + " is " +
                                                                 std::to_string(count * comps)});
    }
    const std::array<uint32_t, 3> n = {dims.nx, dims.ny, dims.nz};
    const float nan = std::bit_cast<float>(0x7FC00000u);
    for (uint64_t p = 0; p < count; ++p) {
        std::array<uint32_t, 2> idx[3];
        std::array<float, 3> t{};
        bool ok = true;
        for (int a = 0; a < 3; ++a) {
            float g = (points[p * 3 + a] - frame.origin[a]) * frame.inv_cell - 0.5f;
            if (!detail::finite32(g)) {  // tested BEFORE the clamp, which would drop a NaN
                ok = false;
                break;
            }
            const float top = static_cast<float>(n[a] - 1);
            if (g < 0.0f) g = 0.0f;
            if (g > top) g = top;
            uint32_t i0 = 0;
            if (n[a] > 1) {
                i0 = static_cast<uint32_t>(g);
                if (i0 > n[a] - 2) i0 = n[a] - 2;
            }
            idx[a] = {i0, n[a] > 1 ? i0 + 1 : 0u};
            t[a] = g - static_cast<float>(i0);
        }
        for (uint32_t c = 0; c < comps; ++c) {
            if (!ok) {
                out[p * comps + c] = nan;
                continue;
            }
            const auto at = [&](uint32_t i, uint32_t j, uint32_t k) {
                return field[((static_cast<uint64_t>(idx[2][k]) * n[1] + idx[1][j]) * n[0] + idx[0][i]) * comps + c];
            };
            const float c00 = lerp32(at(0, 0, 0), at(1, 0, 0), t[0]);
            const float c10 = lerp32(at(0, 1, 0), at(1, 1, 0), t[0]);
            const float c01 = lerp32(at(0, 0, 1), at(1, 0, 1), t[0]);
            const float c11 = lerp32(at(0, 1, 1), at(1, 1, 1), t[0]);
            const float c0 = lerp32(c00, c10, t[1]);
            const float c1 = lerp32(c01, c11, t[1]);
            out[p * comps + c] = lerp32(c0, c1, t[2]);
        }
    }
    return {};
}

}  // namespace spade::world
