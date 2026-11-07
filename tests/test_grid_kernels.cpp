// test_grid_kernels.cpp -- the CPU twins of the domain-neutral grid kernels (module API stage 7, Task B3;
// docs/design/core/plans/2026-11-06-module-api-stage7-plan.md). The device kernels' parity against these twins is
// test_gpu_grid_kernels.cpp; here the twins are pinned on their own: an integer-shift copy that moves bits and wraps
// nothing, and trilinear interpolation at points in a fixed fp32 order of evaluation.
#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "world/grid_kernels.hpp"

namespace {

using spade::world::GridDims;
using spade::world::GridFrame;
using spade::world::grid_points;
using spade::world::grid_shift;
using spade::world::lerp32;

constexpr uint32_t kFill = 0xDEADBEEFu;

[[nodiscard]] uint32_t at(const std::vector<uint32_t>& v, GridDims d, uint32_t x, uint32_t y, uint32_t z) {
    return v[(uint64_t{z} * d.ny + y) * d.nx + x];
}

// The same copy written as a scatter from the source, so it shares no index arithmetic with the twin's gather.
[[nodiscard]] std::vector<uint32_t> scattered(const std::vector<uint32_t>& src, GridDims d, std::array<int32_t, 3> s) {
    std::vector<uint32_t> out(src.size(), kFill);
    for (uint32_t z = 0; z < d.nz; ++z) {
        for (uint32_t y = 0; y < d.ny; ++y) {
            for (uint32_t x = 0; x < d.nx; ++x) {
                const int64_t tx = int64_t{x} - s[0];
                const int64_t ty = int64_t{y} - s[1];
                const int64_t tz = int64_t{z} - s[2];
                if (tx < 0 || ty < 0 || tz < 0 || tx >= d.nx || ty >= d.ny || tz >= d.nz) continue;
                out[(static_cast<uint64_t>(tz) * d.ny + static_cast<uint64_t>(ty)) * d.nx + static_cast<uint64_t>(tx)] =
                    at(src, d, x, y, z);
            }
        }
    }
    return out;
}

}  // namespace

TEST(GridShift, IsBitExactAndWrapsNothing) {
    const GridDims d{4, 3, 2};
    std::vector<uint32_t> src(d.cells());
    for (std::size_t i = 0; i < src.size(); ++i) src[i] = 0x1000u + static_cast<uint32_t>(i);
    // Bit patterns a float conversion would damage: a NaN payload, negative zero, the smallest denormal, infinity.
    src[0] = 0x7FC00001u;
    src[1] = 0x80000000u;
    src[5] = 0x00000001u;
    src[6] = 0xFF800000u;

    const std::array<std::array<int32_t, 3>, 7> shifts = {{{0, 0, 0}, {1, 0, 0}, {-1, 1, 0}, {0, 0, 1},
                                                            {2, -1, 1}, {5, 0, 0}, {-4, -3, -2}}};
    for (const auto& s : shifts) {
        std::vector<uint32_t> dst(src.size(), 0u);
        const auto ok = grid_shift(src, dst, d, s, kFill);
        ASSERT_TRUE(ok.has_value()) << ok.error().context;
        EXPECT_EQ(dst, scattered(src, d, s)) << "shift " << s[0] << "," << s[1] << "," << s[2];
    }
}

// Hand-checked, so the reference above is not the only witness.
TEST(GridShift, AShiftByOneMovesTheWindowAndFillsTheEdge) {
    const GridDims d{3, 1, 1};
    const std::vector<uint32_t> src = {10, 20, 30};
    std::vector<uint32_t> dst(3, 0u);
    ASSERT_TRUE(grid_shift(src, dst, d, {1, 0, 0}, kFill).has_value());
    EXPECT_EQ(dst, (std::vector<uint32_t>{20, 30, kFill}));  // dst(x) = src(x + 1); the cell past the end is fill
    ASSERT_TRUE(grid_shift(src, dst, d, {-1, 0, 0}, kFill).has_value());
    EXPECT_EQ(dst, (std::vector<uint32_t>{kFill, 10, 20}));
    ASSERT_TRUE(grid_shift(src, dst, d, {3, 0, 0}, kFill).has_value());
    EXPECT_EQ(dst, (std::vector<uint32_t>{kFill, kFill, kFill})) << "a shift of the whole extent leaves only fill";
}

TEST(GridShift, ASizeThatIsNotTheCellCountIsRefusedNamingBoth) {
    const GridDims d{4, 3, 2};
    std::vector<uint32_t> src(d.cells());
    std::vector<uint32_t> short_dst(d.cells() - 1);
    const auto refused = grid_shift(src, short_dst, d, {1, 0, 0}, kFill);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("24"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("23"), std::string::npos) << refused.error().context;
}

// ---------------------------------------------------------------------------

// The lerp's form is the contract: a + (b - a) * t is not a * (1 - t) + b * t in fp32. Hand-evaluated: for a = 3, b = 1,
// t = 0.3f the first is 0x4019999A and the second 0x40199999 (one ulp apart).
TEST(GridKernels, TheCpuTwinsFollowTheDeviceOrderOfEvaluation) {
    EXPECT_EQ(std::bit_cast<uint32_t>(lerp32(3.0f, 1.0f, 0.3f)), 0x4019999Au);

    // A 2x2x2 field of one component, x fastest: v(i, j, k) at index (k*2 + j)*2 + i. Three points whose fp32 results were
    // evaluated by hand in the stated order (see the header).
    const std::vector<float> field = {3.0f, 1.0f, 2.0f, 7.0f, 5.0f, 0.5f, 11.0f, 4.0f};
    const GridDims d{2, 2, 2};
    const GridFrame frame{};  // origin 0, cell 1, so g = p - 0.5
    const std::vector<float> points = {0.8f, 0.65f, 0.9f,   // interior: t = 0.3, 0.15, 0.4
                                       -5.0f, 100.0f, 0.5f,  // outside: the border cell, exactly
                                       1.0f, 1.0f, 1.0f};
    std::vector<float> out(3, -1.0f);
    const auto ok = grid_points(field, d, 1, frame, points, out);
    ASSERT_TRUE(ok.has_value()) << ok.error().context;
    EXPECT_EQ(std::bit_cast<uint32_t>(out[0]), 0x40541894u);
    EXPECT_EQ(std::bit_cast<uint32_t>(out[1]), 0x40000000u);
    EXPECT_EQ(std::bit_cast<uint32_t>(out[2]), 0x40860000u);
}

// A point at a cell centre is that cell's value, exactly (t is 0 and a + (b - a) * 0 is a).
TEST(GridPoints, ACellCentreReadsItsCellExactly) {
    const GridDims d{4, 3, 2};
    const uint32_t comps = 3;
    std::vector<float> field(d.cells() * comps);
    for (std::size_t i = 0; i < field.size(); ++i) field[i] = 0.25f * static_cast<float>(i) - 3.0f;
    const GridFrame frame{.origin = {1.0f, -2.0f, 4.0f}, .inv_cell = 2.0f};  // cell edge 0.5
    for (uint32_t k = 0; k < d.nz; ++k) {
        for (uint32_t j = 0; j < d.ny; ++j) {
            for (uint32_t i = 0; i < d.nx; ++i) {
                const std::vector<float> p = {1.0f + (static_cast<float>(i) + 0.5f) * 0.5f,
                                              -2.0f + (static_cast<float>(j) + 0.5f) * 0.5f,
                                              4.0f + (static_cast<float>(k) + 0.5f) * 0.5f};
                std::vector<float> out(comps, -1.0f);
                ASSERT_TRUE(grid_points(field, d, comps, frame, p, out).has_value());
                const std::size_t cell = (static_cast<std::size_t>(k) * d.ny + j) * d.nx + i;
                for (uint32_t c = 0; c < comps; ++c) {
                    EXPECT_EQ(std::bit_cast<uint32_t>(out[c]), std::bit_cast<uint32_t>(field[cell * comps + c]))
                        << "cell " << i << "," << j << "," << k << " component " << c;
                }
            }
        }
    }
}

// Components are independent: the second is ten times the first everywhere, and stays so after interpolation.
TEST(GridPoints, ComponentsInterpolateIndependently) {
    const GridDims d{3, 3, 3};
    std::vector<float> field(d.cells() * 2);
    for (std::size_t c = 0; c < d.cells(); ++c) {
        field[c * 2] = static_cast<float>(c % 7) + 0.5f;
        field[c * 2 + 1] = 10.0f * field[c * 2];
    }
    const std::vector<float> p = {1.1f, 0.7f, 2.3f, 0.2f, 1.9f, 1.4f};
    std::vector<float> out(4, 0.0f);
    ASSERT_TRUE(grid_points(field, d, 2, GridFrame{}, p, out).has_value());
    EXPECT_NEAR(out[1], 10.0f * out[0], 1e-4f * out[1]);
    EXPECT_NEAR(out[3], 10.0f * out[2], 1e-4f * out[3]);
    EXPECT_NE(out[0], out[2]) << "two different points read two different values";
}

TEST(GridPoints, AOneCellAxisReadsThatCellAtAnyCoordinate) {
    const GridDims d{1, 1, 1};
    const std::vector<float> field = {42.0f};
    const std::vector<float> p = {-9.0f, 0.5f, 77.0f};
    std::vector<float> out(1, 0.0f);
    ASSERT_TRUE(grid_points(field, d, 1, GridFrame{}, p, out).has_value());
    EXPECT_EQ(out[0], 42.0f);
}

// Review Focus: a point that is not a number must not read a stray cell or crash; it answers NaN, visibly.
TEST(GridPoints, ANonFinitePointAnswersNaNInEveryComponent) {
    const GridDims d{2, 2, 2};
    const std::vector<float> field(8 * 2, 1.0f);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const std::vector<float> p = {nan, 0.5f, 0.5f, 0.5f, inf, 0.5f, 0.5f, 0.5f, -inf};
    std::vector<float> out(6, 0.0f);
    ASSERT_TRUE(grid_points(field, d, 2, GridFrame{}, p, out).has_value());
    for (std::size_t i = 0; i < out.size(); ++i) EXPECT_TRUE(std::isnan(out[i])) << i;
}

TEST(GridPoints, WrongSizesAreRefusedNamingThem) {
    const GridDims d{2, 2, 2};
    const std::vector<float> field(8, 1.0f);
    const std::vector<float> p = {0.5f, 0.5f, 0.5f};
    std::vector<float> out(1, 0.0f);
    const auto short_field = grid_points(std::span<const float>(field).first(7), d, 1, GridFrame{}, p, out);
    ASSERT_FALSE(short_field.has_value());
    EXPECT_EQ(short_field.error().code, spade::Code::invalid_argument);
    EXPECT_FALSE(grid_points(field, d, 0, GridFrame{}, p, out).has_value()) << "0 components";
    EXPECT_FALSE(grid_points(field, d, 5, GridFrame{}, p, out).has_value()) << "5 components";
    const std::vector<float> ragged = {0.5f, 0.5f};
    EXPECT_FALSE(grid_points(field, d, 1, GridFrame{}, ragged, out).has_value()) << "points not a multiple of 3";
    std::vector<float> wide(2, 0.0f);
    EXPECT_FALSE(grid_points(field, d, 1, GridFrame{}, p, wide).has_value()) << "out the wrong size";
}
