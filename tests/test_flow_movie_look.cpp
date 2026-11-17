// flow_movie's look options for the web masters (tools/flow_movie/look.hpp; the lead's L86 approval, Cameron's ask 1).
//
// The draw adds structure the flow does not carry: a per-frame relative lambda2 level that drifts as the disc rim's
// strongest cell grows, and one-cell specks that read as "many small vortices". Each option below is a declared choice
// on the published volume, never a hidden one:
//
//   FlowMovieLook.Smooth*   --l2-smooth: lambda2-hat smoothed over one cell (a Gaussian of sigma 1 cell, 7 taps a side,
//                           separable). A no-data cell (a solid) stays no-data and is left out of its neighbours' mean, so
//                           a constant field stays constant beside a part.
//   FlowMovieLook.Cull*     --min-cells N: a connected piece (6-connected) of lambda2-hat below -L smaller than N cells is
//                           set to 0; L is --l2-level. Pieces of N cells or more, and every cell above -L, are untouched.
//   FlowMovieLook.Mean*     --mean-frames M: the velocity (u, v, w) averaged over the M frames ending at the drawn one; a
//                           cell that is no-data in any of them is no-data in the mean.
//   FlowMovieLook.Window*   which cached frames the mean reads.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include "../tools/flow_movie/look.hpp"
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"

namespace {

using spade::FlowVolume;
namespace fp16 = spade::fp16;

[[nodiscard]] FlowVolume blank(uint32_t nx, uint32_t ny, uint32_t nz) {
    FlowVolume v;
    v.header.nx = nx;
    v.header.ny = ny;
    v.header.nz = nz;
    v.header.cell = 0.004f;
    const std::size_t halves = std::size_t{nx} * ny * nz * spade::kFlowHalvesPerCell;
    v.v.assign(halves, 0u);  // +0.0 in binary16
    v.s.assign(halves, 0u);
    return v;
}

[[nodiscard]] std::size_t at(const FlowVolume& v, uint32_t i, uint32_t j, uint32_t k) {
    return spade::flow_cell_index(v.header, i, j, k);
}

void set_l2(FlowVolume& v, uint32_t i, uint32_t j, uint32_t k, float value) {
    v.s[at(v, i, j, k) * spade::kFlowHalvesPerCell + (spade::kFlowLambda2Hat % spade::kFlowHalvesPerCell)] =
        fp16::from_f32(value);
}

[[nodiscard]] uint16_t l2_bits(const FlowVolume& v, uint32_t i, uint32_t j, uint32_t k) {
    return v.s[at(v, i, j, k) * spade::kFlowHalvesPerCell + (spade::kFlowLambda2Hat % spade::kFlowHalvesPerCell)];
}

[[nodiscard]] float l2(const FlowVolume& v, uint32_t i, uint32_t j, uint32_t k) { return fp16::to_f32(l2_bits(v, i, j, k)); }

void set_u(FlowVolume& v, uint32_t i, uint32_t j, uint32_t k, float u) {
    v.v[at(v, i, j, k) * spade::kFlowHalvesPerCell + spade::kFlowU] = fp16::from_f32(u);
}

[[nodiscard]] float u_of(const FlowVolume& v, uint32_t i, uint32_t j, uint32_t k) {
    return fp16::to_f32(v.v[at(v, i, j, k) * spade::kFlowHalvesPerCell + spade::kFlowU]);
}

// The kernel the smoothing must use: exp(-x^2 / 2) at x = -3 .. 3, normalised.
[[nodiscard]] std::vector<double> kernel() {
    std::vector<double> w;
    double sum = 0.0;
    for (int x = -3; x <= 3; ++x) {
        w.push_back(std::exp(-0.5 * x * x));
        sum += w.back();
    }
    for (double& x : w) x /= sum;
    return w;
}

}  // namespace

TEST(FlowMovieLook, SmoothSpreadsAOneCellSpikeByTheGaussianAndKeepsItsTotal) {
    FlowVolume v = blank(11, 11, 11);
    set_l2(v, 5, 5, 5, -1000.0f);
    flow_movie::smooth_lambda2(v);
    const std::vector<double> w = kernel();
    EXPECT_NEAR(l2(v, 5, 5, 5), -1000.0 * w[3] * w[3] * w[3], 1.5);  // the centre
    EXPECT_NEAR(l2(v, 6, 5, 5), -1000.0 * w[4] * w[3] * w[3], 1.0);  // a face neighbour
    EXPECT_NEAR(l2(v, 6, 6, 5), -1000.0 * w[4] * w[4] * w[3], 1.0);  // an edge neighbour
    double total = 0.0;
    for (uint32_t k = 0; k < 11; ++k) {
        for (uint32_t j = 0; j < 11; ++j) {
            for (uint32_t i = 0; i < 11; ++i) total += l2(v, i, j, k);
        }
    }
    EXPECT_NEAR(total, -1000.0, 15.0);  // normalised: what the spike held is spread, not lost (binary16 rounding aside)
    EXPECT_EQ(l2(v, 0, 0, 0), 0.0f);    // beyond three cells nothing arrives
}

TEST(FlowMovieLook, SmoothKeepsAConstantAndLeavesANoDataCellAlone) {
    FlowVolume v = blank(9, 9, 9);
    for (uint32_t k = 0; k < 9; ++k) {
        for (uint32_t j = 0; j < 9; ++j) {
            for (uint32_t i = 0; i < 9; ++i) set_l2(v, i, j, k, -5.0f);
        }
    }
    v.s[at(v, 4, 4, 4) * spade::kFlowHalvesPerCell + (spade::kFlowLambda2Hat % spade::kFlowHalvesPerCell)] =
        fp16::kNoDataBits;
    flow_movie::smooth_lambda2(v);
    EXPECT_EQ(l2_bits(v, 4, 4, 4), fp16::kNoDataBits) << "a solid stays no-data";
    for (uint32_t k = 0; k < 9; ++k) {
        for (uint32_t j = 0; j < 9; ++j) {
            for (uint32_t i = 0; i < 9; ++i) {
                if (i == 4 && j == 4 && k == 4) continue;
                EXPECT_NEAR(l2(v, i, j, k), -5.0f, 0.02f) << i << "," << j << "," << k;  // beside the solid too
            }
        }
    }
}

TEST(FlowMovieLook, CullRemovesPiecesUnderMinCellsAndOnlyThose) {
    FlowVolume v = blank(20, 10, 10);
    // A piece of 10 cells (a line), one of 40 (4 x 5 x 2), two cells that touch only at a corner, a shallow cell.
    for (uint32_t i = 0; i < 10; ++i) set_l2(v, i, 1, 1, -500.0f);
    for (uint32_t k = 0; k < 2; ++k) {
        for (uint32_t j = 0; j < 5; ++j) {
            for (uint32_t i = 0; i < 4; ++i) set_l2(v, 12 + i, 3 + j, 5 + k, -500.0f);
        }
    }
    set_l2(v, 0, 8, 8, -500.0f);
    set_l2(v, 1, 9, 9, -500.0f);  // diagonal to the one above: 6-connectivity keeps them apart
    set_l2(v, 8, 8, 8, -100.0f);  // above -L: not part of any piece
    const uint32_t removed = flow_movie::cull_small_vortices(v, 250.0f, 27u);
    EXPECT_EQ(removed, 3u);  // the line and the two corner cells
    for (uint32_t i = 0; i < 10; ++i) EXPECT_EQ(l2(v, i, 1, 1), 0.0f);
    EXPECT_EQ(l2(v, 0, 8, 8), 0.0f);
    EXPECT_EQ(l2(v, 1, 9, 9), 0.0f);
    EXPECT_EQ(l2(v, 13, 5, 5), -500.0f);
    EXPECT_EQ(l2(v, 8, 8, 8), -100.0f);
    EXPECT_EQ(flow_movie::cull_small_vortices(v, 250.0f, 0u), 0u) << "0 turns the cull off";
    EXPECT_EQ(l2(v, 13, 5, 5), -500.0f);
}

TEST(FlowMovieLook, TheLambda2RangeFollowsTheWorkSoAViewReadingItIsNotStale) {
    FlowVolume v = blank(6, 6, 6);
    set_l2(v, 2, 2, 2, -300.0f);
    set_l2(v, 4, 4, 4, 40.0f);
    flow_movie::refresh_lambda2_range(v);
    EXPECT_EQ(v.header.channel_min[spade::kFlowLambda2Hat], -300.0f);
    EXPECT_EQ(v.header.channel_max[spade::kFlowLambda2Hat], 40.0f);
    flow_movie::cull_small_vortices(v, 100.0f, 5u);  // the one cell below -100 is under 5 cells
    EXPECT_EQ(v.header.channel_min[spade::kFlowLambda2Hat], 0.0f) << "cull refreshes the range";
}

TEST(FlowMovieLook, MeanAveragesTheVelocityAndRefusesWhatItCannotAverage) {
    FlowVolume a = blank(4, 4, 4), b = blank(4, 4, 4), c = blank(4, 4, 4);
    for (uint32_t i = 0; i < 4; ++i) {
        set_u(a, i, 1, 1, 1.0f);
        set_u(b, i, 1, 1, 2.0f);
        set_u(c, i, 1, 1, 6.0f);
    }
    b.v[at(b, 3, 1, 1) * spade::kFlowHalvesPerCell + spade::kFlowU] = fp16::kNoDataBits;  // no-data in one frame
    FlowVolume out = blank(4, 4, 4);
    const FlowVolume* frames[] = {&a, &b, &c};
    ASSERT_TRUE(flow_movie::mean_velocity(out, std::span<const FlowVolume* const>(frames)).has_value());
    EXPECT_EQ(u_of(out, 0, 1, 1), 3.0f);  // (1 + 2 + 6) / 3
    EXPECT_EQ(out.v[at(out, 3, 1, 1) * spade::kFlowHalvesPerCell + spade::kFlowU], fp16::kNoDataBits)
        << "no-data in any frame is no-data in the mean";
    EXPECT_EQ(u_of(out, 0, 0, 0), 0.0f);
    // The one frame's mean is that frame; the other channels follow the LAST frame (the drawn one).
    set_l2(c, 1, 1, 1, -77.0f);
    const FlowVolume* last[] = {&c};
    ASSERT_TRUE(flow_movie::mean_velocity(out, std::span<const FlowVolume* const>(last)).has_value());
    EXPECT_EQ(u_of(out, 0, 1, 1), 6.0f);
    EXPECT_EQ(l2(out, 1, 1, 1), -77.0f);
    // Refused by name: nothing to average, and volumes of different shapes.
    EXPECT_FALSE(flow_movie::mean_velocity(out, std::span<const FlowVolume* const>()).has_value());
    FlowVolume other = blank(5, 4, 4);
    const FlowVolume* odd[] = {&a, &other};
    EXPECT_FALSE(flow_movie::mean_velocity(out, std::span<const FlowVolume* const>(odd)).has_value());
}

TEST(FlowMovieLook, TheMeanWindowIsTheFramesEndingAtTheDrawnOneAndStopsAtTheFirst) {
    EXPECT_EQ(flow_movie::mean_window(100u, 10u).first, 91u);
    EXPECT_EQ(flow_movie::mean_window(100u, 10u).count, 10u);
    EXPECT_EQ(flow_movie::mean_window(3u, 10u).first, 0u);
    EXPECT_EQ(flow_movie::mean_window(3u, 10u).count, 4u) << "the first frames have fewer to average";
    EXPECT_EQ(flow_movie::mean_window(7u, 1u).first, 7u);
    EXPECT_EQ(flow_movie::mean_window(7u, 1u).count, 1u);
    EXPECT_EQ(flow_movie::mean_window(7u, 0u).count, 1u) << "0 is the frame alone";
}
