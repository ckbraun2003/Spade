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
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "../tools/flow_movie/look.hpp"
#include "../tools/flow_movie/particles.hpp"
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"
#include "render/scene.hpp"
#include "../tools/flow_movie/q_look.hpp"

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
        fp16::from_f32(value).bits;
}

[[nodiscard]] uint16_t l2_bits(const FlowVolume& v, uint32_t i, uint32_t j, uint32_t k) {
    return v.s[at(v, i, j, k) * spade::kFlowHalvesPerCell + (spade::kFlowLambda2Hat % spade::kFlowHalvesPerCell)];
}

[[nodiscard]] float l2(const FlowVolume& v, uint32_t i, uint32_t j, uint32_t k) { return fp16::to_f32(l2_bits(v, i, j, k)); }

void set_u(FlowVolume& v, uint32_t i, uint32_t j, uint32_t k, float u) {
    v.v[at(v, i, j, k) * spade::kFlowHalvesPerCell + spade::kFlowU] = fp16::from_f32(u).bits;
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

// --l2-level: ONE absolute level for the clip. A frame draws a surface at -L when its deepest core is below -L, and none
// otherwise; with no level the case's own rule stands (the deepest core times the fraction, only past the floor).
TEST(FlowMovieLook, TheVortexLevelIsOneAbsoluteValueWhenAskedAndTheFramesOwnOtherwise) {
    FlowVolume v = blank(4, 4, 4);
    v.header.channel_min[spade::kFlowLambda2Hat] = -600.0f;
    flow_movie::Look fixed_level;
    fixed_level.l2_level = 120.0f;
    ASSERT_TRUE(flow_movie::vortex_iso(v, fixed_level, 1.0f / 6.0f, -5.0f).has_value());
    EXPECT_EQ(*flow_movie::vortex_iso(v, fixed_level, 1.0f / 6.0f, -5.0f), -120.0f);
    v.header.channel_min[spade::kFlowLambda2Hat] = -900.0f;  // the rim's core deepens: the level does not move
    EXPECT_EQ(*flow_movie::vortex_iso(v, fixed_level, 1.0f / 6.0f, -5.0f), -120.0f);
    v.header.channel_min[spade::kFlowLambda2Hat] = -100.0f;   // nothing below -120: no surface this frame
    EXPECT_FALSE(flow_movie::vortex_iso(v, fixed_level, 1.0f / 6.0f, -5.0f).has_value());
    const flow_movie::Look none;
    v.header.channel_min[spade::kFlowLambda2Hat] = -600.0f;
    EXPECT_FLOAT_EQ(*flow_movie::vortex_iso(v, none, 1.0f / 6.0f, -5.0f), -100.0f);  // the case's own: deepest / 6
    v.header.channel_min[spade::kFlowLambda2Hat] = -3.0f;                             // under the floor: noise
    EXPECT_FALSE(flow_movie::vortex_iso(v, none, 1.0f / 6.0f, -5.0f).has_value());
}

// The caption says what the look did, and nothing when it did nothing.
TEST(FlowMovieLook, TheCaptionNamesEachChoiceAndSaysNothingWhenThereIsNone) {
    EXPECT_EQ(flow_movie::look_note(flow_movie::Look{}, 1.0), "");
    EXPECT_FALSE(flow_movie::Look{}.any());
    flow_movie::Look look;
    look.l2_level = 120.0f;
    look.l2_smooth = true;
    look.min_cells = 27u;
    look.mean_frames = 10u;
    EXPECT_TRUE(look.any());
    const std::string note = flow_movie::look_note(look, 1.0);
    EXPECT_NE(note.find("lambda2-hat = -120 1/s"), std::string::npos) << note;
    EXPECT_NE(note.find("smoothed over one cell"), std::string::npos) << note;
    EXPECT_NE(note.find("under 27 cells"), std::string::npos) << note;
    EXPECT_NE(note.find("10 frames"), std::string::npos) << note;
    EXPECT_NE(note.find("10 ms"), std::string::npos) << note;  // 10 frames of 1 ms
}

// The options' text: whole numbers and positive values only, refused by name otherwise.
TEST(FlowMovieLook, TheOptionsAreParsedStrictly) {
    EXPECT_FLOAT_EQ(*flow_movie::parse_l2_level("120"), 120.0f);
    EXPECT_FLOAT_EQ(*flow_movie::parse_l2_level("12.5"), 12.5f);
    for (const char* bad : {"", "0", "-5", "+5", "nan", "inf", "1e2", "12x", " 5", "5 ", "abc", "100000"}) {
        EXPECT_FALSE(flow_movie::parse_l2_level(bad).has_value()) << "'" << bad << "'";
    }
    EXPECT_EQ(*flow_movie::parse_count("27"), 27u);
    EXPECT_EQ(*flow_movie::parse_count("0"), 0u);
    for (const char* bad : {"", "-1", "+1", "2.5", "1e1", "27x", " 3", "99999999999"}) {
        EXPECT_FALSE(flow_movie::parse_count(bad).has_value()) << "'" << bad << "'";
    }
}

// Which clips a look option may touch: refused by name, never silently ignored. The level, smoothing and cull belong to
// the clip that draws vortex surfaces (S3, in the side wind); the mean flow to the D1 clips that read the velocity
// (not S4's pressure coefficient); a cull needs the level that says what a piece is. S5 and S6 run the quad's CPU film.
TEST(FlowMovieLook, AnOptionThatCannotApplyToAClipIsRefusedByNameAndAnyOtherIsNot) {
    flow_movie::Look level_only;
    level_only.l2_level = 120.0f;
    flow_movie::Look everything = level_only;
    everything.l2_smooth = true;
    everything.min_cells = 27u;
    everything.mean_frames = 10u;
    flow_movie::Look mean_only;
    mean_only.mean_frames = 10u;
    flow_movie::Look cull_no_level;
    cull_no_level.min_cells = 27u;

    EXPECT_EQ(flow_movie::look_refusal("S3", true, everything), "");
    EXPECT_EQ(flow_movie::look_refusal("S3", true, level_only), "");
    // S2 is a soft speed volume since Cameron's pick (2026-11-17): the vortex-surface options are not its own, the mean flow is.
    EXPECT_EQ(flow_movie::look_refusal("S2", true, mean_only), "");
    EXPECT_NE(flow_movie::look_refusal("S2", true, level_only).find("--l2-level"), std::string::npos);
    EXPECT_EQ(flow_movie::look_refusal("S1", true, mean_only), "");
    EXPECT_EQ(flow_movie::look_refusal("S8", true, mean_only), "");
    EXPECT_EQ(flow_movie::look_refusal("S1", true, flow_movie::Look{}), "") << "no option, nothing to refuse";
    EXPECT_EQ(flow_movie::look_refusal("S5", false, flow_movie::Look{}), "");
    EXPECT_NE(flow_movie::look_refusal("S1", true, level_only).find("--l2-level"), std::string::npos);  // S1 draws lines
    EXPECT_NE(flow_movie::look_refusal("S8", true, everything), "");
    EXPECT_NE(flow_movie::look_refusal("S4", true, mean_only), "") << "S4 draws C_p, which the mean flow does not touch";
    EXPECT_NE(flow_movie::look_refusal("S5", false, mean_only), "") << "the quad's CPU film is not the D1 flow";
    EXPECT_NE(flow_movie::look_refusal("S2", true, cull_no_level).find("--min-cells"), std::string::npos);
}

// S1's soft streamlines (Cameron, 2026-11-17: "playdoh"; the streamlines should read as light, not tubes): thin lines with a
// glow, each fading out toward both of its ends along its length, so it dissolves instead of stopping. The fade is in the line's
// ALPHAS (the lead, 2026-11-17: "by alpha, additive, not toward black"): a line dimmed to black darkens whatever is behind it,
// the drone's body among it; a line of lower alpha lets that show. A smoothstep of the arc length to the nearer end, over fade_m;
// a line shorter than twice fade_m never reaches full. The colours are not touched.
namespace {
spade::render::FlowPolyline straight_line(std::size_t points, float spacing) {
    spade::render::FlowPolyline line;
    for (std::size_t i = 0; i < points; ++i) {
        line.points.emplace_back(spacing * static_cast<float>(i), 0.0f, 0.0f);
        line.colours.emplace_back(1.0f, 0.5f, 0.25f);
    }
    return line;
}
}  // namespace

TEST(FlowMovieLook, ASoftLineFadesOutByAlphaAtBothEndsKeepsItsColoursAndIsFullInTheMiddle) {
    auto line = straight_line(11, 0.05f);  // 0.5 m
    flow_movie::fade_polyline_ends(line, 0.15f);
    ASSERT_EQ(line.alphas.size(), line.points.size()) << "one alpha a point";
    EXPECT_EQ(line.alphas.front(), 0.0f);
    EXPECT_EQ(line.alphas.back(), 0.0f);
    for (const std::size_t i : {3u, 4u, 5u, 6u, 7u}) EXPECT_EQ(line.alphas[i], 1.0f) << i;
    const float third = 3.0f / 9.0f - 2.0f / 27.0f;  // smoothstep(1/3) = 3t^2 - 2t^3
    EXPECT_NEAR(line.alphas[1], third, 1e-6f);
    EXPECT_NEAR(line.alphas[1], line.alphas[9], 1e-5f) << "the same fade at both ends, to the float's sum";
    const float two_thirds = 2.0f / 3.0f;  // 0.10 m from the end: smoothstep(2/3)
    EXPECT_NEAR(line.alphas[2], two_thirds * two_thirds * (3.0f - 2.0f * two_thirds), 1e-5f);
    EXPECT_EQ(line.colours, straight_line(11, 0.05f).colours) << "the colours are not dimmed: the line fades, it does not darken";
}

TEST(FlowMovieLook, AShortSoftLineNeverReachesFullAndAZeroFadeChangesNothing) {
    auto line = straight_line(5, 0.05f);  // 0.2 m, shorter than twice 0.15
    flow_movie::fade_polyline_ends(line, 0.15f);
    const float t = 0.1f / 0.15f;
    EXPECT_NEAR(line.alphas[2], t * t * (3.0f - 2.0f * t), 1e-5f);
    EXPECT_LT(line.alphas[2], 1.0f);
    auto unchanged = straight_line(5, 0.05f);
    flow_movie::fade_polyline_ends(unchanged, 0.0f);
    EXPECT_TRUE(unchanged.alphas.empty()) << "no fade: no alphas, so the line draws as it always did";
    EXPECT_EQ(unchanged.colours, straight_line(5, 0.05f).colours);
    spade::render::FlowPolyline one;
    one.points.emplace_back(0.0f, 0.0f, 0.0f);
    one.colours.emplace_back(1.0f, 1.0f, 1.0f);
    flow_movie::fade_polyline_ends(one, 0.15f);  // nothing to fade along: no crash, no change
    EXPECT_EQ(one.colours.front(), glm::vec3(1.0f));
    EXPECT_TRUE(one.alphas.empty());
}

TEST(FlowMovieLook, OnlyTheLinesAddedByTheViewAreFaded) {
    spade::render::RenderScene scene;
    scene.flow_polylines.push_back(straight_line(11, 0.05f));  // a rim, say: drawn before the streamlines
    const std::size_t from = scene.flow_polylines.size();
    scene.flow_polylines.push_back(straight_line(11, 0.05f));
    flow_movie::fade_polylines_from(scene, from, 0.15f);
    EXPECT_TRUE(scene.flow_polylines[0].alphas.empty()) << "the rim drawn before is not faded";
    ASSERT_FALSE(scene.flow_polylines[1].alphas.empty());
    EXPECT_EQ(scene.flow_polylines[1].alphas.front(), 0.0f);
    EXPECT_EQ(scene.flow_polylines[1].colours.front(), glm::vec3(1.0f, 0.5f, 0.25f)) << "faded by alpha, not toward black";
}

// The soft edge of a drawn wake (the lead, 2026-11-17: the wake fades out before the frame's bottom edge, and a wake that meets
// the box's side face does not show the face as a straight cut): the velocity scaled by a smoothstep of the distance to the box's
// side faces (x and z) over side_m, and by a smoothstep of the height from floor_from_y (full) down to floor_to_y (nothing). The
// dye channel, a no-data half and every other channel are left alone; the header's velocity range is made again.
namespace {
spade::FlowVolume uniform_down_flow(uint32_t n, float cell, glm::vec3 origin) {
    spade::FlowVolume v;
    v.header.origin = origin;
    v.header.cell = cell;
    v.header.nx = v.header.ny = v.header.nz = n;
    const std::size_t cells = std::size_t{n} * n * n;
    v.v.assign(cells * spade::kFlowHalvesPerCell, 0u);
    v.s.assign(cells * spade::kFlowHalvesPerCell, 0u);
    for (std::size_t c = 0; c < cells; ++c) {
        v.v[c * spade::kFlowHalvesPerCell + 1] = spade::fp16::from_f32(-5.0f).bits;  // v: 5 m/s down
        v.v[c * spade::kFlowHalvesPerCell + 3] = spade::fp16::from_f32(0.75f).bits;  // the dye
    }
    return v;
}
float speed_down(const spade::FlowVolume& v, uint32_t i, uint32_t j, uint32_t k) {
    return -spade::fp16::to_f32(v.v[spade::flow_cell_index(v.header, i, j, k) * spade::kFlowHalvesPerCell + 1]);
}
}  // namespace

TEST(FlowMovieLook, AWakeFadesOutOverTheSideFacesAndBelowTheFloorHeightAndNothingElseMoves) {
    auto v = uniform_down_flow(12, 0.1f, glm::vec3(-0.6f, -1.2f, -0.6f));  // a 1.2 m cube: y from -1.2 to 0
    const auto before = v;
    flow_movie::WakeFade fade;
    fade.side_m = 0.2f;          // two cells
    fade.floor_from_y = -0.6f;   // full above this
    fade.floor_to_y = -1.0f;     // nothing at or below this
    flow_movie::fade_wake_edges(v, fade);
    EXPECT_FLOAT_EQ(speed_down(v, 6, 9, 6), 5.0f) << "the middle of the side faces, above the fade: untouched";
    const float smooth_quarter = 0.25f * 0.25f * (3.0f - 2.0f * 0.25f);  // 0.05 m from a face of a 0.2 m fade
    EXPECT_NEAR(speed_down(v, 0, 9, 6), 5.0f * smooth_quarter, 5.0f * 2e-3f) << "a cell centre 0.05 m from the -x face";
    EXPECT_NEAR(speed_down(v, 6, 9, 11), 5.0f * smooth_quarter, 5.0f * 2e-3f) << "and from the +z face";
    EXPECT_NEAR(speed_down(v, 0, 9, 0), 5.0f * smooth_quarter * smooth_quarter, 5.0f * 2e-3f) << "a corner takes both";
    EXPECT_FLOAT_EQ(speed_down(v, 6, 0, 6), 0.0f) << "below floor_to_y: nothing";
    const float t = (-0.6f - -0.75f) / 0.4f;  // the cell centre at y = -0.75 is 0.375 of the way down the fade
    EXPECT_NEAR(speed_down(v, 6, 4, 6), 5.0f * (1.0f - t * t * (3.0f - 2.0f * t)), 5.0f * 2e-3f);
    for (std::size_t c = 0; c < std::size_t{12} * 12 * 12; ++c) {
        EXPECT_EQ(v.v[c * spade::kFlowHalvesPerCell + 3], before.v[c * spade::kFlowHalvesPerCell + 3]) << "the dye";
        EXPECT_EQ(v.v[c * spade::kFlowHalvesPerCell + 0], before.v[c * spade::kFlowHalvesPerCell + 0]) << "u was zero";
    }
    EXPECT_EQ(v.s, before.s) << "the second texture is not touched";
    EXPECT_NE(v.generation, before.generation) << "a write names new content";
    EXPECT_LE(v.header.channel_max[1], 0.0f) << "the velocity range is made again";
    EXPECT_GE(v.header.channel_min[1], -5.001f);

    auto unchanged = before;
    flow_movie::fade_wake_edges(unchanged, flow_movie::WakeFade{});
    EXPECT_EQ(unchanged.v, before.v) << "no fade asked, nothing changes";
    EXPECT_FALSE(flow_movie::WakeFade{}.on());
    EXPECT_TRUE(fade.on());
}

TEST(FlowMovieLook, ANoDataHalfStaysNoDataUnderTheFade) {
    auto v = uniform_down_flow(8, 0.1f, glm::vec3(-0.4f, -0.8f, -0.4f));
    const std::size_t cell = spade::flow_cell_index(v.header, 0, 4, 4);
    v.v[cell * spade::kFlowHalvesPerCell + 1] = spade::fp16::kNoDataBits;
    flow_movie::WakeFade fade;
    fade.side_m = 0.2f;
    flow_movie::fade_wake_edges(v, fade);
    EXPECT_EQ(v.v[cell * spade::kFlowHalvesPerCell + 1], spade::fp16::kNoDataBits);
}

// The look line declares the fade, never hides it.
TEST(FlowMovieLook, TheFadeIsDeclaredInTheLooksSentence) {
    flow_movie::Look look;
    look.fade.side_m = 0.1f;
    look.fade.floor_from_y = -0.33f;
    look.fade.floor_to_y = -0.55f;
    EXPECT_TRUE(look.any());
    const std::string note = flow_movie::look_note(look, 2.0);
    EXPECT_NE(note.find("fades"), std::string::npos) << note;
    EXPECT_NE(note.find("0.1 m"), std::string::npos) << note;
    EXPECT_NE(note.find("0.55"), std::string::npos) << note;
    EXPECT_FALSE(flow_movie::Look{}.any());
}

// The slow inflow above the rotors' plane is a haze between the camera and the airframe: the fade can also take the wake out
// above a height (full below ceiling_from_y, nothing at or above ceiling_to_y), so the drone reads in front of the wake it makes.
TEST(FlowMovieLook, AWakeFadesOutAboveACeilingHeightToo) {
    auto v = uniform_down_flow(12, 0.1f, glm::vec3(-0.6f, -1.2f, -0.6f));
    flow_movie::WakeFade fade;
    fade.ceiling_from_y = -0.4f;
    fade.ceiling_to_y = -0.2f;
    EXPECT_TRUE(fade.on()) << "a ceiling alone is a fade";
    flow_movie::fade_wake_edges(v, fade);
    EXPECT_FLOAT_EQ(speed_down(v, 6, 4, 6), 5.0f) << "y = -0.75: below the ceiling's start";
    EXPECT_FLOAT_EQ(speed_down(v, 6, 11, 6), 0.0f) << "y = -0.05: above where it is gone";
    const float t = (-0.25f - -0.4f) / 0.2f;  // the cell centre at y = -0.25 is 0.75 of the way up the fade
    EXPECT_NEAR(speed_down(v, 6, 9, 6), 5.0f * (1.0f - t * t * (3.0f - 2.0f * t)), 5.0f * 2e-3f);
    flow_movie::Look look;
    look.fade = fade;
    EXPECT_NE(flow_movie::look_note(look, 2.0).find("above"), std::string::npos) << "declared, not hidden";
}

// The drawing's per-frame field steps are one function (main.cpp's apply_look calls it after the mean flow): smoothing, the cull and the
// wake's fade, in that order. The fade was once written, tested alone and never called: the stills drew the unfaded wake. A Look with
// only a fade must change the volume through THIS function, and a Look with nothing must not.
TEST(FlowMovieLook, TheFieldStepsOfALookIncludeTheFade) {
    auto v = uniform_down_flow(12, 0.1f, glm::vec3(-0.6f, -1.2f, -0.6f));
    const auto before = v;
    flow_movie::Look nothing;
    flow_movie::finish_look(nothing, v);
    EXPECT_EQ(v.v, before.v);
    flow_movie::Look fades;
    fades.fade.side_m = 0.2f;
    EXPECT_TRUE(fades.any());
    flow_movie::finish_look(fades, v);
    EXPECT_LT(speed_down(v, 0, 9, 6), 1.0f) << "the cell 0.05 m from the -x face is faded";
    EXPECT_FLOAT_EQ(speed_down(v, 6, 9, 6), 5.0f) << "the middle is not";
}

// The particle PREVIEW's seeds (Cameron, 2026-11-17: "the airflow needs to surround the drone blades", "it can feel particle-y"):
// a deterministic hashed fill of the shell above and round each rotor, where air is drawn in, and of a far-field box round the
// drone. No RNG: the same inputs give the same seeds. Rotors are (hub, axis, radius); the axis points the way thrust points,
// so "above" is along it.
TEST(FlowMovieLook, ThePreviewSeedsFillTheInflowShellAboveEachRotorAndAFarFieldBoxAndAreTheSameEveryTime) {
    std::vector<flow_movie::PreviewRotor> rotors;
    for (const float x : {-0.125f, 0.125f}) {
        for (const float z : {-0.125f, 0.125f}) rotors.push_back({glm::vec3(x, 0.026f, z), glm::vec3(0.0f, 1.0f, 0.0f), 0.119f});
    }
    const flow_movie::PreviewBox box{glm::vec3(-0.45f, -0.05f, -0.30f), glm::vec3(0.45f, 0.40f, 0.30f)};
    const auto a = flow_movie::preview_seeds(rotors, box, 150u, 400u);
    const auto b = flow_movie::preview_seeds(rotors, box, 150u, 400u);
    ASSERT_EQ(a.size(), 4u * 150u + 400u);
    EXPECT_EQ(a, b) << "deterministic";
    for (std::size_t r = 0; r < 4u; ++r) {
        for (std::size_t i = 0; i < 150u; ++i) {
            const glm::vec3 p = a[r * 150u + i];
            EXPECT_GT(p.y, rotors[r].hub.y) << "above the rotor's plane";
            EXPECT_LT(p.y, rotors[r].hub.y + 0.35f);
            const glm::vec3 d = p - rotors[r].hub;
            EXPECT_LE(std::sqrt(d.x * d.x + d.z * d.z), 2.0f * rotors[r].radius + 1e-5f) << "within two radii of the axis";
        }
    }
    for (std::size_t i = 4u * 150u; i < a.size(); ++i) {
        EXPECT_GE(a[i].x, box.lo.x);
        EXPECT_LE(a[i].x, box.hi.x);
        EXPECT_GE(a[i].y, box.lo.y);
        EXPECT_LE(a[i].y, box.hi.y);
        EXPECT_GE(a[i].z, box.lo.z);
        EXPECT_LE(a[i].z, box.hi.z);
    }
    std::vector<glm::vec3> sorted = a;
    std::sort(sorted.begin(), sorted.end(), [](const glm::vec3& l, const glm::vec3& r) {
        return l.x != r.x ? l.x < r.x : (l.y != r.y ? l.y < r.y : l.z < r.z);
    });
    EXPECT_EQ(std::adjacent_find(sorted.begin(), sorted.end()), sorted.end()) << "no two seeds are the same point";
}

// THE NASA LOOK (the lead, 2026-11-17: Cameron wants the multirotor flow figures of NASA Ames, OVERFLOW: Q-criterion iso-surfaces
// coloured by vorticity magnitude, blue low to magenta high, crisp thin tubes and sheets): the published frame's q_hat channel is
// sqrt(Q) with its sign, so an iso level L on it is Q = L^2. A LADDER of three levels, each a fraction of the frame's largest q_hat
// (the thin cores at the top, the sheets low), each its own opacity (the cores near solid, the outer sheets see-through), all coloured
// by the vorticity channel on the blue-to-magenta map between 0 and a fraction of the frame's largest |omega|.
namespace {
spade::FlowVolume volume_with_ranges(float q_max, float vort_max) {
    spade::FlowVolume v;
    v.header.channel_max[spade::kFlowQHat] = q_max;
    v.header.channel_min[spade::kFlowQHat] = -q_max;
    v.header.channel_max[spade::kFlowVorticity] = vort_max;
    return v;
}
}  // namespace

TEST(FlowMovieQLook, TheLadderIsThreeLevelsAsFractionsOfTheFramesLargestQHatColouredByVorticity) {
    const flow_movie::QLook look;
    const auto isos = flow_movie::q_isos(volume_with_ranges(10.0f, 200.0f), look);
    ASSERT_EQ(isos.size(), 3u);
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(isos[i].quantity, spade::render::FlowQuantity::q_hat) << i;
        EXPECT_NEAR(isos[i].value, 10.0f * look.level[i], 1e-5f) << i;
        EXPECT_FLOAT_EQ(isos[i].opacity, look.opacity[i]) << i;
        ASSERT_TRUE(isos[i].colour_by.has_value()) << i;
        EXPECT_EQ(isos[i].colour_by->quantity, spade::render::FlowQuantity::vorticity) << i;
        EXPECT_EQ(isos[i].colour_by->colours, spade::render::ColourMapId::blue_magenta) << i;
        EXPECT_FALSE(isos[i].colour_by->auto_range) << "a fixed range: the colours mean the same in every frame of a clip";
        EXPECT_FLOAT_EQ(isos[i].colour_by->range_min, 0.0f) << i;
        EXPECT_NEAR(isos[i].colour_by->range_max, 200.0f * look.vorticity_max, 1e-4f) << i;
    }
    EXPECT_GT(isos[0].value, isos[1].value);
    EXPECT_GT(isos[1].value, isos[2].value) << "the highest level first";
    EXPECT_GT(isos[0].opacity, isos[2].opacity) << "the cores are the most solid";
}

TEST(FlowMovieQLook, AFrameWithNoVorticityToShowGivesNoSurfaces) {
    const flow_movie::QLook look;
    EXPECT_TRUE(flow_movie::q_isos(volume_with_ranges(0.0f, 200.0f), look).empty()) << "no q_hat above zero";
    EXPECT_TRUE(flow_movie::q_isos(volume_with_ranges(-1.0f, 200.0f), look).empty());
    EXPECT_TRUE(flow_movie::q_isos(volume_with_ranges(10.0f, 0.0f), look).empty()) << "no vorticity range to colour by";
    EXPECT_TRUE(flow_movie::q_isos(volume_with_ranges(std::numeric_limits<float>::quiet_NaN(), 200.0f), look).empty());
}

TEST(FlowMovieQLook, TheQLookStringSetsTheNamedNumbersAndRefusesWhatCannotBeDrawnByName) {
    const auto none = flow_movie::parse_q_look("");
    ASSERT_TRUE(none.has_value()) << none.error().context;
    const flow_movie::QLook defaults;
    EXPECT_EQ(none->level, defaults.level);
    const auto some = flow_movie::parse_q_look("l1=0.6, l2=0.3,l3=0.05,o1=1,o2=0.5,o3=0.2,vmax=0.8");
    ASSERT_TRUE(some.has_value()) << some.error().context;
    EXPECT_FLOAT_EQ(some->level[0], 0.6f);
    EXPECT_FLOAT_EQ(some->level[2], 0.05f);
    EXPECT_FLOAT_EQ(some->opacity[0], 1.0f);
    EXPECT_FLOAT_EQ(some->opacity[2], 0.2f);
    EXPECT_FLOAT_EQ(some->vorticity_max, 0.8f);
    for (const char* bad : {"colour=red", "l1", "l1=", "l1=abc", "l1=0", "l1=1.5", "o1=0", "o2=1.1", "vmax=0", "vmax=2", "l1=0.2,l2=0.3",
                            "l2=0.05", "l1=0.2,,l2=0.1", "=1"}) {
        const auto r = flow_movie::parse_q_look(bad);
        ASSERT_FALSE(r.has_value()) << bad;
        EXPECT_NE(r.error().context.find("q look"), std::string::npos) << bad << ": " << r.error().context;
    }
    EXPECT_NE(flow_movie::parse_q_look("sparkle=1").error().context.find("sparkle"), std::string::npos);
}

// THE CLOUD (the lead, 2026-11-17: Cameron also asked for "fluffy cloud-like air"): the same vorticity colours as a soft VOLUME, no
// surfaces: |omega| on the blue-to-magenta map over 0 .. vorticity_max x the frame's largest |omega| (the ladder's own ceiling),
// absorbing from an onset of the range up to a modest sigma at its top, so the strongest air is the densest and the weak air a haze.
TEST(FlowMovieQLook, TheCloudIsVorticityOnTheBlueMagentaMapOverTheLaddersRangeAndGetsDenserWithIt) {
    const flow_movie::QLook look;
    const auto tf = flow_movie::vorticity_cloud(volume_with_ranges(10.0f, 200.0f), look);
    ASSERT_TRUE(tf.has_value());
    EXPECT_EQ(tf->quantity, spade::render::FlowQuantity::vorticity);
    EXPECT_EQ(tf->colours, spade::render::ColourMapId::blue_magenta);
    EXPECT_FALSE(tf->auto_range);
    EXPECT_FLOAT_EQ(tf->range_min, 0.0f);
    EXPECT_NEAR(tf->range_max, 200.0f * look.vorticity_max, 1e-4f);
    ASSERT_GE(tf->extinction.size(), 2u);
    EXPECT_EQ(tf->extinction.front().sigma, 0.0f) << "no absorption in the weakest air: a haze, not a wall";
    EXPECT_GT(tf->extinction.back().sigma, 0.0f);
    EXPECT_LE(tf->extinction.back().sigma, 60.0f) << "soft: the densest air is still see-through at a few centimetres";
    for (std::size_t i = 1; i < tf->extinction.size(); ++i) {
        EXPECT_GE(tf->extinction[i].sigma, tf->extinction[i - 1].sigma) << "denser with vorticity, " << i;
    }
    EXPECT_TRUE(spade::render::validate_transfer_function(*tf).has_value());
    EXPECT_FALSE(flow_movie::vorticity_cloud(volume_with_ranges(10.0f, 0.0f), look).has_value()) << "no vorticity to show";
    EXPECT_FALSE(flow_movie::vorticity_cloud(volume_with_ranges(10.0f, std::numeric_limits<float>::infinity()), look).has_value());
}

// THE CLOUD'S TWO NUMBERS (Cameron picked the soft cloud, L86o; the lead 2026-11-18: tune the density and the cloud near the body):
// sigma, the density at the top of the range in 1/m (30 now), and onset, the fraction of the range below which the air is clear
// (0.15 now); both in --q-look, so a redraw of the kept frame shows each. And the cloud THINNED NEAR THE BODY: the vorticity within
// thin_r metres of the drone is scaled by a factor from thin_floor at its centre back to 1 at thin_r (a smoothstep), so the airframe
// reads through the haze. The frame's own header range is not remade: the colours keep the range of the unthinned frame.
TEST(FlowMovieQLook, TheCloudsDensityOnsetAndThinningAreNamedInTheLookString) {
    const flow_movie::QLook defaults;
    EXPECT_FLOAT_EQ(defaults.sigma, 30.0f);
    EXPECT_FLOAT_EQ(defaults.onset, 0.15f);
    EXPECT_FLOAT_EQ(defaults.thin_r, 0.0f) << "no thinning unless asked";
    const auto look = flow_movie::parse_q_look("sigma=60,onset=0.3,thin_r=0.15,thin_floor=0.2");
    ASSERT_TRUE(look.has_value()) << look.error().context;
    EXPECT_FLOAT_EQ(look->sigma, 60.0f);
    EXPECT_FLOAT_EQ(look->onset, 0.3f);
    EXPECT_FLOAT_EQ(look->thin_r, 0.15f);
    EXPECT_FLOAT_EQ(look->thin_floor, 0.2f);
    for (const char* bad : {"sigma=0", "sigma=2000", "onset=1", "onset=-0.1", "thin_r=-1", "thin_r=3", "thin_floor=1.5", "thin_floor=-0.1"}) {
        const auto r = flow_movie::parse_q_look(bad);
        ASSERT_FALSE(r.has_value()) << bad;
        EXPECT_NE(r.error().context.find("q look"), std::string::npos) << bad << ": " << r.error().context;
    }
    // thin_r is a multiple of the drone's own size (Framing::subject_m), so one value fits every drone: up to 2 of it
    const auto wide = flow_movie::parse_q_look("thin_r=1.5");
    ASSERT_TRUE(wide.has_value()) << wide.error().context;
    EXPECT_FLOAT_EQ(wide->thin_r, 1.5f);
    flow_movie::QLook dense;
    dense.sigma = 80.0f;
    dense.onset = 0.4f;
    const auto tf = flow_movie::vorticity_cloud(volume_with_ranges(10.0f, 200.0f), dense);
    ASSERT_TRUE(tf.has_value());
    EXPECT_FLOAT_EQ(tf->extinction.back().sigma, 80.0f) << "the density at the top of the range";
    bool clear_below = false;
    for (const auto& point : tf->extinction) {
        if (std::abs(point.value - 0.4f * tf->range_max) < 1e-3f * tf->range_max && point.sigma == 0.0f) clear_below = true;  // the onset is the last clear point
    }
    EXPECT_TRUE(clear_below) << "no absorption below the onset";
}

TEST(FlowMovieLook, TheVorticityIsThinnedNearTheDroneAndNothingElseChanges) {
    spade::FlowVolume v = uniform_down_flow(16, 0.01f, glm::vec3(-0.08f));
    const std::size_t cells = std::size_t{16} * 16 * 16;
    for (std::size_t c = 0; c < cells; ++c) v.s[c * spade::kFlowHalvesPerCell + 1] = spade::fp16::from_f32(100.0f).bits;  // |omega|
    v.header.channel_max[spade::kFlowVorticity] = 100.0f;
    const spade::FlowVolume before = v;
    flow_movie::CloudThin thin;
    thin.centre = glm::vec3(0.005f);  // the centre of cell (8, 8, 8)
    thin.radius = 0.05f;
    thin.floor = 0.3f;
    EXPECT_TRUE(thin.on());
    EXPECT_FALSE(flow_movie::CloudThin{}.on());
    flow_movie::thin_vorticity(v, thin);
    const auto at = [&v](uint32_t i, uint32_t j, uint32_t k) {
        return spade::fp16::to_f32(v.s[spade::flow_cell_index(v.header, i, j, k) * spade::kFlowHalvesPerCell + 1]);
    };
    EXPECT_NEAR(at(8, 8, 8), 30.0f, 0.1f) << "at the centre: the floor";
    EXPECT_NEAR(at(0, 0, 0), 100.0f, 0.1f) << "far from it: untouched";
    const float near = at(10, 8, 8);  // 2 cells = 0.02 m away
    EXPECT_GT(near, 30.0f);
    EXPECT_LT(near, 100.0f) << "in between, smoothly";
    EXPECT_NEAR(at(13, 8, 8), 100.0f, 0.1f) << "past the radius (5 cells): untouched";
    EXPECT_EQ(v.v, before.v) << "the velocity and the dye are not touched";
    for (std::size_t c = 0; c < cells; ++c) {
        for (std::size_t lane = 0; lane < spade::kFlowHalvesPerCell; ++lane) {
            if (lane == 1) continue;
            ASSERT_EQ(v.s[c * spade::kFlowHalvesPerCell + lane], before.s[c * spade::kFlowHalvesPerCell + lane]) << c << "/" << lane;
        }
    }
    EXPECT_FLOAT_EQ(v.header.channel_max[spade::kFlowVorticity], 100.0f) << "the colours keep the unthinned frame's range";
    flow_movie::Look look;
    look.thin = thin;
    EXPECT_TRUE(look.any()) << "a thinned look is a look";
    EXPECT_NE(flow_movie::look_note(look, 1.0).find("thinned"), std::string::npos);
    spade::FlowVolume w = before;
    flow_movie::finish_look(look, w);
    EXPECT_NEAR(spade::fp16::to_f32(w.s[spade::flow_cell_index(w.header, 8, 8, 8) * spade::kFlowHalvesPerCell + 1]), 30.0f, 0.1f)
        << "finish_look thins";
}

// THE THINNING IS A PRESENTATION CHOICE THAT HIDES AIR NEAR THE DRONE, so the record says it in plain words (the lead, 2026-11-18).
TEST(FlowMovieLook, TheRecordSaysTheAirNearTheDroneIsDrawnThinnedAndTheFieldIsUnchanged) {
    flow_movie::Look look;
    look.thin.radius = 0.7461f;
    look.thin.floor = 0.0f;
    const std::string note = flow_movie::look_note(look, 1.0);
    EXPECT_NE(note.find("the air within 0.75 m of the drone is drawn thinned so the drone shows; the field is unchanged"), std::string::npos) << note;
}

// THE CLIP-WIDE RANGE (the lead, O144: the per-frame top made the cloud step by up to 13.6% between frames): with a peak set (the largest |omega| over the clip's frames, read from the cache's
// headers before drawing) the cloud's, and the ladder's, top is vorticity_max x that peak in EVERY frame, so a colour means the same vorticity throughout; with none set (0) each frame's own
// largest is the reference, as it always was.
TEST(FlowMovieQLook, AClipWidePeakFixesTheTopForEveryFrameAndZeroLeavesEachFramesOwn) {
    flow_movie::QLook look;
    EXPECT_FLOAT_EQ(look.vorticity_peak, 0.0f) << "per frame unless a run sets it";
    const auto own = flow_movie::vorticity_cloud(volume_with_ranges(10.0f, 200.0f), look);
    ASSERT_TRUE(own.has_value());
    EXPECT_NEAR(own->range_max, 200.0f * look.vorticity_max, 1e-4f);
    look.vorticity_peak = 400.0f;
    const auto weak = flow_movie::vorticity_cloud(volume_with_ranges(10.0f, 200.0f), look);
    const auto strong = flow_movie::vorticity_cloud(volume_with_ranges(10.0f, 380.0f), look);
    ASSERT_TRUE(weak.has_value());
    ASSERT_TRUE(strong.has_value());
    EXPECT_NEAR(weak->range_max, 400.0f * look.vorticity_max, 1e-4f);
    EXPECT_FLOAT_EQ(weak->range_max, strong->range_max) << "the same top in a weak frame and a strong one";
    EXPECT_FALSE(flow_movie::vorticity_cloud(volume_with_ranges(10.0f, 0.0f), look).has_value()) << "a frame with no vorticity still shows none";
    const auto isos = flow_movie::q_isos(volume_with_ranges(10.0f, 200.0f), look);
    ASSERT_FALSE(isos.empty());
    ASSERT_TRUE(isos[0].colour_by.has_value());
    EXPECT_NEAR(isos[0].colour_by->range_max, 400.0f * look.vorticity_max, 1e-4f) << "the ladder shares the cloud's range";
}

// ---- O158 / O163: the Q-surface tubes coloured by the air's pressure, kept stable -----------------------------------------------------------------------------------------------
// The tubes are the Q ladder with `colour_by` the PRESSURE on the parts' own cool_warm map (the pressure film's colour, so the whole film is one pressure picture), one fixed range for
// the clip; their levels are fractions of the CLIP's largest q_hat when a run sets q_peak (so a tube means the same Q in every frame and does not pop with the frame's peak); and the q_hat
// the tubes are cut from can be smoothed on a display-only copy.

TEST(FlowMovieQLook, ThePressureColourIsCoolWarmOverTheStillAirsPressurePlusAndMinusTheRangeInDiscLoadings) {
    namespace rd = spade::render;
    const auto tf = flow_movie::pressure_transfer(2.0f, 50.0f, 0.8f);
    ASSERT_TRUE(tf.has_value());
    EXPECT_EQ(tf->quantity, rd::FlowQuantity::pressure);
    EXPECT_EQ(tf->colours, rd::ColourMapId::cool_warm) << "the parts' own map";
    EXPECT_FALSE(tf->auto_range) << "fixed: the same colour is the same pressure in every frame";
    EXPECT_NEAR(tf->range_min, 2.0f - 0.8f * 50.0f, 1e-4f) << "the still air's pressure minus the range in disc loadings";
    EXPECT_NEAR(tf->range_max, 2.0f + 0.8f * 50.0f, 1e-4f);
    EXPECT_TRUE(rd::validate_transfer_function(*tf).has_value());
    // the still air's pressure is the map's neutral middle; the ends are its ends
    spade::FlowVolumeHeader header;
    const rd::PreparedTransfer prepared = rd::prepare_transfer(*tf, header);
    const glm::vec3 mid = rd::apply_transfer(prepared, 2.0f).colour;
    const glm::vec3 neutral = rd::colour_map_at(rd::ColourMapId::cool_warm, 0.5f);
    EXPECT_NEAR(mid.x, neutral.x, 1e-3f);
    EXPECT_NEAR(mid.y, neutral.y, 1e-3f);
    EXPECT_NEAR(mid.z, neutral.z, 1e-3f);
    const glm::vec3 top = rd::apply_transfer(prepared, 2.0f + 0.8f * 50.0f).colour;
    const glm::vec3 warm = rd::colour_map_at(rd::ColourMapId::cool_warm, 1.0f);
    EXPECT_NEAR(top.x, warm.x, 1e-3f);
    EXPECT_NEAR(top.z, warm.z, 1e-3f);
    EXPECT_FALSE(flow_movie::pressure_transfer(2.0f, 0.0f, 0.8f).has_value()) << "no disc loading, no scale";
    EXPECT_FALSE(flow_movie::pressure_transfer(2.0f, 50.0f, 0.0f).has_value());
    EXPECT_FALSE(flow_movie::pressure_transfer(std::numeric_limits<float>::quiet_NaN(), 50.0f, 0.8f).has_value());
}

TEST(FlowMovieQLook, TheTubesTakeAPressureColourWhenGivenOneAndNeedNoVorticityThen) {
    namespace rd = spade::render;
    const auto tf = flow_movie::pressure_transfer(0.0f, 50.0f, 0.8f);
    ASSERT_TRUE(tf.has_value());
    const flow_movie::QLook look;
    const auto isos = flow_movie::q_isos(volume_with_ranges(10.0f, 0.0f), look, &*tf);
    ASSERT_EQ(isos.size(), 3u) << "coloured by pressure, a frame with no vorticity still has its tubes";
    for (std::size_t i = 0; i < 3; ++i) {
        ASSERT_TRUE(isos[i].colour_by.has_value()) << i;
        EXPECT_EQ(isos[i].colour_by->quantity, rd::FlowQuantity::pressure) << i;
        EXPECT_EQ(isos[i].colour_by->colours, rd::ColourMapId::cool_warm) << i;
        EXPECT_FLOAT_EQ(isos[i].colour_by->range_max, tf->range_max) << i;
        EXPECT_NEAR(isos[i].value, 10.0f * look.level[i], 1e-5f) << i;
    }
    // without one, as before: coloured by vorticity, and none for a frame with none
    EXPECT_TRUE(flow_movie::q_isos(volume_with_ranges(10.0f, 0.0f), look).empty());
    EXPECT_EQ(flow_movie::q_isos(volume_with_ranges(10.0f, 200.0f), look)[0].colour_by->quantity, rd::FlowQuantity::vorticity);
}

TEST(FlowMovieQLook, AClipWideQPeakMakesEveryFramesTubeLevelTheSameAndZeroLeavesEachFramesOwn) {
    flow_movie::QLook look;
    EXPECT_FLOAT_EQ(look.q_peak, 0.0f) << "per frame unless a run sets it";
    const auto own = flow_movie::q_isos(volume_with_ranges(10.0f, 200.0f), look);
    ASSERT_EQ(own.size(), 3u);
    EXPECT_NEAR(own[0].value, 10.0f * look.level[0], 1e-5f);
    look.q_peak = 20.0f;
    const auto weak = flow_movie::q_isos(volume_with_ranges(10.0f, 200.0f), look);
    const auto strong = flow_movie::q_isos(volume_with_ranges(15.0f, 200.0f), look);
    ASSERT_EQ(weak.size(), 3u);
    ASSERT_EQ(strong.size(), 3u);
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(weak[i].value, 20.0f * look.level[i], 1e-5f) << "the clip's largest q_hat, not the frame's: " << i;
        EXPECT_FLOAT_EQ(weak[i].value, strong[i].value) << "the same level in a weak frame and a strong one: " << i;
    }
    EXPECT_TRUE(flow_movie::q_isos(volume_with_ranges(0.0f, 200.0f), look).empty()) << "a frame with no Q above zero shows no tubes, whatever the clip's peak";
}

namespace {
void set_q(FlowVolume& v, uint32_t i, uint32_t j, uint32_t k, float value) {
    v.s[at(v, i, j, k) * spade::kFlowHalvesPerCell + (spade::kFlowQHat % spade::kFlowHalvesPerCell)] = fp16::from_f32(value).bits;
}
[[nodiscard]] float q_at(const FlowVolume& v, uint32_t i, uint32_t j, uint32_t k) {
    return fp16::to_f32(v.s[at(v, i, j, k) * spade::kFlowHalvesPerCell + (spade::kFlowQHat % spade::kFlowHalvesPerCell)]);
}
}  // namespace

// q_hat smoothed over one cell, on a copy (the caller's: the cache's frame is never written): the same Gaussian as lambda2's, normalised over the valid neighbours; the header's range
// of q_hat follows the work; the other channels are not touched.
TEST(FlowMovieLook, QHatSmoothsLikeLambda2OnlyInItsOwnChannelAndTheRangeFollows) {
    FlowVolume v = blank(11, 11, 11);
    set_q(v, 5, 5, 5, 1000.0f);
    set_l2(v, 2, 2, 2, -500.0f);
    const uint16_t l2_before = l2_bits(v, 2, 2, 2);
    flow_movie::smooth_s_channel(v, spade::kFlowQHat);
    const std::vector<double> w = kernel();
    EXPECT_NEAR(q_at(v, 5, 5, 5), 1000.0 * w[3] * w[3] * w[3], 1.5) << "the centre";
    EXPECT_NEAR(q_at(v, 6, 5, 5), 1000.0 * w[4] * w[3] * w[3], 1.0) << "a face neighbour";
    double total = 0.0;
    for (uint32_t k = 0; k < 11; ++k) {
        for (uint32_t j = 0; j < 11; ++j) {
            for (uint32_t i = 0; i < 11; ++i) total += q_at(v, i, j, k);
        }
    }
    EXPECT_NEAR(total, 1000.0, 15.0) << "spread, not lost";
    EXPECT_EQ(l2_bits(v, 2, 2, 2), l2_before) << "lambda2-hat is another channel: untouched";
    EXPECT_NEAR(v.header.channel_max[spade::kFlowQHat], q_at(v, 5, 5, 5), 1e-3f) << "the range a view reads is the smoothed field's";
    EXPECT_GE(v.header.channel_min[spade::kFlowQHat], 0.0f);

    // a constant stays constant, and a no-data cell is neither read nor written
    FlowVolume c = blank(7, 7, 7);
    for (uint32_t k = 0; k < 7; ++k) {
        for (uint32_t j = 0; j < 7; ++j) {
            for (uint32_t i = 0; i < 7; ++i) set_q(c, i, j, k, 3.0f);
        }
    }
    const uint16_t hole = 0x7E00u;  // a NaN: the solver's no-data
    c.s[at(c, 3, 3, 3) * spade::kFlowHalvesPerCell + (spade::kFlowQHat % spade::kFlowHalvesPerCell)] = hole;
    flow_movie::smooth_s_channel(c, spade::kFlowQHat);
    EXPECT_FLOAT_EQ(q_at(c, 2, 3, 3), 3.0f);
    EXPECT_FLOAT_EQ(q_at(c, 0, 0, 0), 3.0f);
    EXPECT_EQ(c.s[at(c, 3, 3, 3) * spade::kFlowHalvesPerCell + (spade::kFlowQHat % spade::kFlowHalvesPerCell)], hole);
}

// ---- O158: the tubes' steadiness, part two: a centred mean of the Q and pressure channels, and the caption's words ---------------------------------------------------------------
// The window of frames a centred mean reads: the drawn frame in the middle, fewer at the ends of the clip (never past either end).
TEST(FlowMovieLook, ACentredWindowHasTheDrawnFrameInTheMiddleAndStopsAtTheEnds) {
    const auto mid = flow_movie::centred_window(5, 3, 10);
    EXPECT_EQ(mid.first, 4u);
    EXPECT_EQ(mid.count, 3u);
    const auto start = flow_movie::centred_window(0, 3, 10);
    EXPECT_EQ(start.first, 0u);
    EXPECT_EQ(start.count, 2u) << "the frame and the one after it";
    const auto end = flow_movie::centred_window(9, 3, 10);
    EXPECT_EQ(end.first, 8u);
    EXPECT_EQ(end.count, 2u);
    const auto five = flow_movie::centred_window(5, 5, 10);
    EXPECT_EQ(five.first, 3u);
    EXPECT_EQ(five.count, 5u);
    const auto one = flow_movie::centred_window(7, 1, 10);
    EXPECT_EQ(one.first, 7u);
    EXPECT_EQ(one.count, 1u) << "1 frame: the frame alone";
    const auto whole = flow_movie::centred_window(1, 9, 3);
    EXPECT_EQ(whole.first, 0u);
    EXPECT_EQ(whole.count, 3u) << "a window wider than the clip is the clip";
}

// The drawn frame's chosen s-texture channels become the mean over the window's frames; a cell that is no-data in any is no-data in the mean; the other channels, and the velocity, are the
// drawn frame's own; the header's range of a meaned channel follows.
TEST(FlowMovieLook, TheScalarMeanAveragesOnlyTheNamedChannelsAndAnyNoDataStaysNoData) {
    FlowVolume a = blank(4, 4, 4), b = blank(4, 4, 4), c = blank(4, 4, 4);
    set_q(a, 1, 1, 1, 3.0f);
    set_q(b, 1, 1, 1, 6.0f);
    set_q(c, 1, 1, 1, 9.0f);
    set_q(a, 2, 2, 2, 1.0f);
    set_q(b, 2, 2, 2, 1.0f);
    c.s[at(c, 2, 2, 2) * spade::kFlowHalvesPerCell + (spade::kFlowQHat % spade::kFlowHalvesPerCell)] = 0x7E00u;  // no-data in one frame
    set_l2(b, 0, 0, 0, -40.0f);                                                                              // another channel, in the drawn frame
    set_u(b, 3, 3, 3, 5.0f);
    FlowVolume out = b;  // the drawn frame is the middle one
    const std::vector<const FlowVolume*> frames = {&a, &b, &c};
    const std::vector<uint32_t> channels = {spade::kFlowQHat};
    const auto done = flow_movie::mean_scalars(out, frames, channels);
    ASSERT_TRUE(done.has_value()) << (done ? "" : done.error().context);
    EXPECT_NEAR(q_at(out, 1, 1, 1), 6.0f, 1e-2f);
    EXPECT_TRUE(fp16::to_f32(out.s[at(out, 2, 2, 2) * spade::kFlowHalvesPerCell + (spade::kFlowQHat % spade::kFlowHalvesPerCell)]) != fp16::to_f32(out.s[at(out, 2, 2, 2) * spade::kFlowHalvesPerCell + (spade::kFlowQHat % spade::kFlowHalvesPerCell)]))
        << "a NaN: no-data in any frame is no-data in the mean";
    EXPECT_FLOAT_EQ(l2(out, 0, 0, 0), -40.0f) << "lambda2-hat is the drawn frame's";
    EXPECT_FLOAT_EQ(u_of(out, 3, 3, 3), 5.0f) << "the velocity is the drawn frame's";
    EXPECT_NEAR(out.header.channel_max[spade::kFlowQHat], 6.0f, 1e-2f);
    // refused by name: no frames, another shape, a velocity channel
    EXPECT_FALSE(flow_movie::mean_scalars(out, std::vector<const FlowVolume*>{}, channels).has_value());
    FlowVolume other = blank(3, 4, 4);
    EXPECT_FALSE(flow_movie::mean_scalars(out, std::vector<const FlowVolume*>{&a, &other}, channels).has_value());
    const auto bad = flow_movie::mean_scalars(out, frames, std::vector<uint32_t>{spade::kFlowU});
    ASSERT_FALSE(bad.has_value());
    EXPECT_NE(bad.error().context.find("channel"), std::string::npos) << bad.error().context;
}

// The caption's words for the tubes (O163, Oversight's terms): what they are, what colours them, what is fixed, what is smoothed and that it is for display only, and that they are the simulated
// air and not a measurement. The numbers come from the look, so a changed look changes the words.
TEST(FlowMovieQLook, TheTubeNoteSaysWhatTheTubesAreHowTheyAreColouredAndHowTheyAreKeptSteady) {
    flow_movie::QLook look;
    look.q_peak = 20.0f;
    const std::string note = flow_movie::tube_note(look, 0.8f, 3u, true);
    for (const char* want : {"smooth surfaces", "vortices", "pressure", "same scale as the parts", "0.8", "fixed", "every frame", "one cell", "3 frames", "display only", "not a measurement"}) {
        EXPECT_NE(note.find(want), std::string::npos) << want << ": " << note;
    }
    EXPECT_EQ(note.find("soft cloud"), std::string::npos) << "these are surfaces, not the cloud: " << note;
    EXPECT_EQ(note.find("vorticity"), std::string::npos) << "coloured by pressure, not by vorticity: " << note;
    std::string lower = note;
    for (char& ch : lower) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    EXPECT_EQ(lower.find("nasa"), std::string::npos) << note;
    const std::string wider = flow_movie::tube_note(look, 0.6f, 5u, false);
    EXPECT_NE(wider.find("0.6"), std::string::npos) << wider;
    EXPECT_NE(wider.find("5 frames"), std::string::npos) << wider;
    EXPECT_EQ(wider.find("one cell"), std::string::npos) << "no spatial smoothing asked: none said: " << wider;
}

// ---- O183 (B3): do the tubes reach the box's floor? A tube cut flat by the open floor face is an artifact the wake fade cannot hide (a surface cannot fade). The check reads the solve's own frame:
// whether any cell of the floor layer(s) has q_hat at or above the tubes' LOWEST level, and the height above the floor of the lowest such cell anywhere.
TEST(FlowMovieQLook, ATubeThatReachesTheFloorFailsTheCheckAndOneThatStopsShortPasses) {
    flow_movie::QLook look;
    look.q_peak = 10.0f;  // levels 50%, 25%, 10% of it: the lowest tube is q_hat = 1
    const auto column = [&](uint32_t j_first, float value) {
        FlowVolume v = blank(8, 10, 8);  // 10 cells up, 0.004 m each (blank's cell)
        for (uint32_t j = j_first; j < 10; ++j) set_q(v, 4, j, 4, value);
        return v;
    };
    const auto reaches = flow_movie::tube_floor_check(column(0, 3.0f), look, 1u);
    EXPECT_TRUE(reaches.any_tube);
    EXPECT_TRUE(reaches.reaches_floor) << "a tube down to the floor layer is cut by the face";
    EXPECT_NEAR(reaches.lowest_height_m, 0.5f * 0.004f, 1e-6f) << "the lowest tube cell's centre above the floor";
    const auto short_of = flow_movie::tube_floor_check(column(3, 3.0f), look, 1u);
    EXPECT_TRUE(short_of.any_tube);
    EXPECT_FALSE(short_of.reaches_floor);
    EXPECT_NEAR(short_of.lowest_height_m, 3.5f * 0.004f, 1e-6f);
    // two floor layers: a tube that stops in the second layer fails a two-layer check, passes a one-layer one
    const auto layer2 = column(1, 3.0f);
    EXPECT_FALSE(flow_movie::tube_floor_check(layer2, look, 1u).reaches_floor);
    EXPECT_TRUE(flow_movie::tube_floor_check(layer2, look, 2u).reaches_floor);
    // the level is the lowest of the ladder, and it is >=: 0.9 is below it, 1.0 is at it
    EXPECT_FALSE(flow_movie::tube_floor_check(column(0, 0.9f), look, 1u).any_tube);
    EXPECT_TRUE(flow_movie::tube_floor_check(column(0, 1.0f), look, 1u).reaches_floor);
    // a no-data cell is not a tube
    FlowVolume hole = column(0, 3.0f);
    hole.s[at(hole, 4, 0, 4) * spade::kFlowHalvesPerCell + (spade::kFlowQHat % spade::kFlowHalvesPerCell)] = 0x7E00u;
    EXPECT_FALSE(flow_movie::tube_floor_check(hole, look, 1u).reaches_floor);
    EXPECT_NEAR(flow_movie::tube_floor_check(hole, look, 1u).lowest_height_m, 1.5f * 0.004f, 1e-6f);
    // no tube anywhere
    const auto none = flow_movie::tube_floor_check(blank(8, 10, 8), look, 1u);
    EXPECT_FALSE(none.any_tube);
    EXPECT_FALSE(none.reaches_floor);
}

// The per-frame floor report (O183): its form and its number on known cases. A tube stopping 3.5 mm above the floor in a clear bottom, a tube in the floor layers, and no tube at all.
TEST(FlowMovieQLook, TheFloorReportLineNamesTheVerdictAndTheLowestTubeHeightInMillimetres) {
    flow_movie::TubeFloorCheck short_of;
    short_of.any_tube = true;
    short_of.lowest_height_m = 0.0035f;
    EXPECT_EQ(flow_movie::tube_floor_line(short_of), "[ tubes ] floor check: stops short (the lowest tube cell 3.5 mm above the floor, the bottom 2 layers are clear)");
    flow_movie::TubeFloorCheck reaches;
    reaches.any_tube = true;
    reaches.reaches_floor = true;
    reaches.lowest_height_m = 0.000625f;
    EXPECT_EQ(flow_movie::tube_floor_line(reaches), "[ tubes ] floor check: REACHES THE FLOOR (the lowest tube cell 0.6 mm above the floor, the bottom 2 layers hold a tube)");
    EXPECT_EQ(flow_movie::tube_floor_line(flow_movie::TubeFloorCheck{}), "[ tubes ] floor check: no tube (the lowest tube cell 0.0 mm above the floor, the bottom 2 layers are clear)");
}
