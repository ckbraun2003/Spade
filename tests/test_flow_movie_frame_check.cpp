// ---------------------------------------------------------------------------
// test_flow_movie_frame_check.cpp -- tools/flow_movie/frame_check.hpp, the
// host check every film's frames pass before write_frame() caches them (B7's
// G2, then every film; the lead, 2026-10-27: GpuFluid has no NaN scan yet,
// the CPU's D1 with its parts grows a disc-plane face ~4 m/s a step, and a
// bad frame must never become a film).
//
//   FlowMovieFrameCheck  a finite frame passes; a NaN, an infinity and the
//                        publish's no-data half are each caught, the first
//                        named by channel and cell and all of them counted;
//                        the frame's fastest cell by its published u, v and
//                        w, a non-finite cell passed over (the lead's speed
//                        bound, for a flow that blows up and stays finite);
//                        frame_fault(): the bound, ten times the expected
//                        speed and at least ten; a solid interior's no-data
//                        passes and a poisoned fluid cell beside it doesn't;
//                        a frame without an expected speed is refused;
//                        write_frame() caches a frame that passes and refuses
//                        one that doesn't, unwritten, naming its frame and
//                        step.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "../tools/flow_movie/cache.hpp"
#include "../tools/flow_movie/frame_check.hpp"
#include "core/fp16.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "unique_temp.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace fs = std::filesystem;
using flow_movie::FastestCell;
using flow_movie::fastest_cell;
using flow_movie::first_non_finite;
using flow_movie::frame_fault;
using flow_movie::FrameBound;
using flow_movie::NonFinite;
using flow_movie::speed_bound;

// A 3 x 2 x 2 frame of finite halves: zeros of both signs, the largest
// finite half and the smallest subnormal among them.
[[nodiscard]] spade::FlowVolume finite_frame() {
    spade::FlowVolume f;
    f.header.nx = 3;
    f.header.ny = 2;
    f.header.nz = 2;
    const std::size_t halves = 3u * 2u * 2u * spade::kFlowHalvesPerCell;
    f.v.assign(halves, 0x0000u);
    f.s.assign(halves, 0x8000u);
    f.v[5] = 0x7BFFu;   // 65504
    f.s[9] = 0xFBFFu;   // -65504
    f.s[10] = 0x0001u;  // a subnormal
    return f;
}

// Cell (i, j, k)'s half of `channel` in its texture.
[[nodiscard]] std::size_t at(uint32_t i, uint32_t j, uint32_t k, uint32_t channel) {
    return ((k * 2u + j) * 3u + i) * spade::kFlowHalvesPerCell + channel % spade::kFlowHalvesPerCell;
}

TEST(FlowMovieFrameCheck, AFiniteFramePasses) {
    EXPECT_FALSE(first_non_finite(finite_frame()).has_value());
}

TEST(FlowMovieFrameCheck, NamesTheFirstAndCountsThemAll) {
    spade::FlowVolume f = finite_frame();
    f.s[at(1, 1, 0, spade::kFlowPressure)] = spade::fp16::kNoDataBits;  // the publish's NaN
    f.v[at(2, 0, 1, spade::kFlowW)] = 0x7C00u;                          // +infinity
    f.s[at(0, 1, 1, spade::kFlowLambda2Hat)] = 0xFE01u;                 // a negative NaN
    const std::optional<NonFinite> bad = first_non_finite(f);
    ASSERT_TRUE(bad.has_value());
    EXPECT_EQ(bad->channel, static_cast<uint32_t>(spade::kFlowW)) << "V's halves are scanned before S's";
    EXPECT_EQ(bad->i, 2u);
    EXPECT_EQ(bad->j, 0u);
    EXPECT_EQ(bad->k, 1u);
    EXPECT_EQ(bad->bits, 0x7C00u);
    EXPECT_EQ(bad->count, 3u);
}

TEST(FlowMovieFrameCheck, TheNoDataHalfAloneIsCaught) {
    spade::FlowVolume f = finite_frame();
    f.s[at(2, 1, 1, spade::kFlowVorticity)] = spade::fp16::kNoDataBits;
    const std::optional<NonFinite> bad = first_non_finite(f);
    ASSERT_TRUE(bad.has_value());
    EXPECT_EQ(bad->channel, static_cast<uint32_t>(spade::kFlowVorticity));
    EXPECT_EQ(bad->i, 2u);
    EXPECT_EQ(bad->j, 1u);
    EXPECT_EQ(bad->k, 1u);
    EXPECT_EQ(bad->count, 1u);
}

TEST(FlowMovieFrameCheck, FindsTheFastestCell) {
    spade::FlowVolume f;
    f.header.nx = 3;
    f.header.ny = 2;
    f.header.nz = 2;
    const std::size_t halves = 3u * 2u * 2u * spade::kFlowHalvesPerCell;
    f.v.assign(halves, 0x0000u);
    f.s.assign(halves, 0x0000u);
    const auto put = [&f](uint32_t i, uint32_t j, uint32_t k, uint32_t channel, float x) {
        f.v[at(i, j, k, channel)] = spade::fp16::from_f32(x).bits;
    };
    EXPECT_EQ(fastest_cell(f).speed, 0.0f) << "still air";
    put(2, 1, 0, spade::kFlowU, 3.0f);
    put(2, 1, 0, spade::kFlowV, -4.0f);    // 5 m/s
    put(0, 0, 1, spade::kFlowW, -4.5f);    // 4.5 m/s
    put(1, 0, 1, spade::kFlowDye, 99.0f);  // dye is no speed
    f.v[at(1, 1, 1, spade::kFlowU)] = spade::fp16::kNoDataBits;  // first_non_finite()'s
    const FastestCell fast = fastest_cell(f);
    EXPECT_EQ(fast.speed, 5.0f);
    EXPECT_EQ(fast.i, 2u);
    EXPECT_EQ(fast.j, 1u);
    EXPECT_EQ(fast.k, 0u);
}

TEST(FlowMovieFrameCheck, TheBoundIsTenTimesTheExpectedSpeedAndAtLeastTen) {
    EXPECT_EQ(speed_bound(FrameBound{.expected_speed = 3.0f}), 30.0f);
    EXPECT_EQ(speed_bound(FrameBound{.expected_speed = 0.5f}), 10.0f);
}

// A cell at 100 m/s is past a 5 m/s case's bound, 50, and within a 10 m/s
// case's, 100.
TEST(FlowMovieFrameCheck, ASpeedPastTheBoundIsAFault) {
    spade::FlowVolume f;
    f.header.nx = 3;
    f.header.ny = 2;
    f.header.nz = 2;
    const std::size_t halves = 3u * 2u * 2u * spade::kFlowHalvesPerCell;
    f.v.assign(halves, 0x0000u);
    f.s.assign(halves, 0x0000u);
    f.v[at(2, 1, 0, spade::kFlowU)] = spade::fp16::from_f32(100.0f).bits;
    EXPECT_EQ(frame_fault(f, FrameBound{.expected_speed = 10.0f}), "");
    EXPECT_EQ(frame_fault(f, FrameBound{.expected_speed = 5.0f}),
              "max |v| 100.0 at cell (2, 1, 0), past the bound 50.0 (10 x the case's expected top speed, 5.00): "
              "the flow blew up");
}

TEST(FlowMovieFrameCheck, ANonFiniteHalfIsAFaultNamedWithoutParts) {
    spade::FlowVolume f = finite_frame();
    f.v[5] = 0x0000u;  // finite_frame()'s 65504, no speed for this bound
    f.s[at(2, 1, 1, spade::kFlowVorticity)] = spade::fp16::kNoDataBits;
    EXPECT_EQ(frame_fault(f, FrameBound{.expected_speed = 5.0f}),
              "1 non-finite sample, the first |w| at cell (2, 1, 1), half 0x7e00: the fluid went NaN, infinite or "
              "past binary16's range");
}

TEST(FlowMovieFrameCheck, AFrameWithoutAnExpectedSpeedIsRefused) {
    EXPECT_EQ(frame_fault(finite_frame(), FrameBound{}), "no expected top speed to bound its cells by");
}

// A 6^3 grid in still air published with a 4^3 solid block at its centre:
// the block's 2^3 interior, past its first layer, is no data in all eight
// channels (publish.hpp), 64 halves. A frame of a run with those 64 solid
// cells may hold them; one with none may not; a NaN in a fluid face beside
// the block is a fault either way.
[[nodiscard]] spade::FlowVolume block_frame(bool poisoned, std::vector<uint8_t>& solid) {
    const af::GridShape shape{6, 6, 6, 0.1f};
    spade::Result<af::MacGrid> g = af::make_grid(shape, af::DomainBc{});
    EXPECT_TRUE(g.has_value());
    solid.assign(6u * 6u * 6u, 0u);
    for (uint32_t k = 1; k < 5; ++k) {
        for (uint32_t j = 1; j < 5; ++j) {
            for (uint32_t i = 1; i < 5; ++i) solid[i + 6u * (j + 6u * k)] = 1u;
        }
    }
    if (poisoned) g->u[af::u_index(shape, 1, 0, 0)] = std::numeric_limits<float>::quiet_NaN();
    spade::FlowVolume f;
    af::PublishInput in;
    in.solid = solid;
    EXPECT_TRUE(af::publish_render_volume(*g, in, f).has_value());
    return f;
}

TEST(FlowMovieFrameCheck, ASolidInteriorsNoDataPassesAndAPoisonedCellBesideItDoesNot) {
    std::vector<uint8_t> solid;
    const spade::FlowVolume clean = block_frame(false, solid);
    ASSERT_EQ(clean.header.extrapolated_cells, 56u) << "the block's first layer";
    const std::optional<NonFinite> interior = first_non_finite(clean);
    ASSERT_TRUE(interior.has_value());
    EXPECT_EQ(interior->count, 64u) << "eight halves in each of the 8 interior cells";
    EXPECT_EQ(frame_fault(clean, FrameBound{.expected_speed = 1.0f, .solid_cells = 64}), "");
    EXPECT_NE(frame_fault(clean, FrameBound{.expected_speed = 1.0f}), "") << "a run without parts holds none";
    const spade::FlowVolume poisoned = block_frame(true, solid);
    const std::string why = frame_fault(poisoned, FrameBound{.expected_speed = 1.0f, .solid_cells = 64});
    EXPECT_NE(why.find(" past the 64 its parts' interior publishes as no data: the fluid went NaN"),
              std::string::npos)
        << why;
}

// A film's first frame, still air, passes and is cached; its second, a cell
// at 100 m/s in a 5 m/s case, is refused before anything is written, naming
// its frame (counted from 1), the step it was published after and why.
TEST(FlowMovieFrameCheck, WriteFrameCachesAPassingFrameAndRefusesAFaultUnwritten) {
    spade::FlowVolume f;
    f.header.nx = 3;
    f.header.ny = 2;
    f.header.nz = 2;
    const std::size_t halves = 3u * 2u * 2u * spade::kFlowHalvesPerCell;
    f.v.assign(halves, 0x0000u);
    f.s.assign(halves, 0x0000u);
    const spade::test::TempDir dir("frame_check_cache");
    const FrameBound bound{.expected_speed = 5.0f};
    const flow_movie::FrameLog first{.step = 120, .time = 0.25, .live = {}};
    ASSERT_TRUE(flow_movie::write_frame(dir.path(), 0, f, first, bound).has_value());
    f.v[at(2, 1, 0, spade::kFlowU)] = spade::fp16::from_f32(100.0f).bits;
    const flow_movie::FrameLog second{.step = 240, .time = 0.5, .live = {}};
    const spade::Result<void> refused = flow_movie::write_frame(dir.path(), 1, f, second, bound);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::internal);
    EXPECT_EQ(refused.error().context,
              "frame 2, after step 240 (t 0.500000): max |v| 100.0 at cell (2, 1, 0), past the bound 50.0 (10 x the "
              "case's expected top speed, 5.00): the flow blew up; the frame is not cached and the film stops");
    EXPECT_EQ(flow_movie::cached_frames(dir.path()), 1u);
    EXPECT_FALSE(fs::exists(dir.path() / "frame-00001.bin"));
    EXPECT_FALSE(fs::exists(dir.path() / "frame.tmp")) << "nothing begun";
}

}  // namespace
