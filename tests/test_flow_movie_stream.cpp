// flow_movie's streaming (tools/flow_movie/stream_plan.hpp; the lead's L86b, disk plan): a website master draws each frame as
// the solve writes it and deletes what the clip no longer needs, so a 600-frame solve of the deep box (253 MB a frame) never
// holds more than a few frames on disk. The plan is pure and decides three things, here pinned without a GPU:
//
//   which frames the solve writes at all     (StreamPlan::needed)   -- a frame no draw reads is never written;
//   when a written frame may be deleted      (StreamPlan::spent)    -- once every draw that reads it is done;
//   which frames stay for good               (--keep-frames)        -- the still pairs' frames 150 and 300.
//
// A draw at frame d reads the frames d - M + 1 .. d (M: --mean-frames, 1 the frame alone; the start clips the window at 0).

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "../tools/flow_movie/stream_plan.hpp"

namespace {

[[nodiscard]] std::vector<uint32_t> range(uint32_t first, uint32_t count) {
    std::vector<uint32_t> v;
    for (uint32_t i = 0; i < count; ++i) v.push_back(first + i);
    return v;
}

}  // namespace

TEST(FlowMovieStream, ASingleFrameDrawWritesOnlyTheFramesTheClipDraws) {
    const flow_movie::StreamPlan plan(range(150, 300), 1u, {});  // S8: frames 150 to 449
    EXPECT_FALSE(plan.needed(0u));
    EXPECT_FALSE(plan.needed(149u)) << "the start-up the clip does not show is never written";
    EXPECT_TRUE(plan.needed(150u));
    EXPECT_TRUE(plan.needed(449u));
    EXPECT_FALSE(plan.needed(450u));
    EXPECT_EQ(plan.last_frame(), 449u) << "the solve stops at the clip's last frame";
    // A frame is spent as soon as its own draw is done, and not before.
    EXPECT_FALSE(plan.spent(150u, 149u));
    EXPECT_TRUE(plan.spent(150u, 150u));
    EXPECT_TRUE(plan.spent(449u, 449u));
}

TEST(FlowMovieStream, TheMeanWindowHoldsTheFramesBeforeTheFirstDrawUntilTheLastDrawThatReadsThem) {
    const flow_movie::StreamPlan plan(range(150, 300), 10u, {});
    EXPECT_FALSE(plan.needed(140u));
    EXPECT_TRUE(plan.needed(141u)) << "the first draw's window starts 9 frames back";
    EXPECT_FALSE(plan.spent(141u, 149u));
    EXPECT_TRUE(plan.spent(141u, 150u)) << "frame 141 is read by the draw at 150 and no later one";
    EXPECT_FALSE(plan.spent(200u, 208u));
    EXPECT_FALSE(plan.spent(200u, 209u - 1u));
    EXPECT_TRUE(plan.spent(200u, 209u)) << "frame 200 is read by the draws at 200 .. 209";
    EXPECT_TRUE(plan.spent(449u, 449u)) << "the clip's end cuts the last windows short";
    EXPECT_FALSE(plan.spent(445u, 448u));
    EXPECT_TRUE(plan.spent(445u, 449u));
}

TEST(FlowMovieStream, TheWindowStopsAtFrameZero) {
    const flow_movie::StreamPlan plan(range(0, 360), 10u, {});  // S2 from rest
    EXPECT_TRUE(plan.needed(0u));
    EXPECT_FALSE(plan.spent(0u, 8u)) << "frame 0 is read by the draws at 0 .. 9";
    EXPECT_TRUE(plan.spent(0u, 9u));
}

TEST(FlowMovieStream, AKeptFrameIsWrittenAndNeverSpentEvenWhenNoDrawReadsIt) {
    const flow_movie::StreamPlan plan(range(0, 360), 1u, {150u, 300u, 500u});
    EXPECT_TRUE(plan.kept(150u));
    EXPECT_FALSE(plan.spent(150u, 449u));
    EXPECT_FALSE(plan.spent(300u, 359u));
    EXPECT_TRUE(plan.spent(151u, 151u));
    EXPECT_TRUE(plan.needed(500u)) << "kept: written although the clip ends at 359";
    EXPECT_EQ(plan.last_frame(), 500u) << "the solve runs on to the last frame it must keep";
}

TEST(FlowMovieStream, AStillOnlyRunDrawsTwoFramesAndKeepsThem) {
    // The still pairs: frames 150 and 300 of a solve, nothing else.
    const flow_movie::StreamPlan plan({150u, 300u}, 1u, {150u, 300u});
    EXPECT_TRUE(plan.needed(150u));
    EXPECT_TRUE(plan.needed(300u));
    EXPECT_FALSE(plan.needed(0u));
    EXPECT_FALSE(plan.needed(151u));
    EXPECT_FALSE(plan.needed(299u));
    EXPECT_FALSE(plan.spent(150u, 300u));
    EXPECT_EQ(plan.last_frame(), 300u);
    for (uint32_t j = 0; j < 400u; ++j) {
        if (j != 150u && j != 300u) EXPECT_FALSE(plan.needed(j)) << j;
    }
}

TEST(FlowMovieStream, DrawsThatAreNotConsecutiveKeepEachOnesOwnWindow) {
    const flow_movie::StreamPlan plan({150u, 300u}, 3u, {});
    EXPECT_TRUE(plan.needed(148u));
    EXPECT_FALSE(plan.needed(147u));
    EXPECT_FALSE(plan.needed(151u));
    EXPECT_FALSE(plan.needed(297u));
    EXPECT_TRUE(plan.needed(298u));
    EXPECT_TRUE(plan.spent(148u, 150u));
    EXPECT_FALSE(plan.spent(298u, 299u));
    EXPECT_TRUE(plan.spent(298u, 300u));
}

TEST(FlowMovieStream, ThePeakOnDiskIsTheWindowAndTheKeptFramesNotTheCache) {
    // Walk a 600-frame solve the way the run does: write what is needed, draw, delete what is spent. The most frames held at
    // once is the mean window plus the kept ones: a handful, not 600.
    for (const uint32_t mean : {1u, 10u}) {
        const flow_movie::StreamPlan plan(range(150, 450), mean, {150u, 300u});
        std::vector<uint32_t> on_disk;
        std::size_t peak = 0;
        for (uint32_t k = 0; k <= plan.last_frame(); ++k) {
            if (plan.needed(k)) on_disk.push_back(k);
            peak = std::max(peak, on_disk.size());
            std::erase_if(on_disk, [&](uint32_t j) { return plan.spent(j, k); });
        }
        EXPECT_LE(peak, std::size_t{mean} + 2u) << "mean " << mean;
        EXPECT_EQ(on_disk, (std::vector<uint32_t>{150u, 300u})) << "only the kept frames remain, mean " << mean;
    }
}

TEST(FlowMovieStream, TheKeepListIsParsedStrictly) {
    const auto some = flow_movie::parse_frame_list("300,150,150");
    ASSERT_TRUE(some.has_value());
    EXPECT_EQ(*some, (std::vector<uint32_t>{150u, 300u})) << "sorted, each once";
    EXPECT_EQ(*flow_movie::parse_frame_list("7"), (std::vector<uint32_t>{7u}));
    for (const char* bad : {"", ",", "1,", ",1", "1,,2", "1, 2", " 1", "-1", "+1", "1.5", "1e2", "x", "1;2", "99999999999"}) {
        EXPECT_FALSE(flow_movie::parse_frame_list(bad).has_value()) << "'" << bad << "'";
    }
}

TEST(FlowMovieStream, ThePictureSizeIsParsedStrictly) {
    const auto hd = flow_movie::parse_size("1920x1080");
    ASSERT_TRUE(hd.has_value());
    EXPECT_EQ(hd->first, 1920u);
    EXPECT_EQ(hd->second, 1080u);
    for (const char* bad : {"", "1920", "1920x", "x1080", "1920X1080", "1920x1080x2", "-1920x1080", "19.2x1080", "0x0",
                            "16x16", "99999x99999", " 1920x1080", "1920 x1080"}) {
        EXPECT_FALSE(flow_movie::parse_size(bad).has_value()) << "'" << bad << "'";
    }
}

TEST(FlowMovieStream, ARequestThatCannotStreamIsRefusedByName) {
    flow_movie::StreamAsk ask;
    EXPECT_EQ(flow_movie::stream_refusal(ask), "") << "no --stream, nothing to refuse";
    ask.stream = true;
    ask.d1 = true;
    EXPECT_EQ(flow_movie::stream_refusal(ask), "");
    ask.keep_given = true;
    EXPECT_EQ(flow_movie::stream_refusal(ask), "") << "--keep-frames with --stream";
    flow_movie::StreamAsk keep_alone;
    keep_alone.keep_given = true;
    EXPECT_NE(flow_movie::stream_refusal(keep_alone).find("--keep-frames needs --stream"), std::string::npos);

    for (int which = 0; which < 5; ++which) {
        flow_movie::StreamAsk bad;
        bad.stream = true;
        bad.d1 = true;
        std::string must_name;
        switch (which) {
            case 0: bad.d1 = false; must_name = "D1 clips"; break;
            case 1: bad.deliver = true; must_name = "--web-deliver"; break;
            case 2: bad.export_lines = true; must_name = "--export-lines"; break;
            case 3: bad.simulate_only = true; must_name = "--simulate-only"; break;
            default: bad.frames_on_disk = true; must_name = "--resimulate"; break;
        }
        const std::string why = flow_movie::stream_refusal(bad);
        EXPECT_NE(why.find(must_name), std::string::npos) << which << ": " << why;
    }
    flow_movie::StreamAsk again;
    again.stream = true;
    again.d1 = true;
    again.frames_on_disk = true;
    again.resimulate = true;
    EXPECT_EQ(flow_movie::stream_refusal(again), "") << "--resimulate clears the folder, so frames on disk are no obstacle";
}
