// ---------------------------------------------------------------------------
// test_flow_movie_web.cpp -- tools/flow_movie/web_masters.hpp, the pure parts of
// the website clips (Cameron's render brief, relayed by the lead 2026-11-08):
//
//   FlowMovieWeb  the camera maths (a loop orbit repeats no frame, the fit leaves
//                 the margin, a push-in eases, a slice window fills the frame);
//                 the window's frame-to-step map; the table of the eight clips
//                 (names, frames, one colour range within a scene); the encode's
//                 arguments (libx264 CRF 15, yuv420p, faststart, no audio, no
//                 captions); the file name and MANIFEST line; the delivery guard
//                 (only raw/spade and MANIFEST.md); the caption's standing
//                 caveats; and a blackened scene drawing pure black on the CPU
//                 renderer, the GL views' reference.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <numbers>
#include <string>
#include <vector>

#include "../tools/flow_movie/web_masters.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace {

namespace web = flow_movie::web;
namespace fs = std::filesystem;

constexpr float kFov = 1.0471975511965976f;  // render::Camera's default, 60 degrees
constexpr float kAspect = 16.0f / 9.0f;

[[nodiscard]] bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

// The fraction of the half-frame a sphere of `radius` at `distance` covers along an axis whose half-angle has
// tangent `tan_half`.
[[nodiscard]] double covered(double radius, double distance, double tan_half) {
    return (radius / std::sqrt(distance * distance - radius * radius)) / tan_half;
}

TEST(FlowMovieWeb, ALoopOrbitRepeatsNoFrameAndTheNextFrameClosesIt) {
    const uint32_t n = 450;
    EXPECT_FLOAT_EQ(web::loop_azimuth(30.0f, 360.0f, 0, n), 30.0f);
    EXPECT_NEAR(web::loop_azimuth(30.0f, 360.0f, n, n), 30.0f + 360.0f, 1e-3f);  // frame n would be frame 0
    for (uint32_t k = 1; k < n; ++k) {
        const float step = web::loop_azimuth(0.0f, 360.0f, k, n) - web::loop_azimuth(0.0f, 360.0f, k - 1, n);
        ASSERT_NEAR(step, 360.0f / static_cast<float>(n), 1e-3f) << "frame " << k;
        ASSERT_LT(web::loop_azimuth(0.0f, 360.0f, k, n), 360.0f) << "frame " << k << " must not close the loop";
    }
}

TEST(FlowMovieWeb, TheFitLeavesTheMarginOnTheLimitingAxisAndMoreOnTheOther) {
    const float radius = 0.6f;
    const double tan_y = std::tan(0.5 * kFov);
    {
        const float d = web::fit_distance(radius, kFov, kAspect, web::kMargin);
        EXPECT_NEAR(covered(radius, d, tan_y), 1.0 - 2.0 * web::kMargin, 1e-4);  // landscape: height limits
        EXPECT_LT(covered(radius, d, tan_y * kAspect), 1.0 - 2.0 * web::kMargin);
    }
    {
        const float d = web::fit_distance(radius, kFov, 9.0f / 16.0f, web::kMargin);  // portrait: width limits
        EXPECT_NEAR(covered(radius, d, tan_y * 9.0 / 16.0), 1.0 - 2.0 * web::kMargin, 1e-4);
        EXPECT_LT(covered(radius, d, tan_y), 1.0 - 2.0 * web::kMargin);
    }
}

TEST(FlowMovieWeb, AnObjectFillsItsFractionOfTheFrameHeight) {
    const float d = web::fill_distance(0.37f, 0.6f, kFov);
    const double visible = 2.0 * d * std::tan(0.5 * kFov);
    EXPECT_NEAR(0.37 / visible, 0.6, 1e-5);
}

TEST(FlowMovieWeb, ASliceWindowIsTheFramesShapeAndFillsItEdgeToEdge) {
    EXPECT_NEAR(web::plane_window_height(0.8f, kAspect), 0.45f, 1e-6f);
    const float h = web::plane_window_height(0.8f, kAspect);
    const float d = web::fit_plane_distance(h, kFov);
    EXPECT_NEAR(2.0 * d * std::tan(0.5 * kFov), h, 1e-5);
}

TEST(FlowMovieWeb, APushInStartsWideEndsCloseAndEasesAtBothEnds) {
    const uint32_t n = 360;
    EXPECT_FLOAT_EQ(web::push_in_distance(2.0f, 0.5f, 0, n), 2.0f);
    EXPECT_NEAR(web::push_in_distance(2.0f, 0.5f, n - 1, n), 0.5f, 1e-5f);
    float last = 3.0f;
    for (uint32_t k = 0; k < n; ++k) {
        const float d = web::push_in_distance(2.0f, 0.5f, k, n);
        ASSERT_LE(d, last) << "frame " << k;
        last = d;
    }
    const float linear = 1.5f / static_cast<float>(n - 1);
    EXPECT_LT(web::push_in_distance(2.0f, 0.5f, 0, n) - web::push_in_distance(2.0f, 0.5f, 1, n), linear);
    EXPECT_LT(web::push_in_distance(2.0f, 0.5f, n - 2, n) - web::push_in_distance(2.0f, 0.5f, n - 1, n), linear);
}

TEST(FlowMovieWeb, TheWindowsFrameToStepMapIsMonotoneAndEndsOnTheWindowsEnd) {
    const uint32_t n = 450;
    const double h = 0.0005;
    uint64_t last = 0;
    for (uint32_t k = 0; k < n; ++k) {
        const uint64_t s = web::frame_end_step(k, n, 0.9, 1.5, h);
        ASSERT_GE(s, last) << "frame " << k;
        ASSERT_GT(s, uint64_t{1800}) << "frame " << k << " is inside the window, after t = 0.9 s";
        last = s;
    }
    EXPECT_EQ(web::frame_end_step(n - 1, n, 0.9, 1.5, h), uint64_t{3000});
    EXPECT_EQ(web::frame_end_step(0, 1, 0.0, 0.6, 0.0005), uint64_t{1200});
}

TEST(FlowMovieWeb, TheTableHasTheEightClipsAtThirtyFramesASecondAndOneRangeAScene) {
    const auto& clips = web::web_clips();
    ASSERT_EQ(clips.size(), std::size_t{8});
    const char* ids[] = {"S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8"};
    const uint32_t frames[] = {450, 360, 360, 300, 300, 300, 300, 300};
    for (std::size_t i = 0; i < clips.size(); ++i) {
        EXPECT_EQ(std::string(clips[i].id), ids[i]);
        EXPECT_EQ(clips[i].frames(), frames[i]) << ids[i];
        EXPECT_FALSE(std::string(clips[i].short_name).empty()) << ids[i];
    }
    for (const web::Clip& a : clips) {
        for (const web::Clip& b : clips) {
            if (a.scene == b.scene && a.quantity == b.quantity) {
                EXPECT_EQ(a.range_lo, b.range_lo) << a.id << " and " << b.id << " share a colour range";
                EXPECT_EQ(a.range_hi, b.range_hi) << a.id << " and " << b.id << " share a colour range";
            }
        }
    }
    const char* with_still[] = {"S1", "S3", "S4", "S7"};
    for (const web::Clip& c : clips) {
        bool want = false;
        for (const char* s : with_still) want = want || std::string(c.id) == s;
        EXPECT_EQ(c.still, want) << c.id;
    }
    EXPECT_NE(web::find_clip("S4"), nullptr);
    EXPECT_EQ(web::find_clip("S9"), nullptr);
}

TEST(FlowMovieWeb, TheEncodeIsOneLibx264PassAtCrf15WithNoCaptionsAndNoAudio) {
    const std::vector<std::string> args = web::ffmpeg_web_args("ffmpeg", web::kWidth, web::kHeight, "out.mp4");
    const auto at = [&](const std::string& flag) -> std::string {
        for (std::size_t i = 0; i + 1 < args.size(); ++i) {
            if (args[i] == flag) return args[i + 1];
        }
        return "";
    };
    EXPECT_EQ(args.front(), "ffmpeg");
    EXPECT_EQ(args.back(), "out.mp4");
    EXPECT_EQ(at("-c:v"), "libx264");
    EXPECT_EQ(at("-crf"), "15");
    bool yuv = false;
    for (std::size_t i = 0; i + 1 < args.size(); ++i) yuv = yuv || (args[i] == "-pix_fmt" && args[i + 1] == "yuv420p");
    EXPECT_TRUE(yuv);
    EXPECT_EQ(at("-movflags"), "+faststart");
    EXPECT_EQ(at("-s"), "3840x2160");
    EXPECT_EQ(at("-r"), "30");
    bool no_audio = false;
    for (const std::string& a : args) {
        no_audio = no_audio || a == "-an";
        EXPECT_NE(a, "-vf") << "no caption filter";
    }
    EXPECT_TRUE(no_audio);
}

TEST(FlowMovieWeb, TheFileNameAndTheManifestLineAreTheBriefs) {
    EXPECT_EQ(web::web_file_name("S1", "rotor-wake-streamlines", "sim", "mp4"), "S1_rotor-wake-streamlines_sim.mp4");
    EXPECT_EQ(web::manifest_line("S1_rotor-wake-streamlines_sim.mp4", 3840, 2160, 15.0, "sim", "streamlines of a rotor wake"),
              "- S1_rotor-wake-streamlines_sim.mp4 | 3840x2160 | 15 | sim | streamlines of a rotor wake");
}

TEST(FlowMovieWeb, OnlyRawSpadeAndTheManifestMayBeWritten) {
    const fs::path raw = fs::path("C:/site/media/raw");
    const web::DeliveryGuard guard{raw};
    EXPECT_TRUE(guard.may_write(raw / "spade" / "S1_x_sim.mp4"));
    EXPECT_TRUE(guard.may_write(raw / "spade" / "S1_x_sim.md"));
    EXPECT_TRUE(guard.may_write(raw / "MANIFEST.md"));
    EXPECT_FALSE(guard.may_write(raw / "OTHER.md"));
    EXPECT_FALSE(guard.may_write(raw / "kat" / "K1.mp4"));
    EXPECT_FALSE(guard.may_write(raw / "spade" / ".." / "kat" / "K1.mp4"));
    EXPECT_FALSE(guard.may_write(raw / ".." / "page.html"));
    EXPECT_FALSE(guard.may_write(fs::path("C:/site/index.html")));
    EXPECT_FALSE(guard.may_write(raw / "spade"));  // the folder itself is not a file
}

TEST(FlowMovieWeb, TheCaptionNamesWhatItMustAndClaimsNothingElse) {
    const web::Clip* s3 = web::find_clip("S3");
    ASSERT_NE(s3, nullptr);
    web::CaptionInput in;
    in.clip = s3;
    in.commit = "abc123def456";
    in.date_utc = "2026-11-09 03:00 UTC";
    in.command = "flow_movie --web S3";
    in.ticket = "web-spade-S3";
    in.sim_seconds = 0.567;
    in.screen_seconds = 12.0;
    const std::string text = web::caption_text(in);
    EXPECT_TRUE(has(text, "abc123def456"));
    EXPECT_TRUE(has(text, "2026-11-09 03:00 UTC"));
    EXPECT_TRUE(has(text, "web-spade-S3"));
    EXPECT_TRUE(has(text, "flow_movie --web S3"));
    EXPECT_TRUE(has(text, "Slow motion"));
    EXPECT_TRUE(has(text, "times slower than real time"));
    EXPECT_TRUE(has(text, "the camera loops; the flow does not"));
    EXPECT_TRUE(has(text, "Colour shows relative speed, not validated speed"));
    EXPECT_TRUE(has(text, "crosswind setting supported but not validated"));
    EXPECT_TRUE(has(text, "about 0.18 s of simulated time only"));
    EXPECT_TRUE(has(text, "a single run on the device of record"));
    EXPECT_FALSE(has(text, "identical to the CPU"));
    const std::string standing = web::standing_caveats(*s3);
    EXPECT_FALSE(has(standing, "m/s")) << "no speed figures in the standing caveats";
    EXPECT_TRUE(has(web::standing_caveats(*web::find_clip("S2")), "inviscid"));
    EXPECT_TRUE(has(web::standing_caveats(*web::find_clip("S1")), "four actuator discs"));
    EXPECT_EQ(web::slow_motion_line(0.6, 12.0),
              "Slow motion: 0.0500 simulated seconds in each screen second (20.0 times slower than real time).");
}

TEST(FlowMovieWeb, ABlackenedSceneDrawsPureBlackOnTheCpuRenderer) {
    namespace render = spade::render;
    render::RenderScene scene;
    web::blacken(scene);
    EXPECT_EQ(scene.lighting.sky_zenith, glm::vec3(0.0f));
    EXPECT_EQ(scene.lighting.sky_horizon, glm::vec3(0.0f));
    constexpr uint32_t w = 64, h = 36;
    std::vector<uint8_t> bgrx(static_cast<std::size_t>(w) * h * 4u, 0xFFu);
    render::RenderTarget target{.pixels = std::span<uint8_t>(bgrx), .width = w, .height = h, .stride = w * 4u};
    render::RenderOptions options;
    options.shadows = false;
    options.overlays = false;
    render::Camera camera;
    ASSERT_TRUE(render::render(scene, camera, options, target).has_value());
    std::size_t lit = 0;
    for (std::size_t p = 0; p < bgrx.size(); p += 4u) lit += (bgrx[p] | bgrx[p + 1u] | bgrx[p + 2u]) != 0u ? 1u : 0u;
    EXPECT_EQ(lit, std::size_t{0}) << "every pixel's B, G and R are 0";
}

}  // namespace
