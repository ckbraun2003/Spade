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
//   the perf stopwatch (L74): the median of a run's frames, the `PERF <name> <value> <unit>` line, the lines one web run
//                 prints (the frame's median and each phase's) and the cache's `PERF-NOTE`.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

#include "../tools/flow_movie/look.hpp"
#include "../tools/flow_movie/q_look.hpp"
#include "../tools/flow_movie/sgs_note.hpp"
#include "../tools/flow_movie/web_masters.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "../tools/flow_movie/lines.hpp"

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

// The one jitter the render path owns (the lead, 2026-11-10): frames spread evenly over a window land on
// round(t / h) steps, which for 2.67 steps a frame is 3, 3, 2, so a structure advances 25 % less every third
// frame. In a group every frame is the same whole number of steps after the one before, from the window's start,
// at every step the groups use and for every frame the group holds.
TEST(FlowMovieWeb, EveryFrameOfEveryGroupIsTheSameWholeNumberOfStepsAfterTheOneBefore) {
    for (const web::Group& g : web::web_groups()) {
        const double h = g.step_s;
        const uint64_t from_step = static_cast<uint64_t>(std::llround(g.from_s / h));
        uint64_t last = from_step;
        for (uint32_t k = 0; k < g.frames; ++k) {
            const uint64_t s = web::whole_frame_end_step(k, from_step, g.steps_per_frame);
            ASSERT_EQ(s - last, uint64_t{g.steps_per_frame}) << g.name << " frame " << k;
            last = s;
        }
        // The window the table names ends where its last frame does.
        const double end = web::window_end_s(g.from_s, g.frames, g.steps_per_frame, h);
        EXPECT_EQ(static_cast<uint64_t>(std::llround(end / h)), last) << g.name;
    }
    // The even spread this replaces: 450 frames over 0.9 to 1.5 s at 0.5 ms are 2.67 steps a frame, and the step
    // counts it gives are not all equal. This is the beat the groups no longer have.
    uint64_t lo = ~uint64_t{0}, hi = 0, prev = web::frame_end_step(0, 450, 0.9, 1.5, 0.0005);
    for (uint32_t k = 1; k < 450; ++k) {
        const uint64_t s = web::frame_end_step(k, 450, 0.9, 1.5, 0.0005);
        lo = std::min(lo, s - prev);
        hi = std::max(hi, s - prev);
        prev = s;
    }
    EXPECT_EQ(lo, uint64_t{2});
    EXPECT_EQ(hi, uint64_t{3});
    // The quad's developed wake stays inside the 1.5 s Physics measured: 1800 + 450 * 2 steps is 1.35 s.
    EXPECT_LE(web::window_end_s(0.9, 450, 2, web::kLiveStepS), 1.5);
}

TEST(FlowMovieWeb, TheTableHasTheSevenClipsAtThirtyFramesASecondAndOneRangeAScene) {
    const auto& clips = web::web_clips();
    ASSERT_EQ(clips.size(), std::size_t{7}) << "S7, the Taylor-Green clip, was dropped (L53)";
    const char* ids[] = {"S1", "S2", "S3", "S4", "S5", "S6", "S8"};
    const uint32_t frames[] = {450, 360, 360, 300, 300, 300, 300};
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
    const char* with_still[] = {"S1", "S3", "S4"};
    for (const web::Clip& c : clips) {
        bool want = false;
        for (const char* s : with_still) want = want || std::string(c.id) == s;
        EXPECT_EQ(c.still, want) << c.id;
    }
    EXPECT_NE(web::find_clip("S4"), nullptr);
    EXPECT_EQ(web::find_clip("S7"), nullptr);
    EXPECT_EQ(web::find_clip("S9"), nullptr);
}

TEST(FlowMovieWeb, EveryClipDrawsFromAGroupThatHoldsItsFramesAndTheGroupsAreTheWindows) {
    for (const web::Clip& c : web::web_clips()) {
        const web::Group* g = web::find_group(c.group);
        ASSERT_NE(g, nullptr) << c.id << " names a group that is not in the table";
        EXPECT_GE(g->frames, c.frames()) << c.id << ": its group's cache holds all its frames";
        EXPECT_GT(g->steps_per_frame, 0u) << c.id;
    }
    // One run a group: S5 and S6 share the quad's developed wake; S1, S2 and S8 the D1 still-air run from rest (S2
    // from its frame 0, S1 and S8 from frame 150); S3 and S4 the D1 side-wind run.
    EXPECT_EQ(web::find_clip("S5")->group, web::find_clip("S6")->group);
    EXPECT_EQ(web::find_clip("S1")->group, web::find_clip("S8")->group);
    EXPECT_EQ(web::find_clip("S1")->group, web::find_clip("S2")->group);
    EXPECT_EQ(web::find_clip("S3")->group, web::find_clip("S4")->group);
    EXPECT_NE(web::find_clip("S1")->group, web::find_clip("S3")->group);
    // S2 starts at frame 30 (the lead, 2026-11-17): the first 30 ms show the airframe alone, the wake not yet off the discs.
    EXPECT_EQ(web::find_clip("S2")->first_frame, 30u);
    EXPECT_EQ(web::find_clip("S1")->first_frame, 150u);
    EXPECT_EQ(web::find_clip("S8")->first_frame, 150u);
    // Each clip's frames end inside its group's cache, and S1 and S8 end at its last frame.
    for (const web::Clip& c : web::web_clips()) {
        EXPECT_LE(c.first_frame + c.frames(), web::find_group(c.group)->frames) << c.id;
    }
    EXPECT_EQ(web::find_clip("S1")->first_frame + web::find_clip("S1")->frames(), web::find_group("d1-still")->frames);
    EXPECT_EQ(web::find_group("quad-developed")->frames, 450u);
    EXPECT_DOUBLE_EQ(web::find_group("quad-developed")->from_s, 0.9);
    EXPECT_EQ(web::find_group("d1-crosswind")->steps_per_frame, 7u);
    EXPECT_EQ(web::find_group("d1-still")->steps_per_frame, 5u);
    EXPECT_DOUBLE_EQ(web::find_group("d1-still")->from_s, 0.0) << "from rest: S2's start-up is its frame 0";
    EXPECT_EQ(web::find_group("d1-still")->frames, 600u);
    EXPECT_EQ(web::find_group("nothing"), nullptr);
}

// The hero clips run on D1 at the film's step with the Smagorinsky term (the lead, 2026-11-10, L54): S1 and S8 in
// still air, S2 in the side wind; a frame is about a millisecond of simulated time, as the quad's were.
TEST(FlowMovieWeb, TheHeroClipsRunOnD1AtZeroPointTwoMillisecondsWithCsZeroPointOneSeven) {
    EXPECT_DOUBLE_EQ(web::kD1WebStepS, 0.0002);
    EXPECT_FLOAT_EQ(web::kD1WebSgsCs, 0.17f);
    for (const char* id : {"S1", "S2", "S8"}) EXPECT_TRUE(web::find_clip(id)->scene == web::Scene::d1_still) << id;
    for (const char* id : {"S3", "S4"}) EXPECT_TRUE(web::find_clip(id)->scene == web::Scene::d1) << id;
    for (const char* name : {"d1-still", "d1-crosswind"}) {
        const web::Group& g = *web::find_group(name);
        EXPECT_DOUBLE_EQ(g.step_s, web::kD1WebStepS) << name;
        const double ms_a_frame = static_cast<double>(g.steps_per_frame) * g.step_s * 1000.0;
        EXPECT_GE(ms_a_frame, 1.0) << name;
        EXPECT_LE(ms_a_frame, 1.5) << name;
    }
    EXPECT_DOUBLE_EQ(web::find_group("quad-developed")->step_s, 0.0005) << "the quad's film is as it was";
    // The windows end at 0.60 s (still air) and 0.504 s (side wind).
    const web::Group& still = *web::find_group("d1-still");
    const web::Group& wind = *web::find_group("d1-crosswind");
    EXPECT_NEAR(web::window_end_s(still.from_s, still.frames, still.steps_per_frame, still.step_s), 0.60, 1e-9);
    // S1 and S8 start at 0.15 s of it, S2 at 0, and S2 ends at 0.36 s.
    const double frame_s = static_cast<double>(still.steps_per_frame) * still.step_s;
    EXPECT_NEAR(web::find_clip("S1")->first_frame * frame_s, 0.15, 1e-9);
    EXPECT_NEAR(web::find_clip("S2")->frames() * frame_s, 0.36, 1e-9);
    EXPECT_NEAR(web::window_end_s(wind.from_s, wind.frames, wind.steps_per_frame, wind.step_s), 0.504, 1e-9);
}

// L59: a live film says its subgrid viscosity: LiveRun's line when something bound, else the peak nu_t on its own (a
// Cs 0.17 run where nothing bound would otherwise print nothing), and nothing at Cs 0.
TEST(FlowMovieWeb, ALiveFilmSaysItsPeakEddyViscosityEvenWhenNothingBound) {
    spade::analysis::LiveSgs none;
    EXPECT_EQ(flow_movie::sgs_note(none), "") << "Cs 0: nothing was met";
    spade::analysis::LiveSgs quiet;
    quiet.nu_t_max = 0.0031f;
    const std::string said = flow_movie::sgs_note(quiet);
    EXPECT_NE(said.find("nu_t up to 0.0031 m2/s"), std::string::npos) << said;
    EXPECT_NE(said.find("nothing held at the cap"), std::string::npos) << said;
    spade::analysis::LiveSgs bound = quiet;
    bound.capped_steps = 12;
    bound.capped_cell_steps = 40;
    const std::string line = flow_movie::sgs_note(bound);
    EXPECT_EQ(line, spade::analysis::sgs_health_line(bound)) << "LiveRun's own line when something bound";
    EXPECT_NE(line.find("held at the cap on 12 steps"), std::string::npos) << line;
}

// L60: the D1 cameras aim at the drone's COM, not the box's centre. The old box is symmetric about the drone, so the
// aim is where it was; in the deeper box (the drone 0.354 m below the top face, the floor 1.214 m below it) the box's
// centre is 0.43 m off the drone and the aim must not follow it.
TEST(FlowMovieWeb, TheD1CamerasAimAtTheDronesCentreOfMassNotTheBoxCentre) {
    const auto centre = [](glm::vec3 origin, float cell, uint32_t nx, uint32_t ny, uint32_t nz) {
        return origin + 0.5f * cell * glm::vec3(static_cast<float>(nx), static_cast<float>(ny), static_cast<float>(nz));
    };
    // kD1Grid: 240 x 177 x 168 at 4 mm, the drone 0.354 m above the floor and 0.48 m / 0.336 m from the x / z faces.
    const glm::vec3 old_centre = centre(glm::vec3(-0.48f, -0.354f, -0.336f), 0.004f, 240u, 177u, 168u);
    const glm::vec3 old_aim = web::subject_point(0.08f);
    EXPECT_NEAR(old_aim.x, old_centre.x, 1e-3f) << "the old box's centre was the drone's x";
    EXPECT_NEAR(old_aim.y, old_centre.y - 0.08f, 1e-3f) << "the old aim: the box centre, 0.08 m down";
    EXPECT_NEAR(old_aim.z, old_centre.z, 1e-3f);
    // L60's box: 240 x 392 x 168, the top face 0.354 m above the COM, the floor 1.214 m below it.
    const glm::vec3 deep_centre = centre(glm::vec3(-0.48f, -1.214f, -0.336f), 0.004f, 240u, 392u, 168u);
    EXPECT_NEAR(deep_centre.y, -0.43f, 1e-3f) << "the deeper box's centre is 0.43 m below the drone";
    EXPECT_FLOAT_EQ(web::subject_point(0.08f).y, -0.08f) << "the aim is the same point in either box";
    EXPECT_EQ(web::subject_point(), glm::vec3(0.0f)) << "no offset: the COM itself (S4)";
}

// L63: S1, S8 and S2 look down only 8 degrees from close in, so the deeper box's 1.214 m floor face (open: whatever the
// solve shows there is an artifact) stays below the bottom edge of every frame, with the subject at the 10% margin.
TEST(FlowMovieWeb, TheStillAirFramingKeepsTheFloorFaceBelowEveryFrame) {
    const glm::vec3 aim = web::subject_point(web::kD1StillAimBelowM);
    const auto still = [&](float radius, float azimuth) {
        return web::floor_top_ndc(aim, web::fit_distance(radius, kFov, kAspect, web::kMargin), azimuth,
                                  web::kD1StillElevationDeg, kFov);
    };
    // The subject: a sphere of kD1StillSubjectM at its fitted distance reaches 0.8 of the frame's half-height.
    const float d = web::fit_distance(web::kD1StillSubjectM, kFov, kAspect, web::kMargin);
    EXPECT_NEAR(web::kD1StillSubjectM / std::sqrt(d * d - web::kD1StillSubjectM * web::kD1StillSubjectM) /
                    std::tan(0.5f * kFov),
                1.0f - 2.0f * web::kMargin, 1e-3f);
    // S1: one orbit, the frames' own azimuths (450 frames).
    float worst_s1 = -9.0f;
    for (uint32_t k = 0; k < 450u; ++k) {
        worst_s1 = std::max(worst_s1, still(web::kD1StillSubjectM, web::loop_azimuth(web::kD1StillAzimuthDeg, 360.0f, k, 450u)));
    }
    EXPECT_LE(worst_s1, -1.06f) << "S1's floor top, NDC y (-1 is the bottom edge)";
    // S8: the swing out and back (a 60 degree sweep, 300 frames).
    float worst_s8 = -9.0f;
    for (uint32_t k = 0; k < 300u; ++k) {
        worst_s8 = std::max(worst_s8, still(web::kD1StillSubjectM, web::there_and_back_azimuth(web::kD1StillAzimuthDeg, 60.0f, k, 300u)));
    }
    EXPECT_LE(worst_s8, -1.06f) << "S8's floor top";
    // S2: the push-in from 0.45 m to 0.30 m, 360 frames, at a fixed azimuth.
    const float wide = web::fit_distance(web::kD1PushWideM, kFov, kAspect, web::kMargin);
    const float close = web::fit_distance(web::kD1PushCloseM, kFov, kAspect, web::kMargin);
    float worst_s2 = -9.0f;
    for (uint32_t k = 0; k < 360u; ++k) {
        worst_s2 = std::max(worst_s2, web::floor_top_ndc(aim, web::push_in_distance(wide, close, k, 360u),
                                                         web::kD1StillAzimuthDeg, web::kD1StillElevationDeg, kFov));
    }
    EXPECT_LE(worst_s2, -1.04f) << "S2's floor top";
    EXPECT_NEAR(wide, 1.073f, 1e-3f);
    EXPECT_NEAR(close, 0.715f, 1e-3f);
    // The test discriminates: the quad's old framing (0.55 m subject, 20 degrees) leaves the floor in frame.
    const float old_d = web::fit_distance(0.55f, kFov, kAspect, web::kMargin);
    EXPECT_GT(web::floor_top_ndc(aim, old_d, 25.0f, 20.0f, kFov), -0.8f);
    // A corner behind the camera's plane is not known to be out of frame.
    EXPECT_GT(web::floor_top_ndc(glm::vec3(0.0f, -2.0f, 0.0f), 0.1f, 0.0f, 80.0f, kFov), 1e9f);
}

TEST(FlowMovieWeb, ASwingOutAndBackStartsAndEndsAtRestAndLoops) {
    const uint32_t n = 300;
    EXPECT_FLOAT_EQ(web::there_and_back_azimuth(25.0f, 60.0f, 0, n), 25.0f);
    EXPECT_NEAR(web::there_and_back_azimuth(25.0f, 60.0f, n, n), 25.0f, 1e-4f);  // frame n would be frame 0
    EXPECT_NEAR(web::there_and_back_azimuth(25.0f, 60.0f, n / 2, n), 85.0f, 1e-3f);
    float peak_step = 0.0f;
    for (uint32_t k = 1; k < n; ++k) {
        const float s = std::abs(web::there_and_back_azimuth(0.0f, 60.0f, k, n) - web::there_and_back_azimuth(0.0f, 60.0f, k - 1, n));
        peak_step = std::max(peak_step, s);
        ASSERT_LE(web::there_and_back_azimuth(0.0f, 60.0f, k, n), 60.0f + 1e-4f);
    }
    EXPECT_LT(peak_step, 0.7f);  // never a fast turn: under 21 degrees a second at 30 fps
    EXPECT_LT(web::there_and_back_azimuth(0.0f, 60.0f, 1, n), 0.01f);  // it eases in
}

// The volume shows speed (the lead, 2026-11-10): |w| is a derivative and non-negative, so grid-scale ripple in the
// field rectifies into a fog; the colour rule is viridis and the caption says "speed".
TEST(FlowMovieWeb, TheVolumeClipShowsSpeedOverTheQuadsOneRangeAndItsCaptionSaysSpeed) {
    const web::Clip& s8 = *web::find_clip("S8");
    EXPECT_EQ(s8.quantity, web::Quantity::speed);
    EXPECT_EQ(s8.range_lo, 0.0f);
    EXPECT_EQ(s8.range_hi, 12.0f);
    EXPECT_EQ(s8.range_hi, web::find_clip("S1")->range_hi) << "one colour range a scene and quantity";
    EXPECT_TRUE(has(std::string(s8.shows), "speed"));
    EXPECT_FALSE(has(std::string(s8.shows), "ortic")) << "no vorticity claim in the caption";
}

TEST(FlowMovieWeb, OnlyAFullOrbitOrASwingBackLoopsTheCamera) {
    EXPECT_TRUE(web::find_clip("S1")->camera_loops());
    EXPECT_TRUE(web::find_clip("S8")->camera_loops());
    EXPECT_TRUE(has(web::loop_line(*web::find_clip("S8")), "swings out and back and loops"));
    for (const char* id : {"S2", "S3", "S4", "S5", "S6"}) {
        EXPECT_FALSE(web::find_clip(id)->camera_loops()) << id;
    }
    EXPECT_TRUE(has(web::loop_line(*web::find_clip("S1")), "completes one full orbit and loops"));
    EXPECT_TRUE(has(web::loop_line(*web::find_clip("S5")), "the camera is still"));
    EXPECT_TRUE(has(web::loop_line(*web::find_clip("S3")), "sweeps 180 degrees and does not return"));
    for (const web::Clip& c : web::web_clips()) {
        EXPECT_TRUE(has(web::loop_line(c), "flow does not")) << c.id << ": the flow is never claimed to loop";
    }
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
    EXPECT_TRUE(has(text, "job ticket web-spade-S3")) << "the ticket is the jobs service's (O25)";
    EXPECT_FALSE(has(text, "jobrun")) << "the old runner's name is gone from the caption";
    EXPECT_TRUE(has(text, "flow_movie --web S3"));
    EXPECT_TRUE(has(text, "Slow motion"));
    EXPECT_TRUE(has(text, "times slower than real time"));
    EXPECT_TRUE(has(text, "the camera sweeps 180 degrees and does not return to its start; the flow does not loop"));
    EXPECT_TRUE(has(text, "Colour shows relative speed, not validated speed"));
    EXPECT_TRUE(has(text, "crosswind setting supported but not validated"));
    EXPECT_TRUE(has(text, "the rotors do not respond to it"));
    EXPECT_TRUE(has(text, "the rotor model's push does not respond to the side wind, and the grid carries the wake downwind"));
    EXPECT_TRUE(has(text, "not a prediction of the thrust, power or attitude a real quad would need"));
    EXPECT_TRUE(has(text, "at 0.2 ms with a Smagorinsky subgrid viscosity (Cs 0.17)"));
    EXPECT_FALSE(has(text, "bit-equal")) << "the term is new on the device: no equality with the CPU is claimed";
    EXPECT_TRUE(has(text, "a single run on the device of record"));
    EXPECT_FALSE(has(text, "identical to the CPU"));
    const std::string standing = web::standing_caveats(*s3);
    EXPECT_FALSE(has(standing, "m/s")) << "no speed figures in the standing caveats";
    EXPECT_TRUE(has(web::standing_caveats(*web::find_clip("S5")), "inviscid")) << "the quad's CPU film";
    EXPECT_TRUE(has(web::standing_caveats(*web::find_clip("S3")), "steady wind from the side"));
    EXPECT_FALSE(has(web::standing_caveats(*web::find_clip("S2")), "wind from the side")) << "S2 is in still air";
    const std::string s1_caveats = web::standing_caveats(*web::find_clip("S1"));
    EXPECT_TRUE(has(s1_caveats, "rigid parts as solids in the flow at 4 mm"));
    EXPECT_TRUE(has(s1_caveats, "hover throttle in still air"));
    EXPECT_TRUE(has(s1_caveats, "Smagorinsky subgrid viscosity (Cs 0.17)"));
    EXPECT_FALSE(has(s1_caveats, "wind from the side")) << "S1 and S8 are in still air";
    EXPECT_FALSE(has(s1_caveats, "bit-equal"));
    EXPECT_TRUE(has(web::standing_caveats(*web::find_clip("S4")), "sign on the arms' tops has not been measured"));
    for (const web::Clip& c : web::web_clips()) {
        EXPECT_FALSE(has(web::standing_caveats(c), "m/s")) << c.id << ": no speed figures";
        EXPECT_TRUE(has(web::standing_caveats(c), "relative speed, not validated")) << c.id;
    }
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

// --- the perf stopwatch (L74, Test/Docs' PERF.md) --------------------------------------------------------------

TEST(FlowMovieWeb, ThePerfMedianIsTheMiddleFrameAndOneSlowFrameDoesNotMoveIt) {
    EXPECT_EQ(web::median_of({}), 0.0);
    EXPECT_EQ(web::median_of({0.5}), 0.5);
    EXPECT_EQ(web::median_of({3.0, 1.0, 2.0}), 2.0);
    EXPECT_EQ(web::median_of({4.0, 1.0, 2.0, 3.0}), 2.5);  // an even count: the mean of the middle two
    EXPECT_EQ(web::median_of({0.6, 0.6, 0.6, 0.6, 9.0}), 0.6);  // the first frame's one-off cost is not the frame's cost
}

TEST(FlowMovieWeb, APerfLineIsTheNameTheValueAndTheUnit) {
    EXPECT_EQ(web::perf_line("rendering.web_frame_4k", 0.67149, "s"), "PERF rendering.web_frame_4k 0.6715 s");
    EXPECT_EQ(web::perf_line("rendering.diag_web_box_down", 0.032, "s"), "PERF rendering.diag_web_box_down 0.0320 s");
    EXPECT_EQ(web::web_cache_note("C:/v/cache/d1-still", 180, 123456789u),
              "PERF-NOTE web_cache C:/v/cache/d1-still 180 frames 123456789 bytes");
}

TEST(FlowMovieWeb, AWebRunPrintsTheFramesMedianAndEachPhaseUnderTheLedgersNames) {
    std::vector<web::FrameTimes> frames(3);
    const double frame[3] = {0.9, 0.6, 0.7};  // the whole loop body of each frame
    for (int i = 0; i < 3; ++i) {
        frames[i].frame = frame[i];
        frames[i].cache_read = 0.010 * (i + 1);
        frames[i].streamline_shot = 0.100;
        frames[i].gl_upload = 0.020;
        frames[i].render = 0.050;
        frames[i].readback = 0.080;
        frames[i].flip = 0.030;
        frames[i].box_down = 0.150;
        frames[i].pipe = 0.200;
    }
    const std::vector<std::string> lines = web::web_perf_lines(frames, 3.0);
    ASSERT_FALSE(lines.empty());
    EXPECT_EQ(lines.front(), "PERF rendering.web_frame_4k 0.7000 s");  // the median of 0.9, 0.6, 0.7, first
    const auto line_of = [&](const std::string& name) {
        for (const std::string& line : lines) {
            if (line.rfind("PERF rendering." + name + " ", 0) == 0) return line;
        }
        return std::string("(missing ") + name + ")";
    };
    EXPECT_EQ(line_of("diag_web_cache_read"), "PERF rendering.diag_web_cache_read 0.0200 s");
    EXPECT_EQ(line_of("diag_web_streamline_shot"), "PERF rendering.diag_web_streamline_shot 0.1000 s");
    EXPECT_EQ(line_of("diag_web_gl_upload"), "PERF rendering.diag_web_gl_upload 0.0200 s");
    EXPECT_EQ(line_of("diag_web_render"), "PERF rendering.diag_web_render 0.0500 s");
    EXPECT_EQ(line_of("diag_web_readback"), "PERF rendering.diag_web_readback 0.0800 s");
    EXPECT_EQ(line_of("diag_web_flip"), "PERF rendering.diag_web_flip 0.0300 s");
    EXPECT_EQ(line_of("diag_web_box_down"), "PERF rendering.diag_web_box_down 0.1500 s");
    EXPECT_EQ(line_of("diag_web_pipe"), "PERF rendering.diag_web_pipe 0.2000 s");
    // The encode's tail after the last frame, shared over the clip's frames: 3.0 s over 3 frames.
    EXPECT_EQ(line_of("diag_web_encode_tail"), "PERF rendering.diag_web_encode_tail 1.0000 s");
    EXPECT_TRUE(web::web_perf_lines({}, 1.0).empty());  // no frames, no figures
}

// --sgs-cs X (Physics' ask, L86): a diagnostic run of a D1 clip at another Smagorinsky Cs (0 turns the term off), so a
// still pair at one developed time shows Cameron what the term does. The default stays kD1WebSgsCs, and a run at any
// other Cs keeps its frames in a folder of its own: it can never be mistaken for, or overwrite, the master's cache.
TEST(FlowMovieWeb, TheSgsCsOptionIsParsedStrictlyAndNamesItsOwnCacheFolder) {
    ASSERT_TRUE(web::parse_sgs_cs("0").has_value());
    EXPECT_FLOAT_EQ(*web::parse_sgs_cs("0"), 0.0f);
    EXPECT_FLOAT_EQ(*web::parse_sgs_cs("0.17"), 0.17f);
    EXPECT_FLOAT_EQ(*web::parse_sgs_cs("0.25"), 0.25f);
    EXPECT_FLOAT_EQ(*web::parse_sgs_cs("1"), 1.0f);
    // Anything that is not one finite number from 0 to 1 is refused, whole: no trailing text, no blanks, no sign.
    for (const char* bad : {"", " ", "-0.1", "-0", "+0.1", "nan", "inf", "1.5", "0.17x", " 0.2", "0.2 ", "1e-1", "abc", "0,17"}) {
        EXPECT_FALSE(web::parse_sgs_cs(bad).has_value()) << "'" << bad << "'";
    }
    // The master's Cs names no folder suffix; any other Cs names its own.
    EXPECT_EQ(web::sgs_cs_suffix(web::kD1WebSgsCs), "");
    EXPECT_EQ(web::sgs_cs_suffix(0.0f), "-cs0");
    EXPECT_EQ(web::sgs_cs_suffix(0.25f), "-cs0.25");
    EXPECT_EQ(web::sgs_cs_suffix(0.1f), "-cs0.1");
    EXPECT_NE(web::sgs_cs_suffix(0.0f), web::sgs_cs_suffix(0.1f));
    // The option is for the D1 clips: S5 and S6 run the quad's CPU film, which has no subgrid term.
    for (const char* id : {"S1", "S2", "S3", "S4", "S8"}) EXPECT_TRUE(web::sgs_cs_applies(*web::find_clip(id))) << id;
    for (const char* id : {"S5", "S6"}) EXPECT_FALSE(web::sgs_cs_applies(*web::find_clip(id))) << id;
}

// L106c (the lead, 2026-11-19): the website film at the L106 config. --cell-mm X is the D1 box's cell size (only 2.5, the X500's box, with
// --drone x500 on a still-air clip); --step-us N is the solver step in whole microseconds (50 is L106's 0.05 ms), for a --window run, whose frames
// are spread over the window and so do not fix a step count. Both parse strictly, name their own cache folder, and change nothing when absent.
TEST(FlowMovieWeb, TheCellAndStepOptionsAreParsedStrictlyAndRefusedWhereTheyDoNotApply) {
    ASSERT_TRUE(web::parse_cell_mm("2.5").has_value());
    EXPECT_FLOAT_EQ(*web::parse_cell_mm("2.5"), 2.5f);
    EXPECT_FLOAT_EQ(*web::parse_cell_mm("4"), 4.0f);
    for (const char* bad : {"", " ", "0", "0.0", "-2.5", "+2.5", "nan", "inf", "2.5x", " 2.5", "2.5 ", "1e1", "2,5", "abc", "11", "100"}) {
        EXPECT_FALSE(web::parse_cell_mm(bad).has_value()) << "'" << bad << "'";
    }
    ASSERT_TRUE(web::parse_step_us("50").has_value());
    EXPECT_EQ(*web::parse_step_us("50"), 50u);
    EXPECT_EQ(*web::parse_step_us("1"), 1u);
    EXPECT_EQ(*web::parse_step_us("1000"), 1000u);
    for (const char* bad : {"", " ", "0", "-50", "+50", "50.5", "50us", "5e1", " 50", "50 ", "1001", "abc", "99999999999"}) {
        EXPECT_FALSE(web::parse_step_us(bad).has_value()) << "'" << bad << "'";
    }
    // The cell option: 2.5 with the X500 on a still-air clip, and nothing else, each refusal naming what is supported.
    EXPECT_EQ(web::cell_mm_refusal(2.5f, "x500", true), "");
    for (const std::string& why : {web::cell_mm_refusal(4.0f, "x500", true), web::cell_mm_refusal(2.5f, "reference_quad", true),
                                   web::cell_mm_refusal(2.5f, "x500", false)}) {
        EXPECT_NE(why.find("--cell-mm"), std::string::npos) << why;
        EXPECT_NE(why.find("2.5"), std::string::npos) << why;
        EXPECT_NE(why.find("x500"), std::string::npos) << why;
    }
    // The step option is for a --window run: the group's frames are 5 whole steps at 0.2 ms.
    EXPECT_EQ(web::step_us_refusal(50u, true), "");
    EXPECT_NE(web::step_us_refusal(50u, false).find("--window"), std::string::npos);
    // A run at another cell or step keeps its frames in a folder of its own, as another Cs does.
    EXPECT_EQ(web::cell_step_stem_suffix(std::nullopt, std::nullopt), "");
    EXPECT_EQ(web::cell_step_stem_suffix(2.5f, 50u), "-cell2.5-step50us");
    EXPECT_EQ(web::cell_step_stem_suffix(2.5f, std::nullopt), "-cell2.5");
    EXPECT_EQ(web::cell_step_stem_suffix(std::nullopt, 50u), "-step50us");
    EXPECT_NE(web::cell_step_stem_suffix(2.5f, 50u), web::cell_step_stem_suffix(2.5f, 100u));
}

// The film's caption says what it is (L106h, Oversight O101): the X500's rotors are 10 inch, not 13; a --lines film's rotors are actuator LINES with the fitted
// blade polar, never "discs of fixed thrust and swirl"; a --particles film SHOWS air points (and the speed volume only when --volume put it there); and
// the command line carries no user path (--out and --cache-dir show from build-ninja/ on, else the folder's own name).
TEST(FlowMovieWeb, TheCaptionSaysTenInchLinesAndPointsAndNamesNoUserPath) {
    const web::Drone& x500 = *web::find_drone("x500");
    EXPECT_NE(std::string(x500.description).find("10 inch"), std::string::npos) << x500.description;
    EXPECT_EQ(std::string(x500.description).find("13 inch"), std::string::npos) << x500.description;

    const web::Clip& s8 = *web::find_clip("S8");
    const std::string discs = web::standing_caveats(s8, 2.5f, 50u);
    EXPECT_NE(discs.find("actuator discs"), std::string::npos) << "no --lines: the discs' words, as they were";
    const std::string lines = web::standing_caveats(s8, 2.5f, 50u, true);
    EXPECT_NE(lines.find("actuator lines"), std::string::npos) << lines;
    EXPECT_EQ(lines.find("actuator discs"), std::string::npos) << lines;
    EXPECT_EQ(lines.find("fixed thrust and swirl"), std::string::npos) << lines;
    EXPECT_NE(lines.find("fitted blade polar"), std::string::npos) << lines;
    EXPECT_NE(lines.find("a visualisation of the air, not a prediction of the thrust"), std::string::npos) << "the stand-in sentence stays: " << lines;

    web::CaptionInput in;
    in.clip = &s8;
    in.drone = &x500;
    in.lines = true;
    in.particles = true;
    in.command = "flow_movie --web S8 --drone x500 --out C:/Users/Someone/Projects/spade-wt/rendering-film/build-ninja/film-l106c-film --cache-dir D:/work/cache/one --particles";
    const std::string points = web::caption_text(in);
    EXPECT_NE(points.find("- Shows: Air points"), std::string::npos) << points;
    EXPECT_EQ(points.find("speed volume"), std::string::npos) << "no --volume: the volume is not claimed: " << points;
    EXPECT_EQ(points.find("actuator discs"), std::string::npos) << points;
    EXPECT_NE(points.find("--out build-ninja/film-l106c-film"), std::string::npos) << points;
    EXPECT_NE(points.find("--cache-dir one"), std::string::npos) << points;
    EXPECT_EQ(points.find("Someone"), std::string::npos) << points;
    EXPECT_EQ(points.find("D:/work"), std::string::npos) << points;
    in.volume = true;
    EXPECT_NE(web::caption_text(in).find("over the relative-speed volume"), std::string::npos) << "--volume: the volume is claimed";
    web::CaptionInput old;
    old.clip = &s8;
    old.command = "flow_movie --web S8";
    EXPECT_NE(web::caption_text(old).find("Relative speed of the wake of four actuator discs"), std::string::npos) << "an old film's Shows line is as it was";
}

// --volume (L106h): the S8 particle look draws the points ALONE (its view has no flow layer), so a "volume plus particles" film needs the speed volume under
// the points. --volume adds that layer to the --particles and --particles-preview views; alone it has nothing to add it to, and a film with it keeps a folder of
// its own as every other look does.
TEST(FlowMovieWeb, VolumeIsForTheParticleViewsAndHasAFolderOfItsOwn) {
    EXPECT_TRUE(web::volume_with_particles_refusal(false, false, false).empty()) << "no --volume: nothing to say";
    EXPECT_TRUE(web::volume_with_particles_refusal(true, true, false).empty());
    EXPECT_TRUE(web::volume_with_particles_refusal(true, false, true).empty());
    const std::string alone = web::volume_with_particles_refusal(true, false, false);
    EXPECT_NE(alone.find("--volume"), std::string::npos) << alone;
    EXPECT_NE(alone.find("--particles"), std::string::npos) << alone;
    EXPECT_EQ(web::volume_stem_suffix(false), "");
    EXPECT_EQ(web::volume_stem_suffix(true), "-volume");
}

// --nasa-cloud under a streamed --window run (L106i, Oversight O113): the X500 film's main layer is the purple-blue vorticity cloud (no particles). The cloud
// is allowed with a --window run that streams, as --particles is; the Q SURFACES keep their old rules (S2 and S8, a redraw of kept frames, no particle look),
// and the particle looks stay a different look from --nasa and --nasa-cloud.
TEST(FlowMovieWeb, TheVorticityCloudStreamsAWindow) {
    // nasa_refusal(s2_or_s8, window, still_redraw, stream, particles, particles_preview, cloud)
    EXPECT_EQ(web::nasa_refusal(true, false, false, false, false, false, false), "") << "--nasa on its own: the group's own run, as before";
    EXPECT_EQ(web::nasa_refusal(true, true, true, false, false, false, false), "") << "a window redraw of kept frames, as before";
    EXPECT_EQ(web::nasa_refusal(true, true, true, false, false, false, true), "") << "a redraw of the cloud";
    EXPECT_EQ(web::nasa_refusal(true, true, false, true, false, false, true), "") << "the streamed cloud film: window, stream, cloud";
    EXPECT_EQ(web::nasa_refusal(true, false, false, true, false, false, true), "") << "a streamed run of the group's own window";

    const std::string not_clip = web::nasa_refusal(false, false, false, false, false, false, false);
    EXPECT_NE(not_clip.find("S2 and S8"), std::string::npos) << not_clip;
    const std::string no_stream = web::nasa_refusal(true, true, false, false, false, false, true);
    EXPECT_NE(no_stream.find("--window"), std::string::npos) << no_stream;
    EXPECT_NE(no_stream.find("--stream"), std::string::npos) << no_stream;
    const std::string surfaces = web::nasa_refusal(true, true, false, true, false, false, false);
    EXPECT_NE(surfaces.find("--nasa-cloud"), std::string::npos) << "the Q surfaces do not stream a window: " << surfaces;
    const std::string points = web::nasa_refusal(true, true, false, true, true, false, true);
    EXPECT_NE(points.find("particle"), std::string::npos) << "the cloud and the points are different looks: " << points;
    const std::string preview = web::nasa_refusal(true, false, false, false, false, true, true);
    EXPECT_NE(preview.find("particle"), std::string::npos) << preview;
}

// The caption of a vorticity-cloud film says what the colour is: vorticity (the local spin of the air), blue low to magenta high, never "relative speed" or the
// speed film's "Relative speed of the wake"; the stand-in sentence stays; and a film without --nasa-cloud keeps its words.
TEST(FlowMovieWeb, TheCloudFilmsCaptionSaysVorticityNotSpeed) {
    const web::Clip& s8 = *web::find_clip("S8");
    const std::string speed = web::standing_caveats(s8, 2.5f, 50u, true);
    EXPECT_NE(speed.find("Colour shows relative speed"), std::string::npos) << "no cloud: the words as they were: " << speed;
    const std::string cloud = web::standing_caveats(s8, 2.5f, 50u, true, true);
    EXPECT_EQ(cloud.find("relative speed"), std::string::npos) << cloud;
    EXPECT_NE(cloud.find("Colour shows vorticity"), std::string::npos) << cloud;
    EXPECT_NE(cloud.find("blue low to magenta high"), std::string::npos) << cloud;
    EXPECT_NE(cloud.find("a visualisation of the air, not a prediction of the thrust"), std::string::npos) << "the stand-in sentence stays: " << cloud;
    EXPECT_NE(cloud.find("fitted blade polar"), std::string::npos) << cloud;

    web::CaptionInput in;
    in.clip = &s8;
    in.drone = web::find_drone("x500");
    in.lines = true;
    in.cloud = true;
    in.command = "flow_movie --web S8 --drone x500 --nasa-cloud";
    const std::string text = web::caption_text(in);
    EXPECT_NE(text.find("- Shows: The vorticity of the air"), std::string::npos) << text;
    EXPECT_NE(text.find("four actuator lines"), std::string::npos) << text;
    EXPECT_EQ(text.find("elative speed"), std::string::npos) << text;
    EXPECT_EQ(text.find("actuator discs"), std::string::npos) << text;
    web::CaptionInput old;
    old.clip = &s8;
    old.command = "flow_movie --web S8";
    EXPECT_NE(web::caption_text(old).find("Relative speed of the wake of four actuator discs"), std::string::npos) << "a film without the cloud keeps its words";
}

// --surface-pressure (O124): the third X500 website clip, the air pressure on the airframe's surfaces. An S8 look only; like the cloud it streams a --window run
// (or redraws kept frames), and it is a different look from --nasa, --nasa-cloud and the particle looks.
TEST(FlowMovieWeb, ThePressureFilmIsAnS8LookThatStreamsAWindow) {
    // surface_pressure_refusal(s8, window, still_redraw, stream, particles, particles_preview, nasa)
    EXPECT_EQ(web::surface_pressure_refusal(true, false, false, false, false, false, false), "") << "the group's own run";
    EXPECT_EQ(web::surface_pressure_refusal(true, true, false, true, false, false, false), "") << "the streamed film: window and stream";
    EXPECT_EQ(web::surface_pressure_refusal(true, true, true, false, false, false, false), "") << "a window redraw of kept frames";

    const std::string not_s8 = web::surface_pressure_refusal(false, false, false, false, false, false, false);
    EXPECT_NE(not_s8.find("S8"), std::string::npos) << not_s8;
    const std::string no_stream = web::surface_pressure_refusal(true, true, false, false, false, false, false);
    EXPECT_NE(no_stream.find("--window"), std::string::npos) << no_stream;
    EXPECT_NE(no_stream.find("--stream"), std::string::npos) << no_stream;
    EXPECT_NE(web::surface_pressure_refusal(true, true, false, true, true, false, false).find("particle"), std::string::npos);
    EXPECT_NE(web::surface_pressure_refusal(true, true, false, true, false, true, false).find("particle"), std::string::npos);
    EXPECT_NE(web::surface_pressure_refusal(true, true, false, true, false, false, true).find("--nasa"), std::string::npos);
}

// The pressure film draws surfaces only: no wake is drawn, so the wake fade (the soft edge that hides the box's cut) does not apply.
TEST(FlowMovieWeb, ThePressureFilmDrawsNoWakeFade) {
    EXPECT_TRUE(web::web_fade_applies("S8", false, false, false, false)) << "the speed volume keeps its fade";
    EXPECT_FALSE(web::web_fade_applies("S8", false, false, false, false, true)) << "the pressure film has no wake to fade";
}

// The caption of a pressure film says what the colour is: the air's pressure on the surfaces, blue below ambient to red above, no figures; first order in the cell
// size, so where the push and the suction are and not validated magnitudes; never "relative speed" or vorticity. The stand-in sentence stays.
TEST(FlowMovieWeb, ThePressureFilmsCaptionSaysPressureOnTheSurfaces) {
    const web::Clip& s8 = *web::find_clip("S8");
    const std::string pressure = web::standing_caveats(s8, 2.5f, 50u, true, false, true);
    EXPECT_EQ(pressure.find("relative speed"), std::string::npos) << pressure;
    EXPECT_EQ(pressure.find("vorticity"), std::string::npos) << pressure;
    EXPECT_NE(pressure.find("Colour shows pressure on the surfaces"), std::string::npos) << pressure;
    EXPECT_NE(pressure.find("blue below the air's ambient pressure to red above"), std::string::npos) << pressure;
    EXPECT_NE(pressure.find("no pressure or speed figures"), std::string::npos) << pressure;
    EXPECT_NE(pressure.find("not validated magnitudes"), std::string::npos) << pressure;
    EXPECT_NE(pressure.find("a visualisation of the air, not a prediction of the thrust"), std::string::npos) << "the stand-in sentence stays: " << pressure;
    EXPECT_NE(pressure.find("fitted blade polar"), std::string::npos) << pressure;
    EXPECT_NE(web::standing_caveats(s8, 2.5f, 50u, true).find("Colour shows relative speed"), std::string::npos) << "no pressure: the words as they were";

    web::CaptionInput in;
    in.clip = &s8;
    in.drone = web::find_drone("x500");
    in.lines = true;
    in.pressure = true;
    in.command = "flow_movie --web S8 --drone x500 --surface-pressure";
    const std::string text = web::caption_text(in);
    EXPECT_NE(text.find("- Shows: The air pressure on the surfaces"), std::string::npos) << text;
    EXPECT_NE(text.find("four actuator lines"), std::string::npos) << text;
    EXPECT_EQ(text.find("elative speed"), std::string::npos) << text;
    EXPECT_EQ(text.find("vorticity"), std::string::npos) << text;
    EXPECT_EQ(text.find("actuator discs"), std::string::npos) << text;
}

// The caption's standing caveats say the cell and the step the film ran at: today's are 4 mm and 0.2 ms, and a --cell-mm 2.5 --step-us 50 film says
// 2.5 mm and 0.05 ms, in the same words, so a caption never states another run's numbers. Defaults leave the words as they were.
TEST(FlowMovieWeb, TheCaptionStatesTheCellAndStepTheFilmRanAt) {
    const web::Clip& s8 = *web::find_clip("S8");
    const std::string today = web::standing_caveats(s8);
    EXPECT_NE(today.find("in the flow at 4 mm"), std::string::npos) << today;
    EXPECT_NE(today.find("at 0.2 ms with a Smagorinsky"), std::string::npos) << today;
    EXPECT_NE(today.find("the 4 mm cells cannot hold"), std::string::npos) << today;
    const std::string film = web::standing_caveats(s8, 2.5f, 50u);
    EXPECT_NE(film.find("in the flow at 2.5 mm"), std::string::npos) << film;
    EXPECT_NE(film.find("at 0.05 ms with a Smagorinsky"), std::string::npos) << film;
    EXPECT_NE(film.find("the 2.5 mm cells cannot hold"), std::string::npos) << film;
    EXPECT_EQ(film.find("4 mm"), std::string::npos) << "no other run's number: " << film;
    EXPECT_EQ(film.find("0.2 ms"), std::string::npos) << film;
    EXPECT_NE(film.find("a visualisation of the air, not a prediction of the thrust"), std::string::npos) << film;
    web::CaptionInput in;
    in.clip = &s8;
    in.cell_mm = 2.5f;
    in.step_us = 50u;
    EXPECT_NE(web::caption_text(in).find("at 0.05 ms"), std::string::npos);
    web::CaptionInput plain;
    plain.clip = &s8;
    EXPECT_NE(web::caption_text(plain).find("at 0.2 ms"), std::string::npos) << "a caption with no knobs says today's numbers";
}

// --particles with --window: the points are carried frame to frame, so a --window run that solves (--stream) feeds them in order, and a --web-still
// redraw from kept frames carries them through the files (L106b). A --window run that neither streams nor redraws has no frames to carry them through.
TEST(FlowMovieWeb, ParticlesWithAWindowNeedAStreamOrARedrawFromKeptFrames) {
    EXPECT_EQ(web::particles_window_refusal(false, false, false), "") << "no window: the group's own run";
    EXPECT_EQ(web::particles_window_refusal(false, false, true), "");
    EXPECT_EQ(web::particles_window_refusal(true, true, false), "") << "a window redraw of kept frames";
    EXPECT_EQ(web::particles_window_refusal(true, false, true), "") << "a streamed window run (L106c)";
    const std::string why = web::particles_window_refusal(true, false, false);
    EXPECT_NE(why.find("--window"), std::string::npos) << why;
    EXPECT_NE(why.find("--stream"), std::string::npos) << why;
}

// S1's caption says where the streamlines start, and says what the code does: seeded 6 cm BELOW each disc's hub (in two rings
// of eight round the axis) and traced both ways. It once said "above each of four actuator discs", which the picture contradicts.
TEST(FlowMovieWeb, S1sCaptionSaysWhereTheStreamlinesAreSeededAndHowTheyAreTraced) {
    const std::string shows(web::find_clip("S1")->shows);
    EXPECT_NE(shows.find("6 cm below"), std::string::npos) << shows;
    EXPECT_NE(shows.find("both ways"), std::string::npos) << shows;
    EXPECT_EQ(shows.find("above"), std::string::npos) << shows;
    EXPECT_NE(shows.find("fade out toward their ends"), std::string::npos) << "the soft look is declared: " << shows;
    EXPECT_LE(web::kSoftLineWidthPx, 2.0f) << "thin, not a tube";
    EXPECT_GT(web::kSoftLineGlowPx, web::kSoftLineWidthPx);
    EXPECT_GT(web::kSoftLineFadeM, 0.0f);
    EXPECT_FLOAT_EQ(web::kWebSeedBelowHubM, 0.06f) << "the sentence and the seeds share one number";
}

// S2 is a soft volume of the wake's speed, in a push-in from the start-up (Cameron, 2026-11-17: the vortex surfaces "look like
// playdoh"; S8's volume is the direction), so its caption and its file name say a volume, not vortex surfaces.
TEST(FlowMovieWeb, S2IsAVolumeOfTheWakesSpeedNotVortexSurfaces) {
    const web::Clip& s2 = *web::find_clip("S2");
    const std::string shows(s2.shows);
    EXPECT_NE(shows.find("volume"), std::string::npos) << shows;
    EXPECT_EQ(shows.find("ortex surfaces"), std::string::npos) << shows;
    EXPECT_EQ(shows.find("lambda2"), std::string::npos) << shows;
    EXPECT_EQ(std::string(s2.short_name), "rotor-wake-start-up-volume");
    EXPECT_EQ(std::string(s2.title).find("ortex"), std::string::npos) << s2.title;
}

// A drone's description states its mass, and the figure is the one the flow model flies: the body mass in the model file (the X500's is the summed parts'
// 1.63259995 kg, not the old 2 kg), to two decimals. Every drone with a figure is held to it (Spade lead, 2026-11-20).
TEST(FlowMovieWeb, ADroneDescriptionsMassIsTheModelFilesBodyMass) {
    std::size_t with_figure = 0;
    for (const web::Drone& d : web::drones()) {
        const std::string description(d.description);
        const std::size_t kg = description.find(" kg ");
        if (kg == std::string::npos) continue;  // the reference quad's description is empty
        ++with_figure;
        const double said = std::stod(description.substr(description.find(' ') + 1u));  // "a 1.63 kg ..."
        std::ifstream file(std::filesystem::path(SPADE_ASSETS_DIR) / d.model_file);
        ASSERT_TRUE(file.good()) << d.model_file;
        std::stringstream all;
        all << file.rdbuf();
        const std::string text = all.str();
        const std::size_t at = text.find("body: {mass: ");
        ASSERT_NE(at, std::string::npos) << d.model_file << " has no body mass";
        const double flown = std::stod(text.substr(at + 13u));
        EXPECT_NEAR(said, flown, 0.005) << "'" << description << "' says " << said << " kg; the model file flies " << flown << " kg";
    }
    EXPECT_GE(with_figure, 2u) << "the camera quadcopter and the X500 both state a mass";
}

// --drone takes a library drone by its internal name, and no caption, title or file name carries that name or a brand: the
// clip says "a 1.28 kg camera quadcopter (generated from published dimensions)" (the lead, 2026-11-17; the mass as the model file has it, 2026-11-20). The default is the
// reference quad, with today's words and today's cache names.
TEST(FlowMovieWeb, ALibraryDroneIsNamedOnTheCommandLineAndNeverInACaption) {
    const web::Drone* drone = web::find_drone("phantom3_class");
    ASSERT_NE(drone, nullptr);
    EXPECT_EQ(web::find_drone("no_such_drone"), nullptr);
    EXPECT_EQ(web::find_drone("camera-quadcopter"), drone) << "the caption's alias names the same drone";
    EXPECT_EQ(std::string(web::default_drone().library), "reference_quad");
    EXPECT_EQ(web::drone_stem_suffix(web::default_drone()), "") << "today's caches keep their names";
    EXPECT_EQ(web::drone_stem_suffix(*drone), "-phantom3_class");
    for (const char* id : {"S1", "S2", "S3", "S4", "S8"}) {
        const web::Clip* clip = web::find_clip(id);
        ASSERT_NE(clip, nullptr) << id;
        web::CaptionInput in;
        in.clip = clip;
        in.drone = drone;
        in.commit = "abc123def456";
        in.date_utc = "2026-11-17 03:00 UTC";
        in.command = std::string("flow_movie --web ") + id + " --drone phantom3_class";
        in.ticket = "web-spade-test";
        in.sim_seconds = 0.3;
        in.screen_seconds = 10.0;
        std::string text = web::caption_text(in);
        std::string title(web::clip_title(*clip, *drone));
        std::string lower = text + " " + title;
        for (char& ch : lower) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        EXPECT_EQ(lower.find("phantom"), std::string::npos) << id;
        EXPECT_EQ(lower.find("dji"), std::string::npos) << id;
        EXPECT_EQ(lower.find("reference quad"), std::string::npos) << id << ": " << text;
        EXPECT_NE(text.find("a 1.28 kg camera quadcopter (generated from published dimensions)"), std::string::npos) << id;
        EXPECT_NE(title.find("amera quadcopter"), std::string::npos) << title;
    }
    web::CaptionInput plain;
    plain.clip = web::find_clip("S8");
    plain.commit = "c";
    plain.date_utc = "d";
    plain.command = "x";
    plain.ticket = "t";
    plain.sim_seconds = 0.3;
    plain.screen_seconds = 10.0;
    EXPECT_NE(web::caption_text(plain).find("reference quad"), std::string::npos) << "no drone: today's caption";
    EXPECT_EQ(web::clip_title(*plain.clip, web::default_drone()), std::string(plain.clip->title));
}

// A drone has its own framing (the lead, 2026-11-17): the reference quad's is today's constants and no fade, so its frames do not
// move; the library quadcopter is larger, in the upper third of the frame, with its wake fading out before the frame's bottom.
TEST(FlowMovieWeb, EachDroneHasItsOwnFramingAndTheDefaultsIsTodays) {
    const web::Framing& d = web::default_drone().framing;
    EXPECT_FLOAT_EQ(d.subject_m, web::kD1StillSubjectM);
    EXPECT_FLOAT_EQ(d.aim_below_m, web::kD1StillAimBelowM);
    EXPECT_FLOAT_EQ(d.push_wide_m, web::kD1PushWideM);
    EXPECT_FLOAT_EQ(d.push_close_m, web::kD1PushCloseM);
    EXPECT_FLOAT_EQ(d.elevation_deg, web::kD1StillElevationDeg);
    EXPECT_FALSE(d.fade().on()) << "the reference quad's frames are today's";
    const web::Framing& p = web::find_drone("phantom3_class")->framing;
    EXPECT_LT(p.subject_m, d.subject_m) << "a tighter shot: the drone is bigger in the frame";
    EXPECT_GT(p.elevation_deg, d.elevation_deg) << "looking down on it: the body is not hidden behind the wake of its near rotors";
    EXPECT_LE(p.elevation_deg, 45.0f);
    EXPECT_GT(p.aim_below_m, d.aim_below_m) << "aimed below the drone, so it sits in the upper third";
    EXPECT_GT(p.wake_fade_to_below_m, p.wake_fade_from_below_m) << "the wake fades going down";
    EXPECT_NEAR(p.wake_fade_to_below_m - p.wake_fade_from_below_m, 0.2f, 0.05f) << "over about the last 0.2 m";
    EXPECT_TRUE(p.fade().on());
    EXPECT_LT(p.fade().floor_to_y, p.fade().floor_from_y) << "world y: the floor height is below the start";
    EXPECT_GT(p.side_fade_m, 0.0f) << "the box's side faces are not drawn as a cut";
    EXPECT_GT(p.wake_fade_top_to_above_m, p.wake_fade_top_from_above_m) << "the inflow haze above the rotors fades going up";
    EXPECT_GT(p.fade().ceiling_to_y, p.fade().ceiling_from_y);
    EXPECT_FALSE(web::default_drone().framing.fade().ceiling_to_y > web::default_drone().framing.fade().ceiling_from_y);
}

}  // namespace

// A DIAGNOSTIC CLIP SIZE (--clip-size WxH; the lead, 2026-11-17, a 3 s 1080p motion test of the particles): the web clips are
// 3840x2160 masters, and a clip at another size is a diagnostic that must never pass for one. It is 16:9 (the framing is fitted
// to that aspect), a whole multiple of 16 wide so the encoder takes it, and no larger than the master; its file is labelled with
// its size, so it never takes a master's name; and delivery, which checks the size, refuses it.
TEST(FlowMovieWeb, ADiagnosticClipSizeIsSixteenByNineNoLargerThanTheMasterAndNamedForItsSize) {
    EXPECT_EQ(web::clip_size_refusal(1920u, 1080u), "");
    EXPECT_EQ(web::clip_size_refusal(1280u, 720u), "");
    EXPECT_EQ(web::clip_size_refusal(web::kWidth, web::kHeight), "") << "the master's own size is allowed";
    for (const auto& [w, h] : std::vector<std::pair<uint32_t, uint32_t>>{{0u, 0u}, {1920u, 1081u}, {1000u, 563u}, {7680u, 4320u},
                                                                          {1930u, 1086u}, {640u, 480u}, {16u, 0u}}) {
        const std::string why = web::clip_size_refusal(w, h);
        EXPECT_FALSE(why.empty()) << w << "x" << h;
        EXPECT_NE(why.find("clip-size"), std::string::npos) << why;
    }
    EXPECT_EQ(web::clip_label(0u, 0u), "sim") << "no diagnostic size: the master's label";
    EXPECT_EQ(web::clip_label(1920u, 1080u), "diag-1920x1080");
    EXPECT_NE(web::web_file_name("S8", "rotor-wake-volume", web::clip_label(1920u, 1080u), "mp4"),
              web::web_file_name("S8", "rotor-wake-volume", web::clip_label(0u, 0u), "mp4"));
}

// --lines (the lead, 2026-11-17; Physics' web-lines): the rotors of a web D1 clip as ACTUATOR LINES, one per blade, in place of discs.
// The blade and its polar come from a prop part file under the assets (library/parts/*.prop.yaml, scene::line_propeller); a line run
// is another simulation, so its cache folder has its own name (a lines cache must never be read as a disc cache or the reverse).
TEST(FlowMovieWeb, ALinesRunKeepsItsOwnCacheFolderAndOnlyTheD1ClipsTakeLines) {
    EXPECT_EQ(web::lines_stem_suffix(false), "");
    EXPECT_EQ(web::lines_stem_suffix(true), "-lines");
    EXPECT_EQ(web::lines_refusal(true), "");
    const std::string why = web::lines_refusal(false);
    EXPECT_NE(why.find("--lines"), std::string::npos) << why;
    EXPECT_NE(why.find("D1"), std::string::npos) << why;
}

TEST(FlowMovieWeb, APropPartFileIsTheLinesBladeAndARefusedFileIsNamed) {
    const std::filesystem::path assets(SPADE_ASSETS_DIR);
    const auto lines = flow_movie::live_lines_from_prop_file(assets / "library/parts/uiuc_apce_10x5.prop.yaml", 8u);
    ASSERT_TRUE(lines.has_value()) << lines.error().context;
    EXPECT_EQ(lines->blade.blades, 2u);
    EXPECT_NEAR(lines->blade.radius, 0.1194f, 0.002f) << "the file's geometry is UIUC's at 9.4 in (the 10x5 is the nominal size), so R = 0.1194 m";
    EXPECT_EQ(lines->n_sub, 8u);
    EXPECT_FALSE(lines->blade.stations.empty());
    const auto other = flow_movie::live_lines_from_prop_file(assets / "library/parts/uiuc_mit_5x4.prop.yaml", 4u);
    ASSERT_TRUE(other.has_value()) << other.error().context;
    EXPECT_EQ(other->n_sub, 4u);
    EXPECT_LT(other->blade.radius, lines->blade.radius) << "the 5 inch one is smaller";
    const auto missing = flow_movie::live_lines_from_prop_file(assets / "library/parts/no_such.prop.yaml", 8u);
    ASSERT_FALSE(missing.has_value());
    EXPECT_NE(missing.error().context.find("no_such.prop.yaml"), std::string::npos) << missing.error().context;
    const auto motor = flow_movie::live_lines_from_prop_file(assets / "library/parts/iris_920kv.motor.yaml", 8u);
    ASSERT_FALSE(motor.has_value()) << "a motor is not a propeller";
    EXPECT_NE(motor.error().context.find("iris_920kv.motor.yaml"), std::string::npos) << motor.error().context;
    const auto no_sub = flow_movie::live_lines_from_prop_file(assets / "library/parts/uiuc_apce_10x5.prop.yaml", 0u);
    ASSERT_FALSE(no_sub.has_value());
    EXPECT_NE(no_sub.error().context.find("n_sub"), std::string::npos) << no_sub.error().context;
}

// CAPTIONS NAME NO BRANDS (Oversight, 2026-11-17, via the lead): the command a caption shows has --visual-mesh's and --lines' file
// arguments replaced by plain words, so neither a model's name, a propeller maker nor a path under a vendor's folder is printed; every
// other word of the command stays as it was. The source, the licence and the copyright notice belong in the clip's .md record.
TEST(FlowMovieWeb, ACaptionsCommandNamesNeitherTheVisualMeshNorThePropellerFile) {
    const std::string command =
        "flow_movie --web S8 --drone phantom3_class --lines library/parts/uiuc_apce_10x5.prop.yaml "
        "--visual-mesh C:/Users/Someone/KAT/content/meshes/drone/x500.gltf --nasa --q-look l1=0.5";
    const std::string shown = web::command_in_words(command, web::default_drone());
    for (const char* brand : {"x500", "X500", "apce", "uiuc", "UIUC", "KAT", "gltf", "Someone"}) {
        EXPECT_EQ(shown.find(brand), std::string::npos) << brand << " in: " << shown;
    }
    EXPECT_NE(shown.find("--visual-mesh open-source-quadcopter-model"), std::string::npos) << shown;
    EXPECT_NE(shown.find("--lines measured-propeller-polar"), std::string::npos) << shown;
    EXPECT_NE(shown.find("--web S8"), std::string::npos) << "the rest of the command is as given";
    EXPECT_NE(shown.find("--nasa --q-look l1=0.5"), std::string::npos) << shown;
    const std::string quoted = web::command_in_words("flow_movie --visual-mesh \"C:/a b/x500.gltf\" --nasa", web::default_drone());
    EXPECT_EQ(quoted.find("x500"), std::string::npos) << quoted;
    EXPECT_EQ(quoted.find("a b"), std::string::npos) << "a quoted path with a space is replaced whole: " << quoted;
    EXPECT_NE(quoted.find("--nasa"), std::string::npos) << quoted;
    EXPECT_EQ(web::command_in_words("flow_movie --lines", web::default_drone()), "flow_movie --lines") << "a flag with no value is left alone";
}

// --cache-dir DIR (a still redrawn from a frame someone else's run wrote, as Physics' 2 mm frame): the web run reads DIR instead of its
// own cache folder and must never simulate or write there. So it is refused unless the run is a still-only redraw of a named still
// (--web-still, --still-frame N) whose frame file is already in DIR; each missing piece is named.
TEST(FlowMovieWeb, ACacheDirIsOnlyForAStillOnlyRedrawOfAFrameThatIsThere) {
    EXPECT_EQ(web::cache_dir_refusal("", true, true, true), "") << "no --cache-dir: nothing to refuse";
    EXPECT_EQ(web::cache_dir_refusal("", false, false, false), "");
    EXPECT_EQ(web::cache_dir_refusal("D", true, true, true), "");
    for (const std::string& why : {web::cache_dir_refusal("D", false, true, true), web::cache_dir_refusal("D", true, false, true),
                                   web::cache_dir_refusal("D", true, true, false)}) {
        EXPECT_NE(why.find("--cache-dir"), std::string::npos) << why;
    }
    EXPECT_NE(web::cache_dir_refusal("D", false, true, true).find("--web-still"), std::string::npos);
    EXPECT_NE(web::cache_dir_refusal("D", true, false, true).find("--still-frame"), std::string::npos);
    EXPECT_NE(web::cache_dir_refusal("D", true, true, false).find("D"), std::string::npos) << "the folder is named";
    EXPECT_NE(web::cache_dir_refusal("D", true, true, false).find("never simulates"), std::string::npos);
}

// --loop-blend S (Oversight via the lead, 2026-11-17: the site loops each clip and the flow does not repeat, so S8 jumped every 10 s):
// the clip renders E = round(S x 30) frames PAST its end; its first E frames are a crossfade from the continuation (frame N + j) to
// the clip's own start (frame j), so the last frame N - 1 runs into a head that begins as frame N would have, and ends as frame E, the
// frame after the blend. The weight of the start in frame j is a smoothstep of (j + 1) / (E + 1): strictly between 0 and 1, rising,
// 0.5 in the middle. The caption names the method.
TEST(FlowMovieWeb, ALoopBlendIsAWholeNumberOfFramesLessThanHalfTheClipAndRendersThatManyExtra) {
    EXPECT_EQ(web::loop_blend_frames(1.0, 300u), 30u);
    EXPECT_EQ(web::loop_blend_frames(0.5, 300u), 15u);
    EXPECT_EQ(web::loop_blend_frames(0.0, 300u), 0u) << "off";
    EXPECT_EQ(web::loop_blend_refusal(1.0, 300u), "");
    EXPECT_EQ(web::loop_blend_refusal(0.0, 300u), "");
    for (const auto& [seconds, frames] : std::vector<std::pair<double, uint32_t>>{{-1.0, 300u}, {5.1, 300u}, {10.0, 300u}, {0.01, 300u}, {1.0, 40u}}) {
        const std::string why = web::loop_blend_refusal(seconds, frames);
        EXPECT_NE(why.find("--loop-blend"), std::string::npos) << seconds << " of " << frames << ": " << why;
    }
    EXPECT_EQ(web::loop_blend_total_frames(300u, 30u), 330u) << "the clip's frames and the blend's extra";
    EXPECT_EQ(web::loop_blend_total_frames(300u, 0u), 300u);
}

TEST(FlowMovieWeb, TheLoopBlendWeightRisesStrictlyFromNearNothingToNearOneAndIsHalfInTheMiddle) {
    const uint32_t e = 30;
    float previous = 0.0f;
    for (uint32_t j = 0; j < e; ++j) {
        const float w = web::loop_blend_weight(j, e);
        EXPECT_GT(w, 0.0f) << j;
        EXPECT_LT(w, 1.0f) << j;
        EXPECT_GT(w, previous) << "strictly rising at " << j;
        previous = w;
    }
    EXPECT_LT(web::loop_blend_weight(0, e), 0.01f) << "the first blended frame is almost the continuation";
    EXPECT_GT(web::loop_blend_weight(e - 1, e), 0.99f) << "the last is almost the start";
    EXPECT_NEAR(web::loop_blend_weight(14, 29), 0.5f, 0.02f) << "the middle of an odd count";
    EXPECT_NEAR(web::loop_blend_weight(0, 1), 0.5f, 1e-6f) << "one frame: halfway";
}

TEST(FlowMovieWeb, ABlendedPixelIsTheWeightedMixRoundedAndWeightsOfZeroAndOneKeepTheEnds) {
    const std::vector<uint8_t> tail = {0, 100, 200, 255, 10, 20, 30, 255};   // two RGBA pixels
    const std::vector<uint8_t> head = {100, 200, 0, 255, 250, 20, 30, 255};
    auto mixed = web::blend_rgba(tail, head, 0.25f);
    ASSERT_EQ(mixed.size(), tail.size());
    EXPECT_EQ(mixed[0], 25) << "0.75 x 0 + 0.25 x 100";
    EXPECT_EQ(mixed[1], 125);
    EXPECT_EQ(mixed[2], 150);
    EXPECT_EQ(mixed[3], 255) << "alpha is kept";
    EXPECT_EQ(mixed[4], 70);
    EXPECT_EQ(web::blend_rgba(tail, head, 0.0f), tail);
    EXPECT_EQ(web::blend_rgba(tail, head, 1.0f), head);
    EXPECT_TRUE(web::blend_rgba(tail, std::vector<uint8_t>{1, 2, 3}, 0.5f).empty()) << "frames of different sizes are not blended";
}

TEST(FlowMovieWeb, ALoopBlendedCaptionNamesTheMethod) {
    EXPECT_EQ(web::loop_blend_caption(0u), "");
    const std::string said = web::loop_blend_caption(30u);
    EXPECT_NE(said.find("crossfade"), std::string::npos) << said;
    EXPECT_NE(said.find("1 s"), std::string::npos) << said;
}

// THE OPEN-SOURCE QUADCOPTER (the lead, 2026-11-18, L86k/L88l): the website clips' drone is the X500's flow model (Physics' tool data,
// tools/flow_movie/data/x500, not the default library), named "x500" on the command line and in no caption: the clip says "an open-source
// quadcopter". It is a 13-inch-rotor drone, so its shot is wider than the camera quadcopter's; its model file is reachable from the assets
// directory like the library's; and the parts-build options (--build-parts, --part-store) are words in a caption, never paths.
TEST(FlowMovieWeb, TheOpenSourceQuadcopterIsADroneOfTheClipsWhoseWordsNameNoBrand) {
    const web::Drone* drone = web::find_drone("x500");
    ASSERT_NE(drone, nullptr);
    EXPECT_EQ(web::find_drone("open-source-quadcopter"), drone) << "the caption's alias names the same drone";
    EXPECT_EQ(web::drone_stem_suffix(*drone), "-x500");
    for (const std::string& words : {std::string(drone->alias), std::string(drone->title_noun), std::string(drone->the_noun),
                                     std::string(drone->description)}) {
        for (const char* brand : {"x500", "X500", "PX4", "px4", "Holybro", "holybro", "KAT", "Pixhawk", "UIUC", "APC"}) {
            EXPECT_EQ(words.find(brand), std::string::npos) << "'" << words << "' carries " << brand;
        }
    }
    EXPECT_NE(std::string(drone->title_noun).find("quadcopter"), std::string::npos);
    EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(SPADE_ASSETS_DIR) / drone->model_file))
        << "the model file is found from the assets directory: " << drone->model_file;
    const web::Framing& x = drone->framing;
    const web::Framing& p = web::find_drone("phantom3_class")->framing;
    EXPECT_GT(x.subject_m, p.subject_m) << "13-inch rotors reach 0.35 m from the axis: a wider shot";
    EXPECT_GT(x.push_close_m, p.push_close_m);
    EXPECT_GT(x.push_wide_m, x.push_close_m);
    EXPECT_LE(x.elevation_deg, 45.0f);
}

TEST(FlowMovieWeb, ACaptionsCommandNamesNeitherThePartsTableNorThePartStore) {
    const web::Drone* found = web::find_drone("x500");
    ASSERT_NE(found, nullptr);
    const web::Drone& drone = *found;
    const std::string said = web::command_in_words(
        "flow_movie --web S8 --drone x500 --build-parts C:/Users/someone/Desktop/Projects/KAT/x500_build/parts.yaml --part-store "
        "\"C:/Users/someone/Desktop/Projects/part-store\" --nasa-cloud",
        drone);
    EXPECT_EQ(said.find("x500"), std::string::npos) << said;
    EXPECT_EQ(said.find("KAT"), std::string::npos) << said;
    EXPECT_EQ(said.find("part-store\""), std::string::npos) << said;
    EXPECT_EQ(said.find("someone"), std::string::npos) << said;
    EXPECT_NE(said.find("--drone open-source-quadcopter"), std::string::npos) << said;
    EXPECT_NE(said.find("--build-parts open-source-quadcopter-parts-table"), std::string::npos) << said;
    EXPECT_NE(said.find("--part-store part-store"), std::string::npos) << said;
    EXPECT_NE(said.find("--nasa-cloud"), std::string::npos) << "the rest of the command is kept: " << said;
    EXPECT_EQ(web::command_in_words("flow_movie --web S8 --build-parts", drone), "flow_movie --web S8 --build-parts") << "no value: left alone";
}

// THE SEAM (the lead, 2026-11-18, after the cloud thinning was set on the plan's copy of the look while the film's look state read the
// options' copy, and never reached a frame): a web run's look is made ONCE, by web_look(), and main assigns that one value to both
// opt.look (the film's look state) and plan.look (the film's setup). So what a look option says on the command line is what a frame is
// drawn with, for every field of Look: web_look keeps every one the options set (compared whole, so a field added later is covered
// without editing this test), adds the drone's wake fade only where it applies, and adds the cloud's thinning round the held body
// only when the nasa look asks for it.
TEST(FlowMovieWeb, AWebRunsLookKeepsEveryOptionTheCommandLineSetAndAddsOnlyTheFadeAndTheThinning) {
    const web::Drone& drone = *web::find_drone("x500");
    flow_movie::Look set;
    set.l2_level = 3.5f;
    set.l2_smooth = true;
    set.min_cells = 7u;
    set.mean_frames = 4u;
    ASSERT_TRUE(set.any());

    // nothing applies: the options' look, whole
    EXPECT_EQ(web::web_look(set, drone, false, nullptr), set);

    // the fade applies (S2 and S8 without a particle or nasa look): the drone's fade and nothing else changes
    flow_movie::Look faded = set;
    faded.fade = drone.framing.fade();
    ASSERT_TRUE(faded.fade.on());
    EXPECT_EQ(web::web_look(set, drone, true, nullptr), faded);

    // the nasa look with a thinning: the held body's centre (the drone's own height), the radius and the floor; the rest as set
    // thin_r is a multiple of the drone's own reach (Drone::reach_m, the farthest point of its rotors from the axis): the X500 reaches
    // 0.37 m and the camera quadcopter 0.30, so the same 0.74 thins a larger ball (the lead, 2026-11-18)
    flow_movie::QLook q;
    q.thin_r = 0.74f;
    q.thin_floor = 0.05f;
    flow_movie::Look thinned = set;
    thinned.thin.centre = glm::vec3(0.0f, drone.body_y_m, 0.0f);
    thinned.thin.radius = 0.74f * drone.reach_m;
    thinned.thin.floor = 0.05f;
    EXPECT_EQ(web::web_look(set, drone, false, &q), thinned);
    EXPECT_NEAR(web::web_look(set, drone, false, &q).thin.centre.y, 0.014f, 1e-7f) << "the X500's body is 0.014 m up in its box";
    const web::Drone& small = *web::find_drone("phantom3_class");
    EXPECT_NEAR(web::web_look(set, small, false, &q).thin.radius, 0.74f * small.reach_m, 1e-7f);
    EXPECT_GT(web::web_look(set, drone, false, &q).thin.radius, 1.2f * web::web_look(set, small, false, &q).thin.radius)
        << "the bigger drone's thinned ball is bigger";

    // a nasa look without a thinning leaves the thinning off
    const flow_movie::QLook none;
    EXPECT_EQ(web::web_look(set, drone, false, &none), set);
    EXPECT_FALSE(web::web_look(set, drone, false, &none).thin.on());

    // an option's own fade is not undone by a run that adds none
    flow_movie::Look own = set;
    own.fade = drone.framing.fade();
    EXPECT_EQ(web::web_look(own, drone, false, nullptr), own);
}

// THE WAKE FADE IN THE CLOUD LOOK (the lead, 2026-11-18: the flat cut at the box's top and sides): the soft edge of a drawn wake is for S2
// and S8, and for the nasa CLOUD look (a soft volume); not for S1, the particle looks, or the nasa Q SURFACES (a faded surface vanishes).
TEST(FlowMovieWeb, TheWakeFadeAppliesToTheCloudLookAndNotToTheQSurfacesOrTheParticles) {
    EXPECT_TRUE(web::web_fade_applies("S8", false, false, false, false)) << "the plain S8";
    EXPECT_TRUE(web::web_fade_applies("S2", false, false, false, false));
    EXPECT_TRUE(web::web_fade_applies("S8", false, false, true, true)) << "the nasa cloud";
    EXPECT_FALSE(web::web_fade_applies("S8", false, false, true, false)) << "the nasa Q surfaces";
    EXPECT_FALSE(web::web_fade_applies("S8", false, true, false, false)) << "the particles";
    EXPECT_FALSE(web::web_fade_applies("S8", true, false, false, false)) << "the particle preview";
    EXPECT_FALSE(web::web_fade_applies("S1", false, false, false, false));
    EXPECT_FALSE(web::web_fade_applies("S1", false, false, true, true));
}

// THE X500's SHOT (the lead, 2026-11-18): the drone fills about a third of the height, looked down on at about 20 degrees, the wake's bell in frame.
TEST(FlowMovieWeb, TheOpenSourceQuadcoptersShotIsTightAndLooksDownAboutTwentyDegrees) {
    const web::Framing& x = web::find_drone("x500")->framing;
    EXPECT_NEAR(x.elevation_deg, 20.0f, 0.5f);
    EXPECT_GE(x.subject_m, 0.36f);
    EXPECT_LE(x.subject_m, 0.46f);
    EXPECT_GT(x.aim_below_m, 0.0f);
    EXPECT_GT(x.wake_fade_to_below_m, x.wake_fade_from_below_m) << "the wake fades going down, above the box's floor";
    EXPECT_LT(x.wake_fade_to_below_m, 0.31f) << "the X500's box floor is 0.3135 m below the origin";
}

// A drone's REACH (the thinning radius is a multiple of it): the farthest point of its rotors from the vertical axis through its centre,
// a rotor's hub distance plus its radius; the model's own proxy radius for the camera quadcopter (0.2963 m), the X500's 10 inch rotors
// (R 0.127 m) at 0.174 m on each axis: 0.2461 + 0.127.
TEST(FlowMovieWeb, ADronesReachIsItsFarthestRotorTipFromTheAxis) {
    EXPECT_NEAR(web::find_drone("phantom3_class")->reach_m, 0.2963f, 2e-3f);
    EXPECT_NEAR(web::find_drone("x500")->reach_m, 0.373f, 2e-3f);
    EXPECT_GT(web::find_drone("x500")->reach_m, web::find_drone("phantom3_class")->reach_m);
}

// THE CAMERA'S ELEVATION (--elevation DEG, the web stills): the X500 shot looks down 20 degrees; a close-up of the boards on the top plate
// (which way the controller, the computer and the GPS face) needs a higher camera. One number, whole, above 0 and at most 85 (a camera
// straight overhead has no "up" for the orbit); nothing else is a camera.
TEST(FlowMovieWeb, TheCameraElevationOptionTakesOneNumberAboveZeroAndAtMost85) {
    ASSERT_TRUE(web::parse_elevation_deg("20").has_value());
    EXPECT_FLOAT_EQ(*web::parse_elevation_deg("20"), 20.0f);
    EXPECT_FLOAT_EQ(*web::parse_elevation_deg("62.5"), 62.5f);
    EXPECT_FLOAT_EQ(*web::parse_elevation_deg("85"), 85.0f);
    for (const char* bad : {"", "0", "-5", "86", "90", "1e1", "20deg", " 20", "twenty", "."}) {
        EXPECT_FALSE(web::parse_elevation_deg(bad).has_value()) << "'" << bad << "'";
    }
}

// WHERE WORKING OUTPUT GOES BY DEFAULT (Cameron, O74, 2026-11-18; moved by O84, 2026-11-19): all videos live under Desktop/Vantage/Videos, and Spade's
// footage under Footage/Spade, one dated folder a day. Desktop/Projects/Videos no longer exists, so writing there would make a stray. Anything posted or
// kept goes wherever --out names; this is only the default.
TEST(FlowMovieWeb, TheDefaultOutputFolderIsTheSpadeFootageFolderOfTheVideosArchiveByDate) {
    EXPECT_EQ(web::default_out_dir("C:/Users/Someone", "2026-11-18"),
              std::filesystem::path("C:/Users/Someone") / "Desktop" / "Vantage" / "Videos" / "Footage" / "Spade" / "2026-11-18");
    const std::string text = web::default_out_dir("C:/Users/Someone", "2026-11-18").generic_string();
    EXPECT_EQ(text.find("spade-videos"), std::string::npos) << "the old Desktop folder is not the default any more: " << text;
    EXPECT_EQ(text.find("Projects"), std::string::npos) << "Desktop/Projects/Videos is gone: " << text;
    EXPECT_EQ(web::default_out_dir("", "2026-11-18"), std::filesystem::path(".") / "Desktop" / "Vantage" / "Videos" / "Footage" / "Spade" / "2026-11-18")
        << "no profile: relative to the working directory, as before";
}

