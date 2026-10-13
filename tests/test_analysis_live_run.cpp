// LiveRun, the drone-box airflow run (I1; the harness design,
// docs/design/interface/plans/2026-10-13-i1-harness-design.md): the one
// stepping core behind D1's recording, D2's panel, the queued runner and
// Kat's host. This first slice: the grid, the held drone, time in whole
// nanoseconds, frames at k/30 s, determinism and the refusals. The rotors
// join with Core's throttle command and Physics' P7a; the solids with P2.

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/flow_volume.hpp"
#include "scene/scene_file.hpp"
#include "vehicles/model_identity.hpp"

namespace {

namespace an = spade::analysis;
namespace fs = std::filesystem;

// The scene's quad without its parts: the solids join with P2, and a 5 cm
// test grid is far too coarse for them.
[[nodiscard]] spade::vehicles::ModelType quad() {
    const auto s = spade::scene::load_scene_file(fs::path(SPADE_ASSETS_DIR) / "scenes" / "hover.scene.yaml");
    EXPECT_TRUE(s.has_value());
    spade::vehicles::ModelType m = s ? s->models.at(0) : spade::vehicles::ModelType{};
    m.parts.clear();
    return m;
}

// An 8^3 box of 0.8 m (10 cm cells), stepped every 4 ms: small enough for
// debug ctest's budget.
[[nodiscard]] an::AnalysisCase tiny_case(const spade::vehicles::ModelType& m) {
    an::AnalysisCase c;
    c.model = an::ModelRef{"quad.model.yaml", spade::vehicles::model_identity(m), spade::vehicles::parts_identity(m)};
    c.rotors.assign(m.rotors.size(), an::RotorSetup{});
    c.grid.box = 0.8f;
    c.grid.spacing = 0.1f;
    c.time.duration = 1.0f;
    c.time.step = 0.004f;
    return c;
}

[[nodiscard]] an::ResolvedCase resolved(const an::AnalysisCase& c, const spade::vehicles::ModelType& m) {
    const auto r = an::resolve(c, m);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().front().field + ": " + r.error().front().message);
    return r ? *r : an::ResolvedCase{};
}

[[nodiscard]] an::LiveRun started(const an::AnalysisCase& c, const spade::vehicles::ModelType& m) {
    spade::Result<an::LiveRun> run = an::LiveRun::start(resolved(c, m), m);
    EXPECT_TRUE(run.has_value()) << (run ? "" : run.error().context);
    return std::move(*run);
}

// The design frame's attitude for yaw, then pitch, then roll (degrees), as
// the case's Controls state it: yaw about +Y, pitch about +Z, roll about +X.
[[nodiscard]] glm::quat attitude(float yaw, float pitch, float roll) {
    const float r = 3.14159265358979f / 180.0f;
    return glm::angleAxis(yaw * r, glm::vec3(0, 1, 0)) * glm::angleAxis(pitch * r, glm::vec3(0, 0, 1)) *
           glm::angleAxis(roll * r, glm::vec3(1, 0, 0));
}

[[nodiscard]] float quat_gap(const glm::quat& a, const glm::quat& b) {
    const float d = glm::abs(glm::dot(a, b));
    return 1.0f - d;  // 0 when they are the same rotation
}

}  // namespace

TEST(AnalysisLiveRun, StartsHeldAtRestAtTimeZero) {
    const spade::vehicles::ModelType m = quad();
    const an::LiveRun run = started(tiny_case(m), m);
    EXPECT_EQ(run.state().t_s, 0.0);
    EXPECT_EQ(run.state().steps, 0u);
    EXPECT_EQ(run.state().frames, 0u);
    EXPECT_EQ(run.step_s(), 0.004);
    EXPECT_FALSE(run.state().armed);
    ASSERT_EQ(run.state().rotor_omega.size(), m.rotors.size());
    for (const float w : run.state().rotor_omega) EXPECT_EQ(w, 0.0f);
    EXPECT_LT(quat_gap(run.state().attitude, glm::quat(1, 0, 0, 0)), 1e-6f);
    const std::string line = run.run_line();
    EXPECT_NE(line.find("actuator disc"), std::string::npos) << line;
    EXPECT_NE(line.find("8\xc2\xb3"), std::string::npos) << line;  // 8, superscript three
    EXPECT_NE(line.find("dx 100 mm"), std::string::npos) << line;
    EXPECT_NE(line.find("t = 0.000 s"), std::string::npos) << line;
}

// Until Physics' step rule lands, a live run needs the case's own step.
TEST(AnalysisLiveRun, RefusesACaseWithoutAStep) {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase c = tiny_case(m);
    c.time.step.reset();
    const spade::Result<an::LiveRun> run = an::LiveRun::start(resolved(c, m), m);
    ASSERT_FALSE(run.has_value());
    EXPECT_NE(run.error().context.find("time.step"), std::string::npos) << run.error().context;
}

// Whole nanoseconds: 250 steps of 4 ms are exactly 1 s.
TEST(AnalysisLiveRun, CountsTimeInWholeSteps) {
    const spade::vehicles::ModelType m = quad();
    an::LiveRun run = started(tiny_case(m), m);
    for (int i = 0; i < 250; ++i) ASSERT_TRUE(run.step(an::Controls{}).has_value());
    EXPECT_EQ(run.state().steps, 250u);
    EXPECT_EQ(run.state().t_s, 1.0);
}

// A frame ends at the first step at or past k/30 s: with 4 ms steps, frame 1
// at step 9 (36 ms, past 33.3 ms) and frame 2 at step 17 (68 ms, past 66.7 ms).
TEST(AnalysisLiveRun, AFrameEndsAtTheFirstStepAtOrPastItsTime) {
    const spade::vehicles::ModelType m = quad();
    an::LiveRun run = started(tiny_case(m), m);
    spade::FlowVolume volume;
    const spade::Result<an::LiveFrame> first = run.advance_frame(an::Controls{}, volume);
    ASSERT_TRUE(first.has_value()) << first.error().context;
    EXPECT_EQ(first->frame, 1u);
    EXPECT_EQ(first->steps, 9u);
    EXPECT_DOUBLE_EQ(first->t_s, 0.036);
    EXPECT_EQ(volume.header.nx, 8u);
    EXPECT_EQ(volume.header.ny, 8u);
    EXPECT_EQ(volume.header.nz, 8u);
    // The box is centred on the drone: its minimum corner is -0.4 m.
    EXPECT_FLOAT_EQ(volume.header.origin.x, -0.4f);
    const spade::Result<an::LiveFrame> second = run.advance_frame(an::Controls{}, volume);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->frame, 2u);
    EXPECT_EQ(second->steps, 17u);
    EXPECT_EQ(run.state().frames, 2u);
}

// The hold turns the drone toward the controls' attitude at kHoldRateDegPerS,
// and its rate is the hold's omega_body.
TEST(AnalysisLiveRun, TheHoldTurnsAtItsRate) {
    const spade::vehicles::ModelType m = quad();
    an::LiveRun run = started(tiny_case(m), m);
    an::Controls pitch{};
    pitch.pitch_deg = 90.0f;
    ASSERT_TRUE(run.step(pitch).has_value());
    const float first = an::kHoldRateDegPerS * 0.004f;  // 0.72 degrees
    EXPECT_LT(quat_gap(run.state().attitude, attitude(0, first, 0)), 1e-6f);
    // Pitch is about +Z: pi rad/s while turning.
    EXPECT_NEAR(run.state().omega_body.z, an::kHoldRateDegPerS * 3.14159265f / 180.0f, 1e-4f);
    EXPECT_NEAR(run.state().omega_body.x, 0.0f, 1e-6f);
    for (int i = 0; i < 200; ++i) ASSERT_TRUE(run.step(pitch).has_value());  // 0.8 s: there
    EXPECT_LT(quat_gap(run.state().attitude, attitude(0, 90, 0)), 1e-6f);
    EXPECT_NEAR(glm::length(run.state().omega_body), 0.0f, 1e-6f);
    // Yaw, pitch and roll together reach the controls' attitude.
    const an::Controls all{false, 0.0f, 10.0f, -20.0f, 30.0f};
    for (int i = 0; i < 300; ++i) ASSERT_TRUE(run.step(all).has_value());
    EXPECT_LT(quat_gap(run.state().attitude, attitude(30, 10, -20)), 1e-6f);
}

// Wind blows through the box; still air stays still with the rotors stopped.
TEST(AnalysisLiveRun, TheWindBlowsThroughTheBox) {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase windy = tiny_case(m);
    windy.flight.kind = an::FlightKind::forward;
    windy.flight.speed = 3.0f;
    an::LiveRun run = started(windy, m);
    spade::FlowVolume volume;
    ASSERT_TRUE(run.advance_frame(an::Controls{}, volume).has_value());
    EXPECT_LT(volume.header.channel_min[spade::kFlowU], -1.0f) << "the air moves along -x at 3 m/s";
    an::LiveRun still = started(tiny_case(m), m);
    ASSERT_TRUE(still.advance_frame(an::Controls{}, volume).has_value());
    EXPECT_EQ(volume.header.channel_max[spade::kFlowU], 0.0f);
    EXPECT_EQ(volume.header.channel_min[spade::kFlowU], 0.0f);
}

// The same case and controls give the same bits.
TEST(AnalysisLiveRun, IsDeterministic) {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase windy = tiny_case(m);
    windy.flight.kind = an::FlightKind::forward;
    windy.flight.speed = 3.0f;
    windy.flight.heading_deg = 20.0f;
    an::LiveRun a = started(windy, m);
    an::LiveRun b = started(windy, m);
    spade::FlowVolume va;
    spade::FlowVolume vb;
    const an::Controls turn{false, 0.0f, 5.0f, 5.0f, 5.0f};
    for (int i = 0; i < 2; ++i) {
        ASSERT_TRUE(a.advance_frame(turn, va).has_value());
        ASSERT_TRUE(b.advance_frame(turn, vb).has_value());
    }
    EXPECT_EQ(va.v, vb.v);
    EXPECT_EQ(va.s, vb.s);
    EXPECT_EQ(a.state().attitude, b.state().attitude);
}

// What it flew is a control program: a point wherever the controls changed.
TEST(AnalysisLiveRun, RecordsTheControlsItFlew) {
    const spade::vehicles::ModelType m = quad();
    an::LiveRun run = started(tiny_case(m), m);
    const an::Controls armed{true, 0.5f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 5; ++i) ASSERT_TRUE(run.step(an::Controls{}).has_value());
    for (int i = 0; i < 5; ++i) ASSERT_TRUE(run.step(armed).has_value());
    const an::ControlProgram& flown = run.recorded_controls();
    ASSERT_EQ(flown.points.size(), 2u);
    EXPECT_EQ(flown.points[0].t_s, 0.0f);
    EXPECT_FALSE(flown.points[0].controls.armed);
    EXPECT_FLOAT_EQ(flown.points[1].t_s, 0.02f);  // the controls in force from the 6th step
    EXPECT_TRUE(flown.points[1].controls.armed);
    EXPECT_TRUE(an::program_issues(flown).empty());
}
