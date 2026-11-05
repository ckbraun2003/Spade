// LiveRun, the drone-box airflow run (I1; the harness design,
// docs/design/interface/plans/2026-10-13-i1-harness-design.md): the one
// stepping core behind D1's recording, D2's panel, the queued runner and
// Kat's host. This first slice: the grid, the held drone, time in whole
// nanoseconds, frames at k/30 s, determinism and the refusals; then the
// rotors, through Core's throttle map (armed_duty, rotor_omega_for_duty) and
// Physics' P7a discs. The solids join with P2.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <format>
#include <string>
#include <tuple>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/flow_volume.hpp"
#include "physics/airflow/pcg.hpp"
#include "scene/model_file.hpp"
#include "scene/scene_file.hpp"
#include "sim/design_frame.hpp"
#include "state/layout.hpp"
#include "analysis_powered_quad.hpp"
#include "unique_temp.hpp"
#include "vehicles/airflow_solids.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_part.hpp"
#include "world/sdf.hpp"
#include "vehicles/propulsion_flags.hpp"

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

using Powered = an::test::PoweredQuad;
[[nodiscard]] Powered powered() { return an::test::powered_quad(); }
[[nodiscard]] float commanded(const Powered& p, float throttle, double density) {
    return an::test::commanded_omega(p, throttle, density);
}

// The quad without its rotors: a body alone, for the tests of the hold, the
// time and the frames.
[[nodiscard]] spade::vehicles::ModelType rotorless() {
    spade::vehicles::ModelType m = quad();
    m.rotors.clear();
    return m;
}

// An 8^3 box of 0.8 m (10 cm cells), stepped every 4 ms: small enough for
// debug ctest's budget.
[[nodiscard]] an::AnalysisCase tiny_case(const spade::vehicles::ModelType& m) {
    an::AnalysisCase c;
    c.model = an::pin_model("quad.model.yaml", m);
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

[[nodiscard]] an::LiveRun started(const an::AnalysisCase& c, const Powered& p) {
    spade::Result<an::LiveRun> run = an::LiveRun::start(resolved(an::test::with_power(c, p), p.model), p.model);
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
    const Powered p = powered();
    const spade::vehicles::ModelType& m = p.model;
    const an::LiveRun run = started(tiny_case(m), p);
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
    const spade::vehicles::ModelType m = rotorless();
    an::LiveRun run = started(tiny_case(m), m);
    for (int i = 0; i < 250; ++i) ASSERT_TRUE(run.step(an::Controls{}).has_value());
    EXPECT_EQ(run.state().steps, 250u);
    EXPECT_EQ(run.state().t_s, 1.0);
}

// A model with rotors needs its propulsion chain: without it, no throttle
// turns a rotor, and a run would record still air under a drone that should
// be thrusting (L6).
TEST(AnalysisLiveRun, ARotorModelNeedsItsPropulsion) {
    const spade::vehicles::ModelType m = quad();
    const spade::Result<an::LiveRun> run = an::LiveRun::start(resolved(tiny_case(m), m), m);
    ASSERT_FALSE(run.has_value());
    EXPECT_EQ(run.error().code, spade::Code::invalid_argument);
    EXPECT_NE(run.error().context.find("propulsion"), std::string::npos) << run.error().context;
}

// The chain is the model's, the idle and the state of charge the case's
// (INT-11; a bad one is the case's to refuse, ThePowerIsCheckedByName):
// armed at zero throttle, every rotor settles at the speed the case's idle
// commands, and another idle is another speed.
TEST(AnalysisLiveRun, ItsPowerIsTheCases) {
    float last = 0.0f;
    for (const float idle : {0.05f, 0.12f}) {
        Powered p = powered();
        p.power.idle = idle;
        const an::AnalysisCase c = tiny_case(p.model);
        an::LiveRun run = started(c, p);
        const float want = commanded(p, 0.0f, c.fluid.density);
        ASSERT_GT(want, last);
        for (int i = 0; i < 250; ++i) ASSERT_TRUE(run.step(an::Controls{true, 0.0f, 0, 0, 0}).has_value());  // 1 s
        for (const float w : run.state().rotor_omega) EXPECT_NEAR(w, want, 0.005f * want) << "idle " << idle;
        last = want;
    }
}

// Disarmed, the rotors stay still. Armed, each spins up to the speed the
// chain commands for the throttle (idle + (1 - idle) throttle, through the
// chain at the case's density), lagging on its fitted tau. Disarmed again,
// each spins down: disarm is a command of 0, never a disabled rotor.
TEST(AnalysisLiveRun, ArmedRotorsSpinToTheChainsCommandAndDisarmedSpinDown) {
    const Powered p = powered();
    const an::AnalysisCase c = tiny_case(p.model);
    an::LiveRun run = started(c, p);
    for (int i = 0; i < 25; ++i) ASSERT_TRUE(run.step(an::Controls{false, 0.5f, 0, 0, 0}).has_value());
    for (const float w : run.state().rotor_omega) EXPECT_EQ(w, 0.0f) << "a disarmed rotor turned";
    EXPECT_FALSE(run.state().armed);

    const float want = commanded(p, 0.5f, c.fluid.density);
    ASSERT_GT(want, 100.0f);
    for (int i = 0; i < 250; ++i) ASSERT_TRUE(run.step(an::Controls{true, 0.5f, 0, 0, 0}).has_value());  // 1 s
    EXPECT_TRUE(run.state().armed);
    for (const float w : run.state().rotor_omega) EXPECT_NEAR(w, want, 0.005f * want);

    for (int i = 0; i < 250; ++i) ASSERT_TRUE(run.step(an::Controls{false, 0.5f, 0, 0, 0}).has_value());
    for (const float w : run.state().rotor_omega) EXPECT_LT(w, 0.01f * want) << "a disarmed rotor kept turning";
}

// The rotors' rows after a step, for a host's probes (Test/Docs'
// probe_rotor(), the tour's on-screen downwash check): one per model rotor,
// in order, each turning as the state says, its hub inside the published box.
// The body is held at the box's centre, the published volume's frame, so a
// probe at body() + row agrees with the volume.
TEST(AnalysisLiveRun, ItsRotorRowsAreTheRotorsAfterTheStep) {
    const Powered p = powered();
    const an::AnalysisCase c = tiny_case(p.model);
    an::LiveRun run = started(c, p);
    EXPECT_EQ(run.rotor_rows().size(), p.model.rotors.size());
    for (int i = 0; i < 50; ++i) ASSERT_TRUE(run.step(an::Controls{true, 0.5f, 0, 0, 0}).has_value());
    const auto rows = run.rotor_rows();
    ASSERT_EQ(rows.size(), run.state().rotor_omega.size());
    const float half = 0.5f * *c.grid.box;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        EXPECT_EQ(rows[i].omega, run.state().rotor_omega[i]) << "rotor " << i;
        EXPECT_GT(rows[i].omega, 0.0f) << "rotor " << i;
        EXPECT_GT(rows[i].radius, 0.0f) << "rotor " << i;
        const glm::vec3 hub = run.body().pos + run.body().orient * rows[i].local_pos;
        for (int k = 0; k < 3; ++k) {
            EXPECT_LT(std::abs(hub[k]), half) << "rotor " << i << "'s hub is outside the published box";
        }
    }
}

// The discs push the air down, through the box, from the held drone's rotors;
// a disarmed drone leaves it still.
TEST(AnalysisLiveRun, ArmedRotorsPushTheAirDown) {
    const Powered p = powered();
    const an::AnalysisCase c = tiny_case(p.model);
    an::LiveRun run = started(c, p);
    spade::FlowVolume volume;
    for (int f = 0; f < 6; ++f) ASSERT_TRUE(run.advance_frame(an::Controls{true, 0.5f, 0, 0, 0}, volume).has_value());
    EXPECT_LT(volume.header.channel_min[spade::kFlowV], -0.05f) << "no downwash after 0.2 s at half throttle";
    an::LiveRun still = started(c, p);
    for (int f = 0; f < 6; ++f) {
        ASSERT_TRUE(still.advance_frame(an::Controls{false, 0.5f, 0, 0, 0}, volume).has_value());
    }
    EXPECT_EQ(volume.header.channel_min[spade::kFlowV], 0.0f);
    EXPECT_EQ(volume.header.channel_max[spade::kFlowV], 0.0f);
}

// A limit is an operating point the chain really settles at: the step goes
// on, and the limit is announced in the state and on the run's label.
TEST(AnalysisLiveRun, ALimitIsAnnouncedNotRefused) {
    Powered p = powered();
    p.model.propulsion->current_max = 4.0;  // a motor that can't take full throttle
    an::LiveRun run = started(tiny_case(p.model), p);
    EXPECT_EQ(run.run_label().find("current limit"), std::string::npos) << run.run_label();
    for (int i = 0; i < 25; ++i) ASSERT_TRUE(run.step(an::Controls{true, 1.0f, 0, 0, 0}).has_value());
    EXPECT_NE(run.state().propulsion_flags & spade::vehicles::propulsion_flags::current_limited, 0u);
    EXPECT_NE(run.run_label().find("motors at their current limit"), std::string::npos) << run.run_label();
    EXPECT_NE(run.run_line().find("motors at their current limit"), std::string::npos) << run.run_line();
}

// The run line is the label, then the time.
TEST(AnalysisLiveRun, TheRunLineIsItsLabelThenTheTime) {
    const Powered p = powered();
    an::LiveRun run = started(tiny_case(p.model), p);
    const std::string dot = "\xc2\xb7";  // a middle dot
    EXPECT_EQ(run.run_line(), run.run_label() + " " + dot + " t = 0.000 s");
    for (int i = 0; i < 25; ++i) ASSERT_TRUE(run.step(an::Controls{true, 0.5f, 0, 0, 0}).has_value());
    EXPECT_EQ(run.run_line(), run.run_label() + " " + dot + " t = 0.100 s");
}

// A throttle the case can't hold is refused by name, and the run stays put.
TEST(AnalysisLiveRun, RefusesAThrottleOutOfRange) {
    const Powered p = powered();
    an::LiveRun run = started(tiny_case(p.model), p);
    const spade::Result<void> step = run.step(an::Controls{true, 1.5f, 0, 0, 0});
    ASSERT_FALSE(step.has_value());
    EXPECT_NE(step.error().context.find("throttle"), std::string::npos) << step.error().context;
    EXPECT_EQ(run.state().steps, 0u);
}

// A frame ends at the first step at or past k/30 s: with 4 ms steps, frame 1
// at step 9 (36 ms, past 33.3 ms) and frame 2 at step 17 (68 ms, past 66.7 ms).
TEST(AnalysisLiveRun, AFrameEndsAtTheFirstStepAtOrPastItsTime) {
    const spade::vehicles::ModelType m = rotorless();
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
    const spade::vehicles::ModelType m = rotorless();
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

// The held body row, as Core's posed_parts() takes it, for drawing the solid
// drone (Rendering's bind_parts): through the model's design frame it is the
// hold itself, at the box's centre, at rest but for the turn.
TEST(AnalysisLiveRun, ExposesTheHeldBody) {
    const spade::vehicles::ModelType m = rotorless();
    an::LiveRun run = started(tiny_case(m), m);
    const auto design = [&] {
        const spade::BodyState& b = run.body();
        return spade::to_design_state(spade::FrameState{b.pos, b.orient, b.vel, b.omega_body},
                                      glm::normalize(m.design_to_principal), m.com_offset);
    };
    EXPECT_LT(glm::length(design().pos), 1e-6f);
    an::Controls turn{};
    turn.roll_deg = 30.0f;
    for (int i = 0; i < 10; ++i) ASSERT_TRUE(run.step(turn).has_value());
    const spade::FrameState held = design();
    EXPECT_LT(glm::length(held.pos), 1e-6f);
    EXPECT_LT(glm::length(held.vel), 1e-6f);
    EXPECT_LT(quat_gap(held.orient, run.state().attitude), 1e-6f);
    EXPECT_LT(glm::length(held.omega - run.state().omega_body), 1e-4f);
}

// Wind blows through the box; still air stays still with the rotors stopped.
TEST(AnalysisLiveRun, TheWindBlowsThroughTheBox) {
    const spade::vehicles::ModelType m = rotorless();
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
    const spade::vehicles::ModelType m = rotorless();
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
    const spade::vehicles::ModelType m = rotorless();
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

// ===========================================================================
// SOLIDS (P2, D1's critical path): the model's rigid parts in the air, posed
// once at the hover; set_solids() and the pressure's CG count are the run's,
// and the published volume masks the cells inside them (P6).
// ===========================================================================

namespace {

// The powered quad with one body part: a ball of radius `radius` declaring
// `feature` as its smallest feature.
[[nodiscard]] Powered with_ball_body(float radius, float feature) {
    Powered p = powered();
    spade::vehicles::PartDesc ball;
    ball.name = "body";
    ball.shape.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode n{};
    n.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.params = glm::vec4(radius, 0.0f, 0.0f, 0.0f);
    ball.shape.nodes = {n};
    ball.min_feature = feature;
    p.model.parts = {ball};
    return p;
}

// One armed frame at half throttle, its refusal named.
void armed_frame(an::LiveRun& run, spade::FlowVolume& v) {
    const spade::Result<an::LiveFrame> f = run.advance_frame(an::Controls{true, 0.5f, 0, 0, 0}, v);
    ASSERT_TRUE(f.has_value()) << f.error().context;
}

// tiny_case()'s 0.8 m box (the discs' kernels inside it) at 40 mm cells
// (20^3), stepped every 2 ms.
[[nodiscard]] an::AnalysisCase solid_case(const spade::vehicles::ModelType& m) {
    an::AnalysisCase c = tiny_case(m);
    c.grid.spacing = 0.04f;
    c.time.step = 0.002f;
    return c;
}

// A part of one primitive, `size` its params (a sphere's radius, a box's
// half extents), at `pos` in the design frame, declaring `feature`.
[[nodiscard]] spade::vehicles::PartDesc part(const char* name, spade::SdfPrim prim, glm::vec3 size, float feature,
                                             glm::vec3 pos) {
    spade::vehicles::PartDesc d;
    d.name = name;
    d.shape.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode n{};
    n.kind = static_cast<uint32_t>(prim);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.params = glm::vec4(size, 0.0f);
    d.shape.nodes = {n};
    d.min_feature = feature;
    d.mount_pos = pos;
    return d;
}

// The case resolved as a live run's is (P-1): its thin walls left to the run.
[[nodiscard]] an::ResolvedCase live_resolved(const an::AnalysisCase& c, const spade::vehicles::ModelType& m) {
    const auto r = an::resolve(c, m, an::ThinWalls::drawn_by_the_live_run);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().front().field + ": " + r.error().front().message);
    return r ? *r : an::ResolvedCase{};
}

// Each part the run drew only, as "name#index why[ in inside]", in the order
// it left them out.
[[nodiscard]] std::vector<std::string> fates(const an::LiveRun& run) {
    std::vector<std::string> out;
    for (const an::DrawnPart& d : run.solids().drawn_only) {
        const char* why = "?";
        switch (d.why) {
            case an::DrawnReason::marked: why = "marked"; break;
            case an::DrawnReason::thin: why = "thin"; break;
            case an::DrawnReason::wire: why = "wire"; break;
            case an::DrawnReason::porous: why = "porous"; break;
            case an::DrawnReason::leaks: why = "leaks"; break;
            case an::DrawnReason::exposed: why = "exposed"; break;
            case an::DrawnReason::sibling: why = "sibling"; break;
            case an::DrawnReason::contained: why = "contained"; break;
        }
        out.push_back(d.name + "#" + std::to_string(d.part) + " " + why + (d.inside.empty() ? "" : " in " + d.inside) +
                      (d.of.empty() ? "" : " of " + d.of));
    }
    return out;
}

}  // namespace

// A 20 cm ball body (5 cells across) is in the air: the run says what
// set_solids() made of it, the volume is published with its solid mask,
// and the pressure's residual is measured. The same quad without it masks
// nothing.
TEST(AnalysisLiveRun, ItsPartsAreSolidsInTheAir) {
    const Powered p = with_ball_body(0.1f, 0.2f);
    an::LiveRun run = started(solid_case(p.model), p);
    EXPECT_TRUE(run.solids().any);
    ASSERT_FALSE(run.solids().run_line.empty());
    EXPECT_NE(run.run_label().find(run.solids().run_line), std::string::npos) << run.run_label();
    spade::FlowVolume v;
    armed_frame(run, v);
    if (::testing::Test::HasFatalFailure()) return;
    // Published with the mask (P6): the first solid layer is extrapolated
    // from its fluid side, and every channel of a solid cell under it is
    // no-data. (Measured: here all 12 of the ball's solid cells are first
    // layer, so the no-data count is 0.)
    EXPECT_GT(run.solids().solid_cells, 0u);
    EXPECT_GT(v.header.extrapolated_cells, 0u) << "the volume was published without the solid mask";
    ASSERT_GE(run.solids().solid_cells, v.header.extrapolated_cells);
    EXPECT_EQ(v.header.no_data, spade::kFlowChannelCount * (run.solids().solid_cells - v.header.extrapolated_cells));
    EXPECT_GT(run.solids().pressure_residual, 0.0f);
    EXPECT_TRUE(std::isfinite(run.solids().pressure_residual));

    const Powered bare = powered();
    an::LiveRun plain = started(solid_case(bare.model), bare);
    EXPECT_FALSE(plain.solids().any);
    EXPECT_EQ(plain.solids().solid_cells, 0u);
    EXPECT_EQ(plain.run_label().find("solids"), std::string::npos) << plain.run_label();
    spade::FlowVolume w;
    armed_frame(plain, w);
    if (::testing::Test::HasFatalFailure()) return;
    EXPECT_EQ(w.header.no_data, 0u);
    EXPECT_EQ(plain.solids().pressure_residual, 0.0f);
}

// A part thinner than it declares passes the case (resolve() reads the
// declaration), and the classifier finds it a wire at 40 mm (10 mm across
// every way): the live run draws it, names it, and flies discs only, to the
// bit as the quad without it (P-1; it was refused before). A restored run
// names it too.
TEST(AnalysisLiveRun, APartThinnerThanItDeclaresIsDrawnOnlyNamedAndRestored) {
    const Powered p = with_ball_body(0.005f, 0.1f);
    const an::AnalysisCase c = an::test::with_power(solid_case(p.model), p);
    an::LiveRun run = started(c, p);
    EXPECT_FALSE(run.solids().any);
    EXPECT_EQ(fates(run), std::vector<std::string>{"body#0 wire"});
    const Powered bare = powered();
    an::LiveRun plain = started(solid_case(bare.model), bare);
    EXPECT_EQ(run.run_label(), plain.run_label() + " \xc2\xb7 discs only: every part drawn only at 40 mm: body (a wire)");
    spade::FlowVolume a;
    spade::FlowVolume b;
    armed_frame(run, a);
    armed_frame(plain, b);
    if (::testing::Test::HasFatalFailure()) return;
    EXPECT_EQ(a.v, b.v) << "plan A at the case's n_v, as with no parts";
    EXPECT_EQ(a.s, b.s);

    const spade::test::TempDir dir("live_wire");
    const fs::path file = dir.path() / "run.livesave";
    ASSERT_TRUE(run.save(file).has_value());
    spade::Result<an::LiveRun> back = an::LiveRun::restore(file, resolved(c, p.model), p.model);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(back->run_label(), run.run_label());
    EXPECT_EQ(fates(*back), fates(run));
}

// The classifier's and the safeguard's parts at 40 mm, beside a 20 cm ball
// body that stays in the air: a 2 cm plate is a sheet (thinner than 2
// cells), a porous ball waits for the porous task, and a bead 2 cells across
// on a cell corner, alone, leaks. Each is drawn only, the classifier's in the
// model's order and then the safeguard's, and the label says why.
TEST(AnalysisLiveRun, WhatTheCellCantHoldIsDrawnOnlyAndTheLabelSaysWhy) {
    using spade::SdfPrim;
    spade::vehicles::ModelType m = rotorless();
    m.parts.push_back(part("body", SdfPrim::sphere, {0.1f, 0, 0}, 0.2f, {0, 0, 0}));
    m.parts.push_back(part("plate", SdfPrim::box, {0.1f, 0.01f, 0.1f}, 0.02f, {0, -0.28f, 0}));
    spade::vehicles::PartDesc filter = part("filter", SdfPrim::sphere, {0.05f, 0, 0}, 0.1f, {-0.24f, 0, 0});
    filter.surface = spade::vehicles::PartSurface{spade::vehicles::PartSurface::Kind::porous, 1.0e6f, 0.0f};
    m.parts.push_back(filter);
    m.parts.push_back(part("bead", SdfPrim::sphere, {0.04f, 0, 0}, 0.08f, {0.2f, 0, 0}));
    const an::AnalysisCase c = solid_case(m);
    spade::Result<an::LiveRun> run = an::LiveRun::start(live_resolved(c, m), m);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    EXPECT_TRUE(run->solids().any) << "the body is in the air";
    EXPECT_GT(run->solids().solid_cells, 0u);
    EXPECT_EQ(fates(*run), (std::vector<std::string>{"plate#1 thin", "filter#2 porous", "bead#3 leaks"}));
    EXPECT_NE(run->run_label().find(
                  " \xc2\xb7 drawn only at 40 mm: plate (thinner than 2 cells); filter (porous); bead (leaks)"),
              std::string::npos)
        << run->run_label();
    spade::Result<an::LiveRun> again = an::LiveRun::start(live_resolved(c, m), m);
    ASSERT_TRUE(again.has_value()) << again.error().context;
    EXPECT_EQ(again->run_label(), run->run_label()) << "the same case leaves out the same parts";
    EXPECT_EQ(again->solids().solid_cells, run->solids().solid_cells);
}

// A bead on the face of a 48 cm box body, half inside it, is exposed within
// a cell of its surface (Physics' safeguard): drawn only, naming the body.
TEST(AnalysisLiveRun, APartExposedInsideAnotherIsDrawnOnlyNamingIt) {
    using spade::SdfPrim;
    spade::vehicles::ModelType m = rotorless();
    m.parts.push_back(part("body", SdfPrim::box, {0.24f, 0.24f, 0.24f}, 0.48f, {0, 0, 0}));
    m.parts.push_back(part("bead", SdfPrim::sphere, {0.04f, 0, 0}, 0.08f, {0.24f, 0, 0}));
    spade::Result<an::LiveRun> run = an::LiveRun::start(live_resolved(solid_case(m), m), m);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    EXPECT_TRUE(run->solids().any);
    EXPECT_EQ(fates(*run), std::vector<std::string>{"bead#1 exposed in body"});
    EXPECT_NE(run->run_label().find(" \xc2\xb7 drawn only at 40 mm: bead (inside 'body', exposed)"), std::string::npos)
        << run->run_label();
}

// The label's words (P-1's example, the lead's ruling, 2026-11-03).
TEST(AnalysisLiveRun, TheDrawnOnlyLineGroupsByReasonAndJoinsIndexedNames) {
    using an::DrawnPart;
    using an::DrawnReason;
    EXPECT_EQ(an::drawn_only_line({}, 0.012f, true), "");
    const std::vector<DrawnPart> phantom{
        {"arm[0]", 1, DrawnReason::thin, {}, {}},     {"arm[1]", 2, DrawnReason::thin, {}, {}},
        {"arm[2]", 3, DrawnReason::thin, {}, {}},     {"arm[3]", 4, DrawnReason::thin, {}, {}},
        {"esc", 9, DrawnReason::thin, {}, {}},        {"battery", 10, DrawnReason::exposed, "body", {}},
        {"motor[3]", 8, DrawnReason::leaks, {}, {}},  {"motor[1]", 6, DrawnReason::leaks, {}, {}},
    };
    EXPECT_EQ(an::drawn_only_line(phantom, 0.012f, true),
              "drawn only at 12 mm: arm[0-3], esc (thinner than 2 cells); battery (inside 'body', exposed); "
              "motor[1], motor[3] (leaks)");
    const std::vector<DrawnPart> others{
        {"antenna", 5, DrawnReason::marked, {}, {}}, {"bolt", 2, DrawnReason::wire, {}, {}},
        {"skid[0]", 3, DrawnReason::thin, {}, {}},   {"skid[1]", 4, DrawnReason::thin, {}, {}},
        {"filter", 6, DrawnReason::porous, {}, {}},  {"cell[1]", 8, DrawnReason::exposed, "pack", {}},
        {"cell[2]", 9, DrawnReason::exposed, "shell", {}}, {"lid[a]", 10, DrawnReason::leaks, {}, {}},
        {"lid[b]", 11, DrawnReason::leaks, {}, {}},
    };
    EXPECT_EQ(an::drawn_only_line(others, 0.00175f, true),
              "drawn only at 1.75 mm: antenna (marked drawn only); bolt (a wire); skid[0-1] (thinner than 2 cells); "
              "filter (porous); cell[1] (inside 'pack', exposed); cell[2] (inside 'shell', exposed); lid[a], lid[b] "
              "(leaks)");
    // L8: a part the safeguard refused and the indexed siblings it took with it.
    const std::vector<DrawnPart> siblings{
        {"motor[3]", 8, DrawnReason::leaks, {}, {}},         {"motor[0]", 5, DrawnReason::sibling, {}, "motor[3]"},
        {"motor[1]", 6, DrawnReason::sibling, {}, "motor[3]"}, {"motor[2]", 7, DrawnReason::sibling, {}, "motor[3]"},
        {"cell[2]", 12, DrawnReason::exposed, "pack", {}},   {"cell[1]", 11, DrawnReason::sibling, {}, "cell[2]"},
        {"battery", 10, DrawnReason::exposed, "body", {}},
    };
    EXPECT_EQ(an::drawn_only_line(siblings, 0.012f, true),
              "drawn only at 12 mm: motor[0-3] (motor[3] leaks; its siblings with it); cell[1-2] (cell[2] inside "
              "'pack', exposed; its siblings with it); battery (inside 'body', exposed)");
    const std::vector<DrawnPart> all{{"body", 0, DrawnReason::thin, {}, {}}, {"arm[0]", 1, DrawnReason::wire, {}, {}}};
    EXPECT_EQ(an::drawn_only_line(all, 0.039f, false),
              "discs only: every part drawn only at 39 mm: body (thinner than 2 cells); arm[0] (a wire)");
}

namespace {

// The library's Phantom 3-class quad, as Run air will fly it: its parts
// kept, on a cube of `n` cells of `dx` held at the box's centre, at the tour's
// power (5 % idle, a full pack), its thin walls left to the live run.
[[nodiscard]] spade::Result<an::LiveRun> phantom_run(const spade::vehicles::ModelType& m, uint32_t n, float dx) {
    an::AnalysisCase c;
    c.model = an::pin_model("phantom3_class.model.yaml", m);
    c.power = an::Power{.idle = 0.05f, .state_of_charge = 1.0f};
    c.rotors.assign(m.rotors.size(), an::RotorSetup{});
    c.grid.box = static_cast<float>(n) * dx;
    c.grid.spacing = dx;
    c.time.duration = 1.0f;
    c.time.step = 0.002f;
    return an::LiveRun::start(live_resolved(c, m), m);
}

[[nodiscard]] spade::vehicles::ModelType library_drone(const char* file) {
    auto f = spade::scene::load_model_file(fs::path(SPADE_ASSETS_DIR) / "library" / "drones" / file);
    EXPECT_TRUE(f.has_value()) << file << ": " << (f ? "" : f.error().context);
    return f ? f->model : spade::vehicles::ModelType{};
}

[[nodiscard]] spade::vehicles::ModelType phantom() { return library_drone("phantom3_class.model.yaml"); }

}  // namespace

// At P-1's cell for it, 12 mm, on 48^3 (0.576 m, its 0.49 m span inside;
// held at a cell corner, as on 104^3): the body alone is in the air.
// Measured 2026-11-03, as Physics' note has it: the arms are wires and the
// ESC a sheet; motor[3] leaks where the grid falls on it (the mirrored
// motors fall differently by fp32's rounding), and takes its siblings with
// it (L8: three of four would give the held drone a moment its geometry
// hasn't); the battery, inside the shell, is exposed. Each is named, the
// classifier's first. The same case leaves out the same parts, and it steps.
TEST(AnalysisLiveRun, ThePhantomAt12mmKeepsItsBodyInTheAirAndNamesWhatItDraws) {
    const spade::vehicles::ModelType m = phantom();
    spade::Result<an::LiveRun> run = phantom_run(m, 48, 0.012f);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    EXPECT_TRUE(run->solids().any);
    EXPECT_NE(run->solids().run_line.find("solids: 1 parts, 1 walls, 0 sheets"), std::string::npos)
        << run->solids().run_line;
    EXPECT_EQ(fates(*run),
              (std::vector<std::string>{"arm[0]#1 wire", "arm[1]#2 wire", "arm[2]#3 wire", "arm[3]#4 wire", "esc#9 thin",
                                        "motor[3]#8 leaks", "motor[0]#5 sibling of motor[3]",
                                        "motor[1]#6 sibling of motor[3]", "motor[2]#7 sibling of motor[3]",
                                        "battery#10 exposed in body"}));
    EXPECT_NE(run->run_label().find(" \xc2\xb7 drawn only at 12 mm: arm[0-3] (a wire); esc (thinner than 2 cells); "
                                    "motor[0-3] (motor[3] leaks; its siblings with it); battery (inside 'body', "
                                    "exposed)"),
              std::string::npos)
        << run->run_label();
    std::printf("[ p-1 ] the Phantom at 12 mm on 48^3: %s (%u solid cells)\n", run->run_label().c_str(),
                run->solids().solid_cells);
    spade::Result<an::LiveRun> again = phantom_run(m, 48, 0.012f);
    ASSERT_TRUE(again.has_value()) << again.error().context;
    EXPECT_EQ(again->run_label(), run->run_label());
    for (int i = 0; i < 2; ++i) {
        const spade::Result<void> s = run->step(an::Controls{true, 0.5f, 0, 0, 0});
        ASSERT_TRUE(s.has_value()) << s.error().context;
    }
}

// L13 (2) (Physics' ruling (a); the lead, 2026-11-03): a part left out of
// the air takes every part whose 8 posed box corners all lie inside it (its
// SDF < 0 at each), named "inside '<it>', with it"; lattice-free, and not by
// the centre. A porous shell left out takes the box wholly inside it; a box
// straddling its surface, its centre inside, stays in the air.
TEST(AnalysisLiveRun, APartLeftOutTakesThePartsWhollyInsideIt) {
    Powered p = powered();
    spade::vehicles::PartDesc shell = part("shell", spade::SdfPrim::sphere, glm::vec3(0.2f), 0.4f, glm::vec3(0.0f));
    shell.surface = spade::vehicles::PartSurface{spade::vehicles::PartSurface::Kind::porous, 1.0e6f, 0.0f};
    p.model.parts = {shell,
                     part("core", spade::SdfPrim::box, glm::vec3(0.05f), 0.1f, glm::vec3(0.0f)),
                     part("stub", spade::SdfPrim::box, glm::vec3(0.05f), 0.1f, glm::vec3(0.0f, -0.18f, 0.0f))};
    an::LiveRun run = started(solid_case(p.model), p);
    EXPECT_EQ(fates(run), (std::vector<std::string>{"shell#0 porous", "core#1 contained in shell"}));
    EXPECT_TRUE(run.solids().any) << "the stub, its centre inside the shell but two corners out, stays";
    EXPECT_NE(run.run_label().find("drawn only at 40 mm: shell (porous); core (inside 'shell', with it)"),
              std::string::npos)
        << run.run_label();
}

// A part the user marked drawn only takes nothing: the mark is the user's
// modelling choice, and what lies inside it stays in the air (Physics' review
// of L13 (2); the sweep runs for what the classifier or the safeguard leaves
// out, not for a mark).
TEST(AnalysisLiveRun, AMarkedDrawnOnlyPartTakesNothingInsideIt) {
    Powered p = powered();
    spade::vehicles::PartDesc shell = part("shell", spade::SdfPrim::sphere, glm::vec3(0.2f), 0.4f, glm::vec3(0.0f));
    shell.role = std::string(spade::vehicles::kDrawnOnlyRole);
    p.model.parts = {shell, part("core", spade::SdfPrim::box, glm::vec3(0.05f), 0.1f, glm::vec3(0.0f))};
    an::LiveRun run = started(solid_case(p.model), p);
    EXPECT_EQ(fates(run), (std::vector<std::string>{"shell#0 marked"}));
    EXPECT_TRUE(run.solids().any) << "the core inside the marked shell stays in the air";
}

// At 16 mm the Phantom's body leaks (Physics, 2026-11-03), and the battery
// and the ESC, wholly inside its shell (8 of 8 corners; each arm 2 of 8),
// go with it: no solid is left, so the run is discs only.
TEST(AnalysisLiveRun, ThePhantomAt16mmLeavesOutItsBodyAndThePartsInsideIt) {
    const spade::vehicles::ModelType m = phantom();
    spade::Result<an::LiveRun> run = phantom_run(m, 40, 0.016f);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    std::printf("[ l13 ] the Phantom at 16 mm on 40^3: %s\n", run->run_label().c_str());
    for (const std::string& f : fates(*run)) std::printf("[ l13 ]   %s\n", f.c_str());
    EXPECT_FALSE(run->solids().any);
    const std::vector<std::string> f = fates(*run);
    EXPECT_NE(std::find(f.begin(), f.end(), "body#0 leaks"), f.end());
    EXPECT_NE(std::find(f.begin(), f.end(), "battery#10 contained in body"), f.end());
    EXPECT_NE(run->run_label().find(" \xc2\xb7 discs only: every part drawn only at 16 mm: "), std::string::npos)
        << run->run_label();
    EXPECT_NE(run->run_label().find("battery (inside 'body', with it)"), std::string::npos) << run->run_label();
}

// At Run air's cell today, 39 mm on 32^3, every Phantom part is too thin
// for the grid (P-1: no solid in the air for any library drone; the body a
// sheet, the rest wires): discs only, said so, and to the bit the run of the
// Phantom with its parts cleared.
TEST(AnalysisLiveRun, ThePhantomAt39mmIsDiscsOnlyToTheBitAsItsDiscsAlone) {
    const spade::vehicles::ModelType m = phantom();
    spade::vehicles::ModelType discs = m;
    discs.parts.clear();
    spade::Result<an::LiveRun> run = phantom_run(m, 32, 0.039f);
    spade::Result<an::LiveRun> plain = phantom_run(discs, 32, 0.039f);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    ASSERT_TRUE(plain.has_value()) << plain.error().context;
    EXPECT_FALSE(run->solids().any);
    EXPECT_EQ(run->run_label(), plain->run_label() +
                                    " \xc2\xb7 discs only: every part drawn only at 39 mm: body (thinner than 2 "
                                    "cells); arm[0-3], motor[0-3], esc, battery (a wire)");
    std::printf("[ p-1 ] the Phantom at 39 mm on 32^3: %s\n", run->run_label().c_str());
    spade::FlowVolume a;
    spade::FlowVolume b;
    armed_frame(*run, a);
    armed_frame(*plain, b);
    if (::testing::Test::HasFatalFailure()) return;
    EXPECT_EQ(a.v, b.v);
    EXPECT_EQ(a.s, b.s);
}

// P-1's cell rule (the lead's rulings, 2026-11-03): Run air's box as before,
// 32 cells of R/3 to the mm, at min(R/3, F/4) to the mm, n = ceil(box/dx)
// rounded up to a multiple of 8 (L12: an odd n puts the drone's centre on a
// cell centre, and the Phantom's 10 mm mask then stalls the CG; the cell is
// kept and the box grows), at most 128. Every library drone with rigid parts
// moves (the reference quad's 32^3 at 25 mm to 104^3 at 8 mm); past 128 the
// cell is box/128, as the Crazyflie's and the Phantom's at 9 mm. A drone
// without parts in the air keeps R/3 on 32^3.
TEST(AnalysisLiveRun, RunAirsGridIsP1sCellRule) {
    const auto grid = [](const spade::vehicles::ModelType& m) {
        const spade::Result<an::LiveGrid> g = an::live_grid(m);
        EXPECT_TRUE(g.has_value()) << m.name << ": " << (g ? "" : g.error().context);
        return g ? std::format("{}^3 at {:g} mm", g->n, g->dx * 1000.0f) : std::string("refused");
    };
    const auto at_feature = [](float f) {  // the Phantom with its widest wall at most `f`: F/4 is f/4
        spade::vehicles::ModelType m = phantom();
        for (spade::vehicles::PartDesc& p : m.parts) p.min_feature = std::min(p.min_feature, f);
        return m;
    };
    EXPECT_EQ(grid(phantom()), "104^3 at 12 mm");
    EXPECT_EQ(grid(library_drone("iris_plus_class.model.yaml")), "112^3 at 12 mm") << "107 rounded up";
    EXPECT_EQ(grid(library_drone("reference_quad.model.yaml")), "104^3 at 8 mm") << "100 rounded up";
    EXPECT_EQ(grid(library_drone("basic_quad_6inch.model.yaml")), "104^3 at 8 mm");
    EXPECT_EQ(grid(library_drone("basic_quad_competition.model.yaml")), "104^3 at 8 mm");
    EXPECT_EQ(grid(library_drone("fpv_5inch.model.yaml")), "96^3 at 7 mm");
    EXPECT_EQ(grid(library_drone("crazyflie_class.model.yaml")), "128^3 at 1.75 mm");
    EXPECT_EQ(grid(at_feature(0.040f)), "128^3 at 10 mm") << "125 rounded up, the box 1.28 m";
    EXPECT_EQ(grid(at_feature(0.036f)), "128^3 at 9.75 mm") << "139 cells of 9 mm: the cap, at box/128";
    spade::vehicles::ModelType bare = phantom();
    bare.parts.clear();
    EXPECT_EQ(grid(bare), "32^3 at 39 mm");
    spade::vehicles::ModelType marked = phantom();
    for (spade::vehicles::PartDesc& p : marked.parts) p.role = std::string(spade::vehicles::kDrawnOnlyRole);
    EXPECT_EQ(grid(marked), "32^3 at 39 mm") << "drawn-only parts never enter the air";
    const spade::Result<an::LiveGrid> none = an::live_grid(rotorless());
    ASSERT_FALSE(none.has_value());
    EXPECT_NE(none.error().context.find("no rotors"), std::string::npos) << none.error().context;
}

// The levels' step (L13 and L17; the lead, 2026-11-03): a CFL of at most 2
// under a 4 ms cap, floored past 1 ms to whole drone ticks of 1 ms, which
// the run flies as substeps. The Phantom's body at 12 mm steps 2 ms (2.24 ms
// floored), its real-time discs at 26 mm the cap, and at 4 mm 0.748 ms in
// one tick; a run at 2 ms flies 2 drone ticks a step, one at 1 ms one.
TEST(AnalysisLiveRun, TheLevelsStepIsCfl2UnderTheCapInWholeDroneTicks) {
    const spade::vehicles::ModelType m = phantom();
    an::AnalysisCase c;
    EXPECT_EQ(an::live_level_step_s(c, m, 0.012f), 0.002f);
    EXPECT_EQ(an::live_level_step_s(c, m, 0.026f), an::kLiveLevelStepCapS);
    EXPECT_NEAR(an::live_level_step_s(c, m, 0.004f), 0.7480e-3f, 0.0005e-3f);
    EXPECT_EQ(an::live_level_step_s(c, rotorless(), 0.012f), an::kLiveLevelStepCapS);

    an::AnalysisCase t = tiny_case(rotorless());
    EXPECT_EQ(started(t, rotorless()).drone_substeps(), 4u) << "4 ms in ticks of 1 ms";
    t.time.step = 0.002f;
    EXPECT_EQ(started(t, rotorless()).drone_substeps(), 2u);
    t.time.step = 0.001f;
    EXPECT_EQ(started(t, rotorless()).drone_substeps(), 1u);
    t.time.step = 0.0007f;
    EXPECT_EQ(started(t, rotorless()).drone_substeps(), 1u);
}

// The floor to the millimetre is exact (the lead's review, 2026-11-03): in
// float32, R/3 times 1000 falls a hair under a whole millimetre for 132 of
// the whole-mm thirds from 3 mm to 1 m (R 36 mm gave 11 mm), so the rule
// rounds to whole micrometres first, in integers.
TEST(AnalysisLiveRun, RunAirsCellFloorsAWholeMillimetreExactly) {
    const auto grid = [](float radius, float feature) {
        spade::vehicles::ModelType m = phantom();
        for (spade::vehicles::RotorDesc& r : m.rotors) r.radius = radius;
        if (feature > 0.0f) {
            for (spade::vehicles::PartDesc& p : m.parts) p.min_feature = std::min(p.min_feature, feature);
        } else {
            m.parts.clear();
        }
        const spade::Result<an::LiveGrid> g = an::live_grid(m);
        EXPECT_TRUE(g.has_value()) << (g ? "" : g.error().context);
        return g ? std::format("{}^3 at {:g} mm", g->n, g->dx * 1000.0f) : std::string("refused");
    };
    EXPECT_EQ(grid(0.036f, 0.0f), "32^3 at 12 mm");
    EXPECT_EQ(grid(0.072f, 0.0f), "32^3 at 24 mm");
    EXPECT_EQ(grid(0.144f, 0.0f), "32^3 at 48 mm");
    EXPECT_EQ(grid(0.144f, 0.048f), "128^3 at 12 mm") << "F/4 on a whole millimetre, in 32 cells of 48 mm";
}

// Physics' step rule (the lead's ruling, 2026-11-03): CFL at most 1 on the
// wake's 2 v_h, capped at 1 ms. The Phantom (1.28 kg, R 119.4 mm, v_h 5.35
// m/s in the case's air) keeps 1 ms at 12 mm and takes 0.935 ms at 10 mm; a
// 5 m/s wind shortens it; the run's label names the step.
TEST(AnalysisLiveRun, RunAirsStepIsPhysicsRuleAndTheLabelNamesIt) {
    const spade::vehicles::ModelType m = phantom();
    an::AnalysisCase c;
    EXPECT_EQ(an::live_step_s(c, m, 0.012f), 0.001f);
    EXPECT_NEAR(an::live_step_s(c, m, 0.010f), 0.9350e-3f, 0.0005e-3f);
    EXPECT_NEAR(an::live_step_s(c, m, 0.008f), 0.7480e-3f, 0.0005e-3f);
    EXPECT_NEAR(an::live_step_s(c, m, 0.007f), 0.6545e-3f, 0.0005e-3f);
    an::AnalysisCase windy = c;
    windy.flight.kind = an::FlightKind::hover_in_wind;
    windy.flight.speed = 5.0f;
    EXPECT_NEAR(an::live_step_s(windy, m, 0.012f), 0.7646e-3f, 0.0005e-3f);
    EXPECT_EQ(an::live_step_s(c, rotorless(), 0.012f), an::kLiveStepCapS);

    an::AnalysisCase t = tiny_case(rotorless());
    EXPECT_NE(started(t, rotorless()).run_label().find("dx 100 mm, h 4 ms"), std::string::npos);
    t.time.step = an::live_step_s(c, m, 0.010f);
    const std::string label = started(t, rotorless()).run_label();
    EXPECT_NE(label.find("dx 100 mm, h 0.935 ms"), std::string::npos) << label;
}

// A restored run rebuilds its solids at the hover and gives the frame an
// unbroken one does.
TEST(AnalysisLiveRun, ARestoredRunKeepsItsSolids) {
    const Powered p = with_ball_body(0.1f, 0.2f);
    const an::AnalysisCase c = solid_case(p.model);
    an::LiveRun run = started(c, p);
    spade::FlowVolume v;
    armed_frame(run, v);
    if (::testing::Test::HasFatalFailure()) return;
    const spade::test::TempDir dir("live_solids");
    const fs::path file = dir.path() / "run.livesave";
    ASSERT_TRUE(run.save(file).has_value());
    spade::Result<an::LiveRun> back =
        an::LiveRun::restore(file, resolved(an::test::with_power(c, p), p.model), p.model);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(back->run_label(), run.run_label());
    spade::FlowVolume a;
    spade::FlowVolume b;
    armed_frame(run, a);
    armed_frame(*back, b);
    if (::testing::Test::HasFatalFailure()) return;
    EXPECT_EQ(a.header.no_data, b.header.no_data);
    EXPECT_EQ(a.v, b.v);
    EXPECT_EQ(a.s, b.s);
}

// ===========================================================================
// N15, DRAWN-ONLY AND IMPORTED PARTS (Physics' ruling and condition,
// 2026-10-22; amended for the live run by P-1, the lead, 2026-11-03). A part
// marked vehicles::kDrawnOnlyRole is drawn, not a solid: the run leaves it out
// of the air and names it in its label, whether or not any solid is left.
// Whether the run has solids is decided by the parts left in, so a model whose
// parts are all drawn only flies to the bit as the same model without them
// (plan A at the case's n_v). An imported solid (kImportedPartRole) the grid
// can't resolve is drawn only and named, as an authored one is.
// ===========================================================================

TEST(AnalysisLiveRun, ADrawnOnlyPartIsLeftOutOfTheAirAndNamedInTheRunLine) {
    Powered p = with_ball_body(0.1f, 0.2f);
    spade::vehicles::PartDesc antenna = p.model.parts.front();
    antenna.name = "antenna";
    antenna.role = std::string(spade::vehicles::kDrawnOnlyRole);
    antenna.mount_pos = glm::vec3(0.0f, 0.2f, 0.0f);
    p.model.parts.push_back(antenna);
    const an::AnalysisCase c = solid_case(p.model);
    an::LiveRun run = started(c, p);
    EXPECT_TRUE(run.solids().any);
    EXPECT_EQ(fates(run), std::vector<std::string>{"antenna#1 marked"});
    EXPECT_NE(run.run_label().find(" \xc2\xb7 drawn only at 40 mm: antenna (marked drawn only)"), std::string::npos)
        << run.run_label();

    spade::FlowVolume v;
    armed_frame(run, v);
    if (::testing::Test::HasFatalFailure()) return;
    const spade::test::TempDir dir("live_drawn_only");
    const fs::path file = dir.path() / "run.livesave";
    ASSERT_TRUE(run.save(file).has_value());
    spade::Result<an::LiveRun> back =
        an::LiveRun::restore(file, resolved(an::test::with_power(c, p), p.model), p.model);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(back->run_label(), run.run_label()) << "a restored run names it too";
}

TEST(AnalysisLiveRun, AModelWhoseOnlyPartsAreDrawnOnlyFliesToTheBitAsOneWithoutThem) {
    Powered p = with_ball_body(0.1f, 0.2f);
    p.model.parts.front().role = std::string(spade::vehicles::kDrawnOnlyRole);
    an::LiveRun run = started(solid_case(p.model), p);
    const Powered bare = powered();
    an::LiveRun plain = started(solid_case(bare.model), bare);
    EXPECT_FALSE(run.solids().any);
    EXPECT_EQ(run.solids().solid_cells, 0u);
    EXPECT_EQ(run.run_label(),
              plain.run_label() + " \xc2\xb7 discs only: every part drawn only at 40 mm: body (marked drawn only)");
    spade::FlowVolume a;
    spade::FlowVolume b;
    for (int i = 0; i < 2; ++i) {
        armed_frame(run, a);
        armed_frame(plain, b);
        if (::testing::Test::HasFatalFailure()) return;
    }
    EXPECT_EQ(a.header.no_data, b.header.no_data);
    EXPECT_EQ(a.v, b.v) << "plan A at the case's n_v, as with no parts";
    EXPECT_EQ(a.s, b.s);
    EXPECT_EQ(run.solids().pressure_residual, plain.solids().pressure_residual);
}

TEST(AnalysisLiveRun, TwoDrawnOnlyPartsAreNamedInTheirOrder) {
    Powered p = with_ball_body(0.1f, 0.2f);
    p.model.parts.front().role = std::string(spade::vehicles::kDrawnOnlyRole);
    spade::vehicles::PartDesc second = p.model.parts.front();
    second.name = "skid";
    second.mount_pos = glm::vec3(0.0f, -0.2f, 0.0f);
    p.model.parts.push_back(second);
    an::LiveRun run = started(solid_case(p.model), p);
    EXPECT_NE(
        run.run_label().find(" \xc2\xb7 discs only: every part drawn only at 40 mm: body, skid (marked drawn only)"),
        std::string::npos)
        << run.run_label();
}

// An imported part the grid can't resolve is drawn only and named as an
// authored one is (P-1; it was refused, with the remedy of marking it).
TEST(AnalysisLiveRun, AnImportedPartTooThinForTheGridIsDrawnOnlyAsAnAuthoredOneIs) {
    Powered p = with_ball_body(0.005f, 0.1f);
    p.model.parts.front().role = std::string(an::kImportedPartRole);
    an::LiveRun run = started(solid_case(p.model), p);
    const Powered authored = with_ball_body(0.005f, 0.1f);
    an::LiveRun plain = started(solid_case(authored.model), authored);
    EXPECT_EQ(fates(run), std::vector<std::string>{"body#0 wire"});
    EXPECT_EQ(run.run_label(), plain.run_label());
    EXPECT_EQ(run.run_label().find("visual-only"), std::string::npos) << run.run_label();
}

// P2d (Physics, merged a4ec57f): an unconverged solids step is exactly one
// of stalled (the CG stopped at fp32's floor) or capped (it reached its
// cap). The run counts each and its line says which, naming only what
// occurred (the lead's plan, 2026-10-22).
TEST(AnalysisLiveRun, TheSolidsHealthLineSaysWhyThePressureWasUnconverged) {
    an::LiveSolids s;
    EXPECT_EQ(an::solids_health_line(s, 64), "");
    s.pressure_residual = 2.0e-3f;
    s.unconverged_steps = 3;
    s.stalled_steps = 3;
    EXPECT_EQ(an::solids_health_line(s, 64),
              "pressure unconverged on 3 steps: 3 stalled at fp32's floor (last max|r|/max|b| 2.0e-03)");
    s.unconverged_steps = 1;
    s.stalled_steps = 0;
    s.capped_steps = 1;
    EXPECT_EQ(an::solids_health_line(s, 1),
              "pressure unconverged on 1 step: 1 at the CG's cap of 1 iteration (last max|r|/max|b| 2.0e-03)");
    s.unconverged_steps = 5;
    s.stalled_steps = 2;
    s.capped_steps = 3;
    EXPECT_EQ(an::solids_health_line(s, 64),
              "pressure unconverged on 5 steps: 2 stalled at fp32's floor, 3 at the CG's cap of 64 iterations "
              "(last max|r|/max|b| 2.0e-03)");
}

// L13 (1) (the lead, from Physics' diagnosis, 2026-11-03): a step whose
// pressure ends past kLiveLooseResidual, ten times a healthy solve's 2e-6,
// but not past kResidualAnnounce is loose, counted beside the unconverged
// and said after them, with the largest it reached and what a healthy solve
// ends near. The CG's stopping rule is unchanged.
TEST(AnalysisLiveRun, TheSolidsHealthLineSaysHowLooseThePressureWas) {
    EXPECT_EQ(an::kLiveLooseResidual, 10.0f * an::kLiveHealthyResidual);
    EXPECT_LT(an::kLiveLooseResidual, spade::physics::airflow::kResidualAnnounce);
    an::LiveSolids s;
    s.loose_steps = 12;
    s.loose_max_residual = 8.5e-5f;
    EXPECT_EQ(an::solids_health_line(s, 64),
              "pressure loose on 12 steps (max|r|/max|b| up to 8.5e-5; a healthy solve ends near 2e-6)");
    s.loose_steps = 1;
    s.loose_max_residual = 3.0e-5f;
    EXPECT_EQ(an::solids_health_line(s, 64),
              "pressure loose on 1 step (max|r|/max|b| up to 3.0e-5; a healthy solve ends near 2e-6)");
    s.pressure_residual = 2.0e-3f;
    s.unconverged_steps = 3;
    s.stalled_steps = 3;
    EXPECT_EQ(an::solids_health_line(s, 64),
              "pressure unconverged on 3 steps: 3 stalled at fp32's floor (last max|r|/max|b| 2.0e-03); pressure loose "
              "on 1 step (max|r|/max|b| up to 3.0e-5; a healthy solve ends near 2e-6)");
}

// A run counts each loose step once, beside (not among) the unconverged.
// The ball body's frame (17 steps) with its solves held to k iterations,
// measured 2026-11-04: at 4, 2 loose steps (up to 2.93e-5) and none
// unconverged; at 3, 16 unconverged and the last, at 9.94e-5, loose; at the
// CG's floor of 8, none of either.
TEST(AnalysisLiveRun, ALooseSolveIsCountedBesideTheUnconverged) {
    const Powered p = with_ball_body(0.1f, 0.2f);
    const auto held_to = [&](uint32_t k) {
        an::LiveRun run = started(solid_case(p.model), p);
        EXPECT_TRUE(run.solids().any);
        run.set_test_solids_max_iterations(k);
        spade::FlowVolume v;
        armed_frame(run, v);
        return run;
    };
    const an::LiveRun four = held_to(4);
    if (::testing::Test::HasFatalFailure()) return;
    EXPECT_EQ(four.state().steps, 17u);
    EXPECT_EQ(four.solids().unconverged_steps, 0u);
    EXPECT_EQ(four.solids().loose_steps, 2u);
    EXPECT_GT(four.solids().loose_max_residual, an::kLiveLooseResidual);
    EXPECT_NEAR(four.solids().loose_max_residual, 2.93e-5f, 0.01e-5f);
    EXPECT_EQ(four.pressure_line(),
              "pressure loose on 2 steps (max|r|/max|b| up to 2.9e-5; a healthy solve ends near 2e-6)");
    const an::LiveRun three = held_to(3);
    if (::testing::Test::HasFatalFailure()) return;
    EXPECT_EQ(three.solids().unconverged_steps, 16u);
    EXPECT_EQ(three.solids().loose_steps, 1u);
    EXPECT_EQ(three.solids().unconverged_steps + three.solids().loose_steps, three.state().steps);
    EXPECT_NE(three.pressure_line().find("; pressure loose on 1 step (max|r|/max|b| up to 9.9e-5; a healthy solve"),
              std::string::npos)
        << three.pressure_line();
    const an::LiveRun eight = held_to(8);
    if (::testing::Test::HasFatalFailure()) return;
    EXPECT_EQ(eight.solids().loose_steps, 0u);
    EXPECT_EQ(eight.solids().loose_max_residual, 0.0f);
    EXPECT_EQ(eight.pressure_line(), "");
}

// A test-only limit of 1 iteration caps the ball body's solves (at the CG's
// own floor of 8 they reach ~1e-6, measured 2026-10-22, so nothing a test can
// size caps by itself): the run counts them as capped, stalled and capped sum
// to unconverged, and its pressure line, the end of its run line, names the
// cap the solve ran to.
TEST(AnalysisLiveRun, ACappedSolveIsCountedAndItsCapNamedInTheRunLine) {
    const Powered p = with_ball_body(0.1f, 0.2f);
    an::LiveRun run = started(solid_case(p.model), p);
    ASSERT_TRUE(run.solids().any);
    run.set_test_solids_max_iterations(1);
    spade::FlowVolume v;
    armed_frame(run, v);
    if (::testing::Test::HasFatalFailure()) return;
    const an::LiveSolids& s = run.solids();
    EXPECT_GT(s.capped_steps, 0u) << run.run_line();
    EXPECT_EQ(s.capped_steps + s.stalled_steps, s.unconverged_steps);
    EXPECT_EQ(s.pressure_iterations, 1u);
    EXPECT_NE(run.pressure_line().find("at the CG's cap of 1 iteration ("), std::string::npos) << run.pressure_line();
    const std::string line = run.run_line();
    ASSERT_GE(line.size(), run.pressure_line().size());
    EXPECT_EQ(line.substr(line.size() - run.pressure_line().size()), run.pressure_line()) << line;
}

// The limit set and then unset (0) flies to the bit as a run never given it
// (C8q's nudge hook's rule).
TEST(AnalysisLiveRun, TheSolidsIterationLimitUnsetFliesToTheBit) {
    const Powered p = with_ball_body(0.1f, 0.2f);
    an::LiveRun a = started(solid_case(p.model), p);
    an::LiveRun b = started(solid_case(p.model), p);
    b.set_test_solids_max_iterations(1);
    b.set_test_solids_max_iterations(0);
    spade::FlowVolume va;
    spade::FlowVolume vb;
    for (int i = 0; i < 2; ++i) {
        armed_frame(a, va);
        armed_frame(b, vb);
        if (::testing::Test::HasFatalFailure()) return;
    }
    EXPECT_EQ(va.v, vb.v);
    EXPECT_EQ(va.s, vb.s);
    EXPECT_EQ(a.solids().pressure_residual, b.solids().pressure_residual);
    EXPECT_EQ(a.solids().unconverged_steps, b.solids().unconverged_steps);
    EXPECT_EQ(a.solids().pressure_iterations, b.solids().pressure_iterations);
}
