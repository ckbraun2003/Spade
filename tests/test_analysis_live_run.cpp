// LiveRun, the drone-box airflow run (I1; the harness design,
// docs/design/interface/plans/2026-10-13-i1-harness-design.md): the one
// stepping core behind D1's recording, D2's panel, the queued runner and
// Kat's host. This first slice: the grid, the held drone, time in whole
// nanoseconds, frames at k/30 s, determinism and the refusals; then the
// rotors, through Core's throttle map (armed_duty, rotor_omega_for_duty) and
// Physics' P7a discs. The solids join with P2.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <tuple>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/flow_volume.hpp"
#include "scene/scene_file.hpp"
#include "sim/design_frame.hpp"
#include "state/layout.hpp"
#include "analysis_powered_quad.hpp"
#include "unique_temp.hpp"
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

[[nodiscard]] an::LiveRun started(const an::AnalysisCase& c, const Powered& p) {
    spade::Result<an::LiveRun> run = an::LiveRun::start(resolved(c, p.model), p.model, p.propulsion);
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

// The chain's inputs are checked when the run starts, not on its first step.
TEST(AnalysisLiveRun, RefusesABadIdleOrStateOfCharge) {
    for (const auto& [idle, soc, field] : {std::tuple{1.0, 0.8, "idle"}, std::tuple{-0.1, 0.8, "idle"},
                                           std::tuple{0.05, 1.5, "soc"}}) {
        Powered p = powered();
        p.propulsion.idle = idle;
        p.propulsion.soc = soc;
        const spade::Result<an::LiveRun> run =
            an::LiveRun::start(resolved(tiny_case(p.model), p.model), p.model, p.propulsion);
        ASSERT_FALSE(run.has_value()) << field;
        EXPECT_NE(run.error().context.find(field), std::string::npos) << run.error().context;
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
    p.propulsion.chain.current_max = 4.0;  // a motor that can't take full throttle
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
// declaration) and is refused by set_solids(), naming the part, before a
// step.
TEST(AnalysisLiveRun, APartThinnerThanItDeclaresIsRefusedByName) {
    const Powered p = with_ball_body(0.005f, 0.1f);
    const spade::Result<an::LiveRun> run =
        an::LiveRun::start(resolved(solid_case(p.model), p.model), p.model, p.propulsion);
    ASSERT_FALSE(run.has_value());
    EXPECT_NE(run.error().context.find("'body'"), std::string::npos) << run.error().context;
    EXPECT_NE(run.error().context.find("the model's parts as solids"), std::string::npos) << run.error().context;
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
    spade::Result<an::LiveRun> back = an::LiveRun::restore(file, resolved(c, p.model), p.model, p.propulsion);
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
