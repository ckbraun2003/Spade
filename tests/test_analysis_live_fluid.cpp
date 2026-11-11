// LiveRun's fluid on a device (B7's seam; Rendering's design note,
// docs/design/rendering/plans/2026-10-27-run-air-on-gpu.md, slice 1): a host
// hands LiveRun::start() a LiveFluid maker, and the run's fluid steps there.
// The device here is the CPU's own step behind the seam (CpuTwin), so a run
// handed it must fly to the bit as one that isn't: the seam then carries all
// a device needs. Slice 2's GpuFluid device is held to the same test.
//
//   AnalysisLiveFluid  the twin's run against the run's own, frame by frame
//                      and in its save; what a device starts from; a
//                      device's refusal and failure, by name; the run's
//                      subgrid-viscosity account (LiveSgs) against the sums
//                      of the health the device reported each step; the
//                      off-centre hold (ResolvedCase::hold_from_centre, L63).

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "analysis_powered_quad.hpp"
#include "core/flow_volume.hpp"
#include "live_sgs_probe.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_part.hpp"
#include "world/sdf.hpp"

namespace {

namespace an = spade::analysis;
namespace af = spade::physics::airflow;
namespace fs = std::filesystem;
using Powered = an::test::PoweredQuad;

// What the twin did, read by the test after the run has moved it.
struct TwinLog {
    uint64_t steps = 0;
    uint64_t publishes = 0;
    uint64_t fail_at = 0;  // the step it refuses, 1 for the first; 0 for none
};

// The CPU's fluid behind the seam: what a run does on its own, from what
// LiveRun hands a device.
class CpuTwin final : public an::LiveFluid {
  public:
    [[nodiscard]] static spade::Result<std::unique_ptr<an::LiveFluid>> make(const an::LiveFluidStart& s,
                                                                            TwinLog& log) {
        spade::Result<af::FluidScratch> scratch = af::make_fluid_scratch(s.grid, s.domain, s.config);
        if (!scratch) return std::unexpected(scratch.error());
        return std::unique_ptr<an::LiveFluid>(new CpuTwin(s, std::move(*scratch), log));
    }

    [[nodiscard]] spade::Result<af::FluidHealth> step(std::span<const af::ActuatorDisc> discs) override {
        if (log_.steps + 1 == log_.fail_at) {
            return std::unexpected(spade::Error{spade::Code::internal, "the twin's refusal"});
        }
        for (const af::ActuatorDisc& d : discs) {
            af::spread_actuator(d, grid_.shape, config_.rho, config_.h, scratch_.du, scratch_.dv, scratch_.dw);
        }
        ++log_.steps;
        return af::fluid_step(grid_, domain_, config_, scratch_);
    }

    [[nodiscard]] spade::Result<void> publish(const af::PublishInput& in, spade::FlowVolume& out) override {
        ++log_.publishes;
        const spade::Result<af::PublishCounts> published = af::publish_render_volume(grid_, in, out);
        if (!published) return std::unexpected(published.error());
        return {};
    }

    [[nodiscard]] spade::Result<void> read(af::MacGrid& g) const override {
        g = grid_;
        return {};
    }

  private:
    CpuTwin(const an::LiveFluidStart& s, af::FluidScratch scratch, TwinLog& log)
        : grid_(s.grid), domain_(s.domain), config_(s.config), scratch_(std::move(scratch)), log_(log) {}

    af::MacGrid grid_;
    af::DomainBc domain_;
    af::FluidConfig config_;
    af::FluidScratch scratch_;
    TwinLog& log_;
};

[[nodiscard]] an::LiveFluidMaker twin(TwinLog& log) {
    return [&log](const an::LiveFluidStart& s) { return CpuTwin::make(s, log); };
}

// The powered reference quad in a 3 m/s wind through a 0.8 m box at 5 cm
// (16^3), stepped every 4 ms: the discs, the faces' inflow and the hold all
// reach the fluid, small enough for debug ctest's budget.
[[nodiscard]] an::ResolvedCase windy(const Powered& p, float spacing = 0.05f, float step = 0.004f) {
    an::AnalysisCase c;
    c.model = an::pin_model("quad.model.yaml", p.model);
    c.power = p.power;
    c.grid.box = 0.8f;
    c.grid.spacing = spacing;
    c.time.duration = 1.0f;
    c.time.step = step;
    c.flight.kind = an::FlightKind::forward;
    c.flight.speed = 3.0f;
    c.flight.heading_deg = 20.0f;
    c.rotors.assign(p.model.rotors.size(), an::RotorSetup{});
    const auto r = an::resolve(c, p.model);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().front().field + ": " + r.error().front().message);
    return r ? *r : an::ResolvedCase{};
}

// Armed, the throttle and the attitude changing, so the discs change too.
[[nodiscard]] an::ControlProgram changing() {
    return an::ControlProgram{{{0.0f, an::Controls{true, 0.5f, 0.0f, 0.0f, 0.0f}},
                               {0.06f, an::Controls{true, 0.7f, 20.0f, 0.0f, 0.0f}},
                               {0.15f, an::Controls{true, 0.4f, 20.0f, -15.0f, 10.0f}}}};
}

[[nodiscard]] an::LiveRun started(const an::ResolvedCase& r, const spade::vehicles::ModelType& m,
                                  const an::LiveFluidMaker& fluid) {
    spade::Result<an::LiveRun> run = an::LiveRun::start(r, m, fluid);
    EXPECT_TRUE(run.has_value()) << (run ? "" : run.error().context);
    return std::move(*run);
}

[[nodiscard]] std::size_t differing(const std::vector<uint16_t>& a, const std::vector<uint16_t>& b) {
    if (a.size() != b.size()) return std::max(a.size(), b.size());
    std::size_t n = 0;
    for (std::size_t i = 0; i < a.size(); ++i) n += a[i] != b[i] ? 1u : 0u;
    return n;
}

[[nodiscard]] std::vector<char> bytes(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// The powered quad with one body part, a 20 cm ball, in the air at 4 cm.
[[nodiscard]] Powered with_ball_body() {
    Powered p = an::test::powered_quad();
    spade::vehicles::PartDesc ball;
    ball.name = "body";
    ball.shape.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode n{};
    n.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.params = glm::vec4(0.1f, 0.0f, 0.0f, 0.0f);
    ball.shape.nodes = {n};
    ball.min_feature = 0.2f;
    p.model.parts = {ball};
    return p;
}

}  // namespace

// Over 60 steps of a changing program, every frame's halves, the rotors and
// the save (u, v, w and p read back from the device) are the run's own, to
// the bit; the twin took every step and every publish.
TEST(AnalysisLiveFluid, ACpuDeviceFliesToTheBitAsTheRunsOwn) {
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = windy(p);
    TwinLog log;
    an::LiveRun own = started(r, p.model, {});
    an::LiveRun seam = started(r, p.model, twin(log));
    const an::ControlProgram program = changing();
    spade::FlowVolume a;
    spade::FlowVolume b;
    while (own.state().steps < 60) {
        const an::Controls c = an::controls_at(program, static_cast<float>(own.state().t_s));
        const spade::Result<an::LiveFrame> fa = own.advance_frame(c, a);
        const spade::Result<an::LiveFrame> fb = seam.advance_frame(c, b);
        ASSERT_TRUE(fa.has_value()) << fa.error().context;
        ASSERT_TRUE(fb.has_value()) << fb.error().context;
        ASSERT_EQ(fa->steps, fb->steps);
        ASSERT_EQ(differing(a.v, b.v), 0u) << "frame " << fa->frame;
        ASSERT_EQ(differing(a.s, b.s), 0u) << "frame " << fa->frame;
        EXPECT_EQ(a.header.origin, b.header.origin);
    }
    EXPECT_EQ(log.steps, seam.state().steps);
    EXPECT_EQ(log.publishes, seam.state().frames);
    EXPECT_EQ(own.state().rotor_omega, seam.state().rotor_omega);
    const spade::test::TempDir dir("live-fluid");
    ASSERT_TRUE(own.save(dir.path() / "own.bin").has_value());
    ASSERT_TRUE(seam.save(dir.path() / "seam.bin").has_value());
    const std::vector<char> saved = bytes(dir.path() / "own.bin");
    EXPECT_FALSE(saved.empty());
    EXPECT_TRUE(saved == bytes(dir.path() / "seam.bin"));
}

// A device starts from the run's own first fields and config, and is handed
// no solids when the run has none.
TEST(AnalysisLiveFluid, TheDeviceStartsFromTheRunsOwnFields) {
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = windy(p);
    TwinLog log;
    af::GridShape shape{};
    af::FluidConfig config{};
    bool solids = true;
    const an::LiveFluidMaker capture = [&](const an::LiveFluidStart& s) {
        shape = s.grid.shape;
        config = s.config;
        solids = s.solids != nullptr;
        return CpuTwin::make(s, log);
    };
    const an::LiveRun run = started(r, p.model, capture);
    EXPECT_EQ(shape.nx, r.base.nx);
    EXPECT_EQ(shape.ny, r.base.ny);
    EXPECT_EQ(shape.nz, r.base.nz);
    EXPECT_EQ(shape.dx, r.base.dx);
    EXPECT_EQ(config.h, static_cast<float>(run.step_s()));
    EXPECT_EQ(config.rho, r.spec.fluid.density);
    EXPECT_EQ(config.nu, r.spec.fluid.kinematic_viscosity);
    EXPECT_FALSE(solids);
}

// A run with its parts in the air hands the device their SolidField.
TEST(AnalysisLiveFluid, TheDeviceIsHandedTheRunsSolids) {
    const Powered p = with_ball_body();
    const an::ResolvedCase r = windy(p, 0.04f, 0.002f);
    TwinLog log;
    bool any = false;
    const an::LiveFluidMaker capture = [&](const an::LiveFluidStart& s) {
        any = s.solids != nullptr && s.solids->any;
        return CpuTwin::make(s, log);
    };
    const an::LiveRun run = started(r, p.model, capture);
    EXPECT_TRUE(run.solids().any);
    EXPECT_TRUE(any);
}

// A device's refusal refuses the run, by name: a host that would rather fly
// on the CPU starts again without it.
TEST(AnalysisLiveFluid, ADevicesRefusalRefusesTheRunByName) {
    const Powered p = an::test::powered_quad();
    const an::LiveFluidMaker refusing =
        [](const an::LiveFluidStart&) -> spade::Result<std::unique_ptr<an::LiveFluid>> {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "no device here"});
    };
    const spade::Result<an::LiveRun> run = an::LiveRun::start(windy(p), p.model, refusing);
    ASSERT_FALSE(run.has_value());
    EXPECT_EQ(run.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(run.error().context, "the fluid's device: no device here");
}

// A step the device fails ends the run, naming the step and why, as a fluid
// fault does: every step after is refused the same, and the run isn't saved.
TEST(AnalysisLiveFluid, ADeviceFailureEndsTheRunNamingTheStep) {
    const Powered p = an::test::powered_quad();
    TwinLog log;
    log.fail_at = 3;
    an::LiveRun run = started(windy(p), p.model, twin(log));
    const an::Controls armed{true, 0.5f, 0.0f, 0.0f, 0.0f};
    ASSERT_TRUE(run.step(armed).has_value());
    ASSERT_TRUE(run.step(armed).has_value());
    const spade::Result<void> third = run.step(armed);
    ASSERT_FALSE(third.has_value());
    EXPECT_EQ(third.error().context, "the fluid's device failed at step 3: the twin's refusal");
    const spade::Result<void> fourth = run.step(armed);
    ASSERT_FALSE(fourth.has_value());
    EXPECT_EQ(fourth.error().context, third.error().context);
    EXPECT_EQ(run.state().steps, 2u);
    const spade::test::TempDir dir("live-fluid-fault");
    EXPECT_FALSE(run.save(dir.path() / "run.bin").has_value());
}

// LiveRun::sgs() sums what the device's health reports each step: capped cell-steps and limited face-steps, the
// steps that had any of either, and the largest nu_t. The sgs fields are scripted here (12 steps, a script of 6
// repeating), so the numbers are exact whatever the flow binds.
TEST(AnalysisLiveFluid, TheSgsAccountSumsWhatTheDeviceReportsEveryStep) {
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = windy(p);
    const std::vector<spade::test::ScriptedSgs> script{{0, 0, 0.0f},    {3, 0, 0.0010f}, {0, 100, 0.0030f},
                                                       {5, 200, 0.0020f}, {0, 0, 0.0015f}, {2, 300, 0.0031f}};
    TwinLog log;
    spade::test::SgsSums sums;
    an::LiveRun run = started(r, p.model, spade::test::sgs_probe(twin(log), 0.0f, sums, script));
    for (int i = 0; i < 12; ++i) ASSERT_TRUE(run.step(an::Controls{}).has_value());
    const an::LiveSgs& s = run.sgs();
    EXPECT_EQ(s.capped_cell_steps, 20u);
    EXPECT_EQ(s.limited_face_steps, 1200u);
    EXPECT_EQ(s.capped_steps, 6u);
    EXPECT_EQ(s.limited_steps, 6u);
    EXPECT_EQ(s.nu_t_max, 0.0031f);
    EXPECT_EQ(sums.capped_cell_steps, s.capped_cell_steps) << "the probe's own sums are the oracle";
    EXPECT_EQ(sums.limited_face_steps, s.limited_face_steps);
    EXPECT_EQ(an::sgs_health_line(s),
              "subgrid nu_t up to 0.0031 m2/s; held at the cap on 6 steps (20 cell-steps), wall-limited on 6 steps "
              "(1,200 face-steps)");
}

// The same, with the subgrid viscosity really on (the maker sets Cs 0.17 as a host does): whatever the flow binds,
// the run's account is the sum of the health the fluid reported, and the term ran.
TEST(AnalysisLiveFluid, ARealSubgridViscosityIsAccountedAsTheFluidReportedIt) {
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = windy(p);
    TwinLog log;
    spade::test::SgsSums sums;
    an::LiveRun run = started(r, p.model, spade::test::sgs_probe(twin(log), 0.17f, sums));
    const an::ControlProgram program = changing();
    while (run.state().steps < 60) {
        ASSERT_TRUE(run.step(an::controls_at(program, static_cast<float>(run.state().t_s))).has_value());
    }
    const an::LiveSgs& s = run.sgs();
    ASSERT_EQ(sums.steps, 60u);
    EXPECT_GT(sums.nu_t_max, 0.0f) << "the subgrid viscosity ran";
    EXPECT_EQ(s.nu_t_max, sums.nu_t_max);
    EXPECT_EQ(s.capped_cell_steps, sums.capped_cell_steps);
    EXPECT_EQ(s.limited_face_steps, sums.limited_face_steps);
    EXPECT_EQ(s.capped_steps, sums.capped_steps);
    EXPECT_EQ(s.limited_steps, sums.limited_steps);
    std::printf("[ sgs     ] 60 steps at Cs 0.17: nu_t up to %.4g m2/s; capped %llu cell-steps on %llu steps; limited "
                "%llu face-steps on %llu steps; line '%s'\n",
                static_cast<double>(s.nu_t_max), static_cast<unsigned long long>(s.capped_cell_steps),
                static_cast<unsigned long long>(s.capped_steps), static_cast<unsigned long long>(s.limited_face_steps),
                static_cast<unsigned long long>(s.limited_steps), an::sgs_health_line(s).c_str());
}

// At Cs 0 the device reports 0 and the account stays 0, on the device's path as on the run's own.
TEST(AnalysisLiveFluid, ADeviceAtCsZeroKeepsAnEmptySgsAccount) {
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = windy(p);
    TwinLog log;
    spade::test::SgsSums sums;
    an::LiveRun run = started(r, p.model, spade::test::sgs_probe(twin(log), 0.0f, sums));
    for (int i = 0; i < 20; ++i) ASSERT_TRUE(run.step(an::Controls{true, 0.5f, 0.0f, 0.0f, 0.0f}).has_value());
    EXPECT_EQ(sums.capped_cell_steps + sums.limited_face_steps, 0u);
    const an::LiveSgs& s = run.sgs();
    EXPECT_EQ(s.capped_cell_steps + s.limited_face_steps + s.capped_steps + s.limited_steps, 0u);
    EXPECT_EQ(s.nu_t_max, 0.0f);
    EXPECT_EQ(an::sgs_health_line(s), "");
}

// The account is health, not state: a save does not carry it (the solids' counters aren't carried either), and a
// restored run starts it at zero, the run's own fluid on the CPU stepping on at Cs 0. Documented in live_run.hpp.
TEST(AnalysisLiveFluid, ARestoredRunStartsItsSgsAccountAtZero) {
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = windy(p);
    const std::vector<spade::test::ScriptedSgs> script{{4, 50, 0.0020f}};
    TwinLog log;
    spade::test::SgsSums sums;
    an::LiveRun run = started(r, p.model, spade::test::sgs_probe(twin(log), 0.0f, sums, script));
    for (int i = 0; i < 6; ++i) ASSERT_TRUE(run.step(an::Controls{}).has_value());
    ASSERT_EQ(run.sgs().capped_cell_steps, 24u);
    const spade::test::TempDir dir("live-sgs");
    ASSERT_TRUE(run.save(dir.path() / "run.bin").has_value());
    spade::Result<an::LiveRun> restored = an::LiveRun::restore(dir.path() / "run.bin", r, p.model);
    ASSERT_TRUE(restored.has_value()) << restored.error().context;
    EXPECT_EQ(restored->sgs().capped_cell_steps, 0u);
    EXPECT_EQ(restored->sgs().limited_face_steps, 0u);
    EXPECT_EQ(restored->sgs().nu_t_max, 0.0f);
    EXPECT_EQ(an::sgs_health_line(restored->sgs()), "");
}

// ---------------------------------------------------------------------------
// ResolvedCase::hold_from_centre (L63; Cameron's deeper box for the website wake). The drone is held at the world
// origin; the box's centre sits at -hold from it, so a hold of +y puts the drone ABOVE the box's centre and the box's
// minimum corner at -0.5 dx n - hold. 0, the default, is the centred run, to the bit.
// ---------------------------------------------------------------------------
namespace {

// A LiveFluid that forwards to another and keeps the discs of its last step, in the grid's frame.
class DiscTap final : public an::LiveFluid {
  public:
    DiscTap(std::unique_ptr<an::LiveFluid> inner, std::vector<af::ActuatorDisc>& seen)
        : inner_(std::move(inner)), seen_(seen) {}
    [[nodiscard]] spade::Result<af::FluidHealth> step(std::span<const af::ActuatorDisc> discs) override {
        seen_.assign(discs.begin(), discs.end());
        return inner_->step(discs);
    }
    [[nodiscard]] spade::Result<void> publish(const af::PublishInput& in, spade::FlowVolume& out) override {
        return inner_->publish(in, out);
    }
    [[nodiscard]] spade::Result<void> read(af::MacGrid& g) const override { return inner_->read(g); }

  private:
    std::unique_ptr<an::LiveFluid> inner_;
    std::vector<af::ActuatorDisc>& seen_;
};

[[nodiscard]] an::LiveFluidMaker tapped(TwinLog& log, std::vector<af::ActuatorDisc>& seen) {
    return [&log, &seen](const an::LiveFluidStart& s) -> spade::Result<std::unique_ptr<an::LiveFluid>> {
        spade::Result<std::unique_ptr<an::LiveFluid>> inner = CpuTwin::make(s, log);
        if (!inner) return inner;
        return std::unique_ptr<an::LiveFluid>(new DiscTap(std::move(*inner), seen));
    };
}

// Still air, 32 x 64 x 32 cells at 25 mm (a 0.8 x 1.6 x 0.8 m box), the 4 ms step: the quad's rotors and a 20 cm
// ball in the air, the drone small in a tall box. `hold` is the case's.
[[nodiscard]] an::ResolvedCase tall(const Powered& p, const glm::vec3& hold) {
    an::AnalysisCase c;
    c.model = an::pin_model("quad.model.yaml", p.model);
    c.power = p.power;
    c.grid.box = 0.8f;
    c.grid.spacing = 0.025f;
    c.time.duration = 1.0f;
    c.time.step = 0.004f;
    c.rotors.assign(p.model.rotors.size(), an::RotorSetup{});
    const auto r = an::resolve(c, p.model);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().front().field + ": " + r.error().front().message);
    an::ResolvedCase out = r ? *r : an::ResolvedCase{};
    out.base = af::GridShape{32, 64, 32, 0.025f};
    out.base_bytes = af::fluid_bytes(out.base, out.domain);
    out.hold_from_centre = hold;
    return out;
}

}  // namespace

// The default hold is 0, and a run at it publishes the box centred on the drone, the minimum corner at
// -0.5 dx n (the same float operations as before the field), over every frame.
TEST(AnalysisLiveFluid, TheDefaultHoldIsZeroAndTheBoxIsCentred) {
    EXPECT_EQ(an::ResolvedCase{}.hold_from_centre, glm::vec3(0.0f));
    const Powered p = with_ball_body();
    const an::ResolvedCase r = tall(p, glm::vec3(0.0f));
    an::LiveRun run = started(r, p.model, {});
    spade::FlowVolume v;
    ASSERT_TRUE(run.advance_frame(an::Controls{true, 0.5f, 0.0f, 0.0f, 0.0f}, v).has_value());
    const glm::vec3 centred = -0.5f * 0.025f * glm::vec3(32.0f, 64.0f, 32.0f);
    EXPECT_EQ(v.header.origin, centred);
}

// A hold of (0, 0.4, 0) m (16 cells up): the box's minimum corner is at -0.5 dx n - hold, the drone is 16 cells
// higher in the grid's frame, every rotor disc's centre is the centred run's plus the hold (the grid's frame has
// moved by -hold), and the ball in the air is the same cells shifted by the 16 (the same solid count, 16 cells up).
TEST(AnalysisLiveFluid, AHoldPutsTheDroneHigherInTheBoxAndMovesEverythingWithIt) {
    const Powered p = with_ball_body();
    const glm::vec3 hold(0.0f, 0.4f, 0.0f);
    TwinLog log_a;
    TwinLog log_b;
    std::vector<af::ActuatorDisc> centred_discs;
    std::vector<af::ActuatorDisc> held_discs;
    an::LiveRun centred = started(tall(p, glm::vec3(0.0f)), p.model, tapped(log_a, centred_discs));
    an::LiveRun held = started(tall(p, hold), p.model, tapped(log_b, held_discs));
    const an::Controls armed{true, 0.5f, 0.0f, 0.0f, 0.0f};
    spade::FlowVolume a;
    spade::FlowVolume b;
    ASSERT_TRUE(centred.advance_frame(armed, a).has_value());
    ASSERT_TRUE(held.advance_frame(armed, b).has_value());

    const glm::vec3 shape(32.0f, 64.0f, 32.0f);
    EXPECT_EQ(b.header.origin, -0.5f * 0.025f * shape - hold);
    EXPECT_EQ(a.header.origin.x, b.header.origin.x);
    EXPECT_EQ(a.header.origin.z, b.header.origin.z);
    EXPECT_NEAR(a.header.origin.y - b.header.origin.y, hold.y, 1.0e-6f);

    ASSERT_EQ(centred_discs.size(), p.model.rotors.size());
    ASSERT_EQ(held_discs.size(), centred_discs.size());
    for (std::size_t i = 0; i < held_discs.size(); ++i) {
        EXPECT_NEAR(held_discs[i].centre.x, centred_discs[i].centre.x + hold.x, 1.0e-5f) << "rotor " << i;
        EXPECT_NEAR(held_discs[i].centre.y, centred_discs[i].centre.y + hold.y, 1.0e-5f) << "rotor " << i;
        EXPECT_NEAR(held_discs[i].centre.z, centred_discs[i].centre.z + hold.z, 1.0e-5f) << "rotor " << i;
        EXPECT_EQ(held_discs[i].axis, centred_discs[i].axis) << "the disc's direction is the frame's, not moved";
        EXPECT_EQ(held_discs[i].radius, centred_discs[i].radius);
    }
    EXPECT_GT(centred.solids().solid_cells, 0u);
    EXPECT_EQ(held.solids().solid_cells, centred.solids().solid_cells) << "the ball's cells, 16 cells up";
}
