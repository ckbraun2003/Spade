// LiveRun's fluid on a device (B7's seam; Rendering's design note,
// docs/design/rendering/plans/2026-10-27-run-air-on-gpu.md, slice 1): a host
// hands LiveRun::start() a LiveFluid maker, and the run's fluid steps there.
// The device here is the CPU's own step behind the seam (CpuTwin), so a run
// handed it must fly to the bit as one that isn't: the seam then carries all
// a device needs. Slice 2's GpuFluid device is held to the same test.
//
//   AnalysisLiveFluid  the twin's run against the run's own, frame by frame
//                      and in its save; what a device starts from; a
//                      device's refusal and failure, by name.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
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
