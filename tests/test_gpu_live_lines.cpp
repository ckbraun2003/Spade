// ---------------------------------------------------------------------------
// test_gpu_live_lines.cpp -- L86i/L86j: LiveRun with the rotors as actuator lines (LiveLines; LiveRun::start()'s fourth argument,
// LiveFluid::step(discs, lines), spike/airflow/live_fluid.hpp's GpuLiveFluid and gpu_live_fluid()'s dye), on the powered reference quad
// held in still air on 64^3 at 12.5 mm, h 0.5 ms, its parts out of the air.
//
//   GpuLiveLines.ALineRunFliesOnTheDeviceWithItsDye    30 steps then a frame: the air moves, the dye is in the frame, the label says
//                                                      actuator line, the blade azimuths turn
//   GpuLiveLines.RefusesWhatItCannotMakeALineRunOfByName  no device, a blade that is not the rotors' radius, n_sub 0; a line run is not saved
//   GpuLiveLines.ALiveFluidWithNoLinesRefusesThem      the interface's default step(discs, lines)
//
// The GPU tests start with the vulkan_skip_reason() skip.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "analysis_powered_quad.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/flow_volume.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/blade.hpp"
#include "physics/airflow/dye.hpp"
#include "physics/airflow/line.hpp"
#include "spike/airflow/live_fluid.hpp"
#include "unique_temp.hpp"

namespace {

namespace an = spade::analysis;
namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
using Powered = an::test::PoweredQuad;
using spade::compute::VulkanContext;

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

[[nodiscard]] an::ResolvedCase small_case(const Powered& p) {
    an::AnalysisCase c;
    c.model = an::pin_model("quad.model.yaml", p.model);
    c.power = p.power;
    c.grid.box = 0.8f;
    c.grid.spacing = 0.0125f;
    c.time.duration = 1.0f;
    c.time.step = 0.0005f;
    c.rotors.assign(p.model.rotors.size(), an::RotorSetup{});
    const auto r = an::resolve(c, p.model);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().front().field + ": " + r.error().front().message);
    return r ? *r : an::ResolvedCase{};
}

[[nodiscard]] an::LiveLines lines_for(const Powered& p, float radius_scale = 1.0f) {
    const float radius = p.model.rotors.front().radius * radius_scale;
    an::LiveLines l;
    l.blade.blades = 2;
    l.blade.radius = radius;
    l.blade.stations = {{0.15f * radius, 0.1f * radius, 30.0f}, {0.6f * radius, 0.09f * radius, 18.0f}, {radius, 0.05f * radius, 10.0f}};
    l.polar = af::estimated_polar();
    l.n_sub = 4;
    return l;
}

TEST(GpuLiveLines, ALineRunFliesOnTheDeviceWithItsDye) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = small_case(p);
    const an::LiveLines lines = lines_for(p);
    const ag::DyeSetup dye{af::DyeRelease{}, af::TipRelease{.gain = 4.0f}};
    spade::Result<an::LiveRun> run = an::LiveRun::start(r, p.model, ag::gpu_live_fluid(*ctx, 64u, 0.17f, dye), &lines);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const an::Controls armed{true, 0.5f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 30; ++i) {
        const auto s = run->step(armed);
        ASSERT_TRUE(s.has_value()) << "step " << i + 1 << ": " << s.error().context;
    }
    spade::FlowVolume out;
    const auto frame = run->advance_steps(armed, 1, out);
    ASSERT_TRUE(frame.has_value()) << frame.error().context;
    float fastest = 0.0f;
    for (int c = 0; c < 3; ++c) {
        fastest = std::max({fastest, std::fabs(out.header.channel_min[c]), std::fabs(out.header.channel_max[c])});
    }
    EXPECT_GT(fastest, 0.05f) << "the lines move the air";
    EXPECT_GT(out.header.channel_max[spade::kFlowDye], 0.0f) << "the tips' and the rings' dye is in the frame";
    EXPECT_NE(run->run_label().find("actuator line"), std::string::npos) << run->run_label();
    const std::vector<float> psi(run->blade_azimuth().begin(), run->blade_azimuth().end());
    EXPECT_GT(*std::max_element(psi.begin(), psi.end()), 0.0f) << "the blades turned";
}

TEST(GpuLiveLines, RefusesWhatItCannotMakeALineRunOfByName) {
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = small_case(p);
    const an::LiveLines lines = lines_for(p);
    const auto no_device = an::LiveRun::start(r, p.model, {}, &lines);
    ASSERT_FALSE(no_device.has_value());
    EXPECT_NE(no_device.error().context.find("device"), std::string::npos) << no_device.error().context;
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const an::LiveLines other = lines_for(p, 0.5f);
    const auto radius = an::LiveRun::start(r, p.model, ag::gpu_live_fluid(*ctx), &other);
    ASSERT_FALSE(radius.has_value());
    EXPECT_NE(radius.error().context.find("radius"), std::string::npos) << radius.error().context;
    an::LiveLines none = lines;
    none.n_sub = 0;
    const auto n_sub = an::LiveRun::start(r, p.model, ag::gpu_live_fluid(*ctx), &none);
    ASSERT_FALSE(n_sub.has_value());
    EXPECT_NE(n_sub.error().context.find("n_sub"), std::string::npos) << n_sub.error().context;
    // L105a: a line run with no subgrid model (the maker's Cs 0, the default) is refused by name, by the device's own guard.
    const auto lean = an::LiveRun::start(r, p.model, ag::gpu_live_fluid(*ctx), &lines);
    ASSERT_FALSE(lean.has_value());
    EXPECT_NE(lean.error().context.find("actuator lines need a subgrid model"), std::string::npos) << lean.error().context;
    // A line run's save would restore as a disc run: refused, by name.
    spade::Result<an::LiveRun> run = an::LiveRun::start(r, p.model, ag::gpu_live_fluid(*ctx, 64u, 0.17f), &lines);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const spade::test::TempDir dir("live-lines-save");
    const auto saved = run->save(dir.path() / "run.save");
    ASSERT_FALSE(saved.has_value());
    EXPECT_NE(saved.error().context.find("line"), std::string::npos) << saved.error().context;
}

// A LiveFluid that only has discs.
class DiscsOnly final : public an::LiveFluid {
  public:
    spade::Result<af::FluidHealth> step(std::span<const af::ActuatorDisc>) override { return af::FluidHealth{}; }
    spade::Result<void> publish(const af::PublishInput&, spade::FlowVolume&) override { return {}; }
    spade::Result<void> read(af::MacGrid&) const override { return {}; }
};

TEST(GpuLiveLines, ALiveFluidWithNoLinesRefusesThem) {
    DiscsOnly fluid;
    an::LiveFluid& base = fluid;
    EXPECT_TRUE(base.step({}, {}).has_value()) << "no lines: the disc step";
    const af::BladeShape blade = lines_for(an::test::powered_quad()).blade;
    const af::PolarTable polar = af::estimated_polar();
    af::LineRotor line;
    line.blade = &blade;
    line.polar = &polar;
    const std::vector<af::LineRotor> one{line};
    const auto refused = base.step({}, one);
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("lines"), std::string::npos) << refused.error().context;
}

}  // namespace
