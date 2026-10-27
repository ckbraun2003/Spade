// ---------------------------------------------------------------------------
// test_gpu_live_fluid.cpp -- spike/airflow/live_fluid.hpp, LiveRun's fluid on
// B7's GpuFluid (Rendering's design note, docs/design/rendering/plans/
// 2026-10-27-run-air-on-gpu.md, slice 2), on the editor's Run air grid: the
// powered reference quad held in still air, 32^3 at 25 mm, stepped every
// 1 ms, its parts out of the air (Interface's air_case()).
//
//   GpuLiveFluid  60 steps of a changing program, the rotors' discs changing
//                 every step, the CPU's run to the bit on the device of
//                 record, frame by frame and in the save; a flow past the
//                 volume's range ends the run at its frame, naming it; the
//                 device's refusals (parts, a workgroup size, no device)
//                 fall back to the CPU by name, and the case's own don't;
//                 the frame's wall time on both, opt-in
//                 (SPADE_AIRFLOW_TIMINGS).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "airflow_exact_device.hpp"
#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "analysis_powered_quad.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/flow_volume.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "spike/airflow/live_fluid.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_part.hpp"
#include "world/sdf.hpp"

namespace {

namespace an = spade::analysis;
namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
namespace fs = std::filesystem;
using Powered = an::test::PoweredQuad;
using spade::compute::VulkanContext;

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

// The editor's Run air case for the powered reference quad: 32^3 at 25 mm
// (a 0.8 m box), h 1 ms, still air.
[[nodiscard]] an::ResolvedCase editor_case(const Powered& p) {
    an::AnalysisCase c;
    c.model = an::pin_model("quad.model.yaml", p.model);
    c.power = p.power;
    c.grid.box = 0.8f;
    c.grid.spacing = 0.025f;
    c.time.duration = 1.0f;
    c.time.step = 0.001f;
    c.rotors.assign(p.model.rotors.size(), an::RotorSetup{});
    const auto r = an::resolve(c, p.model);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().front().field + ": " + r.error().front().message);
    return r ? *r : an::ResolvedCase{};
}

// Armed, the throttle and the attitude changing: the rotors lag toward each
// command, so their discs change every step.
[[nodiscard]] an::ControlProgram changing() {
    return an::ControlProgram{{{0.0f, an::Controls{true, 0.5f, 0.0f, 0.0f, 0.0f}},
                               {0.02f, an::Controls{true, 0.7f, 20.0f, 0.0f, 0.0f}},
                               {0.04f, an::Controls{true, 0.4f, 20.0f, -15.0f, 10.0f}}}};
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

// The powered quad with one body part, a 20 cm ball: in the air at 25 mm.
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

// 60 steps of the changing program: every frame's halves, the rotors and the
// save (u, v, w and p read back from the device) are the CPU's run's, to the
// bit on the device of record; elsewhere the differing halves are printed.
TEST(GpuLiveFluid, FliesTheCpusRunToTheBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool exact = spade::testing::airflow_bit_equal_device(*ctx);
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = editor_case(p);
    an::LiveRun cpu = started(r, p.model, {});
    an::LiveRun gpu = started(r, p.model, ag::gpu_live_fluid(*ctx));
    const an::ControlProgram program = changing();
    spade::FlowVolume a;
    spade::FlowVolume b;
    std::size_t differ = 0;
    std::size_t halves = 0;
    while (cpu.state().steps < 60) {
        const an::Controls c = an::controls_at(program, static_cast<float>(cpu.state().t_s));
        const spade::Result<an::LiveFrame> fa = cpu.advance_frame(c, a);
        const spade::Result<an::LiveFrame> fb = gpu.advance_frame(c, b);
        ASSERT_TRUE(fa.has_value()) << fa.error().context;
        ASSERT_TRUE(fb.has_value()) << fb.error().context;
        ASSERT_EQ(fa->steps, fb->steps);
        const std::size_t d = differing(a.v, b.v) + differing(a.s, b.s);
        differ += d;
        halves += a.v.size() + a.s.size();
        if (exact) {
            ASSERT_EQ(d, 0u) << "frame " << fa->frame;
        }
        EXPECT_EQ(a.header.origin, b.header.origin);
    }
    std::printf("[ live    ] %llu steps on 32^3: %zu of %zu published halves differ (device of record: %s)\n",
                static_cast<unsigned long long>(gpu.state().steps), differ, halves, exact ? "yes" : "no");
    EXPECT_EQ(cpu.state().rotor_omega, gpu.state().rotor_omega);
    if (!exact) return;
    const spade::test::TempDir dir("gpu-live-fluid");
    ASSERT_TRUE(cpu.save(dir.path() / "cpu.bin").has_value());
    ASSERT_TRUE(gpu.save(dir.path() / "gpu.bin").has_value());
    const std::vector<char> saved = bytes(dir.path() / "cpu.bin");
    EXPECT_FALSE(saved.empty());
    EXPECT_TRUE(saved == bytes(dir.path() / "gpu.bin"));
}

// A flow past binary16's range (a poisoned start: 100 km/s through the box,
// finite, so the device steps it as the CPU does) reaches the first frame:
// its publish fails, naming the samples it can't hold, and every step after
// is refused the same, so the run ends there with no frame published. (A
// NaN start doesn't serve: the device's first step leaves no NaN where the
// CPU's faults, measured 2026-10-27.)
TEST(GpuLiveFluid, AFlowPastTheVolumesRangeEndsTheRunAtItsFrame) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const an::LiveFluidMaker gpu = ag::gpu_live_fluid(*ctx);
    const an::LiveFluidMaker poisoned = [&gpu](const an::LiveFluidStart& s) {
        af::MacGrid g = s.grid;
        std::fill(g.u.begin(), g.u.end(), 1.0e5f);
        return gpu(an::LiveFluidStart{g, s.domain, s.config, s.solids});
    };
    const Powered p = an::test::powered_quad();
    an::LiveRun run = started(editor_case(p), p.model, poisoned);
    const an::Controls armed{true, 0.5f, 0.0f, 0.0f, 0.0f};
    spade::FlowVolume v;
    const spade::Result<an::LiveFrame> frame = run.advance_frame(armed, v);
    ASSERT_FALSE(frame.has_value());
    EXPECT_NE(frame.error().context.find("the published flow holds "), std::string::npos) << frame.error().context;
    EXPECT_NE(frame.error().context.find(" non-finite samples, the first at cell ("), std::string::npos);
    EXPECT_EQ(run.state().frames, 0u);
    const uint64_t steps = run.state().steps;
    const spade::Result<void> next = run.step(armed);
    ASSERT_FALSE(next.has_value());
    EXPECT_EQ(next.error().context,
              "the fluid's device failed at step " + std::to_string(steps + 1) + ": " + frame.error().context);
}

// What the device refuses falls back to the CPU, the refusal kept by name:
// parts in the air (until G3), a workgroup size with no variant, and no
// device. A refusal of the case itself is start()'s, not a fallback.
TEST(GpuLiveFluid, ItsRefusalsFallBackToTheCpuByName) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = editor_case(p);

    const spade::Result<ag::LiveRunOn> on_gpu = ag::start_live_run(r, p.model, ctx.get());
    ASSERT_TRUE(on_gpu.has_value()) << on_gpu.error().context;
    EXPECT_EQ(on_gpu->cpu_because, "");

    const Powered ball = with_ball_body();
    const spade::Result<ag::LiveRunOn> parts = ag::start_live_run(editor_case(ball), ball.model, ctx.get());
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    EXPECT_EQ(parts->cpu_because, "GpuFluid: a SolidField with parts needs Physics' solids hooks (B7's G3)");
    EXPECT_TRUE(parts->run.solids().any);

    const spade::Result<ag::LiveRunOn> odd = ag::start_live_run(r, p.model, ctx.get(), 48);
    ASSERT_TRUE(odd.has_value()) << odd.error().context;
    EXPECT_NE(odd->cpu_because.find("has no variant compiled for workgroup size 48"), std::string::npos)
        << odd->cpu_because;

    const spade::Result<ag::LiveRunOn> none = ag::start_live_run(r, p.model, nullptr);
    ASSERT_TRUE(none.has_value()) << none.error().context;
    EXPECT_EQ(none->cpu_because, "no Vulkan device");

    an::ResolvedCase stepless = r;
    stepless.spec.time.step.reset();
    const spade::Result<ag::LiveRunOn> refused = ag::start_live_run(stepless, p.model, ctx.get());
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("a live run needs the case's time.step"), std::string::npos)
        << refused.error().context;
}

// The whole LiveRun frame (the drone, its discs, the fluid and the publish)
// on the editor's grid, GPU beside CPU: the wall time a frame, and the share
// of real time it runs at (a frame is 1/30 s).
TEST(GpuLiveFluid, FrameTimeOnTheEditorsGrid) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const bool opted = std::getenv("SPADE_AIRFLOW_TIMINGS") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    if (!opted) GTEST_SKIP() << "opt-in: SPADE_AIRFLOW_TIMINGS";
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const Powered p = an::test::powered_quad();
    const an::ResolvedCase r = editor_case(p);
    const an::ControlProgram program = changing();
    using Clock = std::chrono::steady_clock;
    const auto time_frames = [&](an::LiveRun& run, int frames) {
        spade::FlowVolume v;
        const an::Controls first = an::controls_at(program, 0.0f);
        EXPECT_TRUE(run.advance_frame(first, v).has_value());  // a warm-up frame
        const Clock::time_point t0 = Clock::now();
        for (int k = 0; k < frames; ++k) {
            const an::Controls c = an::controls_at(program, static_cast<float>(run.state().t_s));
            EXPECT_TRUE(run.advance_frame(c, v).has_value());
        }
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / frames;
    };
    an::LiveRun gpu = started(r, p.model, ag::gpu_live_fluid(*ctx));
    an::LiveRun cpu = started(r, p.model, {});
    const double gpu_ms = time_frames(gpu, 30);
    const double cpu_ms = time_frames(cpu, 3);
    const double frame_ms = 1000.0 / 30.0;
    std::printf("measure: LiveRun frame on 32^3 at 25 mm, h 1 ms, 4 discs: GPU %.1f ms (1/%.1f of real time), "
                "CPU %.1f ms (1/%.1f of real time): %.1fx\n",
                gpu_ms, gpu_ms / frame_ms, cpu_ms, cpu_ms / frame_ms, cpu_ms / gpu_ms);
}
