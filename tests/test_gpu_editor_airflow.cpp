// Run air on the GPU (Rendering's design note, docs/design/rendering/plans/
// 2026-10-27-run-air-on-gpu.md, slice 3; the lead, 2026-10-27): the
// editor's AirflowRun started through gpu_air_starter()
// (sandbox/editor_air_gpu.hpp) runs on the device and says so, the CPU's run
// to the bit on the device of record; a device's refusal runs on the CPU,
// naming it verbatim; a frame past ten times the top speed, finite and in
// binary16's range, is stopped by the bound at its frame; the placed
// Phantom's air with its body on the device, the CPU's to the bit; and with
// no device or a refusal, the discs case on the CPU, saying the body needs
// the GPU.
// GpuEditorAirflow* on the device.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <utility>

#include <glm/glm.hpp>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "spike/airflow/live_fluid.hpp"

#include "../sandbox/editor_air_gpu.hpp"
#include "../sandbox/editor_scene_edits.hpp"
#include "editor_test_support.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
namespace an = spade::analysis;
namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
using spade::Result;
using spade::compute::VulkanContext;

constexpr an::Controls kHalf{true, 0.5f, 0.0f, 0.0f, 0.0f};

// The library drone `file` placed at (1, 2, 3) as `name`, with its region:
// the editor's own cell (Run air's rule), or `cells` when given.
[[nodiscard]] spade::scene::SceneDesc with_library_drone(const char* file, const char* name,
                                                         std::optional<af::GridShape> cells = std::nullopt) {
    auto d = ed::make_scene_document(support::sample_scene(), {});
    ed::AddModelFile add;
    add.file = (std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / file).generic_string();
    add.vehicle.name = name;
    add.vehicle.start.pos = glm::vec3(1.0f, 2.0f, 3.0f);
    const Result<void> added = ed::apply(d, add);
    EXPECT_TRUE(added.has_value()) << (added ? "" : added.error().context);
    const Result<void> region = ed::apply(d, ed::AddAirflowRegion{name});
    EXPECT_TRUE(region.has_value()) << (region ? "" : region.error().context);
    spade::scene::SceneDesc s = d.desc();
    if (cells && !s.airflow.empty()) s.airflow.back().cells = *cells;
    return s;
}

// The reference quad at its cell before item 1, 32^3 at 25 mm: every part
// is drawn only there, so the run is discs only, the CPU's case to the bit.
[[nodiscard]] spade::scene::SceneDesc with_reference_quad() {
    return with_library_drone("reference_quad.model.yaml", "ref", af::GridShape{32, 32, 32, 0.025f});
}

[[nodiscard]] std::shared_ptr<VulkanContext> make_context() {
    auto made = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!made.has_value()) return nullptr;
    return std::move(*made);
}

// The run's latest frame once it has published `n` frames or stopped on an
// error; at most 50 s (debug ctest's limit is 60).
[[nodiscard]] std::shared_ptr<const ed::AirFrame> wait_for_frames(const ed::AirflowRun& run, uint64_t n) {
    for (int i = 0; i < 1000; ++i) {
        std::shared_ptr<const ed::AirFrame> f = run.latest();
        if (f && (f->frames >= n || !f->error.empty())) return f;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return run.latest();
}

// `frame` against a one-thread LiveRun on the CPU of the region's case
// `which` (the body's, the device's; or the discs', the CPU's fallback) and
// the same program: the same steps, time and halves, to the bit.
void expect_cpus_frame(const spade::scene::SceneDesc& s, const spade::scene::SceneAirflowRegion& region,
                       const an::ControlProgram& program, const ed::AirFrame& frame,
                       ed::AirCase ed::AirCases::*which = &ed::AirCases::body) {
    const Result<ed::AirCases> cases = ed::air_cases(s, region);
    ASSERT_TRUE(cases.has_value()) << cases.error().context;
    const ed::AirCase& flown = (*cases).*which;
    Result<an::LiveRun> one = an::LiveRun::start(flown.resolved, flown.model);
    ASSERT_TRUE(one.has_value()) << one.error().context;
    spade::FlowVolume v;
    while (one->state().frames < frame.frames) {
        const an::Controls c = an::controls_at(program, static_cast<float>(one->state().t_s));
        ASSERT_TRUE(one->advance_frame(c, v).has_value());
    }
    EXPECT_EQ(frame.steps, one->state().steps);
    EXPECT_EQ(frame.run_label, one->run_label());
    EXPECT_TRUE(frame.volume.v == v.v) << "the velocities differ from the CPU's";
    EXPECT_TRUE(frame.volume.s == v.s) << "the scalars differ from the CPU's";
}

// The device's fluid, stepping as ever, whose every publish has cell (1, 2, 3)
// at 30000 m/s: finite, in binary16's range, and past any drone's bound.
class FastCell final : public an::LiveFluid {
  public:
    explicit FastCell(std::unique_ptr<an::LiveFluid> device) : device_(std::move(device)) {}

    [[nodiscard]] Result<af::FluidHealth> step(std::span<const af::ActuatorDisc> discs) override {
        return device_->step(discs);
    }

    [[nodiscard]] Result<void> publish(const af::PublishInput& in, spade::FlowVolume& out) override {
        if (Result<void> p = device_->publish(in, out); !p) return p;
        const std::size_t at = spade::flow_cell_index(out.header, 1, 2, 3) * spade::kFlowHalvesPerCell;
        out.v[at + spade::kFlowU] = spade::fp16::from_f32(30000.0f).bits;
        return {};
    }

    [[nodiscard]] Result<void> read(af::MacGrid& g) const override { return device_->read(g); }

  private:
    std::unique_ptr<an::LiveFluid> device_;
};

}  // namespace

// On the device: the run says "GPU", and its first two frames (67 steps,
// the Pilot's controls held) are the CPU's to the bit on the device of
// record (slice 2's pin, through the editor's run).
TEST(GpuEditorAirflow, RunAirOnTheDeviceSaysSoAndIsTheCpusRunToTheBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::scene::SceneDesc s = with_reference_quad();
    ASSERT_FALSE(s.airflow.empty());
    const an::ControlProgram program{{{0.0f, kHalf}}};
    Result<std::unique_ptr<ed::AirflowRun>> run =
        ed::AirflowRun::start(s, s.airflow.back(), program, {}, ed::gpu_air_starter());
    ASSERT_TRUE(run.has_value()) << run.error().context;
    EXPECT_EQ((*run)->backend(), "GPU");
    const std::shared_ptr<const ed::AirFrame> frame = wait_for_frames(**run, 2);
    (*run)->stop();
    ASSERT_NE(frame, nullptr);
    ASSERT_TRUE(frame->error.empty()) << frame->error;
    ASSERT_GE(frame->frames, 2u);
    expect_cpus_frame(s, s.airflow.back(), program, *frame);
}

// The library's Phantom placed with its region (Test/Docs' gap 4; the lead,
// 2026-11-03), its body in the air (item 1): at Run air's 12 mm on 48^3
// (0.576 m, its 0.49 m span inside; the region's own 104^3 is Rendering's
// measure), on the device the run says "GPU", its label names the body in
// the air and what is drawn only, and its steps, label and published halves
// after a frame are a one-thread CPU run of the same body case's, to the bit.
TEST(GpuEditorAirflow, ThePlacedPhantomsAirOnTheDeviceIsTheCpusToTheBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::scene::SceneDesc s =
        with_library_drone("phantom3_class.model.yaml", "phantom", af::GridShape{48, 48, 48, 0.012f});
    ASSERT_EQ(s.airflow.size(), 1u);
    const spade::scene::SceneAirflowRegion& region = s.airflow.back();
    const Result<spade::vehicles::ModelType> flown = ed::air_model(s, region);
    ASSERT_TRUE(flown.has_value()) << flown.error().context;
    EXPECT_FALSE(flown->parts.empty()) << "the body's case keeps the meshed body and props";
    EXPECT_EQ(flown->rotors.size(), 4u);

    const an::ControlProgram program{{{0.0f, kHalf}}};
    Result<std::unique_ptr<ed::AirflowRun>> run = ed::AirflowRun::start(s, region, program, {}, ed::gpu_air_starter());
    ASSERT_TRUE(run.has_value()) << run.error().context;
    EXPECT_EQ((*run)->backend(), "GPU");
    const std::shared_ptr<const ed::AirFrame> frame = wait_for_frames(**run, 1);
    (*run)->stop();
    ASSERT_NE(frame, nullptr);
    ASSERT_TRUE(frame->error.empty()) << frame->error;
    ASSERT_GE(frame->frames, 1u);
    EXPECT_NE(frame->run_label.find("solids: 1 parts, 1 walls, 0 sheets"), std::string::npos) << frame->run_label;
    EXPECT_NE(frame->run_label.find("drawn only at 12 mm: "), std::string::npos) << frame->run_label;
    expect_cpus_frame(s, region, program, *frame);
}

// With no device, start_live_run() starts the CPU case it is given, and
// says why: the editor's discs case for a body the CPU can't fly live.
TEST(GpuEditorAirflow, WithNoDeviceStartLiveRunStartsTheCpuCaseItIsGiven) {
    const spade::scene::SceneDesc s = with_library_drone("phantom3_class.model.yaml", "phantom");
    ASSERT_FALSE(s.airflow.empty());
    const Result<ed::AirCases> cases = ed::air_cases(s, s.airflow.back());
    ASSERT_TRUE(cases.has_value()) << cases.error().context;
    ASSERT_NE(ed::grid_line(cases->body.resolved), ed::grid_line(cases->discs.resolved));
    const Result<ag::LiveRunOn> on = ag::start_live_run(cases->body.resolved, cases->body.model, nullptr, 64,
                                                        ag::CpuCase{&cases->discs.resolved, &cases->discs.model});
    ASSERT_TRUE(on.has_value()) << on.error().context;
    EXPECT_EQ(on->cpu_because, "no Vulkan device");
    Result<an::LiveRun> discs = an::LiveRun::start(cases->discs.resolved, cases->discs.model);
    ASSERT_TRUE(discs.has_value()) << discs.error().context;
    EXPECT_EQ(on->run.run_label(), discs->run_label());
}

// A device that refuses the body's case: the run is the discs case on the
// CPU, and its backend names the refusal and that the body needs the GPU.
TEST(GpuEditorAirflow, ADevicesRefusalOfTheBodyFliesItsDiscsOnTheCpuAndSaysSo) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::scene::SceneDesc s =
        with_library_drone("phantom3_class.model.yaml", "phantom", af::GridShape{48, 48, 48, 0.012f});
    ASSERT_FALSE(s.airflow.empty());
    const Result<ed::AirCases> cases = ed::air_cases(s, s.airflow.back());
    ASSERT_TRUE(cases.has_value()) << cases.error().context;
    ASSERT_TRUE(cases->loses_parts);
    constexpr uint32_t kNoVariant = 7;
    Result<ed::AirStart> started = ed::gpu_air_starter(kNoVariant)(*cases);
    ASSERT_TRUE(started.has_value()) << started.error().context;
    EXPECT_TRUE(started->backend.starts_with("CPU: ")) << started->backend;
    EXPECT_TRUE(started->backend.ends_with("; discs only, the body needs the GPU")) << started->backend;
    Result<an::LiveRun> discs = an::LiveRun::start(cases->discs.resolved, cases->discs.model);
    ASSERT_TRUE(discs.has_value()) << discs.error().context;
    EXPECT_EQ(started->run.run_label(), discs->run_label());
}

// A device that refuses the case (here a workgroup size with no kernel
// variant; after Rendering's NaN-preserve fix, a device without the
// feature): the run is on the CPU, its discs case to the bit (at 25 mm every
// reference-quad part is drawn only, so the backend says nothing of the
// body), and its backend is "CPU: " and the refusal verbatim, as
// start_live_run() kept it.
TEST(GpuEditorAirflow, ADevicesRefusalRunsOnTheCpuNamingIt) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::scene::SceneDesc s = with_reference_quad();
    ASSERT_FALSE(s.airflow.empty());
    const spade::scene::SceneAirflowRegion& region = s.airflow.back();
    const Result<an::ResolvedCase> resolved = ed::air_case(s, region);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    const Result<spade::vehicles::ModelType> model = ed::air_model(s, region);
    ASSERT_TRUE(model.has_value()) << model.error().context;
    const std::shared_ptr<VulkanContext> ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    constexpr uint32_t kNoVariant = 7;
    const Result<ag::LiveRunOn> direct = ag::start_live_run(*resolved, *model, ctx.get(), kNoVariant);
    ASSERT_TRUE(direct.has_value()) << direct.error().context;
    ASSERT_FALSE(direct->cpu_because.empty()) << "a workgroup of 7 has no variant";

    const an::ControlProgram program{{{0.0f, kHalf}}};
    Result<std::unique_ptr<ed::AirflowRun>> run =
        ed::AirflowRun::start(s, region, program, {}, ed::gpu_air_starter(kNoVariant));
    ASSERT_TRUE(run.has_value()) << run.error().context;
    EXPECT_EQ((*run)->backend(), "CPU: " + direct->cpu_because);
    const std::shared_ptr<const ed::AirFrame> frame = wait_for_frames(**run, 1);
    (*run)->stop();
    ASSERT_NE(frame, nullptr);
    ASSERT_TRUE(frame->error.empty()) << frame->error;
    ASSERT_GE(frame->frames, 1u);
    expect_cpus_frame(s, region, program, *frame, &ed::AirCases::discs);
}

// The bound (the lead, 2026-10-27): a device run's frame with a cell past ten
// times the top speed, finite and in binary16's range (FastCell), stops the
// air at that frame, named, and the frame is not handed over. (A poisoned
// start doesn't serve: 1000 m/s through the box overflows binary16 in the
// published fields, 8680 halves, and the device's publish refuses the frame
// before the bound sees it, measured 2026-10-27.)
TEST(GpuEditorAirflow, AFlowPastTheBoundStopsTheAirAtItsFrame) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::scene::SceneDesc s = with_reference_quad();
    ASSERT_FALSE(s.airflow.empty());
    const std::shared_ptr<VulkanContext> ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const ed::AirStarter fast = [ctx](const ed::AirCases& cases) -> Result<ed::AirStart> {
        const an::ResolvedCase& r = cases.body.resolved;
        const spade::vehicles::ModelType& m = cases.body.model;
        const an::LiveFluidMaker gpu = ag::gpu_live_fluid(*ctx);
        const an::LiveFluidMaker maker =
            [&gpu](const an::LiveFluidStart& start) -> Result<std::unique_ptr<an::LiveFluid>> {
            Result<std::unique_ptr<an::LiveFluid>> device = gpu(start);
            if (!device) return std::unexpected(device.error());
            return std::make_unique<FastCell>(std::move(*device));
        };
        Result<an::LiveRun> run = an::LiveRun::start(r, m, maker);
        if (!run) return std::unexpected(run.error());
        return ed::AirStart{std::move(*run), "GPU", ctx};
    };
    const an::ControlProgram program{{{0.0f, kHalf}}};
    Result<std::unique_ptr<ed::AirflowRun>> run = ed::AirflowRun::start(s, s.airflow.back(), program, {}, fast);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    const std::shared_ptr<const ed::AirFrame> last = wait_for_frames(**run, 1);
    (*run)->stop();
    ASSERT_NE(last, nullptr);
    EXPECT_EQ(last->frames, 0u) << "the frame past the bound is not handed over";
    EXPECT_NE(last->error.find(": frame 1: the fastest cell, (1, 2, 3), is at "), std::string::npos) << last->error;
    EXPECT_NE(last->error.find(" m/s, past ten times the top speed ("), std::string::npos) << last->error;
}

// Run air's subgrid turbulence on the device (the Advanced row; the lead, 2026-11-11): asked for at kAirSgsCs, the
// run says "GPU", its frame carries the account (nu_t's largest met, non-zero) and the panel's line names Cs; asked
// for nothing, the account is empty and the line says nothing.
TEST(GpuEditorAirflow, TurbulenceOnTheDeviceReachesTheFramesAccountAndLine) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::scene::SceneDesc s = with_reference_quad();
    ASSERT_FALSE(s.airflow.empty());
    const an::ControlProgram program{{{0.0f, kHalf}}};
    Result<std::unique_ptr<ed::AirflowRun>> on = ed::AirflowRun::start(
        s, s.airflow.back(), program, {}, ed::gpu_air_starter(), ed::AirLevel::detailed, ed::kAirSgsCs);
    ASSERT_TRUE(on.has_value()) << on.error().context;
    EXPECT_EQ((*on)->backend(), "GPU");
    const std::shared_ptr<const ed::AirFrame> frame = wait_for_frames(**on, 2);
    (*on)->stop();
    ASSERT_NE(frame, nullptr);
    ASSERT_TRUE(frame->error.empty()) << frame->error;
    EXPECT_GT(frame->sgs.nu_t_max, 0.0f) << "the term ran on the device";
    EXPECT_EQ(frame->turbulence_line.rfind("Cs 0.17 \xC2\xB7 ", 0), 0u) << frame->turbulence_line;
    std::printf("[ turb    ] device, %llu steps at Cs 0.17: %s\n", static_cast<unsigned long long>(frame->steps),
                frame->turbulence_line.c_str());

    Result<std::unique_ptr<ed::AirflowRun>> off =
        ed::AirflowRun::start(s, s.airflow.back(), program, {}, ed::gpu_air_starter());
    ASSERT_TRUE(off.has_value()) << off.error().context;
    const std::shared_ptr<const ed::AirFrame> plain = wait_for_frames(**off, 2);
    (*off)->stop();
    ASSERT_NE(plain, nullptr);
    EXPECT_EQ(plain->sgs.nu_t_max, 0.0f);
    EXPECT_EQ(plain->turbulence_line, "");
}
