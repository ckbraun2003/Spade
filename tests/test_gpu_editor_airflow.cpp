// Run air on the GPU (Rendering's design note, docs/design/rendering/plans/
// 2026-10-27-run-air-on-gpu.md, slice 3; the lead, 2026-10-27): the
// editor's AirflowRun started through gpu_air_starter()
// (sandbox/editor_air_gpu.hpp) runs on the device and says so, the CPU's run
// to the bit on the device of record; a device's refusal runs on the CPU,
// naming it verbatim; and a frame past ten times the top speed, finite and
// in binary16's range, is stopped by the bound at its frame.
// GpuEditorAirflow* on the device.

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
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

// The library's reference quad placed at (1, 2, 3) with its region, the
// editor's own 32^3 at 25 mm.
[[nodiscard]] spade::scene::SceneDesc with_reference_quad() {
    auto d = ed::make_scene_document(support::sample_scene(), {});
    ed::AddModelFile add;
    add.file = (std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml")
                   .generic_string();
    add.vehicle.name = "ref";
    add.vehicle.start.pos = glm::vec3(1.0f, 2.0f, 3.0f);
    const Result<void> added = ed::apply(d, add);
    EXPECT_TRUE(added.has_value()) << (added ? "" : added.error().context);
    const Result<void> region = ed::apply(d, ed::AddAirflowRegion{"ref"});
    EXPECT_TRUE(region.has_value()) << (region ? "" : region.error().context);
    return d.desc();
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

// `frame` against a one-thread LiveRun of the same case and program on the
// CPU, as Run air ran before slice 3: the same steps, time and halves, to
// the bit.
void expect_cpus_frame(const spade::scene::SceneDesc& s, const spade::scene::SceneAirflowRegion& region,
                       const an::ControlProgram& program, const ed::AirFrame& frame) {
    const Result<an::ResolvedCase> resolved = ed::air_case(s, region);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    const Result<spade::vehicles::ModelType> model = ed::air_model(s, region);
    ASSERT_TRUE(model.has_value()) << model.error().context;
    Result<an::LiveRun> one = an::LiveRun::start(*resolved, *model);
    ASSERT_TRUE(one.has_value()) << one.error().context;
    spade::FlowVolume v;
    while (one->state().frames < frame.frames) {
        const an::Controls c = an::controls_at(program, static_cast<float>(one->state().t_s));
        ASSERT_TRUE(one->advance_frame(c, v).has_value());
    }
    EXPECT_EQ(frame.steps, one->state().steps);
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

// A device that refuses the case (here a workgroup size with no kernel
// variant; after Rendering's NaN-preserve fix, a device without the
// feature): the run is on the CPU, today's run to the bit, and its backend is
// "CPU: " and the refusal verbatim, as start_live_run() kept it.
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
    expect_cpus_frame(s, region, program, *frame);
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
    const ed::AirStarter fast = [ctx](const an::ResolvedCase& r,
                                      const spade::vehicles::ModelType& m) -> Result<ed::AirStart> {
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
