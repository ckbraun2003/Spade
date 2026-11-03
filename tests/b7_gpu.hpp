#pragma once

// B7's acceptance harness (tests/b7_harness.hpp) on Rendering's GpuFluid
// (spike/airflow/step_chain.hpp, G1 and G3), for every test that runs it:
// B7's (test_gpu_airflow_d1_device.cpp) and B8's records
// (test_gpu_airflow_records.cpp). With solids (G3) GpuFluid reports the CG's
// health and the wall load; the solid field itself is proved through the
// step (bit-equal u, v, w, p and wall load on a part), not read back, so the
// block names it "not compared" (the lead, 2026-10-28). Without solids its
// health is pressure_iterations alone (plan A), as at G1.
//
// Every run checks the sanity floor (b7_harness.hpp, the lead's ruling of
// 2026-11-03), on any device. The bits are checked on the device of record
// alone (TD-14 as amended): elsewhere the test skips after the floor.

#include <gtest/gtest.h>

#include <cstdio>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>

#include "airflow_exact_device.hpp"
#include "b7_harness.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "spike/airflow/step_chain.hpp"

namespace b7 {

namespace ag = spade::airflow_gpu;
using spade::compute::VulkanContext;

// What GpuFluid reports at G1: pressure_iterations (n_v) beside u, v, w and
// p. No scan for a NaN, no residual, no solids or wall load until G3.
inline constexpr Reports kG1Reports{.fault = false,
                                    .unconverged = false,
                                    .pressure_iterations = true,
                                    .pressure_residual = false,
                                    .stalled = false,
                                    .capped = false,
                                    .wall_load = false,
                                    .solids = false};

// What GpuFluid reports at G3 with solids: the CG's report (the health's
// residual, iterations, unconverged, capped and stalled) and the wall load.
// No scan for a NaN (the fault), and the solid field is proved through the
// step rather than read back (read_solids: YAGNI, the lead, 2026-10-28).
inline constexpr Reports kG3Reports{.fault = false,
                                    .unconverged = true,
                                    .pressure_iterations = true,
                                    .pressure_residual = true,
                                    .stalled = true,
                                    .capped = true,
                                    .wall_load = true,
                                    .solids = false};

// GpuFluid through the seam. Its reads go into a grid of the fluid's shape.
class GpuFluidStepper : public FluidStepper {
  public:
    GpuFluidStepper(std::unique_ptr<ag::GpuFluid> gpu, const MacGrid& shaped, bool solids, std::string device)
        : gpu_(std::move(gpu)), shaped_(shaped), solids_(solids), device_(std::move(device)) {}

    [[nodiscard]] static MakeStepper maker(VulkanContext& ctx) {
        return [&ctx](const MacGrid& g, const DomainBc& bc, const FluidConfig& cfg,
                      const SolidField* solids) -> spade::Result<std::unique_ptr<FluidStepper>> {
            const bool with = solids != nullptr && solids->any;
            spade::Result<std::unique_ptr<ag::GpuFluid>> gpu =
                ag::GpuFluid::create(ctx, g, bc, cfg, with ? solids : nullptr);
            if (!gpu) return std::unexpected(gpu.error());
            return std::make_unique<GpuFluidStepper>(std::move(*gpu), g, with,
                                                     spade::compute::describe(ctx.device_report()));
        };
    }

    [[nodiscard]] std::string name() const override { return solids_ ? "GpuFluid (G3)" : "GpuFluid (G1)"; }

    [[nodiscard]] std::string device() const override { return device_; }

    [[nodiscard]] Reports reports() const override { return solids_ ? kG3Reports : kG1Reports; }

    [[nodiscard]] spade::Result<FluidHealth> step(std::span<const ActuatorDisc> discs) override {
        return gpu_->step(discs);
    }

    [[nodiscard]] spade::Result<void> read(MacGrid& out) override {
        if (out.p.size() != shaped_.p.size()) out = shaped_;
        return gpu_->read(out);
    }

    [[nodiscard]] spade::Result<WallLoad> wall_load(glm::vec3 about) override { return gpu_->wall_load(about); }

    [[nodiscard]] spade::Result<void> read_solids(SolidField&) override {
        return std::unexpected(
            spade::Error{spade::Code::unavailable, "GpuFluid's solid field is proved through the step, not read back"});
    }

    [[nodiscard]] spade::Result<void> publish(const PublishInput& in, spade::FlowVolume& out) override {
        return gpu_->publish(in.origin, out);
    }

  private:
    std::unique_ptr<ag::GpuFluid> gpu_;
    MacGrid shaped_;       // the initial grid: a read's shape
    bool solids_ = false;  // made with a field that has parts (G3)
    std::string device_;   // compute::describe() of the context's device
};

[[nodiscard]] inline std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

// The harness on `box` with GpuFluid under `plan`: the block, the physical
// line and the floor's printed; the CPU's flow physical and GpuFluid's
// within the sanity floor, on any device. On the device of record, also the
// block read back with `measures` measures, the values bit-equal over the
// plan's steps, and the CPU's largest |v| at step N over `speed` (the air
// moved, or the comparison proves nothing; nullopt where the case stops its
// air, S3's sheet, and the pressure carries it). Off it, skipped after the
// floor. The step time is printed for the record, not asserted.
inline void expect_on_device(const spade::Result<B7Box>& box, const RunPlan& plan, std::size_t measures,
                             std::optional<double> speed) {
    ASSERT_TRUE(box.has_value()) << box.error().context;
    const std::unique_ptr<VulkanContext> ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const spade::Result<B7Run> run = run_b7(*box, GpuFluidStepper::maker(*ctx), plan);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    std::fputs(run->block.c_str(), stdout);
    std::printf("the CPU's largest |v| at step %u: %.4f m/s\n", plan.compared, run->c.cpu_speed);
    const std::string physical = physical_line(*run);
    std::fputs(physical.c_str(), stdout);
    const std::string floor = floor_line(*run);
    std::fputs(floor.c_str(), stdout);
    EXPECT_TRUE(run->physical) << physical;
    EXPECT_TRUE(run->floor) << floor;
    if (!spade::testing::airflow_bit_equal_device(*ctx)) {
        GTEST_SKIP() << "not the device of record: the sanity floor is checked (" << (run->floor ? "PASS" : "FAIL")
                     << "), and GpuFluid's bits are held to the CPU's on the RTX 3060 Ti at 572.83 alone (TD-14 as "
                        "amended)";
    }
    const auto record = airflow_test::read_as_record(run->block, "gpu-record.md");
    ASSERT_TRUE(record.has_value()) << record.error().context;
    ASSERT_EQ(record->size(), 1u);
    ASSERT_EQ(record->at(0).measures.size(), measures);
    EXPECT_EQ(record->at(0).measures[0].verdict, an::Verdict::within);
    EXPECT_NE(record->at(0).measures[0].quantity.find("not compared: fault"), std::string::npos)
        << record->at(0).measures[0].quantity;
    if (box->solids.any) {
        // G3 compares the whole health but the fault, and the wall load: a
        // report kG3Reports dropped would join this list (Test/Docs' review).
        EXPECT_NE(record->at(0).measures[0].quantity.find("not compared: fault, the solids)"), std::string::npos)
            << record->at(0).measures[0].quantity;
    }
    EXPECT_NE(record->at(0).path.find(", on " + spade::compute::describe(ctx->device_report())), std::string::npos)
        << record->at(0).path;
    EXPECT_EQ(run->c.compared, plan.compared * values_per_step(box->grid, run->c.reports));
    EXPECT_EQ(run->c.differing, 0u);
    EXPECT_EQ(run->c.first_step, 0u);
    if (speed) {
        EXPECT_GT(run->c.cpu_speed, *speed) << "the air must move, or the comparison proves nothing";
    }
}

}  // namespace b7
