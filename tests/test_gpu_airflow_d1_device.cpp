// B7's acceptance harness (tests/b7_harness.hpp) on Rendering's GpuFluid
// (spike/airflow/step_chain.hpp, G1 and G3): the whole fluid step on the
// device against the CPU's run, bit for bit on the device of record (TD-14 as
// amended 2026-10-22; elsewhere skipped). With solids (G3) GpuFluid reports
// the CG's health and the wall load; the solid field itself is proved through
// the step (bit-equal u, v, w, p and wall load on a part), not read back, so
// the block names it "not compared" (the lead, 2026-10-28). Without solids its
// health is pressure_iterations alone (plan A), as at G1.
//
//   GpuB7Acceptance   2 steps in the stand-in box with its sphere and
//                     without it, and in the D1 box without its solids, under
//                     a minute; with SPADE_AIRFLOW_VALIDATION=1, 32 steps in
//                     each with the step time and the film's 0.50 s, and the
//                     D1 box with its solids, 2 steps then 32, the block
//                     printed.
#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <memory>
#include <span>
#include <string>
#include <utility>

#include "airflow_exact_device.hpp"
#include "b7_harness.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "gpu_skip.hpp"
#include "spike/airflow/step_chain.hpp"

namespace {

using namespace b7;
namespace ag = spade::airflow_gpu;
using spade::compute::VulkanContext;

// What GpuFluid reports at G1: pressure_iterations (n_v) beside u, v, w and
// p. No scan for a NaN, no residual, no solids or wall load until G3.
constexpr Reports kG1Reports{.fault = false,
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
constexpr Reports kG3Reports{.fault = false,
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
    GpuFluidStepper(std::unique_ptr<ag::GpuFluid> gpu, const MacGrid& shaped, bool solids)
        : gpu_(std::move(gpu)), shaped_(shaped), solids_(solids) {}

    [[nodiscard]] static MakeStepper maker(VulkanContext& ctx) {
        return [&ctx](const MacGrid& g, const DomainBc& bc, const FluidConfig& cfg,
                      const SolidField* solids) -> spade::Result<std::unique_ptr<FluidStepper>> {
            const bool with = solids != nullptr && solids->any;
            spade::Result<std::unique_ptr<ag::GpuFluid>> gpu =
                ag::GpuFluid::create(ctx, g, bc, cfg, with ? solids : nullptr);
            if (!gpu) return std::unexpected(gpu.error());
            return std::make_unique<GpuFluidStepper>(std::move(*gpu), g, with);
        };
    }

    [[nodiscard]] std::string name() const override { return solids_ ? "GpuFluid (G3)" : "GpuFluid (G1)"; }

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
    MacGrid shaped_;      // the initial grid: a read's shape
    bool solids_ = false;  // made with a field that has parts (G3)
};

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

// The harness on `box` with GpuFluid under `plan`: the block printed and read
// back, the values bit-equal over the plan's steps, the discs moving the air
// and the flow staying physical. The step time's verdict is printed for the
// record, not asserted.
void expect_bit_equal(const spade::Result<B7Box>& box, const RunPlan& plan, std::size_t measures, double speed) {
    ASSERT_TRUE(box.has_value()) << box.error().context;
    const std::unique_ptr<VulkanContext> ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    if (!spade::testing::airflow_bit_equal_device(*ctx)) {
        GTEST_SKIP() << "not the device of record: GpuFluid's bits are held to the CPU's on the RTX 3060 Ti at "
                        "572.83 (TD-14 as amended)";
    }
    const spade::Result<B7Run> run = run_b7(*box, GpuFluidStepper::maker(*ctx), plan);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    std::fputs(run->block.c_str(), stdout);
    std::printf("the CPU's largest |v| at step %u: %.4f m/s\n", plan.compared, run->c.cpu_speed);
    const std::string physical = physical_line(*run);
    std::fputs(physical.c_str(), stdout);
    EXPECT_TRUE(run->physical) << physical;
    const auto record = airflow_test::read_as_record(run->block, "b7-acceptance.md");
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
    EXPECT_EQ(run->c.compared, plan.compared * values_per_step(box->grid, run->c.reports));
    EXPECT_EQ(run->c.differing, 0u);
    EXPECT_EQ(run->c.first_step, 0u);
    EXPECT_GT(run->c.cpu_speed, speed) << "the air must move, or the comparison proves nothing";
}

TEST(GpuB7Acceptance, TwoStepsMatchTheCpuInTheStandInBoxWithoutItsSphere) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    expect_bit_equal(stand_in_box(BoxSolids::none), kShortPlan, 2u, 0.01);
}

TEST(GpuB7Acceptance, TwoStepsMatchTheCpuInTheD1BoxWithoutItsSolids) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    // The wind is along -x and v starts at 0: any v is the discs'.
    expect_bit_equal(d1_box(BoxSolids::none), kShortPlan, 2u, 0.1);
}

// The full plan: 32 steps compared, the step's median and the film's 0.50 s
// (100 steps in the stand-in box, 1316 in D1), four measures without the
// solids'. The CPU's side of D1 takes some minutes.
TEST(GpuB7Acceptance, ThirtyTwoStepsAndTheTimesInTheStandInBox) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run B7's full GPU plan";
    expect_bit_equal(stand_in_box(BoxSolids::none), kFullPlan, 4u, 0.1);
}

TEST(GpuB7Acceptance, ThirtyTwoStepsAndTheTimesInTheD1Box) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run B7's full GPU plan in D1";
    expect_bit_equal(d1_box(BoxSolids::none), kFullPlan, 4u, 1.0);
}

// G3: the boxes with their solids, GpuFluid reporting the CG's health and the
// wall load. The stand-in's 2 steps are seconds on both sides, so they join
// the short set (Test/Docs, 2026-10-28).
TEST(GpuB7Acceptance, TwoStepsMatchTheCpuInTheStandInBoxWithItsSphere) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    expect_bit_equal(stand_in_box(BoxSolids::with), kShortPlan, 2u, 0.01);
}

TEST(GpuB7Acceptance, ThirtyTwoStepsAndTheTimesInTheStandInBoxWithItsSphere) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run B7's full GPU plan";
    expect_bit_equal(stand_in_box(BoxSolids::with), kFullPlan, 4u, 0.1);
}

// The real D1 (B7a): the reference quad's rigid parts as solids, its discs
// the dense disc's (58720c6). The dense disc's spread costs the CPU's side 30
// to 47 s a step, so both runs are opt-in: 2 steps (83 s), then 32 with the
// times and the film's 0.50 s (21 minutes; Rendering's box, 2026-10-28). The
// flow must stay physical: no known-open line.
TEST(GpuB7Acceptance, TwoStepsMatchTheCpuInTheD1BoxWithItsSolids) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run D1 with its solids";
    expect_bit_equal(d1_box(BoxSolids::with), kShortPlan, 2u, 0.1);
}

TEST(GpuB7Acceptance, ThirtyTwoStepsAndTheTimesInTheD1BoxWithItsSolids) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run B7's full GPU plan in D1";
    expect_bit_equal(d1_box(BoxSolids::with), kFullPlan, 4u, 1.0);
}

}  // namespace
