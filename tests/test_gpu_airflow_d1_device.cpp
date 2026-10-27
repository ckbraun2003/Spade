// B7's acceptance harness (tests/b7_harness.hpp) on Rendering's GpuFluid
// (spike/airflow/step_chain.hpp, G1): the whole fluid step on the device
// against the CPU's run, bit for bit on the device of record (TD-14 as
// amended 2026-10-22; elsewhere skipped). Until G3 GpuFluid takes no solids
// and reports no wall load, and its health is pressure_iterations alone, so
// the boxes run without their solids and the block names the rest "not
// compared" (the lead, 2026-10-27).
//
//   GpuB7Acceptance   2 steps in the stand-in box and in the D1 box, each
//                     without its solids, under a minute; with
//                     SPADE_AIRFLOW_VALIDATION=1, 32 steps in each with the
//                     step time and the film's 0.50 s, the block printed.
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

// GpuFluid through the seam. Its reads go into a grid of the fluid's shape.
class GpuFluidStepper : public FluidStepper {
  public:
    GpuFluidStepper(std::unique_ptr<ag::GpuFluid> gpu, const MacGrid& shaped) : gpu_(std::move(gpu)), shaped_(shaped) {}

    [[nodiscard]] static MakeStepper maker(VulkanContext& ctx) {
        return [&ctx](const MacGrid& g, const DomainBc& bc, const FluidConfig& cfg,
                      const SolidField* solids) -> spade::Result<std::unique_ptr<FluidStepper>> {
            spade::Result<std::unique_ptr<ag::GpuFluid>> gpu =
                ag::GpuFluid::create(ctx, g, bc, cfg, solids != nullptr && solids->any ? solids : nullptr);
            if (!gpu) return std::unexpected(gpu.error());
            return std::make_unique<GpuFluidStepper>(std::move(*gpu), g);
        };
    }

    [[nodiscard]] std::string name() const override { return "GpuFluid (G1)"; }

    [[nodiscard]] Reports reports() const override { return kG1Reports; }

    [[nodiscard]] spade::Result<FluidHealth> step(std::span<const ActuatorDisc> discs) override {
        return gpu_->step(discs);
    }

    [[nodiscard]] spade::Result<void> read(MacGrid& out) override {
        if (out.p.size() != shaped_.p.size()) out = shaped_;
        return gpu_->read(out);
    }

    [[nodiscard]] spade::Result<WallLoad> wall_load(glm::vec3) override {
        return std::unexpected(spade::Error{spade::Code::unavailable, "GpuFluid has no wall load until G3"});
    }

    [[nodiscard]] spade::Result<void> read_solids(SolidField&) override {
        return std::unexpected(spade::Error{spade::Code::unavailable, "GpuFluid takes no solids until G3"});
    }

    [[nodiscard]] spade::Result<void> publish(const PublishInput& in, spade::FlowVolume& out) override {
        return gpu_->publish(in.origin, out);
    }

  private:
    std::unique_ptr<ag::GpuFluid> gpu_;
    MacGrid shaped_;  // the initial grid: a read's shape
};

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

// D1 without its solids diverges on the CPU itself (2026-10-27), and GpuFluid
// follows it bit for bit: its physical line is printed, not asserted, until
// Physics fixes the cause (the lead). The real D1 at G3 gets its own item,
// "known-open: D1 disc-plane face growth (Physics' triage 2)", so one fix
// can't close the other's line.
constexpr const char* kD1KnownOpen =
    "known-open: D1 without solids, plan A's top-face blow-up (Physics' triage 1)";

// The harness on `box` with GpuFluid under `plan`: the block printed and read
// back, the values bit-equal over the plan's steps, the discs moving the air
// and the flow staying physical (unless `known_open` names why it doesn't).
// The step time's verdict is printed for the record, not asserted.
void expect_bit_equal(const spade::Result<B7Box>& box, const RunPlan& plan, std::size_t measures, double speed,
                      const std::string& known_open = {}) {
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
    const std::string physical = physical_line(*run, known_open);
    std::fputs(physical.c_str(), stdout);
    if (known_open.empty()) {
        EXPECT_TRUE(run->physical) << physical;
    }
    const auto record = airflow_test::read_as_record(run->block, "b7-acceptance.md");
    ASSERT_TRUE(record.has_value()) << record.error().context;
    ASSERT_EQ(record->size(), 1u);
    ASSERT_EQ(record->at(0).measures.size(), measures);
    EXPECT_EQ(record->at(0).measures[0].verdict, an::Verdict::within);
    EXPECT_NE(record->at(0).measures[0].quantity.find("not compared: fault"), std::string::npos)
        << record->at(0).measures[0].quantity;
    EXPECT_EQ(run->c.compared, plan.compared * values_per_step(box->grid, kG1Reports));
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
    expect_bit_equal(d1_box(BoxSolids::none), kShortPlan, 2u, 0.1, kD1KnownOpen);
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
    expect_bit_equal(d1_box(BoxSolids::none), kFullPlan, 4u, 1.0, kD1KnownOpen);
}

}  // namespace
