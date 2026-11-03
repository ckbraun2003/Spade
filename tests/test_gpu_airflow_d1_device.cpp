// B7's acceptance harness (tests/b7_harness.hpp) on Rendering's GpuFluid
// (spike/airflow/step_chain.hpp, G1 and G3; tests/b7_gpu.hpp): the whole
// fluid step on the device against the CPU's run, bit for bit on the device
// of record (TD-14 as amended 2026-10-22), and the sanity floor on every
// device (the lead, 2026-11-03).
//
//   GpuB7Acceptance   2 steps in the stand-in box with its sphere and
//                     without it, and in the D1 box without its solids, under
//                     a minute; with SPADE_AIRFLOW_VALIDATION=1, 32 steps in
//                     each with the step time and the film's 0.50 s, and the
//                     D1 box with its solids, 2 steps then 32, the block
//                     printed (d1-hover.md keeps it, B8).
#include <gtest/gtest.h>

#include "b7_gpu.hpp"
#include "gpu_skip.hpp"

namespace {

using namespace b7;

TEST(GpuB7Acceptance, TwoStepsMatchTheCpuInTheStandInBoxWithoutItsSphere) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    expect_on_device(stand_in_box(BoxSolids::none), kShortPlan, 2u, 0.01);
}

TEST(GpuB7Acceptance, TwoStepsMatchTheCpuInTheD1BoxWithoutItsSolids) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    // The wind is along -x and v starts at 0: any v is the discs'.
    expect_on_device(d1_box(BoxSolids::none), kShortPlan, 2u, 0.1);
}

// The full plan: 32 steps compared, the step's median and the film's 0.50 s
// (100 steps in the stand-in box, 1316 in D1), four measures without the
// solids'. The CPU's side of D1 takes some minutes.
TEST(GpuB7Acceptance, ThirtyTwoStepsAndTheTimesInTheStandInBox) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run B7's full GPU plan";
    expect_on_device(stand_in_box(BoxSolids::none), kFullPlan, 4u, 0.1);
}

TEST(GpuB7Acceptance, ThirtyTwoStepsAndTheTimesInTheD1Box) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run B7's full GPU plan in D1";
    expect_on_device(d1_box(BoxSolids::none), kFullPlan, 4u, 1.0);
}

// G3: the boxes with their solids, GpuFluid reporting the CG's health and the
// wall load. The stand-in's 2 steps are seconds on both sides, so they join
// the short set (Test/Docs, 2026-10-28).
TEST(GpuB7Acceptance, TwoStepsMatchTheCpuInTheStandInBoxWithItsSphere) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    expect_on_device(stand_in_box(BoxSolids::with), kShortPlan, 2u, 0.01);
}

TEST(GpuB7Acceptance, ThirtyTwoStepsAndTheTimesInTheStandInBoxWithItsSphere) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run B7's full GPU plan";
    expect_on_device(stand_in_box(BoxSolids::with), kFullPlan, 4u, 0.1);
}

// The real D1 (B7a): the reference quad's rigid parts as solids, its discs
// the dense disc's (58720c6). The dense disc's spread costs the CPU's side 30
// to 47 s a step, so both runs are opt-in: 2 steps (83 s), then 32 with the
// times and the film's 0.50 s (21 minutes; Rendering's box, 2026-10-28). The
// flow must stay physical: no known-open line. The 32 steps' block is D1's
// GPU record (B8, tests/data/airflow/records/d1-hover.md).
TEST(GpuB7Acceptance, TwoStepsMatchTheCpuInTheD1BoxWithItsSolids) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run D1 with its solids";
    expect_on_device(d1_box(BoxSolids::with), kShortPlan, 2u, 0.1);
}

TEST(GpuB7Acceptance, ThirtyTwoStepsAndTheTimesInTheD1BoxWithItsSolids) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run B7's full GPU plan in D1";
    expect_on_device(d1_box(BoxSolids::with), kFullPlan, 4u, 1.0);
}

}  // namespace
