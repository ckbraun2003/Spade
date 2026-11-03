// B8's GPU records (the GPU solver plan, B8; the lead's ruling of 2026-11-03):
// S1, S3 and C8q's boxes (tests/b8_boxes.hpp) through B7's harness on
// GpuFluid (tests/b7_gpu.hpp), each against the CPU's run of the same box
// for 32 steps, bit for bit on the device of record, the sanity floor on
// every device, and the step's median printed as a reading. Each block is
// pasted into its case's record (tests/data/airflow/records/). D1's block is
// B7's own (test_gpu_airflow_d1_device.cpp).
//
//   GpuB8Records   S3 on planes 16 and 17, seconds, always; with
//                  SPADE_AIRFLOW_VALIDATION=1, S1 at R/dx 6 and 12 and C8q.
#include <gtest/gtest.h>

#include "b7_gpu.hpp"
#include "b8_boxes.hpp"
#include "gpu_skip.hpp"

namespace {

using namespace b7;

// Three measures: the values, the first differing step, the step time.
constexpr std::size_t kRecordMeasures = 3;

// S3's sheet stops the air (u ~ 1e-7 m/s after the step), so the pressure
// carries the case and there is no |v| to require.
TEST(GpuB8Records, S3SheetOnAnEvenPlane) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    expect_on_device(s3_box(16), kRecordPlan, kRecordMeasures, std::nullopt);
}

TEST(GpuB8Records, S3SheetOnAnOddPlane) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    expect_on_device(s3_box(17), kRecordPlan, kRecordMeasures, std::nullopt);
}

// The stream along x: any v is the sphere's.
TEST(GpuB8Records, S1SphereAtSixCellsPerRadius) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run S1's GPU record";
    expect_on_device(s1_box(6), kRecordPlan, kRecordMeasures, 0.01);
}

TEST(GpuB8Records, S1SphereAtTwelveCellsPerRadius) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run S1's GPU record";
    expect_on_device(s1_box(12), kRecordPlan, kRecordMeasures, 0.01);
}

// Still air: any v is the discs'.
TEST(GpuB8Records, C8qReferenceQuad) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run C8q's GPU record";
    expect_on_device(c8q_box(), kRecordPlan, kRecordMeasures, 0.1);
}

}  // namespace
