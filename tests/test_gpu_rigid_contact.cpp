// test_gpu_rigid_contact.cpp -- contact tumbling on the device (L68). The device kernel collision_shape.slang mirrors the CPU pass op for op, with
// fp32_math's div32_ftz and sqrt32_ftz for the divides and the normalize, so the CPU-device gap is the SDF walk only. The grade is Test/Docs' record
// (docs/design/test-docs/2026-11-11-l68-device-band.md), fixed before the first comparison: OUTCOMES, never trajectories. The CPU's own ensemble
// (test_rigid_contact.cpp) gives the lateral, yaw and impact bands; the floors are in tests/rigid_contact_support.hpp.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <glm/geometric.hpp>

#include "compute/backend.hpp"
#include "gpu_skip.hpp"
#include "rigid_contact_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

using namespace spade::test::rigid;
using spade::compute::BackendKind;

// PIN: a standard set on the device is what it was: twelve pipelines, no rigid_contact pass.
TEST(GpuRigidContact, AStandardSetRecordsNoRigidContactPass) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation sim = make(spade::modules::standard_modules(), WorldOpts{}, BackendKind::vulkan);
    ASSERT_TRUE(sim.vulkan_pipeline_count().has_value());
    EXPECT_EQ(*sim.vulkan_pipeline_count(), 12u);
    const auto chain = sim.vulkan_recorded_chain();
    ASSERT_TRUE(chain.has_value());
    EXPECT_EQ(std::ranges::count_if(chain->passes, [](const std::string& n) { return n.starts_with("rigid_contact."); }), 0);
}

// The module's pass is a recorded kernel between the two contact passes: thirteen pipelines.
TEST(GpuRigidContact, TheModuleRecordsOneKernelBetweenTheContactPasses) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation sim = make(tumbling_set(), WorldOpts{}, BackendKind::vulkan);
    ASSERT_TRUE(sim.vulkan_pipeline_count().has_value());
    EXPECT_EQ(*sim.vulkan_pipeline_count(), 13u);
    const auto chain = sim.vulkan_recorded_chain();
    ASSERT_TRUE(chain.has_value());
    const auto& p = chain->passes;
    const auto stat = std::ranges::find(p, "static_contact.resolve");
    const auto rigid = std::ranges::find(p, "rigid_contact.resolve");
    const auto dyn = std::ranges::find(p, "dynamic_contact.resolve");
    ASSERT_NE(rigid, p.end());
    EXPECT_LT(stat, rigid);
    EXPECT_LT(rigid, dyn);
}

// Device against itself: memcmp-equal.
TEST(GpuRigidContact, TheScenarioIsBitIdenticalOnTheDeviceRunToRun) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const Outcome a = run_cube(BackendKind::vulkan, kScenarioTiltDeg);
    const Outcome b = run_cube(BackendKind::vulkan, kScenarioTiltDeg);
    ASSERT_TRUE(a.settled && b.settled) << "both runs must be settled at 6 s";
    EXPECT_EQ(std::memcmp(&a.end, &b.end, sizeof(BodyState)), 0);
}

// The band, outcome by outcome. A comparison within 10% of a band edge is a finding for the lead, not a refit.
TEST(GpuRigidContact, TheDeviceSettlesWhereTheCpuDoesWithinTheBand) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const Outcome cpu = run_cube(BackendKind::cpu, kScenarioTiltDeg);
    const Outcome gpu = run_cube(BackendKind::vulkan, kScenarioTiltDeg);
    ASSERT_TRUE(cpu.settled) << "the CPU run is settled at 6 s";
    ASSERT_TRUE(gpu.settled) << "an unsettled device run is reported, not compared";
    EXPECT_EQ(gpu.face, cpu.face) << "the settled face";
    EXPECT_LE(cpu.tilt_deg, kTiltBandDeg);
    EXPECT_LE(gpu.tilt_deg, kTiltBandDeg);
    EXPECT_NEAR(gpu.centre_above, cpu.centre_above, kHeightBand);
    EXPECT_GE(gpu.centre_above, kHeightLow);
    EXPECT_LE(gpu.centre_above, kHeightHigh);
    const float lateral = glm::length(gpu.lateral - cpu.lateral);
    const float yaw = std::abs(heading_difference(gpu.yaw_deg, cpu.yaw_deg));
    const int impacts = std::abs(gpu.impacts - cpu.impacts);
    std::printf("L68 device vs CPU: lateral %.5f m (band %.3f), yaw %.3f deg (band %.1f), impacts %d (band %d), height gap %.2e m, tilt cpu %.3f gpu %.3f deg\n",
                lateral, kLateralBand, yaw, kYawBandDeg, impacts, kImpactBand, std::abs(gpu.centre_above - cpu.centre_above), cpu.tilt_deg, gpu.tilt_deg);
    EXPECT_LE(lateral, kLateralBand);
    EXPECT_LE(yaw, kYawBandDeg);
    EXPECT_LE(impacts, kImpactBand);
}
