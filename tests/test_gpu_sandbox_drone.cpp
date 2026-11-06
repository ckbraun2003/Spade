// The drone sim box's stand on the Vulkan backend (sandbox/drone_sim.hpp).
//
// The stand is held by Core's translation lock (module API stage 6, L34), whose grade is CPU `reference` and Vulkan
// `banded` with `pos` and `vel` bit-exact. So these tests assert BITS only for what the lock selects, `pos == anchor`
// and `vel == 0`; everything else on the device, the attitude and the IMU included, is compared against the CPU stand
// within a band, not bitwise (L34, the lead's point 2).
//
// The band is TD-14's target, 1e-4 relative, as an upper guard: the parity table's own orient rows measure about 1e-6
// absolute (engine/testing/parity.hpp), so a miss here is a real divergence. The largest values seen are reported in
// the READY, not pinned here.
//
// These skip with their reason on a box with no Vulkan device (TD-6).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <glm/gtc/quaternion.hpp>

#include "../sandbox/drone_sim.hpp"
#include "gpu_skip.hpp"

using namespace spade::sandbox;

namespace {

[[nodiscard]] float angle_between(const glm::quat& a, const glm::quat& b) {
    const glm::quat err = glm::conjugate(a) * b;
    return 2.0f * std::acos(std::min(1.0f, std::fabs(err.w)));
}

[[nodiscard]] std::array<uint32_t, 3> bits(const glm::vec3& v) {
    std::array<uint32_t, 3> out{};
    std::memcpy(out.data(), &v, sizeof(v));
    return out;
}

[[nodiscard]] DronePhysicsOptions on_vulkan() {
    DronePhysicsOptions o;
    o.vulkan = true;
    return o;
}

constexpr float kAttitudeBandRad = 1e-4f;

}  // namespace

TEST(GpuSandboxDrone, TheVulkanStandHoldsPositionAndVelocityBitwise) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto drone = DroneSim::create(on_vulkan());
    ASSERT_TRUE(drone.has_value()) << drone.error().context;
    EXPECT_TRUE(drone->options().vulkan);
    drone->target = AttitudeTarget{0.5f, 0.3f, -0.2f};
    for (int block = 0; block < 10; ++block) {
        ASSERT_TRUE(drone->step_fixed(50).has_value());
        const auto* body = *drone->sim().body(drone->vehicle().body);
        ASSERT_EQ(bits(body->pos), bits(glm::vec3(0.0f))) << "block " << block << ": the anchor is the spawn, the origin";
        ASSERT_EQ(bits(body->vel), bits(glm::vec3(0.0f))) << "block " << block << ": +0.0f";
    }
    const auto* body = *drone->sim().body(drone->vehicle().body);
    EXPECT_GT(angle_between(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), body->orient), 0.3f);  // it did move
}

TEST(GpuSandboxDrone, TheVulkanStandHoldsInWindAndTurbulence) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    DronePhysicsOptions o = on_vulkan();
    o.wind_speed_mps = 12.0f;
    o.wind_heading_deg = 30.0f;
    o.turbulence = spade::TurbulenceLevel::severe;
    auto drone = DroneSim::create(o);
    ASSERT_TRUE(drone.has_value()) << drone.error().context;
    drone->target = AttitudeTarget{0.0f, 0.4f, 0.4f};
    ASSERT_TRUE(drone->step_fixed(500).has_value());
    const auto* body = *drone->sim().body(drone->vehicle().body);
    EXPECT_EQ(bits(body->pos), bits(glm::vec3(0.0f)));
    EXPECT_EQ(bits(body->vel), bits(glm::vec3(0.0f)));
}

// The same stand, the same target, the two backends: the attitude and rates agree inside the band.
TEST(GpuSandboxDrone, TheVulkanStandTracksTheCpuStandWithinTheBand) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto cpu = DroneSim::create(DronePhysicsOptions{});
    auto gpu = DroneSim::create(on_vulkan());
    ASSERT_TRUE(cpu.has_value()) << cpu.error().context;
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    cpu->target = gpu->target = AttitudeTarget{0.5f, 0.3f, -0.2f};
    for (int block = 0; block < 8; ++block) {
        ASSERT_TRUE(cpu->step_fixed(25).has_value());
        ASSERT_TRUE(gpu->step_fixed(25).has_value());
        const DroneReadouts c = cpu->readouts();
        const DroneReadouts g = gpu->readouts();
        EXPECT_LE(angle_between(c.orientation, g.orientation), kAttitudeBandRad) << "step " << c.tick;
        EXPECT_LE(glm::length(c.omega_body - g.omega_body), 1e-4f * (1.0f + glm::length(c.omega_body)))
            << "step " << c.tick;
    }
}

// TheStandMountedImuReadsPlusG (1e-4) on the device, at the box's 2 substeps, while the stand is level.
TEST(GpuSandboxDrone, TheVulkanStandsImuReadsPlusGWhileLevel) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto drone = DroneSim::create(on_vulkan());
    ASSERT_TRUE(drone.has_value()) << drone.error().context;
    ASSERT_TRUE(drone->step_fixed(10).has_value());
    const DroneReadouts r = drone->readouts();
    EXPECT_NEAR(r.imu_accel.y, 9.80665f, 1e-4f);
    EXPECT_NEAR(r.imu_accel.x, 0.0f, 1e-4f);
    EXPECT_NEAR(r.imu_accel.z, 0.0f, 1e-4f);
}

// Switching the backend on a running stand rebuilds it, carries the attitude and rates (the spawn renormalizes the
// quaternion, hence 1e-6 rad), and locks again at the origin; switching back does the same.
TEST(GpuSandboxDrone, ACpuToVulkanRebuildCarriesTheAttitudeAndLocksAgain) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    drone->target = AttitudeTarget{0.0f, 0.4f, 0.0f};
    ASSERT_TRUE(drone->step_fixed(100).has_value());
    const DroneReadouts before = drone->readouts();
    ASSERT_GT(glm::length(before.omega_body), 0.0f);

    const auto to_gpu = drone->apply_options(on_vulkan());
    ASSERT_TRUE(to_gpu.has_value()) << to_gpu.error().context;
    EXPECT_TRUE(drone->options().vulkan);
    EXPECT_EQ(drone->target.pitch, 0.4f);
    const DroneReadouts carried = drone->readouts();
    EXPECT_LE(angle_between(carried.orientation, before.orientation), 1e-6f);
    EXPECT_EQ(carried.omega_body, before.omega_body);

    ASSERT_TRUE(drone->step_fixed(100).has_value());
    const auto* body = *drone->sim().body(drone->vehicle().body);
    EXPECT_EQ(bits(body->pos), bits(glm::vec3(0.0f)));
    EXPECT_EQ(bits(body->vel), bits(glm::vec3(0.0f)));

    const auto to_cpu = drone->apply_options(DronePhysicsOptions{});
    ASSERT_TRUE(to_cpu.has_value()) << to_cpu.error().context;
    EXPECT_FALSE(drone->options().vulkan);
    ASSERT_TRUE(drone->step_fixed(100).has_value());
    const auto* back = *drone->sim().body(drone->vehicle().body);
    EXPECT_EQ(bits(back->pos), bits(glm::vec3(0.0f)));
    EXPECT_EQ(bits(back->vel), bits(glm::vec3(0.0f)));
}
