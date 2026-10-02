// The drone sim box (sandbox/drone_sim.hpp): the test stand, the attitude
// controller and mixer, the accumulator and the rebuild. All display-free --
// SL15b's "headless is the test surface".

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

#include <glm/gtc/quaternion.hpp>

#include "../sandbox/drone_sim.hpp"

using namespace spade::sandbox;

namespace {

// The angle, radians, of the rotation taking `a` to `b`.
float angle_between(const glm::quat& a, const glm::quat& b) {
    const glm::quat err = glm::conjugate(a) * b;
    return 2.0f * std::acos(std::min(1.0f, std::fabs(err.w)));
}

}  // namespace

TEST(SandboxDrone, MixerProducesTheRequestedMoments) {
    const float L = 0.18f, c = 1.9e-7f / 1.2e-5f, total = 9.80665f;
    const glm::vec3 M(0.05f, -0.01f, 0.03f);
    const auto T = mix_thrusts(total, M, L, c);
    EXPECT_NEAR(T[0] + T[1] + T[2] + T[3], total, 1e-4f);
    EXPECT_NEAR(L * (T[3] - T[1]), M.x, 1e-5f);                 // roll
    EXPECT_NEAR(-c * (T[0] - T[1] + T[2] - T[3]), M.y, 1e-5f);  // yaw
    EXPECT_NEAR(L * (T[0] - T[2]), M.z, 1e-5f);                 // pitch
}

// The test above checks the mixer against the formulas it was written from, so
// it cannot catch a formula that disagrees with the engine. This one asks the
// engine: command the mixed speeds on a free, level quadrotor and each moment
// axis must spin the body about that axis, in that direction, and no other.
TEST(SandboxDrone, TheMixerSignAgreesWithTheEngine) {
    const auto params = drone_stand_params();
    const float c = params.rotors[0].torque_coeff / params.rotors[0].thrust_coeff;
    for (int axis = 0; axis < 3; ++axis) {
        spade::Environment env;
        auto world = spade::WorldBuilder().name("mix").environment(env)
                         .capacities(spade::Capacities{1, 5, 1, 1}).build();
        ASSERT_TRUE(world.has_value());
        spade::WorldInstanceDesc inst;
        inst.world = *world;
        spade::WorldSetDesc set;
        set.worlds.push_back(inst);
        auto sim = spade::Simulation::create(set, DroneSim::kDtNs, DroneSim::kSubsteps);
        ASSERT_TRUE(sim.has_value());
        auto model = spade::vehicles::make_quadrotor(params);
        ASSERT_TRUE(model.has_value());
        auto id = sim->register_model(*model);
        ASSERT_TRUE(id.has_value());
        spade::VehicleSpawn where;
        where.rotor_omega = spade::vehicles::hover_command(params);
        auto v = sim->spawn(0, *id, where);
        ASSERT_TRUE(v.has_value());
        ASSERT_TRUE(sim->flush_structural().has_value());

        glm::vec3 M(0.0f);
        M[axis] = 0.02f;
        const auto T = mix_thrusts(params.mass * spade::vehicles::kStandardGravity, M, params.arm_length, c);
        std::array<float, 4> omegas{};
        for (std::size_t i = 0; i < 4; ++i) omegas[i] = std::sqrt(T[i] / params.rotors[0].thrust_coeff);
        ASSERT_TRUE(sim->set_rotor_commands(*v, omegas).has_value());
        ASSERT_TRUE(sim->step(50).has_value());  // 0.1 s: five rotor time constants

        const glm::vec3 w = (*sim->body(v->body))->omega_body;
        EXPECT_GT(w[axis], 0.0f) << "axis " << axis;
        for (int other = 0; other < 3; ++other) {
            if (other != axis) EXPECT_LT(std::fabs(w[other]), 1e-2f * std::fabs(w[axis])) << "axis " << axis;
        }
    }
}

TEST(SandboxDrone, ThePinHoldsPositionBitwiseWhileTheAttitudeMoves) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value()) << drone.error().context;
    drone->target = AttitudeTarget{0.5f, 0.3f, -0.2f};
    ASSERT_TRUE(drone->step_fixed(1000).has_value());
    const auto* body = *drone->sim().body(drone->vehicle().body);
    EXPECT_EQ(body->pos, glm::vec3(0.0f));
    EXPECT_EQ(body->vel, glm::vec3(0.0f));
    EXPECT_GT(angle_between(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), body->orient), 0.3f);  // it did move
}

TEST(SandboxDrone, ThePinHoldsInWindAndTurbulence) {
    DronePhysicsOptions o;
    o.wind_speed_mps = 12.0f;
    o.wind_heading_deg = 30.0f;
    o.turbulence = spade::TurbulenceLevel::severe;
    auto drone = DroneSim::create(o);
    ASSERT_TRUE(drone.has_value()) << drone.error().context;
    drone->target = AttitudeTarget{0.0f, 0.4f, 0.4f};
    ASSERT_TRUE(drone->step_fixed(1000).has_value());
    const auto* body = *drone->sim().body(drone->vehicle().body);
    EXPECT_EQ(body->pos, glm::vec3(0.0f));
    EXPECT_EQ(body->vel, glm::vec3(0.0f));
}

// The fourth case is the review-focus one: pitch and roll both at the 60
// degree limit while yawing, which is where a controller that ignores the
// quaternion double cover flips the long way round.
TEST(SandboxDrone, TheControllerReachesEachTarget) {
    for (const AttitudeTarget t : {AttitudeTarget{0.6f, 0.0f, 0.0f}, AttitudeTarget{0.0f, 0.4f, 0.0f},
                                   AttitudeTarget{0.0f, 0.0f, -0.4f},
                                   AttitudeTarget{1.2f, kMaxTiltRad, kMaxTiltRad}}) {
        auto drone = DroneSim::create(DronePhysicsOptions{});
        ASSERT_TRUE(drone.has_value());
        drone->target = t;
        ASSERT_TRUE(drone->step_fixed(1500).has_value());  // 3 s
        EXPECT_LT(angle_between(drone->readouts().orientation, attitude_quat(t)), 0.035f)  // within 2 degrees
            << "target yaw " << t.yaw << " pitch " << t.pitch << " roll " << t.roll;
    }
}

TEST(SandboxDrone, TiltBeyondTheLimitIsClamped) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    drone->target = AttitudeTarget{0.0f, 2.0f, 0.0f};
    ASSERT_TRUE(drone->step_fixed(1500).has_value());
    EXPECT_LT(angle_between(drone->readouts().orientation, attitude_quat(AttitudeTarget{0.0f, kMaxTiltRad, 0.0f})),
              0.035f);
}

TEST(SandboxDrone, VulkanIsRefusedWithAReason) {
    DronePhysicsOptions o;
    o.vulkan = true;
    auto drone = DroneSim::create(o);
    ASSERT_FALSE(drone.has_value());
    EXPECT_EQ(drone.error().code, spade::Code::unavailable);
    EXPECT_FALSE(drone.error().context.empty());
}

// Selecting Vulkan on a running stand must leave the CPU simulation in place,
// still pinned -- never a silently unpinned step.
TEST(SandboxDrone, AVulkanRebuildIsRefusedAndKeepsTheCpuStand) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    ASSERT_TRUE(drone->step_fixed(10).has_value());
    DronePhysicsOptions o;
    o.vulkan = true;
    const auto r = drone->apply_options(o);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::unavailable);
    EXPECT_FALSE(drone->options().vulkan);
    EXPECT_EQ(drone->readouts().tick, 10u);
    ASSERT_TRUE(drone->step_fixed(100).has_value());
    EXPECT_EQ((*drone->sim().body(drone->vehicle().body))->vel, glm::vec3(0.0f));
}

// Spawn renormalizes the carried quaternion, which may move it by an ulp, so
// the attitude is pinned to 1e-6 rad rather than bitwise. The body rates pass
// through spawn untouched and are pinned exactly.
TEST(SandboxDrone, ARebuildCarriesTheAttitudeAcross) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    drone->target = AttitudeTarget{0.0f, 0.4f, 0.0f};
    ASSERT_TRUE(drone->step_fixed(100).has_value());  // mid-manoeuvre: rates are non-zero
    const DroneReadouts before = drone->readouts();
    ASSERT_GT(glm::length(before.omega_body), 0.0f);
    DronePhysicsOptions o;
    o.wind_speed_mps = 6.0f;
    ASSERT_TRUE(drone->apply_options(o).has_value());
    const DroneReadouts after = drone->readouts();
    EXPECT_LT(angle_between(after.orientation, before.orientation), 1e-6f);
    EXPECT_EQ(after.omega_body, before.omega_body);
    float mean_before = 0.0f;
    for (const float w : before.rotor_omega) mean_before += 0.25f * w;
    for (const float w : after.rotor_omega) EXPECT_NEAR(w, mean_before, 1e-3f * mean_before);
    EXPECT_EQ(drone->options(), o);
    EXPECT_EQ(drone->target.pitch, 0.4f);
}

TEST(SandboxDrone, TheAccumulatorCapsAStall) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    ASSERT_TRUE(drone->advance(30.0f).has_value());  // a 30 s stall
    EXPECT_LE(drone->readouts().tick, DroneSim::kMaxStepsPerAdvance);
    ASSERT_TRUE(drone->advance(30.0f).has_value());  // and the backlog was dropped, not carried
    EXPECT_LE(drone->readouts().tick, 2u * DroneSim::kMaxStepsPerAdvance);
}

TEST(SandboxDrone, TheAccumulatorStepsAtTheFixedRate) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    for (int i = 0; i < 60; ++i) ASSERT_TRUE(drone->advance(1.0f / 60.0f).has_value());  // one second
    const auto tick = drone->readouts().tick;
    EXPECT_GE(tick, 499u);
    EXPECT_LE(tick, 500u);
}

TEST(SandboxDrone, ZeroThrottleStaysFiniteAndPinned) {
    DronePhysicsOptions o;
    o.throttle = 0.0f;
    auto drone = DroneSim::create(o);
    ASSERT_TRUE(drone.has_value());
    drone->target = AttitudeTarget{0.3f, 0.2f, 0.1f};
    ASSERT_TRUE(drone->step_fixed(500).has_value());
    const DroneReadouts r = drone->readouts();
    EXPECT_TRUE(std::isfinite(r.orientation.w) && std::isfinite(r.omega_body.x));
    EXPECT_EQ((*drone->sim().body(drone->vehicle().body))->vel, glm::vec3(0.0f));
}

TEST(SandboxDrone, TheStandMountedImuReadsPlusG) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value());
    ASSERT_TRUE(drone->step_fixed(10).has_value());
    const DroneReadouts r = drone->readouts();
    EXPECT_NEAR(r.imu_accel.y, 9.80665f, 1e-4f);
    EXPECT_NEAR(r.imu_accel.x, 0.0f, 1e-4f);
    EXPECT_NEAR(r.imu_accel.z, 0.0f, 1e-4f);
}
