// test_gpu_translation_lock.cpp -- the translation lock on Vulkan (module API stage 6, Task 5;
// docs/design/core/plans/2026-11-05-module-api-stage6-plan.md; spec section 9).
//
// integrate.slang holds a locked body's axes the way integrator.cpp does: after today's arithmetic, per held axis,
// behind PassParams::switches' lock bit. Position and velocity are WRITTEN under a lock, not integrated, so they are
// bit-exact against the CPU (the lock's grade: CPU reference, Vulkan banded, with pos and vel bit-exact). The IMU and
// the free axes are integrate's ordinary banded results.
//
// Every test skips with its reason when no device is admitted (TD-6); the host-only half is in
// test_translation_lock.cpp.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "gpu_skip.hpp"
#include "physics/lock.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/layout.hpp"

namespace {

using spade::BodyRef;
using spade::BodySpawn;
using spade::BodyState;
using spade::Result;
using spade::Simulation;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::physics::kLockAllAxes;
using spade::physics::kLockAxisX;
using spade::physics::kLockAxisZ;
using spade::test::bare_body;
using spade::test::floor_world_set;
using spade::test::row_of;

constexpr uint64_t kDt = 1'000'000;

[[nodiscard]] spade::modules::ModuleSet lock_set() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::lock_module());
    return set;
}

[[nodiscard]] Simulation make(const spade::modules::ModuleSet& set, uint32_t substeps, bool gpu) {
    const Result<spade::WorldSetDesc> world = floor_world_set(4);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    Result<Simulation> sim =
        Simulation::create(*world, kDt, substeps, gpu ? BackendDesc{.kind = BackendKind::vulkan} : BackendDesc{}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

[[nodiscard]] BodyRef spawn(Simulation& sim, const BodySpawn& body) {
    const BodyRef ref = sim.spawn(0, body).value();
    EXPECT_TRUE(sim.flush_structural().has_value());
    return ref;
}

[[nodiscard]] std::array<uint32_t, 3> bits(const glm::vec3& v) {
    std::array<uint32_t, 3> out{};
    std::memcpy(out.data(), &v, sizeof(v));
    return out;
}

[[nodiscard]] BodySpawn moving(glm::vec3 pos, glm::vec3 vel) {
    BodySpawn body = bare_body(pos);  // mass 0.8: not a power of two
    body.vel = vel;
    return body;
}

}  // namespace

TEST(GpuTranslationLock, PositionAndVelocityAreBitExactAgainstTheCpuUnderAFullLock) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation cpu = make(lock_set(), 2, false);
    Simulation gpu = make(lock_set(), 2, true);
    const glm::vec3 anchor(0.0f, 3.0f, 0.0f);
    BodySpawn spin = moving(anchor, glm::vec3(0.5f, -1.0f, 0.25f));
    spin.omega_body = glm::vec3(0.3f, -0.2f, 0.5f);
    const BodyRef on_cpu = spawn(cpu, spin);
    const BodyRef on_gpu = spawn(gpu, spin);
    ASSERT_TRUE(cpu.lock_translation(on_cpu, kLockAllAxes).has_value());
    ASSERT_TRUE(gpu.lock_translation(on_gpu, kLockAllAxes).has_value());
    for (int s = 0; s < 60; ++s) {
        ASSERT_TRUE(cpu.step(1).has_value());
        ASSERT_TRUE(gpu.step(1).has_value());
        const BodyState c = row_of(cpu, on_cpu);
        const BodyState g = row_of(gpu, on_gpu);
        ASSERT_EQ(bits(g.pos), bits(c.pos)) << "step " << s << ": written, not integrated";
        ASSERT_EQ(bits(g.pos), bits(anchor)) << "step " << s;
        ASSERT_EQ(bits(g.vel), bits(c.vel)) << "step " << s;
        ASSERT_EQ(bits(g.vel), bits(glm::vec3(0.0f))) << "step " << s << ": +0.0f";
    }
}

// The rail's free axis is integrate's ordinary arithmetic: on the device it is bit-equal to a free twin on the same
// device (the same kernel path, a body with no axes), and the held axes stay at the anchor.
TEST(GpuTranslationLock, TheYFreeRailHoldsXAndZAndClimbsLikeAFreeTwin) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation gpu = make(lock_set(), 2, true);
    const glm::vec3 a0(0.0f, 2.0f, 0.0f);
    const BodyRef railed = spawn(gpu, moving(a0, glm::vec3(0.4f, 0.1f, -0.3f)));
    const BodyRef twin = spawn(gpu, moving(glm::vec3(5.0f, 2.0f, 0.0f), glm::vec3(0.4f, 0.1f, -0.3f)));
    ASSERT_TRUE(gpu.lock_translation(railed, kLockAxisX | kLockAxisZ).has_value());
    for (int s = 0; s < 60; ++s) {
        for (const BodyRef r : {railed, twin}) {
            ASSERT_TRUE(gpu.apply_wrench(r, glm::vec3(0.0f, 12.0f, 0.0f), glm::vec3(0.0f)).has_value());
        }
        ASSERT_TRUE(gpu.step(1).has_value());
        const BodyState a = row_of(gpu, railed);
        const BodyState b = row_of(gpu, twin);
        ASSERT_EQ(a.pos.y, b.pos.y) << "step " << s;
        ASSERT_EQ(a.vel.y, b.vel.y) << "step " << s;
        ASSERT_EQ(a.pos.x, a0.x) << "step " << s;
        ASSERT_EQ(a.pos.z, a0.z) << "step " << s;
        ASSERT_EQ(bits(glm::vec3(a.vel.x, 0.0f, a.vel.z)), bits(glm::vec3(0.0f))) << "step " << s;
    }
}

// A PIN, green at red: a set that holds the lock module but locks nothing has the switch ON and every axes word 0, so
// its device state is bit-equal to the standard set's, whose switch is off (Review Focus 6).
TEST(GpuTranslationLock, ASetWithLockButNothingLockedIsBitEqualToTheStandardSet) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation standard = make(spade::modules::standard_modules(), 2, true);
    Simulation with_lock = make(lock_set(), 2, true);
    const BodySpawn body = moving(glm::vec3(0.0f, 0.3f, 0.0f), glm::vec3(0.4f, -2.0f, 0.1f));
    const BodyRef a = spawn(standard, body);
    const BodyRef b = spawn(with_lock, body);
    for (int s = 0; s < 100; ++s) {
        ASSERT_TRUE(standard.apply_wrench(a, glm::vec3(0.3f, 1.0f, -0.2f), glm::vec3(0.01f, 0.0f, 0.02f)).has_value());
        ASSERT_TRUE(with_lock.apply_wrench(b, glm::vec3(0.3f, 1.0f, -0.2f), glm::vec3(0.01f, 0.0f, 0.02f)).has_value());
        ASSERT_TRUE(standard.step(1).has_value());
        ASSERT_TRUE(with_lock.step(1).has_value());
        const BodyState x = row_of(standard, a);
        const BodyState y = row_of(with_lock, b);
        ASSERT_EQ(std::memcmp(&x, &y, sizeof(BodyState)), 0) << "step " << s;
    }
}

// The held body's IMU reads -R^T g on the device. integrate is banded against the CPU, so this is a tight tolerance,
// not bits: conj(q) * (-g) is a handful of fp32 operations.
TEST(GpuTranslationLock, AHeldBodysSpecificForceIsMinusRTgOnTheDevice) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation gpu = make(lock_set(), 1, true);
    BodySpawn spawn_body = bare_body(glm::vec3(0.0f, 3.0f, 0.0f));
    spawn_body.orient = glm::normalize(glm::quat(0.9f, 0.1f, 0.3f, -0.2f));
    const BodyRef ref = spawn(gpu, spawn_body);
    ASSERT_TRUE(gpu.lock_translation(ref, kLockAllAxes).has_value());
    const glm::vec3 gravity(0.0f, -9.80665f, 0.0f);
    for (int s = 0; s < 5; ++s) {
        const glm::quat before = row_of(gpu, ref).orient;
        ASSERT_TRUE(gpu.step(1).has_value());
        const glm::vec3 expected = glm::conjugate(before) * (-gravity);
        const glm::vec3 got = row_of(gpu, ref).specific_force;
        EXPECT_LT(glm::length(got - expected), 1e-5f * 9.80665f) << "step " << s;
    }
}

// The sandbox drone box runs two substeps. A level stand reads (0, +g, 0) there (Interface's
// TheStandMountedImuReadsPlusG, 1e-4, on both backends); a spinning held body reads g in norm and lags the step-start
// orientation by at most one substep of rotation (test_translation_lock.cpp has the CPU half and the argument).
TEST(GpuTranslationLock, ALevelHeldBodyReadsPlusGAtTwoSubstepsOnTheDevice) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation gpu = make(lock_set(), 2, true);
    const BodyRef ref = spawn(gpu, bare_body(glm::vec3(0.0f, 3.0f, 0.0f)));
    ASSERT_TRUE(gpu.lock_translation(ref, kLockAllAxes).has_value());
    for (int s = 0; s < 10; ++s) {
        ASSERT_TRUE(gpu.step(1).has_value());
        const glm::vec3 f = row_of(gpu, ref).specific_force;
        EXPECT_NEAR(f.x, 0.0f, 1e-4f) << "step " << s;
        EXPECT_NEAR(f.y, 9.80665f, 1e-4f) << "step " << s;
        EXPECT_NEAR(f.z, 0.0f, 1e-4f) << "step " << s;
    }
}

TEST(GpuTranslationLock, ASpinningHeldBodyReadsGWithinOneSubstepOfRotationOnTheDevice) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    constexpr uint32_t kSubsteps = 2;
    Simulation gpu = make(lock_set(), kSubsteps, true);
    BodySpawn spawn_body = bare_body(glm::vec3(0.0f, 3.0f, 0.0f));
    spawn_body.orient = glm::normalize(glm::quat(0.9f, 0.1f, 0.3f, -0.2f));
    spawn_body.omega_body = glm::vec3(0.3f, -0.2f, 0.5f);
    const BodyRef ref = spawn(gpu, spawn_body);
    ASSERT_TRUE(gpu.lock_translation(ref, kLockAllAxes).has_value());
    const glm::vec3 gravity(0.0f, -9.80665f, 0.0f);
    const float h_sub = static_cast<float>(kDt) * 1e-9f / static_cast<float>(kSubsteps);
    for (int s = 0; s < 20; ++s) {
        const BodyState start = row_of(gpu, ref);
        ASSERT_TRUE(gpu.step(1).has_value());
        const glm::vec3 f = row_of(gpu, ref).specific_force;
        EXPECT_NEAR(glm::length(f), 9.80665f, 1e-4f) << "step " << s;
        const glm::vec3 at_start = glm::conjugate(start.orient) * (-gravity);
        const float angle = std::atan2(glm::length(glm::cross(f, at_start)), glm::dot(f, at_start));
        EXPECT_LE(angle, glm::length(start.omega_body) * h_sub * 1.1f + 2e-5f) << "step " << s;
    }
}
