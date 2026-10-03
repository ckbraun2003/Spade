// test_gpu_module_schedule.cpp -- the Vulkan step records the compiled module
// schedule's GPU passes (module-API plan, stage 2). Device-executing, so every
// case is Gpu* (the gpu label) and skips without a device. Compiled only with
// SPADE_VULKAN: it includes a Vulkan header for vulkan_available(), which
// test_module_schedule.cpp must not.

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace {

// The standard set's compiled order. The two behavior passes record nothing,
// and neither do the two field-sample passes until the GPU samples fields
// (module-API stage 3, Task 3), but they are passes of the chain and are timed
// like any other.
const std::vector<std::string> kStandardGpuOrder = {
    "behaviors.kinematic",    "dryden.advance",          "dryden.sample",       "environment.sample",
    "rotor.forces",           "drag.forces",             "behaviors.force",     "static_contact.resolve",
    "dynamic_contact.resolve", "integrate.integrate",     "imu.synthesize",      "gnss.synthesize"};

[[nodiscard]] spade::compute::BackendDesc vulkan() { return {.kind = spade::compute::BackendKind::vulkan}; }

// test_module_schedule.cpp's one-body world, repeated here: that file is
// compiled without SPADE_VULKAN too, so the two cannot share a translation unit.
[[nodiscard]] spade::WorldSetDesc one_body_world() {
    auto world = spade::WorldBuilder()
                     .name("m")
                     .environment(spade::Environment{})
                     .capacities(spade::Capacities{1, 1, 1, 1})
                     .build();
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::WorldSetDesc{{inst}};
}

}  // namespace

TEST(GpuModuleSchedule, TheRecorderRecordsTheSchedulesPassesInOrder) {
    if (!spade::compute::vulkan_available()) GTEST_SKIP();
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, vulkan());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto chain = sim->vulkan_recorded_chain();
    ASSERT_TRUE(chain.has_value()) << chain.error().context;
    EXPECT_EQ(chain->passes, kStandardGpuOrder);
}

// imu and gnss both read body.pose but neither writes anything the other
// touches, so swapping them in the set swaps them in the schedule; the GPU
// must follow the schedule, not a remembered order.
TEST(GpuModuleSchedule, ReorderedSensorsRecordInScheduleOrder) {
    if (!spade::compute::vulkan_available()) GTEST_SKIP();
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    ASSERT_EQ(set[2].name, "imu");
    ASSERT_EQ(set[4].name, "gnss");
    std::swap(set[2], set[4]);
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, vulkan(), set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto chain = sim->vulkan_recorded_chain();
    ASSERT_TRUE(chain.has_value()) << chain.error().context;
    std::vector<std::string> expected = kStandardGpuOrder;
    std::swap(expected[10], expected[11]);
    EXPECT_EQ(chain->passes, expected);
}

TEST(GpuModuleSchedule, DurationsAreOnePerPassByName) {
    if (!spade::compute::vulkan_available()) GTEST_SKIP();
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, vulkan());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->step(1).has_value());
    const auto d = sim->vulkan_pass_durations_ns();
    ASSERT_TRUE(d.has_value()) << d.error().context;
    if (!d->supported) GTEST_SKIP() << "this device cannot time compute work";
    std::vector<std::string> names;
    for (const auto& p : d->passes) names.push_back(p.pass);
    EXPECT_EQ(names, kStandardGpuOrder);
    EXPECT_FALSE(d->find("no_such.pass").has_value()) << "a missing pass must not read as a zero duration";
}
