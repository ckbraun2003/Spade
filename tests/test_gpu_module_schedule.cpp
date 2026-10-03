// test_gpu_module_schedule.cpp -- the Vulkan step records the compiled module
// schedule's GPU passes (module-API plan, stage 2). Device-executing, so every
// case is Gpu* (the gpu label) and skips without a device. Compiled only with
// SPADE_VULKAN: it includes a Vulkan header for vulkan_available(), which
// test_module_schedule.cpp must not.

#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstdint>
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
// but they are passes of the chain and are timed like any other.
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

// ---------------------------------------------------------------------------
// Module-API stage 3: the GPU samples fields. Two worlds with different
// gravity, density and wind, so a sample written to or read from the wrong
// world's row cannot pass. Gravity and density are copies, so the device row
// matches the CPU row bitwise; wind carries Dryden's GPU band, which the
// parity suite already gates through every rotor and drag force, so here it
// only has to be present and finite.
// ---------------------------------------------------------------------------
namespace {

[[nodiscard]] spade::WorldSetDesc two_different_turbulent_worlds() {
    spade::WorldSetDesc set;
    const float gravity_y[] = {-9.80665f, -3.72076f};
    const float density[] = {1.225f, 0.0200f};
    for (uint32_t w = 0; w < 2; ++w) {
        spade::Environment env;
        env.gravity = glm::vec3(0.0f, gravity_y[w], 0.0f);
        env.air_density = density[w];
        env.wind = glm::vec3(2.0f + static_cast<float>(w), 0.0f, -1.0f);
        spade::WorldInstanceDesc inst;
        inst.world = spade::WorldBuilder()
                         .name(w == 0 ? "earth" : "mars")
                         .environment(env)
                         .capacities(spade::Capacities{1, 1, 1, 1})
                         .build()
                         .value();
        inst.seed = 0xF1E1D5u + w;
        inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::moderate);
        set.worlds.push_back(inst);
    }
    return set;
}

}  // namespace

TEST(GpuModuleSchedule, StoredFieldSamplesMatchTheCpuCopiesBitwise) {
    if (!spade::compute::vulkan_available()) GTEST_SKIP();
    auto cpu = spade::Simulation::create(two_different_turbulent_worlds(), 2'000'000, 2);
    auto gpu = spade::Simulation::create(two_different_turbulent_worlds(), 2'000'000, 2, vulkan());
    ASSERT_TRUE(cpu.has_value()) << cpu.error().context;
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    ASSERT_TRUE(cpu->step(1).has_value());
    ASSERT_TRUE(gpu->step(1).has_value());
    for (uint32_t w = 0; w < 2; ++w) {
        const auto c = cpu->field_samples(w);
        const auto g = gpu->field_samples(w);
        ASSERT_TRUE(c.has_value()) << c.error().context;
        ASSERT_TRUE(g.has_value()) << g.error().context;
        ASSERT_GE(g->size(), std::size_t{spade::modules::kFieldBuiltinFloats});
        for (uint32_t i = 0; i < 4; ++i) {
            EXPECT_EQ(std::bit_cast<uint32_t>((*c)[i]), std::bit_cast<uint32_t>((*g)[i]))
                << "world " << w << " float " << i << " (gravity, density: copies)";
        }
        for (uint32_t i = spade::modules::kFieldWindOffset; i < spade::modules::kFieldWindOffset + 3; ++i) {
            EXPECT_TRUE(std::isfinite((*g)[i])) << "world " << w << " wind float " << i;
            EXPECT_NE((*g)[i], 0.0f) << "world " << w << " wind float " << i << ": the device row was never written";
        }
    }
}
