// test_gpu_world_wind.cpp -- Simulation::set_world_wind on the device (L46). The set writes the registered WorldParams row and marks the
// mirror dirty, so the next step uploads it: the device's field wind, read back, equals the CPU's and the set value bit for bit
// (no turbulence, so the wind is a copy, exact in fp32). The host half is test_world_wind.cpp.
#include <gtest/gtest.h>

#include <bit>
#include <cstdint>
#include <utility>

#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "gpu_skip.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace {

constexpr glm::vec3 kCreated{3.0f, 0.0f, 1.0f};
constexpr glm::vec3 kSet{-4.0f, 5.5f, 0.25f};

[[nodiscard]] spade::WorldSetDesc two_calm_worlds() {
    spade::Environment env;
    env.wind = kCreated;
    const auto world = spade::WorldBuilder().name("w").environment(env).capacities(spade::Capacities{1, 1, 1, 1}).build();
    EXPECT_TRUE(world.has_value());
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    spade::WorldSetDesc set{{inst, inst}};
    set.worlds[1].seed += 1;
    return set;
}

[[nodiscard]] spade::Simulation make(spade::compute::BackendKind kind, uint32_t substeps) {
    auto sim = spade::Simulation::create(two_calm_worlds(), 1'000'000, substeps, spade::compute::BackendDesc{.kind = kind});
    EXPECT_TRUE(sim.has_value()) << (sim ? "" : sim.error().context);
    return std::move(*sim);
}

[[nodiscard]] glm::vec3 field_wind(const spade::Simulation& sim, uint32_t world) {
    const auto f = sim.field_samples(world);
    EXPECT_TRUE(f.has_value());
    if (!f) return glm::vec3(0.0f);
    const uint32_t o = spade::modules::kFieldWindOffset;
    return glm::vec3((*f)[o], (*f)[o + 1], (*f)[o + 2]);
}

[[nodiscard]] bool bits_equal(glm::vec3 a, glm::vec3 b) {
    return std::bit_cast<uint32_t>(a.x) == std::bit_cast<uint32_t>(b.x) && std::bit_cast<uint32_t>(a.y) == std::bit_cast<uint32_t>(b.y) &&
           std::bit_cast<uint32_t>(a.z) == std::bit_cast<uint32_t>(b.z);
}

}  // namespace

// The device and the CPU agree on the set value bit for bit after a set, at one substep and at four (the last substep reads the
// row the set wrote, as does every earlier one); the world that was not set keeps its creation wind on both.
TEST(GpuWorldWind, ASetWindReachesTheDeviceBitEqualToTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    for (const uint32_t substeps : {1u, 4u}) {
        spade::Simulation cpu = make(spade::compute::BackendKind::cpu, substeps);
        spade::Simulation gpu = make(spade::compute::BackendKind::vulkan, substeps);
        ASSERT_TRUE(cpu.step(2).has_value());
        ASSERT_TRUE(gpu.step(2).has_value());
        ASSERT_TRUE(bits_equal(field_wind(gpu, 1), kCreated)) << "before the set, substeps " << substeps;
        ASSERT_TRUE(cpu.set_world_wind(1, kSet).has_value());
        ASSERT_TRUE(gpu.set_world_wind(1, kSet).has_value());
        ASSERT_TRUE(cpu.step(1).has_value());
        ASSERT_TRUE(gpu.step(1).has_value());
        EXPECT_TRUE(bits_equal(field_wind(cpu, 1), kSet)) << "substeps " << substeps;
        EXPECT_TRUE(bits_equal(field_wind(gpu, 1), kSet)) << "substeps " << substeps;
        EXPECT_TRUE(bits_equal(field_wind(gpu, 0), kCreated)) << "world 0 was not set, substeps " << substeps;
        const auto medium = gpu.sample_medium(1, glm::vec3(0.0f));
        ASSERT_TRUE(medium.has_value());
        EXPECT_TRUE(bits_equal(medium->wind, kSet)) << "substeps " << substeps;
    }
}

// A set after a step is not erased by the next readback: the mirror was marked dirty, so the device runs the new wind and the arenas
// keep it (the failure a missing mark_vulkan_dirty() has, as reseed()'s note says).
TEST(GpuWorldWind, ASetIsNotErasedByTheNextReadback) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    spade::Simulation gpu = make(spade::compute::BackendKind::vulkan, 2);
    ASSERT_TRUE(gpu.step(1).has_value());
    ASSERT_TRUE(gpu.set_world_wind(0, kSet).has_value());
    ASSERT_TRUE(gpu.step(3).has_value());
    const auto row = gpu.world_params(0);
    ASSERT_TRUE(row.has_value());
    EXPECT_TRUE(bits_equal(glm::vec3((*row)->wind), kSet));
}
