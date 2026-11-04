// test_gpu_contact_sets.cpp -- L10 on the device: static_contact's contact-set
// table reaches binding 28 at create() and after every register_model(), at
// its fixed capacity (L11), and a refused registration leaves it as it was.
// Device-executing, so every case is Gpu* (the gpu label) and skips without a
// device. Compiled only with SPADE_VULKAN.

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <vector>

#include "compute/backend.hpp"
#include "contact_set_fakes.hpp"
#include "core/error.hpp"
#include "gpu_skip.hpp"
#include "sim/simulation.hpp"

namespace {

using namespace contact_set_fakes;
using spade::Simulation;

[[nodiscard]] spade::compute::BackendDesc vulkan() { return {.kind = spade::compute::BackendKind::vulkan}; }

[[nodiscard]] std::vector<float> host_table(const Simulation& sim) {
    const auto t = sim.config_table(kContactSetsTable);
    EXPECT_TRUE(t.has_value()) << t.error().context;
    if (!t) return {};
    return {t->begin(), t->end()};
}

// The device table is the host's floats, then zeros to the capacity.
void expect_device_is_host(const Simulation& sim, const char* when) {
    const auto device = sim.vulkan_config_table(kContactSetsTable);
    ASSERT_TRUE(device.has_value()) << when << ": " << device.error().context;
    ASSERT_EQ(device->size(), std::size_t{kContactSetsCapacityRows} * kContactSetRowFloats) << when;
    const std::vector<float> host = host_table(sim);
    for (std::size_t i = 0; i < device->size(); ++i) {
        const float want = i < host.size() ? host[i] : 0.0f;
        if ((*device)[i] != want) {
            ADD_FAILURE() << when << ": float " << i << " is " << (*device)[i] << " on the device, " << want
                          << " on the host";
            return;
        }
    }
}

}  // namespace

TEST(GpuContactSets, TheDeviceTableIsTheHostsFromCreateThroughEachRegistration) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto sim = Simulation::create(world(false), 2'000'000, 2, vulkan(), standard_set_with<&build_a>());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    expect_device_is_host(*sim, "at create");

    ASSERT_TRUE(sim->register_model(quad_of_mass(1.0f)).has_value());
    expect_device_is_host(*sim, "after a model with no set");
    ASSERT_TRUE(sim->register_model(quad_of_mass(2.0f)).has_value());
    ASSERT_FALSE(host_table(*sim).empty());
    expect_device_is_host(*sim, "after a model with a set");
    const auto third = sim->register_model(quad_of_mass(4.0f));
    ASSERT_TRUE(third.has_value());
    expect_device_is_host(*sim, "after a second set");

    // Between steps too: the table is uploaded where register_model() runs.
    ASSERT_TRUE(sim->spawn(0, *third, at(0.0f, 10.0f)).has_value());
    ASSERT_TRUE(sim->step(2).has_value());
    ASSERT_TRUE(sim->register_model(quad_of_mass(5.0f)).has_value());
    expect_device_is_host(*sim, "after a registration between steps");
    ASSERT_TRUE(sim->step(2).has_value());
}

TEST(GpuContactSets, ARefusedRegistrationLeavesTheDeviceTableAsItWas) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto sim = Simulation::create(world(false), 2'000'000, 2, vulkan(), standard_set_with<&build_huge>());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->register_model(quad_of_mass(2.0f)).has_value());
    const auto before = sim->vulkan_config_table(kContactSetsTable);
    ASSERT_TRUE(before.has_value()) << before.error().context;

    const auto huge = sim->register_model(quad_of_mass(3.0f));
    ASSERT_FALSE(huge.has_value());
    EXPECT_EQ(huge.error().code, spade::Code::capacity_exceeded);
    const auto after = sim->vulkan_config_table(kContactSetsTable);
    ASSERT_TRUE(after.has_value()) << after.error().context;
    EXPECT_EQ(*after, *before);
    expect_device_is_host(*sim, "after the refusal");
}

TEST(GpuContactSets, TheDiagnosticNamesWhatItCannotRead) {
    auto cpu = Simulation::create(world(false), 2'000'000, 2);
    ASSERT_TRUE(cpu.has_value());
    const auto on_cpu = cpu->vulkan_config_table(kContactSetsTable);
    ASSERT_FALSE(on_cpu.has_value());
    EXPECT_EQ(on_cpu.error().code, spade::Code::unavailable);

    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto sim = Simulation::create(world(false), 2'000'000, 2, vulkan());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto missing = sim->vulkan_config_table("no_such_table");
    ASSERT_FALSE(missing.has_value());
    EXPECT_EQ(missing.error().code, spade::Code::not_found);
    EXPECT_NE(missing.error().context.find("no_such_table"), std::string::npos);
}
