// test_gpu_module_config.cpp -- the device holds the hashed configuration image byte for byte (module API stage 7, Task C1).
//
// ModuleDesc::config is hashed into the schedule identity, and "the same bytes are what the kernels read": a module whose
// config_binding names gen::kBinding_airflow_config has its image uploaded there at create(), and Simulation::
// vulkan_config_image() reads it back. The identity rules are test_module_config.cpp (CPU). Skips with its reason when no
// device is admitted (TD-6).
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/backend.hpp"
#include "gpu_skip.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

namespace {

struct GridConfig {
    uint32_t nx, ny, nz, cell_mm;
};

[[nodiscard]] spade::Simulation make(const spade::modules::ModuleSet& set, spade::compute::BackendKind kind) {
    const spade::Result<spade::WorldSetDesc> world = spade::test::floor_world_set(2);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<spade::Simulation> sim = spade::Simulation::create(*world, 1'000'000, 2, spade::compute::BackendDesc{.kind = kind}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

}  // namespace

TEST(GpuModuleConfig, TheDeviceHoldsTheHashedImageByteForByte) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    static const GridConfig grid{16, 24, 32, 500};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::ModuleDesc{.name = "gridded",
                                             .config = spade::modules::config_image(grid),
                                             .config_binding = spade::compute::gen::kBinding_airflow_config});
    spade::Simulation sim = make(set, spade::compute::BackendKind::vulkan);
    const auto device = sim.vulkan_config_image();
    ASSERT_TRUE(device.has_value()) << device.error().context;
    const auto image = spade::modules::config_image(grid);
    ASSERT_EQ(device->size(), image.size());
    EXPECT_EQ(std::memcmp(device->data(), image.data(), image.size()), 0);
}

TEST(GpuModuleConfig, ASetWithNoConfigBindingHasNoConfigBufferAndNoSetOne) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    spade::Simulation sim = make(spade::modules::standard_modules(), spade::compute::BackendKind::vulkan);
    const auto device = sim.vulkan_config_image();
    ASSERT_FALSE(device.has_value()) << "no module binds a config image";
    EXPECT_EQ(device.error().code, spade::Code::unavailable);
}
