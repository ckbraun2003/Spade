// Device limits and the VRAM budget, checked by name (B0 of Physics' GPU plan:
// docs/design/core/plans/2026-10-21-airflow-gpu-b0-plan.md; stage 7 §7.1 and
// §7.4; CORE-5 as amended 2026-10-12). Pure functions over plain data, so these
// run on every machine; tests/test_gpu_device_limits.cpp reads a real device.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <string_view>

#include "compute/device_limits.hpp"
#include "core/error.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace {

using spade::Code;
using spade::compute::check_buffer;
using spade::compute::check_groups;
using spade::compute::check_push;
using spade::compute::check_storage_buffers;
using spade::compute::check_vram;
using spade::compute::check_workgroup;
using spade::compute::DeviceLimits;
using spade::compute::kGuaranteedLimits;
using spade::compute::MemoryBudget;
using spade::compute::vram_cap;

constexpr std::string_view kDevice = "Test Device 9000";
constexpr uint64_t kGiB = uint64_t{1} << 30;

[[nodiscard]] DeviceLimits guaranteed() {
    DeviceLimits limits = kGuaranteedLimits;
    limits.allocation_size_reported = true;
    return limits;
}

[[nodiscard]] bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

// Refused as capacity_exceeded, naming the device and every needle.
void expect_refused(const spade::Result<void>& r, std::initializer_list<std::string_view> needles) {
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::capacity_exceeded) << r.error().context;
    EXPECT_TRUE(contains(r.error().context, "device 'Test Device 9000'")) << r.error().context;
    for (const std::string_view needle : needles) {
        EXPECT_TRUE(contains(r.error().context, needle)) << "missing '" << needle << "' in: " << r.error().context;
    }
}

}  // namespace

TEST(DeviceLimits, TheGuaranteedMinimumsAreVulkans) {
    const DeviceLimits& g = kGuaranteedLimits;
    EXPECT_EQ(g.max_storage_buffer_range, uint64_t{1} << 27);
    EXPECT_EQ(g.max_memory_allocation_size, uint64_t{1} << 30);
    EXPECT_EQ(g.max_per_stage_storage_buffers, 4u);
    EXPECT_EQ(g.max_set_storage_buffers, 24u);
    EXPECT_EQ(g.max_push_constants_size, 128u);
    EXPECT_EQ(g.max_group_count, (std::array<uint32_t, 3>{65535, 65535, 65535}));
    EXPECT_EQ(g.max_group_size, (std::array<uint32_t, 3>{128, 128, 64}));
    EXPECT_EQ(g.max_group_invocations, 128u);
    EXPECT_FALSE(g.allocation_size_reported);
}

TEST(DeviceLimits, EachLimitPassesAtItsValueAndIsRefusedOnePast) {
    const DeviceLimits l = guaranteed();
    EXPECT_TRUE(check_buffer("airflow_u", l.max_storage_buffer_range, l, kDevice).has_value());
    expect_refused(check_buffer("airflow_u", l.max_storage_buffer_range + 1, l, kDevice),
                   {"buffer 'airflow_u' is 134217729 bytes", "maxStorageBufferRange (134217728 bytes)"});

    // A device whose range passes the allocation size: the allocation answers.
    DeviceLimits wide_range = l;
    wide_range.max_storage_buffer_range = uint64_t{1} << 32;
    EXPECT_TRUE(check_buffer("airflow_advect", l.max_memory_allocation_size, wide_range, kDevice).has_value());
    expect_refused(check_buffer("airflow_advect", l.max_memory_allocation_size + 1, wide_range, kDevice),
                   {"buffer 'airflow_advect' is 1073741825 bytes", "maxMemoryAllocationSize (1073741824 bytes)"});

    EXPECT_TRUE(check_storage_buffers(4, l, kDevice).has_value());
    expect_refused(check_storage_buffers(5, l, kDevice),
                   {"a layout of 5 storage buffers", "maxPerStageDescriptorStorageBuffers (4)"});
    DeviceLimits wide_stage = l;
    wide_stage.max_per_stage_storage_buffers = 1000;
    EXPECT_TRUE(check_storage_buffers(24, wide_stage, kDevice).has_value());
    expect_refused(check_storage_buffers(25, wide_stage, kDevice),
                   {"a layout of 25 storage buffers", "maxDescriptorSetStorageBuffers (24)"});

    EXPECT_TRUE(check_push(128, l, kDevice).has_value());
    expect_refused(check_push(129, l, kDevice),
                   {"a push-constant block of 129 bytes", "maxPushConstantsSize (128 bytes)"});
}

TEST(DeviceLimits, TheGroupCheckNamesThePassAndTheAxis) {
    const DeviceLimits l = guaranteed();
    EXPECT_TRUE(check_groups("advect", {65535, 65535, 65535}, l, kDevice).has_value());
    expect_refused(check_groups("advect", {1, 65536, 1}, l, kDevice),
                   {"pass 'advect' dispatches 65536 groups on axis y", "maxComputeWorkGroupCount[1] (65535)"});
    expect_refused(check_groups("advect", {65536, 1, 1}, l, kDevice), {"on axis x", "maxComputeWorkGroupCount[0]"});
}

TEST(DeviceLimits, TheWorkgroupCheckNamesTheKernel) {
    const DeviceLimits l = guaranteed();
    EXPECT_TRUE(check_workgroup("tile_sum", {4, 4, 4}, l, kDevice).has_value());
    EXPECT_TRUE(check_workgroup("row", {128, 1, 1}, l, kDevice).has_value());
    expect_refused(check_workgroup("tall", {1, 1, 65}, l, kDevice),
                   {"kernel 'tall' has a local size of 65 on axis z", "maxComputeWorkGroupSize[2] (64)"});
    expect_refused(check_workgroup("fat", {8, 8, 4}, l, kDevice),
                   {"kernel 'fat' has 256 invocations per workgroup", "maxComputeWorkGroupInvocations (128)"});
}

TEST(VramBudget, TheCapIsTheSmallerOfFourGibAndTheFreeBudget) {
    static_assert(spade::compute::kAirflowVramCap == spade::physics::airflow::kGridByteCap,
                  "the GPU's ceiling is the CPU's");
    MemoryBudget b{.budget = 7 * kGiB, .usage = 1 * kGiB, .heap = 0, .source = MemoryBudget::Source::ext};
    EXPECT_EQ(vram_cap(b, kGiB, 0), 4 * kGiB);  // 7 - 1 - 1 = 5, over the 4 GiB ceiling
    b.usage = 3 * kGiB;
    EXPECT_EQ(vram_cap(b, kGiB, 0), 3 * kGiB);
    b.usage = 7 * kGiB;
    EXPECT_EQ(vram_cap(b, kGiB, 0), 0u);  // never below 0
    b.usage = 6 * kGiB + kGiB / 2;
    EXPECT_EQ(vram_cap(b, kGiB, 0), 0u);
}

TEST(VramBudget, AGridOverTheLiveBudgetIsRefusedNamingTheBytesAndTheCap) {
    // Another program holds VRAM: the grid is under the 4 GiB ceiling but over
    // what is free, and is refused by name before any allocation.
    const MemoryBudget b{.budget = 7 * kGiB, .usage = 4 * kGiB, .heap = 0, .source = MemoryBudget::Source::ext};
    EXPECT_TRUE(check_vram("the airflow grid", 2 * kGiB, b, kGiB, 0, kDevice).has_value());
    expect_refused(check_vram("the airflow grid", 3 * kGiB, b, kGiB, 0, kDevice),
                   {"the airflow grid asks for 3221225472 bytes of VRAM", "the cap is 2147483648 bytes",
                    "budget 7516192768 - usage 4294967296 - reserve 1073741824", "VK_EXT_memory_budget"});
}

TEST(VramBudget, HalfHeapSubtractsWhatTheCallerAllocated) {
    MemoryBudget half{.budget = 4 * kGiB, .usage = 0, .heap = 1, .source = MemoryBudget::Source::half_heap};
    EXPECT_EQ(vram_cap(half, kGiB, kGiB), 2 * kGiB);
    expect_refused(check_vram("the airflow grid", 2 * kGiB + 1, half, kGiB, kGiB, kDevice),
                   {"the cap is 2147483648 bytes", "half the device-local heap", "allocated 1073741824 itself"});
    // With the extension the heap's usage already counts the caller's own.
    const MemoryBudget ext{.budget = 4 * kGiB, .usage = 0, .heap = 1, .source = MemoryBudget::Source::ext};
    EXPECT_EQ(vram_cap(ext, kGiB, kGiB), 3 * kGiB);
}
