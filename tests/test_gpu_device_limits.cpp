// A real device's limits and VRAM budget (B0 of Physics' GPU plan;
// docs/design/core/plans/2026-10-21-airflow-gpu-b0-plan.md). The checks
// themselves are pure and tested in tests/test_device_limits.cpp; these read
// the device of record and print what it admits, for the gate log.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

#include "compute/backend.hpp"
#include "compute/device_limits.hpp"
#include "compute/vulkan/context.hpp"
#include "gpu_skip.hpp"

namespace {

using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::DeviceLimits;
using spade::compute::kGuaranteedLimits;
using spade::compute::MemoryBudget;
using spade::compute::VulkanContext;

[[nodiscard]] bool advertises(VkPhysicalDevice device, std::string_view name) {
    uint32_t count = 0;
    EXPECT_EQ(vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr), VK_SUCCESS);
    std::vector<VkExtensionProperties> extensions(count);
    EXPECT_EQ(vkEnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data()), VK_SUCCESS);
    for (const VkExtensionProperties& extension : extensions) {
        if (std::string_view(extension.extensionName) == name) return true;
    }
    return false;
}

}  // namespace

TEST(GpuDeviceLimits, TheReportCarriesTheDevicesLimits) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto context = VulkanContext::create(BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(context.has_value()) << context.error().context;
    const DeviceLimits& l = (*context)->device_report().limits;
    const DeviceLimits& g = kGuaranteedLimits;
    std::cout << "[GpuDeviceLimits] " << (*context)->device_name()
              << ": maxStorageBufferRange " << l.max_storage_buffer_range << ", maxMemoryAllocationSize "
              << l.max_memory_allocation_size << (l.allocation_size_reported ? "" : " (not reported: guaranteed)")
              << ", storage buffers per stage " << l.max_per_stage_storage_buffers << " / per set "
              << l.max_set_storage_buffers << ", push " << l.max_push_constants_size << ", groups "
              << l.max_group_count[0] << "x" << l.max_group_count[1] << "x" << l.max_group_count[2] << ", local "
              << l.max_group_size[0] << "x" << l.max_group_size[1] << "x" << l.max_group_size[2] << " ("
              << l.max_group_invocations << " invocations)\n";
    EXPECT_GE(l.max_storage_buffer_range, g.max_storage_buffer_range);
    EXPECT_GE(l.max_memory_allocation_size, g.max_memory_allocation_size);
    EXPECT_GE(l.max_per_stage_storage_buffers, g.max_per_stage_storage_buffers);
    EXPECT_GE(l.max_set_storage_buffers, g.max_set_storage_buffers);
    EXPECT_GE(l.max_push_constants_size, g.max_push_constants_size);
    for (std::size_t axis = 0; axis < 3; ++axis) {
        EXPECT_GE(l.max_group_count[axis], g.max_group_count[axis]) << "axis " << axis;
        EXPECT_GE(l.max_group_size[axis], g.max_group_size[axis]) << "axis " << axis;
    }
    EXPECT_GE(l.max_group_invocations, g.max_group_invocations);
    // Every device of record runs Vulkan 1.1 or later, so it reports the
    // allocation size (Maintenance3 is 1.1 core).
    EXPECT_TRUE(l.allocation_size_reported);

    // Each field is the device's own value, read here straight from the
    // driver: a guaranteed minimum standing in for it, or two fields
    // swapped, fails.
    VkPhysicalDeviceMaintenance3Properties maintenance3{};
    maintenance3.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MAINTENANCE_3_PROPERTIES;
    VkPhysicalDeviceProperties2 properties{};
    properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    properties.pNext = &maintenance3;
    vkGetPhysicalDeviceProperties2((*context)->physical_device(), &properties);
    const VkPhysicalDeviceLimits& d = properties.properties.limits;
    EXPECT_EQ(l.max_storage_buffer_range, d.maxStorageBufferRange);
    EXPECT_EQ(l.max_memory_allocation_size, maintenance3.maxMemoryAllocationSize);
    EXPECT_EQ(l.max_per_stage_storage_buffers, d.maxPerStageDescriptorStorageBuffers);
    EXPECT_EQ(l.max_set_storage_buffers, d.maxDescriptorSetStorageBuffers);
    EXPECT_EQ(l.max_push_constants_size, d.maxPushConstantsSize);
    for (std::size_t axis = 0; axis < 3; ++axis) {
        EXPECT_EQ(l.max_group_count[axis], d.maxComputeWorkGroupCount[axis]) << "axis " << axis;
        EXPECT_EQ(l.max_group_size[axis], d.maxComputeWorkGroupSize[axis]) << "axis " << axis;
    }
    EXPECT_EQ(l.max_group_invocations, d.maxComputeWorkGroupInvocations);
}

TEST(GpuMemoryBudget, ReportsItsSourceAndABudgetWithinTheHeap) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto context = VulkanContext::create(BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(context.has_value()) << context.error().context;
    VkPhysicalDeviceMemoryProperties memory{};
    vkGetPhysicalDeviceMemoryProperties((*context)->physical_device(), &memory);
    const MemoryBudget b = (*context)->memory_budget();
    std::cout << "[GpuMemoryBudget] " << (*context)->device_name() << ": heap " << b.heap << ", budget "
              << b.budget << ", usage " << b.usage << ", source "
              << (b.source == MemoryBudget::Source::ext ? "VK_EXT_memory_budget" : "half the heap") << "\n";
    ASSERT_LT(b.heap, memory.memoryHeapCount);
    const VkMemoryHeap& heap = memory.memoryHeaps[b.heap];
    EXPECT_NE(heap.flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT, 0u);
    // The largest device-local heap, the lowest index on a tie.
    for (uint32_t i = 0; i < memory.memoryHeapCount; ++i) {
        if ((memory.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) == 0) continue;
        EXPECT_TRUE(memory.memoryHeaps[i].size < heap.size || (memory.memoryHeaps[i].size == heap.size && i >= b.heap))
            << "heap " << i;
    }
    EXPECT_GT(b.budget, 0u);
    EXPECT_LE(b.budget, heap.size);
    // The source follows the device: a device that advertises the extension
    // is read through it (so other programs' VRAM counts), and only one that
    // does not falls back to half the heap.
    const bool ext = advertises((*context)->physical_device(), VK_EXT_MEMORY_BUDGET_EXTENSION_NAME);
    EXPECT_EQ(b.source, ext ? MemoryBudget::Source::ext : MemoryBudget::Source::half_heap);
    if (b.source == MemoryBudget::Source::half_heap) {
        EXPECT_EQ(b.budget, heap.size / 2);
        EXPECT_EQ(b.usage, 0u);
    }
}
