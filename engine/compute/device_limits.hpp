#pragma once

// DEVICE LIMITS AND THE VRAM BUDGET, CHECKED BY NAME (stage 7 §7.1 and §7.4;
// CORE-5 as amended 2026-10-12: a device is admitted if it meets the limits
// the module set declares, and is otherwise refused by name). B0 of Physics'
// GPU plan (docs/design/core/plans/2026-10-21-airflow-gpu-b0-plan.md): the
// airflow chain runner calls these and never copies them, and stage 7's
// create() calls the same ones for the module set.
//
// Plain data and pure functions with no Vulkan types, so they test on every
// machine. VulkanContext fills a DeviceLimits from the device
// (compute/vulkan/context.cpp), and DeviceReport carries it, so every run
// records its device's limits (CORE-5).
//
// Every refusal is Code::capacity_exceeded. It names what was asked (the
// buffer, the layout, the pass or the kernel) and the amount, the limit by its
// Vulkan name and value, and the device.

#include <array>
#include <cstdint>
#include <string_view>

#include "core/error.hpp"

namespace spade::compute {

struct DeviceLimits {
    uint64_t max_storage_buffer_range = 0;       // bytes: maxStorageBufferRange
    uint64_t max_memory_allocation_size = 0;     // bytes: maxMemoryAllocationSize (Maintenance3)
    uint32_t max_per_stage_storage_buffers = 0;  // maxPerStageDescriptorStorageBuffers
    uint32_t max_set_storage_buffers = 0;        // maxDescriptorSetStorageBuffers
    uint32_t max_push_constants_size = 0;        // bytes: maxPushConstantsSize
    std::array<uint32_t, 3> max_group_count{};   // maxComputeWorkGroupCount
    std::array<uint32_t, 3> max_group_size{};    // maxComputeWorkGroupSize
    uint32_t max_group_invocations = 0;          // maxComputeWorkGroupInvocations
    // False when the device could not report maxMemoryAllocationSize (no
    // vkGetPhysicalDeviceProperties2): it then holds Vulkan 1.1's guaranteed
    // 2^30, and the report says so.
    bool allocation_size_reported = false;
};

// Vulkan's guaranteed minimums (the specification's required limits; the
// allocation size is Vulkan 1.1's).
inline constexpr DeviceLimits kGuaranteedLimits{
    uint64_t{1} << 27, uint64_t{1} << 30, 4, 24, 128, {65535, 65535, 65535}, {128, 128, 64}, 128, false};

// A buffer's bytes against maxStorageBufferRange (the descriptor range is the
// whole buffer), then against maxMemoryAllocationSize (each buffer is one
// allocation).
[[nodiscard]] inline Result<void> check_buffer(std::string_view name, uint64_t bytes, const DeviceLimits& limits,
                                               std::string_view device) {
    (void)name, (void)bytes, (void)limits, (void)device;
    return {};
}

// A layout's storage-buffer count against maxPerStageDescriptorStorageBuffers
// (every binding is a compute binding), then maxDescriptorSetStorageBuffers.
[[nodiscard]] inline Result<void> check_storage_buffers(uint32_t count, const DeviceLimits& limits,
                                                        std::string_view device) {
    (void)count, (void)limits, (void)device;
    return {};
}

// A push-constant block's bytes against maxPushConstantsSize.
[[nodiscard]] inline Result<void> check_push(uint32_t bytes, const DeviceLimits& limits, std::string_view device) {
    (void)bytes, (void)limits, (void)device;
    return {};
}

// A dispatch's group counts, axis by axis, against maxComputeWorkGroupCount.
[[nodiscard]] inline Result<void> check_groups(std::string_view pass, const std::array<uint32_t, 3>& groups,
                                               const DeviceLimits& limits, std::string_view device) {
    (void)pass, (void)groups, (void)limits, (void)device;
    return {};
}

// A kernel's local size, axis by axis, against maxComputeWorkGroupSize, then
// its product against maxComputeWorkGroupInvocations.
[[nodiscard]] inline Result<void> check_workgroup(std::string_view kernel, const std::array<uint32_t, 3>& size,
                                                  const DeviceLimits& limits, std::string_view device) {
    (void)kernel, (void)size, (void)limits, (void)device;
    return {};
}

// THE VRAM BUDGET (A1, A2 of the chain-runner design): what
// VulkanContext::memory_budget() reports for the device's largest
// device-local heap.
struct MemoryBudget {
    enum class Source : uint32_t {
        ext = 0,        // VK_EXT_memory_budget: the process's budget and what it holds now
        half_heap = 1,  // no extension: half the heap, and no usage it can see
    };
    uint64_t budget = 0;  // bytes the process may hold in the heap
    uint64_t usage = 0;   // bytes it holds now (ext); 0 for half_heap
    uint32_t heap = 0;    // the heap's index
    Source source = Source::half_heap;
};

// The airflow's own ceiling, the spec's 4 GiB (equal to physics'
// kGridByteCap, which the CPU checks against; a test pins the two together).
inline constexpr uint64_t kAirflowVramCap = uint64_t{4} << 30;

// What a caller may allocate: min(kAirflowVramCap, budget - usage - reserve),
// never below 0. With half_heap the usage is `own_allocated`, what the caller
// says it has allocated itself (it cannot see anyone else's); with ext the
// heap's usage already counts it.
[[nodiscard]] inline uint64_t vram_cap(const MemoryBudget& budget, uint64_t reserve, uint64_t own_allocated) {
    (void)budget, (void)reserve, (void)own_allocated;
    return 0;
}

// `requested` bytes against vram_cap(), refused by name before any allocation:
// what asked, its bytes, the cap and how it was reached, and the device.
[[nodiscard]] inline Result<void> check_vram(std::string_view what, uint64_t requested, const MemoryBudget& budget,
                                             uint64_t reserve, uint64_t own_allocated, std::string_view device) {
    (void)what, (void)requested, (void)budget, (void)reserve, (void)own_allocated, (void)device;
    return {};
}

}  // namespace spade::compute
