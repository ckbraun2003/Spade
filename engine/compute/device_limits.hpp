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
#include <cstddef>
#include <cstdint>
#include <expected>
#include <string>
#include <string_view>

#include "core/error.hpp"

namespace spade::compute {

namespace detail {

[[nodiscard]] inline std::unexpected<Error> over_limit(const std::string& what, std::string_view limit,
                                                     uint64_t value, std::string_view unit, std::string_view device) {
    return std::unexpected(Error{Code::capacity_exceeded, what + ", over " + std::string(limit) + " (" +
                                                              std::to_string(value) + std::string(unit) +
                                                              ") on device '" + std::string(device) + "'"});
}

inline constexpr const char* kAxisNames[3] = {"x", "y", "z"};

}  // namespace detail

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
    const std::string what = "buffer '" + std::string(name) + "' is " + std::to_string(bytes) + " bytes";
    if (bytes > limits.max_storage_buffer_range) {
        return detail::over_limit(what, "maxStorageBufferRange", limits.max_storage_buffer_range, " bytes", device);
    }
    if (bytes > limits.max_memory_allocation_size) {
        return detail::over_limit(what, "maxMemoryAllocationSize", limits.max_memory_allocation_size, " bytes",
                                  device);
    }
    return {};
}

// A layout's storage-buffer count against maxPerStageDescriptorStorageBuffers
// (every binding is a compute binding), then maxDescriptorSetStorageBuffers.
[[nodiscard]] inline Result<void> check_storage_buffers(uint32_t count, const DeviceLimits& limits,
                                                        std::string_view device) {
    const std::string what = "a layout of " + std::to_string(count) + " storage buffers";
    if (count > limits.max_per_stage_storage_buffers) {
        return detail::over_limit(what, "maxPerStageDescriptorStorageBuffers", limits.max_per_stage_storage_buffers,
                                  "", device);
    }
    if (count > limits.max_set_storage_buffers) {
        return detail::over_limit(what, "maxDescriptorSetStorageBuffers", limits.max_set_storage_buffers, "", device);
    }
    return {};
}

// A push-constant block's bytes against maxPushConstantsSize.
[[nodiscard]] inline Result<void> check_push(uint32_t bytes, const DeviceLimits& limits, std::string_view device) {
    if (bytes > limits.max_push_constants_size) {
        return detail::over_limit("a push-constant block of " + std::to_string(bytes) + " bytes",
                                  "maxPushConstantsSize", limits.max_push_constants_size, " bytes", device);
    }
    return {};
}

// A dispatch's group counts, axis by axis, against maxComputeWorkGroupCount.
[[nodiscard]] inline Result<void> check_groups(std::string_view pass, const std::array<uint32_t, 3>& groups,
                                               const DeviceLimits& limits, std::string_view device) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (groups[axis] > limits.max_group_count[axis]) {
            return detail::over_limit("pass '" + std::string(pass) + "' dispatches " + std::to_string(groups[axis]) +
                                          " groups on axis " + detail::kAxisNames[axis],
                                      "maxComputeWorkGroupCount[" + std::to_string(axis) + "]",
                                      limits.max_group_count[axis], "", device);
        }
    }
    return {};
}

// A kernel's local size, axis by axis, against maxComputeWorkGroupSize, then
// its product against maxComputeWorkGroupInvocations.
[[nodiscard]] inline Result<void> check_workgroup(std::string_view kernel, const std::array<uint32_t, 3>& size,
                                                  const DeviceLimits& limits, std::string_view device) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (size[axis] > limits.max_group_size[axis]) {
            return detail::over_limit("kernel '" + std::string(kernel) + "' has a local size of " +
                                          std::to_string(size[axis]) + " on axis " + detail::kAxisNames[axis],
                                      "maxComputeWorkGroupSize[" + std::to_string(axis) + "]",
                                      limits.max_group_size[axis], "", device);
        }
    }
    const uint64_t invocations = uint64_t{size[0]} * size[1] * size[2];
    if (invocations > limits.max_group_invocations) {
        return detail::over_limit("kernel '" + std::string(kernel) + "' has " + std::to_string(invocations) +
                                      " invocations per workgroup",
                                  "maxComputeWorkGroupInvocations", limits.max_group_invocations, "", device);
    }
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
    const uint64_t used = budget.source == MemoryBudget::Source::ext ? budget.usage : own_allocated;
    if (budget.budget <= used || budget.budget - used <= reserve) return 0;
    const uint64_t free = budget.budget - used - reserve;
    return free < kAirflowVramCap ? free : kAirflowVramCap;
}

// `requested` bytes against vram_cap(), refused by name before any allocation:
// what asked, its bytes, the cap and how it was reached, and the device.
[[nodiscard]] inline Result<void> check_vram(std::string_view what, uint64_t requested, const MemoryBudget& budget,
                                             uint64_t reserve, uint64_t own_allocated, std::string_view device) {
    const uint64_t cap = vram_cap(budget, reserve, own_allocated);
    if (requested <= cap) return {};
    const bool ext = budget.source == MemoryBudget::Source::ext;
    const uint64_t used = ext ? budget.usage : own_allocated;
    return std::unexpected(Error{
        Code::capacity_exceeded,
        std::string(what) + " asks for " + std::to_string(requested) +
            " bytes of VRAM, over what the device can give it: the cap is " + std::to_string(cap) +
            " bytes = min(" + std::to_string(kAirflowVramCap) + ", budget " + std::to_string(budget.budget) +
            " - usage " + std::to_string(used) + " - reserve " + std::to_string(reserve) + "), from " +
            (ext ? std::string("VK_EXT_memory_budget")
                 : "half the device-local heap (no VK_EXT_memory_budget; the usage is what the caller allocated " +
                       std::to_string(own_allocated) + " itself)") +
            ", on device '" + std::string(device) + "'"});
}

}  // namespace spade::compute
