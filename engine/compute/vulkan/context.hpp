#pragma once

// Headless Vulkan device context: instance + physical/logical device + one
// compute-capable queue + one transfer queue, loaded entirely through volk
// (no linked vulkan-1.lib / libvulkan.so -- every entry point is resolved
// dynamically at runtime, so a box or CI runner with no Vulkan loader
// installed still links and runs; it just gets Code::unavailable /
// vulkan_available() == false).
//
// No VkSurfaceKHR anywhere in this file or its .cpp, by construction --
// camera/rendering/FramePool is S7 (engine A2 deferred the FramePool, A8 the
// camera lane), out of scope for every S6 task; this module never asks the
// platform for a window.
//
// <volk.h> is included here deliberately: this pair of files (context.hpp/
// .cpp) is the actual Vulkan boundary the S6 dependency rule (spec section
// 2 -- "nothing outside engine/compute/ includes a Vulkan header") draws
// around, and vulkan/ is that boundary's own subdirectory within
// engine/compute/. backend.hpp, one directory up, stays Vulkan-free so a
// consumer that only needs BackendKind/BackendDesc never sees this header at
// all.

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include <volk.h>

#include "compute/backend.hpp"
#include "core/error.hpp"

namespace spade::compute {

// RAII owner of a Vulkan instance + logical device (spec section 11: rule of
// five on every resource owner). Move-only: an instance/device pair is a
// single logical resource with a real cleanup order requirement (device
// destroyed before instance), so copying would either double-destroy the
// same handles or require reference counting neither this type nor its
// first caller (a future dispatcher) needs.
class VulkanContext {
public:
    // Creates a headless instance + device bound to BackendDesc::device_index.
    //   - Code::unavailable: no Vulkan loader, or the loader enumerates zero
    //     physical devices -- exactly the condition vulkan_available() below
    //     reports, and the GTEST_SKIP() signal every device-executing test
    //     checks for first. ALSO (S6 Task 4) a device that IS present and
    //     enumerable but cannot preserve fp32 denormals in shaders
    //     (shaderDenormPreserveFloat32 == VK_FALSE, or a pre-1.1 device that
    //     cannot be asked at all). Every spade kernel declares
    //     `OpExecutionMode DenormPreserve 32`; requesting that on such a device
    //     is a Vulkan VALID-USAGE VIOLATION -- undefined behaviour, NOT a
    //     reported pipeline-creation error -- so this factory refuses the
    //     device outright rather than let a silently-flushing pipeline produce
    //     a divergent trajectory. The error's `context` names the capability
    //     and why parity requires it; the .cpp's
    //     device_preserves_fp32_denormals() carries the full argument. It
    //     remains the ONLY capability clause after S6 Task 8's shaderInt64
    //     probe -- see supports_int64() below for why that measurement changed
    //     the kernels instead of adding a second refusal.
    //   - Code::invalid_argument: a loader/device ARE present, but
    //     device_index names no enumerated physical device.
    //   - Code::internal: any other Vulkan API failure while creating the
    //     instance or device.
    [[nodiscard]] static Result<std::unique_ptr<VulkanContext>> create(const BackendDesc& desc);

    VulkanContext(const VulkanContext&) = delete;
    VulkanContext& operator=(const VulkanContext&) = delete;
    VulkanContext(VulkanContext&& other) noexcept;
    VulkanContext& operator=(VulkanContext&& other) noexcept;
    ~VulkanContext();

    // VkPhysicalDeviceProperties::deviceName, decoded once at create() time
    // -- what this task's GpuContext.* tests print as the box's literal
    // device-name verification line.
    [[nodiscard]] std::string_view device_name() const noexcept { return device_name_; }

    // VkPhysicalDeviceProperties::apiVersion (VK_MAKE_API_VERSION-encoded).
    // Decode with VK_API_VERSION_MAJOR/MINOR/PATCH (<volk.h>, transitively
    // <vulkan/vulkan_core.h>).
    [[nodiscard]] uint32_t api_version() const noexcept { return api_version_; }

    [[nodiscard]] uint32_t compute_queue_family() const noexcept { return compute_family_; }
    [[nodiscard]] uint32_t transfer_queue_family() const noexcept { return transfer_family_; }

    // ------------------------------------------------------------------
    // 64-BIT INTEGER ARITHMETIC IN SHADERS -- S6 Task 8's plan-mandated probe,
    // kept as a standing accessor because its answer decided a design and
    // uncovered a defect, and because a future device will answer differently.
    //
    // VkPhysicalDeviceFeatures::shaderInt64 for the device this context was
    // created on. **VK_FALSE ON THIS PROGRAM'S CORRECTNESS DEVICE** (Intel Iris
    // Plus; tests/test_compute_context.cpp's GpuContext.Int64ProbeMatchesThe-
    // Device prints the measurement and this task's report carries the
    // transcript).
    //
    // IT IS NOT A REQUIREMENT AND MUST NOT BECOME ONE. Two consequences follow
    // from the measurement, and both are load-bearing:
    //
    //   1. engine/shaders/rng.slang could not use a 64-bit type for
    //      splitmix64's state, so it uses engine/shaders/u64.slang's uint32-pair
    //      emulation -- exact by construction, and the reason the GPU's rng
    //      stream is a BIT-IDENTICAL claim rather than a banded one.
    //   2. shaders/shared/layouts.slang no longer mirrors ANY field as
    //      `uint64_t` (every one is a `uint2` with the identical std430 offset,
    //      size and alignment). Before Task 8 it did, and slangc consequently
    //      emitted `OpCapability Int64` into forces_drag.spv and integrate.spv
    //      -- a capability whose feature this device cannot enable, i.e. a
    //      Vulkan valid-usage violation that shipped silently from wave A
    //      because this box has no validation layer installed.
    //      engine/testing/spirv_scan.hpp's rule P4 now fails the build if any
    //      module declares it again.
    //
    // So this accessor exists to be REPORTED, not branched on: nothing in the
    // engine reads it, and a kernel that needed it would be a kernel this
    // device could not run.
    // ------------------------------------------------------------------
    [[nodiscard]] bool supports_int64() const noexcept { return int64_supported_; }

    // True iff this context created a VK_EXT_debug_utils messenger (S6 Task
    // 5): a debug build where VK_LAYER_KHRONOS_validation AND the
    // VK_EXT_debug_utils instance extension were both present at create()
    // time. When true, every validation-layer message this context's device
    // triggers for the rest of its lifetime is forwarded to
    // compute::invoke_error_sink() -- see this class's .cpp for the
    // messenger callback. Exposed so a test can assert the wiring actually
    // happened rather than trusting a message that never fires.
    [[nodiscard]] bool has_debug_messenger() const noexcept { return debug_messenger_ != VK_NULL_HANDLE; }

    [[nodiscard]] VkInstance instance() const noexcept { return instance_; }
    [[nodiscard]] VkPhysicalDevice physical_device() const noexcept { return physical_device_; }
    [[nodiscard]] VkDevice device() const noexcept { return device_; }
    [[nodiscard]] VkQueue compute_queue() const noexcept { return compute_queue_; }
    [[nodiscard]] VkQueue transfer_queue() const noexcept { return transfer_queue_; }

private:
    VulkanContext() = default;

    // Destroys device then instance, in that order, if owned; called from
    // the destructor and from move-assignment's "release what I used to
    // own" step. Idempotent -- safe to call on an already-moved-from
    // instance, where every handle is VK_NULL_HANDLE and every vkDestroy*
    // call below is a documented Vulkan no-op on VK_NULL_HANDLE.
    void destroy() noexcept;

    VkInstance instance_ = VK_NULL_HANDLE;
    VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue compute_queue_ = VK_NULL_HANDLE;
    VkQueue transfer_queue_ = VK_NULL_HANDLE;
    // S6 Task 5: VK_NULL_HANDLE unless a debug build found BOTH
    // VK_LAYER_KHRONOS_validation and VK_EXT_debug_utils present at create()
    // time. Destroyed before the instance (destroy()'s ordering), like every
    // other instance-level object this class owns.
    VkDebugUtilsMessengerEXT debug_messenger_ = VK_NULL_HANDLE;
    uint32_t compute_family_ = 0;
    uint32_t transfer_family_ = 0;
    uint32_t api_version_ = 0;
    // S6 Task 8: VkPhysicalDeviceFeatures::shaderInt64 as this device
    // advertised it (see supports_int64() above). Recorded, never required.
    bool int64_supported_ = false;
    std::string device_name_;
};

// Probe-only: true iff a Vulkan loader is present, it enumerates at least one
// physical device, AND that device can run this engine's kernels. Never throws
// (Vulkan's own C API can't; this function adds no exception-throwing calls of
// its own -- deliberately including no heap allocation, which is why it asks
// the loader for one device rather than building a vector of them) and never
// logs -- safe to call from every device-executing test's
// `if (!vulkan_available()) GTEST_SKIP()` guard (global constraint) without
// polluting -V output with driver/loader chatter on a CI runner that has
// neither.
//
// "CAN RUN THIS ENGINE'S KERNELS" IS PART OF THE PREDICATE (S6 Task 4, review
// fix round 2), not an extra condition a caller checks afterwards: every spade
// kernel declares `OpExecutionMode DenormPreserve 32`, so a device whose
// shaderDenormPreserveFloat32 is VK_FALSE (or which is too old to be asked) is
// one VulkanContext::create() refuses with Code::unavailable. STILL ONE
// CAPABILITY AFTER S6 TASK 8: shaderInt64 was probed, found VK_FALSE here, and
// designed around rather than demanded (see supports_int64() above). If this
// function
// answered `true` for such a device, every `GTEST_SKIP()` guard keyed on it
// would sail through and the test would then HARD-FAIL on create() -- on
// exactly the hardware the skip exists for, and against error.hpp's own
// Code::unavailable contract. One predicate, one truth.
//
// IT SPEAKS FOR THE DEFAULT DEVICE -- the first enumerated one, which
// BackendDesc::device_index's default of 0 selects and which every caller of
// this function goes on to create. A caller naming a different index is outside
// what a global predicate can answer and gets create()'s named error instead.
//
// Also false whenever the environment variable SPADE_FORCE_NO_VULKAN is set
// to "1", REGARDLESS of what the real probe would find -- host-only test
// support (this task's brief's forced-unavailable case: "vulkan_available()
// ==false path exercised by env override SPADE_FORCE_NO_VULKAN=1") so that
// path is exercisable even on the one box where the real probe always
// succeeds (this program's Iris device).
[[nodiscard]] bool vulkan_available() noexcept;

}  // namespace spade::compute
