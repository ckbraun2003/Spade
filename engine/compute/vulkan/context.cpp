#include "compute/vulkan/context.hpp"

#include <cstdlib>
#include <limits>
#include <utility>
#include <vector>


namespace spade::compute {

namespace {

// std::getenv itself, not just an unguarded call to it: MSVC's C4996 flags
// getenv as one of its "unsafe" CRT functions (fine to call, but /W4 /WX --
// spade_warnings, linked into this target -- turns that warning into a hard
// build failure), and this project does not carry a blanket
// _CRT_SECURE_NO_WARNINGS suppression anywhere else, so this file doesn't
// start one either. _dupenv_s is MSVC's own suggested safe replacement;
// non-MSVC has no such split, so std::getenv (never flagged by
// -Wall -Wextra -Wpedantic there) stays the POSIX path.
[[nodiscard]] bool env_forces_no_vulkan() noexcept {
#ifdef _WIN32
    char* value = nullptr;
    size_t value_len = 0;
    if (_dupenv_s(&value, &value_len, "SPADE_FORCE_NO_VULKAN") != 0 || value == nullptr) {
        return false;
    }
    const bool forced = std::string_view(value) == "1";
    free(value);
    return forced;
#else
    const char* value = std::getenv("SPADE_FORCE_NO_VULKAN");
    return value != nullptr && std::string_view(value) == "1";
#endif
}

// First queue family advertising VK_QUEUE_COMPUTE_BIT. Every
// Vulkan-conformant device has at least one (the spec requires it), so
// returning "not found" here would itself indicate a driver bug outside
// anything this function can recover from -- callers only reach it after
// vkEnumeratePhysicalDevices has already succeeded for this device.
[[nodiscard]] uint32_t pick_compute_family(const std::vector<VkQueueFamilyProperties>& families) {
    for (uint32_t i = 0; i < families.size(); ++i) {
        if (families[i].queueFlags & VK_QUEUE_COMPUTE_BIT) return i;
    }
    return std::numeric_limits<uint32_t>::max();
}

// Prefers a DEDICATED transfer family (VK_QUEUE_TRANSFER_BIT set, neither
// GRAPHICS nor COMPUTE) -- the classic async-copy-queue pattern -- and
// falls back to the compute family when no such family exists. That
// fallback is not a theoretical branch: the box this program develops
// against (an Intel Iris Plus integrated GPU) exposes exactly one queue
// family, so it is what every GpuContext.* test below actually exercises.
[[nodiscard]] uint32_t pick_transfer_family(const std::vector<VkQueueFamilyProperties>& families,
                                             uint32_t compute_family) {
    for (uint32_t i = 0; i < families.size(); ++i) {
        const VkQueueFlags flags = families[i].queueFlags;
        if ((flags & VK_QUEUE_TRANSFER_BIT) && !(flags & VK_QUEUE_GRAPHICS_BIT) &&
            !(flags & VK_QUEUE_COMPUTE_BIT)) {
            return i;
        }
    }
    return compute_family;
}

// WHAT THIS DEVICE IS, AND HOW IT TREATS fp32 DENORMALS (CORE-5): recorded,
// never a requirement. No kernel requests a denormal mode (SPIR-V rule P3), so
// every device runs them legally with its own default, and its results are
// banded against the CPU (TD-14). The report says which device and driver a
// run used, announced and never silent (L6). Driver and float-control
// properties are Vulkan 1.2 core, so below 1.2 they stay empty and false.
[[nodiscard]] DeviceReport read_device_report(VkPhysicalDevice device,
                                              const VkPhysicalDeviceProperties& properties) {
    DeviceReport report;
    report.device_name = properties.deviceName;
    report.vendor_id = properties.vendorID;
    report.device_id = properties.deviceID;
    report.api_version = properties.apiVersion;
    report.driver_version = properties.driverVersion;
    if (properties.apiVersion < VK_API_VERSION_1_2 || vkGetPhysicalDeviceProperties2 == nullptr) {
        return report;
    }

    VkPhysicalDeviceFloatControlsProperties float_controls{};
    float_controls.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FLOAT_CONTROLS_PROPERTIES;
    VkPhysicalDeviceDriverProperties driver{};
    driver.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DRIVER_PROPERTIES;
    driver.pNext = &float_controls;
    VkPhysicalDeviceProperties2 properties2{};
    properties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    properties2.pNext = &driver;
    vkGetPhysicalDeviceProperties2(device, &properties2);

    report.float_controls_queryable = true;
    report.driver_name = driver.driverName;
    report.driver_info = driver.driverInfo;
    report.denorm_preserve_f32 = float_controls.shaderDenormPreserveFloat32 == VK_TRUE;
    report.denorm_flush_to_zero_f32 = float_controls.shaderDenormFlushToZeroFloat32 == VK_TRUE;
    return report;
}

// Does this physical device support 64-bit integer arithmetic in shaders?
//
// S6 TASK 8's PLAN-MANDATED PROBE, RUN BEFORE A LINE OF rng.slang WAS WRITTEN,
// AND ITS ANSWER ON THIS BOX IS **NO**:
//
//     [GpuContext] shaderInt64 advertised=0 device=Intel(R) Iris(R) Plus Graphics
//
// (tests/test_compute_context.cpp's GpuContext.Int64ProbeMatchesTheDevice
// prints exactly that line; this task's report carries the ctest transcript.)
//
// WHAT THAT DECIDED, AND WHAT IT UNCOVERED. The decision it was run to make was
// rng.slang's spelling: splitmix64's state is a uint64 and its two multiplies
// and three xor-shifts must be EXACT, so with the feature absent the uint32-pair
// emulation is the only legal spelling -- engine/shaders/u64.slang, written for
// that reason.
//
// It also uncovered a PRE-EXISTING VALID-USAGE VIOLATION. Scanning the compiled
// artifacts, forces_drag.spv and integrate.spv each carried `OpCapability
// Int64` from S6 Task 6 onward -- not because any kernel did 64-bit arithmetic,
// but because they subscript `world_params` and shaders/shared/layouts.slang
// mirrored WorldParams::seed as a `uint64_t`, so slangc declared the 64-bit
// TYPE. A SPIR-V module may only declare a capability whose feature the logical
// device enabled, and this device cannot enable that one at all. Task 8's fix
// is to remove the dependence rather than to demand the feature: every
// `uint64_t` in layouts.slang is now a `uint2` (identical std430 offset, size
// and alignment -- the generated static_asserts prove it), so NO module
// declares Int64 any more, and engine/testing/spirv_scan.hpp's rule P4 is the
// build-gated tripwire that keeps it that way.
//
// vkGetPhysicalDeviceFeatures (not ...Features2) deliberately: shaderInt64 is a
// CORE Vulkan 1.0 feature, so the 1.0 entry point answers it on every device
// this engine can otherwise accept, and using it keeps this probe free of the
// 1.1 guard read_device_report()'s Vulkan 1.2 properties need.
[[nodiscard]] bool device_supports_int64(VkPhysicalDevice device) noexcept {
    VkPhysicalDeviceFeatures features{};
    vkGetPhysicalDeviceFeatures(device, &features);
    return features.shaderInt64 == VK_TRUE;
}

// ---------------------------------------------------------------------------
// S6 Task 5: the debug messenger callback trampoline. Forwards every
// validation-layer message to compute::invoke_error_sink() (compute/
// backend.hpp), a no-op when nothing has called compute::set_error_sink()
// (every normal run) and, in the gpu test fixture, ADD_FAILURE() -- "a
// validation message during any GPU test is a test FAILURE" (this task's
// brief).
//
// VKAPI_ATTR/VKAPI_CALL, not a plain function: this is a genuine Vulkan
// callback the LOADER invokes with its own calling convention, exactly like
// every PFN_vk* signature in this file's own vkCreateDevice/vkCreateInstance
// calls -- omitting them is silently correct on this box's calling
// convention and a real ABI mismatch on others.
VKAPI_ATTR VkBool32 VKAPI_CALL debug_messenger_callback(VkDebugUtilsMessageSeverityFlagBitsEXT,
                                                          VkDebugUtilsMessageTypeFlagsEXT,
                                                          const VkDebugUtilsMessengerCallbackDataEXT* callback_data,
                                                          void*) {
    if (callback_data != nullptr && callback_data->pMessage != nullptr) {
        // Unqualified: this trampoline is itself inside namespace
        // spade::compute (nested in an anonymous namespace within it), so
        // invoke_error_sink (declared in compute/backend.hpp, included
        // transitively via context.hpp) resolves via ordinary enclosing-
        // namespace lookup -- `compute::invoke_error_sink` here would
        // (wrongly) look for spade::compute::compute::invoke_error_sink.
        invoke_error_sink(std::string_view(callback_data->pMessage));
    }
    return VK_FALSE;  // never suppress the call the layer would otherwise make
}

}  // namespace

bool vulkan_available() noexcept {
    // THE ENV OVERRIDE IS CHECKED FIRST, DELIBERATELY, and the ordering is now
    // load-bearing rather than incidental: everything below it touches a driver,
    // and SPADE_FORCE_NO_VULKAN=1 exists precisely so a host-only test can get a
    // "no" WITHOUT one. ComputeBackendAvailability.ForcedUnavailableReturnsUnavailable
    // pins the ordering by asserting which clause answered.
    if (env_forces_no_vulkan()) return false;

    if (volkInitialize() != VK_SUCCESS) return false;

    VkApplicationInfo app_info{};
    app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app_info.pApplicationName = "spade_compute_probe";
    app_info.apiVersion = VK_API_VERSION_1_3;

    VkInstanceCreateInfo instance_info{};
    instance_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instance_info.pApplicationInfo = &app_info;

    VkInstance instance = VK_NULL_HANDLE;
    if (vkCreateInstance(&instance_info, nullptr, &instance) != VK_SUCCESS) return false;
    volkLoadInstanceOnly(instance);

    // Ask for ONE device rather than counting and then re-enumerating: the
    // capability clause below needs an actual VkPhysicalDevice handle, and the
    // default device is the only one this predicate can speak for (see below).
    // VK_INCOMPLETE is a SUCCESS here -- it means "more exist than you asked
    // for", and the one written is exactly the one wanted.
    uint32_t device_count = 1;
    VkPhysicalDevice first_device = VK_NULL_HANDLE;
    const VkResult enumerated = vkEnumeratePhysicalDevices(instance, &device_count, &first_device);
    const bool have_device = (enumerated == VK_SUCCESS || enumerated == VK_INCOMPLETE) &&
                             device_count > 0 && first_device != VK_NULL_HANDLE;

    // THE CAPABILITY CLAUSE (S6 Task 4, review fix round 2; CORE-5). A device
    // that is present but below Vulkan 1.1 cannot run the SPIR-V 1.3 kernels
    // -- create() rejects it with Code::unavailable -- so this predicate must
    // say so too, or the two disagree and every
    // `if (!vulkan_available()) GTEST_SKIP()` guard in the suite becomes a hard
    // failure on exactly the hardware the skip exists for. error.hpp's own
    // Code::unavailable contract ("architecturally absent from this
    // process/environment ... callers that can degrade or skip switch on this")
    // is the rule being honoured here.
    //
    // WHICH DEVICE IT SPEAKS FOR, stated because a global predicate cannot
    // speak for all of them: the FIRST enumerated device, i.e. the one
    // BackendDesc::device_index's default of 0 selects. That is what every
    // caller of this function goes on to create. A caller that names a
    // different index is outside what this predicate can answer and gets
    // create()'s named error instead -- which is why that check stays in
    // create() as well, per-index, rather than being folded away as redundant.
    //
    // STILL ONE CAPABILITY, NOT TWO, AFTER S6 TASK 8's INT64 PROBE -- worth
    // saying because the probe was run expecting to add a second clause here.
    // It measured shaderInt64 == VK_FALSE on this box's Iris, and the
    // resolution was to remove the engine's dependence on 64-bit shader
    // integers rather than to refuse the device: see device_supports_int64()
    // above and engine/shaders/u64.slang. A device that lacks shaderInt64 can
    // run every spade kernel, so this predicate must not exclude it, and
    // create() must not refuse on it either -- one predicate, one truth.
    bool usable = false;
    if (have_device) {
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(first_device, &properties);
        // THE ONE CAPABILITY LEFT (CORE-5): Vulkan 1.1, because every kernel is
        // SPIR-V 1.3. create() checks the same clause per device.
        usable = properties.apiVersion >= VK_API_VERSION_1_1;
    }

    vkDestroyInstance(instance, nullptr);
    return usable;
}

Result<std::unique_ptr<VulkanContext>> VulkanContext::create(const BackendDesc& desc) {
    // vulkan_available() IS the unavailable-path source of truth -- the
    // SPADE_FORCE_NO_VULKAN override, the loader, the device, AND (CORE-5) the
    // one capability the kernels require, Vulkan 1.1. Checked first
    // so every failure mode below it only has to handle "a loader and a usable
    // default device are known to exist".
    //
    // THE TWO CHECKS ARE NOT REDUNDANT, and the distinction is the whole reason
    // the capability is tested twice: this one covers the DEFAULT device, which
    // is what the predicate can speak for and what every GTEST_SKIP guard in
    // the suite is asking about; the one further down covers the device
    // `desc.device_index` ACTUALLY names, which may be a different one. A
    // caller that bypasses the predicate, or names a non-default index, still
    // gets the loud, named rejection rather than an undefined-behaviour
    // pipeline.
    if (!vulkan_available()) {
        return std::unexpected(Error{
            Code::unavailable,
            "no Vulkan loader, no physical device, or the default device is below Vulkan 1.1"});
    }

    // vulkan_available() already ran volkInitialize() successfully as part
    // of its own probe; calling it again is a documented re-resolve of the
    // same global function pointers, not a second distinct initialization.
    if (volkInitialize() != VK_SUCCESS) {
        return std::unexpected(Error{Code::unavailable, "volkInitialize failed"});
    }

    VkApplicationInfo app_info{};
    app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app_info.pApplicationName = "spade_compute";
    app_info.apiVersion = VK_API_VERSION_1_3;

    std::vector<const char*> layers;
    std::vector<const char*> extensions;
    bool debug_utils_requested = false;
#ifndef NDEBUG
    // Validation layer, debug builds only, and only if actually present --
    // this task's brief: "VK_LAYER_KHRONOS_validation enabled in debug IF
    // present -- absence is not an error". A Release build skips
    // this whole block, not just the layer itself, so it never even calls
    // vkEnumerateInstanceLayerProperties.
    uint32_t layer_count = 0;
    vkEnumerateInstanceLayerProperties(&layer_count, nullptr);
    std::vector<VkLayerProperties> available_layers(layer_count);
    vkEnumerateInstanceLayerProperties(&layer_count, available_layers.data());
    for (const auto& layer : available_layers) {
        if (std::string_view(layer.layerName) == "VK_LAYER_KHRONOS_validation") {
            layers.push_back("VK_LAYER_KHRONOS_validation");
            break;
        }
    }

    // S6 Task 5: VK_EXT_debug_utils, requested alongside the validation
    // layer and under the identical "IF present" posture -- absence is not
    // an error, this context is simply created without a messenger and
    // validation-layer output (if any) goes wherever the layer's own
    // default sink sends it instead of through compute::invoke_error_sink().
    // Only checked/requested when the layer itself was found: a messenger
    // with no validation layer enabled has nothing to report.
    if (!layers.empty()) {
        uint32_t ext_count = 0;
        vkEnumerateInstanceExtensionProperties(nullptr, &ext_count, nullptr);
        std::vector<VkExtensionProperties> available_extensions(ext_count);
        vkEnumerateInstanceExtensionProperties(nullptr, &ext_count, available_extensions.data());
        for (const auto& ext : available_extensions) {
            if (std::string_view(ext.extensionName) == VK_EXT_DEBUG_UTILS_EXTENSION_NAME) {
                extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
                debug_utils_requested = true;
                break;
            }
        }
    }
#endif

    VkInstanceCreateInfo instance_info{};
    instance_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instance_info.pApplicationInfo = &app_info;
    instance_info.enabledLayerCount = static_cast<uint32_t>(layers.size());
    instance_info.ppEnabledLayerNames = layers.empty() ? nullptr : layers.data();
    instance_info.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    instance_info.ppEnabledExtensionNames = extensions.empty() ? nullptr : extensions.data();

    VkInstance instance = VK_NULL_HANDLE;
    if (vkCreateInstance(&instance_info, nullptr, &instance) != VK_SUCCESS) {
        return std::unexpected(Error{Code::internal, "vkCreateInstance failed"});
    }
    volkLoadInstanceOnly(instance);

    // S6 Task 5: the debug messenger itself, created only when the
    // extension above was actually enabled. vkCreateDebugUtilsMessengerEXT
    // is loaded by volkLoadInstanceOnly() above but is a documented null
    // function pointer if the driver/loader did not resolve it despite the
    // extension being requested -- guarded rather than assumed, so an
    // unusual driver degrades to "no messenger" instead of a null-call
    // crash.
    VkDebugUtilsMessengerEXT debug_messenger = VK_NULL_HANDLE;
    if (debug_utils_requested && vkCreateDebugUtilsMessengerEXT != nullptr) {
        VkDebugUtilsMessengerCreateInfoEXT messenger_info{};
        messenger_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        messenger_info.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                          VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        messenger_info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                                      VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                      VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        messenger_info.pfnUserCallback = debug_messenger_callback;
        // A failed messenger create is NOT fatal to context creation -- it
        // degrades to "no messenger, validation output uncaptured", the
        // same posture as the layer/extension being absent, rather than
        // turning an optional diagnostic aid into a hard dependency.
        vkCreateDebugUtilsMessengerEXT(instance, &messenger_info, nullptr, &debug_messenger);
    }

    uint32_t device_count = 0;
    vkEnumeratePhysicalDevices(instance, &device_count, nullptr);
    std::vector<VkPhysicalDevice> physical_devices(device_count);
    vkEnumeratePhysicalDevices(instance, &device_count, physical_devices.data());

    if (desc.device_index >= device_count) {
        if (debug_messenger != VK_NULL_HANDLE) vkDestroyDebugUtilsMessengerEXT(instance, debug_messenger, nullptr);
        vkDestroyInstance(instance, nullptr);
        return std::unexpected(
            Error{Code::invalid_argument, "device_index " + std::to_string(desc.device_index) +
                                               " out of range (" + std::to_string(device_count) +
                                               " device(s) enumerated)"});
    }

    VkPhysicalDevice physical_device = physical_devices[desc.device_index];

    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(physical_device, &properties);

    // THE ONE CAPABILITY LEFT (CORE-5): Vulkan 1.1, because every kernel is
    // SPIR-V 1.3. fp32 denormal preservation is no longer one: no kernel
    // requests a denormal mode (SPIR-V rule P3), so a device that cannot
    // preserve runs every kernel legally with its own default, and its results
    // are banded against the CPU (TD-14). Checked before any queue is picked or
    // any device is created. Code::unavailable, not invalid_argument: the
    // caller asked for nothing wrong -- the resource is architecturally absent
    // from this environment (core/error.hpp), which is what lets a caller that
    // can degrade or skip switch on it. "SPIR-V 1.3" appears in this per-device
    // refusal and never in the predicate's message above, so
    // ComputeBackendAvailability.ForcedUnavailableReturnsUnavailable can tell
    // which clause answered.
    if (properties.apiVersion < VK_API_VERSION_1_1) {
        // The messenger cleanup belongs on every early return between the
        // messenger's creation and VulkanContext taking ownership of it.
        if (debug_messenger != VK_NULL_HANDLE) vkDestroyDebugUtilsMessengerEXT(instance, debug_messenger, nullptr);
        vkDestroyInstance(instance, nullptr);
        return std::unexpected(Error{
            Code::unavailable,
            std::string("physical device '") + properties.deviceName + "' reports Vulkan " +
                std::to_string(VK_API_VERSION_MAJOR(properties.apiVersion)) + "." +
                std::to_string(VK_API_VERSION_MINOR(properties.apiVersion)) +
                ", and every spade kernel is SPIR-V 1.3, which needs Vulkan 1.1"});
    }
    DeviceReport report = read_device_report(physical_device, properties);

    // S6 Task 8's probe, RECORDED but NOT a requirement -- the one capability
    // question this engine asks and then works around rather than refusing on.
    // See device_supports_int64() above for the measurement (VK_FALSE here) and
    // for why the resolution was to stop declaring Int64 at all.
    const bool int64_supported = device_supports_int64(physical_device);

    uint32_t family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device, &family_count, nullptr);
    std::vector<VkQueueFamilyProperties> families(family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device, &family_count, families.data());

    const uint32_t compute_family = pick_compute_family(families);
    if (compute_family == std::numeric_limits<uint32_t>::max()) {
        if (debug_messenger != VK_NULL_HANDLE) vkDestroyDebugUtilsMessengerEXT(instance, debug_messenger, nullptr);
        vkDestroyInstance(instance, nullptr);
        return std::unexpected(
            Error{Code::internal, "physical device advertises no compute-capable queue family"});
    }
    const uint32_t transfer_family = pick_transfer_family(families, compute_family);

    const float queue_priority = 1.0f;
    std::vector<VkDeviceQueueCreateInfo> queue_infos;
    uint32_t compute_queue_index = 0;
    uint32_t transfer_queue_index = 0;

    if (compute_family == transfer_family) {
        // Same family (this program's Iris box): request a second queue
        // from it when the family actually has one to give, so the compute
        // and transfer queues are genuinely distinct VkQueue handles;
        // otherwise both accessors below return the SAME single queue --
        // "may be the same family... handle both" (this task's brief).
        const uint32_t available = families[compute_family].queueCount;
        const uint32_t requested = available >= 2 ? 2u : 1u;
        transfer_queue_index = requested >= 2 ? 1u : 0u;

        VkDeviceQueueCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        info.queueFamilyIndex = compute_family;
        info.queueCount = requested;
        info.pQueuePriorities = &queue_priority;
        queue_infos.push_back(info);
    } else {
        VkDeviceQueueCreateInfo compute_info{};
        compute_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        compute_info.queueFamilyIndex = compute_family;
        compute_info.queueCount = 1;
        compute_info.pQueuePriorities = &queue_priority;
        queue_infos.push_back(compute_info);

        VkDeviceQueueCreateInfo transfer_info{};
        transfer_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        transfer_info.queueFamilyIndex = transfer_family;
        transfer_info.queueCount = 1;
        transfer_info.pQueuePriorities = &queue_priority;
        queue_infos.push_back(transfer_info);
    }

    // NO pEnabledFeatures, STILL, AND NOW DELIBERATELY (S6 Task 8). An enabled-
    // features struct is a statement of what the compiled modules are allowed
    // to declare, and after this task they declare nothing optional at all:
    // spirv_scan.hpp's rule P4 holds every module to zero OpCapability Int64,
    // and nothing in the set needs Float64, Int16 or an atomic. Requesting a
    // feature no module uses would be noise the next reader has to disprove.
    VkDeviceCreateInfo device_info{};
    device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    device_info.queueCreateInfoCount = static_cast<uint32_t>(queue_infos.size());
    device_info.pQueueCreateInfos = queue_infos.data();

    VkDevice device = VK_NULL_HANDLE;
    if (vkCreateDevice(physical_device, &device_info, nullptr, &device) != VK_SUCCESS) {
        if (debug_messenger != VK_NULL_HANDLE) vkDestroyDebugUtilsMessengerEXT(instance, debug_messenger, nullptr);
        vkDestroyInstance(instance, nullptr);
        return std::unexpected(Error{Code::internal, "vkCreateDevice failed"});
    }
    volkLoadDevice(device);

    // std::unique_ptr<VulkanContext>(new VulkanContext()): VulkanContext's
    // default constructor is private (construction is only ever valid
    // through this factory), so make_unique -- which requires a public
    // constructor -- cannot be used here.
    auto ctx = std::unique_ptr<VulkanContext>(new VulkanContext());
    ctx->instance_ = instance;
    ctx->physical_device_ = physical_device;
    ctx->device_ = device;
    ctx->compute_family_ = compute_family;
    ctx->transfer_family_ = transfer_family;
    ctx->api_version_ = properties.apiVersion;
    ctx->device_name_ = properties.deviceName;
    ctx->debug_messenger_ = debug_messenger;
    ctx->int64_supported_ = int64_supported;
    ctx->report_ = std::move(report);
    vkGetDeviceQueue(device, compute_family, compute_queue_index, &ctx->compute_queue_);
    vkGetDeviceQueue(device, transfer_family, transfer_queue_index, &ctx->transfer_queue_);

    return ctx;
}

VulkanContext::VulkanContext(VulkanContext&& other) noexcept
    : instance_(std::exchange(other.instance_, VK_NULL_HANDLE)),
      physical_device_(std::exchange(other.physical_device_, VK_NULL_HANDLE)),
      device_(std::exchange(other.device_, VK_NULL_HANDLE)),
      compute_queue_(std::exchange(other.compute_queue_, VK_NULL_HANDLE)),
      transfer_queue_(std::exchange(other.transfer_queue_, VK_NULL_HANDLE)),
      debug_messenger_(std::exchange(other.debug_messenger_, VK_NULL_HANDLE)),
      compute_family_(other.compute_family_),
      transfer_family_(other.transfer_family_),
      api_version_(other.api_version_),
      int64_supported_(other.int64_supported_),
      report_(std::move(other.report_)),
      device_name_(std::move(other.device_name_)) {}

VulkanContext& VulkanContext::operator=(VulkanContext&& other) noexcept {
    if (this != &other) {
        destroy();
        instance_ = std::exchange(other.instance_, VK_NULL_HANDLE);
        physical_device_ = std::exchange(other.physical_device_, VK_NULL_HANDLE);
        device_ = std::exchange(other.device_, VK_NULL_HANDLE);
        compute_queue_ = std::exchange(other.compute_queue_, VK_NULL_HANDLE);
        transfer_queue_ = std::exchange(other.transfer_queue_, VK_NULL_HANDLE);
        debug_messenger_ = std::exchange(other.debug_messenger_, VK_NULL_HANDLE);
        compute_family_ = other.compute_family_;
        transfer_family_ = other.transfer_family_;
        api_version_ = other.api_version_;
        int64_supported_ = other.int64_supported_;
        report_ = std::move(other.report_);
        device_name_ = std::move(other.device_name_);
    }
    return *this;
}

VulkanContext::~VulkanContext() { destroy(); }

void VulkanContext::destroy() noexcept {
    if (device_ != VK_NULL_HANDLE) {
        vkDestroyDevice(device_, nullptr);
        device_ = VK_NULL_HANDLE;
    }
    // Destroyed before the instance -- a debug messenger is an INSTANCE-level
    // object (VK_EXT_debug_utils), and vkDestroyInstance implicitly
    // invalidates every such child; destroying it explicitly first keeps
    // this class's own teardown order authoritative rather than relying on
    // implicit-destruction behaviour the spec allows but does not require
    // every layer to implement identically.
    if (debug_messenger_ != VK_NULL_HANDLE) {
        vkDestroyDebugUtilsMessengerEXT(instance_, debug_messenger_, nullptr);
        debug_messenger_ = VK_NULL_HANDLE;
    }
    if (instance_ != VK_NULL_HANDLE) {
        vkDestroyInstance(instance_, nullptr);
        instance_ = VK_NULL_HANDLE;
    }
    physical_device_ = VK_NULL_HANDLE;
    compute_queue_ = VK_NULL_HANDLE;
    transfer_queue_ = VK_NULL_HANDLE;
}

}  // namespace spade::compute
