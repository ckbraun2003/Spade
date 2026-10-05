// gpu_skip.hpp -- why a device-gated test skips (TD-6).
//
// Every Gpu* test that needs a Vulkan device begins with
//     if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
// so a log says WHY it skipped, not only that it did: the override, no loader
// or device, or the default device's missing capability (the engine's one
// requirement, fp32 denormal preservation).
#pragma once

#include <cstdlib>
#include <optional>
#include <string>
#include <string_view>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"

namespace spade::testing {

// Whether SPADE_FORCE_NO_VULKAN=1 is set for this process. _dupenv_s on MSVC,
// where /W4 /WX rejects std::getenv (C4996).
[[nodiscard]] inline bool forced_no_vulkan() {
#ifdef _MSC_VER
    char* value = nullptr;
    std::size_t length = 0;
    const bool forced = _dupenv_s(&value, &length, "SPADE_FORCE_NO_VULKAN") == 0 && value != nullptr &&
                        std::string_view(value) == "1";
    std::free(value);
    return forced;
#else
    const char* value = std::getenv("SPADE_FORCE_NO_VULKAN");
    return value != nullptr && std::string_view(value) == "1";
#endif
}

// The default device's fp32-denormal capability, read straight from the
// driver: NOT through vulkan_available(), whose answer is what the refusal
// test below checks.
struct DefaultDeviceDenorms {
    bool device = false;     // a loader and a default device answered
    bool queryable = false;  // the device reports Vulkan >= 1.2, so float controls can be queried
    VkBool32 preserve_f32 = VK_FALSE;
    std::string name;
};

[[nodiscard]] inline DefaultDeviceDenorms read_default_device_denorms() {
    DefaultDeviceDenorms out;
    if (volkInitialize() != VK_SUCCESS) return out;

    VkApplicationInfo app_info{};
    app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app_info.pApplicationName = "spade_test_denorm_read";
    app_info.apiVersion = VK_API_VERSION_1_3;
    VkInstanceCreateInfo instance_info{};
    instance_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instance_info.pApplicationInfo = &app_info;
    VkInstance instance = VK_NULL_HANDLE;
    if (vkCreateInstance(&instance_info, nullptr, &instance) != VK_SUCCESS) return out;
    volkLoadInstanceOnly(instance);

    uint32_t count = 1;
    VkPhysicalDevice device = VK_NULL_HANDLE;
    const VkResult enumerated = vkEnumeratePhysicalDevices(instance, &count, &device);
    if ((enumerated == VK_SUCCESS || enumerated == VK_INCOMPLETE) && count > 0 && device != VK_NULL_HANDLE) {
        out.device = true;
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(device, &properties);
        out.name = properties.deviceName;
        if (properties.apiVersion >= VK_API_VERSION_1_2) {
            out.queryable = true;
            VkPhysicalDeviceFloatControlsProperties float_controls{};
            float_controls.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FLOAT_CONTROLS_PROPERTIES;
            VkPhysicalDeviceProperties2 properties2{};
            properties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
            properties2.pNext = &float_controls;
            vkGetPhysicalDeviceProperties2(device, &properties2);
            out.preserve_f32 = float_controls.shaderDenormPreserveFloat32;
        }
    }
    vkDestroyInstance(instance, nullptr);
    return out;
}

// Empty when the engine admits a device; otherwise why it does not. The
// device part is read once per process (the device does not change during a
// run); the override is checked on every call, because a test can set it.
[[nodiscard]] inline std::optional<std::string> vulkan_skip_reason() {
    if (forced_no_vulkan()) return "Vulkan unavailable: SPADE_FORCE_NO_VULKAN=1";
    if (compute::vulkan_available()) return std::nullopt;
    static const std::string reason = [] {
        const DefaultDeviceDenorms denorms = read_default_device_denorms();
        if (!denorms.device) return std::string("Vulkan unavailable: no Vulkan loader or physical device");
        if (!denorms.queryable) {
            return "Vulkan unavailable: the default device ('" + denorms.name +
                   "') reports Vulkan < 1.2, so shaderDenormPreserveFloat32 cannot be queried";
        }
        if (denorms.preserve_f32 == VK_FALSE) {
            return "Vulkan unavailable: the default device ('" + denorms.name +
                   "') reports shaderDenormPreserveFloat32 = false, and every spade kernel requires fp32 "
                   "denormal preservation";
        }
        // The engine refused for a reason the direct read does not see: give its own words.
        auto context = compute::VulkanContext::create(compute::BackendDesc{.kind = compute::BackendKind::vulkan});
        return std::string("Vulkan unavailable: ") +
               (context ? std::string("vulkan_available() is false, yet VulkanContext::create admits the device")
                        : context.error().context);
    }();
    return reason;
}

}  // namespace spade::testing
