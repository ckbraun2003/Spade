#pragma once

// ---------------------------------------------------------------------------
// device_features.hpp (camera design section 12, item 2) -- THE OPTIONAL DEVICE FEATURES a pass beside the engine may ask for.
//
// Plain data and one pure rule, no Vulkan types, so the rule is a CPU test and a front end on the CPU backend can include it.
// The context probes which of these the device offers (DeviceReport::optional_supported, read-only, always reported) and
// enables only what BackendDesc::device_features asks for (DeviceReport::optional_enabled). An empty request is today's
// device exactly: no extra extension, no feature chain on vkCreateDevice. A request the device cannot meet is refused by name
// (resolve_features), never silently dropped: a pass that needs ray_query must hear that it is not there.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <initializer_list>
#include <string>
#include <string_view>

#include "core/error.hpp"

namespace spade::compute {

enum class DeviceFeature : uint8_t {
    timeline_semaphore = 0,      // VkSemaphore of type TIMELINE (core in Vulkan 1.2): the step's tick counter
    external_memory_win32,       // VK_KHR_external_memory_win32: device memory shared with another API or process
    external_semaphore_win32,    // VK_KHR_external_semaphore_win32: a semaphore shared the same way
    ray_query,                   // VK_KHR_ray_query: rayQuery in compute shaders
    acceleration_structure,      // VK_KHR_acceleration_structure (+ VK_KHR_deferred_host_operations)
    buffer_device_address,       // bufferDeviceAddress (core in Vulkan 1.2): the acceleration structure builds need it
    storage_8bit,                // storageBuffer8BitAccess + shaderInt8: uint8 storage buffers (the camera's output)
    storage_16bit,               // storageBuffer16BitAccess + shaderInt16: uint16 storage buffers
};

inline constexpr uint32_t kDeviceFeatureCount = 8;

// The name a refusal and a report use: the enumerator's own name.
[[nodiscard]] constexpr std::string_view feature_name(DeviceFeature f) noexcept {
    switch (f) {
        case DeviceFeature::timeline_semaphore: return "timeline_semaphore";
        case DeviceFeature::external_memory_win32: return "external_memory_win32";
        case DeviceFeature::external_semaphore_win32: return "external_semaphore_win32";
        case DeviceFeature::ray_query: return "ray_query";
        case DeviceFeature::acceleration_structure: return "acceleration_structure";
        case DeviceFeature::buffer_device_address: return "buffer_device_address";
        case DeviceFeature::storage_8bit: return "storage_8bit";
        case DeviceFeature::storage_16bit: return "storage_16bit";
    }
    return "unknown";
}

// What the device would have to offer, for a refusal to say.
[[nodiscard]] constexpr std::string_view feature_requires(DeviceFeature f) noexcept {
    switch (f) {
        case DeviceFeature::timeline_semaphore: return "VK_KHR_timeline_semaphore / timelineSemaphore (Vulkan 1.2)";
        case DeviceFeature::external_memory_win32: return "VK_KHR_external_memory_win32";
        case DeviceFeature::external_semaphore_win32:
            return "VK_KHR_external_semaphore_win32 with an exportable timeline semaphore";
        case DeviceFeature::ray_query: return "VK_KHR_ray_query / rayQuery";
        case DeviceFeature::acceleration_structure:
            return "VK_KHR_acceleration_structure + VK_KHR_deferred_host_operations / accelerationStructure";
        case DeviceFeature::buffer_device_address: return "VK_KHR_buffer_device_address / bufferDeviceAddress (Vulkan 1.2)";
        case DeviceFeature::storage_8bit:
            return "VK_KHR_8bit_storage + VK_KHR_shader_float16_int8 / storageBuffer8BitAccess and shaderInt8 (Vulkan 1.2)";
        case DeviceFeature::storage_16bit: return "storageBuffer16BitAccess (Vulkan 1.1) and shaderInt16";
    }
    return "unknown";
}

class FeatureSet {
public:
    constexpr FeatureSet() = default;
    constexpr FeatureSet(std::initializer_list<DeviceFeature> list) {
        for (const DeviceFeature f : list) add(f);
    }
    constexpr void add(DeviceFeature f) noexcept { bits_ |= bit(f); }
    [[nodiscard]] constexpr bool contains(DeviceFeature f) const noexcept { return (bits_ & bit(f)) != 0; }
    [[nodiscard]] constexpr bool empty() const noexcept { return bits_ == 0; }
    [[nodiscard]] constexpr uint32_t bits() const noexcept { return bits_; }
    [[nodiscard]] constexpr bool operator==(const FeatureSet&) const noexcept = default;

    // The members in enumerator order, comma separated ("" for the empty set), for a report line.
    [[nodiscard]] std::string describe() const {
        std::string out;
        for (uint32_t i = 0; i < kDeviceFeatureCount; ++i) {
            const auto f = static_cast<DeviceFeature>(i);
            if (!contains(f)) continue;
            if (!out.empty()) out += ", ";
            out += feature_name(f);
        }
        return out;
    }

private:
    [[nodiscard]] static constexpr uint32_t bit(DeviceFeature f) noexcept { return uint32_t{1} << static_cast<uint32_t>(f); }
    uint32_t bits_ = 0;
};

// What a feature drags in: an acceleration structure is built from device addresses, and a ray query traces one.
[[nodiscard]] constexpr FeatureSet feature_dependencies(DeviceFeature f) noexcept {
    switch (f) {
        case DeviceFeature::acceleration_structure: return FeatureSet{DeviceFeature::buffer_device_address};
        case DeviceFeature::ray_query: return FeatureSet{DeviceFeature::acceleration_structure};
        default: return FeatureSet{};
    }
}

// `requested` and, transitively, everything it depends on.
[[nodiscard]] constexpr FeatureSet with_dependencies(FeatureSet requested) noexcept {
    FeatureSet out = requested;
    for (int round = 0; round < 3; ++round) {
        for (uint32_t i = 0; i < kDeviceFeatureCount; ++i) {
            const auto f = static_cast<DeviceFeature>(i);
            if (!out.contains(f)) continue;
            const FeatureSet needs = feature_dependencies(f);
            for (uint32_t j = 0; j < kDeviceFeatureCount; ++j) {
                if (needs.contains(static_cast<DeviceFeature>(j))) out.add(static_cast<DeviceFeature>(j));
            }
        }
    }
    return out;
}

// THE RULE (pure, no device): what a context must enable for `requested`, or the refusal. Code::unavailable naming the first
// feature (in enumerator order) of the request's closure that `supported` lacks, what it would take, whether it was asked for
// or is a dependency of one that was, and the device. Nothing is ever dropped silently.
[[nodiscard]] inline Result<FeatureSet> resolve_features(FeatureSet requested, FeatureSet supported,
                                                         std::string_view device_name) {
    const FeatureSet closure = with_dependencies(requested);
    for (uint32_t i = 0; i < kDeviceFeatureCount; ++i) {
        const auto f = static_cast<DeviceFeature>(i);
        if (!closure.contains(f) || supported.contains(f)) continue;
        std::string why = "device feature '" + std::string(feature_name(f)) + "' ";
        if (requested.contains(f)) {
            why += "(asked for) ";
        } else {
            // A dependency: name a member of the closure that needs it directly.
            for (uint32_t j = 0; j < kDeviceFeatureCount; ++j) {
                const auto g = static_cast<DeviceFeature>(j);
                if (closure.contains(g) && feature_dependencies(g).contains(f)) {
                    why += "(needed by '" + std::string(feature_name(g)) + "') ";
                    break;
                }
            }
        }
        why += "is not offered by device '" + std::string(device_name) + "': it needs " + std::string(feature_requires(f));
        return std::unexpected(Error{Code::unavailable, std::move(why)});
    }
    return closure;
}

}  // namespace spade::compute
