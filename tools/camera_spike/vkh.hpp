#pragma once
// vkh.hpp -- the camera spike's own small Vulkan setup (instance, device with ray query, buffers, one-shot submits). It is deliberately
// not engine/compute/vulkan/context.*: that context enables no ray-tracing, buffer-address or external-memory extension, and the spike
// must not change its behaviour for current users. Bench code only.

// Vulkan structures are initialised here as `VkX x{VK_STRUCTURE_TYPE_X};` and then filled field by field, the way the Vulkan documentation
// writes it; gcc's -Wextra calls every such line a missing-initializer error. Bench code, so the warning is off for this file's users.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif

#include <volk.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace cs {

[[noreturn]] inline void die(const std::string& message) {
    std::fprintf(stderr, "camera_spike: %s\n", message.c_str());
    std::fflush(stderr);
    std::exit(2);
}

#define CS_VK(call)                                                                                  \
    do {                                                                                             \
        const VkResult cs_r_ = (call);                                                               \
        if (cs_r_ != VK_SUCCESS) cs::die(std::string(#call) + " failed with VkResult " + std::to_string(static_cast<int>(cs_r_))); \
    } while (false)

struct Buffer {
    VkBuffer buf = VK_NULL_HANDLE;
    VkDeviceMemory mem = VK_NULL_HANDLE;
    VkDeviceSize size = 0;
    VkDeviceAddress addr = 0;
    void* map = nullptr;
};

struct Device {
    VkInstance inst = VK_NULL_HANDLE;
    VkPhysicalDevice phys = VK_NULL_HANDLE;
    VkDevice dev = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    uint32_t family = 0;
    VkCommandPool pool = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties mem{};
    float ts_period_ns = 1.0f;
    VkDeviceSize scratch_align = 256;
    bool has_budget = false;
    bool has_external = false;  // win32 external memory and semaphore
    std::string name;
    uint32_t driver = 0;
    uint32_t device_local_heap = 0;
    uint8_t device_uuid[VK_UUID_SIZE]{};
    uint8_t device_luid[VK_LUID_SIZE]{};
    bool luid_valid = false;
};

[[nodiscard]] inline bool has_ext(const std::vector<VkExtensionProperties>& list, const char* name) {
    for (const auto& e : list)
        if (std::strcmp(e.extensionName, name) == 0) return true;
    return false;
}

// Create the device. On a device or feature the spike needs and does not find, returns false with the reason (the spike then refuses,
// it does not fall back).
[[nodiscard]] inline bool create_device(Device& d, bool want_external, std::string& why) {
    if (volkInitialize() != VK_SUCCESS) {
        why = "no Vulkan loader";
        return false;
    }
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "camera_spike";
    app.apiVersion = VK_API_VERSION_1_3;
    VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    ici.pApplicationInfo = &app;
    if (vkCreateInstance(&ici, nullptr, &d.inst) != VK_SUCCESS) {
        why = "vkCreateInstance failed";
        return false;
    }
    volkLoadInstance(d.inst);
    uint32_t n = 0;
    vkEnumeratePhysicalDevices(d.inst, &n, nullptr);
    std::vector<VkPhysicalDevice> devs(n);
    vkEnumeratePhysicalDevices(d.inst, &n, devs.data());
    for (VkPhysicalDevice p : devs) {
        VkPhysicalDeviceProperties pr{};
        vkGetPhysicalDeviceProperties(p, &pr);
        if (pr.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            d.phys = p;
            break;
        }
    }
    if (d.phys == VK_NULL_HANDLE && n > 0) d.phys = devs[0];
    if (d.phys == VK_NULL_HANDLE) {
        why = "no physical device";
        return false;
    }
    uint32_t ne = 0;
    vkEnumerateDeviceExtensionProperties(d.phys, nullptr, &ne, nullptr);
    std::vector<VkExtensionProperties> exts(ne);
    vkEnumerateDeviceExtensionProperties(d.phys, nullptr, &ne, exts.data());
    std::vector<const char*> need = {VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME, VK_KHR_RAY_QUERY_EXTENSION_NAME,
                                     VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME};
    for (const char* e : need)
        if (!has_ext(exts, e)) {
            why = std::string("the device lacks ") + e;
            return false;
        }
    d.has_budget = has_ext(exts, VK_EXT_MEMORY_BUDGET_EXTENSION_NAME);
    d.has_external = false;
#ifdef _WIN32
    d.has_external = has_ext(exts, VK_KHR_EXTERNAL_MEMORY_WIN32_EXTENSION_NAME) && has_ext(exts, VK_KHR_EXTERNAL_SEMAPHORE_WIN32_EXTENSION_NAME);
#endif
    std::vector<const char*> enable = need;
    if (d.has_budget) enable.push_back(VK_EXT_MEMORY_BUDGET_EXTENSION_NAME);
    if (want_external && d.has_external) {
        enable.push_back(VK_KHR_EXTERNAL_MEMORY_EXTENSION_NAME);
        enable.push_back(VK_KHR_EXTERNAL_SEMAPHORE_EXTENSION_NAME);
#ifdef _WIN32
        enable.push_back(VK_KHR_EXTERNAL_MEMORY_WIN32_EXTENSION_NAME);
        enable.push_back(VK_KHR_EXTERNAL_SEMAPHORE_WIN32_EXTENSION_NAME);
#endif
    }

    VkPhysicalDeviceAccelerationStructureFeaturesKHR asf{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR};
    VkPhysicalDeviceRayQueryFeaturesKHR rqf{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR};
    VkPhysicalDeviceVulkan12Features f12{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES};
    VkPhysicalDeviceVulkan11Features f11{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES};
    VkPhysicalDeviceFeatures2 f2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
    f2.pNext = &f11;
    f11.pNext = &f12;
    f12.pNext = &asf;
    asf.pNext = &rqf;
    vkGetPhysicalDeviceFeatures2(d.phys, &f2);
    if (!asf.accelerationStructure) { why = "accelerationStructure feature is off"; return false; }
    if (!rqf.rayQuery) { why = "rayQuery feature is off"; return false; }
    if (!f12.bufferDeviceAddress) { why = "bufferDeviceAddress feature is off"; return false; }
    if (!f12.timelineSemaphore) { why = "timelineSemaphore feature is off"; return false; }
    if (!f12.storageBuffer8BitAccess || !f12.shaderInt8) { why = "8-bit storage or shaderInt8 is off"; return false; }
    if (!f11.storageBuffer16BitAccess || !f2.features.shaderInt16) { why = "16-bit storage or shaderInt16 is off"; return false; }
    // Enable exactly what the kernel needs, nothing else.
    VkPhysicalDeviceAccelerationStructureFeaturesKHR asOn{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR};
    asOn.accelerationStructure = VK_TRUE;
    VkPhysicalDeviceRayQueryFeaturesKHR rqOn{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR};
    rqOn.rayQuery = VK_TRUE;
    rqOn.pNext = &asOn;
    VkPhysicalDeviceVulkan12Features on12{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES};
    on12.bufferDeviceAddress = VK_TRUE;
    on12.timelineSemaphore = VK_TRUE;
    on12.storageBuffer8BitAccess = VK_TRUE;
    on12.shaderInt8 = VK_TRUE;
    on12.pNext = &rqOn;
    VkPhysicalDeviceVulkan11Features on11{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES};
    on11.storageBuffer16BitAccess = VK_TRUE;
    on11.pNext = &on12;
    VkPhysicalDeviceFeatures2 on2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
    on2.features.shaderInt16 = VK_TRUE;
    on2.pNext = &on11;

    uint32_t nq = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(d.phys, &nq, nullptr);
    std::vector<VkQueueFamilyProperties> qf(nq);
    vkGetPhysicalDeviceQueueFamilyProperties(d.phys, &nq, qf.data());
    bool found = false;
    uint32_t tsbits = 0;
    for (uint32_t i = 0; i < nq; ++i)
        if ((qf[i].queueFlags & VK_QUEUE_COMPUTE_BIT) && (qf[i].queueFlags & VK_QUEUE_TRANSFER_BIT || qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)) {
            d.family = i;
            tsbits = qf[i].timestampValidBits;
            found = true;
            break;
        }
    if (!found) { why = "no compute queue"; return false; }
    if (tsbits == 0) { why = "the queue has no timestamps"; return false; }
    const float prio = 1.0f;
    VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    qci.queueFamilyIndex = d.family;
    qci.queueCount = 1;
    qci.pQueuePriorities = &prio;
    VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    dci.pNext = &on2;
    dci.queueCreateInfoCount = 1;
    dci.pQueueCreateInfos = &qci;
    dci.enabledExtensionCount = static_cast<uint32_t>(enable.size());
    dci.ppEnabledExtensionNames = enable.data();
    if (vkCreateDevice(d.phys, &dci, nullptr, &d.dev) != VK_SUCCESS) { why = "vkCreateDevice failed"; return false; }
    volkLoadDevice(d.dev);
    vkGetDeviceQueue(d.dev, d.family, 0, &d.queue);

    VkPhysicalDeviceAccelerationStructurePropertiesKHR asp{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR};
    VkPhysicalDeviceIDProperties idp{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES};
    idp.pNext = &asp;
    VkPhysicalDeviceProperties2 p2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};
    p2.pNext = &idp;
    vkGetPhysicalDeviceProperties2(d.phys, &p2);
    d.ts_period_ns = p2.properties.limits.timestampPeriod;
    d.scratch_align = std::max<VkDeviceSize>(asp.minAccelerationStructureScratchOffsetAlignment, 128);
    d.name = p2.properties.deviceName;
    d.driver = p2.properties.driverVersion;
    std::memcpy(d.device_uuid, idp.deviceUUID, VK_UUID_SIZE);
    std::memcpy(d.device_luid, idp.deviceLUID, VK_LUID_SIZE);
    d.luid_valid = idp.deviceLUIDValid == VK_TRUE;
    vkGetPhysicalDeviceMemoryProperties(d.phys, &d.mem);
    VkDeviceSize best = 0;
    for (uint32_t h = 0; h < d.mem.memoryHeapCount; ++h)
        if ((d.mem.memoryHeaps[h].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) && d.mem.memoryHeaps[h].size > best) {
            best = d.mem.memoryHeaps[h].size;
            d.device_local_heap = h;
        }
    VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    cpi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    cpi.queueFamilyIndex = d.family;
    CS_VK(vkCreateCommandPool(d.dev, &cpi, nullptr, &d.pool));
    return true;
}

// The device-local heap's usage by this process in bytes, from VK_EXT_memory_budget (0 when the extension is absent).
[[nodiscard]] inline uint64_t heap_usage(const Device& d) {
    if (!d.has_budget) return 0;
    VkPhysicalDeviceMemoryBudgetPropertiesEXT b{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_BUDGET_PROPERTIES_EXT};
    VkPhysicalDeviceMemoryProperties2 m{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_PROPERTIES_2};
    m.pNext = &b;
    vkGetPhysicalDeviceMemoryProperties2(d.phys, &m);
    return b.heapUsage[d.device_local_heap];
}

[[nodiscard]] inline uint32_t find_memory(const Device& d, uint32_t type_bits, VkMemoryPropertyFlags want) {
    for (uint32_t i = 0; i < d.mem.memoryTypeCount; ++i)
        if ((type_bits & (1u << i)) && (d.mem.memoryTypes[i].propertyFlags & want) == want) return i;
    die("no memory type for flags " + std::to_string(want));
}

// flags: DEVICE_LOCAL or HOST_VISIBLE|HOST_COHERENT. A buffer is always created with the device-address usage, so any can be an AS input.
[[nodiscard]] inline Buffer make_buffer(const Device& d, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags,
                                        bool exportable = false) {
    Buffer b;
    b.size = std::max<VkDeviceSize>(size, 16);
    VkExternalMemoryBufferCreateInfo ext{VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO};
    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size = b.size;
    bi.usage = usage | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
#ifdef _WIN32
    if (exportable) {
        ext.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
        bi.pNext = &ext;
    }
#else
    (void)exportable;
    (void)ext;
#endif
    CS_VK(vkCreateBuffer(d.dev, &bi, nullptr, &b.buf));
    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(d.dev, b.buf, &req);
    VkMemoryAllocateFlagsInfo fi{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO};
    fi.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;
    VkExportMemoryAllocateInfo ex{VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO};
    fi.pNext = nullptr;
#ifdef _WIN32
    if (exportable) {
        ex.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
        fi.pNext = &ex;
    }
#else
    (void)ex;
#endif
    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.pNext = &fi;
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = find_memory(d, req.memoryTypeBits, flags);
    CS_VK(vkAllocateMemory(d.dev, &ai, nullptr, &b.mem));
    CS_VK(vkBindBufferMemory(d.dev, b.buf, b.mem, 0));
    VkBufferDeviceAddressInfo da{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
    da.buffer = b.buf;
    b.addr = vkGetBufferDeviceAddress(d.dev, &da);
    if (flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) CS_VK(vkMapMemory(d.dev, b.mem, 0, VK_WHOLE_SIZE, 0, &b.map));
    return b;
}

inline void destroy_buffer(const Device& d, Buffer& b) {
    if (b.map != nullptr) vkUnmapMemory(d.dev, b.mem);
    if (b.buf != VK_NULL_HANDLE) vkDestroyBuffer(d.dev, b.buf, nullptr);
    if (b.mem != VK_NULL_HANDLE) vkFreeMemory(d.dev, b.mem, nullptr);
    b = Buffer{};
}

[[nodiscard]] inline VkCommandBuffer alloc_cmd(const Device& d) {
    VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    ai.commandPool = d.pool;
    ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = 1;
    VkCommandBuffer cb = VK_NULL_HANDLE;
    CS_VK(vkAllocateCommandBuffers(d.dev, &ai, &cb));
    return cb;
}

inline void begin_cmd(VkCommandBuffer cb) {
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    CS_VK(vkBeginCommandBuffer(cb, &bi));
}

inline void submit_wait(const Device& d, VkCommandBuffer cb) {
    CS_VK(vkEndCommandBuffer(cb));
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cb;
    CS_VK(vkQueueSubmit(d.queue, 1, &si, VK_NULL_HANDLE));
    CS_VK(vkQueueWaitIdle(d.queue));
}

inline void memory_barrier(VkCommandBuffer cb, VkPipelineStageFlags src, VkAccessFlags srcA, VkPipelineStageFlags dst, VkAccessFlags dstA) {
    VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    mb.srcAccessMask = srcA;
    mb.dstAccessMask = dstA;
    vkCmdPipelineBarrier(cb, src, dst, 0, 1, &mb, 0, nullptr, 0, nullptr);
}

}  // namespace cs
