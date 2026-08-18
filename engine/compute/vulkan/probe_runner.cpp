#include "compute/vulkan/probe_runner.hpp"

#include <cstring>
#include <string>
#include <vector>

namespace spade::compute {

namespace {

// The push-constant block, mirroring ProbeParams in every probe kernel
// (engine/shaders/kernels/fp32_math_probe.slang). Two uints, offset 0.
struct ProbePushConstants {
    uint32_t arg_count;
    uint32_t out_count;
};
static_assert(sizeof(ProbePushConstants) == 8,
              "the probe push-constant block is two uints; a kernel's ProbeParams must match");

// How long a probe dispatch may take before the fence wait is called a
// failure. Ten seconds is three orders of magnitude past the largest chunk
// this program dispatches (2^21 elements finish in single-digit milliseconds
// on the Iris) and still short enough that a hung device fails a test rather
// than hanging CI. VK_WHOLE_SIZE-style "wait forever" (UINT64_MAX) is
// deliberately not used: a test that hangs is strictly worse than one that
// fails with a named reason.
constexpr uint64_t kFenceTimeoutNs = 10ull * 1000ull * 1000ull * 1000ull;

// Every Vulkan object one run_probe() call creates, destroyed in reverse
// creation order by the destructor. A struct with a destructor rather than a
// chain of `goto cleanup` labels or a dozen nested unique_ptr deleters: every
// early return below (there are eleven) must free exactly what has been
// created so far and nothing else, and VK_NULL_HANDLE is a documented no-op
// for every vkDestroy*/vkFree* call used here, so "destroy them all" is
// correct at every point in the sequence.
struct ProbeResources {
    VkDevice device = VK_NULL_HANDLE;

    VkBuffer arg_buffer = VK_NULL_HANDLE;
    VkDeviceMemory arg_memory = VK_NULL_HANDLE;
    VkBuffer out_buffer = VK_NULL_HANDLE;
    VkDeviceMemory out_memory = VK_NULL_HANDLE;

    VkShaderModule shader = VK_NULL_HANDLE;
    VkDescriptorSetLayout set_layout = VK_NULL_HANDLE;
    VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;
    VkCommandPool command_pool = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;

    ProbeResources() = default;
    ProbeResources(const ProbeResources&) = delete;
    ProbeResources& operator=(const ProbeResources&) = delete;
    ProbeResources(ProbeResources&&) = delete;
    ProbeResources& operator=(ProbeResources&&) = delete;

    ~ProbeResources() {
        if (device == VK_NULL_HANDLE) return;
        // The command buffer and the descriptor set are freed with their
        // pools, so neither is tracked separately.
        vkDestroyFence(device, fence, nullptr);
        vkDestroyCommandPool(device, command_pool, nullptr);
        vkDestroyDescriptorPool(device, descriptor_pool, nullptr);
        vkDestroyPipeline(device, pipeline, nullptr);
        vkDestroyPipelineLayout(device, pipeline_layout, nullptr);
        vkDestroyDescriptorSetLayout(device, set_layout, nullptr);
        vkDestroyShaderModule(device, shader, nullptr);
        vkDestroyBuffer(device, out_buffer, nullptr);
        vkFreeMemory(device, out_memory, nullptr);
        vkDestroyBuffer(device, arg_buffer, nullptr);
        vkFreeMemory(device, arg_memory, nullptr);
    }
};

[[nodiscard]] std::unexpected<Error> fail(const char* what, VkResult result) {
    return std::unexpected(
        Error{Code::internal, std::string(what) + " failed (VkResult " + std::to_string(static_cast<int>(result)) + ")"});
}

// A HOST_VISIBLE + HOST_COHERENT memory type, preferring one that is also
// DEVICE_LOCAL. On an integrated device every candidate is device-local (host
// and GPU share the memory), so the preference is free there and only matters
// on a discrete GPU with a resizable-BAR heap. Returns the type index, or
// UINT32_MAX if the device advertises no host-coherent type at all -- which
// cannot happen on a conformant implementation (the spec requires at least one
// HOST_VISIBLE|HOST_COHERENT type) but is reported rather than assumed.
[[nodiscard]] uint32_t pick_host_memory_type(const VkPhysicalDeviceMemoryProperties& props,
                                             uint32_t type_bits) {
    constexpr VkMemoryPropertyFlags kRequired =
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

    uint32_t fallback = UINT32_MAX;
    for (uint32_t i = 0; i < props.memoryTypeCount; ++i) {
        if ((type_bits & (1u << i)) == 0u) continue;
        const VkMemoryPropertyFlags flags = props.memoryTypes[i].propertyFlags;
        if ((flags & kRequired) != kRequired) continue;
        if (flags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) return i;
        if (fallback == UINT32_MAX) fallback = i;
    }
    return fallback;
}

// Creates one storage buffer plus its backing host-visible allocation and
// binds them. On failure the partially-created handles are already recorded in
// `res`, so the caller's early return still frees them.
[[nodiscard]] Result<void> make_host_buffer(VkPhysicalDevice physical, VkDevice device,
                                            VkDeviceSize size, VkBuffer& buffer,
                                            VkDeviceMemory& memory) {
    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = size;
    buffer_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VkResult vr = vkCreateBuffer(device, &buffer_info, nullptr, &buffer);
    if (vr != VK_SUCCESS) return fail("vkCreateBuffer", vr);

    VkMemoryRequirements requirements{};
    vkGetBufferMemoryRequirements(device, buffer, &requirements);

    VkPhysicalDeviceMemoryProperties memory_properties{};
    vkGetPhysicalDeviceMemoryProperties(physical, &memory_properties);
    const uint32_t type_index = pick_host_memory_type(memory_properties, requirements.memoryTypeBits);
    if (type_index == UINT32_MAX) {
        return std::unexpected(Error{
            Code::internal,
            "no HOST_VISIBLE|HOST_COHERENT memory type accepts this storage buffer"});
    }

    VkMemoryAllocateInfo allocate_info{};
    allocate_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocate_info.allocationSize = requirements.size;
    allocate_info.memoryTypeIndex = type_index;

    vr = vkAllocateMemory(device, &allocate_info, nullptr, &memory);
    if (vr != VK_SUCCESS) return fail("vkAllocateMemory", vr);

    vr = vkBindBufferMemory(device, buffer, memory, 0);
    if (vr != VK_SUCCESS) return fail("vkBindBufferMemory", vr);

    return {};
}

}  // namespace

Result<void> run_probe(VulkanContext& ctx, std::span<const std::byte> spirv,
                       std::span<const float> args, std::span<float> out) {
    if (out.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "run_probe: empty result span"});
    }
    if (args.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "run_probe: empty argument span"});
    }
    if (spirv.size() % sizeof(uint32_t) != 0 || spirv.size() < 5 * sizeof(uint32_t)) {
        return std::unexpected(Error{
            Code::invalid_argument,
            "run_probe: SPIR-V blob is not a whole number of words, or is shorter than a header"});
    }

    // The blob is copied into a uint32_t vector rather than reinterpret_cast
    // in place: vkCreateShaderModule requires 4-byte-aligned code, and a
    // std::span<const std::byte> carries no alignment guarantee even when the
    // caller happened to build it from a uint32_t array.
    std::vector<uint32_t> code(spirv.size() / sizeof(uint32_t));
    std::memcpy(code.data(), spirv.data(), spirv.size());

    const uint32_t out_count = static_cast<uint32_t>(out.size());
    const uint32_t arg_count = static_cast<uint32_t>(args.size());
    const uint32_t group_count = (out_count + kProbeWorkgroupSize - 1) / kProbeWorkgroupSize;

    VkPhysicalDeviceProperties device_properties{};
    vkGetPhysicalDeviceProperties(ctx.physical_device(), &device_properties);
    if (group_count > device_properties.limits.maxComputeWorkGroupCount[0]) {
        return std::unexpected(Error{
            Code::invalid_argument,
            "run_probe: " + std::to_string(group_count) + " workgroups exceeds the device limit of " +
                std::to_string(device_properties.limits.maxComputeWorkGroupCount[0]) +
                " -- chunk the dispatch"});
    }

    ProbeResources res;
    res.device = ctx.device();

    // -- buffers ------------------------------------------------------------
    if (auto r = make_host_buffer(ctx.physical_device(), res.device,
                                  static_cast<VkDeviceSize>(args.size_bytes()), res.arg_buffer,
                                  res.arg_memory);
        !r) {
        return r;
    }
    if (auto r = make_host_buffer(ctx.physical_device(), res.device,
                                  static_cast<VkDeviceSize>(out.size_bytes()), res.out_buffer,
                                  res.out_memory);
        !r) {
        return r;
    }

    // -- upload -------------------------------------------------------------
    {
        void* mapped = nullptr;
        const VkResult vr = vkMapMemory(res.device, res.arg_memory, 0, VK_WHOLE_SIZE, 0, &mapped);
        if (vr != VK_SUCCESS) return fail("vkMapMemory(arguments)", vr);
        std::memcpy(mapped, args.data(), args.size_bytes());
        vkUnmapMemory(res.device, res.arg_memory);
    }

    // -- pipeline -----------------------------------------------------------
    VkShaderModuleCreateInfo module_info{};
    module_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    module_info.codeSize = spirv.size();
    module_info.pCode = code.data();
    if (const VkResult vr = vkCreateShaderModule(res.device, &module_info, nullptr, &res.shader);
        vr != VK_SUCCESS) {
        return fail("vkCreateShaderModule", vr);
    }

    VkDescriptorSetLayoutBinding bindings[2]{};
    bindings[0].binding = kProbeArgBinding;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bindings[1].binding = kProbeOutBinding;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutCreateInfo set_layout_info{};
    set_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    set_layout_info.bindingCount = 2;
    set_layout_info.pBindings = bindings;
    if (const VkResult vr =
            vkCreateDescriptorSetLayout(res.device, &set_layout_info, nullptr, &res.set_layout);
        vr != VK_SUCCESS) {
        return fail("vkCreateDescriptorSetLayout", vr);
    }

    VkPushConstantRange push_range{};
    push_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    push_range.offset = 0;
    push_range.size = sizeof(ProbePushConstants);

    VkPipelineLayoutCreateInfo pipeline_layout_info{};
    pipeline_layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipeline_layout_info.setLayoutCount = 1;
    pipeline_layout_info.pSetLayouts = &res.set_layout;
    pipeline_layout_info.pushConstantRangeCount = 1;
    pipeline_layout_info.pPushConstantRanges = &push_range;
    if (const VkResult vr = vkCreatePipelineLayout(res.device, &pipeline_layout_info, nullptr,
                                                   &res.pipeline_layout);
        vr != VK_SUCCESS) {
        return fail("vkCreatePipelineLayout", vr);
    }

    // pName "main". MEASURED, not assumed: slangc v2026.14.1's -target spirv
    // emits `OpEntryPoint GLCompute %2 "main"` and gives the Slang function
    // name only to an OpName debug record (`OpName %2 "fp32_math_probe"`), so
    // the string Vulkan matches on is "main" for every kernel this build
    // produces. Pinning it here rather than threading the kernel's own name
    // through run_probe()'s signature is what keeps the runner
    // kernel-agnostic; GpuFp32Math.ProbeKernelDeclaresTheRunnerBindings reads
    // the OpEntryPoint name out of the compiled module and asserts it, so a
    // slangc upgrade that changed the convention fails a test rather than
    // failing pipeline creation with VK_ERROR_UNKNOWN.
    VkPipelineShaderStageCreateInfo stage_info{};
    stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage_info.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stage_info.module = res.shader;
    stage_info.pName = "main";

    VkComputePipelineCreateInfo pipeline_info{};
    pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipeline_info.stage = stage_info;
    pipeline_info.layout = res.pipeline_layout;
    if (const VkResult vr = vkCreateComputePipelines(res.device, VK_NULL_HANDLE, 1, &pipeline_info,
                                                     nullptr, &res.pipeline);
        vr != VK_SUCCESS) {
        return fail("vkCreateComputePipelines", vr);
    }

    // -- descriptors --------------------------------------------------------
    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    pool_size.descriptorCount = 2;

    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.maxSets = 1;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    if (const VkResult vr =
            vkCreateDescriptorPool(res.device, &pool_info, nullptr, &res.descriptor_pool);
        vr != VK_SUCCESS) {
        return fail("vkCreateDescriptorPool", vr);
    }

    VkDescriptorSetAllocateInfo set_allocate_info{};
    set_allocate_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    set_allocate_info.descriptorPool = res.descriptor_pool;
    set_allocate_info.descriptorSetCount = 1;
    set_allocate_info.pSetLayouts = &res.set_layout;
    VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
    if (const VkResult vr = vkAllocateDescriptorSets(res.device, &set_allocate_info, &descriptor_set);
        vr != VK_SUCCESS) {
        return fail("vkAllocateDescriptorSets", vr);
    }

    VkDescriptorBufferInfo buffer_infos[2]{};
    buffer_infos[0].buffer = res.arg_buffer;
    buffer_infos[0].offset = 0;
    buffer_infos[0].range = VK_WHOLE_SIZE;
    buffer_infos[1].buffer = res.out_buffer;
    buffer_infos[1].offset = 0;
    buffer_infos[1].range = VK_WHOLE_SIZE;

    VkWriteDescriptorSet writes[2]{};
    for (uint32_t i = 0; i < 2; ++i) {
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = descriptor_set;
        writes[i].dstBinding = i == 0 ? kProbeArgBinding : kProbeOutBinding;
        writes[i].descriptorCount = 1;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[i].pBufferInfo = &buffer_infos[i];
    }
    vkUpdateDescriptorSets(res.device, 2, writes, 0, nullptr);

    // -- command buffer -----------------------------------------------------
    VkCommandPoolCreateInfo command_pool_info{};
    command_pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    command_pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    command_pool_info.queueFamilyIndex = ctx.compute_queue_family();
    if (const VkResult vr =
            vkCreateCommandPool(res.device, &command_pool_info, nullptr, &res.command_pool);
        vr != VK_SUCCESS) {
        return fail("vkCreateCommandPool", vr);
    }

    VkCommandBufferAllocateInfo command_allocate_info{};
    command_allocate_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    command_allocate_info.commandPool = res.command_pool;
    command_allocate_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    command_allocate_info.commandBufferCount = 1;
    VkCommandBuffer command_buffer = VK_NULL_HANDLE;
    if (const VkResult vr =
            vkAllocateCommandBuffers(res.device, &command_allocate_info, &command_buffer);
        vr != VK_SUCCESS) {
        return fail("vkAllocateCommandBuffers", vr);
    }

    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (const VkResult vr = vkBeginCommandBuffer(command_buffer, &begin_info); vr != VK_SUCCESS) {
        return fail("vkBeginCommandBuffer", vr);
    }

    vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, res.pipeline);
    vkCmdBindDescriptorSets(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, res.pipeline_layout,
                            kProbeSet, 1, &descriptor_set, 0, nullptr);

    const ProbePushConstants push{arg_count, out_count};
    vkCmdPushConstants(command_buffer, res.pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                       sizeof(push), &push);

    vkCmdDispatch(command_buffer, group_count, 1, 1);

    // Make the shader's writes available to the host. The fence signal below
    // already carries a dependency to the host domain, and the memory is
    // HOST_COHERENT so no invalidate is needed -- this barrier is the explicit
    // statement of the shader-write-to-host-read dependency rather than a
    // reliance on the submission's implicit one, which is the difference
    // between "works" and "is correct by the spec's own rules".
    VkMemoryBarrier host_barrier{};
    host_barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    host_barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    host_barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
    vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &host_barrier, 0, nullptr, 0, nullptr);

    if (const VkResult vr = vkEndCommandBuffer(command_buffer); vr != VK_SUCCESS) {
        return fail("vkEndCommandBuffer", vr);
    }

    // -- submit and wait ----------------------------------------------------
    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (const VkResult vr = vkCreateFence(res.device, &fence_info, nullptr, &res.fence);
        vr != VK_SUCCESS) {
        return fail("vkCreateFence", vr);
    }

    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &command_buffer;
    if (const VkResult vr = vkQueueSubmit(ctx.compute_queue(), 1, &submit_info, res.fence);
        vr != VK_SUCCESS) {
        return fail("vkQueueSubmit", vr);
    }

    const VkResult wait = vkWaitForFences(res.device, 1, &res.fence, VK_TRUE, kFenceTimeoutNs);
    if (wait != VK_SUCCESS) {
        // The queue may still be executing, and ~ProbeResources is about to
        // destroy the pipeline it is executing. Wait the device out first --
        // if THAT fails too the device is lost and there is nothing left to
        // do but report it.
        vkDeviceWaitIdle(res.device);
        return fail("vkWaitForFences", wait);
    }

    // -- readback -----------------------------------------------------------
    {
        void* mapped = nullptr;
        const VkResult vr = vkMapMemory(res.device, res.out_memory, 0, VK_WHOLE_SIZE, 0, &mapped);
        if (vr != VK_SUCCESS) return fail("vkMapMemory(results)", vr);
        std::memcpy(out.data(), mapped, out.size_bytes());
        vkUnmapMemory(res.device, res.out_memory);
    }

    return {};
}

}  // namespace spade::compute
