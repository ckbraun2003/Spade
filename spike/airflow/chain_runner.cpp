#include "spike/airflow/chain_runner.hpp"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <string>
#include <string_view>
#include <utility>

#include "physics/airflow/mac_grid.hpp"

namespace spade::airflow_gpu {

namespace {

// How long one submit or copy may take before the wait is called a failure: a
// hung device fails a test with a named reason rather than hanging it. A
// D1-sized fluid step is far below a second, so a minute is two orders of
// magnitude of headroom.
constexpr uint64_t kFenceTimeoutNs = 60ull * 1000ull * 1000ull * 1000ull;

// Vulkan's guaranteed maxMemoryAllocationSize, used when the device's own value
// cannot be read (no vkGetPhysicalDeviceProperties2).
constexpr uint64_t kGuaranteedAllocationSize = uint64_t{1} << 30;

// runner_memory_stats(): every runner's allocations, process-wide.
std::atomic<uint64_t> g_live_bytes{0};
std::atomic<uint64_t> g_allocations{0};

[[nodiscard]] std::unexpected<Error> refuse(Code code, const std::string& text) {
    return std::unexpected(Error{code, "airflow runner: " + text});
}

// The taxonomy of step_recorder.cpp's and dispatch_chain.cpp's map_vk_error,
// duplicated for the reason those files give (four lines; a shared header for
// them alone would be one more file to open).
[[nodiscard]] Error map_vk_error(VkResult result, std::string_view op) {
    const std::string where = "airflow runner: " + std::string(op);
    switch (result) {
        case VK_ERROR_DEVICE_LOST:
            return Error{Code::internal, "Vulkan device lost during " + where};
        case VK_ERROR_OUT_OF_DEVICE_MEMORY:
        case VK_ERROR_OUT_OF_HOST_MEMORY:
            return Error{Code::capacity_exceeded, "Vulkan allocation failed (VkResult " +
                                                      std::to_string(static_cast<int>(result)) + ") during " + where};
        default:
            return Error{Code::internal, "Vulkan call failed (VkResult " + std::to_string(static_cast<int>(result)) +
                                             ") during " + where};
    }
}

[[nodiscard]] Result<uint32_t> find_memory_type(VkPhysicalDevice physical, uint32_t type_bits,
                                                 VkMemoryPropertyFlags want) {
    VkPhysicalDeviceMemoryProperties props{};
    vkGetPhysicalDeviceMemoryProperties(physical, &props);
    for (uint32_t i = 0; i < props.memoryTypeCount; ++i) {
        if ((type_bits & (1u << i)) != 0u && (props.memoryTypes[i].propertyFlags & want) == want) return i;
    }
    return refuse(Code::internal, "no Vulkan memory type has the properties a runner buffer needs");
}

// One buffer and its own allocation, bound. On failure it destroys what it
// made, so the caller frees only what succeeded. `allocated` is the
// allocation's size, counted into runner_memory_stats().
[[nodiscard]] Result<void> make_buffer(VkDevice device, VkPhysicalDevice physical, uint64_t bytes,
                                       VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer,
                                       VkDeviceMemory& memory, uint64_t& allocated) {
    buffer = VK_NULL_HANDLE;
    memory = VK_NULL_HANDLE;
    allocated = 0;

    VkBufferCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    info.size = bytes;
    info.usage = usage;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (VkResult r = vkCreateBuffer(device, &info, nullptr, &buffer); r != VK_SUCCESS) {
        buffer = VK_NULL_HANDLE;
        return std::unexpected(map_vk_error(r, "vkCreateBuffer"));
    }

    VkMemoryRequirements requirements{};
    vkGetBufferMemoryRequirements(device, buffer, &requirements);
    Result<uint32_t> type = find_memory_type(physical, requirements.memoryTypeBits, properties);
    if (!type) {
        vkDestroyBuffer(device, buffer, nullptr);
        buffer = VK_NULL_HANDLE;
        return std::unexpected(type.error());
    }

    VkMemoryAllocateInfo allocate{};
    allocate.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocate.allocationSize = requirements.size;
    allocate.memoryTypeIndex = *type;
    if (VkResult r = vkAllocateMemory(device, &allocate, nullptr, &memory); r != VK_SUCCESS) {
        vkDestroyBuffer(device, buffer, nullptr);
        buffer = VK_NULL_HANDLE;
        memory = VK_NULL_HANDLE;
        return std::unexpected(map_vk_error(r, "vkAllocateMemory"));
    }
    allocated = requirements.size;
    g_allocations.fetch_add(1);
    g_live_bytes.fetch_add(allocated);

    if (VkResult r = vkBindBufferMemory(device, buffer, memory, 0); r != VK_SUCCESS) {
        vkFreeMemory(device, memory, nullptr);
        vkDestroyBuffer(device, buffer, nullptr);
        g_live_bytes.fetch_sub(allocated);
        buffer = VK_NULL_HANDLE;
        memory = VK_NULL_HANDLE;
        allocated = 0;
        return std::unexpected(map_vk_error(r, "vkBindBufferMemory"));
    }
    return {};
}

void free_buffer(VkDevice device, VkBuffer& buffer, VkDeviceMemory& memory, uint64_t& allocated) noexcept {
    if (buffer != VK_NULL_HANDLE) vkDestroyBuffer(device, buffer, nullptr);
    if (memory != VK_NULL_HANDLE) {
        vkFreeMemory(device, memory, nullptr);
        g_live_bytes.fetch_sub(allocated);
    }
    buffer = VK_NULL_HANDLE;
    memory = VK_NULL_HANDLE;
    allocated = 0;
}

// The memory dependency every recorded command buffer opens with, so ordering
// across submits rests on the barrier and not on the host's fence wait alone:
// earlier compute and transfer writes, made visible to `dst_stage`'s accesses.
void record_entry_barrier(VkCommandBuffer cmd, VkPipelineStageFlags dst_stage, VkAccessFlags dst_access) {
    VkMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = dst_access;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT, dst_stage, 0, 1,
                         &barrier, 0, nullptr, 0, nullptr);
}

[[nodiscard]] uint64_t saturating_add(uint64_t a, uint64_t b) noexcept {
    return a > UINT64_MAX - b ? UINT64_MAX : a + b;
}

}  // namespace

// ---------------------------------------------------------------------------
// limits, budget, stats
// ---------------------------------------------------------------------------

RunnerLimits read_runner_limits(const compute::VulkanContext& ctx) {
    VkPhysicalDeviceMaintenance3Properties maintenance3{};
    maintenance3.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MAINTENANCE_3_PROPERTIES;
    VkPhysicalDeviceProperties properties{};
    if (vkGetPhysicalDeviceProperties2 != nullptr) {
        VkPhysicalDeviceProperties2 properties2{};
        properties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
        properties2.pNext = &maintenance3;
        vkGetPhysicalDeviceProperties2(ctx.physical_device(), &properties2);
        properties = properties2.properties;
    } else {
        vkGetPhysicalDeviceProperties(ctx.physical_device(), &properties);
    }

    const VkPhysicalDeviceLimits& l = properties.limits;
    RunnerLimits out;
    out.max_storage_buffer_range = l.maxStorageBufferRange;
    out.max_memory_allocation_size =
        maintenance3.maxMemoryAllocationSize != 0 ? maintenance3.maxMemoryAllocationSize : kGuaranteedAllocationSize;
    out.max_per_stage_storage_buffers = l.maxPerStageDescriptorStorageBuffers;
    out.max_set_storage_buffers = l.maxDescriptorSetStorageBuffers;
    out.max_push_constants_size = l.maxPushConstantsSize;
    out.max_group_count = {l.maxComputeWorkGroupCount[0], l.maxComputeWorkGroupCount[1],
                           l.maxComputeWorkGroupCount[2]};
    out.max_group_size_x = l.maxComputeWorkGroupSize[0];
    out.max_group_invocations = l.maxComputeWorkGroupInvocations;
    return out;
}

// B0 replaces this body with min(4 GiB, budget - usage - 1 GiB reserve), from
// VulkanContext::memory_budget() (plan B, the lead's decision 4; design A1/A2).
uint64_t runner_budget_bytes(compute::VulkanContext&) { return physics::airflow::kGridByteCap; }

RunnerMemoryStats runner_memory_stats() noexcept {
    return RunnerMemoryStats{.live_bytes = g_live_bytes.load(), .allocations = g_allocations.load()};
}

// ---------------------------------------------------------------------------
// create
// ---------------------------------------------------------------------------

Result<std::unique_ptr<ChainRunner>> ChainRunner::create(compute::VulkanContext& ctx, const RunnerDesc& desc) {
    const RunnerLimits limits = read_runner_limits(ctx);
    const uint32_t wg = desc.workgroup_size;

    // ---- Every refusal, before anything is allocated (L6). ----------------
    if (desc.buffers.empty()) return refuse(Code::invalid_argument, "no buffers");
    for (std::size_t k = 0; k < desc.kernels.size(); ++k) {
        if (desc.kernels[k] == nullptr) {
            return refuse(Code::invalid_argument, "kernel " + std::to_string(k) + " is null");
        }
    }

    // The compiled variant first: a size the build never compiled is a build
    // question, whatever the device allows (make_compute_pipeline's selection,
    // asked here so it refuses before allocation).
    for (const compute::SpirvVariantSet* kernel : desc.kernels) {
        if (kernel->for_size(wg) != nullptr) continue;
        std::string compiled;
        for (const compute::SpirvVariant& v : kernel->all()) {
            compiled += (compiled.empty() ? "" : ", ") + std::to_string(v.workgroup_size);
        }
        return refuse(Code::invalid_argument, "kernel '" + std::string(kernel->name) +
                                                  "' has no variant compiled for workgroup size " +
                                                  std::to_string(wg) + " (compiled: " + compiled + ")");
    }
    if (wg > limits.max_group_size_x) {
        return refuse(Code::capacity_exceeded, "workgroup size " + std::to_string(wg) +
                                                   " is over maxComputeWorkGroupSize[0] (" +
                                                   std::to_string(limits.max_group_size_x) + ")");
    }
    if (wg > limits.max_group_invocations) {
        return refuse(Code::capacity_exceeded, "workgroup size " + std::to_string(wg) +
                                                   " is over maxComputeWorkGroupInvocations (" +
                                                   std::to_string(limits.max_group_invocations) + ")");
    }

    if (desc.push_bytes == 0 || desc.push_bytes % 4 != 0) {
        return refuse(Code::invalid_argument, "push_bytes " + std::to_string(desc.push_bytes) +
                                                  " must be a non-zero multiple of 4 (every dispatch pushes it)");
    }
    if (desc.push_bytes > limits.max_push_constants_size) {
        return refuse(Code::capacity_exceeded, "push_bytes " + std::to_string(desc.push_bytes) +
                                                   " is over maxPushConstantsSize (" +
                                                   std::to_string(limits.max_push_constants_size) + ")");
    }
    if (desc.push_bytes > compute::kMaxChainPushBytes) {
        return refuse(Code::capacity_exceeded, "push_bytes " + std::to_string(desc.push_bytes) +
                                                   " is over kMaxChainPushBytes (" +
                                                   std::to_string(compute::kMaxChainPushBytes) +
                                                   "), the push block a ChainCommand carries");
    }

    const uint64_t count = desc.buffers.size();
    if (count > limits.max_per_stage_storage_buffers) {
        return refuse(Code::capacity_exceeded, std::to_string(count) +
                                                   " buffers is over maxPerStageDescriptorStorageBuffers (" +
                                                   std::to_string(limits.max_per_stage_storage_buffers) + ")");
    }
    if (count > limits.max_set_storage_buffers) {
        return refuse(Code::capacity_exceeded, std::to_string(count) +
                                                   " buffers is over maxDescriptorSetStorageBuffers (" +
                                                   std::to_string(limits.max_set_storage_buffers) + ")");
    }
    if (desc.binding_base > UINT32_MAX - count) {
        return refuse(Code::invalid_argument, "binding_base " + std::to_string(desc.binding_base) + " plus " +
                                                  std::to_string(count) + " buffers passes the largest binding");
    }

    uint64_t device_bytes = 0;
    uint64_t staging_bytes = 0;
    for (const RunnerBuffer& b : desc.buffers) {
        if (b.bytes == 0) return refuse(Code::invalid_argument, "buffer '" + b.name + "' has 0 bytes");
        if (b.bytes > limits.max_storage_buffer_range) {
            return refuse(Code::capacity_exceeded, "buffer '" + b.name + "' asks for " + std::to_string(b.bytes) +
                                                       " bytes, over maxStorageBufferRange (" +
                                                       std::to_string(limits.max_storage_buffer_range) + ")");
        }
        if (b.bytes > limits.max_memory_allocation_size) {
            return refuse(Code::capacity_exceeded, "buffer '" + b.name + "' asks for " + std::to_string(b.bytes) +
                                                       " bytes, over maxMemoryAllocationSize (" +
                                                       std::to_string(limits.max_memory_allocation_size) +
                                                       "); each buffer is one allocation");
        }
        device_bytes = saturating_add(device_bytes, b.bytes);
        // Only a buffer that crosses to the host needs the staging buffer.
        if (b.kind != BufferKind::device_only) staging_bytes = std::max(staging_bytes, b.bytes);
    }
    const uint64_t total = saturating_add(device_bytes, staging_bytes);
    const uint64_t budget = runner_budget_bytes(ctx);
    if (total > budget) {
        return refuse(Code::capacity_exceeded,
                      std::to_string(count) + " buffers need " + std::to_string(total) + " bytes (device " +
                          std::to_string(device_bytes) + " + staging " + std::to_string(staging_bytes) +
                          "), over the budget of " + std::to_string(budget) +
                          " bytes (runner_budget_bytes: the 4 GiB airflow cap until B0's memory_budget())");
    }

    // ---- Allocation. Each step records what it made, so destroy() frees a
    // partial runner on any failure below. --------------------------------
    auto self = std::unique_ptr<ChainRunner>(new ChainRunner());
    self->ctx_ = &ctx;
    self->device_ = ctx.device();
    self->physical_device_ = ctx.physical_device();
    self->queue_ = ctx.compute_queue();
    self->push_bytes_ = desc.push_bytes;
    self->workgroup_size_ = wg;
    self->limits_ = limits;

    constexpr VkBufferUsageFlags kDeviceUsage =
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    self->buffers_.reserve(desc.buffers.size());
    for (const RunnerBuffer& b : desc.buffers) {
        Buffer& made = self->buffers_.emplace_back();
        made.name = b.name;
        made.bytes = b.bytes;
        made.kind = b.kind;
        if (Result<void> r = make_buffer(self->device_, self->physical_device_, b.bytes, kDeviceUsage,
                                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, made.buffer, made.memory, made.allocated);
            !r) {
            return std::unexpected(r.error());
        }
    }

    if (staging_bytes > 0) {
        self->staging_bytes_ = staging_bytes;
        if (Result<void> r = make_buffer(self->device_, self->physical_device_, staging_bytes,
                                         VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                         self->staging_, self->staging_memory_, self->staging_allocated_);
            !r) {
            return std::unexpected(r.error());
        }
        if (VkResult r = vkMapMemory(self->device_, self->staging_memory_, 0, VK_WHOLE_SIZE, 0, &self->staging_mapped_);
            r != VK_SUCCESS) {
            self->staging_mapped_ = nullptr;
            return std::unexpected(map_vk_error(r, "vkMapMemory (staging)"));
        }
    }

    // Set 0 is empty; set 1, the airflow set, holds the buffers.
    VkDescriptorSetLayoutCreateInfo empty_info{};
    empty_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    if (VkResult r = vkCreateDescriptorSetLayout(self->device_, &empty_info, nullptr, &self->empty_set_layout_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateDescriptorSetLayout (set 0)"));
    }

    std::vector<VkDescriptorSetLayoutBinding> bindings(desc.buffers.size());
    for (uint32_t i = 0; i < bindings.size(); ++i) {
        bindings[i].binding = desc.binding_base + i;
        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[i].descriptorCount = 1;
        bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo set_info{};
    set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    set_info.bindingCount = static_cast<uint32_t>(bindings.size());
    set_info.pBindings = bindings.data();
    if (VkResult r = vkCreateDescriptorSetLayout(self->device_, &set_info, nullptr, &self->set_layout_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateDescriptorSetLayout (airflow set)"));
    }

    const VkDescriptorSetLayout set_layouts[] = {self->empty_set_layout_, self->set_layout_};
    static_assert(kAirflowSet == 1, "the pipeline layout lists set 0, then the airflow set");
    VkPushConstantRange push_range{};
    push_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    push_range.offset = 0;
    push_range.size = desc.push_bytes;
    VkPipelineLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layout_info.setLayoutCount = 2;
    layout_info.pSetLayouts = set_layouts;
    layout_info.pushConstantRangeCount = 1;
    layout_info.pPushConstantRanges = &push_range;
    if (VkResult r = vkCreatePipelineLayout(self->device_, &layout_info, nullptr, &self->pipeline_layout_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreatePipelineLayout"));
    }

    // The runner's own descriptor pool (A5): one set, one descriptor a buffer.
    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    pool_size.descriptorCount = static_cast<uint32_t>(desc.buffers.size());
    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.maxSets = 1;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    if (VkResult r = vkCreateDescriptorPool(self->device_, &pool_info, nullptr, &self->descriptor_pool_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateDescriptorPool"));
    }
    VkDescriptorSetAllocateInfo set_allocate{};
    set_allocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    set_allocate.descriptorPool = self->descriptor_pool_;
    set_allocate.descriptorSetCount = 1;
    set_allocate.pSetLayouts = &self->set_layout_;
    if (VkResult r = vkAllocateDescriptorSets(self->device_, &set_allocate, &self->set_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkAllocateDescriptorSets"));
    }
    std::vector<VkDescriptorBufferInfo> infos(desc.buffers.size());
    std::vector<VkWriteDescriptorSet> writes(desc.buffers.size());
    for (uint32_t i = 0; i < infos.size(); ++i) {
        infos[i].buffer = self->buffers_[i].buffer;
        infos[i].offset = 0;
        infos[i].range = self->buffers_[i].bytes;
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = self->set_;
        writes[i].dstBinding = desc.binding_base + i;
        writes[i].descriptorCount = 1;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[i].pBufferInfo = &infos[i];
    }
    vkUpdateDescriptorSets(self->device_, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

    // Pipeline i = kernels[i], Core's selection and creation (dispatch_chain).
    self->shaders_.assign(desc.kernels.size(), VK_NULL_HANDLE);
    self->pipelines_.assign(desc.kernels.size(), VK_NULL_HANDLE);
    for (std::size_t k = 0; k < desc.kernels.size(); ++k) {
        if (Result<void> made =
                compute::make_compute_pipeline(self->device_, self->pipeline_layout_, *desc.kernels[k], wg,
                                               "airflow runner", self->shaders_[k], self->pipelines_[k]);
            !made) {
            return std::unexpected(made.error());
        }
    }

    // The runner's own command pool (A5). RESET_COMMAND_BUFFER because the
    // transfer buffer is re-recorded for every copy; a chain's buffer is
    // recorded once and never reset.
    VkCommandPoolCreateInfo command_pool_info{};
    command_pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    command_pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    command_pool_info.queueFamilyIndex = ctx.compute_queue_family();
    if (VkResult r = vkCreateCommandPool(self->device_, &command_pool_info, nullptr, &self->command_pool_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateCommandPool"));
    }
    VkCommandBufferAllocateInfo command_allocate{};
    command_allocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    command_allocate.commandPool = self->command_pool_;
    command_allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    command_allocate.commandBufferCount = 1;
    if (VkResult r = vkAllocateCommandBuffers(self->device_, &command_allocate, &self->transfer_cmd_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkAllocateCommandBuffers (transfer)"));
    }

    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (VkResult r = vkCreateFence(self->device_, &fence_info, nullptr, &self->fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateFence"));
    }
    return self;
}

// ---------------------------------------------------------------------------
// upload / read
// ---------------------------------------------------------------------------

Result<void> ChainRunner::check_transfer(uint32_t buffer, std::size_t bytes, const char* op) const {
    if (buffer >= buffers_.size()) {
        return refuse(Code::invalid_argument, std::string(op) + ": buffer index " + std::to_string(buffer) +
                                                  " is out of range (the runner has " +
                                                  std::to_string(buffers_.size()) + ")");
    }
    const Buffer& b = buffers_[buffer];
    if (b.kind == BufferKind::device_only) {
        return refuse(Code::invalid_argument, std::string(op) + ": buffer '" + b.name +
                                                  "' is device_only, which never crosses to the host; tag it "
                                                  "resident or staged");
    }
    if (bytes > b.bytes) {
        return refuse(Code::invalid_argument, std::string(op) + ": " + std::to_string(bytes) +
                                                  " bytes for buffer '" + b.name + "', which holds " +
                                                  std::to_string(b.bytes));
    }
    return {};
}

Result<void> ChainRunner::upload(uint32_t buffer, std::span<const std::byte> bytes) {
    if (Result<void> ok = check_transfer(buffer, bytes.size(), "upload"); !ok) return ok;
    if (bytes.empty()) return {};
    std::memcpy(staging_mapped_, bytes.data(), bytes.size());
    return run_transfer(true, buffer, bytes.size());
}

Result<void> ChainRunner::read(uint32_t buffer, std::span<std::byte> out) const {
    if (Result<void> ok = check_transfer(buffer, out.size(), "read"); !ok) return ok;
    if (out.empty()) return {};
    if (Result<void> copied = run_transfer(false, buffer, out.size()); !copied) return copied;
    std::memcpy(out.data(), staging_mapped_, out.size());
    return {};
}

// One copy between the staging buffer and buffer `buffer`, recorded into the
// transfer command buffer, submitted and waited for.
Result<void> ChainRunner::run_transfer(bool to_device, uint32_t buffer, std::size_t bytes) const {
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (VkResult r = vkBeginCommandBuffer(transfer_cmd_, &begin); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkBeginCommandBuffer (transfer)"));
    }
    // Earlier chains' writes (and their reads, by the execution dependency)
    // finish before the copy touches the buffer.
    record_entry_barrier(transfer_cmd_, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_TRANSFER_WRITE_BIT);
    VkBufferCopy region{};
    region.size = bytes;
    if (to_device) {
        vkCmdCopyBuffer(transfer_cmd_, staging_, buffers_[buffer].buffer, 1, &region);
    } else {
        vkCmdCopyBuffer(transfer_cmd_, buffers_[buffer].buffer, staging_, 1, &region);
        // The copy's writes, made visible to the host's read of the mapping.
        VkMemoryBarrier to_host{};
        to_host.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        to_host.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        to_host.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(transfer_cmd_, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &to_host,
                             0, nullptr, 0, nullptr);
    }
    if (VkResult r = vkEndCommandBuffer(transfer_cmd_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkEndCommandBuffer (transfer)"));
    }
    return submit_and_wait(transfer_cmd_, to_device ? "upload" : "read");
}

Result<void> ChainRunner::submit_and_wait(VkCommandBuffer cmd, const char* op) const {
    if (in_flight_) {
        return refuse(Code::internal, std::string(op) + ": an earlier submit never finished; the runner is unusable");
    }
    if (VkResult r = vkResetFences(device_, 1, &fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkResetFences"));
    }
    VkSubmitInfo info{};
    info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    info.commandBufferCount = 1;
    info.pCommandBuffers = &cmd;
    if (VkResult r = vkQueueSubmit(queue_, 1, &info, fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, std::string("vkQueueSubmit (") + op + ")"));
    }
    const VkResult wait = vkWaitForFences(device_, 1, &fence_, VK_TRUE, kFenceTimeoutNs);
    if (wait == VK_TIMEOUT) {
        in_flight_ = true;
        return refuse(Code::internal, std::string(op) + ": the device did not finish within 60 s");
    }
    if (wait != VK_SUCCESS) {
        in_flight_ = true;
        return std::unexpected(map_vk_error(wait, std::string("vkWaitForFences (") + op + ")"));
    }
    return {};
}

// ---------------------------------------------------------------------------
// record / submit / durations
// ---------------------------------------------------------------------------

Result<uint32_t> ChainRunner::record(std::span<const compute::ChainCommand> chain) {
    // ---- Every refusal, before the command buffer exists. ----------------
    uint32_t marks = 0;
    for (std::size_t i = 0; i < chain.size(); ++i) {
        const compute::ChainCommand& c = chain[i];
        if (!c.is_dispatch) {
            ++marks;
            continue;
        }
        if (c.pipeline >= pipelines_.size()) {
            return refuse(Code::invalid_argument, "command " + std::to_string(i) + ": pipeline index " +
                                                      std::to_string(c.pipeline) + " is out of range (the runner has " +
                                                      std::to_string(pipelines_.size()) + " pipelines)");
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (c.groups[axis] > limits_.max_group_count[axis]) {
                return refuse(Code::capacity_exceeded,
                              "command " + std::to_string(i) + " asks for " + std::to_string(c.groups[axis]) +
                                  " groups on axis " + std::to_string(axis) + ", over maxComputeWorkGroupCount[" +
                                  std::to_string(axis) + "] (" + std::to_string(limits_.max_group_count[axis]) + ")");
            }
        }
    }

    Chain made;
    made.marks = marks;
    if (marks > 0) {
        // One mark block, `marks` + 1 slots: slot 0 when the chain begins, slot
        // m at the m-th mark (timestamps.hpp's substep block, used once).
        std::vector<std::string> names;
        names.reserve(marks);
        for (uint32_t m = 1; m <= marks; ++m) names.push_back("mark " + std::to_string(m));
        Result<std::unique_ptr<compute::PassTimestamps>> timestamps =
            compute::PassTimestamps::create(*ctx_, 1, std::move(names));
        if (!timestamps) return std::unexpected(timestamps.error());
        made.timestamps = std::move(*timestamps);
    }

    VkCommandBufferAllocateInfo allocate{};
    allocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocate.commandPool = command_pool_;
    allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocate.commandBufferCount = 1;
    if (VkResult r = vkAllocateCommandBuffers(device_, &allocate, &made.cmd); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkAllocateCommandBuffers (chain)"));
    }
    const auto fail = [&](VkResult r, const char* op) {
        vkFreeCommandBuffers(device_, command_pool_, 1, &made.cmd);
        return std::unexpected(map_vk_error(r, op));
    };

    // Not ONE_TIME_SUBMIT: the same recording is resubmitted.
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    if (VkResult r = vkBeginCommandBuffer(made.cmd, &begin); r != VK_SUCCESS) {
        return fail(r, "vkBeginCommandBuffer (chain)");
    }
    vkCmdBindDescriptorSets(made.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_, kAirflowSet, 1, &set_, 0,
                            nullptr);
    const compute::PassTimestamps* timestamps = made.timestamps.get();
    if (timestamps != nullptr) timestamps->record_reset(made.cmd);
    // The previous submit's (or an upload's) writes, visible to this chain.
    record_entry_barrier(made.cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT);
    if (timestamps != nullptr) timestamps->record_mark(made.cmd, 0, 0);

    uint32_t next_mark = 1;
    made.counts = compute::record_dispatch_chain(
        made.cmd, pipeline_layout_, pipelines_, chain, 0, push_bytes_,
        [timestamps, &next_mark](VkCommandBuffer cmd, uint32_t, uint32_t) {
            if (timestamps != nullptr) timestamps->record_mark(cmd, 0, next_mark);
            ++next_mark;
        });

    if (VkResult r = vkEndCommandBuffer(made.cmd); r != VK_SUCCESS) {
        return fail(r, "vkEndCommandBuffer (chain)");
    }
    chains_.push_back(std::move(made));
    return static_cast<uint32_t>(chains_.size() - 1);
}

Result<void> ChainRunner::submit(uint32_t chain, uint64_t n) {
    if (chain >= chains_.size()) {
        return refuse(Code::invalid_argument, "submit: chain " + std::to_string(chain) +
                                                  " was never recorded (the runner has " +
                                                  std::to_string(chains_.size()) + ")");
    }
    Chain& c = chains_[chain];
    for (uint64_t i = 0; i < n; ++i) {
        if (Result<void> ran = submit_and_wait(c.cmd, "submit"); !ran) return ran;
        ++c.submits;
    }
    return {};
}

Result<std::vector<uint64_t>> ChainRunner::durations_ns(uint32_t chain) const {
    if (chain >= chains_.size()) {
        return refuse(Code::invalid_argument, "durations_ns: chain " + std::to_string(chain) +
                                                  " was never recorded (the runner has " +
                                                  std::to_string(chains_.size()) + ")");
    }
    const Chain& c = chains_[chain];
    // Before the first submit the queries were never written, and reading them
    // with WAIT_BIT would never return.
    if (c.timestamps == nullptr || !c.timestamps->supported() || c.submits == 0) return std::vector<uint64_t>{};
    if (in_flight_) return refuse(Code::internal, "durations_ns: an earlier submit never finished");
    Result<PassDurationsNs> read = c.timestamps->read_durations_ns();
    if (!read) return std::unexpected(read.error());
    std::vector<uint64_t> out;
    out.reserve(read->passes.size());
    for (const PassDuration& d : read->passes) out.push_back(static_cast<uint64_t>(d.ns + 0.5));
    return out;
}

compute::ChainCounts ChainRunner::recorded_counts(uint32_t chain) const noexcept {
    return chain < chains_.size() ? chains_[chain].counts : compute::ChainCounts{};
}

// ---------------------------------------------------------------------------
// teardown
// ---------------------------------------------------------------------------

void ChainRunner::destroy() noexcept {
    if (device_ == VK_NULL_HANDLE) return;
    // A wait that failed may have left work on the queue that uses what is
    // about to be destroyed.
    if (in_flight_) vkDeviceWaitIdle(device_);

    chains_.clear();  // each chain's query pool; its command buffer goes with the pool
    if (fence_ != VK_NULL_HANDLE) vkDestroyFence(device_, fence_, nullptr);
    if (command_pool_ != VK_NULL_HANDLE) vkDestroyCommandPool(device_, command_pool_, nullptr);
    if (descriptor_pool_ != VK_NULL_HANDLE) vkDestroyDescriptorPool(device_, descriptor_pool_, nullptr);
    for (VkPipeline p : pipelines_) {
        if (p != VK_NULL_HANDLE) vkDestroyPipeline(device_, p, nullptr);
    }
    for (VkShaderModule s : shaders_) {
        if (s != VK_NULL_HANDLE) vkDestroyShaderModule(device_, s, nullptr);
    }
    if (pipeline_layout_ != VK_NULL_HANDLE) vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);
    if (set_layout_ != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device_, set_layout_, nullptr);
    if (empty_set_layout_ != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device_, empty_set_layout_, nullptr);
    for (Buffer& b : buffers_) free_buffer(device_, b.buffer, b.memory, b.allocated);
    if (staging_mapped_ != nullptr) vkUnmapMemory(device_, staging_memory_);
    free_buffer(device_, staging_, staging_memory_, staging_allocated_);

    fence_ = VK_NULL_HANDLE;
    command_pool_ = VK_NULL_HANDLE;
    transfer_cmd_ = VK_NULL_HANDLE;
    descriptor_pool_ = VK_NULL_HANDLE;
    set_ = VK_NULL_HANDLE;
    pipelines_.clear();
    shaders_.clear();
    pipeline_layout_ = VK_NULL_HANDLE;
    set_layout_ = VK_NULL_HANDLE;
    empty_set_layout_ = VK_NULL_HANDLE;
    buffers_.clear();
    staging_mapped_ = nullptr;
    device_ = VK_NULL_HANDLE;
}

ChainRunner::~ChainRunner() { destroy(); }

}  // namespace spade::airflow_gpu
