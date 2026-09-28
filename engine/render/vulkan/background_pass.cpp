// ---------------------------------------------------------------------------
// background_pass.cpp -- stage 2 of the GPU rasterizer: the Vulkan
// implementation behind BackgroundPass, and the host half of
// shaders/kernels/raster_background.slang.
//
// ===========================================================================
// ⛔ THIS FILE DOES NOT COMPILE TODAY, AND THAT IS THE SPLIT WORKING, NOT AN
// OVERSIGHT.
//
// The #include of "raster_background.spv.gen.hpp" below names a header that
// DOES NOT EXIST YET: cmake/SpadeSlang.cmake's spade_slang_kernel() is what
// compiles the kernel and makes engine/shaders/tools/embed_spirv.py emit it.
// That registration also appends the kernel's name to CMake's global kernel
// property, which writes gen::kCompiledSpirvKernelCount -- and
// tests/test_slang_layouts.cpp's SlangSpirv.EveryCompiledVariantIsScanned
// compares that count against std::size(kSpirvModules). So the registration
// and a new kSpirvModules[] row MUST land in ONE commit, and that commit is
// the first in this sequence that cannot honestly be written without a build.
//
// Until it lands, no CMakeLists names this file, nothing links it, and the
// missing header cannot break anyone. ⚠ WHAT IT CAN DO IS LOOK FINISHED: an
// inert translation unit passes every "is it there?" check while having been
// compiled zero times. AN ARTIFACT THAT RECORDS A STEP IS EVIDENCE THE STEP
// WAS INTENDED, NEVER THAT IT COMPLETED -- so the reviewer should read this as
// reviewed-by-reading, which is not the same as compiled.
//
// ===========================================================================
// THE HELPERS BELOW ARE A THIRD COPY AND I AM SAYING SO
//
// map_vk_error/find_memory_type/create_buffer are near-verbatim from
// compute/vulkan/state_mirror.cpp, where they are anonymous-namespace and so
// unreachable from here; probe_runner.cpp already carries its own second copy
// of the memory-type half. THREE COPIES IS THE POINT AT WHICH THIS STOPS BEING
// a convention and starts being a maintenance liability -- the taxonomy
// decision in map_vk_error (out-of-memory is capacity_exceeded, device-lost is
// internal) is a CONTRACT, and three independent copies of a contract drift.
// Extracting them into compute/vulkan/ as a shared internal header is owed;
// it is not done here because it would edit a file the physics path links and
// this commit is meant to be inert. ⛔ WHOEVER WIRES THE BUILD SHOULD DO THE
// EXTRACTION IN THAT SAME COMMIT, because that is the first moment the two
// copies are compiled into one program and a divergence becomes checkable.
// ---------------------------------------------------------------------------

#include "render/vulkan/background_pass.hpp"

#include <cmath>
#include <cstring>
#include <string>
#include <string_view>

#include "compute/spirv_variants.hpp"

// The generated header, which the wiring commit creates. It includes
// compute/spirv_variants.hpp itself, but that header is a BUILD ARTIFACT and
// its include list is not a contract -- so the type this file names is
// included above, by its own header, deliberately.
#include "raster_background.spv.gen.hpp"

namespace spade::render {

namespace {

// The kernel's own binding numbers and set index. ⛔ THESE THREE MUST MATCH
// raster_background.slang's [[vk::binding(n, 1)]] literals and nothing checks
// that they do -- same unguarded-mirror caveat as the Params struct, and for
// the same reason (this kernel is deliberately outside the generated physics
// binding registry, so there is no static_assert to lean on).
constexpr uint32_t kBindingPixels = 0;
constexpr uint32_t kBindingPlanes = 1;
constexpr uint32_t kBindingParams = 2;
constexpr uint32_t kRenderSet = 1;

// ⛔ MUST EQUAL the kernel's [numthreads(64, 1, 1)]. The dispatch grid is
// pixel_count / this, so a disagreement does not fail -- it silently leaves
// the tail of the image unwritten (too small) or dispatches threads that
// early-return (too large). The kernel refuses to join the workgroup-variant
// family precisely so this number is a literal in two files rather than a knob
// in three, and a caller cannot move it: GpuRasterizerDesc has no
// workgroup_size field, because A KNOB THAT CANNOT CHANGE THE OUTCOME IS WORSE
// THAN NO KNOB.
constexpr uint32_t kLocalSizeX = 64;

// Ground-plane buffer floor. Every world shipped today has 0 or 1 plane; the
// floor exists so the common case never reallocates and so a zero-plane frame
// still has a valid buffer bound (Vulkan forbids a zero-size buffer).
constexpr uint32_t kMinPlaneCapacity = 8;

constexpr uint32_t kBytesPerTexel = 4;

// ---------------------------------------------------------------------------
// VkResult -> spade::Error. See this file's header note: a copy, deliberately
// identical to state_mirror.cpp's, including the taxonomy -- out-of-memory is
// capacity_exceeded because that is what "the request does not fit" means
// everywhere else in this tree, and device-lost is internal.
// ---------------------------------------------------------------------------
[[nodiscard]] Error map_vk_error(VkResult result, std::string_view op) {
    switch (result) {
        case VK_ERROR_DEVICE_LOST:
            return Error{Code::internal, "Vulkan device lost during " + std::string(op)};
        case VK_ERROR_OUT_OF_DEVICE_MEMORY:
        case VK_ERROR_OUT_OF_HOST_MEMORY:
            return Error{Code::capacity_exceeded,
                         "Vulkan allocation failed (VkResult " + std::to_string(static_cast<int>(result)) +
                             ") during " + std::string(op)};
        default:
            return Error{Code::internal, "Vulkan call failed (VkResult " +
                                             std::to_string(static_cast<int>(result)) + ") during " +
                                             std::string(op)};
    }
}

[[nodiscard]] Result<uint32_t> find_memory_type(VkPhysicalDevice phys, uint32_t type_bits,
                                                VkMemoryPropertyFlags want) {
    VkPhysicalDeviceMemoryProperties props{};
    vkGetPhysicalDeviceMemoryProperties(phys, &props);
    for (uint32_t i = 0; i < props.memoryTypeCount; ++i) {
        if ((type_bits & (1u << i)) && (props.memoryTypes[i].propertyFlags & want) == want) {
            return i;
        }
    }
    return std::unexpected(
        Error{Code::internal, "no Vulkan memory type satisfies the requested properties"});
}

// Creates one buffer + backing memory, optionally persistently mapped. On ANY
// failure, tears down whatever THIS CALL created before returning, so the
// caller's destroy() only ever sees buffers that were finished.
[[nodiscard]] Result<void> create_buffer(VkDevice device, VkPhysicalDevice phys, VkDeviceSize size,
                                         VkBufferUsageFlags usage, VkMemoryPropertyFlags mem_props, bool map,
                                         VkBuffer& out_buffer, VkDeviceMemory& out_memory, void*& out_mapped) {
    out_buffer = VK_NULL_HANDLE;
    out_memory = VK_NULL_HANDLE;
    out_mapped = nullptr;

    const VkDeviceSize alloc_size = size == 0 ? VkDeviceSize{4} : size;

    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = alloc_size;
    buffer_info.usage = usage;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VkResult r = vkCreateBuffer(device, &buffer_info, nullptr, &out_buffer);
    if (r != VK_SUCCESS) {
        out_buffer = VK_NULL_HANDLE;
        return std::unexpected(map_vk_error(r, "vkCreateBuffer"));
    }

    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(device, out_buffer, &req);

    Result<uint32_t> type = find_memory_type(phys, req.memoryTypeBits, mem_props);
    if (!type) {
        vkDestroyBuffer(device, out_buffer, nullptr);
        out_buffer = VK_NULL_HANDLE;
        return std::unexpected(type.error());
    }

    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = req.size;
    alloc_info.memoryTypeIndex = *type;

    r = vkAllocateMemory(device, &alloc_info, nullptr, &out_memory);
    if (r != VK_SUCCESS) {
        vkDestroyBuffer(device, out_buffer, nullptr);
        out_buffer = VK_NULL_HANDLE;
        out_memory = VK_NULL_HANDLE;
        return std::unexpected(map_vk_error(r, "vkAllocateMemory"));
    }

    r = vkBindBufferMemory(device, out_buffer, out_memory, 0);
    if (r != VK_SUCCESS) {
        vkFreeMemory(device, out_memory, nullptr);
        vkDestroyBuffer(device, out_buffer, nullptr);
        out_buffer = VK_NULL_HANDLE;
        out_memory = VK_NULL_HANDLE;
        return std::unexpected(map_vk_error(r, "vkBindBufferMemory"));
    }

    if (map) {
        r = vkMapMemory(device, out_memory, 0, alloc_size, 0, &out_mapped);
        if (r != VK_SUCCESS) {
            vkFreeMemory(device, out_memory, nullptr);
            vkDestroyBuffer(device, out_buffer, nullptr);
            out_buffer = VK_NULL_HANDLE;
            out_memory = VK_NULL_HANDLE;
            out_mapped = nullptr;
            return std::unexpected(map_vk_error(r, "vkMapMemory"));
        }
    }

    return {};
}

// Copies a glm::vec3 into the three floats a std430/std140 float3 field holds.
// Named rather than inlined three-at-a-time because every one of these is a
// mirror field whose ORDER is the contract.
void store_vec3(float (&dst)[3], const glm::vec3& src) noexcept {
    dst[0] = src.x;
    dst[1] = src.y;
    dst[2] = src.z;
}

}  // namespace

// ---------------------------------------------------------------------------
// create
// ---------------------------------------------------------------------------

Result<std::unique_ptr<BackgroundPass>> BackgroundPass::create(compute::VulkanContext& ctx,
                                                              uint32_t max_width, uint32_t max_height) {
    // ARGUMENT VALIDATION BEFORE RESOURCE ACQUISITION, the same ordering
    // raster_gpu.cpp's facade states and for the same reason: a zero bound is
    // wrong independently of what device is present, so the diagnosis must not
    // depend on the environment. The facade already rejects zeros; this is a
    // public factory and does not get to assume its only caller.
    if (max_width == 0 || max_height == 0) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "BackgroundPass::create: max_width/max_height must both be non-zero; got " +
                                         std::to_string(max_width) + "x" + std::to_string(max_height)});
    }

    const VkDevice device = ctx.device();
    const VkPhysicalDevice phys = ctx.physical_device();

    // ------------------------------------------------------------------
    // ⛔ THE 1-D DISPATCH HAS A CEILING AND IT IS REACHABLE AT 4K.
    //
    // The kernel indexes pixels as a flat SV_DispatchThreadID.x, so the grid is
    // ceil(w*h / 64) workgroups in ONE dimension. Vulkan only guarantees
    // maxComputeWorkGroupCount[0] >= 65535, which is 65535*64 = 4,194,240
    // pixels -- about 2048x2048. 2560x1440 (this desc's default) is 3.69 Mpx and
    // fits; 3840x2160 is 8.29 Mpx and DOES NOT.
    //
    // Checked HERE, against the real device limit, rather than at the first
    // render() call: a bound the caller asked for and cannot use is a
    // create()-time fact, and finding it at frame time would mean a rasterizer
    // that constructed successfully and then failed on a size it had already
    // accepted. The fix when someone needs 4K is a 2-D dispatch in the kernel,
    // which is why the error says so instead of just reporting a number.
    // ------------------------------------------------------------------
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(phys, &props);
    const uint64_t pixel_budget = static_cast<uint64_t>(max_width) * static_cast<uint64_t>(max_height);
    const uint64_t groups_needed = (pixel_budget + kLocalSizeX - 1) / kLocalSizeX;
    if (groups_needed > static_cast<uint64_t>(props.limits.maxComputeWorkGroupCount[0])) {
        return std::unexpected(
            Error{Code::invalid_argument,
                  "BackgroundPass::create: " + std::to_string(max_width) + "x" + std::to_string(max_height) +
                      " needs " + std::to_string(groups_needed) +
                      " workgroups in one dimension but this device's maxComputeWorkGroupCount[0] is " +
                      std::to_string(props.limits.maxComputeWorkGroupCount[0]) +
                      ". raster_background.slang dispatches a FLAT 1-D grid over pixels; supporting a target "
                      "this large means giving that kernel a 2-D dispatch, not raising this bound."});
    }

    auto self = std::unique_ptr<BackgroundPass>(new BackgroundPass());
    self->ctx_ = &ctx;
    self->max_width_ = max_width;
    self->max_height_ = max_height;
    self->plane_capacity_ = kMinPlaneCapacity;

    // ------------------------------------------------------------------
    // Buffers. Sized ONCE, from the descriptor's bounds, and never grown --
    // background_pass.hpp's "sized once, never grown" note carries the
    // argument; this is where the sizes are committed to.
    // ------------------------------------------------------------------
    const VkDeviceSize pixels_bytes =
        static_cast<VkDeviceSize>(pixel_budget) * static_cast<VkDeviceSize>(kBytesPerTexel);

    void* unused_mapped = nullptr;  // create_buffer(map=false) always writes null here; never read
    if (Result<void> made = create_buffer(device, phys, pixels_bytes,
                                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, false, self->pixels_,
                                          self->pixels_memory_, unused_mapped);
        !made) {
        return std::unexpected(made.error());
    }

    // HOST_COHERENT on both per-frame buffers and on the readback. Not a
    // convenience: the alternative is an explicit vkFlushMappedMemoryRanges
    // before the submit and a vkInvalidateMappedMemoryRanges after the wait,
    // and a MISSING one of those is invisible on an integrated GPU (where the
    // memory is coherent anyway) and wrong on a discrete one. Requiring
    // coherence makes the correctness a property of the allocation rather than
    // of remembering two calls -- A SHARED LIMITATION IS CHEAPER THAN AN
    // UNSHARED CONVENIENCE.
    constexpr VkMemoryPropertyFlags kHostCoherent =
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

    if (Result<void> made = create_buffer(device, phys, sizeof(PlaneGpu) * self->plane_capacity_,
                                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, kHostCoherent, true,
                                          self->planes_, self->planes_memory_, self->planes_mapped_);
        !made) {
        return std::unexpected(made.error());
    }

    if (Result<void> made =
            create_buffer(device, phys, sizeof(Params), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, kHostCoherent,
                          true, self->params_, self->params_memory_, self->params_mapped_);
        !made) {
        return std::unexpected(made.error());
    }

    if (Result<void> made =
            create_buffer(device, phys, pixels_bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, kHostCoherent, true,
                          self->staging_, self->staging_memory_, self->staging_mapped_);
        !made) {
        return std::unexpected(made.error());
    }

    // ------------------------------------------------------------------
    // Descriptor set layouts. TWO of them, and the empty one is load-bearing
    // -- see background_pass.hpp's empty_set0_layout_ note for why set 1 only
    // means set 1 if set 0 is also described.
    // ------------------------------------------------------------------
    VkDescriptorSetLayoutCreateInfo empty_info{};
    empty_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    empty_info.bindingCount = 0;
    empty_info.pBindings = nullptr;
    if (VkResult r = vkCreateDescriptorSetLayout(device, &empty_info, nullptr, &self->empty_set0_layout_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateDescriptorSetLayout (empty set 0)"));
    }

    // Descriptor TYPES follow the kernel's declarations, not a house default:
    // RWStructuredBuffer and StructuredBuffer are both storage buffers, and
    // ConstantBuffer is a UNIFORM buffer. Getting the last one wrong is a
    // pipeline the driver rejects, which is the good failure -- but it is worth
    // saying out loud that the Slang keyword is the authority here.
    VkDescriptorSetLayoutBinding bindings[3]{};
    bindings[0].binding = kBindingPixels;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bindings[1].binding = kBindingPlanes;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bindings[2].binding = kBindingParams;
    bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[2].descriptorCount = 1;
    bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutCreateInfo set_layout_info{};
    set_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    set_layout_info.bindingCount = 3;
    set_layout_info.pBindings = bindings;
    if (VkResult r = vkCreateDescriptorSetLayout(device, &set_layout_info, nullptr, &self->set_layout_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateDescriptorSetLayout (render set 1)"));
    }

    VkDescriptorPoolSize pool_sizes[2]{};
    pool_sizes[0].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    pool_sizes[0].descriptorCount = 2;
    pool_sizes[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    pool_sizes[1].descriptorCount = 1;

    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.maxSets = 1;
    pool_info.poolSizeCount = 2;
    pool_info.pPoolSizes = pool_sizes;
    if (VkResult r = vkCreateDescriptorPool(device, &pool_info, nullptr, &self->descriptor_pool_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateDescriptorPool"));
    }

    VkDescriptorSetAllocateInfo set_alloc{};
    set_alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    set_alloc.descriptorPool = self->descriptor_pool_;
    set_alloc.descriptorSetCount = 1;
    set_alloc.pSetLayouts = &self->set_layout_;
    if (VkResult r = vkAllocateDescriptorSets(device, &set_alloc, &self->descriptor_set_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkAllocateDescriptorSets"));
    }

    // The set is written ONCE, at create(): every buffer it names persists for
    // this object's lifetime and none is ever reallocated, which is exactly
    // what "sized once, never grown" buys. A path that grew a buffer would
    // have to rewrite the descriptor every frame and would make a frame's cost
    // depend on its history.
    VkDescriptorBufferInfo buffer_infos[3]{};
    buffer_infos[0].buffer = self->pixels_;
    buffer_infos[0].offset = 0;
    buffer_infos[0].range = VK_WHOLE_SIZE;
    buffer_infos[1].buffer = self->planes_;
    buffer_infos[1].offset = 0;
    buffer_infos[1].range = VK_WHOLE_SIZE;
    buffer_infos[2].buffer = self->params_;
    buffer_infos[2].offset = 0;
    buffer_infos[2].range = sizeof(Params);

    VkWriteDescriptorSet writes[3]{};
    for (uint32_t i = 0; i < 3; ++i) {
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = self->descriptor_set_;
        writes[i].dstBinding = bindings[i].binding;
        writes[i].descriptorCount = 1;
        writes[i].descriptorType = bindings[i].descriptorType;
        writes[i].pBufferInfo = &buffer_infos[i];
    }
    vkUpdateDescriptorSets(device, 3, writes, 0, nullptr);

    // ------------------------------------------------------------------
    // Pipeline layout: BOTH sets, in index order. No push constants -- the
    // kernel has none, and the parameter block is a uniform buffer precisely
    // because it is larger than the 128 bytes a push-constant range is
    // guaranteed to offer.
    // ------------------------------------------------------------------
    const VkDescriptorSetLayout layouts[2] = {self->empty_set0_layout_, self->set_layout_};
    VkPipelineLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layout_info.setLayoutCount = 2;
    layout_info.pSetLayouts = layouts;
    if (VkResult r = vkCreatePipelineLayout(device, &layout_info, nullptr, &self->pipeline_layout_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreatePipelineLayout"));
    }

    // ------------------------------------------------------------------
    // The module. One variant, at the kernel's literal 64 -- the same shape
    // fp32_math_probe's set has, and the reason spirv_variants.hpp gives a
    // knobless kernel a set of ONE rather than a bare array: every consumer
    // walks the same structure, so no module can be scanned-by-accident-
    // omitted. for_size() rather than variants[0] because a null answer names
    // a real build-configuration defect, and it is cheaper to ask than to
    // assume the generator emitted what this file expects.
    // ------------------------------------------------------------------
    const compute::SpirvVariant* variant = compute::gen::kSpvVariants_raster_background.for_size(kLocalSizeX);
    if (variant == nullptr) {
        return std::unexpected(
            Error{Code::internal,
                  "BackgroundPass::create: kernel 'raster_background' has no SPIR-V variant compiled for local "
                  "size " +
                      std::to_string(kLocalSizeX) +
                      ". It must be registered with spade_slang_kernel() (a set of ONE at its literal "
                      "[numthreads] size), never spade_slang_kernel_variants()."});
    }

    VkShaderModuleCreateInfo module_info{};
    module_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    module_info.codeSize = variant->word_count * sizeof(uint32_t);
    module_info.pCode = variant->words;
    if (VkResult r = vkCreateShaderModule(device, &module_info, nullptr, &self->module_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateShaderModule (raster_background)"));
    }

    VkPipelineShaderStageCreateInfo stage{};
    stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stage.module = self->module_;
    // "main", NOT "raster_background". slangc's direct-to-SPIR-V compile names
    // the sole compute entry point "main" regardless of the Slang function's
    // name -- measured, and pinned per-module by test_slang_layouts.cpp.
    // Getting this wrong once already produced a pipeline the driver could not
    // run (S6 Task 5's SEH crash).
    stage.pName = "main";

    VkComputePipelineCreateInfo pipeline_info{};
    pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipeline_info.stage = stage;
    pipeline_info.layout = self->pipeline_layout_;
    if (VkResult r =
            vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &self->pipeline_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateComputePipelines (raster_background)"));
    }

    // ------------------------------------------------------------------
    // Command pool + one reusable buffer + one fence. RESET_COMMAND_BUFFER
    // because the recording is NOT fixed the way StepRecorder's is: the
    // dispatch grid and the copy extent both depend on the frame's size, so
    // this pass re-records per frame rather than recording once at create().
    // ------------------------------------------------------------------
    VkCommandPoolCreateInfo cmd_pool_info{};
    cmd_pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    cmd_pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    cmd_pool_info.queueFamilyIndex = ctx.compute_queue_family();
    if (VkResult r = vkCreateCommandPool(device, &cmd_pool_info, nullptr, &self->command_pool_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateCommandPool (BackgroundPass)"));
    }

    VkCommandBufferAllocateInfo cmd_alloc{};
    cmd_alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cmd_alloc.commandPool = self->command_pool_;
    cmd_alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmd_alloc.commandBufferCount = 1;
    if (VkResult r = vkAllocateCommandBuffers(device, &cmd_alloc, &self->command_buffer_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkAllocateCommandBuffers (BackgroundPass)"));
    }

    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (VkResult r = vkCreateFence(device, &fence_info, nullptr, &self->fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateFence (BackgroundPass)"));
    }

    self->plane_scratch_.reserve(self->plane_capacity_);
    return self;
}

// ---------------------------------------------------------------------------
// destructor -- reverse of create(), and idempotent on partial state, because
// a create() that failed halfway destroys `self` through exactly this path.
// Every vkDestroy* tolerates VK_NULL_HANDLE; vkUnmapMemory does not, so each
// unmap is guarded by its own mapped pointer rather than by the buffer handle.
// ---------------------------------------------------------------------------
BackgroundPass::~BackgroundPass() {
    if (ctx_ == nullptr) return;
    const VkDevice device = ctx_->device();
    if (device == VK_NULL_HANDLE) return;

    // The fence is the only thing that can still be in flight. Waiting on it
    // before tearing down is not paranoia: render() returns only after its own
    // wait, but a caller that destroyed this object from a different path would
    // otherwise free a buffer the queue still references.
    if (fence_ != VK_NULL_HANDLE) {
        vkWaitForFences(device, 1, &fence_, VK_TRUE, UINT64_MAX);
        vkDestroyFence(device, fence_, nullptr);
    }
    if (command_pool_ != VK_NULL_HANDLE) {
        // Frees command_buffer_ with it -- vkFreeCommandBuffers would be
        // redundant, and doing both is a double free on some drivers.
        vkDestroyCommandPool(device, command_pool_, nullptr);
    }

    vkDestroyPipeline(device, pipeline_, nullptr);
    vkDestroyShaderModule(device, module_, nullptr);
    vkDestroyPipelineLayout(device, pipeline_layout_, nullptr);
    vkDestroyDescriptorPool(device, descriptor_pool_, nullptr);  // frees descriptor_set_ with it
    vkDestroyDescriptorSetLayout(device, set_layout_, nullptr);
    vkDestroyDescriptorSetLayout(device, empty_set0_layout_, nullptr);

    if (staging_mapped_ != nullptr) vkUnmapMemory(device, staging_memory_);
    vkDestroyBuffer(device, staging_, nullptr);
    vkFreeMemory(device, staging_memory_, nullptr);

    if (params_mapped_ != nullptr) vkUnmapMemory(device, params_memory_);
    vkDestroyBuffer(device, params_, nullptr);
    vkFreeMemory(device, params_memory_, nullptr);

    if (planes_mapped_ != nullptr) vkUnmapMemory(device, planes_memory_);
    vkDestroyBuffer(device, planes_, nullptr);
    vkFreeMemory(device, planes_memory_, nullptr);

    vkDestroyBuffer(device, pixels_, nullptr);
    vkFreeMemory(device, pixels_memory_, nullptr);
}

// ---------------------------------------------------------------------------
// render
// ---------------------------------------------------------------------------

Result<void> BackgroundPass::render(const RenderScene& scene, const Camera& camera,
                                    const RenderOptions& options, RenderTarget& target) {
    if (target.width == 0 || target.height == 0) {
        return std::unexpected(Error{Code::invalid_argument, "BackgroundPass::render: target has a zero dimension"});
    }
    if (target.width > max_width_ || target.height > max_height_) {
        return std::unexpected(
            Error{Code::invalid_argument, "BackgroundPass::render: target " + std::to_string(target.width) + "x" +
                                     std::to_string(target.height) + " exceeds the create() bounds " +
                                     std::to_string(max_width_) + "x" + std::to_string(max_height_) +
                                     ". This path REFUSES rather than reallocating, so that a frame's cost "
                                     "never depends on the frames before it."});
    }
    // ⛔ THE FORMAT IS CHECKED EVEN THOUGH THE ENUM HAS ONE VALUE. The kernel's
    // pack_bgrx() writes B,G,R,X because that is what PixelFormat::bgrx8 means
    // and what raster_cpu.cpp writes; if a second format is ever added, this
    // line is what makes that a refusal instead of a silently wrong frame.
    if (target.format != PixelFormat::bgrx8) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "BackgroundPass::render: only PixelFormat::bgrx8 is supported; the kernel "
                                     "packs B,G,R,X to match raster_cpu.cpp's own pixel writes."});
    }
    if (target.stride < target.width * kBytesPerTexel) {
        return std::unexpected(Error{Code::invalid_argument, "BackgroundPass::render: stride " +
                                                       std::to_string(target.stride) +
                                                       " is narrower than width*4 = " +
                                                       std::to_string(target.width * kBytesPerTexel)});
    }
    if (target.stride % kBytesPerTexel != 0) {
        return std::unexpected(
            Error{Code::invalid_argument, "BackgroundPass::render: stride " + std::to_string(target.stride) +
                                     " is not a whole number of 4-byte texels; the kernel addresses the "
                                     "destination in TEXELS (row_pitch_texels), not bytes."});
    }
    if (target.pixels.size() < static_cast<std::size_t>(target.stride) * target.height) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "BackgroundPass::render: target.pixels holds " +
                                         std::to_string(target.pixels.size()) + " bytes, fewer than stride*height = " +
                                         std::to_string(static_cast<std::size_t>(target.stride) * target.height)});
    }

    const uint32_t row_pitch_texels = target.stride / kBytesPerTexel;
    // A wide stride can push the last row past a buffer sized for
    // max_width*max_height even when width and height both fit -- the kernel
    // writes at py*row_pitch + px, not py*width + px. Checked against the
    // PITCH, which is the quantity the kernel actually uses.
    if (static_cast<uint64_t>(row_pitch_texels) * target.height >
        static_cast<uint64_t>(max_width_) * static_cast<uint64_t>(max_height_)) {
        return std::unexpected(
            Error{Code::invalid_argument, "BackgroundPass::render: stride " + std::to_string(target.stride) +
                                     " over " + std::to_string(target.height) +
                                     " rows addresses more texels than the device buffer holds (sized for " +
                                     std::to_string(max_width_) + "x" + std::to_string(max_height_) + ")."});
    }

    // ------------------------------------------------------------------
    // The per-plane precompute. EVERY per-plane quantity the kernel needs is a
    // function of the camera, the lighting and the plane -- none of them varies
    // per pixel -- so all of it happens once here. That is not an optimisation
    // added on top: raster_cpu.cpp hoists the identical front[] test out of its
    // own pixel loop for the same reason, and its comment says so.
    //
    // `ground_possible` reproduces raster_cpu.cpp's own three-part gate
    // exactly: the analytic ground is DrawMode::shaded only (ruling SR-22),
    // needs at least one plane, and needs a material table to resolve against.
    // A zero plane_count leaves the kernel drawing sky alone, which is what the
    // CPU path does in the same situation.
    // ------------------------------------------------------------------
    const bool shaded = options.mode == DrawMode::shaded;
    const bool ground_possible = shaded && !scene.ground_planes.empty() && !scene.materials.empty();

    plane_scratch_.clear();
    if (ground_possible) {
        for (const GroundPlane& gp : scene.ground_planes) {
            const float n_dot_cam = glm::dot(gp.normal, camera.position);
            // SR-13 parity: the camera must be STRICTLY on the side the normal
            // points to, matching world/sdf.hpp's "dot(p,n) <= offset is
            // solid". A plane the camera sits at or below never contributes a
            // hit, the same way the tessellated ground vanishes from below.
            // Uploaded as a flag rather than re-derived on the device because
            // it is a per-frame fact about one plane, not a per-pixel one.
            PlaneGpu out{};
            store_vec3(out.normal, gp.normal);
            out.offset = gp.offset;
            const uint32_t material_index = gp.material < scene.materials.size() ? gp.material : 0u;
            const Material& mat = scene.materials[material_index];
            store_vec3(out.base_color, glm::vec3(mat.base_color));
            out.is_front = (n_dot_cam > gp.offset) ? 1u : 0u;
            // scene.hpp's own basis function, called HERE rather than
            // reimplemented in Slang: its tie-breaking (the smallest-component
            // axis choice) is arbitrary but must be IDENTICAL on both paths, and
            // an arbitrary choice reimplemented is an arbitrary choice made
            // twice. The kernel receives u and v and never derives them.
            glm::vec3 u{}, v{};
            ground_plane_basis(gp.normal, u, v);
            store_vec3(out.grid_u, u);
            store_vec3(out.grid_v, v);
            // Material::shading, in the slot that used to be pad0. The kernel
            // echoes base_color verbatim for 1 (unlit) and 2 (emissive), which
            // is shade_vertex_color()'s own early return rather than an
            // approximation of it.
            out.shading_mode = mat.shading;
            out.pad1 = 0.0f;
            plane_scratch_.push_back(out);
        }
    }

    if (plane_scratch_.size() > plane_capacity_) {
        return std::unexpected(
            Error{Code::capacity_exceeded,
                  "BackgroundPass::render: the scene has " + std::to_string(plane_scratch_.size()) +
                      " ground planes and this pass was sized for " + std::to_string(plane_capacity_) +
                      ". The plane buffer is allocated once at create() and never grown, for the same "
                      "reason the pixel buffer is not."});
    }
    if (!plane_scratch_.empty()) {
        std::memcpy(planes_mapped_, plane_scratch_.data(), plane_scratch_.size() * sizeof(PlaneGpu));
    }

    // ------------------------------------------------------------------
    // The parameter block. FIELD ORDER HERE IS THE CONTRACT -- see the Params
    // declaration's own note: nothing compares this struct to the kernel's
    // BackgroundParams, so the pairing of each float3 with the scalar that
    // follows it is what keeps std140 and std430 agreeing about every offset.
    // Assigned field-by-field in declaration order rather than brace-
    // initialized, so a reader can check this block against the kernel's
    // struct line for line.
    // ------------------------------------------------------------------
    Params p{};

    // The camera basis, rotated by the camera's local->world orientation. The
    // CPU path reaches the same three vectors the long way round (it stores the
    // world-to-camera CONJUGATE in its ViewContext and conjugates it back), and
    // quatConjugate is its own inverse, so this is the same rotation, not an
    // approximation of it.
    //
    // ⚠ THIS IS THE ENGINE'S FIRST USE OF glm's `quat * vec3`, AND IT DIFFERS
    // FROM raster_cpu.cpp's rotateByQuat() BY ONE ASSOCIATION -- stated precisely
    // because a reader comparing the two paths will find a gap slightly larger
    // than fp32-vs-fp64 alone explains, and should not go looking for a bug:
    //   rotateByQuat:  v + (uv * (2*w)) + (uuv * 2)     ->  (v + X) + Y
    //   glm:           v + ((uv * w) + uuv) * 2         ->  v + (X + Y)
    // Scaling by two is exact in IEEE 754, so X and Y are bit-identical between
    // them; only the FINAL SUM is grouped differently, and addition is not
    // associative in floating point. One rounding, at most one ulp, and it is
    // inside a path that is banded by construction rather than bit-pinned.
    // ⛔ SO IT IS NOT A DEFECT HERE -- but it WOULD BE in any future code that
    // claims bit-identity with the CPU camera, and that claim is exactly what
    // someone will try to make next.
    const glm::vec3 right_world = camera.orientation * glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 up_world = camera.orientation * glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 forward_world = camera.orientation * glm::vec3(0.0f, 0.0f, -1.0f);

    store_vec3(p.cam_pos, camera.position);
    // scene.hpp's SHARED tan32(), never std::tan: it is sin32/cos32 built, so
    // this `f` is bit-identical to the one raster_cpu.cpp and raymarch.cpp
    // compute for the same fov. Using std::tan here would make the two paths'
    // camera rays disagree by a ulp for no reason, and that disagreement would
    // be indistinguishable from the fp32/fp64 gap this path already carries.
    p.inv_tan_half_fov = static_cast<float>(1.0 / tan32(static_cast<double>(camera.fov_y_radians) * 0.5));
    store_vec3(p.right_world, right_world);
    p.aspect = static_cast<float>(target.width) / static_cast<float>(target.height);
    store_vec3(p.up_world, up_world);
    p.width = target.width;
    store_vec3(p.forward_world, forward_world);
    p.height = target.height;

    store_vec3(p.sun_direction, scene.lighting.sun_direction);
    p.sun_intensity = scene.lighting.sun_intensity;
    store_vec3(p.sun_color, scene.lighting.sun_color);
    // Grid and horizon are SHADED-ONLY, distilled at raster_cpu.cpp's own call
    // site (ruling SR-22): a grid painted on a surface that is not drawn would
    // be the same mode-contract violation as the surface itself.
    p.horizon_strength = shaded ? options.horizon_blend_strength : 0.0f;
    store_vec3(p.ambient_color, scene.lighting.ambient_color);
    p.horizon_onset = options.horizon_blend_onset;
    store_vec3(p.sky_zenith, scene.lighting.sky_zenith);
    p.plane_count = static_cast<uint32_t>(plane_scratch_.size());
    store_vec3(p.sky_horizon, scene.lighting.sky_horizon);
    p.row_pitch_texels = row_pitch_texels;

    const GroundGridParams& grid = options.ground_grid_params;
    store_vec3(p.grid_color, grid.color);
    p.grid_spacing = grid.spacing;
    p.grid_line_half_width = grid.line_half_width;
    p.grid_width_growth = grid.width_growth;
    p.grid_fade_distance = grid.fade_distance;
    p.grid_enabled = (shaded && options.ground_grid) ? 1u : 0u;

    std::memcpy(params_mapped_, &p, sizeof(p));

    // ------------------------------------------------------------------
    // Record, submit, wait, copy out. ONE dispatch, ONE readback, and no
    // frame-to-frame device state that can affect a pixel -- the buffers
    // persist but every byte the kernel reads was written by this call.
    // ------------------------------------------------------------------
    const VkDevice device = ctx_->device();

    if (VkResult r = vkResetCommandBuffer(command_buffer_, 0); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkResetCommandBuffer (BackgroundPass)"));
    }

    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (VkResult r = vkBeginCommandBuffer(command_buffer_, &begin); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkBeginCommandBuffer (BackgroundPass)"));
    }

    vkCmdBindPipeline(command_buffer_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
    // firstSet = 1. The empty set 0 is described by the layout and never bound,
    // which is legal precisely because the pipeline uses no descriptor from it.
    vkCmdBindDescriptorSets(command_buffer_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_, kRenderSet, 1,
                            &descriptor_set_, 0, nullptr);

    const uint32_t pixel_count = target.width * target.height;
    const uint32_t group_count = (pixel_count + kLocalSizeX - 1) / kLocalSizeX;
    vkCmdDispatch(command_buffer_, group_count, 1, 1);

    // SHADER_WRITE -> TRANSFER_READ on the pixel buffer. Without this the copy
    // may read bytes the dispatch has not finished writing, and on an
    // integrated GPU it would usually appear to work -- which is the failure
    // mode worth naming, because "usually works" is what makes a missing
    // barrier survive review.
    VkBufferMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.buffer = pixels_;
    barrier.offset = 0;
    barrier.size = VK_WHOLE_SIZE;
    vkCmdPipelineBarrier(command_buffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 1, &barrier, 0, nullptr);

    // Copies exactly the bytes the frame occupies -- stride*height -- not the
    // whole allocation. The allocation is sized for the create() bound and a
    // smaller frame must not pay for the difference.
    const VkDeviceSize frame_bytes = static_cast<VkDeviceSize>(target.stride) * target.height;
    VkBufferCopy copy{};
    copy.srcOffset = 0;
    copy.dstOffset = 0;
    copy.size = frame_bytes;
    vkCmdCopyBuffer(command_buffer_, pixels_, staging_, 1, &copy);

    if (VkResult r = vkEndCommandBuffer(command_buffer_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkEndCommandBuffer (BackgroundPass)"));
    }

    if (VkResult r = vkResetFences(device, 1, &fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkResetFences (BackgroundPass)"));
    }

    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &command_buffer_;
    if (VkResult r = vkQueueSubmit(ctx_->compute_queue(), 1, &submit, fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkQueueSubmit (BackgroundPass)"));
    }

    // ⛔ DO NOT TIME THIS WAIT FROM HERE, and the reason is a guard rather than
    // a preference: M1B.FixedStepNoWallClockSymbolsInEngineSource scans engine
    // source for wall-clock SYMBOLS and cannot tell a diagnostic timer from a
    // timestep input -- which is the point of it. Measure the stall from the
    // GPU timeline or from the caller. tests/bench/bench_render.cpp is the
    // caller that already does.
    if (VkResult r = vkWaitForFences(device, 1, &fence_, VK_TRUE, UINT64_MAX); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkWaitForFences (BackgroundPass)"));
    }

    // HOST_COHERENT staging, so no vkInvalidateMappedMemoryRanges -- see the
    // allocation's own note for why coherence was required rather than
    // remembered.
    std::memcpy(target.pixels.data(), staging_mapped_, static_cast<std::size_t>(frame_bytes));
    return {};
}

}  // namespace spade::render
