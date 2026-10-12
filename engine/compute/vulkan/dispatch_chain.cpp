#include "compute/vulkan/dispatch_chain.hpp"

#include <cassert>
#include <string>
#include <string_view>

namespace spade::compute {

namespace {

// Same taxonomy as step_recorder.cpp's and state_mirror.cpp's map_vk_error,
// duplicated for the reason step_recorder.cpp gives.
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

}  // namespace

ChainCounts record_dispatch_chain(VkCommandBuffer cmd, VkPipelineLayout layout, std::span<const VkPipeline> pipelines,
                                  std::span<const ChainCommand> commands, uint32_t push_offset, uint32_t push_size,
                                  const ChainMark& mark) {
    assert(push_offset + push_size <= kMaxChainPushBytes && "a push must fit the command's block");
    std::size_t last_dispatch = 0;
    for (std::size_t i = 0; i < commands.size(); ++i) {
        if (commands[i].is_dispatch) last_dispatch = i;
    }
    ChainCounts counts;
    for (std::size_t i = 0; i < commands.size(); ++i) {
        const ChainCommand& c = commands[i];
        if (!c.is_dispatch) {
            mark(cmd, c.mark_a, c.mark_b);
            continue;
        }
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelines[c.pipeline]);
        vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_COMPUTE_BIT, push_offset, push_size, c.push.data());
        vkCmdDispatch(cmd, c.groups[0], c.groups[1], c.groups[2]);
        ++counts.dispatches;
        if (i < last_dispatch) {
            VkMemoryBarrier barrier{};
            barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
            barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
            vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                 &barrier, 0, nullptr, 0, nullptr);
            ++counts.barriers;
        }
    }
    return counts;
}

Result<void> make_compute_pipeline(VkDevice device, VkPipelineLayout layout, const SpirvVariantSet& variants,
                                   uint32_t workgroup_size, const char* who, VkShaderModule& out_module,
                                   VkPipeline& out_pipeline) {
    // THE SELECTION (S6 Task 9b). A null answer means the build did not compile
    // this kernel for the size create() accepted -- i.e.
    // cmake/SpadeSlang.cmake's SPADE_SLANG_WORKGROUP_SIZES and
    // compute/backend.hpp's kSupportedWorkgroupSizes have drifted apart. That is
    // a build-configuration defect rather than a caller error, and it must be a
    // loud one HERE, because the alternative -- falling back to some other
    // variant -- would dispatch a module on a grid divided by a different
    // number and produce wrong physics silently.
    const SpirvVariant* variant = variants.for_size(workgroup_size);
    if (variant == nullptr) {
        return std::unexpected(Error{
            Code::invalid_argument,
            std::string(who) + ": kernel '" + variants.name + "' has no variant compiled for local size " +
                std::to_string(workgroup_size) +
                ". cmake/SpadeSlang.cmake's SPADE_SLANG_WORKGROUP_SIZES must list every size "
                "compute/backend.hpp's kSupportedWorkgroupSizes admits."});
    }

    VkShaderModuleCreateInfo shader_info{};
    shader_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    shader_info.codeSize = variant->word_count * sizeof(uint32_t);
    shader_info.pCode = variant->words;
    if (VkResult r = vkCreateShaderModule(device, &shader_info, nullptr, &out_module); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, std::string("vkCreateShaderModule (") + who + ")"));
    }

    VkPipelineShaderStageCreateInfo stage_info{};
    stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage_info.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stage_info.module = out_module;
    // Entry point name: "main", regardless of what the Slang function is
    // called. Verified against every embedded module's own OpEntryPoint
    // literal (tests/test_slang_layouts.cpp asserts it for each): slangc's
    // direct-to-SPIR-V compile names the sole compute entry point "main", so
    // vkCreateComputePipelines's pName must match THAT. Getting this wrong
    // once already produced a pipeline the driver could not run (S6 Task 5's
    // own SEH-crash regression, round 1).
    stage_info.pName = "main";

    VkComputePipelineCreateInfo pipeline_info{};
    pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipeline_info.stage = stage_info;
    pipeline_info.layout = layout;
    if (VkResult r = vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &out_pipeline);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateComputePipelines"));
    }
    return {};
}

}  // namespace spade::compute
