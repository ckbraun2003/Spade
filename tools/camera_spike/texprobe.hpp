#pragma once
// texprobe.hpp -- the texture filter on its own on the GPU (texprobe.slang), for the CPU-against-GPU check on a dense (u, v) grid (V11).
// Bench code.

#include <fstream>
#include <string>
#include <vector>

#include "texpool.hpp"
#include "vkh.hpp"

namespace cs {

// Runs `n` samples (u, v, footprint, texture index as float bits) through the shader's filter; returns the packed RGB words.
inline std::vector<uint32_t> run_tex_probe(Device& d, const TexPool& pool, const std::vector<float>& samples, const std::string& spv_dir) {
    const uint32_t n = static_cast<uint32_t>(samples.size() / 4);
    const VkMemoryPropertyFlags dev_local = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
    const VkMemoryPropertyFlags host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    const VkBufferUsageFlags sto = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    auto upload = [&](Buffer& dst, const void* data, size_t bytes) {
        Buffer stage = make_buffer(d, bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host);
        std::memcpy(stage.map, data, bytes);
        VkCommandBuffer cb = alloc_cmd(d);
        begin_cmd(cb);
        VkBufferCopy bc{0, 0, bytes};
        vkCmdCopyBuffer(cb, stage.buf, dst.buf, 1, &bc);
        submit_wait(d, cb);
        vkFreeCommandBuffers(d.dev, d.pool, 1, &cb);
        destroy_buffer(d, stage);
    };
    Buffer tex = make_buffer(d, pool.words.size() * 4, sto, dev_local);
    upload(tex, pool.words.data(), pool.words.size() * 4);
    Buffer tab = make_buffer(d, pool.table.size() * 4, sto, dev_local);
    upload(tab, pool.table.data(), pool.table.size() * 4);
    Buffer in = make_buffer(d, samples.size() * 4, sto, dev_local);
    upload(in, samples.data(), samples.size() * 4);
    Buffer out = make_buffer(d, static_cast<VkDeviceSize>(n) * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, dev_local);
    Buffer rb = make_buffer(d, static_cast<VkDeviceSize>(n) * 4, VK_BUFFER_USAGE_TRANSFER_DST_BIT, host);

    std::ifstream f(spv_dir + "/texprobe.spv", std::ios::binary);
    if (!f) die("cannot read " + spv_dir + "/texprobe.spv");
    std::vector<char> spv((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    VkShaderModuleCreateInfo mi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    mi.codeSize = spv.size();
    mi.pCode = reinterpret_cast<const uint32_t*>(spv.data());
    VkShaderModule module = VK_NULL_HANDLE;
    CS_VK(vkCreateShaderModule(d.dev, &mi, nullptr, &module));
    VkDescriptorSetLayoutBinding bs[4];
    for (uint32_t i = 0; i < 4; ++i) bs[i] = {i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    VkDescriptorSetLayoutCreateInfo li{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    li.bindingCount = 4;
    li.pBindings = bs;
    VkDescriptorSetLayout dsl = VK_NULL_HANDLE;
    CS_VK(vkCreateDescriptorSetLayout(d.dev, &li, nullptr, &dsl));
    VkPushConstantRange pc{VK_SHADER_STAGE_COMPUTE_BIT, 0, 16};
    VkPipelineLayoutCreateInfo pl{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pl.setLayoutCount = 1;
    pl.pSetLayouts = &dsl;
    pl.pushConstantRangeCount = 1;
    pl.pPushConstantRanges = &pc;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    CS_VK(vkCreatePipelineLayout(d.dev, &pl, nullptr, &layout));
    VkComputePipelineCreateInfo ci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    ci.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_COMPUTE_BIT, module, "main", nullptr};
    ci.layout = layout;
    VkPipeline pipe = VK_NULL_HANDLE;
    CS_VK(vkCreateComputePipelines(d.dev, VK_NULL_HANDLE, 1, &ci, nullptr, &pipe));
    VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4};
    VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    dpi.maxSets = 1;
    dpi.poolSizeCount = 1;
    dpi.pPoolSizes = &ps;
    VkDescriptorPool dpool = VK_NULL_HANDLE;
    CS_VK(vkCreateDescriptorPool(d.dev, &dpi, nullptr, &dpool));
    VkDescriptorSetAllocateInfo dai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    dai.descriptorPool = dpool;
    dai.descriptorSetCount = 1;
    dai.pSetLayouts = &dsl;
    VkDescriptorSet dset = VK_NULL_HANDLE;
    CS_VK(vkAllocateDescriptorSets(d.dev, &dai, &dset));
    const Buffer* bufs[4] = {&tex, &tab, &in, &out};
    VkDescriptorBufferInfo infos[4];
    VkWriteDescriptorSet ws[4];
    for (uint32_t i = 0; i < 4; ++i) {
        infos[i] = {bufs[i]->buf, 0, bufs[i]->size};
        ws[i] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        ws[i].dstSet = dset;
        ws[i].dstBinding = i;
        ws[i].descriptorCount = 1;
        ws[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        ws[i].pBufferInfo = &infos[i];
    }
    vkUpdateDescriptorSets(d.dev, 4, ws, 0, nullptr);

    VkCommandBuffer cb = alloc_cmd(d);
    begin_cmd(cb);
    vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, pipe);
    vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_COMPUTE, layout, 0, 1, &dset, 0, nullptr);
    const uint32_t params[4] = {n, 0, 0, 0};
    vkCmdPushConstants(cb, layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(params), params);
    vkCmdDispatch(cb, (n + 63) / 64, 1, 1);
    memory_barrier(cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_READ_BIT);
    VkBufferCopy rc{0, 0, static_cast<VkDeviceSize>(n) * 4};
    vkCmdCopyBuffer(cb, out.buf, rb.buf, 1, &rc);
    submit_wait(d, cb);
    vkFreeCommandBuffers(d.dev, d.pool, 1, &cb);
    std::vector<uint32_t> result(n);
    std::memcpy(result.data(), rb.map, static_cast<size_t>(n) * 4);

    vkDestroyPipeline(d.dev, pipe, nullptr);
    vkDestroyPipelineLayout(d.dev, layout, nullptr);
    vkDestroyDescriptorPool(d.dev, dpool, nullptr);
    vkDestroyDescriptorSetLayout(d.dev, dsl, nullptr);
    vkDestroyShaderModule(d.dev, module, nullptr);
    for (Buffer* b : {&tex, &tab, &in, &out, &rb}) destroy_buffer(d, *b);
    return result;
}

}  // namespace cs
