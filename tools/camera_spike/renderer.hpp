#pragma once
// renderer.hpp -- the spike's GPU side: the scene's acceleration structures (the lattice TLAS "L" or one TLAS an env "E"), the kernel's
// pipeline, and a frame loop with GPU timestamps. Bench code.

#include <chrono>
#include <fstream>
#include <string>
#include <vector>

#include "scene.hpp"
#include "vkh.hpp"

namespace cs {

struct Setup {
    Config cfg;
    uint32_t envs = 128;
    uint32_t aa = 1;
    uint32_t blur = 1;
    bool lattice = true;  // L (one TLAS, envs on a lattice) or E (one TLAS an env)
    bool dynamic = false; // the mesh instance turns every frame and the TLAS is updated
    bool exportable = false;
    bool depth = false;
    std::string spv_dir;
};

struct Params {
    uint32_t W, H, aa, blur;
    uint32_t envs, side, mode, grey;
    float cell, tmax;
    uint32_t debug_depth, pad;
};
static_assert(sizeof(Params) == 48);

struct FrameTimes {
    double wall_ms = 0, kernel_ms = 0, upload_ms = 0, as_ms = 0, pose_cpu_ms = 0, inst_cpu_ms = 0;
};

inline double ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

class Renderer {
public:
    Renderer(Device& d, const Setup& s) : d_(d), s_(s) {
        const auto t0 = std::chrono::steady_clock::now();
        side_ = 1;
        while (side_ * side_ < s.envs) ++side_;
        planes_ = s.cfg.grey ? 1u : 3u;
        n_tlas_ = s.lattice ? 1u : s.envs;
        mesh_ = make_torus();
        vram_start_ = heap_usage(d_);
        make_resources();
        build_structures();
        make_pipeline();
        vram_end_ = heap_usage(d_);
        setup_ms_ = ms_since(t0);
    }
    ~Renderer() {
        vkDeviceWaitIdle(d_.dev);
        vkDestroyPipeline(d_.dev, pipe_, nullptr);
        vkDestroyPipelineLayout(d_.dev, layout_, nullptr);
        vkDestroyDescriptorPool(d_.dev, dpool_, nullptr);
        vkDestroyDescriptorSetLayout(d_.dev, dsl_, nullptr);
        vkDestroyShaderModule(d_.dev, module_, nullptr);
        vkDestroyQueryPool(d_.dev, qpool_, nullptr);
        vkDestroySemaphore(d_.dev, sem_, nullptr);
        for (auto a : tlas_) vkDestroyAccelerationStructureKHR(d_.dev, a, nullptr);
        vkDestroyAccelerationStructureKHR(d_.dev, blas_gate_, nullptr);
        vkDestroyAccelerationStructureKHR(d_.dev, blas_mesh_, nullptr);
        for (Buffer* b : {&table_, &poses_, &poses_stage_, &out_, &readback_, &depth_, &depth_read_, &verts_, &idx_, &srgb_, &inst_, &aabb_, &arena_, &scratch_})
            destroy_buffer(d_, *b);
    }
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    // Render one frame (index f at 30 Hz). Returns the times; with readback the output is copied to the host (image(), ids(), depth()).
    FrameTimes render(uint32_t f, bool readback) {
        FrameTimes ft;
        const auto t0 = std::chrono::steady_clock::now();
        // 1. poses on the CPU
        {
            const auto tp = std::chrono::steady_clock::now();
            auto* dst = static_cast<float*>(poses_stage_.map);
            for (uint32_t e = 0; e < s_.envs; ++e)
                for (uint32_t k = 0; k < s_.blur; ++k) {
                    const Pose p = camera_pose(e, shutter_time(f, k, s_.blur));
                    float* o = dst + (static_cast<size_t>(e) * s_.blur + k) * 8;
                    o[0] = p.pos[0]; o[1] = p.pos[1]; o[2] = p.pos[2]; o[3] = 0.0f;
                    o[4] = p.quat[0]; o[5] = p.quat[1]; o[6] = p.quat[2]; o[7] = p.quat[3];
                }
            ft.pose_cpu_ms = ms_since(tp);
        }
        // 2. the moving mesh instance, when dynamic
        if (s_.dynamic) {
            const auto ti = std::chrono::steady_clock::now();
            auto* arr = static_cast<VkAccelerationStructureInstanceKHR*>(inst_.map);
            for (uint32_t e = 0; e < s_.envs; ++e) {
                const EnvScene sc = make_env(e, f, true);
                fill_instance(arr[e * kInstancesPerEnv + 4], sc.inst[4], e, 1);
            }
            ft.inst_cpu_ms = ms_since(ti);
        }
        // 3. record and submit
        VkCommandBuffer cb = cmd_;
        CS_VK(vkResetCommandBuffer(cb, 0));
        begin_cmd(cb);
        vkCmdResetQueryPool(cb, qpool_, 0, 4);
        vkCmdWriteTimestamp(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, qpool_, 0);
        VkBufferCopy bc{0, 0, static_cast<VkDeviceSize>(s_.envs) * s_.blur * 32};
        vkCmdCopyBuffer(cb, poses_stage_.buf, poses_.buf, 1, &bc);
        memory_barrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT);
        vkCmdWriteTimestamp(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, qpool_, 1);
        if (s_.dynamic) {
            record_tlas_builds(cb, true);
            memory_barrier(cb, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR,
                           VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR);
        }
        vkCmdWriteTimestamp(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, qpool_, 2);
        vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, pipe_);
        vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_COMPUTE, layout_, 0, 1, &dset_, 0, nullptr);
        Params p{s_.cfg.W, s_.cfg.H, s_.aa, s_.blur, s_.envs, side_, s_.lattice ? 0u : 1u, s_.cfg.grey ? 1u : 0u,
                 static_cast<float>(kLatticeCell), static_cast<float>(kTmax), s_.depth ? 1u : 0u, 0u};
        vkCmdPushConstants(cb, layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(p), &p);
        vkCmdDispatch(cb, (s_.cfg.W + 7) / 8, (s_.cfg.H + 7) / 8, s_.envs);
        vkCmdWriteTimestamp(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, qpool_, 3);
        if (readback) {
            memory_barrier(cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_READ_BIT);
            VkBufferCopy rc{0, 0, out_.size};
            vkCmdCopyBuffer(cb, out_.buf, readback_.buf, 1, &rc);
            if (s_.depth) {
                VkBufferCopy dc{0, 0, depth_.size};
                vkCmdCopyBuffer(cb, depth_.buf, depth_read_.buf, 1, &dc);
            }
        }
        CS_VK(vkEndCommandBuffer(cb));
        ++counter_;
        VkTimelineSemaphoreSubmitInfo ts{VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO};
        ts.signalSemaphoreValueCount = 1;
        ts.pSignalSemaphoreValues = &counter_;
        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.pNext = &ts;
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cb;
        si.signalSemaphoreCount = 1;
        si.pSignalSemaphores = &sem_;
        CS_VK(vkQueueSubmit(d_.queue, 1, &si, VK_NULL_HANDLE));
        VkSemaphoreWaitInfo wi{VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO};
        wi.semaphoreCount = 1;
        wi.pSemaphores = &sem_;
        wi.pValues = &counter_;
        CS_VK(vkWaitSemaphores(d_.dev, &wi, ~0ull));
        ft.wall_ms = ms_since(t0);
        uint64_t ts4[4]{};
        CS_VK(vkGetQueryPoolResults(d_.dev, qpool_, 0, 4, sizeof(ts4), ts4, 8, VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT));
        const double ns = d_.ts_period_ns;
        ft.upload_ms = static_cast<double>(ts4[1] - ts4[0]) * ns * 1e-6;
        ft.as_ms = static_cast<double>(ts4[2] - ts4[1]) * ns * 1e-6;
        ft.kernel_ms = static_cast<double>(ts4[3] - ts4[2]) * ns * 1e-6;
        if (readback) {
            const auto* src = static_cast<const uint8_t*>(readback_.map);
            image_.assign(src, src + img_bytes_);
            ids_.resize(static_cast<size_t>(s_.envs) * s_.cfg.H * s_.cfg.W);
            std::memcpy(ids_.data(), src + ids_off_, ids_.size() * 2);
            if (s_.depth) {
                depth_host_.resize(static_cast<size_t>(s_.envs) * s_.cfg.H * s_.cfg.W);
                std::memcpy(depth_host_.data(), depth_read_.map, depth_host_.size() * 4);
            }
        }
        return ft;
    }

    const std::vector<uint8_t>& image() const { return image_; }
    const std::vector<uint16_t>& ids() const { return ids_; }
    const std::vector<float>& depth() const { return depth_host_; }
    double setup_ms() const { return setup_ms_; }
    double build_ms() const { return build_ms_; }
    double build_gpu_ms() const { return build_gpu_ms_; }
    uint64_t structure_bytes() const { return as_bytes_; }
    uint64_t scratch_bytes() const { return scratch_.size; }
    uint64_t out_bytes() const { return out_.size; }
    uint64_t vram_used() const { return vram_end_ > vram_start_ ? vram_end_ - vram_start_ : 0; }
    uint64_t img_bytes() const { return img_bytes_; }
    uint64_t ids_bytes() const { return ids_bytes_; }
    const Buffer& out_buffer() const { return out_; }
    VkSemaphore semaphore() const { return sem_; }
    uint64_t signalled() const { return counter_; }
    uint32_t side() const { return side_; }
    uint64_t instances() const { return static_cast<uint64_t>(s_.envs) * kInstancesPerEnv; }
    const Setup& setup() const { return s_; }
    const Mesh& mesh() const { return mesh_; }

private:
    void fill_instance(VkAccelerationStructureInstanceKHR& in, const Placed& p, uint32_t env, uint32_t kind) {
        V3 t = p.t;
        if (s_.lattice) {
            t.x += static_cast<double>(env % side_) * kLatticeCell;
            t.z += static_cast<double>(env / side_) * kLatticeCell;
        }
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) in.transform.matrix[r][c] = static_cast<float>(p.R.m[r * 3 + c]);
        }
        in.transform.matrix[0][3] = static_cast<float>(t.x);
        in.transform.matrix[1][3] = static_cast<float>(t.y);
        in.transform.matrix[2][3] = static_cast<float>(t.z);
        in.instanceCustomIndex = (kind << 16) | p.label;
        in.mask = 0xFF;
        in.instanceShaderBindingTableRecordOffset = 0;
        in.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
        in.accelerationStructureReference = kind == 0 ? blas_gate_addr_ : blas_mesh_addr_;
    }

    VkAccelerationStructureKHR make_as(VkDeviceSize size, VkAccelerationStructureTypeKHR type, VkDeviceAddress* addr) {
        arena_used_ = (arena_used_ + 255) & ~static_cast<VkDeviceSize>(255);
        if (arena_used_ + size > arena_.size) die("the acceleration structure arena is too small");
        VkAccelerationStructureCreateInfoKHR ci{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
        ci.buffer = arena_.buf;
        ci.offset = arena_used_;
        ci.size = size;
        ci.type = type;
        VkAccelerationStructureKHR as = VK_NULL_HANDLE;
        CS_VK(vkCreateAccelerationStructureKHR(d_.dev, &ci, nullptr, &as));
        VkAccelerationStructureDeviceAddressInfoKHR ai{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR};
        ai.accelerationStructure = as;
        *addr = vkGetAccelerationStructureDeviceAddressKHR(d_.dev, &ai);
        arena_used_ += size;
        as_bytes_ += size;
        return as;
    }

    void make_resources() {
        const uint32_t W = s_.cfg.W, H = s_.cfg.H;
        const VkMemoryPropertyFlags dev_local = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
        const VkMemoryPropertyFlags host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
        const auto table = make_table(s_.cfg, s_.aa);
        {
            table_ = make_buffer(d_, table.size() * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, dev_local);
            upload(table_, table.data(), table.size() * 4);
        }
        {
            const auto srgb = make_srgb_table();
            srgb_ = make_buffer(d_, srgb.size() * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, dev_local);
            upload(srgb_, srgb.data(), srgb.size() * 4);
        }
        {
            const VkBufferUsageFlags asin = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                            VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
            verts_ = make_buffer(d_, mesh_.verts.size() * 4, asin, dev_local);
            upload(verts_, mesh_.verts.data(), mesh_.verts.size() * 4);
            idx_ = make_buffer(d_, mesh_.idx.size() * 4, asin, dev_local);
            upload(idx_, mesh_.idx.data(), mesh_.idx.size() * 4);
            const float aabb[6] = {-static_cast<float>(kOuter.x + 0.002), -static_cast<float>(kOuter.y + 0.002), -static_cast<float>(kOuter.z + 0.002),
                                   static_cast<float>(kOuter.x + 0.002),  static_cast<float>(kOuter.y + 0.002),  static_cast<float>(kOuter.z + 0.002)};
            aabb_ = make_buffer(d_, sizeof(aabb), asin, dev_local);
            upload(aabb_, aabb, sizeof(aabb));
        }
        poses_ = make_buffer(d_, static_cast<VkDeviceSize>(s_.envs) * s_.blur * 32, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, dev_local);
        poses_stage_ = make_buffer(d_, static_cast<VkDeviceSize>(s_.envs) * s_.blur * 32, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host);
        img_bytes_ = static_cast<uint64_t>(s_.envs) * planes_ * W * H;
        ids_bytes_ = static_cast<uint64_t>(s_.envs) * W * H * 2;
        ids_off_ = (img_bytes_ + 255) & ~uint64_t{255};
        out_ = make_buffer(d_, ids_off_ + ids_bytes_, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, dev_local,
                           s_.exportable);
        readback_ = make_buffer(d_, out_.size, VK_BUFFER_USAGE_TRANSFER_DST_BIT, host);
        const uint64_t depth_bytes = s_.depth ? static_cast<uint64_t>(s_.envs) * W * H * 4 : 16;
        depth_ = make_buffer(d_, depth_bytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, dev_local);
        depth_read_ = make_buffer(d_, depth_bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, host);
        inst_ = make_buffer(d_, static_cast<VkDeviceSize>(s_.envs) * kInstancesPerEnv * sizeof(VkAccelerationStructureInstanceKHR),
                            VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, host);

        VkQueryPoolCreateInfo qi{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
        qi.queryType = VK_QUERY_TYPE_TIMESTAMP;
        qi.queryCount = 4;
        CS_VK(vkCreateQueryPool(d_.dev, &qi, nullptr, &qpool_));
        VkSemaphoreTypeCreateInfo tci{VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO};
        tci.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
        tci.initialValue = 0;
        VkExportSemaphoreCreateInfo xci{VK_STRUCTURE_TYPE_EXPORT_SEMAPHORE_CREATE_INFO};
        VkSemaphoreCreateInfo sci{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
        sci.pNext = &tci;
#ifdef _WIN32
        if (s_.exportable) {
            xci.handleTypes = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT;
            tci.pNext = &xci;
        }
#else
        (void)xci;
#endif
        CS_VK(vkCreateSemaphore(d_.dev, &sci, nullptr, &sem_));
        cmd_ = alloc_cmd(d_);
    }

    void upload(Buffer& dst, const void* data, size_t bytes) {
        Buffer stage = make_buffer(d_, bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        std::memcpy(stage.map, data, bytes);
        VkCommandBuffer cb = alloc_cmd(d_);
        begin_cmd(cb);
        VkBufferCopy bc{0, 0, bytes};
        vkCmdCopyBuffer(cb, stage.buf, dst.buf, 1, &bc);
        submit_wait(d_, cb);
        vkFreeCommandBuffers(d_.dev, d_.pool, 1, &cb);
        destroy_buffer(d_, stage);
    }

    // Build the two BLAS once, then the TLAS (the build is what a reset costs; the update is what a moving instance costs per frame).
    void build_structures() {
        const auto t0 = std::chrono::steady_clock::now();
        // sizes
        VkAccelerationStructureGeometryKHR gg{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
        gg.geometryType = VK_GEOMETRY_TYPE_AABBS_KHR;
        gg.geometry.aabbs.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_AABBS_DATA_KHR;
        gg.geometry.aabbs.data.deviceAddress = aabb_.addr;
        gg.geometry.aabbs.stride = 24;
        VkAccelerationStructureGeometryKHR gm{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
        gm.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
        gm.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
        gm.geometry.triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
        gm.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
        gm.geometry.triangles.vertexData.deviceAddress = verts_.addr;
        gm.geometry.triangles.vertexStride = 16;
        gm.geometry.triangles.maxVertex = static_cast<uint32_t>(mesh_.verts.size() / 4 - 1);
        gm.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
        gm.geometry.triangles.indexData.deviceAddress = idx_.addr;
        const uint32_t mesh_prims = static_cast<uint32_t>(mesh_.idx.size() / 3);

        auto sizes_of = [&](const VkAccelerationStructureGeometryKHR& g, VkAccelerationStructureTypeKHR type, uint32_t prims, VkBuildAccelerationStructureFlagsKHR flags) {
            VkAccelerationStructureBuildGeometryInfoKHR bi{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
            bi.type = type;
            bi.flags = flags;
            bi.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
            bi.geometryCount = 1;
            bi.pGeometries = &g;
            VkAccelerationStructureBuildSizesInfoKHR sz{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};
            vkGetAccelerationStructureBuildSizesKHR(d_.dev, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &bi, &prims, &sz);
            return sz;
        };
        const VkBuildAccelerationStructureFlagsKHR blas_flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        VkBuildAccelerationStructureFlagsKHR tlas_flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        if (s_.dynamic) tlas_flags |= VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;

        VkAccelerationStructureGeometryKHR gi{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
        gi.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
        gi.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
        gi.geometry.instances.arrayOfPointers = VK_FALSE;
        gi.geometry.instances.data.deviceAddress = inst_.addr;
        const uint32_t tlas_prims = s_.lattice ? s_.envs * kInstancesPerEnv : kInstancesPerEnv;

        const auto sg = sizes_of(gg, VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR, 1, blas_flags);
        const auto sm = sizes_of(gm, VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR, mesh_prims, blas_flags);
        const auto st = sizes_of(gi, VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR, tlas_prims, tlas_flags);
        tlas_build_scratch_ = st.buildScratchSize;
        tlas_update_scratch_ = st.updateScratchSize;
        const VkDeviceSize per_scratch = (std::max(st.buildScratchSize, st.updateScratchSize) + d_.scratch_align - 1) / d_.scratch_align * d_.scratch_align;
        per_tlas_scratch_ = per_scratch;
        const VkDeviceSize arena_need = ((sg.accelerationStructureSize + 255) & ~255ull) + ((sm.accelerationStructureSize + 255) & ~255ull) +
                                        static_cast<VkDeviceSize>(n_tlas_) * ((st.accelerationStructureSize + 255) & ~255ull) + 4096;
        arena_ = make_buffer(d_, arena_need, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        const VkDeviceSize scratch_need = std::max<VkDeviceSize>(std::max(sg.buildScratchSize, sm.buildScratchSize) + d_.scratch_align, static_cast<VkDeviceSize>(n_tlas_) * per_scratch);
        scratch_ = make_buffer(d_, scratch_need + d_.scratch_align, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        scratch_base_ = (scratch_.addr + d_.scratch_align - 1) / d_.scratch_align * d_.scratch_align;

        blas_gate_ = make_as(sg.accelerationStructureSize, VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR, &blas_gate_addr_);
        blas_mesh_ = make_as(sm.accelerationStructureSize, VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR, &blas_mesh_addr_);
        tlas_.resize(n_tlas_);
        tlas_addr_.resize(n_tlas_);
        for (uint32_t i = 0; i < n_tlas_; ++i) tlas_[i] = make_as(st.accelerationStructureSize, VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR, &tlas_addr_[i]);

        // the instances (a reset's work)
        auto* arr = static_cast<VkAccelerationStructureInstanceKHR*>(inst_.map);
        for (uint32_t e = 0; e < s_.envs; ++e) {
            const EnvScene sc = make_env(e, 0, false);
            for (uint32_t i = 0; i < 4; ++i) fill_instance(arr[e * kInstancesPerEnv + i], sc.inst[i], e, 0);
            fill_instance(arr[e * kInstancesPerEnv + 4], sc.inst[4], e, 1);
        }

        // BLAS builds, then the TLAS builds, timed on the GPU
        VkQueryPoolCreateInfo qi{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
        qi.queryType = VK_QUERY_TYPE_TIMESTAMP;
        qi.queryCount = 2;
        VkQueryPool qp = VK_NULL_HANDLE;
        CS_VK(vkCreateQueryPool(d_.dev, &qi, nullptr, &qp));
        VkCommandBuffer cb = alloc_cmd(d_);
        begin_cmd(cb);
        vkCmdResetQueryPool(cb, qp, 0, 2);
        {
            VkAccelerationStructureBuildGeometryInfoKHR bg{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
            bg.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
            bg.flags = blas_flags;
            bg.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
            bg.geometryCount = 1;
            bg.pGeometries = &gg;
            bg.dstAccelerationStructure = blas_gate_;
            bg.scratchData.deviceAddress = scratch_base_;
            VkAccelerationStructureBuildRangeInfoKHR rg{1, 0, 0, 0};
            const VkAccelerationStructureBuildRangeInfoKHR* prg = &rg;
            vkCmdBuildAccelerationStructuresKHR(cb, 1, &bg, &prg);
            memory_barrier(cb, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR | VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR,
                           VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR | VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR);
            VkAccelerationStructureBuildGeometryInfoKHR bm = bg;
            bm.pGeometries = &gm;
            bm.dstAccelerationStructure = blas_mesh_;
            VkAccelerationStructureBuildRangeInfoKHR rm{mesh_prims, 0, 0, 0};
            const VkAccelerationStructureBuildRangeInfoKHR* prm = &rm;
            vkCmdBuildAccelerationStructuresKHR(cb, 1, &bm, &prm);
            memory_barrier(cb, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR | VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR,
                           VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR | VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR);
        }
        vkCmdWriteTimestamp(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, qp, 0);
        tlas_geom_ = gi;
        tlas_flags_ = tlas_flags;
        tlas_prims_ = tlas_prims;
        record_tlas_builds(cb, false);
        vkCmdWriteTimestamp(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, qp, 1);
        submit_wait(d_, cb);
        vkFreeCommandBuffers(d_.dev, d_.pool, 1, &cb);
        uint64_t ts[2]{};
        CS_VK(vkGetQueryPoolResults(d_.dev, qp, 0, 2, sizeof(ts), ts, 8, VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT));
        build_gpu_ms_ = static_cast<double>(ts[1] - ts[0]) * d_.ts_period_ns * 1e-6;
        vkDestroyQueryPool(d_.dev, qp, nullptr);
        build_ms_ = ms_since(t0);
    }

    // Record the TLAS builds (or updates) for every TLAS in one call.
    void record_tlas_builds(VkCommandBuffer cb, bool update) {
        std::vector<VkAccelerationStructureBuildGeometryInfoKHR> infos(n_tlas_);
        std::vector<VkAccelerationStructureGeometryKHR> geoms(n_tlas_, tlas_geom_);
        std::vector<VkAccelerationStructureBuildRangeInfoKHR> ranges(n_tlas_);
        std::vector<const VkAccelerationStructureBuildRangeInfoKHR*> pranges(n_tlas_);
        for (uint32_t i = 0; i < n_tlas_; ++i) {
            infos[i] = VkAccelerationStructureBuildGeometryInfoKHR{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
            infos[i].type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
            infos[i].flags = tlas_flags_;
            infos[i].mode = update ? VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR : VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
            infos[i].srcAccelerationStructure = update ? tlas_[i] : VK_NULL_HANDLE;
            infos[i].dstAccelerationStructure = tlas_[i];
            infos[i].geometryCount = 1;
            infos[i].pGeometries = &geoms[i];
            infos[i].scratchData.deviceAddress = scratch_base_ + static_cast<VkDeviceAddress>(i) * per_tlas_scratch_;
            ranges[i] = VkAccelerationStructureBuildRangeInfoKHR{tlas_prims_, s_.lattice ? 0u : i * kInstancesPerEnv * static_cast<uint32_t>(sizeof(VkAccelerationStructureInstanceKHR)), 0, 0};
            pranges[i] = &ranges[i];
        }
        vkCmdBuildAccelerationStructuresKHR(cb, n_tlas_, infos.data(), pranges.data());
    }

    void make_pipeline() {
        const char* name = s_.lattice ? "camera_spike_L.spv" : (s_.envs == 128 ? "camera_spike_E128.spv" : (s_.envs == 1024 ? "camera_spike_E1024.spv" : nullptr));
        if (name == nullptr) die("structure E is compiled for 128 and 1024 envs only");
        std::ifstream f(s_.spv_dir + "/" + name, std::ios::binary);
        if (!f) die("cannot read " + s_.spv_dir + "/" + name);
        std::vector<char> spv((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        VkShaderModuleCreateInfo mi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        mi.codeSize = spv.size();
        mi.pCode = reinterpret_cast<const uint32_t*>(spv.data());
        CS_VK(vkCreateShaderModule(d_.dev, &mi, nullptr, &module_));
        std::vector<VkDescriptorSetLayoutBinding> bs(9);
        for (uint32_t i = 0; i < 9; ++i) {
            bs[i] = {i, i == 0 ? VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, i == 0 ? n_tlas_ : 1u, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
        }
        VkDescriptorSetLayoutCreateInfo li{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        li.bindingCount = 9;
        li.pBindings = bs.data();
        CS_VK(vkCreateDescriptorSetLayout(d_.dev, &li, nullptr, &dsl_));
        VkPushConstantRange pc{VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Params)};
        VkPipelineLayoutCreateInfo pl{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        pl.setLayoutCount = 1;
        pl.pSetLayouts = &dsl_;
        pl.pushConstantRangeCount = 1;
        pl.pPushConstantRanges = &pc;
        CS_VK(vkCreatePipelineLayout(d_.dev, &pl, nullptr, &layout_));
        VkComputePipelineCreateInfo ci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        ci.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_COMPUTE_BIT, module_, "main", nullptr};
        ci.layout = layout_;
        CS_VK(vkCreateComputePipelines(d_.dev, VK_NULL_HANDLE, 1, &ci, nullptr, &pipe_));
        VkDescriptorPoolSize ps[2] = {{VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, n_tlas_}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 8}};
        VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        dpi.maxSets = 1;
        dpi.poolSizeCount = 2;
        dpi.pPoolSizes = ps;
        CS_VK(vkCreateDescriptorPool(d_.dev, &dpi, nullptr, &dpool_));
        VkDescriptorSetAllocateInfo dai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        dai.descriptorPool = dpool_;
        dai.descriptorSetCount = 1;
        dai.pSetLayouts = &dsl_;
        CS_VK(vkAllocateDescriptorSets(d_.dev, &dai, &dset_));
        VkWriteDescriptorSetAccelerationStructureKHR wa{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR};
        wa.accelerationStructureCount = n_tlas_;
        wa.pAccelerationStructures = tlas_.data();
        std::vector<VkWriteDescriptorSet> ws;
        VkWriteDescriptorSet w0{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        w0.pNext = &wa;
        w0.dstSet = dset_;
        w0.dstBinding = 0;
        w0.descriptorCount = n_tlas_;
        w0.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
        ws.push_back(w0);
        std::vector<VkDescriptorBufferInfo> infos;
        infos.reserve(8);
        auto add = [&](uint32_t binding, const Buffer& b, VkDeviceSize off, VkDeviceSize range) {
            infos.push_back({b.buf, off, range});
            VkWriteDescriptorSet w{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            w.dstSet = dset_;
            w.dstBinding = binding;
            w.descriptorCount = 1;
            w.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            w.pBufferInfo = &infos.back();
            ws.push_back(w);
        };
        add(1, table_, 0, table_.size);
        add(2, poses_, 0, poses_.size);
        add(3, out_, 0, img_bytes_);
        add(4, out_, ids_off_, ids_bytes_);
        add(5, verts_, 0, verts_.size);
        add(6, idx_, 0, idx_.size);
        add(7, srgb_, 0, srgb_.size);
        add(8, depth_, 0, depth_.size);
        vkUpdateDescriptorSets(d_.dev, static_cast<uint32_t>(ws.size()), ws.data(), 0, nullptr);
    }

    Device& d_;
    Setup s_;
    uint32_t side_ = 1, planes_ = 3, n_tlas_ = 1;
    Mesh mesh_;
    Buffer table_, poses_, poses_stage_, out_, readback_, depth_, depth_read_, verts_, idx_, srgb_, inst_, aabb_, arena_, scratch_;
    VkDeviceSize arena_used_ = 0;
    VkAccelerationStructureKHR blas_gate_ = VK_NULL_HANDLE, blas_mesh_ = VK_NULL_HANDLE;
    VkDeviceAddress blas_gate_addr_ = 0, blas_mesh_addr_ = 0;
    std::vector<VkAccelerationStructureKHR> tlas_;
    std::vector<VkDeviceAddress> tlas_addr_;
    VkAccelerationStructureGeometryKHR tlas_geom_{};
    VkBuildAccelerationStructureFlagsKHR tlas_flags_ = 0;
    uint32_t tlas_prims_ = 0;
    VkDeviceSize tlas_build_scratch_ = 0, tlas_update_scratch_ = 0, per_tlas_scratch_ = 0;
    VkDeviceAddress scratch_base_ = 0;
    VkShaderModule module_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout dsl_ = VK_NULL_HANDLE;
    VkPipelineLayout layout_ = VK_NULL_HANDLE;
    VkPipeline pipe_ = VK_NULL_HANDLE;
    VkDescriptorPool dpool_ = VK_NULL_HANDLE;
    VkDescriptorSet dset_ = VK_NULL_HANDLE;
    VkQueryPool qpool_ = VK_NULL_HANDLE;
    VkSemaphore sem_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    uint64_t counter_ = 0;
    uint64_t img_bytes_ = 0, ids_bytes_ = 0, ids_off_ = 0, as_bytes_ = 0;
    uint64_t vram_start_ = 0, vram_end_ = 0;
    double setup_ms_ = 0, build_ms_ = 0, build_gpu_ms_ = 0;
    std::vector<uint8_t> image_;
    std::vector<uint16_t> ids_;
    std::vector<float> depth_host_;
};

}  // namespace cs
