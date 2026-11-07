// test_gpu_dispatch_3d.cpp -- the 3D dispatch on a device (module API stage 7, Task B1;
// docs/design/core/plans/2026-11-06-module-api-stage7-plan.md; design 7.5). The arithmetic (local shapes, group counts,
// the worlds' z blocks) is test_dispatch_shape.cpp; this is the kernel's view of it, through the ChainRunner (which has
// the set-1 buffers the host can read, so it needs no engine-side set-1 user yet).
//
// tests/kernels/dispatch_3d_id.slang records, for every thread of the dispatch, the cell it visited (or 0 for a thread
// outside its world's extent). So "every global id once" is checked from the output alone: the non-zero values are exactly
// 1..cells, each once, and the rest are zero. Every test skips with its reason when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/backend.hpp"
#include "compute/dispatch_shape.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "dispatch_3d_id.spv.gen.hpp"
#include "gpu_skip.hpp"
#include "spike/airflow/chain_runner.hpp"

namespace {

using spade::airflow_gpu::BufferKind;
using spade::airflow_gpu::ChainRunner;
using spade::airflow_gpu::kAirflowSetBuffers;
using spade::airflow_gpu::RunnerBuffer;
using spade::airflow_gpu::RunnerDesc;
using spade::compute::ChainCommand;
using spade::compute::grid_groups;
using spade::compute::GridExtent;
using spade::compute::local_shape;
using spade::compute::SpirvVariantSet;
using spade::compute::VulkanContext;
namespace gen = spade::compute::gen;

constexpr uint32_t kU = gen::kBinding_airflow_u;
constexpr uint32_t kV = gen::kBinding_airflow_v;

struct IdPush {
    uint32_t nx, ny, nz, worlds, groups_x, groups_y, groups_z, unused;
};
static_assert(sizeof(IdPush) == 32);

struct Case {
    GridExtent extent;
    uint32_t worlds;
};

struct Visited {
    std::vector<uint32_t> threads;  // airflow_u: per thread, 0 or cell + 1
    std::vector<uint32_t> cells;    // airflow_v: per cell, a function of the index alone
    uint64_t cell_count = 0;
    uint64_t thread_count = 0;
};

// Runs the id kernel over `c` at `workgroup_size` and reads both outputs back.
[[nodiscard]] bool run_ids(VulkanContext& ctx, const Case& c, uint32_t workgroup_size, Visited& out, std::string& why) {
    const auto groups = grid_groups(c.extent, workgroup_size, c.worlds);
    const auto shape = local_shape(workgroup_size);
    out.cell_count = uint64_t{c.extent.x} * c.extent.y * c.extent.z * c.worlds;
    out.thread_count = uint64_t{groups[0]} * shape.x * groups[1] * shape.y * groups[2] * shape.z;
    if (out.thread_count == 0 || out.cell_count == 0) {
        why = "the stub answers an empty dispatch";
        return false;
    }

    std::vector<RunnerBuffer> buffers;
    for (uint32_t b = 0; b <= kV; ++b) {
        buffers.push_back(RunnerBuffer{std::string(kAirflowSetBuffers[b].name), 4, kAirflowSetBuffers[b].kind});
    }
    buffers[kU] = {"airflow_u", out.thread_count * 4, BufferKind::staged};
    buffers[kV] = {"airflow_v", out.cell_count * 4, BufferKind::staged};
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_dispatch_3d_id};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers, .kernels = kernels, .push_bytes = sizeof(IdPush),
                                                      .workgroup_size = workgroup_size});
    if (!runner) {
        why = runner.error().context;
        return false;
    }

    // Sentinels, so a thread that never ran shows: u starts at 0xFFFFFFFF, v at 0.
    const std::vector<uint32_t> u_init(out.thread_count, 0xFFFFFFFFu);
    const std::vector<uint32_t> v_init(out.cell_count, 0u);
    if (!(*runner)->upload(kU, std::as_bytes(std::span(u_init))) || !(*runner)->upload(kV, std::as_bytes(std::span(v_init)))) {
        why = "upload";
        return false;
    }

    ChainCommand dispatch{.is_dispatch = true, .pipeline = 0, .groups = groups};
    const IdPush push{c.extent.x, c.extent.y, c.extent.z, c.worlds, groups[0], groups[1], groups[2], 0};
    std::memcpy(dispatch.push.data(), &push, sizeof(push));
    const ChainCommand chain[] = {dispatch};
    auto id = (*runner)->record(chain);
    if (!id) {
        why = id.error().context;
        return false;
    }
    if (auto ok = (*runner)->submit(*id, 1); !ok) {
        why = ok.error().context;
        return false;
    }

    out.threads.assign(out.thread_count, 0u);
    out.cells.assign(out.cell_count, 0u);
    if (!(*runner)->read(kU, std::as_writable_bytes(std::span(out.threads))) ||
        !(*runner)->read(kV, std::as_writable_bytes(std::span(out.cells)))) {
        why = "read";
        return false;
    }
    return true;
}

}  // namespace

TEST(Dispatch3D, AKernelSeesEveryGlobalIdOnce) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;

    const Case cases[] = {{{16, 16, 16}, 1}, {{33, 17, 9}, 1}, {{33, 17, 9}, 2}};
    for (const Case& c : cases) {
        for (const uint32_t size : {32u, 64u, 128u}) {
            Visited v;
            std::string why;
            ASSERT_TRUE(run_ids(**ctx, c, size, v, why))
                << c.extent.x << "x" << c.extent.y << "x" << c.extent.z << " worlds " << c.worlds << " size " << size << ": "
                << why;
            std::vector<uint32_t> seen(v.cell_count, 0u);
            uint64_t zeros = 0;
            for (const uint32_t t : v.threads) {
                if (t == 0u) {
                    ++zeros;
                    continue;
                }
                ASSERT_NE(t, 0xFFFFFFFFu) << "a thread of the dispatch never ran (size " << size << ")";
                ASSERT_LE(t, v.cell_count) << "a thread wrote a cell outside the grid";
                ++seen[t - 1];
            }
            for (uint64_t cell = 0; cell < v.cell_count; ++cell) {
                ASSERT_EQ(seen[cell], 1u) << "cell " << cell << " visited " << seen[cell] << " times (size " << size << ", "
                                          << c.extent.x << "x" << c.extent.y << "x" << c.extent.z << ", worlds " << c.worlds
                                          << ")";
            }
            EXPECT_EQ(zeros, v.thread_count - v.cell_count) << "every thread outside the extent writes 0";
        }
    }
}

// The A7 invariance sweep over a grid kernel: the per-cell result is the same bits at every workgroup size.
TEST(Dispatch3D, TheThreeWorkgroupSizesAreBitIdentical) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;

    const Case c{{33, 17, 9}, 2};
    std::vector<uint32_t> reference;
    for (const uint32_t size : {32u, 64u, 128u}) {
        Visited v;
        std::string why;
        ASSERT_TRUE(run_ids(**ctx, c, size, v, why)) << "size " << size << ": " << why;
        if (reference.empty()) {
            reference = v.cells;
        } else {
            EXPECT_EQ(v.cells, reference) << "size " << size << " differs from size 32";
        }
    }
    ASSERT_FALSE(reference.empty());
    EXPECT_NE(reference[1], 0u) << "the cell values are the kernel's, not the sentinel";
}
