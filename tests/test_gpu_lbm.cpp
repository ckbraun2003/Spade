// ---------------------------------------------------------------------------
// test_gpu_lbm.cpp -- the LBM tier's GPU twin (L1; docs/design/physics/plans/
// 2026-11-06-l1-gpu-twin-plan.md). T1 so far: the lattice tables and the push
// block, and the compiled kernel's determinism rules.
//
//   LbmSpirv           the compiled modules (no device; not gpu-labelled)
//   LbmAdmit           what a created LBM runner asks of a device, through
//                      compute::admit() (host-only, synthetic limits)
//   GpuLbmLattice      the device's D3Q27 tables and push block against the
//                      CPU's (engine/physics/lbm/lattice.hpp), bit for bit, and
//                      what the real device admits
//
// Every Gpu* test starts with the vulkan_skip_reason() skip (gpu_skip.hpp).
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "compute/admit.hpp"
#include "compute/backend.hpp"
#include "compute/device_limits.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "gpu_skip.hpp"
#include "lbm_lattice_dump.spv.gen.hpp"
#include "physics/lbm/lattice.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "spike/lbm/lbm_chain.hpp"
#include "testing/spirv_scan.hpp"

namespace {

namespace lbm = spade::physics::lbm;
namespace gen = spade::compute::gen;
using spade::airflow_gpu::ChainRunner;
using spade::airflow_gpu::RunnerBuffer;
using spade::airflow_gpu::RunnerDesc;
using spade::compute::ChainCommand;
using spade::compute::VulkanContext;
using spade::lbm_gpu::kLbmBindingBase;
using spade::lbm_gpu::kLbmPushWords;
using spade::lbm_gpu::LbmPush;

constexpr uint32_t kWorkgroupSize = 64;
constexpr uint32_t kDumpInvocations = lbm::kQ + kLbmPushWords;  // 27 table rows and the 25 echoed push words
constexpr std::size_t kTableWords = std::size_t{5} * lbm::kQ;   // cx, cy, cz, w, opposite per direction

[[nodiscard]] uint32_t bits(float v) { return std::bit_cast<uint32_t>(v); }

// A push block with every word distinct and recognisable.
[[nodiscard]] LbmPush sample_push() {
    LbmPush p;
    p.nx = 12;
    p.ny = 10;
    p.nz = 7;
    p.parity = 1;
    p.nu = 0.0123f;
    p.cw = 0.5f;
    p.periodic_mask = 5;
    for (uint32_t k = 0; k < 18; ++k) p.wall_u[k] = 0.25f * static_cast<float>(k) - 1.5f;
    return p;
}

// Runs lbm_lattice_dump with `push` and returns lbm_f's first (135 + 25) words.
[[nodiscard]] std::vector<uint32_t> run_dump(VulkanContext& ctx, const LbmPush& push) {
    constexpr std::size_t kWords = kTableWords + kLbmPushWords;
    const std::vector<RunnerBuffer> buffers = {
        {"lbm_f", kWords * 4, spade::airflow_gpu::BufferKind::staged},
        {"lbm_u", 4, spade::airflow_gpu::BufferKind::staged},
    };
    const spade::compute::SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_lbm_lattice_dump};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers,
                                                      .kernels = kernels,
                                                      .push_bytes = sizeof(LbmPush),
                                                      .workgroup_size = kWorkgroupSize,
                                                      .binding_base = kLbmBindingBase});
    if (!runner.has_value()) {
        ADD_FAILURE() << runner.error().context;
        return {};
    }
    const std::vector<uint32_t> zeros(kWords, 0xDEADBEEFu);  // a word the kernel fails to write stays visible
    if (auto up = (*runner)->upload(0, std::as_bytes(std::span<const uint32_t>(zeros))); !up) {
        ADD_FAILURE() << up.error().context;
        return {};
    }
    const uint32_t groups = (kDumpInvocations + kWorkgroupSize - 1) / kWorkgroupSize;
    ChainCommand c{.is_dispatch = true, .pipeline = 0, .groups = {groups, 1, 1}};
    std::memcpy(c.push.data(), &push, sizeof(push));
    const ChainCommand chain[] = {c};
    auto id = (*runner)->record(chain);
    if (!id.has_value()) {
        ADD_FAILURE() << id.error().context;
        return {};
    }
    if (auto s = (*runner)->submit(*id, 1); !s) {
        ADD_FAILURE() << s.error().context;
        return {};
    }
    std::vector<uint32_t> out(kWords, 0);
    if (auto r = (*runner)->read(0, std::as_writable_bytes(std::span<uint32_t>(out))); !r) {
        ADD_FAILURE() << r.error().context;
        return {};
    }
    return out;
}

// What an LBM runner of an nx x ny x nz grid asks of the device (stage 7 Task A1's admit()): one pipeline layout
// of an empty set 0 and the two LBM bindings of set 1 (the subset layout, 0 + 2 storage buffers), the push block,
// the dump kernel's local size, lbm_f (27 floats a node) and lbm_u (3), and the dump's dispatch.
[[nodiscard]] spade::compute::AdmitShape lbm_shape(uint64_t nodes) {
    spade::compute::AdmitShape shape;
    shape.layouts = {{"lbm (set 0 empty + set 1 subset 40..41)", {0u, 2u}}};
    shape.buffers = {{"lbm_f", nodes * 27u * 4u}, {"lbm_u", nodes * 3u * 4u}};
    shape.kernels = {{"lbm_lattice_dump", {kWorkgroupSize, 1u, 1u}}};
    shape.dispatches = {{"lbm_lattice_dump", {(kDumpInvocations + kWorkgroupSize - 1) / kWorkgroupSize, 1u, 1u}}};
    shape.push_bytes = sizeof(LbmPush);
    return shape;
}

// OpDecorate DescriptorSet (34) and Binding (33): every (set, binding) the module declares.
[[nodiscard]] std::vector<std::pair<uint32_t, uint32_t>> declared_bindings(std::span<const uint32_t> words) {
    std::vector<uint32_t> set_of(words.size() ? words[3] : 0, 0xFFFFFFFFu);
    std::vector<uint32_t> binding_of(set_of.size(), 0xFFFFFFFFu);
    std::size_t i = 5;
    while (i < words.size()) {
        const uint32_t count = words[i] >> 16;
        const uint32_t op = words[i] & 0xFFFFu;
        if (count == 0 || i + count > words.size()) break;
        if (op == 71 && count >= 4 && words[i + 1] < set_of.size()) {  // OpDecorate
            if (words[i + 2] == 34) set_of[words[i + 1]] = words[i + 3];
            if (words[i + 2] == 33) binding_of[words[i + 1]] = words[i + 3];
        }
        i += count;
    }
    std::vector<std::pair<uint32_t, uint32_t>> out;
    for (std::size_t id = 0; id < set_of.size(); ++id) {
        if (set_of[id] != 0xFFFFFFFFu && binding_of[id] != 0xFFFFFFFFu) out.emplace_back(set_of[id], binding_of[id]);
    }
    std::sort(out.begin(), out.end());
    return out;
}

}  // namespace

// ===========================================================================
// LbmSpirv -- the compiled modules, no device needed
// ===========================================================================

// The LBM kernels are scanned under the exact profile (P1-P4, E1/E2) and
// SignedZeroInfNanPreserve 32 (rule P6, as every airflow kernel), and bind only
// set 1's LBM bindings. The dump kernel has no float arithmetic (conversions,
// selects and stores), so it is integer-only for the contractable-op instrument.
TEST(LbmSpirv, TheLatticeDumpHoldsTheDeterminismRules) {
    const auto& variants = gen::kSpvVariants_lbm_lattice_dump;
    ASSERT_EQ(variants.count, spade::compute::kSupportedWorkgroupSizes.size());
    for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
        EXPECT_NE(variants.for_size(wg), nullptr) << "no variant for " << wg;
    }
    for (const spade::compute::SpirvVariant& v : variants.all()) {
        const std::string module =
            std::string(variants.name) + " (local size " + std::to_string(v.workgroup_size) + ")";
        const spade::testing::SpirvScanResult scan = spade::testing::scan_spirv(
            v.code(), spade::testing::SpirvProfile::exact, spade::testing::NanPreserve::required);
        EXPECT_TRUE(scan.well_formed) << module;
        for (const spade::testing::SpirvFinding& finding : scan.findings) {
            ADD_FAILURE() << module << " [" << finding.rule << "] " << finding.message;
        }
        EXPECT_EQ(scan.decorated_ops, scan.contractable_ops) << module << ": an op lacks NoContraction";
        EXPECT_EQ(scan.contractable_ops, 0u) << module << ": float arithmetic in a kernel declared to have none";
        EXPECT_GT(scan.store_count, 0u) << module;
        EXPECT_FALSE(scan.requests_denorm_mode()) << module;
        EXPECT_TRUE(scan.preserves_nan()) << module << ": SignedZeroInfNanPreserve 32 is not declared";
        EXPECT_FALSE(scan.declares_int64) << module;
        EXPECT_EQ(scan.entry_point_name, "main") << module;
        // Set 1, binding 40 (lbm_f) and nothing else: the dump kernel reads no other array.
        const auto declared = declared_bindings(v.code());
        ASSERT_EQ(declared.size(), std::size_t{1}) << module;
        EXPECT_EQ(declared[0].first, gen::kSet_airflow) << module;
        EXPECT_EQ(declared[0].second, gen::kBinding_lbm_f) << module;
    }
    // The variants differ in the LocalSize operand only.
    const std::span<const spade::compute::SpirvVariant> all = variants.all();
    for (std::size_t a = 0; a + 1 < all.size(); ++a) {
        const std::span<const uint32_t> x = all[a].code();
        const std::span<const uint32_t> y = all[a + 1].code();
        ASSERT_EQ(x.size(), y.size());
        std::size_t differ = 0;
        for (std::size_t w = 0; w < x.size(); ++w) differ += x[w] != y[w] ? 1u : 0u;
        EXPECT_EQ(differ, 1u);
    }
}

// The push block fits a chain command and the runner binds LBM buffers from
// binding 40 (the generated bindings, not a number typed here).
TEST(LbmSpirv, TheBindingsAreSectionFsFortyAndFortyOne) {
    EXPECT_EQ(kLbmBindingBase, 40u);
    EXPECT_EQ(gen::kBinding_lbm_f, 40u);
    EXPECT_EQ(gen::kBinding_lbm_u, 41u);
    EXPECT_EQ(sizeof(LbmPush), std::size_t{100});
    EXPECT_LE(sizeof(LbmPush), std::size_t{spade::compute::kMaxChainPushBytes});
}

// ===========================================================================
// LbmAdmit -- host-only
// ===========================================================================

// On Vulkan's guaranteed minimums a 64^3 field is admitted (lbm_f 28 MB) and a 128^3 one is not (226 MB over the
// 2^27 B range), by name: the LBM tier needs a device that reports more, and says so before any allocation.
TEST(LbmAdmit, ASmallFieldFitsTheGuaranteedLimitsAndALargeOneIsRefusedByName) {
    const spade::compute::DeviceLimits& g = spade::compute::kGuaranteedLimits;
    EXPECT_TRUE(spade::compute::admit(lbm_shape(uint64_t{64} * 64 * 64), g, "guaranteed").has_value());
    const auto big = spade::compute::admit(lbm_shape(uint64_t{128} * 128 * 128), g, "guaranteed");
    ASSERT_FALSE(big.has_value());
    EXPECT_NE(big.error().context.find("lbm_f"), std::string::npos) << big.error().context;
    EXPECT_NE(big.error().context.find("maxStorageBufferRange"), std::string::npos) << big.error().context;
}

// The layout rule counts every set of a pipeline layout: the LBM layout is 0 + 2 storage buffers, far under the
// guaranteed 4 per stage, and a layout that carried set 0's 30 as well would not be (the empty set 0 is the point).
TEST(LbmAdmit, TheLayoutIsTwoStorageBuffersNotThirtyTwo) {
    spade::compute::AdmitShape shape = lbm_shape(1000);
    EXPECT_TRUE(spade::compute::admit(shape, spade::compute::kGuaranteedLimits, "guaranteed").has_value());
    shape.layouts = {{"state+lbm", {30u, 2u}}};
    const auto refused = spade::compute::admit(shape, spade::compute::kGuaranteedLimits, "guaranteed");
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("state+lbm"), std::string::npos) << refused.error().context;
}

// ===========================================================================
// GpuLbmLattice
// ===========================================================================

// What the real device admits for the LBM shape, printed: the counts admit() saw and the largest field one buffer
// holds (min of maxStorageBufferRange and maxMemoryAllocationSize over lbm_f's 108 bytes a node).
TEST(GpuLbmLattice, TheDeviceAdmitsTheLbmShape) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    const spade::compute::DeviceLimits& limits = (*ctx)->device_report().limits;
    const std::string device(std::string_view((*ctx)->device_name()));

    const spade::compute::AdmitShape small = lbm_shape(uint64_t{128} * 128 * 128);
    const auto ok = spade::compute::admit(small, limits, device);
    EXPECT_TRUE(ok.has_value()) << (ok ? "" : ok.error().context);
    const uint64_t buffer_cap = std::min(limits.max_storage_buffer_range, limits.max_memory_allocation_size);
    const uint64_t largest_nodes = buffer_cap / (27u * 4u);
    std::printf("[GpuLbmLattice] admit() saw on '%s': %zu layout (storage buffers %u+%u of %u per stage), %zu kernel, "
                "%zu buffers, %zu dispatch, push %u of %u B; one lbm_f holds up to %llu nodes (%llu^3)\n",
                device.c_str(), small.layouts.size(), small.layouts[0].bindings_per_set[0],
                small.layouts[0].bindings_per_set[1], limits.max_per_stage_storage_buffers, small.kernels.size(),
                small.buffers.size(), small.dispatches.size(), small.push_bytes, limits.max_push_constants_size,
                static_cast<unsigned long long>(largest_nodes),
                static_cast<unsigned long long>(std::cbrt(static_cast<double>(largest_nodes))));
    // One node over the buffer cap is refused by name, before any allocation.
    const auto over = spade::compute::admit(lbm_shape(largest_nodes + 1), limits, device);
    ASSERT_FALSE(over.has_value());
    EXPECT_NE(over.error().context.find("lbm_f"), std::string::npos) << over.error().context;
}

// kC, kW and opposite(), written out by the device, equal lattice.hpp's bit
// for bit. Mutation: one table entry wrong (lbm_lattice.slang's kLbmCx[7], say).
TEST(GpuLbmLattice, TheTablesAreTheCpusBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;

    const std::vector<uint32_t> got = run_dump(**ctx, sample_push());
    ASSERT_EQ(got.size(), kTableWords + kLbmPushWords);

    std::size_t wrong = 0;
    for (uint32_t i = 0; i < lbm::kQ; ++i) {
        const std::array<uint32_t, 5> want = {
            bits(static_cast<float>(lbm::kC[i].x)), bits(static_cast<float>(lbm::kC[i].y)),
            bits(static_cast<float>(lbm::kC[i].z)), bits(lbm::kW[i]), bits(static_cast<float>(lbm::opposite(i)))};
        for (uint32_t k = 0; k < 5; ++k) {
            if (got[5 * i + k] != want[k]) {
                ++wrong;
                ADD_FAILURE() << "direction " << i << " word " << k << ": device 0x" << std::hex << got[5 * i + k]
                              << ", CPU 0x" << want[k];
            }
        }
    }
    std::printf("[GpuLbmLattice] %u directions x 5 words: wrong=%zu\n", lbm::kQ, wrong);
    EXPECT_EQ(wrong, std::size_t{0});
}

// LbmPush as lbm_params.slang lays it out, echoed by the device word for word.
// Mutation: a field reordered in either declaration.
TEST(GpuLbmLattice, ThePushBlockReadsBackWordForWord) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;

    const LbmPush push = sample_push();
    std::array<uint32_t, kLbmPushWords> want{};
    std::memcpy(want.data(), &push, sizeof(push));

    const std::vector<uint32_t> got = run_dump(**ctx, push);
    ASSERT_EQ(got.size(), kTableWords + kLbmPushWords);
    std::size_t wrong = 0;
    for (uint32_t k = 0; k < kLbmPushWords; ++k) {
        if (got[kTableWords + k] != want[k]) {
            ++wrong;
            ADD_FAILURE() << "push word " << k << ": device 0x" << std::hex << got[kTableWords + k] << ", host 0x"
                          << want[k];
        }
    }
    std::printf("[GpuLbmLattice] push block %zu bytes: wrong=%zu\n", sizeof(LbmPush), wrong);
    EXPECT_EQ(wrong, std::size_t{0});
}
