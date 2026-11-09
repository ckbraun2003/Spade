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
#include <limits>
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
#include "lbm_collide_stream.spv.gen.hpp"
#include "lbm_lattice_dump.spv.gen.hpp"
#include "lbm_macro.spv.gen.hpp"
#include "lbm_support.hpp"
#include "physics/lbm/lattice.hpp"
#include "physics/lbm/lbm_flow.hpp"
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

// What the step's kernels must not hold: atomics (OpAtomic*, the flag ops, the float min/max/add extensions), barriers
// and groupshared (Workgroup storage class) variables.
struct OpCensus {
    std::size_t atomics = 0;
    std::size_t barriers = 0;
    std::size_t workgroup_vars = 0;
};

[[nodiscard]] OpCensus op_census(std::span<const uint32_t> words) {
    OpCensus out;
    std::size_t i = 5;  // past the header
    while (i < words.size()) {
        const uint32_t count = words[i] >> 16;
        const uint32_t op = words[i] & 0xFFFFu;
        if (count == 0 || i + count > words.size()) break;
        if ((op >= 227 && op <= 242) || op == 318 || op == 319 || op == 5614 || op == 5615 || op == 6035) ++out.atomics;
        if (op == 224 || op == 225) ++out.barriers;
        if (op == 59 && count >= 4 && words[i + 3] == 4) ++out.workgroup_vars;  // OpVariable, StorageClass Workgroup
        i += count;
    }
    return out;
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

namespace {

// An LBM kernel is scanned under the exact profile (P1-P4, E1/E2) and SignedZeroInfNanPreserve 32 (rule P6, as
// every airflow kernel), in every workgroup-size variant, and binds only the set-1 bindings it names. The dump
// kernel has no float arithmetic (conversions, selects and stores), so it is integer-only for the contractable-op
// instrument; the step's kernels divide and take roots only through div32_ftz and sqrt32_ftz, whose estimates the
// scan traces to an integer conversion (E1/E2), so they must show some.
void expect_kernel_rules(const spade::compute::SpirvVariantSet& variants, bool integer_only, bool estimates,
                         const std::vector<uint32_t>& bindings) {
    ASSERT_EQ(variants.count, spade::compute::kSupportedWorkgroupSizes.size()) << variants.name;
    for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
        EXPECT_NE(variants.for_size(wg), nullptr) << variants.name << ": no variant for " << wg;
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
        if (integer_only) {
            EXPECT_EQ(scan.contractable_ops, 0u) << module << ": float arithmetic in a kernel declared to have none";
            EXPECT_GT(scan.store_count, 0u) << module;
        } else {
            EXPECT_GT(scan.contractable_ops, 0u) << module << ": no float op; is this the right module?";
        }
        if (estimates) {
            EXPECT_GT(scan.estimate_ops, 0u) << module << ": no div32_ftz / sqrt32_ftz estimate traced";
        } else {
            EXPECT_EQ(scan.estimate_ops, 0u) << module << ": an estimate where no div32_ftz is called";
        }
        EXPECT_FALSE(scan.requests_denorm_mode()) << module;
        EXPECT_TRUE(scan.preserves_nan()) << module << ": SignedZeroInfNanPreserve 32 is not declared";
        EXPECT_FALSE(scan.declares_int64) << module;
        EXPECT_EQ(scan.entry_point_name, "main") << module;
        const auto declared = declared_bindings(v.code());
        ASSERT_EQ(declared.size(), bindings.size()) << module;
        for (std::size_t b = 0; b < bindings.size(); ++b) {
            EXPECT_EQ(declared[b].first, gen::kSet_airflow) << module;
            EXPECT_EQ(declared[b].second, bindings[b]) << module;
        }
        // Rule 3 of the plan: no atomic, barrier or groupshared memory (the esoteric rule is why there is none).
        const OpCensus ops = op_census(v.code());
        EXPECT_EQ(ops.atomics, 0u) << module << ": atomics";
        EXPECT_EQ(ops.barriers, 0u) << module << ": a barrier inside the kernel";
        EXPECT_EQ(ops.workgroup_vars, 0u) << module << ": groupshared memory";
    }
    // The variants differ in the LocalSize operand only.
    const std::span<const spade::compute::SpirvVariant> all = variants.all();
    for (std::size_t a = 0; a + 1 < all.size(); ++a) {
        const std::span<const uint32_t> x = all[a].code();
        const std::span<const uint32_t> y = all[a + 1].code();
        ASSERT_EQ(x.size(), y.size());
        std::size_t differ = 0;
        for (std::size_t w = 0; w < x.size(); ++w) differ += x[w] != y[w] ? 1u : 0u;
        EXPECT_EQ(differ, 1u) << variants.name;
    }
}

}  // namespace

TEST(LbmSpirv, TheLatticeDumpHoldsTheDeterminismRules) {
    expect_kernel_rules(gen::kSpvVariants_lbm_lattice_dump, true, false, {gen::kBinding_lbm_f});
}

// T2 and T3: the velocity pass reads lbm_f and writes lbm_u; the collide-and-stream pass reads lbm_u and
// reads and writes lbm_f. Both divide (macro_of, the wall mean, omega, WALE's quotient) and root (WALE) only
// through the exact operations.
TEST(LbmSpirv, TheStepKernelsHoldTheDeterminismRules) {
    expect_kernel_rules(gen::kSpvVariants_lbm_macro, false, true, {gen::kBinding_lbm_f, gen::kBinding_lbm_u});
    expect_kernel_rules(gen::kSpvVariants_lbm_collide_stream, false, true,
                        {gen::kBinding_lbm_f, gen::kBinding_lbm_u});
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

// ===========================================================================
// GpuLbmStep -- the step's two passes against the CPU reference, whole arrays, memcmp
// ===========================================================================

namespace {

using spade::lbm_gpu::lbm_push_of;
using spade::lbm_gpu::lbm_step_chain;
using spade::physics::lbm::LbmFlow;
namespace support = spade::testing::lbm_support;

struct DeviceState {
    std::vector<float> f;
    std::vector<float> u;
};

// How many words of `a` differ in bits from `b` (sizes must match), and the first.
struct WordDiff {
    std::size_t count = 0;
    std::size_t first = 0;
};

[[nodiscard]] WordDiff diff_words(const std::vector<float>& a, const std::vector<float>& b) {
    WordDiff d;
    if (a.size() != b.size()) {
        d.count = std::max(a.size(), b.size());
        return d;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (bits(a[i]) != bits(b[i])) {
            if (d.count == 0) d.first = i;
            ++d.count;
        }
    }
    return d;
}

// Uploads `start`'s populations and u to a runner of the step kernels, records `steps` LBM steps from start's step
// counter (the velocity pass alone with `macro_only`), runs them once and reads f and u back.
[[nodiscard]] bool run_steps(VulkanContext& ctx, const LbmFlow& start, uint32_t steps, bool macro_only,
                             DeviceState& out) {
    const std::vector<RunnerBuffer> buffers = {
        {"lbm_f", start.grid.f.size() * 4ull, spade::airflow_gpu::BufferKind::staged},
        {"lbm_u", start.u.size() * 4ull, spade::airflow_gpu::BufferKind::staged},
    };
    const spade::compute::SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_lbm_macro,
                                                              &gen::kSpvVariants_lbm_collide_stream};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers,
                                                      .kernels = kernels,
                                                      .push_bytes = sizeof(LbmPush),
                                                      .workgroup_size = kWorkgroupSize,
                                                      .binding_base = kLbmBindingBase});
    if (!runner.has_value()) {
        ADD_FAILURE() << runner.error().context;
        return false;
    }
    if (auto up = (*runner)->upload(0, std::as_bytes(std::span<const float>(start.grid.f))); !up) {
        ADD_FAILURE() << up.error().context;
        return false;
    }
    if (auto up = (*runner)->upload(1, std::as_bytes(std::span<const float>(start.u))); !up) {
        ADD_FAILURE() << up.error().context;
        return false;
    }
    const auto push = lbm_push_of(start);
    if (!push.has_value()) {
        ADD_FAILURE() << push.error().context;
        return false;
    }
    const std::vector<ChainCommand> chain =
        lbm_step_chain(*push, start.grid.step, steps, kWorkgroupSize, macro_only);
    auto id = (*runner)->record(chain);
    if (!id.has_value()) {
        ADD_FAILURE() << id.error().context;
        return false;
    }
    if (auto s = (*runner)->submit(*id, 1); !s) {
        ADD_FAILURE() << s.error().context;
        return false;
    }
    out.f.assign(start.grid.f.size(), 0.0f);
    out.u.assign(start.u.size(), 0.0f);
    if (auto r = (*runner)->read(0, std::as_writable_bytes(std::span<float>(out.f))); !r) {
        ADD_FAILURE() << r.error().context;
        return false;
    }
    if (auto r = (*runner)->read(1, std::as_writable_bytes(std::span<float>(out.u))); !r) {
        ADD_FAILURE() << r.error().context;
        return false;
    }
    return true;
}

// The reference: n scoped CPU steps (the flush scope, as the device flushes).
void cpu_steps(LbmFlow& fl, uint32_t n) {
    for (uint32_t t = 0; t < n; ++t) support::step_ok_flushed(fl);
}

// The device's n steps from `start` against the CPU's, f and u, and the words that differ.
void expect_steps_match(VulkanContext& ctx, const LbmFlow& start, uint32_t n, const std::string& what) {
    LbmFlow cpu = start;
    cpu_steps(cpu, n);
    DeviceState dev;
    ASSERT_TRUE(run_steps(ctx, start, n, false, dev)) << what;
    // The comparison means something only on a field that has not blown up: NaN payloads are the hardware's own
    // (see ABlownUpFieldIsNaNOnBothSidesWithTheSamePattern).
    EXPECT_TRUE(std::all_of(cpu.grid.f.begin(), cpu.grid.f.end(), [](float v) { return std::isfinite(v); }))
        << what << ": " << n << " step(s): the CPU field is not finite, pick a calmer field";
    support::expect_mass_steady(cpu, what.c_str());  // a field that gained or lost mass is not a physical case
    const WordDiff df = diff_words(dev.f, cpu.grid.f);
    const WordDiff du = diff_words(dev.u, cpu.u);
    EXPECT_EQ(df.count, std::size_t{0}) << what << ": " << n << " step(s), f differs in " << df.count
                                        << " words, first at " << df.first << ": device 0x" << std::hex
                                        << (df.count != 0 ? bits(dev.f[df.first]) : 0u) << ", CPU 0x"
                                        << (df.count != 0 ? bits(cpu.grid.f[df.first]) : 0u);
    EXPECT_EQ(du.count, std::size_t{0}) << what << ": " << n << " step(s), u differs in " << du.count
                                        << " words, first at " << du.first;
    std::printf("[GpuLbmStep] %s, %u step(s): f wrong=%zu of %zu, u wrong=%zu of %zu\n", what.c_str(), n, df.count,
                cpu.grid.f.size(), du.count, cpu.u.size());
}

using support::six_wall_field;

// A uniform fluid at rest, or drifting at u0 along x.
[[nodiscard]] LbmFlow uniform_field(float u0) {
    LbmFlow fl = support::box_field(false);
    for (uint32_t z = 0; z < fl.grid.nz; ++z) {
        for (uint32_t y = 0; y < fl.grid.ny; ++y) {
            for (uint32_t x = 0; x < fl.grid.nx; ++x) {
                spade::physics::lbm::set_node_equilibrium(fl, x, y, z, 1.0f, u0, 0.0f, 0.0f);
            }
        }
    }
    return fl;
}

// A field of populations of the size `scale`, from a fixed stream (splitmix64), signed.
[[nodiscard]] LbmFlow tiny_field(float scale) {
    LbmFlow fl = support::box_field(false);
    uint64_t state = 0x1B3C0FFEEull;
    for (float& v : fl.grid.f) {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= z >> 31;
        const float unit = static_cast<float>(static_cast<int64_t>(z >> 40) - (int64_t{1} << 23)) / 8388608.0f;
        v = scale * unit;
    }
    return fl;
}

}  // namespace

// T2: the velocity pass alone, on both parities, over the periodic Taylor-Green field of LbmThreads (odd sizes, every
// wrap), the lid box, the six-wall box, a quiet field and a uniform drift, after the CPU has stepped to that parity.
// Mutation: the parity word swapped (lbm_step_chain's parity); a division replaced by the bare operator (the scan
// rejects it in LbmSpirv).
TEST(GpuLbmMacro, UIsTheCpusOnBothParities) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;

    const std::pair<const char*, LbmFlow> fields[] = {{"periodic Taylor-Green", support::box_field(false)},
                                                      {"lid box", support::box_field(true)},
                                                      {"six-wall box", six_wall_field()},
                                                      {"quiet", uniform_field(0.0f)},
                                                      {"uniform drift", uniform_field(0.05f)}};
    for (const auto& [name, field] : fields) {
        for (uint32_t parity = 0; parity < 2; ++parity) {
            LbmFlow cpu = field;
            cpu_steps(cpu, parity);  // the step counter is `parity`: the next velocity pass reads that parity's slots
            const LbmFlow start = cpu;
            (void)spade::physics::lbm::flow_sums_flushed(cpu);  // the velocity pass under the flush scope
            DeviceState dev;
            ASSERT_TRUE(run_steps(**ctx, start, 1, true, dev)) << name;
            const WordDiff du = diff_words(dev.u, cpu.u);
            EXPECT_EQ(du.count, std::size_t{0}) << name << ", parity " << parity << ": u differs in " << du.count
                                                << " words, first at " << du.first;
            EXPECT_EQ(diff_words(dev.f, start.grid.f).count, std::size_t{0})
                << name << ", parity " << parity << ": the velocity pass wrote lbm_f";
            std::printf("[GpuLbmMacro] %s, parity %u: u wrong=%zu of %zu\n", name, parity, du.count, cpu.u.size());
        }
    }
}

// T3: n whole steps (both passes, both parities many times over) from the periodic field, the whole of f and u
// memcmp'd to the serial scoped CPU step, WALE on and off, nu 0 and 1e-3. Mutation: contraction left on one op (the
// scan and this); the third-order recursion dropped; a slot swapped on the odd write.
TEST(GpuLbmStep, ThePeriodicStepIsTheCpusBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    for (const float cw : {0.5f, 0.0f}) {
        for (const float nu : {0.0f, 1.0e-3f}) {
            LbmFlow start = support::box_field(false);
            start.cw = cw;
            start.nu = nu;
            const std::string what = "periodic cw " + std::to_string(cw) + " nu " + std::to_string(nu);
            for (const uint32_t n : {1u, 2u, 7u, 64u}) expect_steps_match(**ctx, start, n, what);
        }
    }
}

// T4: the walls and the lid. The lid box of LbmThreads (the moving-wall term, the corner links, the periodic z) and a
// box walled on every face with a different velocity on each. Mutation: the wall term dropped; the wall mean 1 /
// crossed replaced by a multiply by a rounded reciprocal.
TEST(GpuLbmStep, TheWalledBoxesAreTheCpusBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    for (const uint32_t n : {1u, 2u, 7u, 64u}) expect_steps_match(**ctx, support::box_field(true), n, "lid box");
    for (const uint32_t n : {1u, 2u, 7u, 64u}) expect_steps_match(**ctx, six_wall_field(), n, "six-wall box");
}

// A field that blows up: the periodic Taylor-Green box with one population set to infinity, so the first collision
// computes inf - inf (an invalid operation) and the NaNs spread to every node within a few steps. TD-14's bit equality holds for numbers; a
// NaN's PAYLOAD is the hardware's own (an invalid operation gives x86 the default 0xffc00000 and this device
// 0x7fffffff), so the contract here is the NaN pattern: the same words are NaN on both sides and every other word
// is the same bits. The count of words whose NaN payload differs is printed, as the measured fact the claim excludes.
TEST(GpuLbmStep, ABlownUpFieldIsNaNOnBothSidesAtTheSameWords) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    LbmFlow start = support::box_field(false);
    start.grid.f[0] = std::numeric_limits<float>::infinity();
    constexpr uint32_t kSteps = 64;
    LbmFlow cpu = start;
    cpu_steps(cpu, kSteps);
    DeviceState dev;
    ASSERT_TRUE(run_steps(**ctx, start, kSteps, false, dev));
    std::size_t nan_cpu = 0;
    std::size_t pattern_wrong = 0;
    std::size_t payload_differs = 0;
    for (const auto& [d, c] : {std::pair{&dev.f, &cpu.grid.f}, std::pair{&dev.u, &cpu.u}}) {
        ASSERT_EQ(d->size(), c->size());
        for (std::size_t i = 0; i < d->size(); ++i) {
            const bool nd = std::isnan((*d)[i]);
            const bool nc = std::isnan((*c)[i]);
            nan_cpu += nc ? 1u : 0u;
            if (nd != nc) {
                ++pattern_wrong;
            } else if (!nc && bits((*d)[i]) != bits((*c)[i])) {
                ++pattern_wrong;
            } else if (nc && bits((*d)[i]) != bits((*c)[i])) {
                ++payload_differs;
            }
        }
    }
    std::printf("[GpuLbmStep] blown-up field, %u steps: NaN words (CPU) %zu, pattern wrong=%zu, payload differs %zu\n",
                kSteps, nan_cpu, pattern_wrong, payload_differs);
    EXPECT_GT(nan_cpu, std::size_t{0}) << "the field did not blow up: pick a more violent one";
    EXPECT_EQ(pattern_wrong, std::size_t{0});
}

// The subnormal probe on the LBM's own op mix (the plan's global constraints: the device's arithmetic flush is its
// default, measured for the device of record, not enforced in the kernel): fields whose populations are subnormal, and
// fields whose products are, stepped on both sides and compared whole. Mutation: a flush removed from sqrt32_ftz's
// path.
TEST(GpuLbmFlush, SubnormalsFlushAsTheScopeDoes) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    // 1e-40 and 5e-41 are subnormal populations; 1e-20 populations make u about 1e-20 and u^2 about 1e-40 (a
    // subnormal product); 1e-30 makes the products of products subnormal.
    for (const float scale : {1.0e-40f, 5.0e-41f, 1.0e-30f, 1.0e-20f}) {
        char label[48];
        std::snprintf(label, sizeof(label), "tiny populations %g", static_cast<double>(scale));
        const std::string what = label;
        for (const uint32_t n : {1u, 2u, 3u}) expect_steps_match(**ctx, tiny_field(scale), n, what);
    }
}
