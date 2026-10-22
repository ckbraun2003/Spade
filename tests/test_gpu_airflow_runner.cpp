// ---------------------------------------------------------------------------
// test_gpu_airflow_runner.cpp -- plan B's first task (B1): the airflow chain
// runner (spike/airflow/chain_runner.hpp) and its three test kernels, before
// any solver kernel (docs/design/physics/plans/2026-10-12-airflow-chain-
// runner-design.md §4; plan B, 2026-10-22-airflow-gpu-solver-plan.md, B1).
//
//   GpuAirflowRunner          upload/read exact; the barrier; record once,
//                             submit n; each refusal by name, before any
//                             allocation; timestamps per mark or empty
//   GpuAirflowRunnerTileSum   A4: the fixed-tile sum gives the same bits at
//                             workgroup sizes 32, 64 and 128, and the same bits
//                             as physics/airflow/reduce.hpp on the CPU
//   AirflowRunnerSpirv        the kernels' compiled modules (plan B's): no
//                             contraction, no atomics, no shared memory, every
//                             binding in set 1 (the airflow set), and variants
//                             that differ in their local size only (no device;
//                             not gpu-labelled)
//   AirflowRunnerStaging      the staging buffer's memory type: cached and
//                             coherent first, then coherent (no device; not
//                             gpu-labelled)
//
// The kernels bind bindings.slang section F's buffers, so every runner here
// lists section F's bindings from 0 up to the last one its kernels read
// (airflow_set() below), the unused ones as 4-byte placeholders.
//
// Every Gpu* test starts with the vulkan_skip_reason() skip (gpu_skip.hpp).
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "airflow_actuator_gather.spv.gen.hpp"
#include "airflow_add_one.spv.gen.hpp"
#include "airflow_advection.spv.gen.hpp"
#include "airflow_copy.spv.gen.hpp"
#include "airflow_divergence.spv.gen.hpp"
#include "airflow_gradient_subtract.spv.gen.hpp"
#include "airflow_mg_prolong.spv.gen.hpp"
#include "airflow_mg_residual.spv.gen.hpp"
#include "airflow_mg_restrict.spv.gen.hpp"
#include "airflow_mg_smooth.spv.gen.hpp"
#include "airflow_pack_f16.spv.gen.hpp"
#include "airflow_publish_fields.spv.gen.hpp"
#include "airflow_remove_mean.spv.gen.hpp"
#include "airflow_tile_sum.spv.gen.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/reduce.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "testing/spirv_scan.hpp"

namespace {

using spade::Code;
using spade::airflow_gpu::BufferKind;
using spade::airflow_gpu::ChainRunner;
using spade::airflow_gpu::kAirflowSetBuffers;
using spade::airflow_gpu::RunnerBuffer;
using spade::airflow_gpu::RunnerDesc;
using spade::compute::ChainCommand;
using spade::compute::DeviceLimits;
using spade::compute::SpirvVariantSet;
using spade::compute::VulkanContext;
namespace gen = spade::compute::gen;

// Section F's bindings the kernels here read.
constexpr uint32_t kParams = gen::kBinding_airflow_params;
constexpr uint32_t kU = gen::kBinding_airflow_u;
constexpr uint32_t kV = gen::kBinding_airflow_v;
constexpr uint32_t kPressure = gen::kBinding_airflow_pressure;
constexpr uint32_t kRhs = gen::kBinding_airflow_rhs;
constexpr uint32_t kScalars = gen::kBinding_airflow_scalars;
constexpr uint32_t kPartials = gen::kBinding_airflow_partials;

// The runner's list for section F's bindings 0 .. through, named and kinded by
// kAirflowSetBuffers, each a 4-byte placeholder until a test sizes it.
[[nodiscard]] std::vector<RunnerBuffer> airflow_set(uint32_t through) {
    std::vector<RunnerBuffer> out;
    for (uint32_t b = 0; b <= through; ++b) {
        out.push_back(RunnerBuffer{std::string(kAirflowSetBuffers[b].name), 4, kAirflowSetBuffers[b].kind});
    }
    return out;
}

// The copy and add-one kernels' list: params (a placeholder), u and v, each of
// u and v `bytes` long and staged so a test can fill and read it.
[[nodiscard]] std::vector<RunnerBuffer> copy_buffers(uint64_t bytes) {
    std::vector<RunnerBuffer> out = airflow_set(kV);
    out[kU] = {"airflow_u", bytes, BufferKind::staged};
    out[kV] = {"airflow_v", bytes, BufferKind::staged};
    return out;
}

[[nodiscard]] const DeviceLimits& limits_of(const VulkanContext& ctx) { return ctx.device_report().limits; }

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

// splitmix64: a fixed-seed stream, the same on every compiler.
struct SplitMix {
    uint64_t state = 0;
    uint64_t next() noexcept {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
};

[[nodiscard]] uint32_t bits(float f) noexcept { return std::bit_cast<uint32_t>(f); }

// A float of either sign between 2^-12 and 2^13, built from bits (no libm):
// sums of these depend on their order in fp32.
[[nodiscard]] float mixed_magnitude(uint64_t r) noexcept {
    const uint32_t sign = static_cast<uint32_t>(r >> 63) << 31;
    const uint32_t exponent = 127u - 12u + static_cast<uint32_t>((r >> 32) % 25u);
    const uint32_t mantissa = static_cast<uint32_t>(r) & 0x7FFFFFu;
    return std::bit_cast<float>(sign | (exponent << 23) | mantissa);
}

template <class T>
[[nodiscard]] std::span<const std::byte> bytes_of(const std::vector<T>& v) noexcept {
    return std::as_bytes(std::span<const T>(v));
}

template <class T>
[[nodiscard]] std::span<std::byte> writable_bytes_of(std::vector<T>& v) noexcept {
    return std::as_writable_bytes(std::span<T>(v));
}

// Every airflow kernel pushes 16 bytes with its count in word 2
// (engine/shaders/airflow_params_layout.slang).
constexpr uint32_t kPushBytes = 16;

// The push block of airflow_copy and airflow_add_one.
struct CountPush {
    uint32_t unused0 = 0;
    uint32_t unused1 = 0;
    uint32_t count = 0;
    uint32_t unused3 = 0;
};

[[nodiscard]] CountPush count_push(uint32_t count) { return CountPush{.count = count}; }

// The push block of airflow_tile_sum.
struct TileSumPush {
    uint32_t field = 0;  // 0 airflow_pressure, 1 airflow_rhs
    uint32_t pass = 0;   // 0 the tiles; 1 the total
    uint32_t cell_count = 0;
    uint32_t slot = 0;  // airflow_scalars' slot for the total
};

static_assert(sizeof(CountPush) == kPushBytes && sizeof(TileSumPush) == kPushBytes);

template <class P>
[[nodiscard]] ChainCommand dispatch(uint32_t pipeline, const P& push, std::array<uint32_t, 3> groups) {
    static_assert(sizeof(P) <= spade::compute::kMaxChainPushBytes, "a push must fit a chain command's block");
    ChainCommand c{.is_dispatch = true, .pipeline = pipeline, .groups = groups};
    std::memcpy(c.push.data(), &push, sizeof(P));
    return c;
}

[[nodiscard]] ChainCommand mark() { return ChainCommand{}; }

[[nodiscard]] uint32_t groups_for(uint64_t threads, uint32_t workgroup_size) {
    return static_cast<uint32_t>((threads + workgroup_size - 1) / workgroup_size);
}

[[nodiscard]] bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

constexpr uint32_t kWorkgroupSize = 64;

// ===========================================================================
// GpuAirflowRunner
// ===========================================================================

// Bytes in, the same bytes out: every 32-bit pattern from a fixed seed, NaNs
// and subnormals included, through the staging buffer both ways. Two buffers
// of different sizes and kinds, so the staging buffer serves both and the
// second upload cannot disturb the first buffer. The staging memory is
// whichever kStagingMemoryPreferences found (printed): cached and coherent on
// the 3060 Ti.
TEST(GpuAirflowRunner, UploadThenReadIsExactBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    constexpr std::size_t kSmall = std::size_t{1} << 16;
    constexpr std::size_t kLarge = (std::size_t{1} << 18) + 3;
    SplitMix rng{0xA1F10Cull};
    std::vector<uint32_t> small(kSmall);
    std::vector<uint32_t> large(kLarge);
    for (uint32_t& w : small) w = static_cast<uint32_t>(rng.next());
    for (uint32_t& w : large) w = static_cast<uint32_t>(rng.next());

    std::vector<RunnerBuffer> buffers = airflow_set(kV);
    buffers[kU] = {"airflow_u", kSmall * 4, BufferKind::staged};
    buffers[kV] = {"airflow_v", kLarge * 4, BufferKind::resident};
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy};
    auto runner = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                       .push_bytes = kPushBytes, .workgroup_size = kWorkgroupSize});
    ASSERT_TRUE(runner.has_value()) << runner.error().context;

    ASSERT_TRUE((*runner)->upload(kU, bytes_of(small)).has_value());
    ASSERT_TRUE((*runner)->upload(kV, bytes_of(large)).has_value());

    std::vector<uint32_t> small_back(kSmall, 0xDEADBEEFu);
    std::vector<uint32_t> large_back(kLarge, 0xDEADBEEFu);
    auto read_small = (*runner)->read(kU, writable_bytes_of(small_back));
    ASSERT_TRUE(read_small.has_value()) << read_small.error().context;
    auto read_large = (*runner)->read(kV, writable_bytes_of(large_back));
    ASSERT_TRUE(read_large.has_value()) << read_large.error().context;

    std::size_t mismatches = 0;
    for (std::size_t i = 0; i < kSmall; ++i) mismatches += small_back[i] != small[i] ? 1u : 0u;
    for (std::size_t i = 0; i < kLarge; ++i) mismatches += large_back[i] != large[i] ? 1u : 0u;
    std::printf("[GpuAirflowRunner] upload/read: %zu words, mismatches=%zu, staging memory %s\n", kSmall + kLarge,
                mismatches, (*runner)->staging_cached() ? "cached and coherent" : "coherent, not cached");
    EXPECT_EQ(mismatches, std::size_t{0});
}

// copy (src -> dst), then add-one on dst, in ONE chain: dst comes back src + 1
// only if the second dispatch saw the first's write. dst starts at a sentinel
// so a stale read shows, and the array is large enough (4 MiB) that the two
// dispatches would overlap on the device without the barrier.
TEST(GpuAirflowRunner, SecondDispatchSeesTheFirstsWrite) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    constexpr uint32_t kCount = 1u << 20;
    std::vector<float> src(kCount);
    for (uint32_t i = 0; i < kCount; ++i) src[i] = static_cast<float>(i % 4096u);
    const std::vector<float> sentinel(kCount, -1000.0f);

    const std::vector<RunnerBuffer> buffers = copy_buffers(kCount * 4ull);
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy, &gen::kSpvVariants_airflow_add_one};
    auto runner = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                       .push_bytes = kPushBytes, .workgroup_size = kWorkgroupSize});
    ASSERT_TRUE(runner.has_value()) << runner.error().context;
    ASSERT_TRUE((*runner)->upload(kU, bytes_of(src)).has_value());
    ASSERT_TRUE((*runner)->upload(kV, bytes_of(sentinel)).has_value());

    const CountPush push = count_push(kCount);
    const ChainCommand chain[] = {dispatch(0, push, {groups_for(kCount, kWorkgroupSize), 1, 1}),
                                  dispatch(1, push, {groups_for(kCount, kWorkgroupSize), 1, 1})};
    auto id = (*runner)->record(chain);
    ASSERT_TRUE(id.has_value()) << id.error().context;

    // dispatch_chain's rule: one barrier between the two dispatches, none after.
    const spade::compute::ChainCounts counts = (*runner)->recorded_counts(*id);
    EXPECT_EQ(counts.dispatches, 2u);
    EXPECT_EQ(counts.barriers, 1u);

    auto submitted = (*runner)->submit(*id, 1);
    ASSERT_TRUE(submitted.has_value()) << submitted.error().context;

    std::vector<float> dst(kCount, 0.0f);
    ASSERT_TRUE((*runner)->read(kV, writable_bytes_of(dst)).has_value());
    std::vector<float> src_back(kCount, 0.0f);
    ASSERT_TRUE((*runner)->read(kU, writable_bytes_of(src_back)).has_value());

    std::size_t stale = 0;
    std::size_t src_changed = 0;
    for (uint32_t i = 0; i < kCount; ++i) {
        stale += bits(dst[i]) != bits(src[i] + 1.0f) ? 1u : 0u;
        src_changed += bits(src_back[i]) != bits(src[i]) ? 1u : 0u;
    }
    std::printf("[GpuAirflowRunner] copy then add-one: %u floats, wrong=%zu, src changed=%zu\n", kCount, stale,
                src_changed);
    EXPECT_EQ(stale, std::size_t{0});
    EXPECT_EQ(src_changed, std::size_t{0});
}

// One recording, five submits: exactly five increments, and still one chain.
TEST(GpuAirflowRunner, RecordOnceSubmitFiveTimesAddsFive) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    constexpr uint32_t kCount = 100'003;
    std::vector<float> x(kCount);
    for (uint32_t i = 0; i < kCount; ++i) x[i] = static_cast<float>(i % 1000u);

    // add_one works on airflow_v alone; params and u stay placeholders.
    std::vector<RunnerBuffer> buffers = airflow_set(kV);
    buffers[kV] = {"airflow_v", kCount * 4ull, BufferKind::staged};
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_add_one};
    auto runner = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                       .push_bytes = kPushBytes, .workgroup_size = kWorkgroupSize});
    ASSERT_TRUE(runner.has_value()) << runner.error().context;
    ASSERT_TRUE((*runner)->upload(kV, bytes_of(x)).has_value());

    const ChainCommand chain[] = {dispatch(0, count_push(kCount), {groups_for(kCount, kWorkgroupSize), 1, 1})};
    auto id = (*runner)->record(chain);
    ASSERT_TRUE(id.has_value()) << id.error().context;
    auto submitted = (*runner)->submit(*id, 5);
    ASSERT_TRUE(submitted.has_value()) << submitted.error().context;
    EXPECT_EQ((*runner)->chain_count(), 1u);

    std::vector<float> back(kCount, 0.0f);
    ASSERT_TRUE((*runner)->read(kV, writable_bytes_of(back)).has_value());
    std::size_t wrong = 0;
    for (uint32_t i = 0; i < kCount; ++i) wrong += bits(back[i]) != bits(x[i] + 5.0f) ? 1u : 0u;
    std::printf("[GpuAirflowRunner] record once, submit 5: %u floats, wrong=%zu (x[1]: %g -> %g)\n", kCount, wrong,
                static_cast<double>(x[1]), static_cast<double>(back[1]));
    EXPECT_EQ(wrong, std::size_t{0});
}

// One buffer of maxStorageBufferRange + 4 bytes: refused by that limit's name,
// with nothing allocated. And where the device's maxMemoryAllocationSize is the
// smaller limit, a buffer just past it is refused by that name.
TEST(GpuAirflowRunner, OversizedBufferIsRefusedByNameBeforeAnyAllocation) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const DeviceLimits& limits = limits_of(*ctx);
    std::printf("[GpuAirflowRunner] maxStorageBufferRange=%" PRIu64 " maxMemoryAllocationSize=%" PRIu64 "\n",
                limits.max_storage_buffer_range, limits.max_memory_allocation_size);
    ASSERT_GT(limits.max_storage_buffer_range, 0u);

    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy};
    const uint64_t asked = limits.max_storage_buffer_range + 4;
    const RunnerBuffer big[] = {{"big", asked, BufferKind::staged}};
    const spade::airflow_gpu::RunnerMemoryStats before = spade::airflow_gpu::runner_memory_stats();
    auto refused = ChainRunner::create(*ctx, RunnerDesc{.buffers = big, .kernels = kernels,
                                                        .push_bytes = kPushBytes, .workgroup_size = kWorkgroupSize});
    const spade::airflow_gpu::RunnerMemoryStats after = spade::airflow_gpu::runner_memory_stats();
    ASSERT_FALSE(refused.has_value());
    std::printf("[GpuAirflowRunner] refused: %s\n", refused.error().context.c_str());
    EXPECT_EQ(refused.error().code, Code::capacity_exceeded);
    EXPECT_TRUE(contains(refused.error().context, "maxStorageBufferRange")) << refused.error().context;
    EXPECT_TRUE(contains(refused.error().context, std::to_string(asked))) << refused.error().context;
    EXPECT_TRUE(contains(refused.error().context, "big")) << refused.error().context;
    EXPECT_EQ(after.allocations, before.allocations) << "an allocation was made before the refusal";
    EXPECT_EQ(after.live_bytes, before.live_bytes);

    if (limits.max_memory_allocation_size == 0 ||
        limits.max_memory_allocation_size >= limits.max_storage_buffer_range) {
        std::printf("[GpuAirflowRunner] maxMemoryAllocationSize is not the smaller limit here; its case is not "
                    "reachable on this device\n");
        return;
    }
    const uint64_t over_allocation = limits.max_memory_allocation_size + 4;
    const RunnerBuffer too_big[] = {{"too_big", over_allocation, BufferKind::device_only}};
    auto refused_allocation = ChainRunner::create(*ctx, RunnerDesc{.buffers = too_big, .kernels = kernels,
                                                                   .push_bytes = kPushBytes,
                                                                   .workgroup_size = kWorkgroupSize});
    ASSERT_FALSE(refused_allocation.has_value());
    std::printf("[GpuAirflowRunner] refused: %s\n", refused_allocation.error().context.c_str());
    EXPECT_EQ(refused_allocation.error().code, Code::capacity_exceeded);
    EXPECT_TRUE(contains(refused_allocation.error().context, "maxMemoryAllocationSize"))
        << refused_allocation.error().context;
    EXPECT_TRUE(contains(refused_allocation.error().context, std::to_string(over_allocation)))
        << refused_allocation.error().context;
    EXPECT_EQ(spade::airflow_gpu::runner_memory_stats().allocations, before.allocations);
}

// Buffers each inside every per-buffer limit whose total passes the budget:
// refused by the budget's name, with nothing allocated. The budget is
// runner_budget_bytes(), min(4 GiB, budget - usage - 1 GiB reserve) from
// memory_budget(); create() reads it again and it may have moved (other
// programs allocate), so the message is held to the total and the reserve,
// not to this reading of the cap. The total passes the cap by up to one slab,
// far more than a desktop's VRAM use moves between the two readings.
TEST(GpuAirflowRunner, OverBudgetTotalIsRefusedByName) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const DeviceLimits& limits = limits_of(*ctx);
    const spade::compute::MemoryBudget memory = ctx->memory_budget();
    const uint64_t budget = spade::airflow_gpu::runner_budget_bytes(*ctx);
    std::printf("[GpuAirflowRunner] memory_budget: budget=%" PRIu64 " usage=%" PRIu64 " source=%s; "
                "runner_budget_bytes=%" PRIu64 "\n",
                memory.budget, memory.usage,
                memory.source == spade::compute::MemoryBudget::Source::ext ? "VK_EXT_memory_budget" : "half heap",
                budget);
    EXPECT_LE(budget, spade::compute::kAirflowVramCap);

    uint64_t each = limits.max_storage_buffer_range;
    if (limits.max_memory_allocation_size != 0) each = std::min(each, limits.max_memory_allocation_size);
    each &= ~uint64_t{3};
    ASSERT_GT(each, 0u);
    const uint64_t count = budget / each + 1;
    if (count > limits.max_per_stage_storage_buffers || count > limits.max_set_storage_buffers) {
        GTEST_SKIP() << "the budget needs " << count << " buffers of " << each
                     << " bytes, past this device's descriptor limits";
    }

    std::vector<RunnerBuffer> buffers;
    for (uint64_t i = 0; i < count; ++i) {
        buffers.push_back(RunnerBuffer{"slab" + std::to_string(i), each, BufferKind::device_only});
    }
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy};
    const spade::airflow_gpu::RunnerMemoryStats before = spade::airflow_gpu::runner_memory_stats();
    auto refused = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                        .push_bytes = kPushBytes, .workgroup_size = kWorkgroupSize});
    ASSERT_FALSE(refused.has_value());
    std::printf("[GpuAirflowRunner] refused: %s\n", refused.error().context.c_str());
    EXPECT_EQ(refused.error().code, Code::capacity_exceeded);
    EXPECT_TRUE(contains(refused.error().context, "runner_budget_bytes")) << refused.error().context;
    EXPECT_TRUE(contains(refused.error().context, "budget")) << refused.error().context;
    EXPECT_TRUE(contains(refused.error().context, std::to_string(each * count))) << refused.error().context;
    EXPECT_TRUE(contains(refused.error().context, "reserve " + std::to_string(spade::airflow_gpu::kRunnerReserveBytes)))
        << refused.error().context;
    EXPECT_EQ(spade::airflow_gpu::runner_memory_stats().allocations, before.allocations);
}

// push_bytes past maxPushConstantsSize, and past kMaxChainPushBytes (the block
// a ChainCommand carries) where that is the smaller: each refused by name.
TEST(GpuAirflowRunner, PushSizeOverTheLimitIsRefusedByName) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const DeviceLimits& limits = limits_of(*ctx);

    const std::vector<RunnerBuffer> buffers = copy_buffers(64);
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy};
    const spade::airflow_gpu::RunnerMemoryStats before = spade::airflow_gpu::runner_memory_stats();

    const uint32_t over_device = limits.max_push_constants_size + 4;
    auto refused = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                        .push_bytes = over_device, .workgroup_size = kWorkgroupSize});
    ASSERT_FALSE(refused.has_value());
    std::printf("[GpuAirflowRunner] refused: %s\n", refused.error().context.c_str());
    EXPECT_EQ(refused.error().code, Code::capacity_exceeded);
    EXPECT_TRUE(contains(refused.error().context, "maxPushConstantsSize")) << refused.error().context;
    EXPECT_TRUE(contains(refused.error().context, std::to_string(over_device))) << refused.error().context;

    const uint32_t over_chain = spade::compute::kMaxChainPushBytes + 4;
    if (over_chain <= limits.max_push_constants_size) {
        auto refused_chain = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                                  .push_bytes = over_chain,
                                                                  .workgroup_size = kWorkgroupSize});
        ASSERT_FALSE(refused_chain.has_value());
        std::printf("[GpuAirflowRunner] refused: %s\n", refused_chain.error().context.c_str());
        EXPECT_EQ(refused_chain.error().code, Code::capacity_exceeded);
        EXPECT_TRUE(contains(refused_chain.error().context, "kMaxChainPushBytes")) << refused_chain.error().context;
    }
    EXPECT_EQ(spade::airflow_gpu::runner_memory_stats().allocations, before.allocations);
}

// A dispatch with one more group than maxComputeWorkGroupCount allows on its
// tightest axis: record() refuses by name and records nothing, so the next
// good chain is chain 0.
TEST(GpuAirflowRunner, GroupsOverTheLimitAreRefusedByName) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const DeviceLimits& limits = limits_of(*ctx);

    const std::vector<RunnerBuffer> buffers = copy_buffers(256);
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy};
    auto runner = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                       .push_bytes = kPushBytes, .workgroup_size = kWorkgroupSize});
    ASSERT_TRUE(runner.has_value()) << runner.error().context;

    std::size_t axis = 0;
    for (std::size_t a = 1; a < 3; ++a) {
        if (limits.max_group_count[a] < limits.max_group_count[axis]) axis = a;
    }
    if (limits.max_group_count[axis] == std::numeric_limits<uint32_t>::max()) {
        GTEST_SKIP() << "maxComputeWorkGroupCount is UINT32_MAX on every axis: no count can exceed it";
    }
    std::array<uint32_t, 3> groups{1, 1, 1};
    groups[axis] = limits.max_group_count[axis] + 1;
    const ChainCommand too_wide[] = {dispatch(0, count_push(64), groups)};
    const spade::airflow_gpu::RunnerMemoryStats before = spade::airflow_gpu::runner_memory_stats();
    auto refused = (*runner)->record(too_wide);
    ASSERT_FALSE(refused.has_value());
    std::printf("[GpuAirflowRunner] refused: %s\n", refused.error().context.c_str());
    EXPECT_EQ(refused.error().code, Code::capacity_exceeded);
    EXPECT_TRUE(contains(refused.error().context, "maxComputeWorkGroupCount")) << refused.error().context;
    EXPECT_TRUE(contains(refused.error().context, std::to_string(groups[axis]))) << refused.error().context;
    EXPECT_EQ((*runner)->chain_count(), 0u);
    EXPECT_EQ(spade::airflow_gpu::runner_memory_stats().allocations, before.allocations);

    const ChainCommand fine[] = {dispatch(0, count_push(64), {1, 1, 1})};
    auto id = (*runner)->record(fine);
    ASSERT_TRUE(id.has_value()) << id.error().context;
    EXPECT_EQ(*id, 0u);
}

// A dispatch naming a pipeline the runner does not have: refused by name.
TEST(GpuAirflowRunner, PipelineIndexOutOfRangeIsRefusedByName) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    const std::vector<RunnerBuffer> buffers = copy_buffers(256);
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy};
    auto runner = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                       .push_bytes = kPushBytes, .workgroup_size = kWorkgroupSize});
    ASSERT_TRUE(runner.has_value()) << runner.error().context;

    const ChainCommand chain[] = {dispatch(0, count_push(64), {1, 1, 1}), dispatch(1, count_push(64), {1, 1, 1})};
    auto refused = (*runner)->record(chain);
    ASSERT_FALSE(refused.has_value());
    std::printf("[GpuAirflowRunner] refused: %s\n", refused.error().context.c_str());
    EXPECT_EQ(refused.error().code, Code::invalid_argument);
    EXPECT_TRUE(contains(refused.error().context, "pipeline index 1")) << refused.error().context;
    EXPECT_EQ((*runner)->chain_count(), 0u);
}

// A workgroup size the build compiled no variant for: refused by name, before
// any allocation (the selection make_compute_pipeline would refuse later).
TEST(GpuAirflowRunner, WorkgroupSizeWithoutAVariantIsRefusedByName) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    const std::vector<RunnerBuffer> buffers = copy_buffers(256);
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy};
    const spade::airflow_gpu::RunnerMemoryStats before = spade::airflow_gpu::runner_memory_stats();
    auto refused = ChainRunner::create(
        *ctx, RunnerDesc{.buffers = buffers, .kernels = kernels, .push_bytes = kPushBytes, .workgroup_size = 256});
    ASSERT_FALSE(refused.has_value());
    std::printf("[GpuAirflowRunner] refused: %s\n", refused.error().context.c_str());
    EXPECT_EQ(refused.error().code, Code::invalid_argument);
    EXPECT_TRUE(contains(refused.error().context, "airflow_copy")) << refused.error().context;
    EXPECT_TRUE(contains(refused.error().context, "256")) << refused.error().context;
    EXPECT_EQ(spade::airflow_gpu::runner_memory_stats().allocations, before.allocations);
}

// Three marks around two dispatches: three durations on a device that times
// compute work, none on one that cannot, and none before the first submit or
// for a chain with no marks. Never an error.
TEST(GpuAirflowRunner, DurationsArePerMarkOrEmpty) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    // Can this device time compute work? timestamps.hpp's two checks.
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(ctx->physical_device(), &properties);
    uint32_t family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(ctx->physical_device(), &family_count, nullptr);
    std::vector<VkQueueFamilyProperties> families(family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(ctx->physical_device(), &family_count, families.data());
    const bool times_compute = properties.limits.timestampComputeAndGraphics == VK_TRUE &&
                               ctx->compute_queue_family() < families.size() &&
                               families[ctx->compute_queue_family()].timestampValidBits > 0;

    constexpr uint32_t kCount = 1u << 16;
    const std::vector<RunnerBuffer> buffers = copy_buffers(kCount * 4ull);
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy, &gen::kSpvVariants_airflow_add_one};
    auto runner = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                       .push_bytes = kPushBytes, .workgroup_size = kWorkgroupSize});
    ASSERT_TRUE(runner.has_value()) << runner.error().context;

    const CountPush push = count_push(kCount);
    const uint32_t groups = groups_for(kCount, kWorkgroupSize);
    const ChainCommand marked[] = {mark(), dispatch(0, push, {groups, 1, 1}), mark(),
                                   dispatch(1, push, {groups, 1, 1}), mark()};
    const ChainCommand unmarked[] = {dispatch(0, push, {groups, 1, 1})};
    auto marked_id = (*runner)->record(marked);
    ASSERT_TRUE(marked_id.has_value()) << marked_id.error().context;
    auto unmarked_id = (*runner)->record(unmarked);
    ASSERT_TRUE(unmarked_id.has_value()) << unmarked_id.error().context;

    auto before_submit = (*runner)->durations_ns(*marked_id);
    ASSERT_TRUE(before_submit.has_value()) << before_submit.error().context;
    EXPECT_TRUE(before_submit->empty()) << "durations before the chain ever ran";

    ASSERT_TRUE((*runner)->submit(*marked_id, 1).has_value());
    ASSERT_TRUE((*runner)->submit(*unmarked_id, 1).has_value());

    auto durations = (*runner)->durations_ns(*marked_id);
    ASSERT_TRUE(durations.has_value()) << durations.error().context;
    std::printf("[GpuAirflowRunner] device times compute: %s; durations:", times_compute ? "yes" : "no");
    for (const uint64_t ns : *durations) std::printf(" %" PRIu64 " ns", ns);
    std::printf("\n");
    if (times_compute) {
        EXPECT_EQ(durations->size(), 3u);
    } else {
        EXPECT_TRUE(durations->empty());
    }

    auto none = (*runner)->durations_ns(*unmarked_id);
    ASSERT_TRUE(none.has_value()) << none.error().context;
    EXPECT_TRUE(none->empty());
}

// ===========================================================================
// GpuAirflowRunnerTileSum -- A4
// ===========================================================================

// airflow_scalars' slots in this test (the solver's are [0] remove_mean's
// sum, [1-7] CG's dots): each field's total in a slot of its own, every other
// slot left at a sentinel, so a total written to the wrong slot shows.
constexpr uint32_t kScalarSlots = 8;
constexpr std::array<uint32_t, 2> kTotalSlot{3, 5};
constexpr float kScalarSentinel = -12345.0f;

// The fields airflow_tile_sum reads, by its push's `field`.
constexpr std::array<const char*, 2> kFieldName{"airflow_pressure", "airflow_rhs"};

struct TileSumRun {
    std::array<std::vector<float>, 2> tiles;  // airflow_partials after each field's sum
    std::vector<float> scalars;               // airflow_scalars after both
};

// The two-dispatch tile sum of airflow_pressure (field 0), then of
// airflow_rhs (field 1), at `workgroup_size`: one chain per field, each pass
// 0 then pass 1 with the same push but for the pass, since airflow_partials
// is shared by the reductions in turn.
[[nodiscard]] TileSumRun run_tile_sum(VulkanContext& ctx, const std::array<std::vector<float>, 2>& fields,
                                      uint32_t nx, uint32_t ny, uint32_t nz, uint32_t workgroup_size) {
    const uint32_t tiles = ((nx + 3) / 4) * ((ny + 3) / 4) * ((nz + 3) / 4);
    const uint32_t cells = nx * ny * nz;

    // airflow_params: its dims, words 0-2 (engine/shaders/airflow_params_layout.slang),
    // and nothing else this kernel reads.
    std::vector<uint32_t> params(32, 0u);
    params[0] = nx;
    params[1] = ny;
    params[2] = nz;

    std::vector<RunnerBuffer> buffers = airflow_set(kPartials);
    buffers[kParams] = {"airflow_params", params.size() * 4ull, BufferKind::staged};
    buffers[kPressure] = {"airflow_pressure", cells * 4ull, BufferKind::staged};
    buffers[kRhs] = {"airflow_rhs", cells * 4ull, BufferKind::staged};
    buffers[kScalars] = {"airflow_scalars", kScalarSlots * 4ull, BufferKind::staged};
    buffers[kPartials] = {"airflow_partials", tiles * 4ull, BufferKind::staged};
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_tile_sum};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                      .push_bytes = kPushBytes, .workgroup_size = workgroup_size});
    EXPECT_TRUE(runner.has_value()) << runner.error().context;
    TileSumRun out;
    for (std::vector<float>& t : out.tiles) t.assign(tiles, std::numeric_limits<float>::quiet_NaN());
    out.scalars.assign(kScalarSlots, std::numeric_limits<float>::quiet_NaN());
    if (!runner.has_value()) return out;

    const std::vector<float> sentinel(kScalarSlots, kScalarSentinel);
    EXPECT_TRUE((*runner)->upload(kParams, bytes_of(params)).has_value());
    EXPECT_TRUE((*runner)->upload(kPressure, bytes_of(fields[0])).has_value());
    EXPECT_TRUE((*runner)->upload(kRhs, bytes_of(fields[1])).has_value());
    EXPECT_TRUE((*runner)->upload(kScalars, bytes_of(sentinel)).has_value());

    for (uint32_t field = 0; field < 2; ++field) {
        const TileSumPush tiles_pass{.field = field, .pass = 0, .cell_count = cells, .slot = kTotalSlot[field]};
        const TileSumPush total_pass{.field = field, .pass = 1, .cell_count = cells, .slot = kTotalSlot[field]};
        const ChainCommand chain[] = {dispatch(0, tiles_pass, {groups_for(tiles, workgroup_size), 1, 1}),
                                      dispatch(0, total_pass, {1, 1, 1})};
        auto id = (*runner)->record(chain);
        EXPECT_TRUE(id.has_value()) << id.error().context;
        if (!id.has_value()) return out;
        auto submitted = (*runner)->submit(*id, 1);
        EXPECT_TRUE(submitted.has_value()) << submitted.error().context;
        EXPECT_TRUE((*runner)->read(kPartials, writable_bytes_of(out.tiles[field])).has_value());
    }
    EXPECT_TRUE((*runner)->read(kScalars, writable_bytes_of(out.scalars)).has_value());
    return out;
}

// One tile's own sum per tile, in airflow_partials' order: a tile is a
// fixed_tile_sum_of over its own box, one tile, added to 0.
[[nodiscard]] std::vector<float> cpu_tile_sums(const std::vector<float>& field, uint32_t nx, uint32_t ny,
                                               uint32_t nz) {
    std::vector<float> out;
    for (uint32_t tk = 0; tk < nz; tk += 4) {
        for (uint32_t tj = 0; tj < ny; tj += 4) {
            for (uint32_t ti = 0; ti < nx; ti += 4) {
                const uint32_t bx = std::min(4u, nx - ti);
                const uint32_t by = std::min(4u, ny - tj);
                const uint32_t bz = std::min(4u, nz - tk);
                out.push_back(spade::physics::airflow::fixed_tile_sum_of<float>(
                    bx, by, bz, [&](uint32_t i, uint32_t j, uint32_t k, std::size_t) {
                        return field[(ti + i) + std::size_t{nx} * ((tj + j) + std::size_t{ny} * (tk + k))];
                    }));
            }
        }
    }
    return out;
}

void check_tile_sum(uint32_t nx, uint32_t ny, uint32_t nz, uint64_t seed) {
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);

    // Two fields from two streams of the seed: what the kernel reads for
    // field 0 and for field 1 differ in every cell, so reading the wrong one
    // shows.
    const std::size_t cells = std::size_t{nx} * ny * nz;
    std::array<std::vector<float>, 2> fields;
    std::array<float, 2> cpu{};
    std::array<std::vector<float>, 2> cpu_tiles;
    for (uint32_t f = 0; f < 2; ++f) {
        SplitMix rng{seed + f};
        fields[f].resize(cells);
        for (float& v : fields[f]) v = mixed_magnitude(rng.next());

        // The CPU reference: the total, and each tile's own sum.
        cpu[f] = spade::physics::airflow::fixed_tile_sum(fields[f], nx, ny, nz);
        cpu_tiles[f] = cpu_tile_sums(fields[f], nx, ny, nz);

        // The data is order-sensitive: the plain array-order sum differs, so
        // a GPU that summed in any other order would show.
        float plain = 0.0f;
        for (const float v : fields[f]) plain += v;
        EXPECT_NE(bits(plain), bits(cpu[f])) << "the test data does not tell orders apart; change the seed";
        std::printf("[GpuAirflowRunnerTileSum] %ux%ux%u %s: cpu=%.9g (0x%08x), array order=%.9g (0x%08x)\n", nx, ny,
                    nz, kFieldName[f], static_cast<double>(cpu[f]), bits(cpu[f]), static_cast<double>(plain),
                    bits(plain));
    }
    EXPECT_NE(bits(cpu[0]), bits(cpu[1]));

    for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
        const TileSumRun gpu = run_tile_sum(*ctx, fields, nx, ny, nz, wg);
        for (uint32_t f = 0; f < 2; ++f) {
            std::size_t tile_mismatches = 0;
            for (std::size_t t = 0; t < cpu_tiles[f].size(); ++t) {
                tile_mismatches += bits(gpu.tiles[f][t]) != bits(cpu_tiles[f][t]) ? 1u : 0u;
            }
            const float total = gpu.scalars[kTotalSlot[f]];
            std::printf("[GpuAirflowRunnerTileSum] %ux%ux%u wg %u %s: gpu=%.9g (0x%08x) in slot %u, tiles %zu, "
                        "tile mismatches=%zu\n",
                        nx, ny, nz, wg, kFieldName[f], static_cast<double>(total), bits(total), kTotalSlot[f],
                        cpu_tiles[f].size(), tile_mismatches);
            EXPECT_EQ(bits(total), bits(cpu[f])) << kFieldName[f] << ", workgroup size " << wg;
            EXPECT_EQ(tile_mismatches, std::size_t{0}) << kFieldName[f] << ", workgroup size " << wg;
        }
        std::size_t disturbed = 0;
        for (uint32_t s = 0; s < kScalarSlots; ++s) {
            if (s == kTotalSlot[0] || s == kTotalSlot[1]) continue;
            disturbed += bits(gpu.scalars[s]) != bits(kScalarSentinel) ? 1u : 0u;
        }
        EXPECT_EQ(disturbed, std::size_t{0}) << "a slot other than the two totals' was written, workgroup size " << wg;
    }
}

// Sides that are not multiples of 4: partial tiles on every far edge.
TEST(GpuAirflowRunnerTileSum, RaggedShapeSameBitsAtEveryWorkgroupSizeAndAsTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    check_tile_sum(5, 7, 9, 0x7115E5ull);
}

TEST(GpuAirflowRunnerTileSum, CubeSameBitsAtEveryWorkgroupSizeAndAsTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    check_tile_sum(32, 32, 32, 0x32C0BEull);
}

// ===========================================================================
// AirflowRunnerSpirv -- the compiled modules, no device needed
// ===========================================================================

struct OpCensus {
    std::size_t atomics = 0;         // OpAtomic*
    std::size_t barriers = 0;        // OpControlBarrier, OpMemoryBarrier
    std::size_t workgroup_vars = 0;  // OpVariable in the Workgroup storage class
    std::vector<uint32_t> sets;      // each DescriptorSet decoration's set
};

[[nodiscard]] OpCensus census(std::span<const uint32_t> words) {
    OpCensus out;
    std::size_t i = 5;  // past the header
    while (i < words.size()) {
        const uint32_t count = words[i] >> 16;
        const uint32_t op = words[i] & 0xFFFFu;
        if (count == 0 || i + count > words.size()) break;
        if ((op >= 227 && op <= 242) || op == 318 || op == 319 || op == 5614 || op == 5615 || op == 6035) {
            ++out.atomics;  // OpAtomicLoad..OpAtomicXor, the flag ops, the float min/max/add extensions
        }
        if (op == 224 || op == 225) ++out.barriers;
        if (op == 59 && count >= 4 && words[i + 3] == 4) ++out.workgroup_vars;  // OpVariable, StorageClass Workgroup
        if (op == 71 && count >= 4 && words[i + 2] == 34) out.sets.push_back(words[i + 3]);  // OpDecorate DescriptorSet
        i += count;
    }
    return out;
}

TEST(AirflowRunnerSpirv, KernelsHoldTheDeterminismRules) {
    struct Kernel {
        const SpirvVariantSet& variants;
        bool integer_only;  // no float arithmetic: integer work, or a copy's loads and stores
    };
    const Kernel kernels[] = {{gen::kSpvVariants_airflow_copy, true},
                              {gen::kSpvVariants_airflow_add_one, false},
                              {gen::kSpvVariants_airflow_tile_sum, false},
                              // B2's projection kernels (Rendering). The smoother's one OpFDiv is
                              // the parity profile's to allow: it is banded, not exact.
                              {gen::kSpvVariants_airflow_divergence, false},
                              {gen::kSpvVariants_airflow_gradient_subtract, false},
                              {gen::kSpvVariants_airflow_mg_smooth, false},
                              {gen::kSpvVariants_airflow_mg_residual, false},
                              {gen::kSpvVariants_airflow_mg_restrict, false},
                              {gen::kSpvVariants_airflow_mg_prolong, false},
                              {gen::kSpvVariants_airflow_remove_mean, false},
                              // B3's advection kernel (Rendering).
                              {gen::kSpvVariants_airflow_advection, false},
                              // B6's publish kernels (Rendering): the pack is bit work on the
                              // channels' words.
                              {gen::kSpvVariants_airflow_publish_fields, false},
                              {gen::kSpvVariants_airflow_pack_f16, true},
                              // B4's rotor discs (Rendering).
                              {gen::kSpvVariants_airflow_actuator_gather, false}};
    for (const Kernel& k : kernels) {
        ASSERT_EQ(k.variants.count, spade::compute::kSupportedWorkgroupSizes.size()) << k.variants.name;
        for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
            EXPECT_NE(k.variants.for_size(wg), nullptr) << k.variants.name << " has no variant for " << wg;
        }
        for (const spade::compute::SpirvVariant& v : k.variants.all()) {
            const std::string module = std::string(k.variants.name) + " (local size " +
                                       std::to_string(v.workgroup_size) + ")";
            const spade::testing::SpirvScanResult scan =
                spade::testing::scan_spirv(v.code(), spade::testing::SpirvProfile::parity);
            EXPECT_TRUE(scan.well_formed) << module;
            EXPECT_TRUE(scan.findings.empty()) << module << ": " << scan.findings.size() << " policy findings";
            EXPECT_EQ(scan.decorated_ops, scan.contractable_ops) << module << ": an op lacks NoContraction";
            if (k.integer_only) {
                EXPECT_EQ(scan.contractable_ops, 0u) << module;
                EXPECT_GT(scan.store_count, 0u) << module;
            } else {
                EXPECT_GT(scan.contractable_ops, 0u) << module << ": no float op; is this the right module?";
            }
            EXPECT_FALSE(scan.requests_denorm_mode()) << module;
            EXPECT_FALSE(scan.declares_int64) << module;
            EXPECT_EQ(scan.entry_point_name, "main") << module;

            const OpCensus ops = census(v.code());
            EXPECT_EQ(ops.atomics, 0u) << module << ": atomics";
            EXPECT_EQ(ops.barriers, 0u) << module << ": a barrier inside the kernel";
            EXPECT_EQ(ops.workgroup_vars, 0u) << module << ": groupshared memory";
            // Every buffer in the airflow set, which the runner binds (Core's B0 plan).
            EXPECT_FALSE(ops.sets.empty()) << module << ": no descriptor declared";
            for (const uint32_t set : ops.sets) {
                EXPECT_EQ(set, spade::airflow_gpu::kAirflowSet) << module << ": a binding outside the airflow set";
            }
        }

        // The variants differ in one word, the LocalSize operand: nothing the
        // kernel computes depends on SPADE_WG.
        const std::span<const spade::compute::SpirvVariant> all = k.variants.all();
        for (std::size_t a = 0; a + 1 < all.size(); ++a) {
            const std::span<const uint32_t> x = all[a].code();
            const std::span<const uint32_t> y = all[a + 1].code();
            ASSERT_EQ(x.size(), y.size()) << k.variants.name;
            std::size_t differ = 0;
            std::size_t where = 0;
            for (std::size_t w = 0; w < x.size(); ++w) {
                if (x[w] != y[w]) {
                    ++differ;
                    where = w;
                }
            }
            EXPECT_EQ(differ, 1u) << k.variants.name;
            if (differ == 1) {
                EXPECT_EQ(x[where], all[a].workgroup_size) << k.variants.name;
                EXPECT_EQ(y[where], all[a + 1].workgroup_size) << k.variants.name;
            }
        }
    }
}


// ===========================================================================
// AirflowRunnerStaging -- the staging buffer's memory type, no device needed
// ===========================================================================

// A memory-properties table holding these types, in this order, on one heap.
[[nodiscard]] VkPhysicalDeviceMemoryProperties memory_types(std::initializer_list<VkMemoryPropertyFlags> flags) {
    VkPhysicalDeviceMemoryProperties props{};
    props.memoryHeapCount = 1;
    for (const VkMemoryPropertyFlags f : flags) {
        props.memoryTypes[props.memoryTypeCount].propertyFlags = f;
        props.memoryTypes[props.memoryTypeCount].heapIndex = 0;
        ++props.memoryTypeCount;
    }
    return props;
}

// kStagingMemoryPreferences over the tables a device may report (each in
// Vulkan's order: a type whose flags are a strict subset of another's comes
// first): cached and coherent where the buffer may use it, else coherent, and
// never a cached type that is not coherent.
TEST(AirflowRunnerStaging, PrefersCachedCoherentThenCoherentNeverCachedAlone) {
    using spade::airflow_gpu::choose_memory_type;
    using spade::airflow_gpu::kStagingMemoryPreferences;
    constexpr VkMemoryPropertyFlags kDevice = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
    constexpr VkMemoryPropertyFlags kCoherent =
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    constexpr VkMemoryPropertyFlags kCachedCoherent = kCoherent | VK_MEMORY_PROPERTY_HOST_CACHED_BIT;
    constexpr VkMemoryPropertyFlags kCachedOnly =
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT;

    // A discrete GPU's table (the 3060 Ti's shape): device-local, coherent,
    // cached coherent, and the device-local host-visible window.
    const VkPhysicalDeviceMemoryProperties desktop =
        memory_types({kDevice, kCoherent, kCachedCoherent, kDevice | kCoherent});
    EXPECT_EQ(choose_memory_type(desktop, 0xFu, kStagingMemoryPreferences), std::optional<uint32_t>{2u});
    // The buffer may not use the cached type: the coherent one.
    EXPECT_EQ(choose_memory_type(desktop, 0xBu, kStagingMemoryPreferences), std::optional<uint32_t>{1u});

    // No cached type: the first coherent one, not the device-local window.
    const VkPhysicalDeviceMemoryProperties uncached = memory_types({kDevice, kCoherent, kDevice | kCoherent});
    EXPECT_EQ(choose_memory_type(uncached, 0x7u, kStagingMemoryPreferences), std::optional<uint32_t>{1u});

    // A cached type that is not coherent is passed over for a coherent one.
    const VkPhysicalDeviceMemoryProperties cached_alone = memory_types({kDevice, kCachedOnly, kCoherent});
    EXPECT_EQ(choose_memory_type(cached_alone, 0x7u, kStagingMemoryPreferences), std::optional<uint32_t>{2u});

    // An integrated GPU's table (the Iris Plus's shape: every type device-local).
    const VkPhysicalDeviceMemoryProperties integrated =
        memory_types({kDevice, kDevice | kCoherent, kDevice | kCachedCoherent});
    EXPECT_EQ(choose_memory_type(integrated, 0x7u, kStagingMemoryPreferences), std::optional<uint32_t>{2u});

    // Nothing host-visible and coherent that the buffer may use: none.
    const VkPhysicalDeviceMemoryProperties none = memory_types({kDevice, kCachedOnly});
    EXPECT_FALSE(choose_memory_type(none, 0x3u, kStagingMemoryPreferences).has_value());
    EXPECT_FALSE(choose_memory_type(desktop, 0x1u, kStagingMemoryPreferences).has_value());
}

}  // namespace
