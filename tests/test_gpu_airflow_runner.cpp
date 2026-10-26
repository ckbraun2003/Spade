// ---------------------------------------------------------------------------
// test_gpu_airflow_runner.cpp -- plan B's first task (B1): the airflow chain
// runner (spike/airflow/chain_runner.hpp) and its three test kernels, before
// any solver kernel (docs/design/physics/plans/2026-10-12-airflow-chain-
// runner-design.md §4; plan B, 2026-10-22-airflow-gpu-solver-plan.md, B1).
//
//   GpuAirflowRunner          upload/read exact; the barrier; record once,
//                             submit n; each refusal by name, before any
//                             allocation; timestamps per mark or empty
//   GpuAirflowRunnerTileSum   A4: the fixed-tile sum, in B5.1's two-level
//                             order, gives the same bits at workgroup sizes
//                             32, 64 and 128, and the same bits as
//                             physics/airflow/reduce.hpp on the CPU; and,
//                             opt-in (SPADE_AIRFLOW_TIMINGS=1), B5.0's times
//                             of its two passes and of a word's readback,
//                             and B5.0b's: the last pass with loads issued
//                             ahead, and a two-level order and its
//                             invariance (a test-only probe kernel)
//   AirflowRunnerSpirv       the kernels' compiled modules (plan B's): no
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
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>

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
#include "airflow_tile_sum_probe.spv.gen.hpp"
#include "airflow_velocity_bc.spv.gen.hpp"
#include "airflow_viscous.spv.gen.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "core/detail/flush_subnormals.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pcg.hpp"
#include "physics/airflow/pressure.hpp"
#include "physics/airflow/reduce.hpp"
#include "physics/airflow/regions.hpp"
#include "reference_airframe.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "spike/airflow/projection_chain.hpp"
#include "state/layout.hpp"
#include "testing/spirv_scan.hpp"
#include "vehicles/airflow_solids.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/part_pose.hpp"
#include "vehicles/quadrotor.hpp"

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

// The push block of airflow_tile_sum. Word 1 is projection_chain.hpp's
// tile_sum_pass_word(): the pass, and the block in tiles.
struct TileSumPush {
    uint32_t field = 0;       // 0 airflow_pressure, 1 airflow_rhs
    uint32_t pass_block = 0;  // the pass (0 the tiles, 1 the blocks, 2 the total) and the block
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
// GpuAirflowRunnerTileSum -- A4, in B5.1's two-level order
// ===========================================================================

// airflow_scalars' slots in this test (the solver's are [0] remove_mean's
// sum, [1-7] CG's dots): each field's total in a slot of its own, every other
// slot left at a sentinel, so a total written to the wrong slot shows.
constexpr uint32_t kScalarSlots = 8;
constexpr std::array<uint32_t, 2> kTotalSlot{3, 5};
constexpr float kScalarSentinel = -12345.0f;

// The fields airflow_tile_sum reads, by its push's `field`.
constexpr std::array<const char*, 2> kFieldName{"airflow_pressure", "airflow_rhs"};

[[nodiscard]] std::string hex_bits(float f) {
    char text[16];
    std::snprintf(text, sizeof(text), "0x%08x", bits(f));
    return text;
}

// Each tile's sum from 0 in (k, j, i) order, the tiles in (tk, tj, ti) order:
// reduce.hpp's tiles, spelled out.
[[nodiscard]] std::vector<float> tile_partials(const std::vector<float>& f, uint32_t nx, uint32_t ny, uint32_t nz) {
    std::vector<float> out;
    for (uint32_t tk = 0; tk < nz; tk += 4) {
        for (uint32_t tj = 0; tj < ny; tj += 4) {
            for (uint32_t ti = 0; ti < nx; ti += 4) {
                float s = 0.0f;
                for (uint32_t k = tk; k < std::min(tk + 4u, nz); ++k) {
                    for (uint32_t j = tj; j < std::min(tj + 4u, ny); ++j) {
                        for (uint32_t i = ti; i < std::min(ti + 4u, nx); ++i) {
                            s += f[i + std::size_t{nx} * (j + std::size_t{ny} * k)];
                        }
                    }
                }
                out.push_back(s);
            }
        }
    }
    return out;
}

// B1's order, before B5.1: the tile sums from 0 in tile order, one chain.
[[nodiscard]] float serial_total(const std::vector<float>& tiles) {
    float total = 0.0f;
    for (const float s : tiles) total += s;
    return total;
}

// Block b's sum: its tiles' sums from 0 in tile order (the last block holds
// the rest).
[[nodiscard]] float block_sum(const std::vector<float>& tiles, std::size_t b, uint32_t block) {
    float s = 0.0f;
    for (std::size_t t = b * block; t < std::min((b + 1) * block, tiles.size()); ++t) s += tiles[t];
    return s;
}

// THE TWO-LEVEL ORDER (reduce.hpp since B5.1), spelled out: blocks of `block`
// consecutive tiles, each from 0 in tile order, then the block sums from 0 in
// block order.
[[nodiscard]] float two_level_total(const std::vector<float>& tiles, uint32_t block) {
    float total = 0.0f;
    for (std::size_t b = 0; b * block < tiles.size(); ++b) total += block_sum(tiles, b, block);
    return total;
}

// airflow_partials after the three passes: each tile's own sum, but block b's
// first tile, which holds the block's sum (pass 1 writes it in place).
[[nodiscard]] std::vector<float> partials_after(const std::vector<float>& tiles, uint32_t block) {
    std::vector<float> out = tiles;
    for (std::size_t b = 0; b * block < tiles.size(); ++b) out[b * block] = block_sum(tiles, b, block);
    return out;
}

struct TileSumRun {
    std::array<std::vector<float>, 2> partials;  // airflow_partials after each field's sum
    std::vector<float> scalars;                  // airflow_scalars after both
};

// The three-dispatch tile sum of airflow_pressure (field 0), then of
// airflow_rhs (field 1), at `workgroup_size`, with `block` tiles a block in
// the push: one chain per field, passes 0, 1 and 2 with the same push but for
// the pass, since airflow_partials is shared by the reductions in turn.
[[nodiscard]] TileSumRun run_tile_sum(VulkanContext& ctx, const std::array<std::vector<float>, 2>& fields,
                                      uint32_t nx, uint32_t ny, uint32_t nz, uint32_t workgroup_size,
                                      uint32_t block) {
    const uint32_t tiles = ((nx + 3) / 4) * ((ny + 3) / 4) * ((nz + 3) / 4);
    const uint32_t blocks = (tiles + block - 1) / block;
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
    buffers[kPartials] = {"airflow_partials", tiles * 4ull, BufferKind::staged};  // one float a tile
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_tile_sum};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers, .kernels = kernels,
                                                      .push_bytes = kPushBytes, .workgroup_size = workgroup_size});
    EXPECT_TRUE(runner.has_value()) << runner.error().context;
    TileSumRun out;
    for (std::vector<float>& t : out.partials) t.assign(tiles, std::numeric_limits<float>::quiet_NaN());
    out.scalars.assign(kScalarSlots, std::numeric_limits<float>::quiet_NaN());
    if (!runner.has_value()) return out;

    const std::vector<float> sentinel(kScalarSlots, kScalarSentinel);
    EXPECT_TRUE((*runner)->upload(kParams, bytes_of(params)).has_value());
    EXPECT_TRUE((*runner)->upload(kPressure, bytes_of(fields[0])).has_value());
    EXPECT_TRUE((*runner)->upload(kRhs, bytes_of(fields[1])).has_value());
    EXPECT_TRUE((*runner)->upload(kScalars, bytes_of(sentinel)).has_value());

    const std::array<uint32_t, 3> threads{tiles, blocks, 1u};
    for (uint32_t field = 0; field < 2; ++field) {
        std::vector<ChainCommand> chain;
        for (uint32_t pass = 0; pass < 3; ++pass) {
            const TileSumPush push{.field = field,
                                   .pass_block = spade::airflow_gpu::tile_sum_pass_word(pass, block),
                                   .cell_count = cells,
                                   .slot = kTotalSlot[field]};
            chain.push_back(dispatch(0, push, {groups_for(threads[pass], workgroup_size), 1, 1}));
        }
        auto id = (*runner)->record(chain);
        EXPECT_TRUE(id.has_value()) << id.error().context;
        if (!id.has_value()) return out;
        auto submitted = (*runner)->submit(*id, 1);
        EXPECT_TRUE(submitted.has_value()) << submitted.error().context;
        EXPECT_TRUE((*runner)->read(kPartials, writable_bytes_of(out.partials[field])).has_value());
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

// A block other than the solver's, pushed by the check below: the kernel
// takes its block from the push, not from a constant of its own.
constexpr uint32_t kOtherBlockTiles = 64;

// THE INVARIANCE (A4; B5.1, as GnssTumble's): at workgroup sizes 32, 64 and
// 128 the GPU's total has one set of bits, reduce.hpp's two-level sum on the
// CPU, and its partials are the CPU's tiles with each block's sum on its
// first tile. Then, at workgroup size 64, the push's block at 64 tiles gives
// the two-level sum at 64.
void check_tile_sum(uint32_t nx, uint32_t ny, uint32_t nz, uint64_t seed) {
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    constexpr uint32_t kBlock = spade::physics::airflow::kReduceBlockTiles;

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
        EXPECT_EQ(bits(two_level_total(cpu_tiles[f], kBlock)), bits(cpu[f]))
            << "the spelled-out two-level order is not reduce.hpp's";

        // The data is order-sensitive: the plain array-order sum differs, and
        // past one block so does B1's chain, so a GPU that summed in any
        // other order would show.
        float plain = 0.0f;
        for (const float v : fields[f]) plain += v;
        const float serial = serial_total(cpu_tiles[f]);
        EXPECT_NE(bits(plain), bits(cpu[f])) << "the test data does not tell orders apart; change the seed";
        if (cpu_tiles[f].size() > kBlock) {
            EXPECT_NE(bits(serial), bits(cpu[f])) << "the data does not tell B1's chain apart; change the seed";
        }
        std::printf("[GpuAirflowRunnerTileSum] %ux%ux%u %s: cpu=%.9g (%s), B1's chain=%.9g (%s), array order=%.9g "
                    "(%s), %zu tiles in %zu blocks of %u\n",
                    nx, ny, nz, kFieldName[f], static_cast<double>(cpu[f]), hex_bits(cpu[f]).c_str(),
                    static_cast<double>(serial), hex_bits(serial).c_str(), static_cast<double>(plain),
                    hex_bits(plain).c_str(), cpu_tiles[f].size(), (cpu_tiles[f].size() + kBlock - 1) / kBlock,
                    kBlock);
    }
    EXPECT_NE(bits(cpu[0]), bits(cpu[1]));

    for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
        const TileSumRun gpu = run_tile_sum(*ctx, fields, nx, ny, nz, wg, kBlock);
        for (uint32_t f = 0; f < 2; ++f) {
            const std::vector<float> expected = partials_after(cpu_tiles[f], kBlock);
            std::size_t partial_mismatches = 0;
            for (std::size_t t = 0; t < expected.size(); ++t) {
                partial_mismatches += bits(gpu.partials[f][t]) != bits(expected[t]) ? 1u : 0u;
            }
            const float total = gpu.scalars[kTotalSlot[f]];
            std::printf("[GpuAirflowRunnerTileSum] %ux%ux%u wg %u %s: gpu=%.9g (%s) in slot %u, partials %zu, "
                        "partial mismatches=%zu\n",
                        nx, ny, nz, wg, kFieldName[f], static_cast<double>(total), hex_bits(total).c_str(),
                        kTotalSlot[f], expected.size(), partial_mismatches);
            EXPECT_EQ(bits(total), bits(cpu[f])) << kFieldName[f] << ", workgroup size " << wg;
            EXPECT_EQ(partial_mismatches, std::size_t{0}) << kFieldName[f] << ", workgroup size " << wg;
        }
        std::size_t disturbed = 0;
        for (uint32_t s = 0; s < kScalarSlots; ++s) {
            if (s == kTotalSlot[0] || s == kTotalSlot[1]) continue;
            disturbed += bits(gpu.scalars[s]) != bits(kScalarSentinel) ? 1u : 0u;
        }
        EXPECT_EQ(disturbed, std::size_t{0}) << "a slot other than the two totals' was written, workgroup size " << wg;
    }

    const TileSumRun other = run_tile_sum(*ctx, fields, nx, ny, nz, 64, kOtherBlockTiles);
    for (uint32_t f = 0; f < 2; ++f) {
        const float want = two_level_total(cpu_tiles[f], kOtherBlockTiles);
        const float total = other.scalars[kTotalSlot[f]];
        std::printf("[GpuAirflowRunnerTileSum] %ux%ux%u wg 64 %s, the push's block %u: gpu %s, cpu two-level at %u %s\n",
                    nx, ny, nz, kFieldName[f], kOtherBlockTiles, hex_bits(total).c_str(), kOtherBlockTiles,
                    hex_bits(want).c_str());
        EXPECT_EQ(bits(total), bits(want)) << kFieldName[f] << ": the block is not the push's";
    }
}

// Sides that are not multiples of 4: partial tiles on every far edge, one
// block (12 tiles).
TEST(GpuAirflowRunnerTileSum, RaggedShapeSameBitsAtEveryWorkgroupSizeAndAsTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    check_tile_sum(5, 7, 9, 0x7115E5ull);
}

// Two whole blocks (512 tiles). Since B5.1 the CPU's sum here is the
// two-level one: pressure 0x43af0d40 and rhs 0x472228a0, where B1's chain gave
// 0x43af0a00 and 0x47222898 (B5.0b).
TEST(GpuAirflowRunnerTileSum, CubeSameBitsAtEveryWorkgroupSizeAndAsTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    check_tile_sum(32, 32, 32, 0x32C0BEull);
}

// B5.1: 128 whole blocks (128^3, 32,768 tiles), and a ragged last block with
// ragged tiles (61x45x43, 2112 tiles: 8 blocks and 64). Mutations: the block
// from the workgroup size (SPADE_WG * 4, right at 64 only); pass 2 over the
// tiles in one chain (B1's order); the ragged last block dropped from pass 2.
TEST(GpuAirflowRunnerTileSum, ManyBlocksSameBitsAtEveryWorkgroupSizeAndAsTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    {
        SCOPED_TRACE("128^3");
        check_tile_sum(128, 128, 128, 0xB51128ull);
    }
    SCOPED_TRACE("61x45x43");
    check_tile_sum(61, 45, 43, 0xB51061ull);
}

// ===========================================================================
// GpuAirflowRunnerTileSum -- B5.0's measurements (plan B, B5 as amended
// 2026-10-22; the lead's rulings 1 and 2). Opt-in, as B2's timings are:
// SPADE_AIRFLOW_TIMINGS=1. They decide B5's next step and assert only that
// what they time is correct.
// ===========================================================================

[[nodiscard]] bool timings_enabled() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_AIRFLOW_TIMINGS") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

struct Spread {
    std::size_t runs = 0;
    double min = 0.0;
    double median = 0.0;
    double max = 0.0;
};

[[nodiscard]] Spread spread_of(std::vector<double> v) {
    Spread s;
    s.runs = v.size();
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    s.min = v.front();
    s.max = v.back();
    s.median = v.size() % 2 == 1 ? v[v.size() / 2] : 0.5 * (v[v.size() / 2 - 1] + v[v.size() / 2]);
    return s;
}

void print_spread(const char* what, const Spread& s, const char* unit) {
    std::printf("measure: %s: median %.4f %s, min %.4f, max %.4f (%zu runs)\n", what, s.median, unit, s.min, s.max,
                s.runs);
}

// THE DOT'S PASSES (ruling 1). B5's tile_dot is tile_sum with a product in
// pass 0 and the same last passes. Here, tile_sum's passes on the box, at
// workgroup size 64, from the runner's marks: 3 warm-up submits, then 20,
// each read back. The sum's bits are checked against reduce.hpp first, so the
// time is of a correct sum. Since B5.1 the last passes are two: the blocks
// (one thread a block) and their total (one thread).
//
// THE DECISION WORD'S READBACK (ruling 2), at the D1 box only: the host's
// round trip for read() of 16 bytes after a submit and its fence, and for the
// submit of a one-dispatch chain (airflow_copy of one float), timed on the
// host: 10 warm-ups, then 200 of each, alternating.
void time_dot_passes(VulkanContext& ctx, const char* name, uint32_t nx, uint32_t ny, uint32_t nz, bool readback) {
    constexpr uint32_t kWg = 64;
    constexpr int kWarmups = 3;
    constexpr int kRuns = 20;
    const uint32_t tiles = ((nx + 3) / 4) * ((ny + 3) / 4) * ((nz + 3) / 4);
    const uint32_t cells = nx * ny * nz;

    std::vector<float> field(cells);
    SplitMix rng{0xB50ull + cells};
    for (float& v : field) v = mixed_magnitude(rng.next());
    const float cpu = spade::physics::airflow::fixed_tile_sum(field, nx, ny, nz);

    std::vector<uint32_t> params(32, 0u);
    params[0] = nx;
    params[1] = ny;
    params[2] = nz;
    std::vector<RunnerBuffer> buffers = airflow_set(kPartials);
    buffers[kParams] = {"airflow_params", params.size() * 4ull, BufferKind::staged};
    buffers[kPressure] = {"airflow_pressure", cells * 4ull, BufferKind::staged};
    buffers[kScalars] = {"airflow_scalars", kScalarSlots * 4ull, BufferKind::staged};
    buffers[kPartials] = {"airflow_partials", tiles * 4ull, BufferKind::device_only};
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_tile_sum, &gen::kSpvVariants_airflow_copy};
    const uint32_t blocks = (tiles + spade::physics::airflow::kReduceBlockTiles - 1) /
                            spade::physics::airflow::kReduceBlockTiles;
    auto runner = ChainRunner::create(
        ctx, RunnerDesc{.buffers = buffers, .kernels = kernels, .push_bytes = kPushBytes, .workgroup_size = kWg});
    ASSERT_TRUE(runner.has_value()) << runner.error().context;
    ASSERT_TRUE((*runner)->upload(kParams, bytes_of(params)).has_value());
    ASSERT_TRUE((*runner)->upload(kPressure, bytes_of(field)).has_value());

    constexpr uint32_t kSlot = 1;
    const auto pass_push = [&](uint32_t pass) {
        return TileSumPush{.field = 0, .pass_block = spade::airflow_gpu::tile_sum_pass_word(pass),
                           .cell_count = cells, .slot = kSlot};
    };
    const ChainCommand chain[] = {dispatch(0, pass_push(0), {groups_for(tiles, kWg), 1, 1}), mark(),
                                  dispatch(0, pass_push(1), {groups_for(blocks, kWg), 1, 1}), mark(),
                                  dispatch(0, pass_push(2), {1, 1, 1}), mark()};
    auto id = (*runner)->record(chain);
    ASSERT_TRUE(id.has_value()) << id.error().context;

    std::vector<double> pass0_ms;
    std::vector<double> pass1_ms;
    std::vector<double> pass2_ms;
    std::vector<double> whole_ms;
    std::vector<float> scalars(kScalarSlots, 0.0f);
    for (int run = 0; run < kWarmups + kRuns; ++run) {
        ASSERT_TRUE((*runner)->submit(*id, 1).has_value());
        if (run == 0) {
            ASSERT_TRUE((*runner)->read(kScalars, writable_bytes_of(scalars)).has_value());
            ASSERT_EQ(bits(scalars[kSlot]), bits(cpu)) << name << ": the timed sum is not reduce.hpp's";
        }
        auto d_ns = (*runner)->durations_ns(*id);
        ASSERT_TRUE(d_ns.has_value()) << d_ns.error().context;
        if (d_ns->empty()) GTEST_SKIP() << "the device cannot time compute work";
        ASSERT_EQ(d_ns->size(), 3u);
        if (run < kWarmups) continue;
        pass0_ms.push_back(static_cast<double>((*d_ns)[0]) * 1e-6);
        pass1_ms.push_back(static_cast<double>((*d_ns)[1]) * 1e-6);
        pass2_ms.push_back(static_cast<double>((*d_ns)[2]) * 1e-6);
        whole_ms.push_back(static_cast<double>((*d_ns)[0] + (*d_ns)[1] + (*d_ns)[2]) * 1e-6);
    }
    std::printf("measure: %s, %u cells, %u tiles in %u blocks, workgroup size %u, sum 0x%08x as reduce.hpp\n", name,
                cells, tiles, blocks, kWg, bits(cpu));
    const std::string label(name);
    print_spread((label + ", pass 0 (one thread a tile)").c_str(), spread_of(pass0_ms), "ms");
    print_spread((label + ", pass 1 (one thread a block)").c_str(), spread_of(pass1_ms), "ms");
    print_spread((label + ", pass 2 (one thread over the blocks)").c_str(), spread_of(pass2_ms), "ms");
    print_spread((label + ", the whole sum (passes 0-2)").c_str(), spread_of(whole_ms), "ms");
    if (!readback) return;

    // One dispatch of one thread's work: the cheapest chain a host loop submits.
    const ChainCommand tiny[] = {dispatch(1, count_push(1), {1, 1, 1})};
    auto tiny_id = (*runner)->record(tiny);
    ASSERT_TRUE(tiny_id.has_value()) << tiny_id.error().context;
    constexpr int kReadWarmups = 10;
    constexpr int kReads = 200;
    std::vector<double> submit_us;
    std::vector<double> read_us;
    std::array<uint32_t, 4> word{};
    for (int run = 0; run < kReadWarmups + kReads; ++run) {
        const auto t0 = std::chrono::steady_clock::now();
        ASSERT_TRUE((*runner)->submit(*tiny_id, 1).has_value());
        const auto t1 = std::chrono::steady_clock::now();
        ASSERT_TRUE((*runner)->read(kScalars, std::as_writable_bytes(std::span(word))).has_value());
        const auto t2 = std::chrono::steady_clock::now();
        if (run < kReadWarmups) continue;
        submit_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        read_us.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
    }
    EXPECT_EQ(word[kSlot], bits(cpu)) << "the word read back is not the sum";
    print_spread((label + ", a one-dispatch chain's submit and fence (host)").c_str(), spread_of(submit_us), "us");
    print_spread((label + ", read() of a 16-byte word after it (host)").c_str(), spread_of(read_us), "us");
}

TEST(GpuAirflowRunnerTileSum, B5ZeroTimesTheDotsPassesAndAWordsReadback) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!timings_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_TIMINGS=1 to time B5.0's dot passes and readback";
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    {
        SCOPED_TRACE("128^3");
        time_dot_passes(*ctx, "128^3", 128, 128, 128, false);
    }
    if (HasFatalFailure() || IsSkipped()) return;
    SCOPED_TRACE("D1 box");
    time_dot_passes(*ctx, "D1 box 240x177x168", 240, 177, 168, true);
}

// ===========================================================================
// GpuAirflowRunnerTileSum -- B5.0b's measurements (the lead, 2026-10-22,
// after B5.0: the serial last pass is 4.86 ms at the D1 box). Opt-in, as
// B5.0's: SPADE_AIRFLOW_TIMINGS=1. airflow_tile_sum_probe.slang is a
// TEST-ONLY kernel; no chain of the solver records it.
//
//   (i)  THE ORDER KEPT: the last pass issues 8 to 64 loads before adding
//        them, in the same order, so it must give B1's serial chain's bits
//        (reduce.hpp's until B5.1).
//   (ii) A TWO-LEVEL ORDER, measured before it was chosen: blocks of B
//        consecutive tiles in (tk, tj, ti) order, each summed from 0 in tile
//        order (the last block holds the rest), then the block sums from 0 in
//        block order. Since B5.1 it is reduce.hpp's, at B = kReduceBlockTiles
//        (Cameron, 2026-10-22). The push carries B to the device, and the CPU
//        reference here reads it. It is never a workgroup size or a thread
//        count, so the sum is the same at every workgroup size (checked at 32,
//        64 and 128, as GnssTumble's invariance is) and equals the CPU's.
// ===========================================================================

// THE TWO-LEVEL ORDER'S BLOCK, in tiles: reduce.hpp's one statement of it.
constexpr uint32_t kTreeBlockTiles = spade::physics::airflow::kReduceBlockTiles;

// The push block of airflow_tile_sum_probe.
struct TileSumProbePush {
    uint32_t pass = 0;   // 0 tiles, 1 the kept order's chain, 2 block sums, 3 the block sums' total
    uint32_t ahead = 1;  // loads issued before their adds: 1, 8, 16, 32 or 64
    uint32_t cell_count = 0;
    uint32_t block = 0;  // passes 2-3: the tiles in a block
};

static_assert(sizeof(TileSumProbePush) == kPushBytes);

constexpr uint32_t kProbeSlot = 1;  // airflow_tile_sum_probe's kSlot

struct ProbeBox {
    const char* name;
    uint32_t nx, ny, nz;
};

// One probe runner over a box's field at a workgroup size.
struct ProbeRig {
    std::unique_ptr<ChainRunner> runner;
    uint32_t tiles = 0;
    uint32_t cells = 0;
    uint32_t wg = 0;
};

[[nodiscard]] ProbeRig make_probe_rig(VulkanContext& ctx, const ProbeBox& box, const std::vector<float>& field,
                                      uint32_t wg) {
    ProbeRig rig;
    rig.wg = wg;
    rig.tiles = ((box.nx + 3) / 4) * ((box.ny + 3) / 4) * ((box.nz + 3) / 4);
    rig.cells = box.nx * box.ny * box.nz;
    std::vector<uint32_t> params(32, 0u);
    params[0] = box.nx;
    params[1] = box.ny;
    params[2] = box.nz;
    std::vector<RunnerBuffer> buffers = airflow_set(kPartials);
    buffers[kParams] = {"airflow_params", params.size() * 4ull, BufferKind::staged};
    buffers[kPressure] = {"airflow_pressure", rig.cells * 4ull, BufferKind::staged};
    buffers[kScalars] = {"airflow_scalars", kScalarSlots * 4ull, BufferKind::staged};
    buffers[kPartials] = {"airflow_partials", 2ull * rig.tiles * 4ull, BufferKind::device_only};
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_tile_sum_probe};
    auto runner = ChainRunner::create(
        ctx, RunnerDesc{.buffers = buffers, .kernels = kernels, .push_bytes = kPushBytes, .workgroup_size = wg});
    EXPECT_TRUE(runner.has_value()) << runner.error().context;
    if (!runner.has_value()) return rig;
    rig.runner = std::move(*runner);
    EXPECT_TRUE(rig.runner->upload(kParams, bytes_of(params)).has_value());
    EXPECT_TRUE(rig.runner->upload(kPressure, bytes_of(field)).has_value());
    return rig;
}

// The chain of one sum: pass 0, then (block 0) pass 1 with `ahead`, or
// (block > 0) passes 2 and 3; a mark after each dispatch.
[[nodiscard]] std::vector<ChainCommand> probe_chain(const ProbeRig& rig, uint32_t ahead, uint32_t block) {
    const TileSumProbePush tiles_pass{.pass = 0, .ahead = ahead, .cell_count = rig.cells, .block = block};
    std::vector<ChainCommand> out{dispatch(0, tiles_pass, {groups_for(rig.tiles, rig.wg), 1, 1}), mark()};
    if (block == 0) {
        out.push_back(dispatch(0, TileSumProbePush{.pass = 1, .ahead = ahead, .cell_count = rig.cells}, {1, 1, 1}));
        out.push_back(mark());
        return out;
    }
    const uint32_t blocks = (rig.tiles + block - 1) / block;
    out.push_back(dispatch(0, TileSumProbePush{.pass = 2, .ahead = ahead, .cell_count = rig.cells, .block = block},
                           {groups_for(blocks, rig.wg), 1, 1}));
    out.push_back(mark());
    out.push_back(
        dispatch(0, TileSumProbePush{.pass = 3, .ahead = ahead, .cell_count = rig.cells, .block = block}, {1, 1, 1}));
    out.push_back(mark());
    return out;
}

struct ProbeTimes {
    float sum = 0.0f;
    std::vector<std::vector<double>> segment_ms;  // per mark, per timed run
};

// Records the chain once and submits it warmups + runs times; the sum from the first.
[[nodiscard]] ProbeTimes time_probe(ProbeRig& rig, const std::vector<ChainCommand>& chain, int warmups, int runs) {
    ProbeTimes out;
    out.sum = std::numeric_limits<float>::quiet_NaN();
    auto id = rig.runner->record(chain);
    EXPECT_TRUE(id.has_value()) << id.error().context;
    if (!id.has_value()) return out;
    std::vector<float> scalars(kScalarSlots, 0.0f);
    for (int run = 0; run < warmups + runs; ++run) {
        EXPECT_TRUE(rig.runner->submit(*id, 1).has_value());
        if (run == 0) {
            EXPECT_TRUE(rig.runner->read(kScalars, writable_bytes_of(scalars)).has_value());
            out.sum = scalars[kProbeSlot];
        }
        auto d_ns = rig.runner->durations_ns(*id);
        EXPECT_TRUE(d_ns.has_value()) << d_ns.error().context;
        if (!d_ns.has_value() || d_ns->empty()) return out;
        if (out.segment_ms.empty()) out.segment_ms.resize(d_ns->size());
        if (run < warmups) continue;
        for (std::size_t m = 0; m < d_ns->size(); ++m) {
            out.segment_ms[m].push_back(static_cast<double>((*d_ns)[m]) * 1e-6);
        }
    }
    return out;
}

[[nodiscard]] std::vector<double> sum_of_segments(const ProbeTimes& t, std::size_t from, std::size_t to) {
    std::vector<double> out;
    if (t.segment_ms.size() < to) return out;
    for (std::size_t r = 0; r < t.segment_ms[from].size(); ++r) {
        double s = 0.0;
        for (std::size_t m = from; m < to; ++m) s += t.segment_ms[m][r];
        out.push_back(s);
    }
    return out;
}

constexpr std::array<uint32_t, 5> kAheads{1u, 8u, 16u, 32u, 64u};
constexpr std::array<uint32_t, 6> kBlocks{64u, 128u, 256u, 512u, 1024u, 2048u};

[[nodiscard]] std::vector<float> probe_field(uint32_t cells) {
    std::vector<float> field(cells);
    SplitMix rng{0xB50ull + cells};  // B5.0's field
    for (float& v : field) v = mixed_magnitude(rng.next());
    return field;
}

TEST(GpuAirflowRunnerTileSum, B5ZeroBTimesTheKeptOrderUnrolledAndATwoLevelOrder) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!timings_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_TIMINGS=1 to time B5.0b's two ways to end the sum";
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    constexpr int kWarmups = 3;
    constexpr int kRuns = 20;
    for (const ProbeBox& box : {ProbeBox{"128^3", 128, 128, 128}, ProbeBox{"D1 box 240x177x168", 240, 177, 168}}) {
        SCOPED_TRACE(box.name);
        const std::vector<float> field = probe_field(box.nx * box.ny * box.nz);
        const std::vector<float> tiles = tile_partials(field, box.nx, box.ny, box.nz);
        const float serial = serial_total(tiles);
        ASSERT_EQ(bits(two_level_total(tiles, kTreeBlockTiles)),
                  bits(spade::physics::airflow::fixed_tile_sum(field, box.nx, box.ny, box.nz)))
            << "the spelled-out two-level order is not reduce.hpp's";
        ProbeRig rig = make_probe_rig(*ctx, box, field, 64);
        ASSERT_NE(rig.runner, nullptr);
        std::printf("measure: %s, %u tiles, workgroup size 64, B1's serial sum %s\n", box.name, rig.tiles,
                    hex_bits(serial).c_str());

        // (i) the order kept, loads issued ahead.
        for (const uint32_t ahead : kAheads) {
            const ProbeTimes t = time_probe(rig, probe_chain(rig, ahead, 0), kWarmups, kRuns);
            if (t.segment_ms.empty()) GTEST_SKIP() << "the device cannot time compute work";
            EXPECT_EQ(bits(t.sum), bits(serial)) << "(i) ahead " << ahead << ": not B1's serial bits";
            const std::string label = std::string(box.name) + ", (i) kept order, " + std::to_string(ahead) +
                                      " loads ahead, sum " + hex_bits(t.sum) +
                                      (bits(t.sum) == bits(serial) ? " = B1's serial" : " DIFFERS");
            print_spread((label + ": last pass").c_str(), spread_of(t.segment_ms[1]), "ms");
            print_spread((label + ": whole sum").c_str(), spread_of(sum_of_segments(t, 0, 2)), "ms");
        }

        // (ii) the two-level order, per block size and loads ahead.
        for (const uint32_t block : kBlocks) {
            const float cpu = two_level_total(tiles, block);
            for (const uint32_t ahead : {1u, 8u, 16u, 32u}) {
                const ProbeTimes t = time_probe(rig, probe_chain(rig, ahead, block), kWarmups, kRuns);
                if (t.segment_ms.empty()) GTEST_SKIP() << "the device cannot time compute work";
                EXPECT_EQ(bits(t.sum), bits(cpu)) << "(ii) block " << block << " ahead " << ahead;
                const std::string label = std::string(box.name) + ", (ii) B " + std::to_string(block) + ", " +
                                          std::to_string(ahead) + " ahead, sum " + hex_bits(t.sum) +
                                          (bits(t.sum) == bits(cpu) ? " = CPU two-level" : " DIFFERS from CPU");
                print_spread((label + ": levels 1 + 2").c_str(), spread_of(sum_of_segments(t, 1, 3)), "ms");
                print_spread((label + ": whole sum").c_str(), spread_of(sum_of_segments(t, 0, 3)), "ms");
                if (ahead == 16u) {
                    print_spread((label + ": level 1").c_str(), spread_of(t.segment_ms[1]), "ms");
                    print_spread((label + ": level 2").c_str(), spread_of(t.segment_ms[2]), "ms");
                }
            }
        }
    }
}

// THE INVARIANCE RUN (the lead's requirement 2, GnssTumble's pattern): the
// two-level sum at workgroup sizes 32, 64 and 128 gives one set of bits,
// equal to the CPU reference's, at kTreeBlockTiles and at every block size
// timed above, on boxes of one block (5x7x9), two (32^3), whole blocks
// (128^3) and a ragged last block (the D1 box).
TEST(GpuAirflowRunnerTileSum, B5ZeroBTheTwoLevelOrderIsTheSameAtEveryWorkgroupSize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!timings_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_TIMINGS=1 for B5.0b's invariance run";
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    constexpr std::array<uint32_t, 3> kWorkgroupSizes{32u, 64u, 128u};
    for (const ProbeBox& box : {ProbeBox{"5x7x9", 5, 7, 9}, ProbeBox{"32^3", 32, 32, 32},
                                ProbeBox{"128^3", 128, 128, 128}, ProbeBox{"D1 box 240x177x168", 240, 177, 168}}) {
        SCOPED_TRACE(box.name);
        const std::vector<float> field = probe_field(box.nx * box.ny * box.nz);
        const std::vector<float> tiles = tile_partials(field, box.nx, box.ny, box.nz);
        std::vector<uint32_t> blocks{kTreeBlockTiles};
        for (const uint32_t b : kBlocks) {
            if (b != kTreeBlockTiles) blocks.push_back(b);
        }
        for (const uint32_t wg : kWorkgroupSizes) {
            ProbeRig rig = make_probe_rig(*ctx, box, field, wg);
            ASSERT_NE(rig.runner, nullptr);
            for (const uint32_t block : blocks) {
                const float cpu = two_level_total(tiles, block);
                for (const uint32_t ahead : {1u, 16u}) {
                    const ProbeTimes t = time_probe(rig, probe_chain(rig, ahead, block), 0, 1);
                    EXPECT_EQ(bits(t.sum), bits(cpu)) << "wg " << wg << " block " << block << " ahead " << ahead;
                    if (block == kTreeBlockTiles && ahead == 16u) {
                        std::printf("invariance: %s, %zu tiles, B %u, workgroup size %u: gpu %s, cpu two-level %s, "
                                    "serial %s\n",
                                    box.name, tiles.size(), block, wg, hex_bits(t.sum).c_str(),
                                    hex_bits(cpu).c_str(), hex_bits(serial_total(tiles)).c_str());
                    }
                }
            }
        }
    }
}

// B5.0b's ACCURACY (the lead's requirement 3), no device, opt-in with the
// timings: the D1 box from rest (P2d's D1 probe: the reference quad at the
// box's centre, ambient 3 m/s, dx 4 mm), two fluid steps. After each, the
// step's CG is replayed from the step's start (x = the p it began from, b as
// it left it), under the step's subnormal scope, for the floor's 8 iterations
// and one more, taking pcg.cpp's dots in its order: r.z at the start, then d.q
// and r.z in each iteration (19 a step). Every dot is summed three ways from
// the same fp32 products: B1's serial order, the two-level order
// (kTreeBlockTiles; reduce.hpp's fixed_tile_dot since B5.1, on which the
// replay steps), and in double, compensated (the reference).
struct DotAccuracy {
    std::vector<double> serial_rel;
    std::vector<double> tree_rel;
    uint32_t dots = 0;
    uint32_t differ = 0;
    uint32_t tree_closer = 0;
    uint32_t serial_closer = 0;
    double product_rounding = 0.0;  // max |sum of fp32 products - exact dot| / |exact dot|, in double
};

// Neumaier's compensated sum, in double.
struct CompensatedSum {
    double sum = 0.0;
    double c = 0.0;
    void add(double v) {
        const double t = sum + v;
        c += std::fabs(sum) >= std::fabs(v) ? (sum - t) + v : (v - t) + sum;
        sum = t;
    }
    [[nodiscard]] double value() const { return sum + c; }
};

float accuracy_dot(const char* what, int step, const std::vector<float>& a, const std::vector<float>& b, uint32_t nx,
                   uint32_t ny, uint32_t nz, DotAccuracy& acc) {
    const std::size_t n = a.size();
    std::vector<float> products(n);
    CompensatedSum rounded;
    CompensatedSum exact;
    for (std::size_t c = 0; c < n; ++c) {
        products[c] = a[c] * b[c];
        rounded.add(static_cast<double>(products[c]));
        exact.add(static_cast<double>(a[c]) * static_cast<double>(b[c]));
    }
    const std::vector<float> tiles = tile_partials(products, nx, ny, nz);
    const float serial = serial_total(tiles);
    const float tree = two_level_total(tiles, kTreeBlockTiles);
    const float production = spade::physics::airflow::fixed_tile_dot(a, b, nx, ny, nz);
    EXPECT_EQ(bits(tree), bits(production)) << what << ": the spelled-out two-level dot is not reduce.hpp's";
    const double ref = rounded.value();
    const double scale = ref != 0.0 ? std::fabs(ref) : 1.0;
    const double serial_rel = std::fabs(static_cast<double>(serial) - ref) / scale;
    const double tree_rel = std::fabs(static_cast<double>(tree) - ref) / scale;
    const double ex = exact.value();
    if (ex != 0.0) acc.product_rounding = std::max(acc.product_rounding, std::fabs(ref - ex) / std::fabs(ex));
    acc.serial_rel.push_back(serial_rel);
    acc.tree_rel.push_back(tree_rel);
    ++acc.dots;
    if (bits(serial) != bits(tree)) ++acc.differ;
    if (tree_rel < serial_rel) ++acc.tree_closer;
    if (serial_rel < tree_rel) ++acc.serial_closer;
    std::printf("accuracy: step %d %-6s ref %.9e  serial %s rel %.3e  two-level %s rel %.3e%s\n", step, what, ref,
                hex_bits(serial).c_str(), serial_rel, hex_bits(tree).c_str(), tree_rel,
                bits(serial) == bits(tree) ? "  (same bits)" : "");
    return production;
}

void replay_cg_dots(spade::physics::airflow::Multigrid& mg, spade::physics::airflow::PcgScratch& s,
                    spade::physics::airflow::FluidRegions& regions, const spade::physics::airflow::GridShape& shape,
                    const std::vector<float>& x0, int step, DotAccuracy& acc) {
    namespace af = spade::physics::airflow;
    const spade::math::ScopedFlushSubnormals scope;  // as fluid_step() runs the solve
    af::PoissonLevel& L = mg.levels[0];
    const std::size_t n = s.x.size();
    const auto mask = [&](std::vector<float>& v) {
        for (std::size_t i = 0; i < n; ++i) {
            if (regions.solid[i] != 0u) v[i] = 0.0f;
        }
    };
    const auto gauge = [&](std::vector<float>& v) { af::remove_region_means(v, regions, shape); };
    const auto precond = [&]() {
        std::copy(s.r.begin(), s.r.end(), L.b.begin());
        af::precondition(mg);
        std::copy(L.p.begin(), L.p.end(), s.z.begin());
        mask(s.z);
        gauge(s.z);
    };
    std::copy(x0.begin(), x0.end(), s.x.begin());
    mask(s.x);
    af::apply_operator(L, s.x, s.q);
    for (std::size_t i = 0; i < n; ++i) s.r[i] = s.b[i] - s.q[i];
    mask(s.r);
    gauge(s.r);
    precond();
    std::copy(s.z.begin(), s.z.end(), s.d.begin());
    float rz = accuracy_dot("r.z 0", step, s.r, s.z, L.nx, L.ny, L.nz, acc);
    for (uint32_t it = 1; it <= af::kSolidsCgIterations + 1; ++it) {
        af::apply_operator(L, s.d, s.q);
        const std::string dq_name = "d.q " + std::to_string(it);
        const float dq = accuracy_dot(dq_name.c_str(), step, s.d, s.q, L.nx, L.ny, L.nz, acc);
        if (dq == 0.0f || rz == 0.0f) return;
        const float alpha = rz / dq;
        for (std::size_t i = 0; i < n; ++i) {
            s.x[i] += alpha * s.d[i];
            s.r[i] -= alpha * s.q[i];
        }
        gauge(s.r);
        precond();
        const std::string rz_name = "r.z " + std::to_string(it);
        const float rz_next = accuracy_dot(rz_name.c_str(), step, s.r, s.z, L.nx, L.ny, L.nz, acc);
        const float beta = rz_next / rz;
        for (std::size_t i = 0; i < n; ++i) s.d[i] = s.z[i] + beta * s.d[i];
        rz = rz_next;
    }
}

TEST(GpuAirflowRunnerTileSum, B5ZeroBTheTwoOrdersAccuracyOnTheD1BoxsDots) {
    if (!timings_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_TIMINGS=1 for B5.0b's accuracy of the two orders";
    namespace af = spade::physics::airflow;
    static const spade::vehicles::ModelType quad =
        spade::vehicles::compile_airframe(spade::vehicles::fixtures::reference_quad()).value().model;
    const float dx = 0.004f;
    const af::GridShape shape{240, 177, 168, dx};
    spade::BodyState body{};
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const std::array<float, 4> omega{};
    const std::array<float, 4> psi{};
    const auto posed = spade::vehicles::posed_parts(quad, body, omega, psi, spade::vehicles::PartFilter::rigid_only);
    ASSERT_TRUE(posed.has_value());
    const glm::vec3 origin = -0.5f * dx * glm::vec3(240.0f, 177.0f, 168.0f);
    const spade::vehicles::AirflowSolids solids(quad, *posed, origin);
    const auto parts = solids.parts();
    ASSERT_TRUE(parts.has_value());
    af::DomainBc bc;
    bc.ambient = glm::vec3(3.0f, 0.0f, 0.0f);
    const af::FluidConfig cfg{.rho = 1.225f, .nu = 1.4607e-5f, .h = 0.38e-3f, .n_v = af::kSolidsCgIterations};
    af::MacGrid g = *af::make_grid(shape, bc);
    std::fill(g.u.begin(), g.u.end(), 3.0f);
    af::FluidScratch s = *af::make_fluid_scratch(g, bc, cfg);
    ASSERT_TRUE(af::set_solids(s, g, bc, *parts, solids.sampler()).has_value()) << s.solids.run_line;
    std::printf("accuracy: D1 box 240x177x168 from rest, %s; B %u tiles\n", af::describe_regions(s.regions).c_str(),
                kTreeBlockTiles);
    DotAccuracy acc;
    for (int step = 1; step <= 2; ++step) {
        const std::vector<float> x0 = g.p;
        const af::FluidHealth h = af::fluid_step(g, bc, cfg, s);
        std::printf("accuracy: step %d: r %.4e in %u iterations, stalled %d\n", step,
                    static_cast<double>(h.pressure_residual), h.pressure_iterations, h.stalled ? 1 : 0);
        replay_cg_dots(s.projection.mg, s.projection.pcg, s.regions, shape, x0, step, acc);
    }
    ASSERT_FALSE(acc.serial_rel.empty());
    const auto stats = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        double mean = 0.0;
        for (const double e : v) mean += e;
        return std::array<double, 3>{v[v.size() / 2], mean / static_cast<double>(v.size()), v.back()};
    };
    const auto se = stats(acc.serial_rel);
    const auto te = stats(acc.tree_rel);
    std::printf("accuracy: %u dots; |sum - ref| / |ref|: serial median %.3e mean %.3e max %.3e; two-level median "
                "%.3e mean %.3e max %.3e\n",
                acc.dots, se[0], se[1], se[2], te[0], te[1], te[2]);
    std::printf("accuracy: bits differ in %u of %u; two-level closer in %u, serial closer in %u; fp32 products' own "
                "rounding (sum of products vs the exact dot) up to %.3e\n",
                acc.differ, acc.dots, acc.tree_closer, acc.serial_closer, acc.product_rounding);
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

// Every airflow kernel is scanned under the exact profile (P1-P4 and E1/E2):
// no bare OpFDiv, Sqrt or InverseSqrt whose result reaches an output, so a
// device's own division or root cannot enter one unseen (01-verification's
// scope rule; the lead, 2026-10-22). The kernels that divide or take a root
// do it through fp32_math's div32_ftz and sqrt32_ftz: airflow_mg_smooth,
// airflow_advection's periodic wrap, airflow_publish_fields and
// airflow_actuator_gather (Rendering, rendering/exact-kernels). The others
// hold no float division or root and pass as they did.
TEST(AirflowRunnerSpirv, KernelsHoldTheDeterminismRules) {
    struct Kernel {
        const SpirvVariantSet& variants;
        bool integer_only;  // no float arithmetic: integer work, or a copy's loads and stores
    };
    const Kernel kernels[] = {{gen::kSpvVariants_airflow_copy, true},
                              {gen::kSpvVariants_airflow_add_one, false},
                              {gen::kSpvVariants_airflow_tile_sum, false},
                              // B5.0b's test-only probe (no chain of the solver records it).
                              {gen::kSpvVariants_airflow_tile_sum_probe, false},
                              // B2's projection kernels (Rendering). The smoother divides with
                              // div32_ftz.
                              {gen::kSpvVariants_airflow_divergence, false},
                              {gen::kSpvVariants_airflow_gradient_subtract, false},
                              {gen::kSpvVariants_airflow_mg_smooth, false},
                              {gen::kSpvVariants_airflow_mg_residual, false},
                              {gen::kSpvVariants_airflow_mg_restrict, false},
                              {gen::kSpvVariants_airflow_mg_prolong, false},
                              {gen::kSpvVariants_airflow_remove_mean, false},
                              // B3's advection, velocity conditions and viscosity (Rendering). The
                              // conditions are copies, selects and two sign tests: no float
                              // arithmetic.
                              {gen::kSpvVariants_airflow_advection, false},
                              {gen::kSpvVariants_airflow_velocity_bc, true},
                              {gen::kSpvVariants_airflow_viscous, false},
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
                spade::testing::scan_spirv(v.code(), spade::testing::SpirvProfile::exact);
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
