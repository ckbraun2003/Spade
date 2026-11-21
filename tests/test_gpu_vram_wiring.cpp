// test_gpu_vram_wiring.cpp -- the VRAM ledger's first two consumers (camera design section 12, item 3; compute/vram_ledger.hpp):
// the world set's mirror reserves the share "world set" and a ChainRunner reserves "flow solver", each on the VulkanContext it
// was made on, AFTER today's per-allocation check (check_vram) has passed and BEFORE it allocates. A ledger belongs to one
// context, so consumers on SEPARATE contexts do not sum (the film's fluid and the editor's make their own); these tests use the
// shared-context case only. Every test skips with its reason when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "airflow_copy.spv.gen.hpp"
#include "compute/backend.hpp"
#include "compute/device_limits.hpp"
#include "compute/vram_ledger.hpp"
#include "compute/vulkan/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/pose_view.hpp"
#include "gpu_skip.hpp"
#include "set_body_state_support.hpp"
#include "sim/simulation.hpp"
#include "spike/airflow/chain_runner.hpp"

namespace {

using spade::Code;
using spade::Simulation;
using spade::airflow_gpu::BufferKind;
using spade::airflow_gpu::ChainRunner;
using spade::airflow_gpu::kAirflowSetBuffers;
using spade::airflow_gpu::RunnerBuffer;
using spade::airflow_gpu::RunnerDesc;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::SpirvVariantSet;
using spade::compute::VramLedger;
using spade::compute::VulkanContext;
using spade::test::floor_world_set;
namespace gen = spade::compute::gen;

constexpr uint64_t kMiB = uint64_t{1} << 20;
constexpr uint32_t kV = gen::kBinding_airflow_v;
constexpr uint32_t kU = gen::kBinding_airflow_u;
constexpr uint32_t kPushBytes = 16;
constexpr uint32_t kWorkgroupSize = 64;

[[nodiscard]] bool contains(const std::string& text, const std::string& needle) {
    return text.find(needle) != std::string::npos;
}

// The bytes the ledger holds under `name`, or 0 when no such share is held.
[[nodiscard]] uint64_t share_bytes(const VramLedger& ledger, const std::string& name) {
    for (const VramLedger::Share& s : ledger.shares()) {
        if (s.name == name) return s.bytes;
    }
    return 0;
}

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(BackendDesc{.kind = BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

// The runner's buffer list: section F's bindings 0 .. kV as 4-byte placeholders, u and v `bytes` long and staged.
[[nodiscard]] std::vector<RunnerBuffer> copy_buffers(uint64_t bytes) {
    std::vector<RunnerBuffer> out;
    for (uint32_t b = 0; b <= kV; ++b) {
        out.push_back(RunnerBuffer{std::string(kAirflowSetBuffers[b].name), 4, kAirflowSetBuffers[b].kind});
    }
    out[kU] = {"airflow_u", bytes, BufferKind::staged};
    out[kV] = {"airflow_v", bytes, BufferKind::staged};
    return out;
}

// What a runner of `buffers` asks of the card, as ChainRunner::create() sums it: every buffer on the device plus one staging
// buffer the size of the largest buffer that crosses to the host.
[[nodiscard]] uint64_t runner_total(const std::vector<RunnerBuffer>& buffers) {
    uint64_t device = 0, staging = 0;
    for (const RunnerBuffer& b : buffers) {
        device += b.bytes;
        if (b.kind != BufferKind::device_only) staging = std::max(staging, b.bytes);
    }
    return device + staging;
}

// A one-world floor set on Vulkan, asking for the previous-tick poses or not. step_timeline is on only so the test can reach the
// backend's VulkanContext (step_semaphore() names it); it allocates nothing the mirror's share counts.
[[nodiscard]] Simulation make_sim(bool pose_view) {
    const spade::Result<spade::WorldSetDesc> set = floor_world_set(2);
    EXPECT_TRUE(set.has_value()) << set.error().context;
    BackendDesc desc{.kind = BackendKind::vulkan};
    desc.device_pose_view = pose_view;
    desc.step_timeline = true;
    spade::Result<Simulation> sim = Simulation::create(*set, 1'000'000, 2, desc);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

[[nodiscard]] const VramLedger& ledger_of(Simulation& sim) {
    const auto semaphore = spade::compute::step_semaphore(*sim.vulkan_backend());
    EXPECT_TRUE(semaphore.has_value()) << semaphore.error().context;
    return semaphore->context->vram_ledger();
}

}  // namespace

// The mirror reserves "world set" on its context for as long as it lives; keeping the previous poses (device_pose_view) makes the
// share exactly one "bodies" array bigger (2 bodies x 128 bytes in this set), the buffer the old pre-allocation total left out.
TEST(GpuVramWiring, TheWorldSetHoldsAShareAndKeepingThePreviousPosesAddsExactlyOneBodiesArrayToIt) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation off = make_sim(false);
    Simulation on = make_sim(true);
    const uint64_t bytes_off = share_bytes(ledger_of(off), "world set");
    const uint64_t bytes_on = share_bytes(ledger_of(on), "world set");
    std::printf("[GpuVramWiring] world set share: %llu bytes without the previous poses, %llu with\n",
                static_cast<unsigned long long>(bytes_off), static_cast<unsigned long long>(bytes_on));
    EXPECT_GT(bytes_off, 0u) << "no 'world set' share is held: " << ledger_of(off).describe();
    EXPECT_EQ(bytes_on, bytes_off + uint64_t{2} * 128u) << "the previous-pose buffer is one bodies array";
}

// A runner made on a context whose ledger already holds a share that leaves it too little is refused by the LEDGER, naming the
// asker ("flow solver") and the holder ("camera"), before anything is allocated, and holds no share afterwards.
TEST(GpuVramWiring, ARunnerThatTheHeldSharesLeaveNoRoomForIsRefusedByNameBeforeAllocatingAndHoldsNoShare) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const spade::compute::MemoryBudget budget = ctx->memory_budget();
    const std::vector<RunnerBuffer> buffers = copy_buffers(4 * kMiB);
    const uint64_t asked = runner_total(buffers);
    const uint64_t limit = budget.budget - spade::compute::kMirrorVramReserve;
    if (budget.budget < spade::compute::kMirrorVramReserve + 4 * asked) GTEST_SKIP() << "the device's budget is too small";
    // The camera promises all but half of what the runner needs.
    auto camera = ctx->reserve_vram("camera", limit - asked / 2);
    ASSERT_TRUE(camera.has_value()) << camera.error().context;

    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy};
    const auto before = spade::airflow_gpu::runner_memory_stats();
    auto refused = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers, .kernels = kernels, .push_bytes = kPushBytes,
                                                        .workgroup_size = kWorkgroupSize});
    const auto after = spade::airflow_gpu::runner_memory_stats();
    ASSERT_FALSE(refused.has_value()) << "the runner was made although the camera's share left it no room";
    std::printf("[GpuVramWiring] refused: %s\n", refused.error().context.c_str());
    EXPECT_EQ(refused.error().code, Code::capacity_exceeded);
    EXPECT_TRUE(contains(refused.error().context, "flow solver")) << refused.error().context;
    EXPECT_TRUE(contains(refused.error().context, "camera")) << refused.error().context;
    EXPECT_TRUE(contains(refused.error().context, std::to_string(asked))) << refused.error().context;
    EXPECT_EQ(after.allocations, before.allocations) << "an allocation was made before the refusal";
    ASSERT_EQ(ctx->vram_ledger().shares().size(), 1u) << ctx->vram_ledger().describe();
    EXPECT_EQ(ctx->vram_ledger().shares()[0].name, "camera");
}

// A runner holds "flow solver" for its total (device + staging) while it lives and gives it back when it is destroyed; a second
// unnamed runner on the same context holds "flow solver 2" (two fluids on one context each hold their own); a name given is taken
// as it is, and the ledger refuses one already held.
TEST(GpuVramWiring, ARunnerHoldsFlowSolverForItsTotalAndReleasesItAndASecondOneNamesItself) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const std::vector<RunnerBuffer> buffers = copy_buffers(kMiB);
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_copy};
    const auto make = [&](const char* name) {
        RunnerDesc desc{.buffers = buffers, .kernels = kernels, .push_bytes = kPushBytes, .workgroup_size = kWorkgroupSize};
        if (name != nullptr) desc.vram_share = name;
        return ChainRunner::create(*ctx, desc);
    };
    auto first = make(nullptr);
    ASSERT_TRUE(first.has_value()) << first.error().context;
    EXPECT_EQ(share_bytes(ctx->vram_ledger(), "flow solver"), runner_total(buffers)) << ctx->vram_ledger().describe();

    auto repeated = make("flow solver");
    ASSERT_FALSE(repeated.has_value()) << "two runners held the same share name";
    EXPECT_EQ(repeated.error().code, Code::invalid_argument);
    EXPECT_TRUE(contains(repeated.error().context, "already held")) << repeated.error().context;
    EXPECT_EQ(ctx->vram_ledger().shares().size(), 1u) << "the refused runner left a share: " << ctx->vram_ledger().describe();

    auto second = make(nullptr);
    ASSERT_TRUE(second.has_value()) << second.error().context;
    EXPECT_EQ(ctx->vram_ledger().shares().size(), 2u) << ctx->vram_ledger().describe();
    EXPECT_EQ(share_bytes(ctx->vram_ledger(), "flow solver 2"), runner_total(buffers)) << ctx->vram_ledger().describe();
    auto named = make("fluid B");
    ASSERT_TRUE(named.has_value()) << named.error().context;
    EXPECT_EQ(share_bytes(ctx->vram_ledger(), "fluid B"), runner_total(buffers));
    named->reset();
    first->reset();
    EXPECT_EQ(share_bytes(ctx->vram_ledger(), "flow solver"), 0u) << "the destroyed runner still holds its share";
    EXPECT_EQ(share_bytes(ctx->vram_ledger(), "flow solver 2"), runner_total(buffers));
    second->reset();
    EXPECT_TRUE(ctx->vram_ledger().shares().empty()) << ctx->vram_ledger().describe();
}
