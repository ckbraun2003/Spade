// Device limits and the VRAM budget, checked by name (B0 of Physics' GPU plan:
// docs/design/core/plans/2026-10-21-airflow-gpu-b0-plan.md; stage 7 §7.1 and
// §7.4; CORE-5 as amended 2026-10-12). Pure functions over plain data, so these
// run on every machine; tests/test_gpu_device_limits.cpp reads a real device.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <string_view>
#include <utility>

#include "compute/device_limits.hpp"
#include "compute/vram_ledger.hpp"
#include "core/error.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace {

using spade::Code;
using spade::compute::check_buffer;
using spade::compute::check_groups;
using spade::compute::check_push;
using spade::compute::check_storage_buffers;
using spade::compute::check_vram;
using spade::compute::check_workgroup;
using spade::compute::device_lost_text;
using spade::compute::DeviceLimits;
using spade::compute::kGuaranteedLimits;
using spade::compute::MemoryBudget;
using spade::compute::VramLedger;
using spade::compute::VramShare;
using spade::compute::vram_cap;

constexpr std::string_view kDevice = "Test Device 9000";
constexpr uint64_t kGiB = uint64_t{1} << 30;

[[nodiscard]] DeviceLimits guaranteed() {
    DeviceLimits limits = kGuaranteedLimits;
    limits.allocation_size_reported = true;
    return limits;
}

[[nodiscard]] bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

// Refused as capacity_exceeded, naming the device and every needle.
void expect_refused(const spade::Result<void>& r, std::initializer_list<std::string_view> needles) {
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::capacity_exceeded) << r.error().context;
    EXPECT_TRUE(contains(r.error().context, "device 'Test Device 9000'")) << r.error().context;
    for (const std::string_view needle : needles) {
        EXPECT_TRUE(contains(r.error().context, needle)) << "missing '" << needle << "' in: " << r.error().context;
    }
}

}  // namespace

TEST(DeviceLimits, TheGuaranteedMinimumsAreVulkans) {
    const DeviceLimits& g = kGuaranteedLimits;
    EXPECT_EQ(g.max_storage_buffer_range, uint64_t{1} << 27);
    EXPECT_EQ(g.max_memory_allocation_size, uint64_t{1} << 30);
    EXPECT_EQ(g.max_per_stage_storage_buffers, 4u);
    EXPECT_EQ(g.max_set_storage_buffers, 24u);
    EXPECT_EQ(g.max_push_constants_size, 128u);
    EXPECT_EQ(g.max_group_count, (std::array<uint32_t, 3>{65535, 65535, 65535}));
    EXPECT_EQ(g.max_group_size, (std::array<uint32_t, 3>{128, 128, 64}));
    EXPECT_EQ(g.max_group_invocations, 128u);
    EXPECT_FALSE(g.allocation_size_reported);
}

TEST(DeviceLimits, EachLimitPassesAtItsValueAndIsRefusedOnePast) {
    const DeviceLimits l = guaranteed();
    EXPECT_TRUE(check_buffer("airflow_u", l.max_storage_buffer_range, l, kDevice).has_value());
    expect_refused(check_buffer("airflow_u", l.max_storage_buffer_range + 1, l, kDevice),
                   {"buffer 'airflow_u' is 134217729 bytes", "maxStorageBufferRange (134217728 bytes)"});

    // A device whose range passes the allocation size: the allocation answers.
    DeviceLimits wide_range = l;
    wide_range.max_storage_buffer_range = uint64_t{1} << 32;
    EXPECT_TRUE(check_buffer("airflow_advect", l.max_memory_allocation_size, wide_range, kDevice).has_value());
    expect_refused(check_buffer("airflow_advect", l.max_memory_allocation_size + 1, wide_range, kDevice),
                   {"buffer 'airflow_advect' is 1073741825 bytes", "maxMemoryAllocationSize (1073741824 bytes)"});

    EXPECT_TRUE(check_storage_buffers(4, l, kDevice).has_value());
    expect_refused(check_storage_buffers(5, l, kDevice),
                   {"a layout of 5 storage buffers", "maxPerStageDescriptorStorageBuffers (4)"});
    DeviceLimits wide_stage = l;
    wide_stage.max_per_stage_storage_buffers = 1000;
    EXPECT_TRUE(check_storage_buffers(24, wide_stage, kDevice).has_value());
    expect_refused(check_storage_buffers(25, wide_stage, kDevice),
                   {"a layout of 25 storage buffers", "maxDescriptorSetStorageBuffers (24)"});

    EXPECT_TRUE(check_push(128, l, kDevice).has_value());
    expect_refused(check_push(129, l, kDevice),
                   {"a push-constant block of 129 bytes", "maxPushConstantsSize (128 bytes)"});
}

TEST(DeviceLimits, TheGroupCheckNamesThePassAndTheAxis) {
    const DeviceLimits l = guaranteed();
    EXPECT_TRUE(check_groups("advect", {65535, 65535, 65535}, l, kDevice).has_value());
    expect_refused(check_groups("advect", {1, 65536, 1}, l, kDevice),
                   {"pass 'advect' dispatches 65536 groups on axis y", "maxComputeWorkGroupCount[1] (65535)"});
    expect_refused(check_groups("advect", {65536, 1, 1}, l, kDevice), {"on axis x", "maxComputeWorkGroupCount[0]"});
}

TEST(DeviceLimits, TheWorkgroupCheckNamesTheKernel) {
    const DeviceLimits l = guaranteed();
    EXPECT_TRUE(check_workgroup("tile_sum", {4, 4, 4}, l, kDevice).has_value());
    EXPECT_TRUE(check_workgroup("row", {128, 1, 1}, l, kDevice).has_value());
    expect_refused(check_workgroup("tall", {1, 1, 65}, l, kDevice),
                   {"kernel 'tall' has a local size of 65 on axis z", "maxComputeWorkGroupSize[2] (64)"});
    expect_refused(check_workgroup("fat", {8, 8, 4}, l, kDevice),
                   {"kernel 'fat' has 256 invocations per workgroup", "maxComputeWorkGroupInvocations (128)"});
}

TEST(VramBudget, TheCapIsTheSmallerOfFourGibAndTheFreeBudget) {
    static_assert(spade::compute::kAirflowVramCap == spade::physics::airflow::kGridByteCap,
                  "the GPU's ceiling is the CPU's");
    MemoryBudget b{.budget = 7 * kGiB, .usage = 1 * kGiB, .heap = 0, .source = MemoryBudget::Source::ext};
    EXPECT_EQ(vram_cap(b, kGiB, 0), 4 * kGiB);  // 7 - 1 - 1 = 5, over the 4 GiB ceiling
    b.usage = 3 * kGiB;
    EXPECT_EQ(vram_cap(b, kGiB, 0), 3 * kGiB);
    b.usage = 7 * kGiB;
    EXPECT_EQ(vram_cap(b, kGiB, 0), 0u);  // never below 0
    b.usage = 6 * kGiB + kGiB / 2;
    EXPECT_EQ(vram_cap(b, kGiB, 0), 0u);
}

TEST(VramBudget, AGridOverTheLiveBudgetIsRefusedNamingTheBytesAndTheCap) {
    // Another program holds VRAM: the grid is under the 4 GiB ceiling but over
    // what is free, and is refused by name before any allocation.
    const MemoryBudget b{.budget = 7 * kGiB, .usage = 4 * kGiB, .heap = 0, .source = MemoryBudget::Source::ext};
    EXPECT_TRUE(check_vram("the airflow grid", 2 * kGiB, b, kGiB, 0, kDevice).has_value());
    expect_refused(check_vram("the airflow grid", 3 * kGiB, b, kGiB, 0, kDevice),
                   {"the airflow grid asks for 3221225472 bytes of VRAM", "the cap is 2147483648 bytes",
                    "budget 7516192768 - usage 4294967296 - reserve 1073741824", "VK_EXT_memory_budget"});
}

TEST(VramBudget, HalfHeapSubtractsWhatTheCallerAllocated) {
    MemoryBudget half{.budget = 4 * kGiB, .usage = 0, .heap = 1, .source = MemoryBudget::Source::half_heap};
    EXPECT_EQ(vram_cap(half, kGiB, kGiB), 2 * kGiB);
    expect_refused(check_vram("the airflow grid", 2 * kGiB + 1, half, kGiB, kGiB, kDevice),
                   {"the cap is 2147483648 bytes", "half the device-local heap", "allocated 1073741824 itself"});
    // With the extension the heap's usage already counts the caller's own.
    const MemoryBudget ext{.budget = 4 * kGiB, .usage = 0, .heap = 1, .source = MemoryBudget::Source::ext};
    EXPECT_EQ(vram_cap(ext, kGiB, kGiB), 3 * kGiB);
}

// ---------------------------------------------------------------------------
// THE VRAM LEDGER (camera design section 12, item 3): the world set, the flow solver and the camera each reserve a stated
// share, and the sum of the shares is checked against the device's budget, by name, before anything is allocated.
// ---------------------------------------------------------------------------

namespace {

constexpr uint64_t kLedgerReserve = uint64_t{64} << 20;  // what the engine leaves free (kMirrorVramReserve's size)

// A card like the 3060 Ti: 8 GiB, the process's budget a little under it.
[[nodiscard]] MemoryBudget the_card() {
    return {.budget = 8 * kGiB - kGiB / 4, .usage = 0, .heap = 0, .source = MemoryBudget::Source::ext};
}

[[nodiscard]] spade::Result<VramShare> reserve(VramLedger& ledger, std::string_view name, uint64_t bytes,
                                               const MemoryBudget& budget = the_card()) {
    return ledger.reserve(name, bytes, budget, kLedgerReserve, kDevice);
}

}  // namespace

TEST(VramLedger, SharesThatFitAreHeldAndSummedInTheOrderReserved) {
    VramLedger ledger;
    auto world = reserve(ledger, "world set", kGiB);
    auto flow = reserve(ledger, "flow solver", 4 * kGiB);
    auto camera = reserve(ledger, "camera", kGiB);  // the design's 1 GiB at 128 envs: 6 GiB of 7.75
    ASSERT_TRUE(world.has_value()) << world.error().context;
    ASSERT_TRUE(flow.has_value()) << flow.error().context;
    ASSERT_TRUE(camera.has_value()) << camera.error().context;
    EXPECT_TRUE(camera->held());
    EXPECT_EQ(camera->bytes(), kGiB);
    EXPECT_EQ(ledger.reserved(), 6 * kGiB);
    ASSERT_EQ(ledger.shares().size(), 3u);
    EXPECT_EQ(ledger.shares()[0].name, "world set");
    EXPECT_EQ(ledger.shares()[1].name, "flow solver");
    EXPECT_EQ(ledger.shares()[2].name, "camera");
    EXPECT_EQ(ledger.shares()[1].bytes, 4 * kGiB);
}

TEST(VramLedger, ASumOverTheBudgetIsRefusedNamingTheAskerTheHoldersAndTheLimit) {
    VramLedger ledger;
    auto world = reserve(ledger, "world set", kGiB);
    auto flow = reserve(ledger, "flow solver", 4 * kGiB);
    ASSERT_TRUE(world.has_value() && flow.has_value());
    // 5 GiB held, 7.75 GiB of budget less 64 MiB left free: a 3 GiB camera does not fit, and nothing is held for it.
    const auto camera = reserve(ledger, "camera", 3 * kGiB);
    ASSERT_FALSE(camera.has_value());
    EXPECT_EQ(camera.error().code, Code::capacity_exceeded);
    const std::string& why = camera.error().context;
    for (const std::string_view needle :
         {"camera", "3221225472 bytes", "world set", "flow solver", "5368709120", "8321499136", "VK_EXT_memory_budget",
          "device 'Test Device 9000'"}) {
        EXPECT_TRUE(contains(why, needle)) << "missing '" << needle << "' in: " << why;
    }
    EXPECT_EQ(ledger.reserved(), 5 * kGiB) << "a refused reservation holds nothing";
    EXPECT_EQ(ledger.shares().size(), 2u);
}

TEST(VramLedger, AShareIsReleasedWhenItsHandleIsDestroyedOrReleased) {
    VramLedger ledger;
    {
        auto flow = reserve(ledger, "flow solver", 4 * kGiB);
        ASSERT_TRUE(flow.has_value());
        EXPECT_EQ(ledger.reserved(), 4 * kGiB);
    }
    EXPECT_EQ(ledger.reserved(), 0u);
    EXPECT_TRUE(ledger.shares().empty());
    auto camera = reserve(ledger, "camera", kGiB);
    ASSERT_TRUE(camera.has_value());
    camera->release();
    EXPECT_FALSE(camera->held());
    EXPECT_EQ(ledger.reserved(), 0u);
    // The bytes are free to be asked for again, under the same name.
    EXPECT_TRUE(reserve(ledger, "camera", 7 * kGiB).has_value());
}

TEST(VramLedger, MovingAHandleMovesTheShareAndReleasesItOnce) {
    VramLedger ledger;
    auto a = reserve(ledger, "camera", kGiB);
    ASSERT_TRUE(a.has_value());
    VramShare moved = std::move(*a);
    EXPECT_FALSE(a->held());
    EXPECT_TRUE(moved.held());
    EXPECT_EQ(ledger.reserved(), kGiB);
    VramShare other;
    other = std::move(moved);
    EXPECT_EQ(ledger.reserved(), kGiB) << "a move is not a second share";
    other.release();
    EXPECT_EQ(ledger.reserved(), 0u);
}

TEST(VramLedger, BadAsksAreInvalidArgumentsNotCapacityErrors) {
    VramLedger ledger;
    auto held = reserve(ledger, "camera", kGiB);
    ASSERT_TRUE(held.has_value());
    const auto zero = reserve(ledger, "flow solver", 0);
    ASSERT_FALSE(zero.has_value());
    EXPECT_EQ(zero.error().code, Code::invalid_argument) << zero.error().context;
    const auto unnamed = reserve(ledger, "", kGiB);
    ASSERT_FALSE(unnamed.has_value());
    EXPECT_EQ(unnamed.error().code, Code::invalid_argument) << unnamed.error().context;
    const auto twice = reserve(ledger, "camera", kGiB);
    ASSERT_FALSE(twice.has_value());
    EXPECT_EQ(twice.error().code, Code::invalid_argument) << twice.error().context;
    EXPECT_TRUE(contains(twice.error().context, "camera")) << twice.error().context;
    EXPECT_EQ(ledger.reserved(), kGiB);
}

TEST(VramLedger, WithoutTheExtensionTheBudgetIsHalfTheHeapAndTheRefusalSaysSo) {
    VramLedger ledger;
    const MemoryBudget half{.budget = 4 * kGiB, .usage = 0, .heap = 0, .source = MemoryBudget::Source::half_heap};
    const auto world = reserve(ledger, "world set", 3 * kGiB, half);  // kept: a dropped handle releases its share
    ASSERT_TRUE(world.has_value());
    // The sum is what the ledger holds; the (blind) usage of the heap plays no part.
    const auto camera = reserve(ledger, "camera", kGiB, half);
    ASSERT_FALSE(camera.has_value());
    EXPECT_TRUE(contains(camera.error().context, "half the device-local heap")) << camera.error().context;
}

TEST(VramLedger, DescribeListsTheSharesInOneLine) {
    VramLedger ledger;
    auto world = reserve(ledger, "world set", kGiB);
    auto camera = reserve(ledger, "camera", kGiB / 2);
    ASSERT_TRUE(world.has_value() && camera.has_value());
    const std::string line = ledger.describe();
    for (const std::string_view needle : {"world set 1073741824", "camera 536870912", "1610612736 in all"}) {
        EXPECT_TRUE(contains(line, needle)) << "missing '" << needle << "' in: " << line;
    }
    EXPECT_EQ(line.find('\n'), std::string::npos);
}

// ---------------------------------------------------------------------------
// WHAT A LOST DEVICE SAYS (L106e's TDR findings): the old words, the device, and the VRAM budget and usage at create() against the
// read taken when the device was lost, so a reader can tell another program growing from the work being slow.
// ---------------------------------------------------------------------------

TEST(DeviceLostText, KeepsTheOldWordsNamesTheDeviceAndSaysWhatTheVramDid) {
    const MemoryBudget at_create{.budget = 7 * kGiB, .usage = kGiB, .heap = 0, .source = MemoryBudget::Source::ext};
    const MemoryBudget now{.budget = 7 * kGiB, .usage = 6 * kGiB, .heap = 0, .source = MemoryBudget::Source::ext};
    const std::string text = device_lost_text("airflow runner: submit", kDevice, &at_create, &now);
    for (const std::string_view needle :
         {"Vulkan device lost during airflow runner: submit", "device 'Test Device 9000'", "TDR", "at create", "when it was lost",
          "7516192768", "1073741824", "6442450944", "5368709120 bytes more", "VK_EXT_memory_budget"}) {
        EXPECT_TRUE(contains(text, needle)) << "missing '" << needle << "' in: " << text;
    }
    EXPECT_EQ(text.find('\n'), std::string::npos) << "one line";
}

TEST(DeviceLostText, AUsageThatDidNotGrowSaysSoAndOnlyTheWorkIsLeftToBlame) {
    const MemoryBudget at_create{.budget = 7 * kGiB, .usage = 3 * kGiB, .heap = 0, .source = MemoryBudget::Source::ext};
    const MemoryBudget now{.budget = 7 * kGiB, .usage = 3 * kGiB, .heap = 0, .source = MemoryBudget::Source::ext};
    const std::string text = device_lost_text("submit", kDevice, &at_create, &now);
    EXPECT_TRUE(contains(text, "no more than at create")) << text;
    EXPECT_FALSE(contains(text, "bytes more")) << text;
}

TEST(DeviceLostText, WithoutTheBudgetExtensionTheUsageIsNotVisibleAndTheTextSaysSo) {
    const MemoryBudget half{.budget = 4 * kGiB, .usage = 0, .heap = 0, .source = MemoryBudget::Source::half_heap};
    const std::string text = device_lost_text("submit", kDevice, &half, &half);
    EXPECT_TRUE(contains(text, "no VK_EXT_memory_budget")) << text;
    EXPECT_TRUE(contains(text, "other programs' VRAM use is not visible")) << text;
    EXPECT_TRUE(contains(text, "Vulkan device lost during submit")) << text;
}

TEST(DeviceLostText, WithNoReadsItIsTheOldWordsWithTheDeviceAndTheUsualCause) {
    const std::string text = device_lost_text("submit", kDevice, nullptr, nullptr);
    EXPECT_TRUE(contains(text, "Vulkan device lost during submit")) << text;
    EXPECT_TRUE(contains(text, "device 'Test Device 9000'")) << text;
    EXPECT_TRUE(contains(text, "TDR")) << text;
    EXPECT_FALSE(contains(text, "at create")) << text;
}
