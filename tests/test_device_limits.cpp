// Device limits and the VRAM budget, checked by name (B0 of Physics' GPU plan:
// docs/design/core/plans/2026-10-21-airflow-gpu-b0-plan.md; stage 7 §7.1 and
// §7.4; CORE-5 as amended 2026-10-12). Pure functions over plain data, so these
// run on every machine; tests/test_gpu_device_limits.cpp reads a real device.

#include <limits>
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

// O193 (Cameron: "lift the limit, tight margin"): the device's cap is what the heap's budget leaves after what is in use and the reserve. There is no
// fixed 4 GiB ceiling any more; a card with more to give gives it. A user cap can still only lower it.
TEST(VramBudget, TheCapIsTheFreeBudgetWithNoFixedCeiling) {
    MemoryBudget b{.budget = 7 * kGiB, .usage = 1 * kGiB, .heap = 0, .source = MemoryBudget::Source::ext};
    EXPECT_EQ(vram_cap(b, kGiB, 0), 5 * kGiB);  // 7 - 1 - 1: the old 4 GiB ceiling is gone
    b.usage = 3 * kGiB;
    EXPECT_EQ(vram_cap(b, kGiB, 0), 3 * kGiB);
    b.usage = 7 * kGiB;
    EXPECT_EQ(vram_cap(b, kGiB, 0), 0u);  // never below 0
    b.usage = 6 * kGiB + kGiB / 2;
    EXPECT_EQ(vram_cap(b, kGiB, 0), 0u);
    const MemoryBudget big{.budget = 24 * kGiB, .usage = 0, .heap = 0, .source = MemoryBudget::Source::ext};
    EXPECT_EQ(vram_cap(big, kGiB, 0), 23 * kGiB) << "a 24 GiB card gives 23 GiB, not 4";
    EXPECT_EQ(vram_cap(big, kGiB, 0, 2 * kGiB), 2 * kGiB) << "a user cap only lowers";
    EXPECT_EQ(vram_cap(big, kGiB, 0, 40 * kGiB), 23 * kGiB) << "and never raises";
}

// The case that is the point: a grid of 6.5 GiB, refused at 4 GiB today, passes on the 3060 Ti's budget (8 GiB less a quarter, the reserve 1 GiB);
// one over budget less reserve is still refused, by name, and says what the cap is made of.
TEST(VramBudget, AGridOverFourGibThatFitsTheBudgetPassesAndOneOverTheBudgetIsStillRefused) {
    const MemoryBudget card{.budget = 8 * kGiB - kGiB / 4, .usage = 0, .heap = 0, .source = MemoryBudget::Source::ext};
    EXPECT_TRUE(check_vram("the airflow grid", 6 * kGiB + kGiB / 2, card, kGiB, 0, kDevice).has_value());
    expect_refused(check_vram("the airflow grid", 7 * kGiB, card, kGiB, 0, kDevice),
                   {"the airflow grid asks for 7516192768 bytes of VRAM", "the cap is 7247757312 bytes = budget 8321499136 - usage 0 - reserve 1073741824",
                    "VK_EXT_memory_budget"});
}

// The reserve is a named, signed option (vram_reserve_bytes): the default is 1 GiB, a run may LOWER it (the O158 film run asks for 0.3 GiB), and a
// reserve under 0.3 GiB or a negative one is refused by the option's name.
TEST(VramBudget, TheReserveOptionIsNamedAndRefusedBelowThreeTenthsOfAGibOrNegative) {
    using spade::Code;
    using spade::compute::check_vram_reserve;
    using spade::compute::kMinVramReserve;
    EXPECT_EQ(kMinVramReserve, (uint64_t{3} << 30) / 10) << "0.3 GiB";
    for (const int64_t ok : {static_cast<int64_t>(kMinVramReserve), static_cast<int64_t>(kGiB), static_cast<int64_t>(4 * kGiB)}) {
        const auto r = check_vram_reserve(ok);
        ASSERT_TRUE(r.has_value()) << ok << ": " << r.error().context;
        EXPECT_EQ(*r, static_cast<uint64_t>(ok));
    }
    for (const int64_t bad : {static_cast<int64_t>(kMinVramReserve) - 1, int64_t{0}, int64_t{1}, int64_t{-1}, std::numeric_limits<int64_t>::min()}) {
        const auto r = check_vram_reserve(bad);
        ASSERT_FALSE(r.has_value()) << bad << " accepted";
        EXPECT_EQ(r.error().code, Code::invalid_argument) << r.error().context;
        EXPECT_TRUE(contains(r.error().context, "vram_reserve_bytes")) << "the option is named: " << r.error().context;
        EXPECT_TRUE(contains(r.error().context, std::to_string(bad))) << "the value asked is quoted: " << r.error().context;
    }
    EXPECT_TRUE(contains(check_vram_reserve(-1).error().context, "negative")) << check_vram_reserve(-1).error().context;
    EXPECT_TRUE(contains(check_vram_reserve(1).error().context, std::to_string(kMinVramReserve))) << "the least allowed is named";
}

TEST(VramBudget, AGridOverTheLiveBudgetIsRefusedNamingTheBytesAndTheCap) {
    // Another program holds VRAM: the grid is under the 4 GiB ceiling but over
    // what is free, and is refused by name before any allocation.
    const MemoryBudget b{.budget = 7 * kGiB, .usage = 4 * kGiB, .heap = 0, .source = MemoryBudget::Source::ext};
    EXPECT_TRUE(check_vram("the airflow grid", 2 * kGiB, b, kGiB, 0, kDevice).has_value());
    expect_refused(check_vram("the airflow grid", 3 * kGiB, b, kGiB, 0, kDevice),
                   {"the airflow grid asks for 3221225472 bytes of VRAM", "the cap is 2147483648 bytes = budget",
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

namespace {

// VK_EXT_memory_budget's two numbers move for different reasons: `usage` is what THIS process holds in the heap, `budget` is what the
// OS lets it hold, and the budget FALLS when another program takes VRAM. So a lost device's text compares both.
[[nodiscard]] MemoryBudget ext_budget(uint64_t budget, uint64_t usage) {
    return {.budget = budget, .usage = usage, .heap = 0, .source = MemoryBudget::Source::ext};
}

}  // namespace

TEST(DeviceLostText, KeepsTheOldWordsNamesTheDeviceAndPrintsBothBudgetsAndBothUsages) {
    const MemoryBudget at_create = ext_budget(7 * kGiB, kGiB);
    const MemoryBudget now = ext_budget(7 * kGiB, 6 * kGiB);
    const std::string text = device_lost_text("airflow runner: submit", kDevice, &at_create, &now);
    for (const std::string_view needle :
         {"Vulkan device lost during airflow runner: submit", "device 'Test Device 9000'", "TDR", "VK_EXT_memory_budget",
          "budget 7516192768 at create, 7516192768 when lost", "this process held 1073741824 at create, 6442450944 when lost",
          "this process holds 5368709120 bytes more than at create"}) {
        EXPECT_TRUE(contains(text, needle)) << "missing '" << needle << "' in: " << text;
    }
    // The budget did not fall and the process is inside it: neither of the other two clauses.
    EXPECT_FALSE(contains(text, "another program")) << text;
    EXPECT_FALSE(contains(text, "over its budget")) << text;
    EXPECT_EQ(text.find('\n'), std::string::npos) << "one line";
}

// The Vantage-sandbox case: another program takes VRAM, so the BUDGET falls while this process's usage stays flat.
TEST(DeviceLostText, AFallingBudgetWithFlatUsageNamesAnotherProgram) {
    const MemoryBudget at_create = ext_budget(7 * kGiB, 3 * kGiB);
    const MemoryBudget now = ext_budget(4 * kGiB, 3 * kGiB);
    const std::string text = device_lost_text("submit", kDevice, &at_create, &now);
    for (const std::string_view needle :
         {"budget 7516192768 at create, 4294967296 when lost", "this process held 3221225472 at create, 3221225472 when lost",
          "the budget fell by 3221225472 bytes (another program took VRAM after the run began)"}) {
        EXPECT_TRUE(contains(text, needle)) << "missing '" << needle << "' in: " << text;
    }
    EXPECT_FALSE(contains(text, "bytes more than at create")) << "the process did not grow: " << text;
    EXPECT_FALSE(contains(text, "look at the work itself")) << "the wrong conclusion for this case: " << text;
    EXPECT_FALSE(contains(text, "over its budget")) << text;
}

// Usage over the budget: the driver pages to system memory, the usual way a submit passes the 2 s watchdog.
TEST(DeviceLostText, UsageOverTheBudgetNamesPagingAndWhatEachSideDid) {
    const MemoryBudget at_create = ext_budget(7 * kGiB, 3 * kGiB);
    const MemoryBudget now = ext_budget(4 * kGiB, 5 * kGiB);
    const std::string text = device_lost_text("submit", kDevice, &at_create, &now);
    for (const std::string_view needle :
         {"the budget fell by 3221225472 bytes (another program took VRAM after the run began)",
          "this process holds 2147483648 bytes more than at create",
          "over its budget by 1073741824 bytes (the driver pages to system memory, the usual way a submit passes the 2 s watchdog)"}) {
        EXPECT_TRUE(contains(text, needle)) << "missing '" << needle << "' in: " << text;
    }
}

TEST(DeviceLostText, NothingChangedSaysSoAndOnlyTheWorkIsLeftToBlame) {
    const MemoryBudget same = ext_budget(7 * kGiB, 3 * kGiB);
    const std::string text = device_lost_text("submit", kDevice, &same, &same);
    EXPECT_TRUE(contains(text, "neither the budget nor this process's use changed, so look at the work itself")) << text;
    EXPECT_FALSE(contains(text, "bytes more")) << text;
    EXPECT_FALSE(contains(text, "another program")) << text;
    // A usage that FELL (and a budget that held) is not a change for the worse either.
    const MemoryBudget fell = ext_budget(7 * kGiB, 2 * kGiB);
    const std::string eased = device_lost_text("submit", kDevice, &same, &fell);
    EXPECT_TRUE(contains(eased, "did not fall and this process's use did not grow")) << eased;
    EXPECT_FALSE(contains(eased, "neither the budget nor this process's use changed")) << eased;
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
