// test_admit.cpp -- compute::admit(): every device-limit check of a created shape, in one pure function (module API
// stage 7, Task A1; docs/design/core/plans/2026-11-06-module-api-stage7-plan.md; design 7.1).
//
// Host-only: admit() takes plain data and synthetic limits, so these run on every machine. The device half (the real
// StepShape's AdmitShape, and create() refusing before it allocates) is in test_gpu_state_mirror.cpp.
//
// THE LAYOUT RULE under test: the per-stage and per-set storage-buffer limits count every set of a pipeline layout, so
// a layout's count is the SUM over its sets (the lead's 2026-11-06 ruling). The first refusal wins, in the order
// layouts, push block, kernels, buffers, dispatches.
#include <gtest/gtest.h>

#include <initializer_list>
#include <string>
#include <string_view>

#include "compute/admit.hpp"
#include "compute/device_limits.hpp"
#include "core/error.hpp"

namespace {

using spade::Code;
using spade::compute::AdmitBuffer;
using spade::compute::AdmitDispatch;
using spade::compute::AdmitKernel;
using spade::compute::AdmitLayout;
using spade::compute::AdmitShape;
using spade::compute::DeviceLimits;
using spade::compute::kGuaranteedLimits;

constexpr std::string_view kDevice = "Test GPU";

// A roomy device: nothing below is near any of its limits.
[[nodiscard]] DeviceLimits roomy() {
    DeviceLimits l;
    l.max_storage_buffer_range = uint64_t{1} << 32;
    l.max_memory_allocation_size = uint64_t{1} << 32;
    l.max_per_stage_storage_buffers = 1u << 20;
    l.max_set_storage_buffers = 1u << 20;
    l.max_push_constants_size = 256;
    l.max_group_count = {65535, 65535, 65535};
    l.max_group_size = {1024, 1024, 64};
    l.max_group_invocations = 1024;
    l.allocation_size_reported = true;
    return l;
}

// What the standard shape asks for, in miniature: one layout of set 0's thirty bindings, a buffer, a kernel, a
// dispatch and the 48-byte push block.
[[nodiscard]] AdmitShape standard_like() {
    AdmitShape s;
    s.layouts = {AdmitLayout{.name = "state", .bindings_per_set = {30}}};
    s.buffers = {AdmitBuffer{.name = "bodies", .bytes = uint64_t{1} << 20}};
    s.kernels = {AdmitKernel{.name = "integrate", .local = {64, 1, 1}}};
    s.dispatches = {AdmitDispatch{.pass = "integrate", .groups = {4096, 1, 1}}};
    s.push_bytes = 48;
    return s;
}

void expect_refused(const spade::Result<void>& r, std::initializer_list<std::string_view> names) {
    ASSERT_FALSE(r.has_value()) << "admitted";
    EXPECT_EQ(r.error().code, Code::capacity_exceeded) << r.error().context;
    for (const std::string_view n : names) {
        EXPECT_NE(r.error().context.find(n), std::string::npos) << "the refusal must name '" << n << "': " << r.error().context;
    }
}

}  // namespace

// PIN, green at red: a roomy device admits the standard-shaped request and an empty request. Named as pins so a stub
// that admits everything is not mistaken for a passing check.
TEST(Admit, ARoomyDeviceAdmitsAStandardLikeShape) {
    EXPECT_TRUE(spade::compute::admit(standard_like(), roomy(), kDevice).has_value());
    EXPECT_TRUE(spade::compute::admit(AdmitShape{}, roomy(), kDevice).has_value());
}

// Review Focus 1: today's thirty-binding set 0 on a device at Vulkan's guaranteed minimums (four storage buffers per
// stage). Such a device cannot legally create the layout today; admit() turns that into a named refusal.
TEST(Admit, AMinimumSpecDeviceIsRefusedByLayoutCountAndLimit) {
    expect_refused(spade::compute::admit(standard_like(), kGuaranteedLimits, kDevice),
                   {"'state'", "30", "maxPerStageDescriptorStorageBuffers", "(4)", "Test GPU"});
}

// The per-set limit is checked too, after the per-stage one.
TEST(Admit, ThePerSetLimitIsRefusedByName) {
    DeviceLimits l = roomy();
    l.max_set_storage_buffers = 24;
    expect_refused(spade::compute::admit(standard_like(), l, kDevice),
                   {"'state'", "30", "maxDescriptorSetStorageBuffers", "(24)"});
}

// The count is per layout ACROSS sets: set 0's 30 plus a set-1 subset of 3 is 33.
TEST(Admit, TheCountIsPerLayoutAcrossSets) {
    AdmitShape s = standard_like();
    s.layouts = {AdmitLayout{.name = "state+points", .bindings_per_set = {30, 3}}};
    DeviceLimits l = roomy();
    l.max_per_stage_storage_buffers = 32;
    l.max_set_storage_buffers = 32;
    expect_refused(spade::compute::admit(s, l, kDevice),
                   {"'state+points'", "set 0: 30", "set 1: 3", "33", "maxPerStageDescriptorStorageBuffers", "(32)"});
    l.max_per_stage_storage_buffers = 33;
    l.max_set_storage_buffers = 33;
    EXPECT_TRUE(spade::compute::admit(s, l, kDevice).has_value()) << "exactly at the limit is admitted";
}

// Every layout is checked, and the refusal names the one that failed.
TEST(Admit, ASecondLayoutIsCheckedAndNamed) {
    AdmitShape s = standard_like();
    s.layouts.push_back(AdmitLayout{.name = "state+airflow", .bindings_per_set = {30, 28}});
    DeviceLimits l = roomy();
    l.max_per_stage_storage_buffers = 40;
    l.max_set_storage_buffers = 40;
    expect_refused(spade::compute::admit(s, l, kDevice), {"'state+airflow'", "58", "(40)"});
}

TEST(Admit, ABufferOverMaxStorageBufferRangeIsRefusedByName) {
    DeviceLimits l = roomy();
    l.max_storage_buffer_range = uint64_t{1} << 27;  // 128 MiB, Vulkan's guaranteed minimum
    AdmitShape s = standard_like();
    s.buffers = {AdmitBuffer{.name = "bodies", .bytes = 1024}, AdmitBuffer{.name = "airflow_u", .bytes = (uint64_t{1} << 27) + 1}};
    expect_refused(spade::compute::admit(s, l, kDevice), {"'airflow_u'", "maxStorageBufferRange", "134217728", "Test GPU"});
}

TEST(Admit, ABufferOverMaxMemoryAllocationSizeIsRefusedByName) {
    DeviceLimits l = roomy();
    l.max_memory_allocation_size = uint64_t{1} << 30;
    AdmitShape s = standard_like();
    s.buffers = {AdmitBuffer{.name = "airflow_advect", .bytes = (uint64_t{1} << 30) + 1}};
    expect_refused(spade::compute::admit(s, l, kDevice), {"'airflow_advect'", "maxMemoryAllocationSize"});
}

TEST(Admit, ADispatchPastTheGroupCountOnAnyAxisIsRefusedByPassAndAxis) {
    DeviceLimits l = roomy();
    l.max_group_count = {65535, 65535, 65535};
    AdmitShape s = standard_like();
    s.dispatches = {AdmitDispatch{.pass = "integrate", .groups = {1, 1, 1}},
                    AdmitDispatch{.pass = "airflow_step", .groups = {1, 70000, 1}}};
    expect_refused(spade::compute::admit(s, l, kDevice), {"'airflow_step'", "axis y", "maxComputeWorkGroupCount[1]", "70000"});
}

TEST(Admit, AKernelOverTheLocalSizeIsRefusedByKernel) {
    DeviceLimits l = roomy();
    l.max_group_invocations = 256;
    AdmitShape s = standard_like();
    s.kernels = {AdmitKernel{.name = "grid_raster", .local = {8, 8, 8}}};  // 512 invocations
    expect_refused(spade::compute::admit(s, l, kDevice), {"'grid_raster'", "maxComputeWorkGroupInvocations", "512"});
    s.kernels = {AdmitKernel{.name = "wide", .local = {2048, 1, 1}}};
    expect_refused(spade::compute::admit(s, l, kDevice), {"'wide'", "maxComputeWorkGroupSize[0]"});
}

TEST(Admit, APushBlockPastTheLimitIsRefused) {
    DeviceLimits l = roomy();
    l.max_push_constants_size = 128;  // Vulkan's guaranteed minimum
    AdmitShape s = standard_like();
    s.push_bytes = 129;
    expect_refused(spade::compute::admit(s, l, kDevice), {"maxPushConstantsSize", "129", "(128"});
}

// The order: a layout over its limit is named before a buffer over its.
TEST(Admit, TheLayoutIsRefusedBeforeTheBuffer) {
    DeviceLimits l = roomy();
    l.max_per_stage_storage_buffers = 4;
    l.max_storage_buffer_range = 16;
    AdmitShape s = standard_like();  // 30 bindings and a 1 MiB buffer: both over
    const spade::Result<void> r = spade::compute::admit(s, l, kDevice);
    ASSERT_FALSE(r.has_value()) << "admitted";
    expect_refused(r, {"'state'", "maxPerStageDescriptorStorageBuffers"});
    EXPECT_EQ(r.error().context.find("maxStorageBufferRange"), std::string::npos) << r.error().context;
}
