// test_gpu_mirror_staging.cpp -- scratch and derived buffers lose their staging twins (module API stage 7, Task A2;
// docs/design/core/plans/2026-11-06-module-api-stage7-plan.md; design 7.3).
//
// Today every mirror buffer is a device-local buffer plus a host-visible twin the size of the buffer (state_mirror.cpp's
// entry and derived builders). A buffer the host never writes or reads has no use for the twin: the declared scratch
// (never uploaded or read back), body_snapshot (the kernels' own copy) and the dummy bound for an absent optional
// binding (stage 6, zero-filled once). At a 128^3 grid that is about 194 MiB of host-visible memory (answers doc 5).
// The host-written uploads (dryden_params, the SDF buffers, contact_params, grid_params, step_params), the read-on-demand
// diagnostics (step_witness, grid_entries, field_samples) and every walk entry keep theirs.
//
// Every test skips with its reason when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/state_mirror.hpp"
#include "gpu_skip.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

namespace {

using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::DerivedBufferShape;
using spade::compute::StateMirror;
using spade::compute::StepShape;
using spade::compute::VulkanContext;

[[nodiscard]] const spade::modules::CompiledSchedule& standard_schedule() {
    static const spade::modules::CompiledSchedule schedule =
        *spade::modules::compile_schedule(spade::modules::standard_modules());
    return schedule;
}

// The standard set at a small shape, with the standard set's declared scratch (contact_dv) bound, as a Simulation's
// StepShape would be.
[[nodiscard]] StepShape standard_shape() {
    StepShape shape{};
    shape.world_count = 2;
    shape.body_capacity = 8;
    shape.element_capacity = 4;
    shape.sensor_capacity = 2;
    shape.substeps = 1;
    shape.h = 0.001f;
    auto arrays = spade::state_array_shapes(standard_schedule(), shape);
    EXPECT_TRUE(arrays.has_value());
    shape.arrays = std::move(*arrays);
    shape.scratch = spade::scratch_shapes(standard_schedule(), shape);
    return shape;
}

struct Made {
    std::unique_ptr<VulkanContext> ctx;
    std::unique_ptr<StateMirror> mirror;
};

[[nodiscard]] Made make_mirror(const StepShape& shape) {
    Made out;
    auto ctx = VulkanContext::create(BackendDesc{.kind = BackendKind::vulkan});
    EXPECT_TRUE(ctx.has_value()) << ctx.error().context;
    if (!ctx) return out;
    out.ctx = std::move(*ctx);
    auto mirror = StateMirror::create(*out.ctx, shape);
    EXPECT_TRUE(mirror.has_value()) << mirror.error().context;
    if (mirror) out.mirror = std::move(*mirror);
    return out;
}

}  // namespace

TEST(GpuMirrorStaging, AScratchBufferHasNoStagingTwin) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const StepShape shape = standard_shape();
    ASSERT_FALSE(shape.scratch.empty()) << "the standard set declares contact_dv";
    Made made = make_mirror(shape);
    ASSERT_TRUE(made.mirror);
    for (const auto& scratch : shape.scratch) {
        if (scratch.binding == spade::compute::kNoBinding) continue;
        EXPECT_FALSE(made.mirror->has_staging(scratch.name)) << "'" << scratch.name << "' is never uploaded or read back";
    }
}

// The derived buffers nothing on the host writes or reads: body_snapshot, and the dummy for an absent optional binding.
TEST(GpuMirrorStaging, ADerivedBufferTheHostNeverTouchesHasNoStagingTwin) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Made made = make_mirror(standard_shape());
    ASSERT_TRUE(made.mirror);
    EXPECT_FALSE(made.mirror->has_staging("body_snapshot"));
    EXPECT_FALSE(made.mirror->has_staging("lock_rows (absent)")) << "the standard set holds no lock array: a zeroed dummy";
}

// What keeps its twin keeps it: this is the other half of the claim, or "no twins anywhere" would pass the tests above.
TEST(GpuMirrorStaging, WhatTheHostWritesOrReadsKeepsItsStagingTwin) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const StepShape shape = standard_shape();
    Made made = make_mirror(shape);
    ASSERT_TRUE(made.mirror);
    for (const char* name : {"dryden_params", "sdf_nodes", "sdf_transforms", "sdf_world_ranges", "step_params",
                             "step_witness", "contact_params", "grid_params", "grid_entries", "field_samples"}) {
        EXPECT_TRUE(made.mirror->has_staging(name)) << name;
    }
    for (const auto& array : shape.arrays) {
        EXPECT_TRUE(made.mirror->has_staging(array.name)) << "walk entry '" << array.name << "'";
    }
}

// The host-visible footprint is exactly the walk plus the derived buffers that keep a twin.
TEST(GpuMirrorStaging, TheHostVisibleBytesAreTheWalkPlusTheDerivedBuffersThatKeepATwin) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const StepShape shape = standard_shape();
    Made made = make_mirror(shape);
    ASSERT_TRUE(made.mirror);

    const auto derived = spade::compute::derived_buffer_shapes(shape, spade::compute::gen::kBindingCount_state);
    ASSERT_TRUE(derived.has_value()) << derived.error().context;
    uint64_t expected = 0;
    for (const auto& array : shape.arrays) {
        expected += uint64_t{shape.world_count} * array.capacity_per_world * array.elem_size;
    }
    for (const DerivedBufferShape& d : *derived) {
        const bool scratch_or_dummy = d.name == "body_snapshot" || d.name.ends_with(" (absent)") ||
                                      d.name == "contact_dv" || d.name == "world_dv";
        if (!scratch_or_dummy) expected += d.byte_size;
    }
    EXPECT_EQ(made.mirror->host_visible_bytes(), expected);
}
