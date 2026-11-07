// test_memory_report.cpp -- memory_report(), the VRAM cap and the allocation ledger (module API stage 7, Task A3;
// docs/design/core/plans/2026-11-06-module-api-stage7-plan.md; design 7.4, spec Q10).
//
// "Announced" means this query, not a log line: every buffer the backend created, with its kind, bytes, memory type and
// heap, and the host arena's bytes. BackendDesc::vram_cap_bytes can only LOWER what create() admits (check_vram against
// min(cap, the device's budget)); it changes no result, only whether create() refuses, so it is a result-neutral knob
// (engine A7) with its invariance test below.
//
// The CPU half runs everywhere; every Gpu* test skips with its reason when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "compute/vulkan/backend.hpp"
#include "compute/vulkan/state_mirror.hpp"
#include "gpu_skip.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "testing/replay.hpp"

namespace {

using spade::Result;
using spade::Simulation;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::MemoryKind;
using spade::compute::MemoryRecord;
using spade::compute::MemoryReport;
using spade::compute::StepShape;
using spade::test::bare_body;
using spade::test::floor_world_set;

constexpr uint64_t kDt = 1'000'000;

[[nodiscard]] Simulation make(const BackendDesc& backend) {
    const Result<spade::WorldSetDesc> world = floor_world_set(4);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    Result<Simulation> sim = Simulation::create(*world, kDt, 2, backend);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

// The registered walk's bytes on the host: what the mirror sizes each walk entry from.
[[nodiscard]] uint64_t walk_bytes(const Simulation& sim) {
    StepShape shape{};
    shape.world_count = sim.layout().world_count;
    shape.body_capacity = sim.layout().body_capacity;
    shape.element_capacity = sim.layout().element_capacity;
    shape.sensor_capacity = sim.layout().sensor_capacity;
    shape.substeps = sim.substeps();
    shape.h = sim.substep_h();
    shape.batch_dynamic_collision = sim.layout().uniform_dynamic_params;
    auto arrays = spade::state_array_shapes(sim.schedule(), shape);
    EXPECT_TRUE(arrays.has_value());
    uint64_t total = 0;
    for (const auto& a : *arrays) total += uint64_t{shape.world_count} * a.capacity_per_world * a.elem_size;
    return total;
}

}  // namespace

TEST(MemoryReport, TheCpuBackendReportsOnlyTheHostArena) {
    Simulation sim = make(BackendDesc{});
    const MemoryReport report = sim.memory_report();
    EXPECT_TRUE(report.buffers.empty());
    EXPECT_EQ(report.device_bytes, 0u);
    EXPECT_EQ(report.staging_bytes, 0u);
    EXPECT_GT(report.host_arena_bytes, 0u);
    EXPECT_EQ(report.host_arena_bytes, walk_bytes(sim));
}

TEST(GpuMemoryReport, ListsEveryBufferWithKindBytesMemoryTypeAndHeap) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation sim = make(BackendDesc{.kind = BackendKind::vulkan});
    const MemoryReport report = sim.memory_report();
    ASSERT_FALSE(report.buffers.empty());
    uint64_t device = 0;
    uint64_t staging = 0;
    for (const MemoryRecord& r : report.buffers) {
        EXPECT_FALSE(r.name.empty());
        EXPECT_GT(r.bytes, 0u) << r.name;
        (r.staging ? staging : device) += r.bytes;
    }
    EXPECT_EQ(report.device_bytes, device);
    EXPECT_EQ(report.staging_bytes, staging);
    EXPECT_EQ(report.host_arena_bytes, walk_bytes(sim));
    // Every walk entry has a device record of the arena's own size.
    uint64_t walk_device = 0;
    for (const MemoryRecord& r : report.buffers) {
        if (!r.staging && r.kind == MemoryKind::walk) walk_device += r.bytes;
    }
    EXPECT_EQ(walk_device, walk_bytes(sim));
    // The standard set's declared scratch is listed as scratch, with no staging twin (Task A2).
    const auto scratch = std::ranges::find_if(report.buffers, [](const MemoryRecord& r) { return r.name == "contact_dv"; });
    ASSERT_NE(scratch, report.buffers.end());
    EXPECT_EQ(scratch->kind, MemoryKind::scratch);
    EXPECT_FALSE(scratch->staging);
    EXPECT_EQ(std::ranges::count_if(report.buffers, [](const MemoryRecord& r) { return r.name == "contact_dv"; }), 1);
}

TEST(GpuMemoryReport, TheDummyForAnAbsentOptionalBindingIsReported) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation sim = make(BackendDesc{.kind = BackendKind::vulkan});
    const MemoryReport report = sim.memory_report();
    const auto dummy = std::ranges::find_if(report.buffers, [](const MemoryRecord& r) { return r.name == "lock_rows (absent)"; });
    ASSERT_NE(dummy, report.buffers.end()) << "the standard set holds no lock array";
    EXPECT_EQ(dummy->kind, MemoryKind::dummy);
    EXPECT_EQ(dummy->bytes, 16u);
    EXPECT_FALSE(dummy->staging);
}

// Design 7.3: the staging twin is host-visible memory and not device-local on a discrete device. On a unified-memory
// device (every heap device-local) there is nothing to assert and the test says so.
TEST(GpuMemoryReport, StagingIsNotDeviceLocalOnADiscreteDevice) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation sim = make(BackendDesc{.kind = BackendKind::vulkan});
    const MemoryReport report = sim.memory_report();
    ASSERT_TRUE(std::ranges::any_of(report.buffers, [](const MemoryRecord& r) { return r.staging; }));
    // On a unified-memory device every chosen type is device-local, and there is nothing to assert.
    const bool all_device_local = std::ranges::all_of(report.buffers, [](const MemoryRecord& r) { return r.device_local; });
    if (all_device_local) GTEST_SKIP() << "unified memory: every chosen type is device-local";
    for (const MemoryRecord& r : report.buffers) {
        if (r.staging) {
            EXPECT_FALSE(r.device_local) << "staging buffer '" << r.name << "' sits in device-local memory";
        }
    }
}

// The cap can only lower what create() admits, and the refusal names what asked, the cap and the device (check_vram).
TEST(GpuMemoryReport, ACapBelowTheTotalRefusesCreateNamingTheLargestBuffer) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation uncapped = make(BackendDesc{.kind = BackendKind::vulkan});
    const MemoryReport report = uncapped.memory_report();
    ASSERT_FALSE(report.buffers.empty());
    const auto largest = std::ranges::max_element(report.buffers, {}, [](const MemoryRecord& r) { return r.staging ? 0 : r.bytes; });

    const Result<spade::WorldSetDesc> world = floor_world_set(4);
    ASSERT_TRUE(world.has_value());
    Result<Simulation> capped = Simulation::create(*world, kDt, 2, BackendDesc{.kind = BackendKind::vulkan, .vram_cap_bytes = 1024});
    ASSERT_FALSE(capped.has_value()) << "a 1 KiB cap must refuse the standard shape";
    EXPECT_EQ(capped.error().code, spade::Code::capacity_exceeded) << capped.error().context;
    EXPECT_NE(capped.error().context.find("VRAM"), std::string::npos) << capped.error().context;
    EXPECT_NE(capped.error().context.find("cap is 1024"), std::string::npos) << capped.error().context;
    EXPECT_NE(capped.error().context.find(largest->name), std::string::npos)
        << "the refusal names the largest buffer '" << largest->name << "': " << capped.error().context;
}

// The result-neutral knob: a generous cap and no cap give the same digest after the same steps (engine A7). A PIN: green
// at red (the cap is ignored there), and it must stay green when the cap starts to act.
TEST(GpuMemoryReport, ACapChangesNoResult) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    uint64_t digests[2] = {0, 0};
    const uint64_t caps[2] = {0, uint64_t{1} << 40};
    for (int i = 0; i < 2; ++i) {
        Simulation sim = make(BackendDesc{.kind = BackendKind::vulkan, .vram_cap_bytes = caps[i]});
        ASSERT_TRUE(sim.spawn(0, bare_body(glm::vec3(0.0f, 3.0f, 0.0f))).has_value());
        ASSERT_TRUE(sim.flush_structural().has_value());
        for (int s = 0; s < 20; ++s) ASSERT_TRUE(sim.step(1).has_value());
        digests[i] = spade::testing::state_digest(sim);
    }
    EXPECT_EQ(digests[0], digests[1]);
}

namespace {

// Creates the backend for a shape with the given capacities and returns the refusal, after checking the ledger did not
// move: a refusal happens before the first allocation (Review Focus 1; the lead's note on Task A1: this assertion lives
// here, where the ledger exists).
[[nodiscard]] spade::Error refused_without_allocating(uint32_t body_capacity, uint32_t sensor_capacity) {
    Simulation sim = make(BackendDesc{.kind = BackendKind::vulkan});
    const uint64_t before = spade::compute::mirror_allocation_count();
    EXPECT_GT(before, 0u) << "a created mirror has allocated; the ledger must count it";

    StepShape absurd{};
    absurd.world_count = 1;
    absurd.body_capacity = body_capacity;
    absurd.element_capacity = 1;
    absurd.sensor_capacity = sensor_capacity;
    absurd.substeps = 1;
    absurd.h = 0.001f;
    const spade::modules::CompiledSchedule schedule = *spade::modules::compile_schedule(spade::modules::standard_modules());
    auto arrays = spade::state_array_shapes(schedule, absurd);
    EXPECT_TRUE(arrays.has_value()) << arrays.error().context;
    if (!arrays) return arrays.error();
    absurd.arrays = std::move(*arrays);
    const auto passes = spade::modules::gpu_passes(schedule);
    const Result<std::unique_ptr<spade::compute::VulkanBackend>> backend =
        spade::compute::VulkanBackend::create(BackendDesc{.kind = BackendKind::vulkan}, absurd, passes);
    EXPECT_FALSE(backend.has_value());
    EXPECT_EQ(spade::compute::mirror_allocation_count(), before) << "the refusal came after an allocation";
    if (backend) return spade::Error{spade::Code::internal, "created"};
    return backend.error();
}

}  // namespace

// The ledger counts the mirror's allocations. A valid create counts more than zero; an over-limit create leaves the
// count where it was. Sensor rows hundreds of GB wide reach the device's range check, named.
TEST(GpuMemoryReport, ARefusalHappensBeforeAnyAllocation) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::Error refused = refused_without_allocating(1, 0x3FFFFFFu);  // imu_ring: 64 deep, ~150 GB
    EXPECT_EQ(refused.code, spade::Code::capacity_exceeded) << refused.context;
    EXPECT_NE(refused.context.find("maxStorageBufferRange"), std::string::npos) << refused.context;
}

// The other refusal that was found: a body_capacity of 0xFFFFFFFF never reaches the device's limits, because the
// CollisionDynamic key array would pass kMaxGridEntries -- refused by that name, before any allocation.
TEST(GpuMemoryReport, ABodyShapePastTheGridCeilingIsRefusedByNameBeforeAnyAllocation) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::Error refused = refused_without_allocating(0xFFFFFFFFu, 1);
    EXPECT_NE(refused.context.find("kMaxGridEntries"), std::string::npos) << refused.context;
}
