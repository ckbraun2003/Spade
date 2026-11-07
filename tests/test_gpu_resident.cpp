// test_gpu_resident.cpp -- device-resident arrays on a device (module API stage 7, Task C2; Review Focus 4).
//
// The test module is resident_module_support.hpp's: one per_world grid (16^3 floats a world) that a kernel increments in every
// substep. A resident array is neither uploaded nor read back by the step; a host read syncs it first, or refuses by name, and
// never returns stale bytes. Every test skips with its reason when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/backend.hpp"
#include "compute/recipe.hpp"
#include "gpu_skip.hpp"
#include "resident_add_one.spv.gen.hpp"
#include "resident_module_support.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "testing/replay.hpp"

namespace {

using spade::Simulation;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::modules::Residency;
using spade::test::kResidentCells;
using spade::test::ResidentModule;
using spade::test::ResidentRow;

constexpr uint32_t kSubsteps = 2;
constexpr uint32_t kWorlds = 2;
constexpr uint64_t kGridBytes = uint64_t{kWorlds} * sizeof(ResidentRow);

void resident_record(spade::compute::RecipeEmitter& emitter, const spade::compute::RecipeContext& context) {
    const uint64_t cells = uint64_t{context.base.world_count} * kResidentCells;
    emitter.dispatch(0, context.base,
                     {static_cast<uint32_t>((cells + context.workgroup_size - 1) / context.workgroup_size), 1, 1});
}
constexpr uint32_t kSet1[] = {spade::compute::gen::kBinding_airflow_u};
constexpr spade::compute::RecipeKernel kKernels[] = {{&spade::compute::gen::kSpvVariants_resident_add_one, kSet1}};
constexpr spade::compute::RecipeDesc kRecipe{.name = "resident_add_one", .kernels = kKernels, .record = &resident_record};

[[nodiscard]] Simulation make(const ResidentModule& m, BackendKind kind, uint32_t bodies = 4) {
    const spade::Result<spade::WorldSetDesc> world = spade::test::floor_world_set(bodies);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<Simulation> sim = Simulation::create(*world, 1'000'000, kSubsteps, BackendDesc{.kind = kind}, m.set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

}  // namespace

TEST(GpuResident, AStepUploadsAndReadsBackNothingResident) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ResidentModule m(Residency::device, &kRecipe);
    Simulation sim = make(m, BackendKind::vulkan);
    ASSERT_TRUE(sim.step(1).has_value());
    const auto first = sim.vulkan_transfer_bytes();
    ASSERT_TRUE(sim.step(9).has_value());
    const auto later = sim.vulkan_transfer_bytes();
    ASSERT_TRUE(first.has_value() && later.has_value());
    EXPECT_EQ(later->uploaded, first->uploaded) << "nothing is uploaded again: the grid is on the device";
    EXPECT_LT((later->read_back - first->read_back) / 9, kGridBytes) << "the per-step readback leaves the grid out";
    EXPECT_GT(first->uploaded, 0u);
}

TEST(GpuResident, ASetRotorCommandsLoopDoesNotReuploadTheGrid) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ResidentModule m(Residency::device, &kRecipe);
    Simulation sim = make(m, BackendKind::vulkan, 8);
    ASSERT_TRUE(sim.step(1).has_value());
    const uint64_t base = sim.vulkan_transfer_bytes()->uploaded;
    // Structural changes between steps mark the mirror dirty each time: the whole non-resident state goes up, the grid does not.
    for (int i = 0; i < 5; ++i) {
        ASSERT_TRUE(sim.spawn(0, spade::test::bare_body(glm::vec3(float(i), 2.0f, 0.0f))).has_value());
        ASSERT_TRUE(sim.step(1).has_value());
    }
    const uint64_t moved = sim.vulkan_transfer_bytes()->uploaded - base;
    EXPECT_LT(moved, 5 * kGridBytes) << "five re-uploads of the whole state would carry the grid five times";
}

TEST(GpuResident, AHostWriteSyncsFirstThenMarksForUpload) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ResidentModule m(Residency::device, &kRecipe);
    Simulation sim = make(m, BackendKind::vulkan);
    ASSERT_TRUE(sim.step(1).has_value());  // every cell 2.0 on the device, the host copy stale
    std::vector<ResidentRow> row(1);
    for (float& c : row[0].cell) c = 100.0f;
    ASSERT_TRUE(sim.write_module_rows("airflow_u", 0, std::as_bytes(std::span(row))).has_value());
    ASSERT_TRUE(sim.step(1).has_value());
    const auto w0 = sim.module_rows<ResidentRow>("airflow_u", 0);
    const auto w1 = sim.module_rows<ResidentRow>("airflow_u", 1);
    ASSERT_TRUE(w0.has_value() && w1.has_value());
    EXPECT_EQ((*w0)[0].cell[17], 102.0f) << "the written world: 100 + two substeps";
    EXPECT_EQ((*w1)[0].cell[17], 4.0f) << "the other world kept the device's value: 2 + two substeps";
}

TEST(GpuResident, AConstSnapshotOfAStaleArrayRefusesNamingTheArrayAndSyncResident) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ResidentModule m(Residency::device, &kRecipe);
    Simulation sim = make(m, BackendKind::vulkan);
    ASSERT_TRUE(sim.step(1).has_value());
    EXPECT_FALSE(sim.resident_current());
    const Simulation& view = sim;
    const auto refused = view.snapshot();
    ASSERT_FALSE(refused.has_value()) << "a const snapshot cannot sync, so it must not return stale bytes";
    EXPECT_EQ(refused.error().code, spade::Code::unavailable);
    EXPECT_NE(refused.error().context.find("airflow_u"), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find("sync_resident"), std::string::npos) << refused.error().context;

    ASSERT_TRUE(sim.sync_resident().has_value());
    EXPECT_TRUE(sim.resident_current());
    EXPECT_TRUE(view.snapshot().has_value());
}

// Review Focus 4: every host path -- snapshot, module_rows, arenas, the digest -- syncs or refuses.
TEST(GpuResident, ModuleRowsAndArenasAndDigestSyncOrRefuse) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ResidentModule m(Residency::device, &kRecipe);
    Simulation sim = make(m, BackendKind::vulkan);
    ASSERT_TRUE(sim.step(1).has_value());
    const Simulation& view = sim;

    const auto stale = view.module_rows<ResidentRow>("airflow_u", 0);
    ASSERT_FALSE(stale.has_value()) << "the const module_rows refuses a stale array";
    EXPECT_EQ(stale.error().code, spade::Code::unavailable);
    EXPECT_NE(stale.error().context.find("airflow_u"), std::string::npos) << stale.error().context;

    const auto checked = spade::testing::checked_state_digest(view);
    ASSERT_FALSE(checked.has_value()) << "the checked digest refuses rather than fold stale bytes";
    EXPECT_NE(checked.error().context.find("airflow_u"), std::string::npos) << checked.error().context;

    // The non-const forms sync.
    const auto rows = sim.module_rows<ResidentRow>("airflow_u", 0);
    ASSERT_TRUE(rows.has_value()) << rows.error().context;
    EXPECT_EQ((*rows)[0].cell[0], 2.0f);
    EXPECT_TRUE(sim.resident_current());

    ASSERT_TRUE(sim.step(1).has_value());
    EXPECT_FALSE(sim.resident_current());
    const uint64_t synced = spade::testing::state_digest(sim);  // the non-const form syncs
    EXPECT_TRUE(sim.resident_current());
    EXPECT_EQ(synced, spade::testing::state_digest(view));
    ASSERT_TRUE(sim.step(1).has_value());
    const auto arenas_rows = sim.arenas();  // non-const arenas() syncs
    (void)arenas_rows;
    EXPECT_TRUE(sim.resident_current());
}

TEST(GpuResident, StagingIsOneBufferSizedToTheLargestRow) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ResidentModule m(Residency::device, &kRecipe);
    Simulation sim = make(m, BackendKind::vulkan);
    const spade::compute::MemoryReport report = sim.memory_report();
    int staging = 0;
    for (const spade::compute::MemoryRecord& r : report.buffers) {
        if (r.name == "resident_staging") {
            ++staging;
            EXPECT_EQ(r.bytes, kGridBytes);
        }
    }
    EXPECT_EQ(staging, 1);
}

TEST(GpuResident, TheDigestFoldsTheResidentArrayAfterASync) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ResidentModule device(Residency::device, &kRecipe);
    const ResidentModule cpu(Residency::device);
    Simulation vk = make(device, BackendKind::vulkan);
    Simulation ref = make(cpu, BackendKind::cpu);
    ASSERT_TRUE(vk.step(3).has_value());
    ASSERT_TRUE(ref.step(3).has_value());
    EXPECT_EQ(spade::testing::state_digest(vk), spade::testing::state_digest(ref))
        << "the grid is exact in fp32 and nothing else moves: D1 = A, the digest folds it after a sync";
}

TEST(GpuResident, AVulkanSnapshotRestoreResumeIsBitIdenticalToTheUninterruptedRun) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ResidentModule m(Residency::device, &kRecipe);
    Simulation straight = make(m, BackendKind::vulkan);
    ASSERT_TRUE(straight.step(6).has_value());

    Simulation first = make(m, BackendKind::vulkan);
    ASSERT_TRUE(first.step(3).has_value());
    const auto blob = first.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    Simulation second = make(m, BackendKind::vulkan);
    ASSERT_TRUE(second.restore(*blob).has_value());
    ASSERT_TRUE(second.step(3).has_value());
    EXPECT_EQ(spade::testing::state_digest(second), spade::testing::state_digest(straight));
}

TEST(GpuResident, ACpuBlobRestoresIntoVulkanWithinTheBandAndTheReverse) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ResidentModule device(Residency::device, &kRecipe);
    const ResidentModule cpu(Residency::device);
    Simulation ref = make(cpu, BackendKind::cpu);
    ASSERT_TRUE(ref.step(6).has_value());

    Simulation first = make(cpu, BackendKind::cpu);
    ASSERT_TRUE(first.step(3).has_value());
    const auto cpu_blob = first.snapshot();
    ASSERT_TRUE(cpu_blob.has_value());
    Simulation vk = make(device, BackendKind::vulkan);
    ASSERT_TRUE(vk.restore(*cpu_blob).has_value());
    ASSERT_TRUE(vk.step(3).has_value());
    EXPECT_EQ(spade::testing::state_digest(vk), spade::testing::state_digest(ref)) << "cpu -> vulkan";

    Simulation vk_first = make(device, BackendKind::vulkan);
    ASSERT_TRUE(vk_first.step(3).has_value());
    const auto vk_blob = vk_first.snapshot();
    ASSERT_TRUE(vk_blob.has_value()) << vk_blob.error().context;
    Simulation back = make(cpu, BackendKind::cpu);
    ASSERT_TRUE(back.restore(*vk_blob).has_value());
    ASSERT_TRUE(back.step(3).has_value());
    EXPECT_EQ(spade::testing::state_digest(back), spade::testing::state_digest(ref)) << "vulkan -> cpu";
}

// A set with a set-1 user builds a second descriptor set; one without builds none (the existing pin is
// GpuStateMirror.ASetWithNoSetOneUserCreatesNoSetOneDescriptorSet).
TEST(GpuResident, ASetOneUserBuildsASecondDescriptorSetAndAStandardSetDoesNot) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const ResidentModule m(Residency::device, &kRecipe);
    Simulation with = make(m, BackendKind::vulkan);
    ASSERT_TRUE(with.vulkan_descriptor_set_count().has_value());
    EXPECT_EQ(*with.vulkan_descriptor_set_count(), 2u);

    Simulation standard = [] {
        const spade::Result<spade::WorldSetDesc> world = spade::test::floor_world_set(4);
        spade::Result<Simulation> sim = Simulation::create(*world, 1'000'000, kSubsteps, BackendDesc{.kind = BackendKind::vulkan},
                                                           spade::modules::standard_modules());
        EXPECT_TRUE(sim.has_value());
        return std::move(*sim);
    }();
    EXPECT_EQ(*standard.vulkan_descriptor_set_count(), 1u);
}
