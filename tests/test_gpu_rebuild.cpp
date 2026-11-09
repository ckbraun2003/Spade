// test_gpu_rebuild.cpp -- the rebuild hook on a device (module API stage 7, Task C3).
//
// Module "gridded": a resident grid ("airflow_u"), a DERIVED scratch ("airflow_v", not state) and a configuration constant k.
// The step adds the derived buffer to the grid in every substep; the rebuild (an on-demand recording, submitted once at
// create() and once per restore(), never per step) fills the derived buffer with k. A restore that missed its rebuild would
// leave the scratch at zero and the grid would stop growing. Every test skips with its reason when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <utility>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/backend.hpp"
#include "compute/recipe.hpp"
#include "gpu_skip.hpp"
#include "rebuild_add_derived.spv.gen.hpp"
#include "rebuild_fill_derived.spv.gen.hpp"
#include "resident_module_support.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "testing/replay.hpp"

namespace {

using spade::Simulation;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::modules::ModuleDesc;
using spade::test::kResidentCells;
using spade::test::ResidentRow;

// A configuration image must be a type with unique object representations, which a float is not: the value rides as its bit
// pattern (0.25f, exact in fp32) and the kernel reads it back with asfloat.
struct GridConfig {
    uint32_t k_bits;
    uint32_t pad[3];
};
static_assert(sizeof(GridConfig) == 16);
constexpr GridConfig kConfig{0x3e800000u, {0, 0, 0}};
const float kConfigK = std::bit_cast<float>(kConfig.k_bits);

void groups_record(spade::compute::RecipeEmitter& emitter, const spade::compute::RecipeContext& context) {
    const uint64_t cells = uint64_t{context.base.world_count} * kResidentCells;
    emitter.dispatch(0, context.base,
                     {static_cast<uint32_t>((cells + context.workgroup_size - 1) / context.workgroup_size), 1, 1});
}
constexpr uint32_t kAddSet1[] = {spade::compute::gen::kBinding_airflow_u, spade::compute::gen::kBinding_airflow_v};
constexpr uint32_t kFillSet1[] = {spade::compute::gen::kBinding_airflow_v, spade::compute::gen::kBinding_airflow_config};
constexpr spade::compute::RecipeKernel kAddKernels[] = {{&spade::compute::gen::kSpvVariants_rebuild_add_derived, kAddSet1}};
constexpr spade::compute::RecipeKernel kFillKernels[] = {{&spade::compute::gen::kSpvVariants_rebuild_fill_derived, kFillSet1}};
constexpr spade::compute::RecipeDesc kAddRecipe{.name = "rebuild_add_derived", .kernels = kAddKernels, .record = &groups_record};
constexpr spade::compute::RecipeDesc kFillRecipe{.name = "rebuild_fill_derived", .kernels = kFillKernels, .record = &groups_record};

// The CPU halves, the reference the device is held to: the pass adds the derived scratch to the grid; the rebuild fills it.
void add_cpu(const spade::physics::SubstepContext& ctx) noexcept {
    if (ctx.state.size() != 2) return;
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        const std::span<ResidentRow> u = spade::world_rows<ResidentRow>(ctx.state[0], w);
        const std::span<const ResidentRow> v = spade::world_rows<const ResidentRow>(ctx.state[1], w);
        if (u.empty() || v.empty()) continue;
        for (uint32_t i = 0; i < kResidentCells; ++i) u[0].cell[i] += v[0].cell[i];
    }
}
void fill_cpu(const spade::physics::SubstepContext& ctx) noexcept {
    if (ctx.state.size() != 1) return;
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        const std::span<ResidentRow> v = spade::world_rows<ResidentRow>(ctx.state[0], w);
        if (v.empty()) continue;
        for (float& c : v[0].cell) c = kConfigK;
    }
}

constexpr spade::modules::QuantityAccess kAddAccess[] = {{"gridded.airflow_u", spade::modules::Access::write},
                                                         {"gridded.airflow_v", spade::modules::Access::read}};
constexpr spade::modules::QuantityAccess kFillAccess[] = {{"gridded.airflow_v", spade::modules::Access::write}};
constexpr spade::modules::ArrayDecl kArrays[] = {{.name = "airflow_u", .elem_size = sizeof(ResidentRow),
                                                  .extent = spade::modules::Extent::per_world,
                                                  .residency = spade::modules::Residency::device}};
constexpr spade::modules::ScratchDecl kScratch[] = {{.name = "airflow_v", .elem_size = sizeof(ResidentRow),
                                                     .extent = spade::modules::Extent::per_world}};

struct GriddedModule {
    std::array<spade::modules::PassDecl, 1> passes;
    spade::modules::ModuleSet set;

    explicit GriddedModule(bool gpu) {
        passes[0] = {.name = "add_derived", .phase = spade::modules::Phase::sensors, .access = kAddAccess, .cpu = &add_cpu,
                     .recipe = gpu ? &kAddRecipe : nullptr};
        set = spade::modules::standard_modules();
        set.push_back(ModuleDesc{.name = "gridded",
                                 .passes = passes,
                                 .state = kArrays,
                                 .scratch = kScratch,
                                 .grade = {.cpu = spade::Grade::reference, .vulkan = spade::Grade::banded},
                                 .config = spade::modules::config_image(kConfig),
                                 .config_binding = spade::compute::gen::kBinding_airflow_config,
                                 .rebuild_access = kFillAccess,
                                 .rebuild_cpu = &fill_cpu,
                                 .rebuild_gpu = gpu ? &kFillRecipe : nullptr});
    }
    GriddedModule(const GriddedModule&) = delete;
    GriddedModule& operator=(const GriddedModule&) = delete;
};

[[nodiscard]] Simulation make(const GriddedModule& m, BackendKind kind) {
    const spade::Result<spade::WorldSetDesc> world = spade::test::floor_world_set(2);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<Simulation> sim = Simulation::create(*world, 1'000'000, 2, BackendDesc{.kind = kind}, m.set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

}  // namespace

TEST(GpuRebuild, TheOnDemandRecordingIsSubmittedOnceNotPerStep) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const GriddedModule m(true);
    Simulation sim = make(m, BackendKind::vulkan);
    ASSERT_TRUE(sim.vulkan_on_demand_count().has_value());
    EXPECT_EQ(*sim.vulkan_on_demand_count(), 1u) << "once at create()";
    ASSERT_TRUE(sim.step(10).has_value());
    EXPECT_EQ(*sim.vulkan_on_demand_count(), 1u) << "not once per step";

    const auto blob = sim.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    ASSERT_TRUE(sim.restore(*blob).has_value());
    EXPECT_EQ(*sim.vulkan_on_demand_count(), 2u) << "once more at restore()";
}

TEST(GpuRebuild, AStandardSetSubmitsNothingOnDemand) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::Result<spade::WorldSetDesc> world = spade::test::floor_world_set(2);
    spade::Result<Simulation> sim = Simulation::create(*world, 1'000'000, 2, BackendDesc{.kind = BackendKind::vulkan},
                                                       spade::modules::standard_modules());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->step(2).has_value());
    ASSERT_TRUE(sim->vulkan_on_demand_count().has_value());
    EXPECT_EQ(*sim->vulkan_on_demand_count(), 0u);
}

TEST(GpuRebuild, AVulkanRestoreResumeIsBitIdenticalWithADerivedBuffer) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const GriddedModule device(true);
    const GriddedModule cpu(false);
    Simulation straight = make(device, BackendKind::vulkan);
    ASSERT_TRUE(straight.step(6).has_value());

    Simulation first = make(device, BackendKind::vulkan);
    ASSERT_TRUE(first.step(3).has_value());
    const auto blob = first.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    Simulation second = make(device, BackendKind::vulkan);
    ASSERT_TRUE(second.restore(*blob).has_value());
    ASSERT_TRUE(second.step(3).has_value());
    EXPECT_EQ(spade::testing::state_digest(second), spade::testing::state_digest(straight))
        << "a restore that missed its rebuild would leave the derived buffer at zero";

    // The grid grew by k every substep: 6 steps x 2 substeps x 0.25.
    const auto rows = straight.module_rows<ResidentRow>("airflow_u", 0);
    ASSERT_TRUE(rows.has_value()) << rows.error().context;
    EXPECT_EQ((*rows)[0].cell[100], 3.0f);

    // And the cpu run is the same arithmetic.
    Simulation ref = make(cpu, BackendKind::cpu);
    ASSERT_TRUE(ref.step(6).has_value());
    EXPECT_EQ(spade::testing::state_digest(ref), spade::testing::state_digest(straight));
}
