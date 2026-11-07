// test_gpu_recipes.cpp -- the recorder's use of recipes as data (module API stage 7, Task B1; the stage 7 plan, B1, and the
// lead's ruling of 2026-11-07): the standard chain comes out identical, only the recipes the schedule names get a
// pipeline, and a module's own recipe is recorded where the schedule puts its pass. The recipes themselves are
// test_recipes.cpp (host-only). Every test skips with its reason when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/state_mirror.hpp"
#include "gpu_skip.hpp"
#include "probe_module_support.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

namespace {

using spade::Simulation;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::RecordedChain;
using spade::test::floor_world_set;
using spade::test::ProbeModule;

constexpr uint32_t kSubsteps = 2;

const std::vector<std::string> kStandardGpuOrder = {
    "behaviors.kinematic",    "dryden.advance",          "dryden.sample",       "environment.sample",
    "rotor.forces",           "drag.forces",             "behaviors.force",     "static_contact.resolve",
    "dynamic_contact.resolve", "integrate.integrate",     "imu.synthesize",      "gnss.synthesize"};

[[nodiscard]] Simulation make(const spade::modules::ModuleSet& set) {
    const spade::Result<spade::WorldSetDesc> world = floor_world_set(2);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<Simulation> sim =
        Simulation::create(*world, 1'000'000, kSubsteps, BackendDesc{.kind = BackendKind::vulkan}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

}  // namespace

// A PIN, green at red and it must stay green when the twelve built-ins become RecipeDescs: the standard chain's dispatch
// count, order and barriers, and one recording, and today's twelve pipelines. (The dispatch count is the existing
// GpuStateMirror test's formula: eleven dispatches per substep outside the bitonic network, plus its stages.)
TEST(GpuRecordedChain, TheStandardChainIsUnchanged) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation sim = make(spade::modules::standard_modules());
    const auto chains = sim.vulkan_recorded_chains();
    ASSERT_TRUE(chains.has_value()) << chains.error().context;
    ASSERT_EQ(chains->size(), 1u) << "no cadence: one recording";
    const RecordedChain& chain = (*chains)[0];
    EXPECT_EQ(chain.passes, kStandardGpuOrder);
    EXPECT_EQ(chain.dispatches, kSubsteps * (11u + chain.sort_stages));
    EXPECT_EQ(chain.barriers, chain.dispatches - 1u) << "CORE-2: a barrier between every adjacent pair, none after the last";
    const auto pipelines = sim.vulkan_pipeline_count();
    ASSERT_TRUE(pipelines.has_value());
    EXPECT_EQ(*pipelines, 12u);
}

TEST(GpuRecordedChain, OnlyTheRecipesTheScheduleNamesAreBuilt) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    // A module's own recipe brings one pipeline more.
    const ProbeModule probe;
    Simulation with_probe = make(probe.set);
    ASSERT_TRUE(with_probe.vulkan_pipeline_count().has_value());
    EXPECT_EQ(*with_probe.vulkan_pipeline_count(), 13u);

    // A set without the gnss module never names sensor_gnss: its pipeline is not built.
    // (The module keeps its array, which the engine's own calls need; it just has no pass.)
    spade::modules::ModuleSet no_gnss = spade::modules::standard_modules();
    const auto gnss = std::ranges::find_if(no_gnss, [](const spade::modules::ModuleDesc& m) { return m.name == "gnss"; });
    ASSERT_NE(gnss, no_gnss.end());
    gnss->passes = {};
    Simulation without = make(no_gnss);
    ASSERT_TRUE(without.vulkan_pipeline_count().has_value());
    EXPECT_EQ(*without.vulkan_pipeline_count(), 11u);
}

// A module's recipe is recorded where the schedule puts its pass, once per substep, with the barriers CORE-2 asks for.
TEST(GpuRecordedChain, AModulesOwnRecipeIsRecordedWhereTheSchedulePutsIt) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    Simulation standard = make(spade::modules::standard_modules());
    const ProbeModule probe;
    Simulation with_probe = make(probe.set);
    const auto base = standard.vulkan_recorded_chain();
    const auto chain = with_probe.vulkan_recorded_chain();
    ASSERT_TRUE(base.has_value() && chain.has_value());

    const auto at = std::ranges::find(chain->passes, "probe.step");
    ASSERT_NE(at, chain->passes.end()) << "the module's pass is in the recorded GPU chain";
    std::vector<std::string> without = chain->passes;
    without.erase(without.begin() + (at - chain->passes.begin()));
    EXPECT_EQ(without, kStandardGpuOrder) << "the standard passes keep their order around it";
    EXPECT_EQ(chain->dispatches, base->dispatches + kSubsteps) << "one dispatch per substep";
    EXPECT_EQ(chain->barriers, chain->dispatches - 1u);
}

// Review Focus 2: a set with no set-1 user builds no set-1 descriptor set. A PIN today (nothing builds set 1); it must
// stay green when batch D gives the mirror a set-1 user.
TEST(GpuStateMirror, ASetWithNoSetOneUserCreatesNoSetOneDescriptorSet) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const spade::modules::CompiledSchedule schedule = *spade::modules::compile_schedule(spade::modules::standard_modules());
    spade::compute::StepShape shape{};
    shape.world_count = 2;
    shape.body_capacity = 8;
    shape.element_capacity = 4;
    shape.sensor_capacity = 2;
    shape.substeps = 1;
    shape.h = 0.001f;
    auto arrays = spade::state_array_shapes(schedule, shape);
    ASSERT_TRUE(arrays.has_value()) << arrays.error().context;
    shape.arrays = std::move(*arrays);
    shape.scratch = spade::scratch_shapes(schedule, shape);

    auto ctx = spade::compute::VulkanContext::create(BackendDesc{.kind = BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    auto mirror = spade::compute::StateMirror::create(**ctx, shape);
    ASSERT_TRUE(mirror.has_value()) << mirror.error().context;
    EXPECT_EQ((*mirror)->descriptor_set_count(), 1u) << "set 0 only";
}
