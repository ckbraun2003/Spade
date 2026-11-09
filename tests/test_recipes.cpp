// test_recipes.cpp -- the built-in GPU recipes as data and recording functions (module API stage 7, Task B1; the lead's
// ruling of 2026-11-07: one path, the twelve built-ins converted to the same form as a module's own recipe, no global
// registry). Host-only: a recording function appends plain data to an emitter, so the whole table is checked without a
// device. The recorder's use of it (the standard chain unchanged, pipelines on demand) is test_gpu_recipes.cpp.
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "compute/backend.hpp"
#include "compute/pass_params.hpp"
#include "compute/recipe.hpp"

namespace {

using spade::compute::builtin_recipe;
using spade::compute::EmittedDispatch;
using spade::compute::GpuRecipe;
using spade::compute::PassParams;
using spade::compute::RecipeContext;
using spade::compute::RecipeDesc;
using spade::compute::RecipeEmitter;
using Groups = std::array<uint32_t, 3>;

constexpr uint32_t kBodies = 11;
constexpr uint32_t kGrid = 13;
constexpr uint32_t kWorlds = 3;
constexpr uint32_t kSensors = 5;

[[nodiscard]] RecipeContext context(uint32_t segment = 4, uint32_t substep = 1) {
    RecipeContext c;
    c.base.world_count = 2;
    c.base.body_capacity = 8;
    c.base.substep = substep;
    c.base.grid_segment = segment;
    c.bodies_groups = kBodies;
    c.grid_groups = kGrid;
    c.worlds_groups = kWorlds;
    c.sensors_groups = kSensors;
    return c;
}

[[nodiscard]] std::vector<EmittedDispatch> record(const RecipeDesc& desc, const RecipeContext& c) {
    RecipeEmitter emitter;
    desc.record(emitter, c);
    const auto d = emitter.dispatches();
    return {d.begin(), d.end()};
}

[[nodiscard]] std::vector<std::string> kernel_names(const RecipeDesc& desc) {
    std::vector<std::string> out;
    for (const auto& k : desc.kernels) out.emplace_back(k.variants->name);
    return out;
}

struct Row {
    GpuRecipe recipe;
    std::vector<std::string> kernels;
    uint32_t groups_of;  // which of the context's group counts its one dispatch uses
};

}  // namespace

// The emitter is the one real piece here: green at red, a pin. A recording function appends, in order.
TEST(Recipes, TheEmitterKeepsDispatchesInTheOrderTheyWereAppended) {
    RecipeEmitter e;
    e.dispatch(2, PassParams{.substep = 7}, {5, 1, 1});
    e.dispatch(0, PassParams{.substep = 8}, {6, 2, 3});
    ASSERT_EQ(e.dispatches().size(), 2u);
    EXPECT_EQ(e.dispatches()[0].kernel, 2u);
    EXPECT_EQ(e.dispatches()[0].params.substep, 7u);
    EXPECT_EQ(e.dispatches()[1].groups, (Groups{6, 2, 3}));
}

TEST(BuiltinRecipes, NoneHasNoDesc) { EXPECT_EQ(builtin_recipe(GpuRecipe::none), nullptr); }

// The twelve pipelines of today, by the recipe that dispatches them: one kernel each, but collision_dynamic's three.
TEST(BuiltinRecipes, EveryRecipeNamesItsKernelsAndTakesItsOwnGridsGroups) {
    constexpr uint32_t kB = 0, kG = 1, kW = 2, kS = 3;
    const std::vector<Row> rows = {
        {GpuRecipe::medium_update, {"medium_update"}, kW},
        {GpuRecipe::rotors, {"rotors"}, kB},
        {GpuRecipe::drag, {"forces_drag"}, kB},
        {GpuRecipe::drag_points, {"forces_drag_points"}, kB},
        {GpuRecipe::points_positions, {"points_positions"}, kB},
        {GpuRecipe::collision_static, {"collision_static"}, kB},
        {GpuRecipe::integrate, {"integrate"}, kB},
        {GpuRecipe::sensor_imu, {"sensor_imu"}, kS},
        {GpuRecipe::sensor_gnss, {"sensor_gnss"}, kS},
        {GpuRecipe::environment_sample, {"field_environment"}, kW},
        {GpuRecipe::dryden_sample, {"field_dryden"}, kW},
    };
    (void)kG;
    const RecipeContext c = context();
    const std::array<uint32_t, 4> counts = {kBodies, kGrid, kWorlds, kSensors};
    for (const Row& row : rows) {
        const RecipeDesc* desc = builtin_recipe(row.recipe);
        ASSERT_NE(desc, nullptr) << "recipe " << static_cast<int>(row.recipe) << " has no desc";
        EXPECT_FALSE(desc->name.empty());
        EXPECT_EQ(kernel_names(*desc), row.kernels) << desc->name;
        ASSERT_NE(desc->record, nullptr) << desc->name;
        const auto d = record(*desc, c);
        ASSERT_EQ(d.size(), 1u) << desc->name << " is one dispatch";
        EXPECT_EQ(d[0].kernel, 0u);
        EXPECT_EQ(d[0].groups, (Groups{counts[row.groups_of], 1, 1})) << desc->name;
        EXPECT_EQ(d[0].params.substep, c.base.substep) << desc->name << " carries the substep it is recorded at";
        EXPECT_EQ(d[0].params.world_count, c.base.world_count) << desc->name;
    }
}

// CORE-1: a behaviors pass records nothing -- not a stub dispatch, not a barrier, not a pipeline.
TEST(BuiltinRecipes, TheBehaviorsRecordNothing) {
    for (const GpuRecipe r : {GpuRecipe::behaviors_kinematic, GpuRecipe::behaviors_force}) {
        const RecipeDesc* desc = builtin_recipe(r);
        ASSERT_NE(desc, nullptr);
        EXPECT_TRUE(desc->kernels.empty());
        ASSERT_NE(desc->record, nullptr);
        EXPECT_TRUE(record(*desc, context()).empty());
    }
}

// physics/grid.cpp's five-stage shape as three kernels: grid_build, the bitonic network's (k, j) stages, then the sweep.
TEST(BuiltinRecipes, CollisionDynamicUnrollsTheSortNetworkInTheDevicesOrder) {
    const RecipeDesc* desc = builtin_recipe(GpuRecipe::collision_dynamic);
    ASSERT_NE(desc, nullptr);
    EXPECT_EQ(kernel_names(*desc), (std::vector<std::string>{"grid_build", "grid_sort", "collision_dynamic"}));

    const auto d = record(*desc, context(/*segment=*/4));
    // k = 2: j = 1; k = 4: j = 2, 1 -- three sort stages between the build and the sweep.
    ASSERT_EQ(d.size(), 5u);
    EXPECT_EQ(d[0].kernel, 0u);
    EXPECT_EQ(d[0].groups, (Groups{kGrid, 1, 1}));
    const std::array<std::array<uint32_t, 2>, 3> stages = {{{2, 1}, {4, 2}, {4, 1}}};
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(d[1 + i].kernel, 1u);
        EXPECT_EQ(d[1 + i].params.sort_k, stages[i][0]) << "stage " << i;
        EXPECT_EQ(d[1 + i].params.sort_j, stages[i][1]) << "stage " << i;
        EXPECT_EQ(d[1 + i].groups, (Groups{kGrid, 1, 1}));
    }
    EXPECT_EQ(d[4].kernel, 2u);
    EXPECT_EQ(d[4].groups, (Groups{kWorlds, 1, 1})) << "one thread per world";
    // A segment of 1 has no sort network: the build and the sweep only.
    EXPECT_EQ(record(*desc, context(/*segment=*/1)).size(), 2u);
}

// The push block grows by the airflow module's generic lanes (level, colour, sweep, fluid sub-step), four u32, to 64
// bytes: still inside the 128-byte guaranteed minimum push range, and a multiple of 16.
TEST(PassParamsLayout, GrowsByFourGenericLanesAndStaysUnderTheGuaranteedPushRange) {
    EXPECT_EQ(sizeof(PassParams), 64u);
    EXPECT_LE(sizeof(PassParams), 128u);
}
