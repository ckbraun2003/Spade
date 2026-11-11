// builtin_recipes.cpp -- the twelve built-in GPU recipes as RecipeDescs (module API stage 7, Task B1; compute/recipe.hpp).
//
// Moved out of StepRecorder::record()'s switch unchanged: each recipe names its kernel(s) and appends the dispatches the
// switch emitted, on the grid the switch used. The recorded chain of the standard set is therefore the one it always was
// (tests/test_gpu_recipes.cpp's RecordedChain.TheStandardChainIsUnchanged). The generated kernel headers are included here
// and nowhere else in compute/'s Vulkan-free half.
#include <cstdint>

#include "collision_dynamic.spv.gen.hpp"
#include "collision_shape.spv.gen.hpp"
#include "collision_static.spv.gen.hpp"
#include "bindings.gen.hpp"
#include "compute/recipe.hpp"
#include "field_dryden.spv.gen.hpp"
#include "field_environment.spv.gen.hpp"
#include "forces_drag.spv.gen.hpp"
#include "forces_drag_points.spv.gen.hpp"
#include "grid_build.spv.gen.hpp"
#include "grid_sort.spv.gen.hpp"
#include "integrate.spv.gen.hpp"
#include "medium_update.spv.gen.hpp"
#include "points_positions.spv.gen.hpp"
#include "rotors.spv.gen.hpp"
#include "sensor_gnss.spv.gen.hpp"
#include "sensor_imu.spv.gen.hpp"

namespace spade::compute {
namespace {

constexpr RecipeKernel kMedium[] = {{&gen::kSpvVariants_medium_update, {}}};
constexpr RecipeKernel kRotors[] = {{&gen::kSpvVariants_rotors, {}}};
constexpr RecipeKernel kDrag[] = {{&gen::kSpvVariants_forces_drag, {}}};
// POINT SAMPLES (stage 7 D1): both kernels name set-1 binding airflow_points, and read the rows-per-world lane in `sweep`.
constexpr uint32_t kPointsSet1[] = {gen::kBinding_airflow_points};
constexpr RecipeKernel kDragPoints[] = {{&gen::kSpvVariants_forces_drag_points, kPointsSet1}};
constexpr RecipeKernel kPointsPositions[] = {{&gen::kSpvVariants_points_positions, kPointsSet1}};
constexpr RecipeKernel kStatic[] = {{&gen::kSpvVariants_collision_static, {}}};
// RIGID CONTACT (L68): one thread per body slot, naming set-1 binding contact_shapes.
constexpr uint32_t kShapeSet1[] = {gen::kBinding_contact_shapes};
constexpr RecipeKernel kShape[] = {{&gen::kSpvVariants_collision_shape, kShapeSet1}};
// physics/grid.cpp's five-stage shape as three kernels, in the order record_dynamic() indexes them.
constexpr RecipeKernel kDynamic[] = {{&gen::kSpvVariants_grid_build, {}},
                                     {&gen::kSpvVariants_grid_sort, {}},
                                     {&gen::kSpvVariants_collision_dynamic, {}}};
constexpr RecipeKernel kIntegrate[] = {{&gen::kSpvVariants_integrate, {}}};
constexpr RecipeKernel kImu[] = {{&gen::kSpvVariants_sensor_imu, {}}};
constexpr RecipeKernel kGnss[] = {{&gen::kSpvVariants_sensor_gnss, {}}};
constexpr RecipeKernel kEnvironment[] = {{&gen::kSpvVariants_field_environment, {}}};
constexpr RecipeKernel kDryden[] = {{&gen::kSpvVariants_field_dryden, {}}};

// Nothing is recorded -- not a stub dispatch, not a barrier, not a pipeline bind (CORE-1).
void record_nothing(RecipeEmitter&, const RecipeContext&) {}

void record_per_world(RecipeEmitter& e, const RecipeContext& c) { e.dispatch(0, c.base, {c.worlds_groups, 1, 1}); }
void record_per_body(RecipeEmitter& e, const RecipeContext& c) { e.dispatch(0, c.base, {c.bodies_groups, 1, 1}); }
// The two point-sample recipes dispatch per body like their standard twins, with the rows per world in the generic `sweep` lane.
void record_per_body_points(RecipeEmitter& e, const RecipeContext& c) {
    PassParams p = c.base;
    p.sweep = c.point_rows;
    e.dispatch(0, p, {c.bodies_groups, 1, 1});
}
void record_per_sensor(RecipeEmitter& e, const RecipeContext& c) { e.dispatch(0, c.base, {c.sensors_groups, 1, 1}); }

// 1. grid_build, one thread per key-array entry; 2. grid_sort, one dispatch per (k, j) stage of the bitonic network, a
// host-side unroll of the textbook double loop; 3. collision_dynamic, one thread per WORLD (the sweep is sequential within a
// world). A degenerate shape records the same chain with empty grids (a zero group count is legal and runs nothing).
void record_dynamic(RecipeEmitter& e, const RecipeContext& c) {
    e.dispatch(0, c.base, {c.grid_groups, 1, 1});
    for (uint32_t k = 2; k <= c.base.grid_segment; k <<= 1) {
        for (uint32_t j = k >> 1; j > 0; j >>= 1) {
            PassParams stage = c.base;
            stage.sort_k = k;
            stage.sort_j = j;
            e.dispatch(1, stage, {c.grid_groups, 1, 1});
        }
    }
    e.dispatch(2, c.base, {c.worlds_groups, 1, 1});
}

constexpr RecipeDesc kBehaviorsKinematic{"behaviors_kinematic", {}, &record_nothing};
constexpr RecipeDesc kBehaviorsForce{"behaviors_force", {}, &record_nothing};
constexpr RecipeDesc kMediumUpdate{"medium_update", kMedium, &record_per_world};
constexpr RecipeDesc kRotorsDesc{"rotors", kRotors, &record_per_body};
constexpr RecipeDesc kDragDesc{"drag", kDrag, &record_per_body};
constexpr RecipeDesc kDragPointsDesc{"drag_points", kDragPoints, &record_per_body_points};
constexpr RecipeDesc kPointsPositionsDesc{"points_positions", kPointsPositions, &record_per_body_points};
constexpr RecipeDesc kCollisionStatic{"collision_static", kStatic, &record_per_body};
constexpr RecipeDesc kCollisionShape{"collision_shape", kShape, &record_per_body};
constexpr RecipeDesc kCollisionDynamic{"collision_dynamic", kDynamic, &record_dynamic};
constexpr RecipeDesc kIntegrateDesc{"integrate", kIntegrate, &record_per_body};
constexpr RecipeDesc kSensorImu{"sensor_imu", kImu, &record_per_sensor};
constexpr RecipeDesc kSensorGnss{"sensor_gnss", kGnss, &record_per_sensor};
constexpr RecipeDesc kEnvironmentSample{"environment_sample", kEnvironment, &record_per_world};
constexpr RecipeDesc kDrydenSample{"dryden_sample", kDryden, &record_per_world};

}  // namespace

const RecipeDesc* builtin_recipe(GpuRecipe recipe) noexcept {
    switch (recipe) {
        case GpuRecipe::none: return nullptr;
        case GpuRecipe::behaviors_kinematic: return &kBehaviorsKinematic;
        case GpuRecipe::behaviors_force: return &kBehaviorsForce;
        case GpuRecipe::medium_update: return &kMediumUpdate;
        case GpuRecipe::rotors: return &kRotorsDesc;
        case GpuRecipe::drag: return &kDragDesc;
        case GpuRecipe::collision_static: return &kCollisionStatic;
        case GpuRecipe::collision_dynamic: return &kCollisionDynamic;
        case GpuRecipe::integrate: return &kIntegrateDesc;
        case GpuRecipe::sensor_imu: return &kSensorImu;
        case GpuRecipe::sensor_gnss: return &kSensorGnss;
        case GpuRecipe::environment_sample: return &kEnvironmentSample;
        case GpuRecipe::dryden_sample: return &kDrydenSample;
        case GpuRecipe::drag_points: return &kDragPointsDesc;
        case GpuRecipe::points_positions: return &kPointsPositionsDesc;
        case GpuRecipe::collision_shape: return &kCollisionShape;
    }
    return nullptr;
}

}  // namespace spade::compute
