// point_samples_gpu_support.hpp -- a test provider of point samples with a device recipe (module API stage 7, Task D1).
//
// Module "gusts" declares position-dependent points (k per element, host rows) and one Fields pass after points.positions that
// fills its own element rows and writes wind.x = 2 * pos.y, density = pos.x into every row: on the CPU as point_provide_cpu(), on
// the device through kProvideRecipe (tests/kernels/points_probe_provide.slang). Both are exact in fp32. Built in place, never
// copied: the spans point into the object.
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string_view>

#include "bindings.gen.hpp"
#include "compute/recipe.hpp"
#include "core/grade.hpp"
#include "physics/schedule.hpp"
#include "points_probe_provide.spv.gen.hpp"
#include "sim/module.hpp"
#include "state/state_view.hpp"

namespace spade::test {

inline constexpr uint32_t kPointsK = 2;
inline constexpr uint32_t kPointsHostRows = 3;

inline void point_provide_cpu(const physics::SubstepContext& ctx) noexcept {
    if (ctx.state.size() != 2) return;  // state[i] views the pass's i-th access: field.swirl has no view, the points array is state[1]
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        const std::span<physics::PointRow> rows = world_rows<physics::PointRow>(ctx.state[1], w);
        const uint32_t bodies = static_cast<uint32_t>(ctx.worlds[w].bodies.size());
        // The element capacity is not in the view; the rows after the bodies up to the host rows are element and sensor rows:
        // the module places the first kPointsK * elements of them. The test worlds declare 4 force elements.
        for (uint32_t i = 0; i < rows.size(); ++i) {
            physics::PointRow& row = rows[i];
            if (i >= bodies && i < bodies + 4u * kPointsK) row.pos[0] = row.pos[1] = row.pos[2] = static_cast<float>(i);
            row.wind[0] = 2.0f * row.pos[1];
            row.density = row.pos[0];
        }
    }
}

inline void point_provide_record(compute::RecipeEmitter& emitter, const compute::RecipeContext& c) {
    compute::PassParams p = c.base;
    p.sweep = c.point_rows;
    p.level = kPointsK;
    const uint32_t threads = c.base.world_count * c.point_rows;
    emitter.dispatch(0, p, {(threads + c.workgroup_size - 1u) / c.workgroup_size, 1, 1});
}

inline constexpr uint32_t kProvideSet1[] = {compute::gen::kBinding_airflow_points};
inline constexpr compute::RecipeKernel kProvideKernels[] = {{&compute::gen::kSpvVariants_points_probe_provide, kProvideSet1}};
inline constexpr compute::RecipeDesc kProvideRecipe{
    .name = "points_probe_provide", .kernels = kProvideKernels, .record = &point_provide_record};

inline constexpr spade::modules::FieldDecl kSwirlField[] = {
    {.name = "swirl", .kind = spade::modules::FieldKind::scalar, .unit = "1"}};
inline constexpr modules::QuantityAccess kProvideAccess[] = {{"field.swirl", modules::Access::write},
                                                              {"points.airflow_points", modules::Access::write}};
inline constexpr std::string_view kAfterPositions[] = {"points.positions"};

struct PointsGpuModule {
    std::array<modules::PassDecl, 1> passes;
    modules::ModuleSet set;

    explicit PointsGpuModule(bool with_recipe = true) {
        passes[0] = {.name = "sample", .phase = modules::Phase::fields, .access = kProvideAccess, .after = kAfterPositions,
                     .cpu = &point_provide_cpu, .recipe = with_recipe ? &kProvideRecipe : nullptr};
        set = modules::standard_modules();
        set.push_back(modules::ModuleDesc{
            .name = "gusts",
            .passes = passes,
            .fields = kSwirlField,
            .points = modules::PointsDecl{.k = kPointsK, .host_requests = kPointsHostRows, .position_dependent = true},
            .grade = {.cpu = Grade::reference, .vulkan = Grade::banded}});
    }
    PointsGpuModule(const PointsGpuModule&) = delete;
    PointsGpuModule& operator=(const PointsGpuModule&) = delete;
};

}  // namespace spade::test
