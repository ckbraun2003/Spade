// probe_module_support.hpp -- a test module whose one pass has its OWN GPU recipe (module API stage 7, Tasks B1 and B2).
//
// The pass "probe.step" dispatches tests/kernels/recipe_probe.slang (a kernel that does nothing) once per recording of
// its substep, so the recorder's handling of a module's recipe -- pipelines on demand, the recorded order, the cadence's
// two recordings, the timing by name -- is observable without any physics. Its CPU function does nothing either.
// Built in place and never copied: the pass span points into the object.
#pragma once

#include <array>
#include <cstdint>

#include "compute/recipe.hpp"
#include "physics/schedule.hpp"
#include "recipe_probe.spv.gen.hpp"
#include "sim/module.hpp"

namespace spade::test {

inline void probe_cpu(const physics::SubstepContext&) noexcept {}

inline void probe_record(compute::RecipeEmitter& emitter, const compute::RecipeContext& context) {
    emitter.dispatch(0, context.base, {1, 1, 1});
}

inline constexpr compute::RecipeKernel kProbeKernels[] = {{&compute::gen::kSpvVariants_recipe_probe, {}}};
inline constexpr compute::RecipeDesc kProbeRecipe{.name = "recipe_probe", .kernels = kProbeKernels, .record = &probe_record};

inline constexpr modules::QuantityAccess kProbeAccess[] = {{"body.pose", modules::Access::read}};

struct ProbeModule {
    std::array<modules::PassDecl, 1> passes;
    modules::ModuleSet set;

    // `every_steps` 1 is every substep of every step; j > 1 is the cadence.
    explicit ProbeModule(uint32_t every_steps = 1) {
        passes[0] = {.name = "step", .phase = modules::Phase::sensors, .access = kProbeAccess, .cpu = &probe_cpu,
                     .every_steps = every_steps, .recipe = &kProbeRecipe};
        set = modules::standard_modules();
        set.push_back(modules::ModuleDesc{.name = "probe", .passes = passes});
    }
    ProbeModule(const ProbeModule&) = delete;
    ProbeModule& operator=(const ProbeModule&) = delete;
};

}  // namespace spade::test
