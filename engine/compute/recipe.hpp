// recipe.hpp -- a GPU recipe as data and a recording function (module API stage 7, Task B1; the stage 7 plan, B1, and the
// lead's ruling of 2026-11-07).
//
// A recipe was an enumerator in one switch (compute/backend.hpp's GpuRecipe, step_recorder.cpp's record()). It is now a
// RecipeDesc: the kernels the recipe dispatches (each with the set-1 bindings it names, compute/layout_classes.hpp), and a
// function that appends its dispatches, in order, to an emitter, from the shape and the substep. The twelve built-in
// recipes are RecipeDescs of the same form (builtin_recipe()), so there is ONE path from a pass to its dispatches and no
// global registry: a module's pass carries a pointer to its RecipeDesc (PassDecl::recipe, GpuPass::custom), a built-in
// pass is named by its enumerator, and the recorder asks each for its desc.
//
// Vulkan-free: the recording functions run on the host, append plain data, and are unit-tested without a device. The
// recorder turns the appended dispatches into ChainCommands (compute/vulkan/dispatch_chain.hpp) and barriers.
//
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "compute/backend.hpp"
#include "compute/pass_params.hpp"
#include "compute/spirv_variants.hpp"

namespace spade::compute {

// One kernel of a recipe: its compiled variants, and the set-1 bindings it names (none: a plain state-set kernel).
struct RecipeKernel {
    const SpirvVariantSet* variants = nullptr;
    std::span<const uint32_t> set1_bindings{};
};

// What a recording function may read: the push block every dispatch starts from (the shape's lanes and the substep
// filled in), and the group counts the recorder derived from the shape for the four kinds of grid the standard recipes
// use. A module's own recipe computes its own groups (compute/dispatch_shape.hpp) from the shape.
struct RecipeContext {
    PassParams base{};
    uint32_t workgroup_size = 64;
    uint32_t bodies_groups = 0;   // ceil(world_count * body_capacity / workgroup_size)
    uint32_t grid_groups = 0;     // the key array's entries
    uint32_t worlds_groups = 0;   // one thread per world
    uint32_t sensors_groups = 0;  // one thread per sensor slot
};

struct EmittedDispatch {
    uint32_t kernel = 0;  // an index into the recipe's kernels
    PassParams params{};
    std::array<uint32_t, 3> groups{0, 0, 0};
};

// The recorder's end of a recording function: it appends, in order. A barrier between adjacent dispatches is the
// recorder's (CORE-2), never the recipe's.
class RecipeEmitter {
public:
    void dispatch(uint32_t kernel, const PassParams& params, std::array<uint32_t, 3> groups) {
        dispatches_.push_back(EmittedDispatch{kernel, params, groups});
    }
    [[nodiscard]] std::span<const EmittedDispatch> dispatches() const noexcept { return dispatches_; }

private:
    std::vector<EmittedDispatch> dispatches_{};
};

using RecipeRecordFn = void (*)(RecipeEmitter&, const RecipeContext&);

struct RecipeDesc {
    std::string_view name{};
    std::span<const RecipeKernel> kernels{};
    RecipeRecordFn record = nullptr;
};

// The desc of a built-in recipe; nullptr for GpuRecipe::none. behaviors_kinematic and behaviors_force have a desc with no
// kernels whose record function appends nothing (CORE-1: nothing is recorded, not a stub dispatch).
// Defined in compute/vulkan/builtin_recipes.cpp, where the generated kernel headers are.
[[nodiscard]] const RecipeDesc* builtin_recipe(GpuRecipe recipe) noexcept;

}  // namespace spade::compute
