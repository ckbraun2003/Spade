// pass_params.hpp -- the push block every standard dispatch carries (moved from compute/vulkan/step_recorder.hpp, module API
// stage 7 Task B1, so the Vulkan-free recording functions of compute/recipe.hpp can name it). Mirrored by
// engine/shaders/shared/layouts.slang; the generated layout check pins every offset. The history of each lane (and why no
// `contact`, `grid` or `tick` lane exists) is in step_recorder.hpp.
#pragma once

#include <cstdint>

namespace spade::compute {

struct PassParams {
    uint32_t world_count = 0;
    uint32_t body_capacity = 0;
    uint32_t element_capacity = 0;
    uint32_t sensor_capacity = 0;
    float h = 0.0f;
    uint32_t substep = 0;
    uint32_t grid_entry_count = 0;
    uint32_t grid_segment = 0;
    uint32_t grid_segment_slots = 0;
    uint32_t sort_k = 0;
    uint32_t sort_j = 0;
    uint32_t switches = 0;  // compute/optional_bindings.hpp's kSwitch* bits (module API stage 6)
    // Four generic lanes a module's recipe fills (module API stage 7 Task B2): a multigrid level, a colour of a coloured
    // sweep, the sweep's number and the module's own sub-step within a step. The standard recipes leave them 0 and no
    // standard kernel reads them; the push block is 64 bytes, inside the 128-byte guaranteed minimum.
    uint32_t level = 0;
    uint32_t colour = 0;
    uint32_t sweep = 0;
    uint32_t fluid_substep = 0;
};

}  // namespace spade::compute
