#include "vehicles/model_part.hpp"

// ---------------------------------------------------------------------------
// part_sample() -- one part's field, whichever form its geometry has (CORE-6,
// CORE-7; the parts and meshes plan, Task B4). See model_part.hpp.
// ---------------------------------------------------------------------------

namespace spade::vehicles {

SdfSample part_sample(const PartDesc& part, glm::vec3 p_part) noexcept {
    return sample(part.shape, p_part);
}

}  // namespace spade::vehicles
