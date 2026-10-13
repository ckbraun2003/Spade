#pragma once

// ---------------------------------------------------------------------------
// world/aabb.hpp -- an axis-aligned box. sdf_bounds() (world/sdf.hpp) and
// part_bounds() (vehicles/model_part.hpp) return one; Physics' P2 sizes its
// grid around a part's box in its part frame.
// ---------------------------------------------------------------------------

#include <glm/vec3.hpp>

namespace spade {

// The points p with min <= p <= max on every axis. A box the engine returns
// has min <= max on every axis, both finite.
struct Aabb {
    glm::vec3 min{0.0f};
    glm::vec3 max{0.0f};
};

}  // namespace spade
