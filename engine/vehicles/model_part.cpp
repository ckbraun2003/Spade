#include "vehicles/model_part.hpp"

// ---------------------------------------------------------------------------
// part_sample() -- one part's field, whichever form its geometry has (CORE-6,
// CORE-7; the parts and meshes plan, Task B4). See model_part.hpp.
//
// The mesh branch hands back mesh_sample()'s distance and gradient as they
// are: exact Euclidean distance, negative inside, and a unit gradient out of
// the solid -- the {distance, gradient} contract sample() keeps for an
// SdfProgram part. The closest triangle stays in the mesh's own MeshSample;
// no caller of the part interface needs it yet. A non-finite p_part gives
// mesh_sample()'s MeshSample{}: kSdfEmptyDistance and a zero gradient, which
// never wins a smallest-distance comparison.
// ---------------------------------------------------------------------------

namespace spade::vehicles {

SdfSample part_sample(const PartDesc& part, glm::vec3 p_part) noexcept {
    if (part.mesh != nullptr) {
        const MeshSample s = mesh_sample(*part.mesh, p_part);
        return SdfSample{.distance = s.distance, .gradient = s.gradient};
    }
    return sample(part.shape, p_part);
}

}  // namespace spade::vehicles

namespace spade::vehicles {

// RED (part_bounds): declared, not yet implemented.
Result<Aabb> part_bounds(const PartDesc& /*part*/) {
    return std::unexpected(Error{Code::internal, "part_bounds: not implemented yet"});
}

}  // namespace spade::vehicles
