#include "vehicles/model_part.hpp"

#include <span>
#include <string>

#include <glm/common.hpp>

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
//
// part_bounds() -- one part's box, whichever form (the lead's ask of
// 2026-10-13). A mesh's solid is the polyhedron its triangles close, and
// make_mesh() refuses an unreferenced vertex, so the vertices' least and
// greatest are its box exactly. A shape's is sdf_bounds()'s, with the part
// named in front of any refusal.
// ---------------------------------------------------------------------------

namespace spade::vehicles {

SdfSample part_sample(const PartDesc& part, glm::vec3 p_part) noexcept {
    if (part.mesh != nullptr) {
        const MeshSample s = mesh_sample(*part.mesh, p_part);
        return SdfSample{.distance = s.distance, .gradient = s.gradient};
    }
    return sample(part.shape, p_part);
}

Result<Aabb> part_bounds(const PartDesc& part) {
    const std::string who = "part '" + part.name + "': ";
    const bool has_shape = !part.shape.empty();
    const bool has_mesh = part.mesh != nullptr;
    if (has_shape && has_mesh) {
        return std::unexpected(
            Error{Code::invalid_argument, who + "both shape and mesh are set: a part's geometry is exactly one of them"});
    }
    if (!has_shape && !has_mesh) {
        return std::unexpected(
            Error{Code::invalid_argument, who + "neither shape nor mesh is set, so it has no solid to bound"});
    }

    if (has_mesh) {
        const std::span<const glm::vec3> vertices = part.mesh->positions();
        if (vertices.empty()) {
            return std::unexpected(Error{Code::invalid_argument, who + "its mesh has no vertices"});
        }
        Aabb box{vertices.front(), vertices.front()};
        for (const glm::vec3& v : vertices) {
            box.min = glm::min(box.min, v);
            box.max = glm::max(box.max, v);
        }
        return box;
    }

    Result<Aabb> box = sdf_bounds(part.shape);
    if (!box) {
        return std::unexpected(Error{box.error().code, who + box.error().context});
    }
    return box;
}

}  // namespace spade::vehicles
