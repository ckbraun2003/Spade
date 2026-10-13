// world/mesh_import/weld.cpp -- see weld.hpp. A stub at the red commit: it
// copies its input through unwelded.
#include "world/mesh_import/weld.hpp"

namespace spade::mesh_import {

MeshInput weld_exact(std::span<const glm::dvec3> positions, std::span<const uint32_t> indices, MeshUnit unit) {
    MeshInput out;
    out.positions.assign(positions.begin(), positions.end());
    out.indices.assign(indices.begin(), indices.end());
    out.unit = unit;
    return out;
}

}  // namespace spade::mesh_import
