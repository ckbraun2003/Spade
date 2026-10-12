// world/mesh.cpp -- see mesh.hpp. RED STUB (Task B1 step 2): accepts every
// input, scales nothing and hashes nothing.
#include "world/mesh.hpp"

namespace spade {

double mesh_unit_scale(MeshUnit /*unit*/) noexcept {
    return 0.0;
}

Result<std::shared_ptr<const TriangleMesh>> make_mesh(const MeshInput& /*input*/, std::string_view /*name*/) {
    return std::shared_ptr<const TriangleMesh>(new TriangleMesh());
}

}  // namespace spade
