#include "render/mesh_resolver.hpp"

namespace spade::render {

// RED: stubs until the green step.
MeshData flat_mesh(const TriangleMesh&) { return {}; }

MeshData missing_mesh_stand_in() { return {}; }

ResolvedMeshes resolve_meshes(const WorldDesc&, std::span<const std::filesystem::path>) { return {}; }

}  // namespace spade::render
