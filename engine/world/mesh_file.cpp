// world/mesh_file.cpp -- see mesh_file.hpp for the layout and the refusals.
#include "world/mesh_file.hpp"

namespace spade {

Result<void> save_mesh_file(const std::filesystem::path&, const TriangleMesh&, const MeshProvenance&) {
    return std::unexpected(Error{Code::unavailable, "save_mesh_file: not built yet"});
}

Result<LoadedMesh> load_mesh_file(const std::filesystem::path&, std::optional<uint64_t>) {
    return std::unexpected(Error{Code::unavailable, "load_mesh_file: not built yet"});
}

}  // namespace spade
