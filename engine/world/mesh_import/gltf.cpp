// world/mesh_import/gltf.cpp -- see world/mesh_import.hpp. A stub at the red
// commit: every read is unavailable.
#include "world/mesh_import.hpp"

#include <expected>

namespace spade {

Result<std::vector<MeshObject>> read_gltf(const std::filesystem::path&) {
    return std::unexpected(Error{Code::unavailable, "read_gltf: not built yet"});
}

Result<std::vector<MeshObject>> read_gltf_bytes(std::span<const std::byte>, const std::filesystem::path&) {
    return std::unexpected(Error{Code::unavailable, "read_gltf_bytes: not built yet"});
}

}  // namespace spade
