// world/mesh_import/stl.cpp -- the binary STL reader. See
// world/mesh_import.hpp for the contract, the refusals and their order.
#include "world/mesh_import.hpp"

#include <expected>

namespace spade {

Result<std::vector<MeshObject>> read_stl(const std::filesystem::path& path, MeshUnit unit) {
    (void)path;
    (void)unit;
    return std::unexpected(Error{Code::unavailable, "read_stl: not built yet"});
}

}  // namespace spade
