// world/mesh_import/obj.cpp -- the OBJ reader. See world/mesh_import.hpp for
// the contract and the refusals.
#include "world/mesh_import.hpp"

#include <expected>

namespace spade {

Result<std::vector<MeshObject>> read_obj(const std::filesystem::path& path, MeshUnit unit) {
    (void)path;
    (void)unit;
    return std::unexpected(Error{Code::unavailable, "read_obj: not built yet"});
}

}  // namespace spade
