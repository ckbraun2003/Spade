// world/mesh_import/threemf.cpp -- the 3MF reader. See world/mesh_import.hpp
// for the contract, the refusals and their order.
#include "world/mesh_import.hpp"

#include <expected>

#include "core/path_text.hpp"

namespace spade {

Result<std::vector<MeshObject>> read_3mf(const std::filesystem::path& path) {
    return std::unexpected(Error{Code::unavailable, path_text(path) + ": the 3MF reader is not built yet (B8 RED)"});
}

}  // namespace spade
