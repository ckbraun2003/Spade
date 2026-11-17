#include "world/terrain_map.hpp"

namespace spade {

Result<world::TerrainInput> read_terrain_map(const std::filesystem::path&, double, double, double) {
    return std::unexpected(Error{Code::unavailable, "RED stub"});
}

}  // namespace spade
