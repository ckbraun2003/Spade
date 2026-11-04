// tools/mesh_pack/mesh_pack.cpp -- spade_mesh_pack's run(). See mesh_pack.hpp.
#include "mesh_pack.hpp"

namespace spade::mesh_pack {

Outcome run(std::span<const std::filesystem::path> /*args*/) {
    return Outcome{1, {}, "spade_mesh_pack: not built yet (S1 RED)\n"};
}

}  // namespace spade::mesh_pack
