#include "world/mesh_import.hpp"

namespace spade {

// RED: a stub until tests/test_png_decode.cpp has been run against it.
Result<world::Image8> decode_png(std::span<const std::byte>) { return world::Image8{}; }

}  // namespace spade
