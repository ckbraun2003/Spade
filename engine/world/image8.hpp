#pragma once

// A decoded 8-bit RGB image: plain data. decode_png() (world/mesh_import.hpp) returns it and the terrain map holds one
// (world/terrain_input.hpp). Rows run top to bottom, three bytes a pixel, no padding: rgb.size() == width * height * 3.

#include <cstdint>
#include <vector>

namespace spade::world {

struct Image8 {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint8_t> rgb;
};

}  // namespace spade::world
