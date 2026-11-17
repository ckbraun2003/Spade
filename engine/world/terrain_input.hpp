#pragma once

// ---------------------------------------------------------------------------
// The geo terrain's map data (SP-4/SP-5, KAT's geo plan G2): plain data, no behaviour. The map cache's reader
// (world/map_cache.hpp, Interface's) returns it, and the renderer's attach_terrain() (render/terrain.hpp) takes it, so it
// lives here: render depends on world, not the reverse.
//
// Everything is already decoded and already in metres from the anchor. The reader owns every datum and geodesy step: it
// subtracts the anchor's ground height from the ellipsoid heights and turns the cache's WGS84 box into `extent` with
// the geodesy library's local east-north-up at the anchor.
//
// Both rasters are PIXEL-IS-AREA and cover `extent` edge to edge; a height sample is the value at its pixel's centre.
// Rows run from the NORTH edge to the south, columns from the WEST edge to the east.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <vector>

#include "world/image8.hpp"

namespace spade::world {

// The map's outer edges in metres from the anchor: x east, north up the page (world z is -north).
struct TerrainExtent {
    double east_min = 0.0, east_max = 0.0, north_min = 0.0, north_max = 0.0;
};

struct TerrainInput {
    uint32_t nx = 0, ny = 0;       // the height grid's samples across (west to east) and down (north to south)
    std::vector<float> heights;    // nx * ny, metres up from the anchor, never NaN
    Image8 image;                  // the aerial image, RGB8, any size (it need not match the grid's)
    TerrainExtent extent;
};

}  // namespace spade::world
