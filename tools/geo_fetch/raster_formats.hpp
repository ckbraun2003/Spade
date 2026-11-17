#pragma once

// ---------------------------------------------------------------------------
// geo_fetch/raster_formats.hpp -- the two answers' file formats, read just enough to check them and store them (L85, SP-5).
//
//   read_tiff_f32   3DEP's answer: a baseline TIFF of one band of float32, uncompressed, in strips. Anything else is refused
//                   by name (a compressed answer says which compression, so the live check can tell what the service sends).
//   read_png_header NAIP's answer: the PNG's signature and IHDR, so the tool can check the size, the bit depth and that it is
//                   RGB before it stores the file as received. The pixels are not decoded here (decode_png is Rendering's).
// ---------------------------------------------------------------------------

#include <cstdint>
#include <string_view>
#include <vector>

#include "core/error.hpp"

namespace spade::geofetch {

struct TiffGrid {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<float> samples;  // width * height, row 0 first (the TIFF's own order: the north edge for these services)
};

[[nodiscard]] Result<TiffGrid> read_tiff_f32(std::string_view bytes);

struct PngHeader {
    uint32_t width = 0;
    uint32_t height = 0;
    int bit_depth = 0;
    int color_type = 0;  // 2 is RGB, 6 is RGBA
};

[[nodiscard]] Result<PngHeader> read_png_header(std::string_view bytes);

}  // namespace spade::geofetch
