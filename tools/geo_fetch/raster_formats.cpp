#include "raster_formats.hpp"

namespace spade::geofetch {

// RED: stubs.
Result<TiffGrid> read_tiff_f32(std::string_view) { return TiffGrid{}; }
Result<PngHeader> read_png_header(std::string_view) { return PngHeader{}; }

}  // namespace spade::geofetch
