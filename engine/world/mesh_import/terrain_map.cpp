// read_terrain_map (L85e part 2, geo G2 / SP-5): the cache reader. The contract, the error vocabulary and the reason it is in
// spade::mesh_import are in world/terrain_map.hpp. It trusts nothing in an entry: the key, the sizes, the hashes and the pixels are
// all checked, and every limit is checked before the file it guards is read.

#include "world/terrain_map.hpp"

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include "core/geodesy.hpp"
#include "world/map_cache.hpp"
#include "world/mesh_import.hpp"
#include "world/sha256.hpp"

namespace spade {

namespace {

namespace fs = std::filesystem;

static_assert(std::endian::native == std::endian::little, "height.f32 is little-endian float32 and is read as it is");

constexpr std::uint64_t kMaxGeorefBytes = 1024ull * 1024ull;
constexpr std::uint32_t kMaxGridSide = 1024;   // the tool's own limits (tools/geo_fetch/fetch.cpp): the grid
constexpr std::uint32_t kMaxImageSide = 4096;  // and the image

[[nodiscard]] std::unexpected<Error> invalid(const std::string& file, const std::string& why) {
    return std::unexpected(Error{Code::schema_mismatch, "map_invalid: " + file + ": " + why});
}

[[nodiscard]] std::string read_all(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// The size of a file the georef says is there: nullopt (and `problem` set) when it is gone, or over `cap`.
[[nodiscard]] std::optional<std::uint64_t> sized(const fs::path& path, std::uint64_t cap, std::string& problem) {
    std::error_code ec;
    const std::uintmax_t n = fs::file_size(path, ec);
    if (ec) {
        problem = "the file is missing or cannot be read";
        return std::nullopt;
    }
    if (n > cap) {
        problem = "the file is " + std::to_string(n) + " bytes, over the limit of " + std::to_string(cap);
        return std::nullopt;
    }
    return static_cast<std::uint64_t>(n);
}

}  // namespace

Result<world::TerrainInput> read_terrain_map(const fs::path& cache_root, double anchor_lat_deg, double anchor_lon_deg, double side_m) {
    if (const Result<void> anchor = geo::validate_anchor({anchor_lat_deg, anchor_lon_deg, 0.0}); !anchor) {
        return std::unexpected(anchor.error());
    }
    if (!(side_m >= kMapMinSideM && side_m <= kMapMaxSideM)) {
        return std::unexpected(Error{Code::invalid_argument, "the side must be between " + std::to_string(static_cast<int>(kMapMinSideM)) +
                                                                  " and " + std::to_string(static_cast<int>(kMapMaxSideM)) + " m"});
    }

    const std::string key = map_key(anchor_lat_deg, anchor_lon_deg, side_m);
    const fs::path dir = map_entry_dir(cache_root, key);

    // ---- the georef: written last, so without it there is no entry -----------------------------------------------------------
    std::error_code ec;
    if (!fs::is_regular_file(dir / "georef.yaml", ec)) {
        return std::unexpected(Error{Code::not_found, "map_missing: no map for " + key + " in " + dir.string()});
    }
    std::string problem;
    if (!sized(dir / "georef.yaml", kMaxGeorefBytes, problem)) return invalid("georef.yaml", problem);
    const Result<MapGeoref> parsed = parse_georef(read_all(dir / "georef.yaml"));
    if (!parsed) return std::unexpected(parsed.error());
    const MapGeoref& g = *parsed;

    if (g.key != key) return invalid("georef.yaml", "the key is " + g.key + " but the directory is " + key);
    if (g.vertical != "wgs84_ellipsoid_m") {
        return invalid("georef.yaml", "the heights are " + g.vertical + ", not wgs84_ellipsoid_m");
    }
    const std::uint32_t nx = g.elevation.width;
    const std::uint32_t ny = g.elevation.height;
    if (nx < 2 || ny < 2 || nx > kMaxGridSide || ny > kMaxGridSide) {
        return invalid("georef.yaml", "the elevation grid is " + std::to_string(nx) + " x " + std::to_string(ny) + " samples; 2 to " +
                                          std::to_string(kMaxGridSide) + " a side are allowed");
    }
    if (g.image.width < 2 || g.image.height < 2 || g.image.width > kMaxImageSide || g.image.height > kMaxImageSide) {
        return invalid("georef.yaml", "the image is " + std::to_string(g.image.width) + " x " + std::to_string(g.image.height) +
                                          " pixels; 2 to " + std::to_string(kMaxImageSide) + " a side are allowed");
    }
    if (!(std::isfinite(g.west_deg) && std::isfinite(g.east_deg) && std::isfinite(g.south_deg) && std::isfinite(g.north_deg) &&
          g.west_deg < g.east_deg && g.south_deg < g.north_deg)) {
        return invalid("georef.yaml", "the box (west, south, east, north) is not a box");
    }
    const double ground = g.anchor_ground_ellipsoid_height_m;
    if (const Result<void> a = geo::validate_anchor({anchor_lat_deg, anchor_lon_deg, ground}); !a) {
        return invalid("georef.yaml", "the anchor's ground height is not usable: " + a.error().context);
    }

    // ---- the heights -----------------------------------------------------------------------------------------------------------
    const std::uint64_t want_bytes = static_cast<std::uint64_t>(nx) * ny * 4ull;
    const std::optional<std::uint64_t> height_bytes = sized(dir / "height.f32", kMaxMapHeightBytes, problem);
    if (!height_bytes) return invalid("height.f32", problem);
    if (*height_bytes != want_bytes) {
        return invalid("height.f32", "its size is " + std::to_string(*height_bytes) + " bytes but a " + std::to_string(nx) + " x " +
                                         std::to_string(ny) + " grid is " + std::to_string(want_bytes));
    }
    const std::string height_file = read_all(dir / "height.f32");
    if (height_file.size() != want_bytes) return invalid("height.f32", "the file cannot be read");
    if (const std::string hash = sha256_hex(height_file); hash != g.height_sha256) {
        return invalid("height.f32", "its sha256 is " + hash + " but the georef says " + g.height_sha256);
    }

    // ---- the image -------------------------------------------------------------------------------------------------------------
    if (!sized(dir / "image.png", kMaxMapImageBytes, problem)) return invalid("image.png", problem);
    const std::string image_file = read_all(dir / "image.png");
    if (image_file.empty()) return invalid("image.png", "the file cannot be read");
    if (const std::string hash = sha256_hex(image_file); hash != g.image_sha256) {
        return invalid("image.png", "its sha256 is " + hash + " but the georef says " + g.image_sha256);
    }
    Result<world::Image8> image = decode_png(std::as_bytes(std::span<const char>(image_file)));
    if (!image) return invalid("image.png", image.error().context);
    if (image->width != g.image.width || image->height != g.image.height) {
        return invalid("image.png", "it is " + std::to_string(image->width) + " x " + std::to_string(image->height) + " pixels but the georef says " +
                                        std::to_string(g.image.width) + " x " + std::to_string(g.image.height));
    }

    // ---- metres up from the anchor's ground -------------------------------------------------------------------------------------
    world::TerrainInput out;
    out.nx = nx;
    out.ny = ny;
    out.heights.resize(static_cast<std::size_t>(nx) * ny);
    for (std::size_t i = 0; i < out.heights.size(); ++i) {
        float h = 0.0f;
        std::memcpy(&h, height_file.data() + 4 * i, 4);
        if (!std::isfinite(h)) return invalid("height.f32", "sample " + std::to_string(i) + " is not finite");
        out.heights[i] = static_cast<float>(static_cast<double>(h) - ground);
    }
    out.image = std::move(*image);

    // ---- the box in metres: the four corners on the CALLER'S anchor's frame (east is x, north is -z); each edge is the mean of its two
    // corners (the skew of a 1 km box is about 0.15 m).
    const geo::LocalFrame frame = geo::make_local_frame({anchor_lat_deg, anchor_lon_deg, ground});
    const auto corner = [&](double lat, double lon) { return geo::geodetic_to_world(frame, {lat, lon, ground}); };
    const glm::dvec3 sw = corner(g.south_deg, g.west_deg);
    const glm::dvec3 se = corner(g.south_deg, g.east_deg);
    const glm::dvec3 nw = corner(g.north_deg, g.west_deg);
    const glm::dvec3 ne = corner(g.north_deg, g.east_deg);
    out.extent.east_min = 0.5 * (sw.x + nw.x);
    out.extent.east_max = 0.5 * (se.x + ne.x);
    out.extent.north_min = -0.5 * (sw.z + se.z);
    out.extent.north_max = -0.5 * (nw.z + ne.z);
    if (!(out.extent.east_min < out.extent.east_max && out.extent.north_min < out.extent.north_max)) {
        return invalid("georef.yaml", "the box does not make a map around the anchor");
    }
    return out;
}

}  // namespace spade
