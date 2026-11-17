#pragma once

// ---------------------------------------------------------------------------
// world/map_cache.hpp -- the geo map cache's layout: where it lives, how an area is named, and the georef file that says
// what a cached entry holds (L85, geo G2, SP-5; the design note is docs/design/interface/plans/2026-11-17-geo-map-fetch-design.md,
// section 4, agreed with Rendering's SP-4).
//
// The engine never touches the network. The fetch tool (tools/geo_fetch) writes an entry; the engine only READS one, through
// this layout. Nothing here opens a socket, reads an environment variable or writes the cache: the caller hands in the
// environment lookup and the directory.
//
//   <root>/v1/<key>/georef.yaml   written last: its presence means the entry is complete
//   <root>/v1/<key>/height.f32    raw little-endian float32, width * height, row 0 the NORTH edge, column 0 the WEST edge,
//                                 each sample the middle of its cell, metres above the WGS84 ellipsoid
//   <root>/v1/<key>/image.png     the imagery service's 8-bit RGB PNG, same row and column order
//
// The key names the area: the anchor rounded to 1e-6 degrees and the side in whole metres, e.g. n40760000_w113900000_s1000.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <string_view>

#include "core/error.hpp"

namespace spade {

inline constexpr int kMapCacheLayoutVersion = 1;  // the "v1" directory; a layout change makes v2 beside it
inline constexpr double kMapMaxSideM = 1000.0;    // REQ-G07
inline constexpr double kMapMinSideM = 10.0;

// The area's key: "n40760000_w113900000_s1000". Latitude and longitude are rounded to 1e-6 degrees (0.11 m), the side to
// whole metres; the hemisphere letters are n or s, e or w (zero is n and e). Pure arithmetic, so the same on every platform.
[[nodiscard]] std::string map_key(double lat_deg, double lon_deg, double side_m);

// The entry's directory: <root>/v1/<key>.
[[nodiscard]] std::filesystem::path map_entry_dir(const std::filesystem::path& root, std::string_view key);

// What a system looks up in its environment (a name to its value, or nullptr when unset).
using EnvLookup = std::function<const char*(const char* name)>;
enum class MapPlatform { windows, posix };

// The default cache root when the caller names none: SPADE_MAP_CACHE if set; else %LOCALAPPDATA%\Spade\maps (windows), or
// $XDG_CACHE_HOME/spade/maps and then $HOME/.cache/spade/maps (posix). Empty when none of them is set.
[[nodiscard]] std::filesystem::path default_map_cache_root(const EnvLookup& env, MapPlatform platform);

// True when `dir` (or any parent of it, whether or not it exists yet) holds a ".git": the cache must stay out of a repository
// (REQ-G10, never committed).
[[nodiscard]] bool inside_git_work_tree(const std::filesystem::path& dir);

// ---- the georef file -------------------------------------------------------

struct MapRaster {
    std::string file;
    uint32_t width = 0;
    uint32_t height = 0;
    double cell_m = 0.0;  // the grid's cell, or the image's pixel, in metres
};

struct MapSource {
    std::string name;           // "USGS 3DEP", "USGS NAIP", "NOAA NGS"
    std::string url;            // the request as sent
    std::string retrieved_utc;  // "2026-11-17T01:23:45Z"
    std::string acquired;       // the capture date when the service gives one; empty when it does not
};

struct MapGeoref {
    int georef_version = 1;
    std::string key;
    double anchor_lat_deg = 0.0;
    double anchor_lon_deg = 0.0;
    double side_m = 0.0;
    double west_deg = 0.0, south_deg = 0.0, east_deg = 0.0, north_deg = 0.0;  // the area both layers cover
    MapRaster elevation;
    MapRaster image;
    std::string vertical = "wgs84_ellipsoid_m";
    std::string geoid_model;
    double geoid_height_m = 0.0;                  // N at the anchor: ellipsoid height = NAVD88 height + N
    double anchor_ground_ellipsoid_height_m = 0.0;  // the elevation at the anchor, plus N
    std::string height_sha256;                      // of height.f32 as stored
    std::string image_sha256;                       // of image.png as stored
    MapSource elevation_source, image_source, geoid_source;
    std::string datum_note;   // the NAD83(2011) / WGS84 offset, recorded not corrected (L85d Q3)
    std::string fetched_utc;
};

// The georef file's text, in a fixed key order, so equal values give equal bytes.
[[nodiscard]] std::string serialize_georef(const MapGeoref& georef);

// Parses a georef file. A missing or mistyped key, or a version other than 1, is schema_mismatch whose context starts
// "map_invalid: georef.yaml: " and names the key.
[[nodiscard]] Result<MapGeoref> parse_georef(std::string_view text);

}  // namespace spade
