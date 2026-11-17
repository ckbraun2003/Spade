#pragma once

// ---------------------------------------------------------------------------
// geo_fetch/geo_services.hpp -- the three services the map fetch asks, and nothing else (L85, SP-5; REQ-G08, REQ-G09): USGS 3DEP
// (elevation), USGS NAIP (imagery) and NOAA NGS (geoid height). The request URLs are written from the services' documentation
// and have NOT been run: every parameter is UNVERIFIED until the one live check (the design note, section 7). They are pure
// string functions, so a test pins each one and a guard test pins the host list.
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

#include "core/error.hpp"

namespace spade::geofetch {

// The area both layers cover, WGS84 degrees (EPSG:4326, regular in longitude and latitude).
struct Bbox {
    double west_deg = 0.0;
    double south_deg = 0.0;
    double east_deg = 0.0;
    double north_deg = 0.0;
};

// The only hosts this tool may name (REQ-G08). A guard test fails on any other host string in tools/geo_fetch/.
inline constexpr std::array<std::string_view, 3> kServiceHosts = {"elevation.nationalmap.gov", "imagery.nationalmap.gov",
                                                                  "geodesy.noaa.gov"};

[[nodiscard]] std::string elevation_url(const Bbox& area, uint32_t cols, uint32_t rows);  // USGS 3DEP, a float32 TIFF
[[nodiscard]] std::string image_url(const Bbox& area, uint32_t cols, uint32_t rows);      // USGS NAIP, an RGB PNG
[[nodiscard]] std::string geoid_url(double lat_deg, double lon_deg);                      // NOAA NGS, JSON

struct GeoidAnswer {
    double height_m = 0.0;  // N: ellipsoid height = NAVD88 height + N
    std::string model;      // "GEOID18"
};

// Reads the NGS answer: a JSON object with a number "geoidHeight" and, when present, a string "geoidModel" (which must name
// GEOID18). Anything else is a schema_mismatch whose context says what was missing.
[[nodiscard]] Result<GeoidAnswer> parse_geoid_json(std::string_view body);

}  // namespace spade::geofetch
