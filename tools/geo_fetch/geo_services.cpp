#include "geo_services.hpp"

namespace spade::geofetch {

// RED: stubs.
std::string elevation_url(const Bbox&, uint32_t, uint32_t) { return {}; }
std::string image_url(const Bbox&, uint32_t, uint32_t) { return {}; }
std::string geoid_url(double, double) { return {}; }
Result<GeoidAnswer> parse_geoid_json(std::string_view) { return GeoidAnswer{}; }

}  // namespace spade::geofetch
