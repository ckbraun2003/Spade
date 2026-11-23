#include "area.hpp"

#include <cmath>
#include <string>

#include "core/geodesy.hpp"

namespace spade::geofetch {

namespace {
// The conterminous extent NGS lists on its DEFLEC18 page, used as a proxy for GEOID18's own grid (see area.hpp).
constexpr double kHeightModelMinLatDeg = 24.0;
constexpr double kHeightModelMaxLatDeg = 58.0;
constexpr double kHeightModelMinLonDeg = -130.0;
constexpr double kHeightModelMaxLonDeg = -60.0;
}  // namespace

Bbox area_bbox(double lat_deg, double lon_deg, double side_m) {
    // The anchor at height 0 on its own plane (x east, z south). Each edge is the point half a side out along one axis, converted
    // to latitude or longitude by the geodesy: west and east give the longitudes, north and south the latitudes.
    const geo::LocalFrame frame = geo::make_local_frame({lat_deg, lon_deg, 0.0});
    const double half = side_m / 2.0;
    Bbox box;
    box.west_deg = geo::world_to_geodetic(frame, {-half, 0.0, 0.0}).lon_deg;
    box.east_deg = geo::world_to_geodetic(frame, {half, 0.0, 0.0}).lon_deg;
    box.south_deg = geo::world_to_geodetic(frame, {0.0, 0.0, half}).lat_deg;
    box.north_deg = geo::world_to_geodetic(frame, {0.0, 0.0, -half}).lat_deg;
    return box;
}

std::string height_model_refusal(double lat_deg, double lon_deg) {
    constexpr const char* kNeed = "the height model needs NAVD88 heights (USGS 3DEP) and NOAA NGS's GEOID18, which hold over the conterminous United States";
    if (!std::isfinite(lat_deg) || !std::isfinite(lon_deg)) {
        return std::string("the anchor is not a finite position; ") + kNeed;
    }
    // Puerto Rico and the US Virgin Islands: GEOID18 covers them (NGS) but their elevations are on PRVD02 and VIVD09, not NAVD88.
    if (lat_deg >= 15.0 && lat_deg <= 21.0 && lon_deg >= -69.0 && lon_deg <= -64.0) {
        return std::string("this anchor is in Puerto Rico or the US Virgin Islands, whose heights are on PRVD02 and VIVD09, not NAVD88; ") + kNeed;
    }
    // Hawaii: elevations are on local mean sea level.
    if (lat_deg >= 18.5 && lat_deg <= 22.5 && lon_deg >= -161.0 && lon_deg <= -154.0) {
        return std::string("this anchor is in Hawaii, whose heights are on local mean sea level, not NAVD88; ") + kNeed;
    }
    if (lat_deg >= kHeightModelMinLatDeg && lat_deg <= kHeightModelMaxLatDeg && lon_deg >= kHeightModelMinLonDeg && lon_deg <= kHeightModelMaxLonDeg) {
        return {};
    }
    return std::string("this anchor is outside the conterminous United States (24 to 58 N, 130 to 60 W); ") + kNeed;
}

}  // namespace spade::geofetch
