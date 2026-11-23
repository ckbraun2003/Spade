#include "area.hpp"

#include "core/geodesy.hpp"

namespace spade::geofetch {

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

std::string height_model_refusal(double, double) { return {}; }  // RED: the guard is not written

}  // namespace spade::geofetch
