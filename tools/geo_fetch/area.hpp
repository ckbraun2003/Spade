#pragma once

// ---------------------------------------------------------------------------
// geo_fetch/area.hpp -- the box a fetch asks both services for (L85, SP-5; the design note, section 4): the square of `side_m`
// metres centred on the anchor, as WGS84 degrees. It is Core's geodesy (core/geodesy.hpp, SP-1) and nothing of this tool's own, so
// the tool and the engine place the map on one frame: the box's edges are the points `side_m / 2` east, west, north and south of the
// anchor on the anchor's local plane. The reader (world/terrain_map.hpp) turns the box back into metres with the same functions.
// ---------------------------------------------------------------------------

#include "geo_services.hpp"

namespace spade::geofetch {

// The anchor must be a valid one (core/geodesy.hpp validate_anchor); fetch_map checks that before it asks. An area across the 180th
// meridian comes back with west >= east, which fetch_map refuses by name.
[[nodiscard]] Bbox area_bbox(double lat_deg, double lon_deg, double side_m);

}  // namespace spade::geofetch
