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

// The height model holds only where USGS 3DEP is NAVD88 and NOAA NGS's GEOID18 is defined: the conterminous United States. The tool turns
// NAVD88 heights into WGS84 ellipsoid heights by adding the geoid height N, so a place whose elevation is on another datum (Hawaii, the
// Pacific territories, Alaska: local mean sea level or another vertical datum; Puerto Rico and the US Virgin Islands: PRVD02 and VIVD09,
// which GEOID18 covers but 3DEP does not give in NAVD88) would get a wrong height by a metre or more, silently. This returns the reason
// the anchor is refused, by name, or an empty string where the model holds. The box is 24 to 58 N, 130 to 60 W, the conterminous
// extent NGS lists on its DEFLEC18 page: a PROXY for GEOID18's own grid, whose page the search did not return. It is a box, not a border:
// a point in Canada, Mexico or the Bahamas inside it passes this check, and 3DEP's no-data answer refuses it after a request.
[[nodiscard]] std::string height_model_refusal(double lat_deg, double lon_deg);

}  // namespace spade::geofetch
