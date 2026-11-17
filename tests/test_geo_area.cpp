#include <gtest/gtest.h>

#include <cmath>

#include "../tools/geo_fetch/area.hpp"
#include "core/geodesy.hpp"

// The box a fetch asks for (L85, SP-5; the design note section 4): Core's geodesy places the four edges, and the tool has none of its
// own. The expectations are the WGS84 radii of curvature written out here, which is a different route to the same degrees.

namespace {

using namespace spade;
using namespace spade::geofetch;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegPerRad = 180.0 / kPi;

// Metres to degrees on the ellipsoid at latitude `lat_deg`: along a meridian (radius M) and along a parallel (radius N cos(lat)).
[[nodiscard]] double lat_span_deg(double lat_deg, double metres) {
    const double s = std::sin(lat_deg / kDegPerRad);
    const double m = geo::kWgs84A * (1.0 - geo::kWgs84E2) / std::pow(1.0 - geo::kWgs84E2 * s * s, 1.5);
    return metres / m * kDegPerRad;
}
[[nodiscard]] double lon_span_deg(double lat_deg, double metres) {
    const double s = std::sin(lat_deg / kDegPerRad);
    const double n = geo::kWgs84A / std::sqrt(1.0 - geo::kWgs84E2 * s * s);
    return metres / (n * std::cos(lat_deg / kDegPerRad)) * kDegPerRad;
}

TEST(GeoArea, BonnevilleKilometreMatchesTheRadiiOfCurvatureWithinAMillimetre) {
    const Bbox b = area_bbox(40.76, -113.90, 1000.0);
    constexpr double kMm = 1e-3 / 111000.0;  // a millimetre, in degrees of latitude
    EXPECT_NEAR(b.north_deg, 40.76 + lat_span_deg(40.76, 500.0), 2 * kMm);
    EXPECT_NEAR(b.south_deg, 40.76 - lat_span_deg(40.76, 500.0), 2 * kMm);
    EXPECT_NEAR(b.east_deg, -113.90 + lon_span_deg(40.76, 500.0), 2 * kMm);
    EXPECT_NEAR(b.west_deg, -113.90 - lon_span_deg(40.76, 500.0), 2 * kMm);
    // The sizes a person knows: a kilometre is a little over 0.009 degrees of latitude and, at 40.76 north, about 0.0119 of longitude.
    EXPECT_NEAR(b.north_deg - b.south_deg, 0.00902, 2e-5);
    EXPECT_NEAR(b.east_deg - b.west_deg, 0.01187, 2e-5);
}

TEST(GeoArea, TheBoxCornersRoundTripThroughTheGeodesyToPlusAndMinusHalfTheSide) {
    for (const double side : {10.0, 40.0, 333.0, 1000.0}) {
        for (const double lat : {-60.0, -0.5, 0.0, 40.76, 71.2}) {
            for (const double lon : {-179.9, -113.9, 0.0, 77.7}) {
                const Bbox b = area_bbox(lat, lon, side);
                const geo::LocalFrame frame = geo::make_local_frame({lat, lon, 0.0});
                const glm::dvec3 sw = geo::geodetic_to_world(frame, {b.south_deg, b.west_deg, 0.0});
                const glm::dvec3 ne = geo::geodetic_to_world(frame, {b.north_deg, b.east_deg, 0.0});
                // x east, z south: the box's south-west corner is west (-x) and south (+z) of the anchor. The corners sit on the
                // geodetic grid, not the tangent plane, which differs by the skew of the meridians: 0.15 m at 1 km, so 0.2 m.
                EXPECT_NEAR(sw.x, -side / 2.0, 0.2 * side / 1000.0 + 1e-3) << lat << "," << lon << " " << side;
                EXPECT_NEAR(sw.z, side / 2.0, 0.2 * side / 1000.0 + 1e-3);
                EXPECT_NEAR(ne.x, side / 2.0, 0.2 * side / 1000.0 + 1e-3);
                EXPECT_NEAR(ne.z, -side / 2.0, 0.2 * side / 1000.0 + 1e-3);
            }
        }
    }
}

TEST(GeoArea, TheAnchorIsInsideAndTheBoxIsTheRightWayRound) {
    const Bbox b = area_bbox(-33.9, 151.2, 200.0);
    EXPECT_LT(b.west_deg, 151.2);
    EXPECT_GT(b.east_deg, 151.2);
    EXPECT_LT(b.south_deg, -33.9);
    EXPECT_GT(b.north_deg, -33.9);
}

TEST(GeoArea, TheBoxIsWiderInDegreesOfLongitudeTheFurtherFromTheEquator) {
    const double equator = area_bbox(0.0, 10.0, 1000.0).east_deg - area_bbox(0.0, 10.0, 1000.0).west_deg;
    const double mid = area_bbox(45.0, 10.0, 1000.0).east_deg - area_bbox(45.0, 10.0, 1000.0).west_deg;
    const double high = area_bbox(70.0, 10.0, 1000.0).east_deg - area_bbox(70.0, 10.0, 1000.0).west_deg;
    EXPECT_NEAR(mid / equator, 1.0 / std::cos(45.0 / kDegPerRad), 2e-3);
    EXPECT_GT(high, mid);
}

TEST(GeoArea, AnAreaAcrossTheAntimeridianComesBackWithWestNotBelowEast) {
    // fetch_map refuses it by name (test_geo_fetch.cpp); the box itself must not hide the wrap.
    const Bbox b = area_bbox(52.0, 179.9999, 1000.0);
    EXPECT_GE(b.west_deg, b.east_deg);
}

}  // namespace
