#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

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
    // The sizes a person knows: a kilometre is a little over 0.009 degrees of latitude and, at 40.76 north, about 0.01184 of longitude.
    EXPECT_NEAR(b.north_deg - b.south_deg, 0.00902, 2e-5);
    EXPECT_NEAR(b.east_deg - b.west_deg, 0.01184, 2e-5);
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
    // On the ellipsoid, not a sphere: the parallel's radius is N cos(lat), and N is 0.17 % larger at 45 degrees than at the equator.
    EXPECT_NEAR(mid / equator, lon_span_deg(45.0, 1000.0) / lon_span_deg(0.0, 1000.0), 1e-6);
    EXPECT_GT(high, mid);
}

TEST(GeoArea, AnAreaAcrossTheAntimeridianComesBackWithWestNotBelowEast) {
    // fetch_map refuses it by name (test_geo_fetch.cpp); the box itself must not hide the wrap.
    const Bbox b = area_bbox(52.0, 179.9999, 1000.0);
    EXPECT_GE(b.west_deg, b.east_deg);
}

// The height model guard (O167: a place where the model does not hold is refused by name, not captioned). Fixtures: places on the other
// datums, and the box's edges.
TEST(GeoArea, TheHeightModelHoldsInTheConterminousBoxAndThePointsInsideItPass) {
    EXPECT_EQ(height_model_refusal(40.76, -113.90), "") << "Bonneville";
    EXPECT_EQ(height_model_refusal(24.1, -100.0), "");
    EXPECT_EQ(height_model_refusal(57.9, -129.9), "");
    EXPECT_EQ(height_model_refusal(30.0, -60.1), "");
}

TEST(GeoArea, APlaceOnAnotherVerticalDatumIsRefusedByNameWithItsReason) {
    struct Case {
        const char* what;
        double lat, lon;
        const char* word;  // the refusal must say this
    };
    const Case cases[] = {
        {"Honolulu", 21.31, -157.86, "Hawaii"},
        {"Guam", 13.44, 144.79, "outside"},
        {"Pago Pago", -14.28, -170.70, "outside"},
        {"San Juan", 18.47, -66.11, "PRVD02"},
        {"St Thomas", 18.34, -64.93, "VIVD09"},
        {"Anchorage", 61.22, -149.90, "outside"},
    };
    for (const Case& c : cases) {
        const std::string why = height_model_refusal(c.lat, c.lon);
        EXPECT_FALSE(why.empty()) << c.what;
        EXPECT_NE(why.find(c.word), std::string::npos) << c.what << ": " << why;
        EXPECT_NE(why.find("NAVD88"), std::string::npos) << c.what << " names the datum the model needs: " << why;
    }
}

TEST(GeoArea, JustOutsideEachEdgeOfTheBoxIsRefused) {
    EXPECT_FALSE(height_model_refusal(23.9, -100.0).empty()) << "south";
    EXPECT_FALSE(height_model_refusal(58.1, -100.0).empty()) << "north";
    EXPECT_FALSE(height_model_refusal(40.0, -130.1).empty()) << "west";
    EXPECT_FALSE(height_model_refusal(40.0, -59.9).empty()) << "east";
    EXPECT_FALSE(height_model_refusal(std::numeric_limits<double>::quiet_NaN(), -100.0).empty()) << "NaN";
}

}  // namespace
