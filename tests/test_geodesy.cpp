// SP-1 of the geo plan (L85, docs/design/core/2026-11-17-geo-g1-design.md): WGS84 geodesy in double, with no libm (core/geodesy.hpp).
//
// WHAT IS PINNED. (1) The constants against the published ones. (2) The cardinal points the defining constants fix exactly. (3) Two
// PUBLISHED worked examples, each to 1 mm. (4) A round trip over 1 km at six anchors, to 1e-9 degrees. (5) The world frame's axes.
// (6) The sin and cos kernel against std::sin and std::cos, and exact at the multiples of 90. (7) The anchor's validation and identity.
// (8) Exact BIT PATTERNS of both directions at 24 points: the same table runs on MSVC and on the Linux gcc leg, and bit-equality on
// both is the claim (REQ-G06); there is no tolerance there.
//
// THE SOURCES of the published numbers (fetched once, 2026-11-17, read-only into a folder outside the tree; nothing but the numbers
// below is in the repository):
//   IOGP Geomatics Guidance Note 7, part 2, "Coordinate Conversions and Transformations including Formulas", Publication 373-7-2,
//   revised September 2019. https://www.iogp.org/wp-content/uploads/2019/09/373-07-02.pdf  sha256
//   8951d9641e6a65a071e865d1e6bb0611d0d7248772fa3054496fc14041183465 (2,611,009 bytes).
//     - Section 4.1.1 "Geographic/Geocentric conversions", the example (page 95-96): a North Sea point in WGS 84, 53 48' 33.820" N,
//       2 07' 46.380" E, h 73.0 m, geocentric X 3771793.968, Y 140253.342, Z 5124304.349 m, with p = 3774400.712 and nu = 6392088.017;
//       the ellipsoid there: a 6378137.000 m, b 6356752.314 m, e2 0.006694380.
//     - Section 4.1.2 "Geographic/topocentric conversions", the example (page 99): origin X 3652755.3058, Y 319574.6799, Z 5201547.3536
//       (which is 55 N, 5 E, h 200 m); the point above is U -189013.869, V -128642.040, W -4220.171 m.
//   NGA "World Geodetic System 1984" sheet for the ICG, whose references are NIMA TR 8350.2:
//   https://www.unoosa.org/pdf/icg/2012/template/WGS_84.pdf  sha256 ff6bc9776119f7edff86a4011dbbf4b1c279c0a521da2e79fdcd199b412a03d3
//   (32,391 bytes): a 6378137.0 m, 1/f 298.257223563.
// NGA TR 8350.2 itself could not be fetched (nsgreg.nga.mil and earth-info.nga.mil did not answer; DTIC returned 403), so the constants
// come from that sheet and the guidance note; the test says "as published" only for what those two print.
#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "core/detail/math_ops_f64.hpp"
#include "core/geodesy.hpp"

namespace {

namespace geo = spade::geo;

constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] uint64_t bits(double v) { return std::bit_cast<uint64_t>(v); }

[[nodiscard]] uint64_t ulp_distance(double a, double b) {
    auto ordered = [](double v) {
        const int64_t i = std::bit_cast<int64_t>(v);
        return i < 0 ? std::numeric_limits<int64_t>::min() - i : i;
    };
    const int64_t x = ordered(a);
    const int64_t y = ordered(b);
    return x > y ? static_cast<uint64_t>(x) - static_cast<uint64_t>(y) : static_cast<uint64_t>(y) - static_cast<uint64_t>(x);
}

[[nodiscard]] double dms(double d, double m, double s) { return d + m / 60.0 + s / 3600.0; }

[[nodiscard]] bool print_pins_asked() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_GEODESY_PINS") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// The published worked example (IOGP GN 7-2, 4.1.1).
constexpr double kGnX = 3771793.968;
constexpr double kGnY = 140253.342;
constexpr double kGnZ = 5124304.349;

}  // namespace

TEST(Geodesy, TheConstantsAreTheDefiningOnesAndThePublishedDerivedOnes) {
    EXPECT_EQ(geo::kWgs84A, 6378137.0);
    EXPECT_EQ(geo::kWgs84InvF, 298.257223563);
    EXPECT_NEAR(geo::kWgs84B, 6356752.314, 0.001) << "b as the guidance note prints it, to its own rounding";
    EXPECT_NEAR(geo::kWgs84E2, 0.006694380, 5e-10) << "e2 as the guidance note prints it, to its own rounding";
}

TEST(Geodesy, TheCardinalPointsFollowFromTheConstantsAlone) {
    const geo::Ecef east0 = geo::geodetic_to_ecef({0.0, 0.0, 0.0});
    EXPECT_EQ(east0.x, geo::kWgs84A);
    EXPECT_EQ(east0.y, 0.0);
    EXPECT_EQ(east0.z, 0.0);
    const geo::Ecef east90 = geo::geodetic_to_ecef({0.0, 90.0, 0.0});
    EXPECT_EQ(east90.x, 0.0);
    EXPECT_EQ(east90.y, geo::kWgs84A);
    EXPECT_EQ(east90.z, 0.0);
    const geo::Ecef west = geo::geodetic_to_ecef({0.0, 180.0, 0.0});
    EXPECT_EQ(west.x, -geo::kWgs84A);
    EXPECT_EQ(west.y, 0.0);
    const geo::Ecef west90 = geo::geodetic_to_ecef({0.0, -90.0, 0.0});
    EXPECT_EQ(west90.y, -geo::kWgs84A);
    EXPECT_EQ(geo::geodetic_to_ecef({0.0, 0.0, 1000.0}).x, geo::kWgs84A + 1000.0) << "a height is added along the normal";
    const geo::Ecef north = geo::geodetic_to_ecef({90.0, 0.0, 0.0});
    EXPECT_EQ(north.x, 0.0);
    EXPECT_NEAR(north.z, 6356752.314, 0.001);
    EXPECT_NEAR(geo::geodetic_to_ecef({-90.0, 33.0, 0.0}).z, -6356752.314, 0.001);

    // And back: the equator and the pole.
    const geo::Geodetic g0 = geo::ecef_to_geodetic({geo::kWgs84A, 0.0, 0.0});
    EXPECT_NEAR(g0.lat_deg, 0.0, 1e-12);
    EXPECT_NEAR(g0.lon_deg, 0.0, 1e-12);
    EXPECT_NEAR(g0.height_m, 0.0, 1e-6);
    const geo::Geodetic gp = geo::ecef_to_geodetic({0.0, 0.0, geo::kWgs84B + 250.0});
    EXPECT_NEAR(gp.lat_deg, 90.0, 1e-9);
    EXPECT_NEAR(gp.height_m, 250.0, 1e-6);
}

TEST(Geodesy, ThePublishedNorthSeaPointConvertsBothWaysToAMillimetre) {
    const double lat = dms(53.0, 48.0, 33.820);
    const double lon = dms(2.0, 7.0, 46.380);
    const geo::Ecef e = geo::geodetic_to_ecef({lat, lon, 73.0});
    EXPECT_NEAR(e.x, kGnX, 0.001);
    EXPECT_NEAR(e.y, kGnY, 0.001);
    EXPECT_NEAR(e.z, kGnZ, 0.001);

    const geo::Geodetic g = geo::ecef_to_geodetic({kGnX, kGnY, kGnZ});
    EXPECT_NEAR(g.lat_deg, lat, 1e-8) << "0.001 mm of latitude is 1e-8 degrees: the published ECEF is rounded to 1 mm (1e-8 deg)";
    EXPECT_NEAR(g.lon_deg, lon, 1e-8);
    EXPECT_NEAR(g.height_m, 73.0, 0.001);

    // The note's intermediate values: p, and nu at the point's latitude.
    const double p = std::sqrt(kGnX * kGnX + kGnY * kGnY);
    EXPECT_NEAR(p, 3774400.712, 0.001);
    double s = 0.0;
    double c = 0.0;
    spade::math::sincos_deg_arith(g.lat_deg, s, c);
    const double nu = geo::kWgs84A / std::sqrt(1.0 - geo::kWgs84E2 * s * s);
    EXPECT_NEAR(nu, 6392088.017, 0.001);
}

TEST(Geodesy, ThePublishedTopocentricExampleIsTheLocalFrame) {
    // The origin: the note's geocentric origin is 55 N, 5 E, h 200 m.
    const geo::LocalFrame frame = geo::make_local_frame({55.0, 5.0, 200.0});
    EXPECT_NEAR(frame.origin.x, 3652755.3058, 0.001);
    EXPECT_NEAR(frame.origin.y, 319574.6799, 0.001);
    EXPECT_NEAR(frame.origin.z, 5201547.3536, 0.001);
    // The note derives the origin's latitude and longitude from its geocentric values; so does ecef_to_geodetic.
    const geo::Geodetic o = geo::ecef_to_geodetic({3652755.3058, 319574.6799, 5201547.3536});
    EXPECT_NEAR(o.lat_deg, 55.0, 1e-8);
    EXPECT_NEAR(o.lon_deg, 5.0, 1e-8);
    EXPECT_NEAR(o.height_m, 200.0, 0.001);

    const glm::dvec3 enu = geo::ecef_to_enu(frame, {kGnX, kGnY, kGnZ});
    EXPECT_NEAR(enu.x, -189013.869, 0.001) << "U, east";
    EXPECT_NEAR(enu.y, -128642.040, 0.001) << "V, north";
    EXPECT_NEAR(enu.z, -4220.171, 0.001) << "W, up";
    const geo::Ecef back = geo::enu_to_ecef(frame, enu);
    EXPECT_NEAR(back.x, kGnX, 1e-6);
    EXPECT_NEAR(back.y, kGnY, 1e-6);
    EXPECT_NEAR(back.z, kGnZ, 1e-6);
}

TEST(Geodesy, ARoundTripOverAKilometreReturnsWithin1e9DegreesAndAMicrometre) {
    const geo::Geodetic anchors[] = {{0.0, 0.0, 0.0},          {40.76, -113.9, 1286.5}, {60.0, 10.0, 0.0},
                                     {80.0, 170.0, 300.0},     {-40.0, 150.0, 20.0},    {10.0, 179.9995, 5.0}};
    const double offsets[] = {-1000.0, -500.0, -1.0, 0.0, 1.0, 640.0, 1000.0};
    const double ups[] = {-50.0, 0.0, 200.0};
    double worst_deg = 0.0;
    double worst_m = 0.0;
    for (const geo::Geodetic& a : anchors) {
        const geo::LocalFrame frame = geo::make_local_frame(a);
        for (const double x : offsets) {
            for (const double z : offsets) {
                for (const double y : ups) {
                    const glm::dvec3 w(x, y, z);
                    // world -> geodetic -> world
                    const geo::Geodetic g = geo::world_to_geodetic(frame, w);
                    const glm::dvec3 w2 = geo::geodetic_to_world(frame, g);
                    worst_m = std::max(worst_m, std::max({std::abs(w2.x - w.x), std::abs(w2.y - w.y), std::abs(w2.z - w.z)}));
                    // geodetic -> world -> geodetic
                    const glm::dvec3 wg = geo::geodetic_to_world(frame, g);
                    const geo::Geodetic g2 = geo::world_to_geodetic(frame, wg);
                    worst_deg = std::max(worst_deg, std::max(std::abs(g2.lat_deg - g.lat_deg), std::abs(g2.lon_deg - g.lon_deg)));
                }
            }
        }
    }
    EXPECT_LT(worst_deg, 1e-9) << "worst latitude or longitude difference, degrees";
    EXPECT_LT(worst_m, 1e-6) << "worst world difference, metres";
}

TEST(Geodesy, TheWorldAxesAreEastUpSouth) {
    const geo::Geodetic anchor{40.76, -113.9, 1286.5};
    const geo::LocalFrame frame = geo::make_local_frame(anchor);
    const geo::Geodetic at0 = geo::world_to_geodetic(frame, glm::dvec3(0.0));
    EXPECT_NEAR(at0.lat_deg, anchor.lat_deg, 1e-9);
    EXPECT_NEAR(at0.lon_deg, anchor.lon_deg, 1e-9);
    EXPECT_NEAR(at0.height_m, anchor.height_m, 1e-6);
    const geo::Geodetic east = geo::world_to_geodetic(frame, glm::dvec3(100.0, 0.0, 0.0));
    EXPECT_GT(east.lon_deg, anchor.lon_deg) << "+x is east";
    EXPECT_NEAR(east.lat_deg, anchor.lat_deg, 1e-6);
    const geo::Geodetic north = geo::world_to_geodetic(frame, glm::dvec3(0.0, 0.0, -100.0));
    EXPECT_GT(north.lat_deg, anchor.lat_deg) << "-z is north";
    EXPECT_NEAR(north.lon_deg, anchor.lon_deg, 1e-6);
    const geo::Geodetic south = geo::world_to_geodetic(frame, glm::dvec3(0.0, 0.0, 100.0));
    EXPECT_LT(south.lat_deg, anchor.lat_deg) << "+z is south";
    const geo::Geodetic up = geo::world_to_geodetic(frame, glm::dvec3(0.0, 100.0, 0.0));
    EXPECT_NEAR(up.height_m, anchor.height_m + 100.0, 1e-6) << "+y is up";
    // 100 m of arc is 100 / (111 km per degree) of latitude, to a few parts in a thousand.
    EXPECT_NEAR(north.lat_deg - anchor.lat_deg, 100.0 / 111000.0, 2e-6);
}

TEST(Geodesy, SinAndCosInDegreesAreExactAtTheQuadrantsAndWithin2UlpOfTheLibm) {
    double s = 1.0;
    double c = 1.0;
    for (int k = -8; k <= 8; ++k) {
        spade::math::sincos_deg_arith(90.0 * k, s, c);
        const int q = ((k % 4) + 4) % 4;
        const double es = q == 0 ? 0.0 : q == 2 ? 0.0 : q == 1 ? 1.0 : -1.0;
        const double ec = q == 0 ? 1.0 : q == 2 ? -1.0 : 0.0;
        EXPECT_EQ(s, es) << "sin(" << 90 * k << " deg)";
        EXPECT_EQ(c, ec) << "cos(" << 90 * k << " deg)";
    }
    // 100000 seeded angles over the geodesy's whole range, +-360.
    uint64_t state = 0x9E3779B97F4A7C15ull;
    uint64_t worst = 0;
    double worst_at = 0.0;
    double worst_identity = 0.0;
    for (int i = 0; i < 100000; ++i) {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        const double deg = (static_cast<double>(state >> 11) / 9007199254740992.0 * 2.0 - 1.0) * 360.0;
        spade::math::sincos_deg_arith(deg, s, c);
        const double rad = deg * (kPi / 180.0);
        // std::sin of the nearest double to the radians is itself only within an ulp or so of the true value, so compare to 2 ulp
        // plus the radians' own rounding (a relative 1e-16 of the angle).
        const double rs = std::sin(rad);
        const double rc = std::cos(rad);
        const uint64_t d = std::max(ulp_distance(s, rs), ulp_distance(c, rc));
        const double slack = 4.0 * std::abs(rad) * 1.2e-16;
        if (std::abs(s - rs) > slack && std::abs(c - rc) > slack && d > worst) {
            worst = d;
            worst_at = deg;
        }
        if (std::abs(s - rs) > 2.3e-16 * std::abs(rs) + slack || std::abs(c - rc) > 2.3e-16 * std::abs(rc) + slack) {
            ADD_FAILURE() << "sincos_deg_arith(" << deg << ") = (" << s << ", " << c << ") against std (" << rs << ", " << rc << ")";
            break;
        }
        worst_identity = std::max(worst_identity, std::abs(s * s + c * c - 1.0));
    }
    EXPECT_LT(worst_identity, 3e-16);
    std::printf("sincos_deg_arith: worst identity error %.3g, worst ulp distance where it matters %llu (at %.17g deg)\n", worst_identity,
                static_cast<unsigned long long>(worst), worst_at);
    // The domain: a NaN, an infinity or a huge angle gives NaN for both.
    spade::math::sincos_deg_arith(std::numeric_limits<double>::quiet_NaN(), s, c);
    EXPECT_TRUE(std::isnan(s) && std::isnan(c));
    spade::math::sincos_deg_arith(2.0e6, s, c);
    EXPECT_TRUE(std::isnan(s) && std::isnan(c));
}

TEST(Geodesy, AnAnchorIsValidWithinItsLimitsAndRefusedByFieldOutsideThem) {
    EXPECT_TRUE(geo::validate_anchor({40.76, -113.9, 1286.5}).has_value());
    EXPECT_TRUE(geo::validate_anchor({89.0, 180.0, -10000.0}).has_value());
    EXPECT_TRUE(geo::validate_anchor({-89.0, -180.0, 100000.0}).has_value());
    const struct {
        geo::Geodetic g;
        const char* field;
    } bad[] = {{{89.0001, 0.0, 0.0}, "lat_deg"},  {{-89.0001, 0.0, 0.0}, "lat_deg"},
               {{0.0, 180.0001, 0.0}, "lon_deg"}, {{0.0, -180.0001, 0.0}, "lon_deg"},
               {{0.0, 0.0, -10000.001}, "height_m"}, {{0.0, 0.0, 100000.001}, "height_m"},
               {{std::nan(""), 0.0, 0.0}, "lat_deg"}, {{0.0, std::numeric_limits<double>::infinity(), 0.0}, "lon_deg"},
               {{0.0, 0.0, std::nan("")}, "height_m"}};
    for (const auto& row : bad) {
        const spade::Result<void> r = geo::validate_anchor(row.g);
        ASSERT_FALSE(r.has_value()) << row.field;
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << row.field;
        EXPECT_NE(r.error().context.find(row.field), std::string::npos) << r.error().context;
    }
}

TEST(Geodesy, AnAnchorsIdentityIsNeverZeroAndFollowsEveryFieldButNotTheSignOfZero) {
    const geo::Geodetic a{40.76, -113.9, 1286.5};
    EXPECT_NE(geo::anchor_identity(a), 0u);
    EXPECT_EQ(geo::anchor_identity(a), geo::anchor_identity(a));
    EXPECT_NE(geo::anchor_identity(a), geo::anchor_identity({40.7600001, -113.9, 1286.5}));
    EXPECT_NE(geo::anchor_identity(a), geo::anchor_identity({40.76, -113.9000001, 1286.5}));
    EXPECT_NE(geo::anchor_identity(a), geo::anchor_identity({40.76, -113.9, 1286.6}));
    EXPECT_EQ(geo::anchor_identity({0.0, -0.0, 0.0}), geo::anchor_identity({-0.0, 0.0, -0.0}));
    EXPECT_NE(geo::anchor_identity({0.0, 0.0, 0.0}), 0u) << "an anchor at (0, 0, 0) is an anchor, not no anchor";
    const geo::Geodetic z = geo::canonical_anchor({-0.0, -0.0, -0.0});
    EXPECT_EQ(bits(z.lat_deg), bits(0.0));
    EXPECT_EQ(bits(z.lon_deg), bits(0.0));
    EXPECT_EQ(bits(z.height_m), bits(0.0));
}

// ---------------------------------------------------------------------------
// The bit pins. Each row: a geodetic point, the exact bits of geodetic_to_ecef (x, y, z), and the exact bits of ecef_to_geodetic of
// that result (lat, lon, height). The table was written from MSVC's output (SPADE_GEODESY_PINS=1 prints rows in this form) and the
// Linux gcc leg must match every bit. A mismatch is a portability failure, not a tolerance question.
// ---------------------------------------------------------------------------
namespace {

struct Pin {
    geo::Geodetic g;
    uint64_t ecef[3];
    uint64_t back[3];
};

const std::vector<geo::Geodetic>& pin_points() {
    static const std::vector<geo::Geodetic> points = {
        {0.0, 0.0, 0.0},
        {0.0, 90.0, 0.0},
        {0.0, 180.0, 0.0},
        {0.0, -90.0, 0.0},
        {89.0, 0.0, 0.0},
        {-89.0, 0.0, 0.0},
        {89.0, 179.0, 0.0},
        {45.0, 45.0, 0.0},
        {40.76, -113.9, 1286.5},
        {53.80939444444444, 2.12955, 73.0},
        {55.0, 5.0, 200.0},
        {60.0, 10.0, -100.0},
        {80.0, 170.0, 9000.0},
        {-40.0, 150.0, 0.0},
        {-33.8688, 151.2093, 58.0},
        {10.0, 179.9995, 500.0},
        {10.0, -179.9995, 500.0},
        {-0.0001, 0.0001, 0.0},
        {0.5, -0.5, -10000.0},
        {37.7749, -122.4194, 16.0},
        {-77.85, 166.67, 30.0},
        {64.1466, -21.9426, 5.0},
        {1.2921, 36.8219, 1795.0},
        {12.3456789, 123.456789, 12345.6789},
    };
    return points;
}

const std::vector<Pin>& pins();  // the table below

}  // namespace

TEST(Geodesy, TheBitPatternsOfBothDirectionsAreThePinnedOnes) {
    if (print_pins_asked()) {
        for (const geo::Geodetic& g : pin_points()) {
            const geo::Ecef e = geo::geodetic_to_ecef(g);
            const geo::Geodetic b = geo::ecef_to_geodetic(e);
            std::printf("    {{%a, %a, %a},\n     {0x%016llxull, 0x%016llxull, 0x%016llxull},\n     {0x%016llxull, 0x%016llxull, 0x%016llxull}},\n",
                        g.lat_deg, g.lon_deg, g.height_m, static_cast<unsigned long long>(bits(e.x)),
                        static_cast<unsigned long long>(bits(e.y)), static_cast<unsigned long long>(bits(e.z)),
                        static_cast<unsigned long long>(bits(b.lat_deg)), static_cast<unsigned long long>(bits(b.lon_deg)),
                        static_cast<unsigned long long>(bits(b.height_m)));
        }
    }
    const std::vector<Pin>& table = pins();
    ASSERT_EQ(table.size(), pin_points().size()) << "the pin table is missing or out of step with the points";
    for (std::size_t i = 0; i < table.size(); ++i) {
        const Pin& p = table[i];
        ASSERT_EQ(bits(p.g.lat_deg), bits(pin_points()[i].lat_deg)) << "row " << i;
        const geo::Ecef e = geo::geodetic_to_ecef(p.g);
        EXPECT_EQ(bits(e.x), p.ecef[0]) << "row " << i << " x " << e.x;
        EXPECT_EQ(bits(e.y), p.ecef[1]) << "row " << i << " y " << e.y;
        EXPECT_EQ(bits(e.z), p.ecef[2]) << "row " << i << " z " << e.z;
        const geo::Geodetic b = geo::ecef_to_geodetic(e);
        EXPECT_EQ(bits(b.lat_deg), p.back[0]) << "row " << i << " lat " << b.lat_deg;
        EXPECT_EQ(bits(b.lon_deg), p.back[1]) << "row " << i << " lon " << b.lon_deg;
        EXPECT_EQ(bits(b.height_m), p.back[2]) << "row " << i << " height " << b.height_m;
    }
}

namespace {

// RED: the table is filled in the GREEN commit, from MSVC's output (SPADE_GEODESY_PINS=1).
const std::vector<Pin>& pins() {
    static const std::vector<Pin> table = {};
    return table;
}

}  // namespace
