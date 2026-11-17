#include "core/geodesy.hpp"

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "core/detail/math_ops_f64.hpp"

// ---------------------------------------------------------------------------
// core/geodesy.cpp (L85 SP-1). Built once, in spade_core, under spade_fp_strict (no contraction): every expression below is spelt in
// the order it is evaluated, with + - * /, std::sqrt (IEEE 754 mandates its correct rounding, so it is bit-exact everywhere) and
// Spade's own sincos_deg_arith and atan2_arith. No libm transcendental.
// ---------------------------------------------------------------------------

namespace spade::geo {
namespace {

// 180 / pi as hi + lo (hex, computed to 300 bits): radians to degrees without a rounding of the constant.
constexpr double kRadToDegHi = 0x1.ca5dc1a63c1f8p+5;
constexpr double kRadToDegLo = -0x1.1e7ab456405f9p-49;

constexpr double kOneMinusF = 1.0 - kWgs84F;
constexpr double kEp2 = kWgs84E2 / (1.0 - kWgs84E2);  // the second eccentricity squared
constexpr double kEp2B = kEp2 * kWgs84B;
constexpr double kE2A = kWgs84E2 * kWgs84A;
constexpr double kOneMinusE2 = 1.0 - kWgs84E2;

[[nodiscard]] double to_degrees(double radians) noexcept { return radians * kRadToDegHi + radians * kRadToDegLo; }

}  // namespace

Ecef geodetic_to_ecef(const Geodetic& g) noexcept {
    double sin_lat = 0.0;
    double cos_lat = 0.0;
    double sin_lon = 0.0;
    double cos_lon = 0.0;
    math::sincos_deg_arith(g.lat_deg, sin_lat, cos_lat);
    math::sincos_deg_arith(g.lon_deg, sin_lon, cos_lon);
    // nu, the prime vertical radius of curvature.
    const double nu = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    Ecef e;
    e.x = (nu + g.height_m) * cos_lat * cos_lon;
    e.y = (nu + g.height_m) * cos_lat * sin_lon;
    e.z = (nu * kOneMinusE2 + g.height_m) * sin_lat;
    return e;
}

Geodetic ecef_to_geodetic(const Ecef& e) noexcept {
    const double p = std::sqrt(e.x * e.x + e.y * e.y);
    const double za = e.z * kWgs84A;
    const double pb = p * kWgs84B;
    const double hyp = std::sqrt(za * za + pb * pb);
    if (!(hyp > 0.0)) return Geodetic{0.0, 0.0, -kWgs84A};  // the earth's centre: no direction

    // Bowring: the parametric latitude's sine and cosine from (z a, p b), then phi = atan2(z + e'^2 b sin^3, p - e^2 a cos^3), and
    // the parametric latitude again from phi: tan = (1 - f) tan(phi). A fixed three rounds, so no stopping test chooses the bits.
    double sin_t = za / hyp;
    double cos_t = pb / hyp;
    double num = 0.0;
    double den = 0.0;
    double sin_phi = 0.0;
    double cos_phi = 1.0;
    for (int round = 0; round < 3; ++round) {
        num = e.z + kEp2B * sin_t * sin_t * sin_t;
        den = p - kE2A * cos_t * cos_t * cos_t;
        const double norm = std::sqrt(num * num + den * den);
        sin_phi = num / norm;
        cos_phi = den / norm;
        if (round < 2) {
            const double ys = kOneMinusF * sin_phi;
            const double h2 = std::sqrt(ys * ys + cos_phi * cos_phi);
            sin_t = ys / h2;
            cos_t = cos_phi / h2;
        }
    }
    Geodetic g;
    g.lat_deg = to_degrees(math::atan2_arith(num, den));
    g.lon_deg = to_degrees(math::atan2_arith(e.y, e.x));
    // The stable height: p cos(phi) + z sin(phi) - a sqrt(1 - e^2 sin^2(phi)), good at every latitude.
    g.height_m = p * cos_phi + e.z * sin_phi - kWgs84A * std::sqrt(1.0 - kWgs84E2 * sin_phi * sin_phi);
    return g;
}

LocalFrame make_local_frame(const Geodetic& anchor) noexcept {
    double sin_lat = 0.0;
    double cos_lat = 0.0;
    double sin_lon = 0.0;
    double cos_lon = 0.0;
    math::sincos_deg_arith(anchor.lat_deg, sin_lat, cos_lat);
    math::sincos_deg_arith(anchor.lon_deg, sin_lon, cos_lon);
    LocalFrame f;
    f.origin = geodetic_to_ecef(anchor);
    f.east = glm::dvec3(-sin_lon, cos_lon, 0.0);
    f.north = glm::dvec3(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    f.up = glm::dvec3(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
    return f;
}

Ecef enu_to_ecef(const LocalFrame& frame, const glm::dvec3& enu) noexcept {
    Ecef e;
    e.x = ((frame.origin.x + frame.east.x * enu.x) + frame.north.x * enu.y) + frame.up.x * enu.z;
    e.y = ((frame.origin.y + frame.east.y * enu.x) + frame.north.y * enu.y) + frame.up.y * enu.z;
    e.z = ((frame.origin.z + frame.east.z * enu.x) + frame.north.z * enu.y) + frame.up.z * enu.z;
    return e;
}

glm::dvec3 ecef_to_enu(const LocalFrame& frame, const Ecef& e) noexcept {
    const double dx = e.x - frame.origin.x;
    const double dy = e.y - frame.origin.y;
    const double dz = e.z - frame.origin.z;
    return glm::dvec3((frame.east.x * dx + frame.east.y * dy) + frame.east.z * dz,
                      (frame.north.x * dx + frame.north.y * dy) + frame.north.z * dz,
                      (frame.up.x * dx + frame.up.y * dy) + frame.up.z * dz);
}

Geodetic world_to_geodetic(const LocalFrame& frame, const glm::dvec3& world) noexcept {
    // x east, y up, z south: east = x, north = -z, up = y.
    return ecef_to_geodetic(enu_to_ecef(frame, glm::dvec3(world.x, -world.z, world.y)));
}

glm::dvec3 geodetic_to_world(const LocalFrame& frame, const Geodetic& g) noexcept {
    const glm::dvec3 enu = ecef_to_enu(frame, geodetic_to_ecef(g));
    return glm::dvec3(enu.x, enu.z, -enu.y);
}

Result<void> validate_anchor(const Geodetic& anchor) {
    const auto refuse = [](const std::string& text) -> Result<void> {
        return std::unexpected(Error{Code::invalid_argument, "anchor: " + text});
    };
    if (!(std::isfinite(anchor.lat_deg) && anchor.lat_deg >= -kAnchorMaxLatDeg && anchor.lat_deg <= kAnchorMaxLatDeg)) {
        return refuse("lat_deg must be finite and within [-89, 89] degrees (east is undefined at a pole)");
    }
    if (!(std::isfinite(anchor.lon_deg) && anchor.lon_deg >= -kAnchorMaxLonDeg && anchor.lon_deg <= kAnchorMaxLonDeg)) {
        return refuse("lon_deg must be finite and within [-180, 180] degrees");
    }
    if (!(std::isfinite(anchor.height_m) && anchor.height_m >= kAnchorMinHeightM && anchor.height_m <= kAnchorMaxHeightM)) {
        return refuse("height_m must be finite and within [-10000, 100000] metres above the WGS84 ellipsoid");
    }
    return {};
}

Geodetic canonical_anchor(const Geodetic& anchor) noexcept {
    const auto zero = [](double v) { return v == 0.0 ? 0.0 : v; };  // -0.0 == 0.0, so this writes +0.0
    return Geodetic{zero(anchor.lat_deg), zero(anchor.lon_deg), zero(anchor.height_m)};
}

uint64_t anchor_identity(const Geodetic& anchor) noexcept {
    constexpr uint64_t kOffset = 0xcbf29ce484222325ull;
    constexpr uint64_t kPrime = 0x100000001b3ull;
    uint64_t h = kOffset;
    const auto byte = [&h](uint8_t b) { h = (h ^ b) * kPrime; };
    for (const char c : std::string_view("geo.anchor")) byte(static_cast<uint8_t>(c));
    const Geodetic g = canonical_anchor(anchor);
    for (const double v : {g.lat_deg, g.lon_deg, g.height_m}) {
        const uint64_t bits = std::bit_cast<uint64_t>(v);
        for (int i = 0; i < 8; ++i) byte(static_cast<uint8_t>(bits >> (8 * i)));  // little-endian, whatever the host
    }
    return h == 0 ? 1 : h;
}

}  // namespace spade::geo
