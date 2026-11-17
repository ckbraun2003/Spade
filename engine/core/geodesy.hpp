#pragma once

#include <cstdint>

#include <glm/vec3.hpp>

#include "core/error.hpp"

// ---------------------------------------------------------------------------
// core/geodesy.hpp -- WGS84 latitude, longitude and height, earth-centred coordinates and the world's local frame, in double
// precision (L85 SP-1; docs/design/core/2026-11-17-geo-g1-design.md). INSTALLED: Kat's editor turns typed coordinates into metres
// with it.
//
// THE FRAME. A world with an anchor has the anchor at world (0, 0, 0), the axes x east, y up, z south (right-handed, Y-up, aligned to
// the compass). A world point is the tangent-plane point at the anchor: east = x, north = -z, up = y. It maps to earth-centred
// coordinates as `origin + east * x + north * (-z) + up * y` and from there to latitude, longitude and height by the exact
// conversion below, so a point on the plane 1 km out reports a height about 8 cm above the ellipsoid. That is the model, not an error.
//
// NO LIBM. Every function is built from + - * /, sqrt and Spade's own kernels (core/detail/math_ops_f64.hpp: atan2_arith and
// sincos_deg_arith), each expression spelt left to right, in spade_core under spade_fp_strict, so MSVC and gcc give the same BITS. sqrt
// is std::sqrt: IEEE 754 mandates a correctly rounded square root, so it is bit-exact everywhere (core/rng.hpp and sensors/gnss.hpp
// use it for the same reason) and the TD-3 scan does not list it. tests/test_geodesy.cpp pins exact bit patterns and runs on every leg.
//
// Angles are in degrees at this interface; heights are metres above the WGS84 ELLIPSOID (not above the geoid or sea level).
// ---------------------------------------------------------------------------

namespace spade::geo {

// WGS84's defining parameters (NGA TR8350.2, as the NGA ICG sheet repeats them): the semi-major axis in metres and the inverse
// flattening. The rest is derived with + - * / only, so it is the same bits everywhere.
inline constexpr double kWgs84A = 6378137.0;
inline constexpr double kWgs84InvF = 298.257223563;
inline constexpr double kWgs84F = 1.0 / kWgs84InvF;
inline constexpr double kWgs84B = kWgs84A * (1.0 - kWgs84F);   // the semi-minor axis, m
inline constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);  // the first eccentricity squared

// The anchor a world may have: refused outside these (east is undefined at a pole and the frame degenerates; the height range
// covers every place on land and the first 100 km of flight).
inline constexpr double kAnchorMaxLatDeg = 89.0;
inline constexpr double kAnchorMaxLonDeg = 180.0;
inline constexpr double kAnchorMinHeightM = -10000.0;
inline constexpr double kAnchorMaxHeightM = 100000.0;

struct Geodetic {
    double lat_deg = 0.0;    // north positive
    double lon_deg = 0.0;    // east positive
    double height_m = 0.0;   // above the WGS84 ellipsoid

    friend bool operator==(const Geodetic&, const Geodetic&) noexcept = default;
};

struct Ecef {
    double x = 0.0;  // metres; through the equator at longitude 0
    double y = 0.0;  // through the equator at longitude 90 east
    double z = 0.0;  // toward the north pole

    friend bool operator==(const Ecef&, const Ecef&) noexcept = default;
};

// The anchor's local frame, built once: its earth-centred point and its east, north and up unit vectors in earth-centred coordinates.
struct LocalFrame {
    Ecef origin{};
    glm::dvec3 east{0.0};
    glm::dvec3 north{0.0};
    glm::dvec3 up{0.0};
};

// The earth-centred point of a geodetic one. Defined for every finite input; the callers validate.
[[nodiscard]] Ecef geodetic_to_ecef(const Geodetic& g) noexcept;

// The geodetic point of an earth-centred one: Bowring's method with a FIXED three iterations (no data-dependent loop, so the bits do
// not depend on a stopping test), the height from the stable form. Valid from the surface outward and a few kilometres below it,
// within 1e-9 degrees and 1e-6 m of the exact answer there; a point near the earth's centre gets a deterministic, meaningless result.
[[nodiscard]] Geodetic ecef_to_geodetic(const Ecef& e) noexcept;

// The frame at `anchor`: its east, north and up vectors from sin and cos of its latitude and longitude.
[[nodiscard]] LocalFrame make_local_frame(const Geodetic& anchor) noexcept;

// East, north, up metres from the frame's origin, and the earth-centred point they name; the two are inverses.
[[nodiscard]] Ecef enu_to_ecef(const LocalFrame& frame, const glm::dvec3& enu) noexcept;
[[nodiscard]] glm::dvec3 ecef_to_enu(const LocalFrame& frame, const Ecef& e) noexcept;

// The world frame (x east, y up, z south) against geodetic coordinates.
[[nodiscard]] Geodetic world_to_geodetic(const LocalFrame& frame, const glm::dvec3& world) noexcept;
[[nodiscard]] glm::dvec3 geodetic_to_world(const LocalFrame& frame, const Geodetic& g) noexcept;

// An anchor is valid when finite and inside the kAnchor* limits. Refused as invalid_argument, naming the field.
[[nodiscard]] Result<void> validate_anchor(const Geodetic& anchor);

// An anchor with -0.0 written as +0.0, so that two anchors of one value are one value to the identity below.
[[nodiscard]] Geodetic canonical_anchor(const Geodetic& anchor) noexcept;

// A 64-bit identity of a valid anchor: FNV-1a over the three doubles' bit patterns (little-endian bytes, canonical zeros) under
// the domain tag "geo.anchor". Never 0, because 0 means "no anchor" wherever an identity is stored.
[[nodiscard]] uint64_t anchor_identity(const Geodetic& anchor) noexcept;

}  // namespace spade::geo
