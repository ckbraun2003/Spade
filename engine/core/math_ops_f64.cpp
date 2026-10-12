// core/math_ops_f64.cpp -- see math_ops_f64.hpp for the contract. Every
// expression below is spelt in the order it is evaluated; nothing here calls
// libm but std::sqrt (correctly rounded by IEEE 754).
#include "core/math_ops_f64.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>

namespace spade::math {
namespace {

// ===========================================================================
// atan2_arith
// ===========================================================================

// atan(k/8) for k = 2..8 as hi + lo: hi is the double nearest atan(k/8) and lo
// the double nearest the remainder, computed to 300 bits (mpmath). Hex, so the
// bits are what the reader sees. atan(1/2) and atan(1) match fdlibm's tables.
constexpr double kAtanHi[9] = {
    0.0, 0.0,
    0x1.f5b75f92c80ddp-3,  // atan(2/8)
    0x1.6f61941e4def1p-2,  // atan(3/8)
    0x1.dac670561bb4fp-2,  // atan(4/8)
    0x1.1e00babdefeb4p-1,  // atan(5/8)
    0x1.4978fa3269ee1p-1,  // atan(6/8)
    0x1.700a7c5784634p-1,  // atan(7/8)
    0x1.921fb54442d18p-1,  // atan(8/8) = pi/4
};
constexpr double kAtanLo[9] = {
    0.0, 0.0,
    0x1.8ab6e3cf7afbdp-57,
    -0x1.c63aae6f6e918p-56,
    0x1.a2b7f222f65e2p-56,
    -0x1.928df287a668fp-58,
    0x1.2419a87f2a458p-56,
    -0x1.8c34d25aadef6p-56,
    0x1.1a62633145c07p-55,
};

// pi/2 and pi as hi + lo.
constexpr double kHalfPiHi = 0x1.921fb54442d18p+0;
constexpr double kHalfPiLo = 0x1.1a62633145c07p-54;
constexpr double kPiHi = 0x1.921fb54442d18p+1;
constexpr double kPiLo = 0x1.1a62633145c07p-53;

// Below this t, the series runs on t itself (c = 0): |t| <= 3/16, so the
// first dropped term, t^25/25, is under 1.5e-19 of t.
constexpr double kSeriesEnd = 0.1875;

// Below this t, atan(t) = t to within t^3/3 < 2^-56 t: the quotient alone.
constexpr double kTinyRatio = 0x1p-27;

// Dekker's product needs its operands below 2^996 and their product's low
// part above the subnormals; atan2 is homogeneous, so (num, den) are scaled
// by a power of two, exactly, when they stray.
constexpr double kScaleHigh = 0x1p+990;
constexpr double kScaleLow = 0x1p-900;
constexpr double kScaleDown = 0x1p-600;
constexpr double kScaleUp = 0x1p+600;

// The Taylor coefficients of (atan(u) - u) / u^3 = -1/3 + u^2/5 - u^4/7 + ...,
// as exact rationals: IEEE division is correctly rounded, so a compiler that
// folds them gets the bits one that evaluates them gets (fp32_math's rule).
constexpr double kTaylor[11] = {
    -1.0 / 3.0,  1.0 / 5.0,  -1.0 / 7.0,  1.0 / 9.0,  -1.0 / 11.0, 1.0 / 13.0,
    -1.0 / 15.0, 1.0 / 17.0, -1.0 / 19.0, 1.0 / 21.0, -1.0 / 23.0,
};

[[nodiscard]] bool sign_bit(double v) noexcept {
    return (std::bit_cast<uint64_t>(v) >> 63) != 0;
}

struct Pair {
    double hi = 0.0;
    double lo = 0.0;
};

// Knuth's two-sum: hi = fl(a + b) and hi + lo = a + b exactly.
[[nodiscard]] Pair two_sum(double a, double b) noexcept {
    const double s = a + b;
    const double bb = s - a;
    return {s, (a - (s - bb)) + (b - bb)};
}

// Veltkamp's split: hi carries the top 26 bits of a, lo the rest, exactly.
[[nodiscard]] Pair split(double a) noexcept {
    const double c = 134217729.0 * a;  // 2^27 + 1
    const double hi = c - (c - a);
    return {hi, a - hi};
}

// Dekker's product: hi = fl(a * b) and hi + lo = a * b exactly (for operands
// below 2^996 whose product's low part is normal).
[[nodiscard]] Pair two_product(double a, double b) noexcept {
    const double p = a * b;
    const Pair as = split(a);
    const Pair bs = split(b);
    return {p, (((as.hi * bs.hi - p) + as.hi * bs.lo) + as.lo * bs.hi) + as.lo * bs.lo};
}

// atan(u) - u, for |u| <= 3/16: u * (u^2 * Q(u^2)), Q by Horner from the
// highest coefficient down.
[[nodiscard]] double atan_tail(double u) noexcept {
    const double z = u * u;
    double q = kTaylor[10];
    for (int i = 9; i >= 0; --i) q = kTaylor[i] + z * q;
    return u * (z * q);
}

// atan2 for finite, non-zero ay = |y| and ax = |x|, with x's sign: the angle
// in [0, pi].
[[nodiscard]] double angle_of(double ay, double ax, bool x_negative) noexcept {
    const bool swapped = ay > ax;  // past the diagonal: atan(ay/ax) = pi/2 - atan(ax/ay)
    double num = swapped ? ax : ay;
    double den = swapped ? ay : ax;
    const double t = num / den;  // in (0, 1], or underflowed

    // atan(t) = A + s, A = atan(c) as kAtanHi[k] + kAtanLo[k] (0 for k = 0).
    int k = 0;
    double s = t;
    if (t >= kTinyRatio) {
        if (den > kScaleHigh) {
            num = num * kScaleDown;
            den = den * kScaleDown;
        } else if (den < kScaleLow) {
            num = num * kScaleUp;
            den = den * kScaleUp;
        }
        // The quotient's rounding: num = t den + r exactly, so the true ratio
        // is t + r/den, and atan moves by (r/den) / (1 + t^2) to first order.
        const Pair td = two_product(t, den);
        const double r = (num - td.hi) - td.lo;  // num - td.hi is exact (Sterbenz)
        const double correction = (r / den) / (1.0 + t * t);
        double u = t;
        if (t >= kSeriesEnd) {
            k = static_cast<int>(8.0 * t + 0.5);  // 2..8
            const double c = static_cast<double>(k) * 0.125;
            u = (t - c) / (1.0 + t * c);  // t - c is exact (Sterbenz)
        }
        s = u + (atan_tail(u) + correction);  // one rounding at the end
    }

    // The angle is B + sign * (A + s), with B and the sign set by the octant
    // and the quadrant; B - A or B + A is formed as a hi + lo pair.
    double base_hi = 0.0;
    double base_lo = 0.0;
    double sign = 1.0;
    if (!x_negative && swapped) {  // pi/2 - atan(t)
        base_hi = kHalfPiHi;
        base_lo = kHalfPiLo;
        sign = -1.0;
    } else if (x_negative && !swapped) {  // pi - atan(t)
        base_hi = kPiHi;
        base_lo = kPiLo;
        sign = -1.0;
    } else if (x_negative && swapped) {  // pi/2 + atan(t)
        base_hi = kHalfPiHi;
        base_lo = kHalfPiLo;
    }
    const Pair hi = two_sum(base_hi, sign * kAtanHi[k]);
    const double lo = hi.lo + (base_lo + sign * kAtanLo[k]);
    return hi.hi + (lo + sign * s);
}

// ===========================================================================
// closest_on_triangle (fp32)
// ===========================================================================

[[nodiscard]] glm::vec3 sub(glm::vec3 a, glm::vec3 b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

[[nodiscard]] float dot(glm::vec3 a, glm::vec3 b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] float clamp_to(float v, float lo, float hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

// v clamped, coordinate by coordinate, into the bounds of x, y and z.
[[nodiscard]] glm::vec3 clamp_to_bounds(glm::vec3 v, glm::vec3 x, glm::vec3 y, glm::vec3 z) noexcept {
    const auto lo = [](float a, float b, float c) { return a < b ? (a < c ? a : c) : (b < c ? b : c); };
    const auto hi = [](float a, float b, float c) { return a > b ? (a > c ? a : c) : (b > c ? b : c); };
    return {clamp_to(v.x, lo(x.x, y.x, z.x), hi(x.x, y.x, z.x)), clamp_to(v.y, lo(x.y, y.y, z.y), hi(x.y, y.y, z.y)),
            clamp_to(v.z, lo(x.z, y.z, z.z), hi(x.z, y.z, z.z))};
}

// The closest point of the edge (x, y) to p, projected from the lower-indexed
// end so it depends on the edge alone.
[[nodiscard]] glm::vec3 on_edge(glm::vec3 p, glm::vec3 x, glm::vec3 y, uint32_t ix, uint32_t iy) noexcept {
    const glm::vec3 lo = ix < iy ? x : y;
    const glm::vec3 hi = ix < iy ? y : x;
    const glm::vec3 e = sub(hi, lo);
    float t = dot(sub(p, lo), e) / dot(e, e);
    t = t > 0.0f ? (t < 1.0f ? t : 1.0f) : 0.0f;  // a NaN lands on 0
    const glm::vec3 c{lo.x + e.x * t, lo.y + e.y * t, lo.z + e.z * t};
    return clamp_to_bounds(c, lo, hi, hi);
}

[[nodiscard]] TriangleClosest at(glm::vec3 p, glm::vec3 c, TriangleFeature feature) noexcept {
    const glm::vec3 d = sub(p, c);
    return {c, dot(d, d), feature};
}

// ===========================================================================
// Intersection predicates (double)
// ===========================================================================

[[nodiscard]] glm::dvec3 sub(const glm::dvec3& a, const glm::dvec3& b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

[[nodiscard]] double dot(const glm::dvec3& a, const glm::dvec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] glm::dvec3 cross(const glm::dvec3& a, const glm::dvec3& b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// A point of the plane a triangle spans, in the two coordinates that keep
// the triangle's projection largest (the normal's biggest component dropped).
struct Flat {
    double u = 0.0;
    double v = 0.0;
};

[[nodiscard]] Flat flatten(const glm::dvec3& p, int dropped) noexcept {
    if (dropped == 0) return {p.y, p.z};
    if (dropped == 1) return {p.z, p.x};
    return {p.x, p.y};
}

[[nodiscard]] double orient2d(Flat a, Flat b, Flat c) noexcept {
    return (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
}

// p on the closed segment ab, given that the three are collinear.
[[nodiscard]] bool within(Flat a, Flat b, Flat p) noexcept {
    const bool u = (a.u <= p.u && p.u <= b.u) || (b.u <= p.u && p.u <= a.u);
    const bool v = (a.v <= p.v && p.v <= b.v) || (b.v <= p.v && p.v <= a.v);
    return u && v;
}

[[nodiscard]] bool opposite(double a, double b) noexcept {
    return (a > 0.0 && b < 0.0) || (a < 0.0 && b > 0.0);
}

// Whether the closed segments p1p2 and q1q2 meet (collinear overlap included).
[[nodiscard]] bool segments_meet(Flat p1, Flat p2, Flat q1, Flat q2) noexcept {
    const double d1 = orient2d(q1, q2, p1);
    const double d2 = orient2d(q1, q2, p2);
    const double d3 = orient2d(p1, p2, q1);
    const double d4 = orient2d(p1, p2, q2);
    if (opposite(d1, d2) && opposite(d3, d4)) return true;
    return (d1 == 0.0 && within(q1, q2, p1)) || (d2 == 0.0 && within(q1, q2, p2)) ||
           (d3 == 0.0 && within(p1, p2, q1)) || (d4 == 0.0 && within(p1, p2, q2));
}

// Whether p is in the closed triangle abc (either winding).
[[nodiscard]] bool in_triangle(Flat p, Flat a, Flat b, Flat c) noexcept {
    const double d1 = orient2d(a, b, p);
    const double d2 = orient2d(b, c, p);
    const double d3 = orient2d(c, a, p);
    const bool negative = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    const bool positive = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    return !(negative && positive);
}

// The segment pq lies in the plane of abc.
[[nodiscard]] bool coplanar_segment_meets_triangle(const glm::dvec3& p, const glm::dvec3& q, const glm::dvec3& a,
                                                   const glm::dvec3& b, const glm::dvec3& c) noexcept {
    const glm::dvec3 n = cross(sub(b, a), sub(c, a));
    const double nx = n.x < 0.0 ? -n.x : n.x;
    const double ny = n.y < 0.0 ? -n.y : n.y;
    const double nz = n.z < 0.0 ? -n.z : n.z;
    const int dropped = (nx >= ny && nx >= nz) ? 0 : (ny >= nz ? 1 : 2);
    const Flat fp = flatten(p, dropped);
    const Flat fq = flatten(q, dropped);
    const Flat fa = flatten(a, dropped);
    const Flat fb = flatten(b, dropped);
    const Flat fc = flatten(c, dropped);
    return in_triangle(fp, fa, fb, fc) || in_triangle(fq, fa, fb, fc) || segments_meet(fp, fq, fa, fb) ||
           segments_meet(fp, fq, fb, fc) || segments_meet(fp, fq, fc, fa);
}

}  // namespace

double atan2_arith(double y, double x) noexcept {
    if (y != y || x != x) return x + y;  // a NaN in is a NaN out
    const bool y_negative = sign_bit(y);
    const bool x_negative = sign_bit(x);
    double ay = y_negative ? -y : y;
    double ax = x_negative ? -x : x;
    constexpr double kInf = std::numeric_limits<double>::infinity();
    if (ay == kInf || ax == kInf) {  // the limits: inf against inf is the diagonal
        const bool y_inf = ay == kInf;
        const bool x_inf = ax == kInf;
        ay = y_inf ? 1.0 : 0.0;
        ax = x_inf ? 1.0 : 0.0;
    }
    double angle = 0.0;  // in [0, pi]
    if (ay == 0.0) {
        angle = x_negative ? kPiHi : 0.0;
    } else if (ax == 0.0) {
        angle = kHalfPiHi;
    } else {
        angle = angle_of(ay, ax, x_negative);
    }
    return y_negative ? -angle : angle;
}

double corner_angle(const glm::dvec3& a, const glm::dvec3& b) noexcept {
    const glm::dvec3 c = cross(a, b);
    return atan2_arith(std::sqrt(dot(c, c)), dot(a, b));
}

TriangleClosest closest_on_triangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c, uint32_t ia,
                                    uint32_t ib, uint32_t ic) noexcept {
    // Ericson, Real-Time Collision Detection, 5.1.5, in his order.
    const glm::vec3 ab = sub(b, a);
    const glm::vec3 ac = sub(c, a);
    const glm::vec3 ap = sub(p, a);
    const float d1 = dot(ab, ap);
    const float d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return at(p, a, TriangleFeature::vertex_a);

    const glm::vec3 bp = sub(p, b);
    const float d3 = dot(ab, bp);
    const float d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return at(p, b, TriangleFeature::vertex_b);

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) return at(p, on_edge(p, a, b, ia, ib), TriangleFeature::edge_ab);

    const glm::vec3 cp = sub(p, c);
    const float d5 = dot(ab, cp);
    const float d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return at(p, c, TriangleFeature::vertex_c);

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) return at(p, on_edge(p, c, a, ic, ia), TriangleFeature::edge_ca);

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        return at(p, on_edge(p, b, c, ib, ic), TriangleFeature::edge_bc);
    }

    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    const glm::vec3 q{a.x + ab.x * v + ac.x * w, a.y + ab.y * v + ac.y * w, a.z + ab.z * v + ac.z * w};
    return at(p, clamp_to_bounds(q, a, b, c), TriangleFeature::face);
}

double orient3d(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& d) noexcept {
    return dot(sub(d, a), cross(sub(b, a), sub(c, a)));
}

bool segment_meets_triangle(const glm::dvec3& p, const glm::dvec3& q, const glm::dvec3& a, const glm::dvec3& b,
                            const glm::dvec3& c) noexcept {
    const double sp = orient3d(a, b, c, p);
    const double sq = orient3d(a, b, c, q);
    if ((sp > 0.0 && sq > 0.0) || (sp < 0.0 && sq < 0.0)) return false;  // both strictly on one side
    if (sp == 0.0 && sq == 0.0) return coplanar_segment_meets_triangle(p, q, a, b, c);
    // The line pq crosses the plane (or touches it at an end); the crossing
    // is in the triangle iff the line passes each edge on the same side.
    const double e1 = orient3d(p, q, a, b);
    const double e2 = orient3d(p, q, b, c);
    const double e3 = orient3d(p, q, c, a);
    return (e1 >= 0.0 && e2 >= 0.0 && e3 >= 0.0) || (e1 <= 0.0 && e2 <= 0.0 && e3 <= 0.0);
}

bool triangles_meet(const glm::dvec3 (&s)[3], const glm::dvec3 (&t)[3]) noexcept {
    // Moller's rejection: one triangle strictly on one side of the other's plane.
    const auto apart = [](const glm::dvec3 (&plane)[3], const glm::dvec3 (&other)[3]) {
        const double o0 = orient3d(plane[0], plane[1], plane[2], other[0]);
        const double o1 = orient3d(plane[0], plane[1], plane[2], other[1]);
        const double o2 = orient3d(plane[0], plane[1], plane[2], other[2]);
        return (o0 > 0.0 && o1 > 0.0 && o2 > 0.0) || (o0 < 0.0 && o1 < 0.0 && o2 < 0.0);
    };
    if (apart(s, t) || apart(t, s)) return false;
    for (int k = 0; k < 3; ++k) {
        if (segment_meets_triangle(s[k], s[(k + 1) % 3], t[0], t[1], t[2])) return true;
        if (segment_meets_triangle(t[k], t[(k + 1) % 3], s[0], s[1], s[2])) return true;
    }
    return false;
}

}  // namespace spade::math
