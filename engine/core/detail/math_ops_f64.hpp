#pragma once

// ---------------------------------------------------------------------------
// core/detail/math_ops_f64.hpp -- Spade's own kernels for a triangle mesh (CORE-6;
// the parts and meshes plan, Task B2). INTERNAL: world/mesh.cpp's, and not
// installed (engine/CMakeLists.txt). Beside fp32_math and math_ops because
// Spade's own transcendentals have one home (TD-9).
//
// TWO KINDS OF KERNEL LIVE HERE.
//   * Import time, in double: atan2_arith() and corner_angle() (the corner
//     angles that weight a vertex's pseudonormal), and the triangle and
//     segment intersection predicates (the self-intersection check). They run
//     once per mesh, in make_mesh(), and what they produce is rounded to fp32
//     once and stored; no state path calls them.
//   * Query time, in fp32: closest_on_triangle(), the per-triangle step of
//     mesh_sample(). It is written to be mirrored on the GPU later.
//
// THE RULES THEY FOLLOW (TD-3, TD-12). Only + - * / and sqrt, each
// correctly rounded by IEEE 754, in an order the source pins: every
// expression is spelt out left to right, and this file's .cpp is compiled
// once, in spade_core, under spade_fp_strict (no contraction), so MSVC and
// gcc agree bit for bit. No libm transcendental: std::atan2 differs between
// libms in the last place, and the TD-3 canary scan refuses it under engine/.
// ---------------------------------------------------------------------------

#include <cstdint>

#include <glm/vec3.hpp>

namespace spade::math {

// ---------------------------------------------------------------------------
// atan2 in double, from + - * / and sqrt only.
//
// METHOD. The signs and the octant reduce (y, x) to t = min/max of |y|, |x|,
// in [0, 1]. Below t = 3/16, atan(t) is its Taylor series through t^23/23.
// From 3/16 to 1, t is reduced against the nearest c = k/8 (k = 2..8):
// atan(t) = atan(c) + atan(u), u = (t - c) / (1 + t c), |u| <= 1/16, with
// atan(c) a pinned hi + lo pair of doubles (hex literals, computed to 300
// bits). The quotient's own rounding is corrected to first order (its exact
// remainder, from Dekker's product). The octant and quadrant are put back
// against pi/2 and pi as hi + lo pairs, by Knuth's two-sum.
//
// ACCURACY: within 1 ulp of the correctly rounded atan2 everywhere (measured
// against a 200-bit reference at 300 000 points over the whole double range,
// 1.0% of them 1 ulp off), so within 2 ulp of any faithful libm; the test
// measures it against std::atan2 (tests/test_mesh_field.cpp).
//
// EXACT, and equal to std::atan2 bit for bit, at the special operands:
// atan2(+-0, x > 0 or +0) = +-0; atan2(+-0, x < 0 or -0) = +-pi;
// atan2(y != 0, +-0) = +-pi/2; the infinities as C's Annex F says
// (+-pi/4, +-3pi/4, +-pi/2, +-0, +-pi); a NaN operand returns a NaN. The
// diagonal |y| = |x| gives the doubles nearest pi/4 and 3pi/4.
// ---------------------------------------------------------------------------
[[nodiscard]] double atan2_arith(double y, double x) noexcept;

// The angle between a and b, in [0, pi]: atan2_arith(|a x b|, a . b), the
// cross and dot products spelt left to right. Well conditioned at every
// angle, unlike an acos of the normalised dot product.
[[nodiscard]] double corner_angle(const glm::dvec3& a, const glm::dvec3& b) noexcept;

// ---------------------------------------------------------------------------
// The closest point of a triangle, fp32 (the mesh query's per-triangle step).
// ---------------------------------------------------------------------------

// Which feature of a triangle (a, b, c) holds the closest point.
enum class TriangleFeature : uint32_t {
    face = 0,
    edge_ab,
    edge_bc,
    edge_ca,
    vertex_a,
    vertex_b,
    vertex_c,
};

struct TriangleClosest {
    glm::vec3 point{0.0f};   // the closest point
    float distance2 = 0.0f;  // (p - point) . (p - point), x then y then z
    TriangleFeature feature = TriangleFeature::face;
};

// The closest point of the triangle (a, b, c) to p. ia, ib and ic are the
// vertices' indices in their mesh.
//
// REGIONS: Ericson's (Real-Time Collision Detection, 5.1.5), in his order:
// vertex a, vertex b, edge ab, vertex c, edge ca, edge bc, face. A vertex
// region returns the vertex itself, exactly.
//
// AN EDGE IS PROJECTED FROM ITS LOWER-INDEXED END: t = (p - lo) . e / e . e,
// e = hi - lo, clamped to [0, 1]. The point then depends on the edge alone,
// not on which of its two triangles asks, so a shared edge gives the same
// bits from both (the mesh's tie rule needs that).
//
// THE POINT IS CLAMPED to the feature's bounds (the triangle's, or the
// edge's), coordinate by coordinate. The exact closest point lies inside
// them, so the clamp only removes rounding; and it keeps every returned point
// inside the box of any BVH node that holds the triangle, which is what makes
// the BVH's pruning exact (world/mesh.cpp).
//
// PRECONDITION: a non-degenerate triangle and a finite p.
[[nodiscard]] TriangleClosest closest_on_triangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c,
                                                  uint32_t ia, uint32_t ib, uint32_t ic) noexcept;

// ---------------------------------------------------------------------------
// Intersection predicates, double, for the self-intersection check. Signs of
// orientation determinants only (no division). CLOSED sets: touching counts,
// and so does a zero determinant that the coplanar branch cannot resolve
// (conservative, as the plan says). Inputs are fp32 positions promoted to
// double, so each determinant is far above double's rounding unless the
// points are exactly coplanar, which is what the coplanar branches handle.
// ---------------------------------------------------------------------------

// det[b - a, c - a, d - a]: positive when d is on the side of the plane
// (a, b, c) that (b - a) x (c - a) points to.
[[nodiscard]] double orient3d(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c,
                              const glm::dvec3& d) noexcept;

// Whether the closed segment pq meets the closed triangle abc.
[[nodiscard]] bool segment_meets_triangle(const glm::dvec3& p, const glm::dvec3& q, const glm::dvec3& a,
                                          const glm::dvec3& b, const glm::dvec3& c) noexcept;

// Whether two closed triangles meet: Moller's plane-side rejection, then
// each edge of either against the other (in exact arithmetic, two triangles
// meet iff an edge of one meets the other). For triangles that share no
// vertex; a mesh's adjacent pairs take world/mesh.cpp's own tests.
[[nodiscard]] bool triangles_meet(const glm::dvec3 (&s)[3], const glm::dvec3 (&t)[3]) noexcept;

}  // namespace spade::math
