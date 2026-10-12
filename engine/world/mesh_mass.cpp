// world/mesh_mass.cpp -- see mesh_mass.hpp for the contract and the
// conventions.
//
// Eberly's polyhedral mass properties, every expression spelt in its
// evaluation order (C++ evaluates a + b + c as (a + b) + c, and this file
// builds under spade_fp_strict, so nothing is reassociated or fused). Only
// + - * / (TD-3). The ten sums run over the triangles in index order.
#include "world/mesh_mass.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

namespace spade {
namespace {

// Eberly's subexpressions of one coordinate over a triangle's three vertices
// w0, w1, w2: f1, f2 and f3 are the sums of the monomials of degree 1, 2
// and 3; g_k feeds the products of inertia.
struct Sub {
    double f1;
    double f2;
    double f3;
    double g0;
    double g1;
    double g2;
};

[[nodiscard]] Sub subexpressions(double w0, double w1, double w2) noexcept {
    const double temp0 = w0 + w1;
    const double f1 = temp0 + w2;
    const double temp1 = w0 * w0;
    const double temp2 = temp1 + w1 * temp0;
    const double f2 = temp2 + w2 * f1;
    const double f3 = w0 * temp1 + w1 * temp2 + w2 * f2;
    const double g0 = f2 + w0 * (f1 + w0);
    const double g1 = f2 + w1 * (f1 + w1);
    const double g2 = f2 + w2 * (f1 + w2);
    return {f1, f2, f3, g0, g1, g2};
}

}  // namespace

MeshMassProperties mesh_mass_properties(const TriangleMesh& mesh) noexcept {
    const std::span<const glm::vec3> p = mesh.positions();
    const std::span<const uint32_t> idx = mesh.indices();

    // The bounds' centre, in double: every vertex is referenced, so each
    // counts. min and max are exact, and so is (lo + hi) * 0.5 but for a
    // last-place rounding of the sum, which only moves the point the sums
    // are taken about.
    double lo[3] = {p[0].x, p[0].y, p[0].z};
    double hi[3] = {lo[0], lo[1], lo[2]};
    for (const glm::vec3& v : p) {
        const double q[3] = {v.x, v.y, v.z};
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], q[k]);
            hi[k] = std::max(hi[k], q[k]);
        }
    }
    const double cx = (lo[0] + hi[0]) * 0.5;
    const double cy = (lo[1] + hi[1]) * 0.5;
    const double cz = (lo[2] + hi[2]) * 0.5;

    // The integrals, about the centre, times their denominators: 1; x, y,
    // z; x^2, y^2, z^2; xy, yz, zx.
    double s[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const std::size_t triangle_count = idx.size() / 3;
    for (std::size_t t = 0; t < triangle_count; ++t) {
        const glm::vec3& v0 = p[idx[3 * t]];
        const glm::vec3& v1 = p[idx[3 * t + 1]];
        const glm::vec3& v2 = p[idx[3 * t + 2]];
        const double x0 = static_cast<double>(v0.x) - cx;
        const double y0 = static_cast<double>(v0.y) - cy;
        const double z0 = static_cast<double>(v0.z) - cz;
        const double x1 = static_cast<double>(v1.x) - cx;
        const double y1 = static_cast<double>(v1.y) - cy;
        const double z1 = static_cast<double>(v1.z) - cz;
        const double x2 = static_cast<double>(v2.x) - cx;
        const double y2 = static_cast<double>(v2.y) - cy;
        const double z2 = static_cast<double>(v2.z) - cz;

        // The edges, and their cross product (twice the area vector).
        const double a1 = x1 - x0;
        const double b1 = y1 - y0;
        const double c1 = z1 - z0;
        const double a2 = x2 - x0;
        const double b2 = y2 - y0;
        const double c2 = z2 - z0;
        const double d0 = b1 * c2 - b2 * c1;
        const double d1 = a2 * c1 - a1 * c2;
        const double d2 = a1 * b2 - a2 * b1;

        const Sub x = subexpressions(x0, x1, x2);
        const Sub y = subexpressions(y0, y1, y2);
        const Sub z = subexpressions(z0, z1, z2);

        s[0] = s[0] + d0 * x.f1;
        s[1] = s[1] + d0 * x.f2;
        s[2] = s[2] + d1 * y.f2;
        s[3] = s[3] + d2 * z.f2;
        s[4] = s[4] + d0 * x.f3;
        s[5] = s[5] + d1 * y.f3;
        s[6] = s[6] + d2 * z.f3;
        s[7] = s[7] + d0 * (y0 * x.g0 + y1 * x.g1 + y2 * x.g2);
        s[8] = s[8] + d1 * (z0 * y.g0 + z1 * y.g1 + z2 * y.g2);
        s[9] = s[9] + d2 * (x0 * z.g0 + x1 * z.g1 + x2 * z.g2);
    }

    // Divided once each (Eberly multiplies by 1/6, ..., which rounds twice).
    const double volume = s[0] / 6.0;
    const double mx = s[1] / 24.0;
    const double my = s[2] / 24.0;
    const double mz = s[3] / 24.0;
    const double xx = s[4] / 60.0;
    const double yy = s[5] / 60.0;
    const double zz = s[6] / 60.0;
    const double xy = s[7] / 120.0;
    const double yz = s[8] / 120.0;
    const double zx = s[9] / 120.0;

    // The centroid from the centre, then the parallel-axis rule: the second
    // moments about the centroid are those about the centre less V d d^T.
    // A product is written V dx dy - xy, not -(xy - V dx dy), so a symmetric
    // solid's is +0.
    const double dx = mx / volume;
    const double dy = my / volume;
    const double dz = mz / volume;
    const double ixx = (yy + zz) - volume * (dy * dy + dz * dz);
    const double iyy = (zz + xx) - volume * (dz * dz + dx * dx);
    const double izz = (xx + yy) - volume * (dx * dx + dy * dy);
    const double ixy = volume * dx * dy - xy;
    const double iyz = volume * dy * dz - yz;
    const double izx = volume * dz * dx - zx;

    MeshMassProperties out;
    out.volume = volume;
    out.centroid = glm::dvec3(cx + dx, cy + dy, cz + dz);
    out.inertia[0][0] = ixx;
    out.inertia[1][1] = iyy;
    out.inertia[2][2] = izz;
    out.inertia[0][1] = ixy;
    out.inertia[1][0] = ixy;
    out.inertia[1][2] = iyz;
    out.inertia[2][1] = iyz;
    out.inertia[0][2] = izx;
    out.inertia[2][0] = izx;
    return out;
}

}  // namespace spade
