// lambda2 (core/lambda2.hpp): a fixed-sweep cyclic Jacobi solve.
#include "core/lambda2.hpp"

#include <algorithm>
#include <cmath>

namespace spade {

Symmetric3 lambda2_tensor(const VelocityGradient& g) noexcept {
    // P = G^2, each entry summed in the same order, so the transpose's P is
    // exactly this one's transpose.
    float p[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) p[i][j] = g[i][0] * g[0][j] + g[i][1] * g[1][j] + g[i][2] * g[2][j];
    }
    return {p[0][0], p[1][1], p[2][2], 0.5f * (p[0][1] + p[1][0]), 0.5f * (p[0][2] + p[2][0]),
            0.5f * (p[1][2] + p[2][1])};
}

std::array<float, 3> symmetric_eigenvalues(const Symmetric3& m, int sweeps) noexcept {
    float a[3][3] = {{m.xx, m.xy, m.xz}, {m.xy, m.yy, m.yz}, {m.xz, m.yz, m.zz}};
    // Each sweep's pivots in a fixed order: p, q, and the third index r.
    constexpr int kPivots[3][3] = {{0, 1, 2}, {0, 2, 1}, {1, 2, 0}};
    for (int sweep = 0; sweep < sweeps; ++sweep) {
        for (const auto& pivot : kPivots) {
            const int p = pivot[0], q = pivot[1], r = pivot[2];
            const float apq = a[p][q];
            if (apq == 0.0f) continue;  // already diagonal in this plane
            // The rotation that zeroes a[p][q]. t is the tangent of its angle,
            // the smaller root of t^2 + 2 theta t - 1 = 0 (Numerical Recipes
            // section 11.1).
            const float theta = (a[q][q] - a[p][p]) / (2.0f * apq);
            // The guard compares against a literal constant, not a library
            // limit, so both compilers take the same branch. Past 1e18,
            // theta^2 + 1 nears float's overflow (3.4e38 at |theta| 1.8e19),
            // and t = 1 / (2 theta) to within 1 / (4 theta^2). An infinite
            // theta (apq below the diagonal gap by 2^128) gives t = 0.
            float t = 0.0f;
            if (std::fabs(theta) > 1.0e18f) {
                t = 0.5f / theta;
            } else {
                t = 1.0f / (std::fabs(theta) + std::sqrt(theta * theta + 1.0f));
                if (theta < 0.0f) t = -t;
            }
            const float c = 1.0f / std::sqrt(t * t + 1.0f);
            const float s = t * c;
            const float tau = s / (1.0f + c);
            a[p][p] -= t * apq;
            a[q][q] += t * apq;
            a[p][q] = 0.0f;
            a[q][p] = 0.0f;
            const float arp = a[r][p], arq = a[r][q];
            a[r][p] = arp - s * (arq + tau * arp);
            a[p][r] = a[r][p];
            a[r][q] = arq + s * (arp - tau * arq);
            a[q][r] = a[r][q];
        }
    }
    const float d0 = a[0][0], d1 = a[1][1], d2 = a[2][2];
    const float lo = std::min(std::min(d0, d1), d2);
    const float hi = std::max(std::max(d0, d1), d2);
    const float mid = std::max(std::min(d0, d1), std::min(std::max(d0, d1), d2));
    return {lo, mid, hi};
}

float lambda2(const VelocityGradient& g) noexcept {
    return symmetric_eigenvalues(lambda2_tensor(g), kLambda2Sweeps)[1];
}

float lambda2_hat(const VelocityGradient& g) noexcept {
    const float l = lambda2(g);
    return l < 0.0f ? -std::sqrt(-l) : std::sqrt(l);
}

}  // namespace spade
