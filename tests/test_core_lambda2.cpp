// lambda2 (core/lambda2.hpp): the exact cases, Lamb-Oseen's closed form,
// a double-precision solve, the sweep count's justification, and the same
// bits on every compiler.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "core/fp16.hpp"
#include "core/lambda2.hpp"

namespace {

using spade::lambda2;
using spade::lambda2_hat;
using spade::lambda2_tensor;
using spade::Symmetric3;
using spade::symmetric_eigenvalues;
using spade::VelocityGradient;

constexpr double kPi = 3.14159265358979323846;

// The bands, each against the double-precision answer and over the scale
// its test names, measured 2026-10-12 on MSVC (each test prints its worst)
// and pinned with headroom. gcc must give the same bits (the digest).
constexpr double kPinnedLambOseen = 1e-7;      // measured 4.97e-8
constexpr double kPinnedRandom = 2e-7;         // measured 9.0e-8
constexpr double kPinnedSweeps = 3e-7;         // measured 1.94e-7 from 3 sweeps on
constexpr double kPinnedSweepsMiddle = 3e-7;   // lambda2's own eigenvalue: the same
constexpr uint64_t kPinnedDigest = 0x8F870E7D8C0F6973ull;

// A deterministic generator, the same on every compiler (std's
// distributions are not).
struct XorShift {
    uint64_t state = 0x9E3779B97F4A7C15ull;
    uint64_t next() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    }
    // Uniform in [-1, 1), on a 2^-23 grid, so exact in float.
    double signed_unit() { return static_cast<double>(static_cast<int64_t>(next() >> 40) - (1ll << 23)) / 8388608.0; }
};

using Mat3 = std::array<std::array<double, 3>, 3>;

// The eigenvalues of `m`, ascending, in double by the trigonometric closed
// form (Smith 1961): independent of the Jacobi solve under test.
std::array<double, 3> reference_eigenvalues(const Mat3& m) {
    const double p1 = m[0][1] * m[0][1] + m[0][2] * m[0][2] + m[1][2] * m[1][2];
    const double q = (m[0][0] + m[1][1] + m[2][2]) / 3.0;
    const double p2 =
        (m[0][0] - q) * (m[0][0] - q) + (m[1][1] - q) * (m[1][1] - q) + (m[2][2] - q) * (m[2][2] - q) + 2.0 * p1;
    if (p2 == 0.0) return {q, q, q};
    const double p = std::sqrt(p2 / 6.0);
    const double b00 = (m[0][0] - q) / p, b11 = (m[1][1] - q) / p, b22 = (m[2][2] - q) / p;
    const double b01 = m[0][1] / p, b02 = m[0][2] / p, b12 = m[1][2] / p;
    const double det = b00 * (b11 * b22 - b12 * b12) - b01 * (b01 * b22 - b12 * b02) + b02 * (b01 * b12 - b11 * b02);
    const double phi = std::acos(std::clamp(det / 2.0, -1.0, 1.0)) / 3.0;
    const double hi = q + 2.0 * p * std::cos(phi);
    const double lo = q + 2.0 * p * std::cos(phi + 2.0 * kPi / 3.0);
    return {lo, 3.0 * q - hi - lo, hi};
}

Mat3 widen(const Symmetric3& m) {
    return {{{m.xx, m.xy, m.xz}, {m.xy, m.yy, m.yz}, {m.xz, m.yz, m.zz}}};
}

// (G^2 + (G^2)^T) / 2 in double.
Mat3 tensor_of(const Mat3& g) {
    Mat3 m{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double pij = 0.0, pji = 0.0;
            for (int k = 0; k < 3; ++k) {
                pij += g[i][k] * g[k][j];
                pji += g[j][k] * g[k][i];
            }
            m[i][j] = 0.5 * (pij + pji);
        }
    }
    return m;
}

VelocityGradient narrow(const Mat3& g) {
    VelocityGradient f{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) f[i][j] = static_cast<float>(g[i][j]);
    }
    return f;
}

Mat3 widen(const VelocityGradient& g) {
    Mat3 d{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) d[i][j] = g[i][j];
    }
    return d;
}

double frobenius_squared(const Mat3& g) {
    double s = 0.0;
    for (const auto& row : g) {
        for (const double v : row) s += v * v;
    }
    return s;
}

// A rotation from a unit quaternion, and R A R^T.
Mat3 rotation(double w, double x, double y, double z) {
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    w /= n;
    x /= n;
    y /= n;
    z /= n;
    return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
             {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
             {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}

Mat3 rotate(const Mat3& r, const Mat3& a) {
    Mat3 ra{}, out{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) ra[i][j] += r[i][k] * a[k][j];
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) out[i][j] += ra[i][k] * r[j][k];
        }
    }
    return out;
}

// Solid-body rotation at W about z (u = -W y, v = W x): S = 0, and
// Omega^2 = diag(-W^2, -W^2, 0), so lambda2 = -W^2 exactly.
TEST(CoreLambda2, SolidBodyRotationIsMinusItsRateSquared) {
    const VelocityGradient g = {{{0.0f, -3.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};
    EXPECT_EQ(lambda2(g), -9.0f);
    EXPECT_EQ(lambda2_hat(g), -3.0f);
}

// Pure strain (u = a x, v = -a y): S^2 = diag(a^2, a^2, 0), Omega = 0, so
// lambda2 = a^2 > 0: no vortex.
TEST(CoreLambda2, PureStrainIsPositive) {
    const VelocityGradient g = {{{2.0f, 0.0f, 0.0f}, {0.0f, -2.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};
    EXPECT_EQ(lambda2(g), 4.0f);
    EXPECT_EQ(lambda2_hat(g), 2.0f);
}

// A drone wash's vortex core: a rotation of 1e4 1/s, as millimetre cells see
// it. lambda2 = -1e8 1/s^2 would be no-data in a half; the stored root, -1e4
// 1/s, is in range and keeps its sign.
TEST(CoreLambda2, ADroneWashsVortexCoreStaysInAHalf) {
    const VelocityGradient g = {{{0.0f, -1.0e4f, 0.0f}, {1.0e4f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};
    EXPECT_TRUE(spade::fp16::from_f32(lambda2(g)).no_data) << "why the channel stores the root";
    const spade::fp16::Half stored = spade::fp16::from_f32(lambda2_hat(g));
    ASSERT_FALSE(stored.no_data);
    EXPECT_EQ(spade::fp16::to_f32(stored.bits), -1.0e4f);  // 10000 is a half exactly
}

// Pure shear (u = 5 y) has vorticity 5 but no vortex: S^2 + Omega^2 = 0.
// This is lambda2's point against vorticity magnitude.
TEST(CoreLambda2, PureShearIsNotAVortex) {
    const VelocityGradient g = {{{0.0f, 5.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};
    EXPECT_EQ(lambda2(g), 0.0f);
}

// The other gradient convention, d u_a / d x_c, gives the same bits.
TEST(CoreLambda2, TheTransposedGradientGivesTheSameLambda2) {
    XorShift rng;
    for (int n = 0; n < 1000; ++n) {
        VelocityGradient g{}, gt{};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) g[i][j] = static_cast<float>(rng.signed_unit());
        }
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) gt[i][j] = g[j][i];
        }
        ASSERT_EQ(std::bit_cast<uint32_t>(lambda2(g)), std::bit_cast<uint32_t>(lambda2(gt))) << "case " << n;
    }
}

// A Lamb-Oseen vortex, its axis oblique to the grid: u_theta(r) =
// (Gamma / 2 pi r)(1 - exp(-r^2 / r_c^2)), with Gamma = 2 pi and r_c = 1. In
// any plane flow lambda2 = -det(G) = -u_theta u_theta' / r. That is negative
// inside the radius of peak swirl r*, where 2 eta exp(-eta) = 1 - exp(-eta)
// for eta = r*^2 (r* = 1.1209), and positive outside, where the flow is
// nearly irrotational.
TEST(CoreLambda2, LambOseenMatchesItsClosedForm) {
    const Mat3 r = rotation(0.9, 0.3, -0.2, 0.25);
    double lo = 1.0, hi = 1.5;  // eta* by bisection
    for (int i = 0; i < 60; ++i) {
        const double eta = 0.5 * (lo + hi);
        if (2.0 * eta * std::exp(-eta) > -std::expm1(-eta)) {
            lo = eta;
        } else {
            hi = eta;
        }
    }
    const double r_star = std::sqrt(lo);
    EXPECT_NEAR(r_star, 1.1209, 1e-4);

    double worst = 0.0;
    for (int i = 1; i <= 60; ++i) {
        const double rad = 0.05 * i;
        const double phi = 0.7 + 0.37 * i;  // around the axis, varied
        const double x = rad * std::cos(phi), y = rad * std::sin(phi);
        const double eta = rad * rad;
        const double one_minus = -std::expm1(-eta);
        const double u_theta = one_minus / rad;
        const double du_theta = (2.0 * eta * std::exp(-eta) - one_minus) / (rad * rad);
        const double want = -u_theta * du_theta / rad;
        const double f = u_theta / rad;               // angular velocity
        const double df = (du_theta - f) / rad;       // its radial derivative
        const Mat3 local = {{{-df * x * y / rad, -f - df * y * y / rad, 0.0},
                             {f + df * x * x / rad, df * x * y / rad, 0.0},
                             {0.0, 0.0, 0.0}}};
        const Mat3 world = rotate(r, local);
        const float got = lambda2(narrow(world));
        const double err = std::abs(got - want) / frobenius_squared(world);
        worst = std::max(worst, err);
        if (rad < r_star - 0.03) {
            EXPECT_LT(got, 0.0f) << "r = " << rad << " lies inside the core";
        }
        if (rad > r_star + 0.03) {
            EXPECT_GT(got, 0.0f) << "r = " << rad << " lies outside the core";
        }
    }
    std::printf("[ lambda2 ] Lamb-Oseen: worst |error| / |G|^2 = %.3g\n", worst);
    EXPECT_LE(worst, kPinnedLambOseen);
}

// Against the double-precision solve, over general gradients (not trace
// free) at three scales.
TEST(CoreLambda2, MatchesADoublePrecisionSolveOnRandomGradients) {
    XorShift rng;
    double worst = 0.0;
    for (const double scale : {1.0 / 1024.0, 1.0, 1024.0}) {
        for (int n = 0; n < 4096; ++n) {
            Mat3 g{};
            for (auto& row : g) {
                for (double& v : row) v = scale * rng.signed_unit();
            }
            const VelocityGradient gf = narrow(g);
            const Mat3 exact = widen(gf);  // the float inputs, exactly
            const double want = reference_eigenvalues(tensor_of(exact))[1];
            const double err = std::abs(lambda2(gf) - want) / frobenius_squared(exact);
            worst = std::max(worst, err);
        }
    }
    std::printf("[ lambda2 ] random gradients: worst |error| / |G|^2 = %.3g\n", worst);
    EXPECT_LE(worst, kPinnedRandom);
}

// The sweep count. The hardest matrices for a fixed-sweep Jacobi solve that
// I could construct: two or three eigenvalues a relative delta apart, delta
// from 1e-1 down to 0, under 256 rotations each, at unit and at mixed
// scales. Each is rounded to float and then solved with 1 to 8 sweeps;
// the error is the worst of the three eigenvalues against the double solve
// of the same float matrix, over the largest eigenvalue's magnitude.
// Measured 2026-10-12: 1 sweep 6.8e-2, 2 sweeps 3.7e-4, and 3 to 8 sweeps
// 1.94e-7, float's own rounding (1.6 epsilon). kLambda2Sweeps = 6 is twice
// what this needs, and costs about 60 flops a sweep.
TEST(CoreLambda2, SixSweepsConvergeOnNearDegenerateEigenvalues) {
    struct Pattern {  // eigenvalue k is base[k] + step[k] * delta
        double base[3];
        double step[3];
    };
    const Pattern patterns[] = {
        {{-1.0, -1.0, 0.0}, {0.0, 1.0, 0.0}},              // a vortex: the pair is lambda2's
        {{0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}},                // a small pair under one large
        {{1.0, 1.0, 1.0}, {0.0, 1.0, 2.0}},                // a triple cluster
        {{-1000.0, -1000.0, 0.001}, {0.0, 1000.0, 0.0}},   // a pair, six decades from the third
    };
    constexpr int kMaxSweeps = 8;
    std::array<double, kMaxSweeps + 1> worst{}, worst_middle{};
    XorShift rng;
    for (const Pattern& p : patterns) {
        for (const double delta : {1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 0.0}) {
            const Mat3 diag = {{{p.base[0] + p.step[0] * delta, 0.0, 0.0},
                                {0.0, p.base[1] + p.step[1] * delta, 0.0},
                                {0.0, 0.0, p.base[2] + p.step[2] * delta}}};
            for (int n = 0; n < 256; ++n) {
                const Mat3 r = rotation(rng.signed_unit(), rng.signed_unit(), rng.signed_unit(), rng.signed_unit());
                const Mat3 a = rotate(r, diag);
                const Symmetric3 m{static_cast<float>(a[0][0]), static_cast<float>(a[1][1]),
                                   static_cast<float>(a[2][2]), static_cast<float>(a[0][1]),
                                   static_cast<float>(a[0][2]), static_cast<float>(a[1][2])};
                const std::array<double, 3> want = reference_eigenvalues(widen(m));
                const double scale = std::max(std::abs(want[0]), std::abs(want[2]));
                for (int sweeps = 1; sweeps <= kMaxSweeps; ++sweeps) {
                    const std::array<float, 3> got = symmetric_eigenvalues(m, sweeps);
                    for (int k = 0; k < 3; ++k) {
                        worst[sweeps] = std::max(worst[sweeps], std::abs(got[k] - want[k]) / scale);
                    }
                    worst_middle[sweeps] = std::max(worst_middle[sweeps], std::abs(got[1] - want[1]) / scale);
                }
            }
        }
    }
    for (int sweeps = 1; sweeps <= kMaxSweeps; ++sweeps) {
        std::printf("[ lambda2 ] %d sweep%s: worst |error| / |largest eigenvalue| = %.3g (the middle: %.3g)\n",
                    sweeps, sweeps == 1 ? " " : "s", worst[sweeps], worst_middle[sweeps]);
    }
    EXPECT_LE(worst[spade::kLambda2Sweeps], kPinnedSweeps) << "kLambda2Sweeps must reach float precision";
    EXPECT_LE(worst_middle[spade::kLambda2Sweeps], kPinnedSweepsMiddle);
    EXPECT_GT(worst[2], kPinnedSweeps) << "the cases must be hard enough that too few sweeps fail";
}

// The bits a gcc build must reproduce (TD-3): an FNV-1a digest of lambda2
// over 4096 gradients at three power-of-two scales. Physics' airflow golden
// folds lambda2, so it relies on this.
TEST(CoreLambda2, TheSameBitsOnEveryCompiler) {
    XorShift rng;
    uint64_t digest = 0xCBF29CE484222325ull;
    for (const float scale : {1.0f / 1024.0f, 1.0f, 1024.0f}) {
        for (int n = 0; n < 4096; ++n) {
            VelocityGradient g{};
            for (auto& row : g) {
                for (float& v : row) v = scale * static_cast<float>(rng.signed_unit());
            }
            const uint32_t bits = std::bit_cast<uint32_t>(lambda2(g));
            for (int byte = 0; byte < 4; ++byte) {
                digest ^= (bits >> (8 * byte)) & 0xFFu;
                digest *= 0x100000001B3ull;
            }
        }
    }
    std::printf("[ lambda2 ] digest %016llx\n", static_cast<unsigned long long>(digest));
    EXPECT_EQ(digest, kPinnedDigest);
}

}  // namespace
