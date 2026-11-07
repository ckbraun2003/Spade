// test_lbm_collision.cpp -- L1's collision, per node (design: docs/design/
// physics/plans/2026-11-04-l1-lbm-design.md, sections 1 and 6). Every expected
// value is derived in double from the moments, not copied from the code.
//
//   LbmCollision  the shifted third-order equilibrium's moments; mass and
//                 momentum conserved; tau = 1 returns the equilibrium exactly;
//                 the stress and the recursion's third-order part relax by
//                 (1 - omega); WALE is 0 in uniform flow and pure shear and
//                 equals its closed form in solid rotation.
//
// Mutations (each fails a test below): the third-order recursion dropped (the
// third-order moment test); the shift left out of the density (the equilibrium
// test); WALE's Sd without its trace (the rotation test).

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>

#include "physics/lbm/lattice.hpp"
#include "physics/lbm/lbm_collide.hpp"

namespace {

using namespace spade::physics::lbm;

double cv(uint32_t i, int axis) {
    const Vel c = kC[i];
    return axis == 0 ? c.x : (axis == 1 ? c.y : c.z);
}

// Populations at (rho, u) plus a symmetric traceless second-order deviation
// a2 (the Hermite form w 4.5 (c.a2.c - tr/3)) and a small rough part.
Pops make_pops(double rho, const double u[3], const double a2[3][3], double rough) {
    Macro m;
    m.rho = static_cast<float>(rho);
    m.rho_m1 = static_cast<float>(rho - 1.0);
    m.ux = static_cast<float>(u[0]);
    m.uy = static_cast<float>(u[1]);
    m.uz = static_cast<float>(u[2]);
    Pops f{};
    uint32_t seed = 12345u;
    for (uint32_t i = 0; i < kQ; ++i) {
        double q = 0.0;
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) q += cv(i, a) * cv(i, b) * a2[a][b];
        }
        const double tr = a2[0][0] + a2[1][1] + a2[2][2];
        seed = seed * 1664525u + 1013904223u;
        const double r = (static_cast<double>(seed >> 8) / 16777216.0 - 0.5) * rough;
        f[i] = equilibrium_shifted(i, m) + static_cast<float>(static_cast<double>(kW[i]) * 4.5 * (q - tr / 3.0) + r);
    }
    return f;
}

constexpr double kU[3] = {0.05, -0.03, 0.02};
constexpr double kA2[3][3] = {{2.0e-4, 1.0e-4, -5.0e-5}, {1.0e-4, -1.5e-4, 3.0e-5}, {-5.0e-5, 3.0e-5, -0.5e-4}};

TEST(LbmCollision, TheShiftedEquilibriumHasTheRightMoments) {
    const double rho = 1.03;
    Macro m;
    m.rho = static_cast<float>(rho);
    m.rho_m1 = static_cast<float>(rho - 1.0);
    m.ux = static_cast<float>(kU[0]);
    m.uy = static_cast<float>(kU[1]);
    m.uz = static_cast<float>(kU[2]);
    double s = 0.0;
    double j[3] = {0, 0, 0};
    double p[3][3] = {};
    double xxy = 0.0;
    double xyz = 0.0;
    for (uint32_t i = 0; i < kQ; ++i) {
        const double fe = static_cast<double>(equilibrium_shifted(i, m));
        s += fe;
        for (int a = 0; a < 3; ++a) {
            j[a] += cv(i, a) * fe;
            for (int b = 0; b < 3; ++b) p[a][b] += cv(i, a) * cv(i, b) * (fe + static_cast<double>(kW[i]));
        }
        xxy += (cv(i, 0) * cv(i, 0) * cv(i, 1)) * (fe + static_cast<double>(kW[i]));
        xyz += (cv(i, 0) * cv(i, 1) * cv(i, 2)) * (fe + static_cast<double>(kW[i]));
    }
    EXPECT_NEAR(s, rho - 1.0, 2.0e-7);
    for (int a = 0; a < 3; ++a) EXPECT_NEAR(j[a], rho * kU[a], 2.0e-7) << a;
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            EXPECT_NEAR(p[a][b], rho * (kU[a] * kU[b] + (a == b ? 1.0 / 3.0 : 0.0)), 4.0e-7) << a << b;
        }
    }
    // The third-order equilibrium: sum c_x c_x c_y f = rho (u_x^2 u_y + u_y / 3),
    // and sum c_x c_y c_z f = rho u_x u_y u_z.
    EXPECT_NEAR(xxy, rho * (kU[0] * kU[0] * kU[1] + kU[1] / 3.0), 4.0e-7);
    EXPECT_NEAR(xyz, rho * kU[0] * kU[1] * kU[2], 4.0e-7);
}

TEST(LbmCollision, MassAndMomentumAreConservedAtEveryRelaxationRate) {
    for (const float omega : {0.2f, 1.0f, 1.7f, 1.999f}) {
        Pops f = make_pops(1.02, kU, kA2, 2.0e-5);
        const Macro before = macro_of(f);
        collide_rr3(f, omega);
        const Macro after = macro_of(f);
        EXPECT_NEAR(after.rho_m1, before.rho_m1, 1.0e-6) << omega;
        EXPECT_NEAR(after.rho * after.ux, before.rho * before.ux, 1.0e-6) << omega;
        EXPECT_NEAR(after.rho * after.uy, before.rho * before.uy, 1.0e-6) << omega;
        EXPECT_NEAR(after.rho * after.uz, before.rho * before.uz, 1.0e-6) << omega;
    }
}

// tau = 1 (omega = 1) keeps nothing of the deviation: the result is the equilibrium, bit for bit.
TEST(LbmCollision, AtTauOneTheResultIsTheEquilibriumExactly) {
    Pops f = make_pops(0.98, kU, kA2, 5.0e-5);
    const Macro m = macro_of(f);
    collide_rr3(f, 1.0f);
    for (uint32_t i = 0; i < kQ; ++i) EXPECT_EQ(f[i], equilibrium_shifted(i, m)) << i;
}

// The stress relaxes by (1 - omega); the recursion's third-order part is the
// u-weighted sum of the same stress, relaxed by the same factor.
TEST(LbmCollision, TheStressAndTheRecursionsThirdOrderPartRelaxByOneMinusOmega) {
    const float omega = 1.4f;
    Pops f = make_pops(1.0, kU, kA2, 0.0);  // no rough part: f_neq is exactly the stress form
    const Macro m = macro_of(f);
    Pops fe{};
    for (uint32_t i = 0; i < kQ; ++i) fe[i] = equilibrium_shifted(i, m);
    // The stress actually present (computed from f - feq, in double).
    double a2[3][3] = {};
    for (uint32_t i = 0; i < kQ; ++i) {
        const double n = static_cast<double>(f[i]) - static_cast<double>(fe[i]);
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) a2[a][b] += cv(i, a) * cv(i, b) * n;
        }
    }
    collide_rr3(f, omega);
    const double keep = 1.0 - static_cast<double>(omega);
    double b2[3][3] = {};
    double b_xyz = 0.0;
    double b_xxy = 0.0;
    for (uint32_t i = 0; i < kQ; ++i) {
        const double n = static_cast<double>(f[i]) - static_cast<double>(fe[i]);
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) b2[a][b] += cv(i, a) * cv(i, b) * n;
        }
        b_xyz += cv(i, 0) * cv(i, 1) * cv(i, 2) * n;
        b_xxy += (cv(i, 0) * cv(i, 0) - 1.0 / 3.0) * cv(i, 1) * n;
    }
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) EXPECT_NEAR(b2[a][b], keep * a2[a][b], 3.0e-8) << a << b;
    }
    const double ux = kU[0];
    const double uy = kU[1];
    const double uz = kU[2];
    const double a_xyz = ux * a2[1][2] + uy * a2[0][2] + uz * a2[0][1];
    const double a_yxx = uy * a2[0][0] + 2.0 * ux * a2[0][1];  // a3 with index y once and x twice
    EXPECT_NEAR(b_xyz, keep * a_xyz, 3.0e-8);
    EXPECT_NEAR(b_xxy, keep * a_yxx, 3.0e-8);
}

// Uniform flow and pure shear carry no eddy viscosity; solid rotation does, as WALE's operator gives:
// S = 0, Sd = diag(-1/3, -1/3, 2/3) Omega^2, so nu_t = C_w^2 (2/3)^(1/4) |Omega|.
TEST(LbmCollision, WaleVanishesInUniformFlowAndShearAndHasItsClosedFormInRotation) {
    const float cw = 0.5f;
    const std::array<float, 9> zero{};
    EXPECT_EQ(wale_nu_t(zero, cw), 0.0f);

    std::array<float, 9> shear{};
    shear[1] = 0.01f;  // du_x/dy
    EXPECT_EQ(wale_nu_t(shear, cw), 0.0f);

    const float om = 0.02f;
    std::array<float, 9> rot{};
    rot[1] = -om;  // du_x/dy
    rot[3] = om;   // du_y/dx
    const double want = 0.25 * std::pow(2.0 / 3.0, 0.25) * 0.02;
    EXPECT_NEAR(static_cast<double>(wale_nu_t(rot, cw)), want, 1.0e-7 * want * 1.0e3);
    EXPECT_GT(wale_nu_t(rot, cw), 0.0f);
}

}  // namespace
