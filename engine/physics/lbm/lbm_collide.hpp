#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "physics/lbm/lattice.hpp"

// ---------------------------------------------------------------------------
// physics/lbm/lbm_collide.hpp -- the LBM tier's collision, per node (L1; design
// section 1): the recursive-regularized D3Q27 collision to third order (RR3;
// Malaspinas 2015, Coreixas et al. 2017) with the relaxation rate supplied by
// the caller, and WALE's eddy viscosity (Nicoud and Ducros 1999) from a
// velocity gradient.
//
// Populations are the SHIFTED ones (f' = f - w, lbm_grid.hpp). Every sum runs
// in direction order 0..26 and every operation is + - * / or sqrt on fp32, so a
// GPU twin written line for line (NoContraction) gives the same bits.
//
//   rho - 1 = sum f'        rho u = sum c f'        (the rest state: no momentum)
//   f_eq' = w [ (rho - 1) + rho (3 cu + 4.5 cu^2 - 1.5 u^2 + 4.5 cu^3 - 4.5 u^2 cu) ]
//   f_neq = f' - f_eq'     (the shifts cancel)
//   a2    = sum c c f_neq                              (6 components)
//   a3    = u a2 + u a2 + u a2  (the recursion; 7 non-aliased components used)
//   f_reg = w [ 4.5 (c.a2.c - tr a2 / 3) + 4.5 H3:a3 ]
//   f*'   = f_eq' + (1 - omega) f_reg,   omega = 1 / tau,  tau = 3 (nu + nu_t) + 1/2
// ---------------------------------------------------------------------------

namespace spade::physics::lbm {

using Pops = std::array<float, kQ>;

struct Macro {
    float rho_m1 = 0.0f;  // rho - 1, summed directly (the shifted storage's point)
    float rho = 1.0f;
    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;
};

[[nodiscard]] inline Macro macro_of(const Pops& f) noexcept {
    float s = 0.0f;
    float jx = 0.0f;
    float jy = 0.0f;
    float jz = 0.0f;
    for (uint32_t i = 0; i < kQ; ++i) {
        s += f[i];
        jx += static_cast<float>(kC[i].x) * f[i];
        jy += static_cast<float>(kC[i].y) * f[i];
        jz += static_cast<float>(kC[i].z) * f[i];
    }
    Macro m;
    m.rho_m1 = s;
    m.rho = 1.0f + s;
    m.ux = jx / m.rho;
    m.uy = jy / m.rho;
    m.uz = jz / m.rho;
    return m;
}

// The shifted third-order equilibrium of direction i.
[[nodiscard]] inline float equilibrium_shifted(uint32_t i, const Macro& m) noexcept {
    const float cx = static_cast<float>(kC[i].x);
    const float cy = static_cast<float>(kC[i].y);
    const float cz = static_cast<float>(kC[i].z);
    const float cu = cx * m.ux + cy * m.uy + cz * m.uz;
    const float u2 = m.ux * m.ux + m.uy * m.uy + m.uz * m.uz;
    const float cu2 = cu * cu;
    const float h = 3.0f * cu + (4.5f * cu2 - 1.5f * u2) + 4.5f * (cu * cu2 - u2 * cu);
    return kW[i] * (m.rho_m1 + m.rho * h);
}

// Collide one node in place with the relaxation rate omega = 1 / tau.
inline void collide_rr3(Pops& f, float omega) noexcept {
    const Macro m = macro_of(f);
    Pops feq{};
    float pxx = 0.0f;
    float pyy = 0.0f;
    float pzz = 0.0f;
    float pxy = 0.0f;
    float pxz = 0.0f;
    float pyz = 0.0f;
    for (uint32_t i = 0; i < kQ; ++i) {
        feq[i] = equilibrium_shifted(i, m);
        const float n = f[i] - feq[i];
        const float cx = static_cast<float>(kC[i].x);
        const float cy = static_cast<float>(kC[i].y);
        const float cz = static_cast<float>(kC[i].z);
        pxx += cx * cx * n;
        pyy += cy * cy * n;
        pzz += cz * cz * n;
        pxy += cx * cy * n;
        pxz += cx * cz * n;
        pyz += cy * cz * n;
    }
    // The recursion's third-order moments, a_abb = u_a a2_bb + 2 u_b a2_ab and a_xyz.
    const float a_xyy = m.ux * pyy + 2.0f * m.uy * pxy;
    const float a_xzz = m.ux * pzz + 2.0f * m.uz * pxz;
    const float a_yxx = m.uy * pxx + 2.0f * m.ux * pxy;
    const float a_yzz = m.uy * pzz + 2.0f * m.uz * pyz;
    const float a_zxx = m.uz * pxx + 2.0f * m.ux * pxz;
    const float a_zyy = m.uz * pyy + 2.0f * m.uy * pyz;
    const float a_xyz = m.ux * pyz + m.uy * pxz + m.uz * pxy;
    const float tr = pxx + pyy + pzz;
    const float keep = 1.0f - omega;
    for (uint32_t i = 0; i < kQ; ++i) {
        const float cx = static_cast<float>(kC[i].x);
        const float cy = static_cast<float>(kC[i].y);
        const float cz = static_cast<float>(kC[i].z);
        const float q2 = cx * cx * pxx + cy * cy * pyy + cz * cz * pzz +
                         2.0f * (cx * cy * pxy + cx * cz * pxz + cy * cz * pyz);
        const float x2 = 3.0f * cx * cx - 1.0f;
        const float y2 = 3.0f * cy * cy - 1.0f;
        const float z2 = 3.0f * cz * cz - 1.0f;
        const float t3 = cx * (y2 * a_xyy + z2 * a_xzz) + cy * (x2 * a_yxx + z2 * a_yzz) +
                         cz * (x2 * a_zxx + y2 * a_zyy) + 6.0f * cx * cy * cz * a_xyz;
        const float reg = kW[i] * (4.5f * (q2 - tr * 0.33333334f) + 4.5f * t3);
        f[i] = feq[i] + keep * reg;
    }
}

// WALE's eddy viscosity in lattice units (filter width 1) from g[3 i + j] = du_i / dx_j.
// Zero when the denominator is zero (uniform flow, pure shear's Sd).
[[nodiscard]] inline float wale_nu_t(const std::array<float, 9>& g, float cw) noexcept {
    float s2 = 0.0f;
    float sd2 = 0.0f;
    float g2[9];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            g2[3 * i + j] = g[3 * i + 0] * g[0 + j] + g[3 * i + 1] * g[3 + j] + g[3 * i + 2] * g[6 + j];
        }
    }
    const float tr = (g2[0] + g2[4] + g2[8]) * 0.33333334f;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const float s = 0.5f * (g[3 * i + j] + g[3 * j + i]);
            float sd = 0.5f * (g2[3 * i + j] + g2[3 * j + i]);
            if (i == j) sd -= tr;
            s2 += s * s;
            sd2 += sd * sd;
        }
    }
    const float num = sd2 * std::sqrt(sd2);
    const float den = s2 * s2 * std::sqrt(s2) + sd2 * std::sqrt(std::sqrt(sd2));
    if (den == 0.0f) return 0.0f;
    return cw * cw * (num / den);
}

// tau = 3 (nu + nu_t) + 1/2, and omega = 1 / tau.
[[nodiscard]] inline float omega_of(float nu, float nu_t) noexcept { return 1.0f / (3.0f * (nu + nu_t) + 0.5f); }

}  // namespace spade::physics::lbm
