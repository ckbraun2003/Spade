// ---------------------------------------------------------------------------
// lbm_collide.slang -- the LBM tier's per-node maths on the device (L1, the GPU
// twin, T2 and T3; docs/design/physics/plans/2026-11-06-l1-gpu-twin-plan.md).
// A LINE-CITED PORT of engine/physics/lbm/lbm_collide.hpp (the CPU twin and the
// authority): each function is the same statements in the same order, so a
// reviewer diffs the two side by side. Populations are the SHIFTED ones
// (f' = f - w).
//
// WHAT THE PORT KEEPS (fp32_math.slang's rule 1):
//   * every operation in the C++ order, NO simplification: a multiply by 0 or 1
//     stays (a sum of c f' with c = 0 keeps the NaN or -0 the CPU keeps);
//   * every `/` is div32_ftz and every sqrt is sqrt32_ftz (rules E1/E2: no bare
//     OpFDiv, Sqrt or InverseSqrt), bit-equal to the host's divss/sqrtss under
//     FTZ|DAZ; no other division or root occurs;
//   * every other float op is NoContraction (rule P1) by the -fp-mode precise
//     compile, so a driver fuses nothing: 3.0f * cu + (...) stays a separately
//     rounded multiply and add;
//   * + - * flush subnormals as this device does by default (GpuDenormProbe),
//     as the CPU step does under ScopedFlushSubnormals: the bit-equal claim is
//     the device of record's (plan, "Subnormals").
// ---------------------------------------------------------------------------
import fp32_math;
import lbm_lattice;

// lbm_collide.hpp lines 36-62: macro_of(). rho_m1 = sum f', rho = 1 + rho_m1,
// u = (sum c f') / rho by three div32_ftz (CPU: m.ux = jx / m.rho, ...).
struct LbmMacro {
    float rho_m1;
    float rho;
    float ux;
    float uy;
    float uz;
};

LbmMacro lbm_macro_of(float f[27]) {
    float s = 0.0f;
    float jx = 0.0f;
    float jy = 0.0f;
    float jz = 0.0f;
    for (uint i = 0u; i < 27u; ++i) {
        s += f[i];
        jx += float(kLbmCx[i]) * f[i];
        jy += float(kLbmCy[i]) * f[i];
        jz += float(kLbmCz[i]) * f[i];
    }
    LbmMacro m;
    m.rho_m1 = s;
    m.rho = 1.0f + s;
    m.ux = div32_ftz(jx, m.rho);
    m.uy = div32_ftz(jy, m.rho);
    m.uz = div32_ftz(jz, m.rho);
    return m;
}

// lbm_collide.hpp lines 65-75: equilibrium_shifted(i, m).
float lbm_equilibrium_shifted(uint i, LbmMacro m) {
    const float cx = float(kLbmCx[i]);
    const float cy = float(kLbmCy[i]);
    const float cz = float(kLbmCz[i]);
    const float cu = cx * m.ux + cy * m.uy + cz * m.uz;
    const float u2 = m.ux * m.ux + m.uy * m.uy + m.uz * m.uz;
    const float cu2 = cu * cu;
    const float h = 3.0f * cu + (4.5f * cu2 - 1.5f * u2) + 4.5f * (cu * cu2 - u2 * cu);
    return lbm_weight(i) * (m.rho_m1 + m.rho * h);
}

// lbm_collide.hpp lines 78-125: collide_rr3(f, omega), in place.
void lbm_collide_rr3(inout float f[27], float omega) {
    const LbmMacro m = lbm_macro_of(f);
    float feq[27];
    float pxx = 0.0f;
    float pyy = 0.0f;
    float pzz = 0.0f;
    float pxy = 0.0f;
    float pxz = 0.0f;
    float pyz = 0.0f;
    for (uint i = 0u; i < 27u; ++i) {
        feq[i] = lbm_equilibrium_shifted(i, m);
        const float n = f[i] - feq[i];
        const float cx = float(kLbmCx[i]);
        const float cy = float(kLbmCy[i]);
        const float cz = float(kLbmCz[i]);
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
    for (uint i = 0u; i < 27u; ++i) {
        const float cx = float(kLbmCx[i]);
        const float cy = float(kLbmCy[i]);
        const float cz = float(kLbmCz[i]);
        const float q2 = cx * cx * pxx + cy * cy * pyy + cz * cz * pzz +
                         2.0f * (cx * cy * pxy + cx * cz * pxz + cy * cz * pyz);
        const float x2 = 3.0f * cx * cx - 1.0f;
        const float y2 = 3.0f * cy * cy - 1.0f;
        const float z2 = 3.0f * cz * cz - 1.0f;
        const float t3 = cx * (y2 * a_xyy + z2 * a_xzz) + cy * (x2 * a_yxx + z2 * a_yzz) +
                         cz * (x2 * a_zxx + y2 * a_zyy) + 6.0f * cx * cy * cz * a_xyz;
        const float reg = lbm_weight(i) * (4.5f * (q2 - tr * 0.33333334f) + 4.5f * t3);
        f[i] = feq[i] + keep * reg;
    }
}

// lbm_collide.hpp lines 128-152: wale_nu_t(g, cw) from g[3 i + j] = du_i / dx_j. Two divisions' worth of
// care: num / den is div32_ftz; the three roots are sqrt32_ftz, in the C++ nesting (sqrt(sqrt(sd2)) is
// two calls).
float lbm_wale_nu_t(float g[9], float cw) {
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
            if (i == j) { sd -= tr; }
            s2 += s * s;
            sd2 += sd * sd;
        }
    }
    const float num = sd2 * sqrt32_ftz(sd2);
    const float den = s2 * s2 * sqrt32_ftz(s2) + sd2 * sqrt32_ftz(sqrt32_ftz(sd2));
    if (den == 0.0f) { return 0.0f; }
    return cw * cw * div32_ftz(num, den);
}

// lbm_collide.hpp line 155: omega_of(nu, nu_t) = 1 / (3 (nu + nu_t) + 1/2).
float lbm_omega_of(float nu, float nu_t) {
    return div32_ftz(1.0f, 3.0f * (nu + nu_t) + 0.5f);
}
