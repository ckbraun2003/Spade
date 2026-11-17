// ---------------------------------------------------------------------------
// airflow_line_tipcorr.slang (L72k, A1) -- the actuator line's tip/smearing correction between airflow_line_sample and
// airflow_line_load: engine/physics/airflow/line.cpp's line_tip_correct() and line_segment_induction() term for term (the CPU
// twins, and the authority for every expression).
//
// The fixed point couples the elements of a blade, so one pass is three dispatches, one thread per element, the push's word 1 the
// phase, the host recording the chain in the CPU's order (phase 0, 2, 3, then 1, 2, 3 for each further pass):
//   0  the first pass's circulation: saves the sampled air, then as phase 1
//   1  each element's axial speed ua and circulation Gamma = L' / (rho W) from the air it now feels (airflow_line_vel)
//   2  Gamma filtered along the span by the kernel's own Gaussian (the group's table), zero beyond the blade's ends
//   3  the induction of every filament within 3 eps, the damped update of the air: u += relax * ((sampled + du) - u)
// Scratch is airflow_line_force, a slot each element of kForceStride floats that airflow_line_load overwrites after this:
// [0-2] the sampled air  [3] ua  [4] Gamma  [5] the filtered Gamma.
//
// An element belongs to a tip-corrected blade if its record's word 14 is its group's number + 1 (0: none, the thread leaves the
// air as the sample wrote it) and word 15 its place in the blade. The group's record (airflow_line_elems after the element records,
// spike/airflow/line_chain.hpp) carries the geometry and the tables line_tip_tables() built: the kernel computes no sin, cos or
// Gaussian table. The filaments' vertices are formed where they are used: the same expression of the same floats, so the same bits.
//
// PARITY. -fp-mode precise keeps the arithmetic uncontracted; the dot products are written out; the square roots are sqrt32_ftz
// and the divisions div32_ftz (exact, under the CPU's FTZ|DAZ); the Gaussian factor is fp32_math's exp32, the CPU's.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

struct LinePush {
    uint element_count;  // word 0
    uint phase;          // word 1
    uint count;          // word 2: the invocations (the element count)
    uint unused2;        // word 3
};

[[vk::push_constant]] ConstantBuffer<LinePush> line_push;

static const uint kLineStride = 16u + 3u * (7u + 3u * 13u);
static const uint kForceStride = 8u;
static const uint kTipHeader = 16u;
static const uint kTipStride = 160u;

float dot3(float3 a, float3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// blade.cpp's diamond_angle(): finite c and s, else NaN.
float diamond_angle(float c, float s) {
    if (!isfinite(c) || !isfinite(s)) { return asfloat(0x7FC00000u); }
    const float ac = c < 0.0 ? -c : c;
    const float as = s < 0.0 ? -s : s;
    const float d = ac + as;
    if (d == 0.0) { return 0.0; }
    const float t = div32_ftz(s, d);
    if (c >= 0.0) { return t; }
    const float m = 2.0 - div32_ftz(as, d);
    return s < 0.0 ? -m : m;
}

// line.cpp's line_segment_induction(): f * the Biot-Savart velocity of a straight filament a -> b of unit circulation at p.
float3 segment_induction(float3 p, float3 a, float3 b, float eps_m) {
    const float3 r1 = p - a;
    const float3 r2 = p - b;
    const float3 l = b - a;
    const float cx = r1.y * r2.z - r1.z * r2.y;
    const float cy = r1.z * r2.x - r1.x * r2.z;
    const float cz = r1.x * r2.y - r1.y * r2.x;
    const float c2 = cx * cx + cy * cy + cz * cz;
    const float l2 = l.x * l.x + l.y * l.y + l.z * l.z;
    const float n1 = sqrt32_ftz(r1.x * r1.x + r1.y * r1.y + r1.z * r1.z);
    const float n2 = sqrt32_ftz(r2.x * r2.x + r2.y * r2.y + r2.z * r2.z);
    if (!(l2 > 0.0) || !(n1 > 0.0) || !(n2 > 0.0) || !(c2 > 1.0e-12 * l2)) { return float3(0.0, 0.0, 0.0); }
    float f = 1.0;
    if (eps_m > 0.0) {
        const float q = div32_ftz(div32_ftz(c2, l2), eps_m * eps_m);
        if (q > 20.0) { return float3(0.0, 0.0, 0.0); }
        f = exp32(-q);
    }
    const float pi = asfloat(0x40490FDBu);  // std::numbers::pi_v<float>
    const float k = div32_ftz(l.x * r1.x + l.y * r1.y + l.z * r1.z, n1) -
                    div32_ftz(l.x * r2.x + l.y * r2.y + l.z * r2.z, n2);
    const float scale = div32_ftz(f * k, (4.0 * pi) * c2);
    return float3(cx * scale, cy * scale, cz * scale);
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_line_tipcorr(uint3 tid : SV_DispatchThreadID) {
    const uint e = tid.x;
    if (e >= line_push.count) { return; }
    const uint rec = e * kLineStride;
    const uint gi = uint(airflow_line_elems[rec + 14u]);
    if (gi == 0u) { return; }
    const uint k = uint(airflow_line_elems[rec + 15u]);
    const uint tip0 = line_push.element_count * kLineStride + kTipHeader;
    const uint grp = tip0 + (gi - 1u) * kTipStride;
    const uint first = uint(airflow_line_elems[grp]);
    const uint per = uint(airflow_line_elems[grp + 1u]);
    const uint scratch = e * kForceStride;
    const float3 axis = float3(airflow_line_elems[rec], airflow_line_elems[rec + 1u], airflow_line_elems[rec + 2u]);
    const float3 motion =
        float3(airflow_line_elems[rec + 3u], airflow_line_elems[rec + 4u], airflow_line_elems[rec + 5u]);
    const float omega = airflow_line_elems[rec + 11u];
    const float r = airflow_line_elems[rec + 9u];
    const float3 u = float3(airflow_line_vel[e * 4u], airflow_line_vel[e * 4u + 1u], airflow_line_vel[e * 4u + 2u]);
    const uint phase = line_push.phase;

    if (phase <= 1u) {
        if (phase == 0u) {
            airflow_line_force[scratch] = u.x;
            airflow_line_force[scratch + 1u] = u.y;
            airflow_line_force[scratch + 2u] = u.z;
        }
        const float chord = airflow_line_elems[rec + 6u];
        const float sin_beta = airflow_line_elems[rec + 7u];
        const float cos_beta = airflow_line_elems[rec + 8u];
        const float rho = airflow_line_elems[rec + 13u];
        const float ua = -dot3(u, axis);
        const float wt = omega * r - dot3(u, motion);
        float gamma = 0.0;
        const float w2 = ua * ua + wt * wt;
        if (w2 != 0.0) {
            // section_load(), as airflow_line_load.slang has it.
            const float w = sqrt32_ftz(w2);
            const float cphi = div32_ftz(wt, w);
            const float sphi = div32_ftz(ua, w);
            const float sin_alpha = sin_beta * cphi - cos_beta * sphi;
            const float cos_alpha = cos_beta * cphi + sin_beta * sphi;
            const uint n = uint(airflow_line_polar[0]);
            const float kd = diamond_angle(cos_alpha, sin_alpha);
            float cl = 0.0;
            float cd = 0.0;
            if (n > 0u) {
                const uint keys = 1u;
                const uint cls = 1u + n;
                const uint cds = 1u + 2u * n;
                if (!isfinite(kd)) {
                    cl = kd;
                    cd = kd;
                } else if (kd <= airflow_line_polar[keys]) {
                    cl = airflow_line_polar[cls];
                    cd = airflow_line_polar[cds];
                } else if (kd >= airflow_line_polar[keys + n - 1u]) {
                    cl = airflow_line_polar[cls + n - 1u];
                    cd = airflow_line_polar[cds + n - 1u];
                } else {
                    uint a = 0u;
                    uint b = n;
                    while (a < b) {
                        const uint mid = (a + b) / 2u;
                        if (airflow_line_polar[keys + mid] <= kd) { a = mid + 1u; } else { b = mid; }
                    }
                    const uint hi = a;
                    const uint lo = hi - 1u;
                    const float k_lo = airflow_line_polar[keys + lo];
                    const float t = div32_ftz(kd - k_lo, airflow_line_polar[keys + hi] - k_lo);
                    cl = airflow_line_polar[cls + lo] +
                         (airflow_line_polar[cls + hi] - airflow_line_polar[cls + lo]) * t;
                    cd = airflow_line_polar[cds + lo] +
                         (airflow_line_polar[cds + hi] - airflow_line_polar[cds + lo]) * t;
                }
            }
            const float q = 0.5 * rho * w2 * chord;
            const float lift = q * cl;
            const float drag = q * cd;
            const float axial = lift * cphi - drag * sphi;
            const float tangent = lift * sphi + drag * cphi;
            gamma = div32_ftz(axial * cphi + tangent * sphi, rho * w);  // L' / (rho W)
        }
        airflow_line_force[scratch + 3u] = ua;
        airflow_line_force[scratch + 4u] = gamma;
        return;
    }

    if (phase == 2u) {
        const int reach = int(airflow_line_elems[grp + 4u]);
        const float kern_sum = airflow_line_elems[grp + 9u];
        float acc = 0.0;
        for (int off = -reach; off <= reach; ++off) {
            const int j = int(k) + off;
            if (j < 0 || j >= int(per)) { continue; }
            acc += airflow_line_elems[grp + 128u + uint(off < 0 ? -off : off)] *
                   airflow_line_force[(first + uint(j)) * kForceStride + 4u];
        }
        airflow_line_force[scratch + 5u] = div32_ftz(acc, kern_sum);
        return;
    }

    // Phase 3: the induction at this element, the damped update.
    const uint segs = uint(airflow_line_elems[grp + 3u]);
    const float spin = airflow_line_elems[grp + 5u];
    const float dr = airflow_line_elems[grp + 6u];
    const float r_in = airflow_line_elems[grp + 7u];
    const float eps_m = airflow_line_elems[grp + 8u];
    const float relax = airflow_line_elems[grp + 10u];
    const float3 centre =
        float3(airflow_line_elems[grp + 16u], airflow_line_elems[grp + 17u], airflow_line_elems[grp + 18u]);
    const float3 dir =
        float3(airflow_line_elems[grp + 19u], airflow_line_elems[grp + 20u], airflow_line_elems[grp + 21u]);
    const float3 m =
        float3(airflow_line_elems[grp + 22u], airflow_line_elems[grp + 23u], airflow_line_elems[grp + 24u]);
    const float3 p = centre + dir * r;
    float3 du = float3(0.0, 0.0, 0.0);
    for (uint v = 0u; v <= per; ++v) {
        const float rv = r_in + float(v) * dr;
        float g_in = 0.0;  // ifs, not selects: no read of an element the blade does not have
        float g_out = 0.0;
        if (v > 0u) { g_in = airflow_line_force[(first + v - 1u) * kForceStride + 5u]; }
        if (v < per) { g_out = airflow_line_force[(first + v) * kForceStride + 5u]; }
        const float strength = spin * (g_in - g_out);
        const float d_r = rv - r;
        if (strength == 0.0 || (d_r < 0.0 ? -d_r : d_r) > 3.0 * eps_m) { continue; }
        float ua_v = 0.0;
        if (v == 0u) {
            ua_v = airflow_line_force[first * kForceStride + 3u];
        } else if (v == per) {
            ua_v = airflow_line_force[(first + per - 1u) * kForceStride + 3u];
        } else {
            ua_v = 0.5 * (airflow_line_force[(first + v - 1u) * kForceStride + 3u] +
                          airflow_line_force[(first + v) * kForceStride + 3u]);
        }
        const float sink = div32_ftz(max(ua_v, 0.0), omega);
        float3 a = float3(0.0, 0.0, 0.0);
        for (uint j = 0u; j <= segs; ++j) {
            const float ct = airflow_line_elems[grp + 64u + j];
            const float st = airflow_line_elems[grp + 96u + j];
            const float th = airflow_line_elems[grp + 32u + j];
            const float3 b = centre + (dir * ct - m * st) * rv - axis * (sink * th);
            if (j > 0u) { du += strength * segment_induction(p, a, b, eps_m); }
            a = b;
        }
    }
    const float3 sampled =
        float3(airflow_line_force[scratch], airflow_line_force[scratch + 1u], airflow_line_force[scratch + 2u]);
    const float3 next = u + relax * ((sampled + du) - u);
    airflow_line_vel[e * 4u] = next.x;
    airflow_line_vel[e * 4u + 1u] = next.y;
    airflow_line_vel[e * 4u + 2u] = next.z;
}
