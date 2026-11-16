// ---------------------------------------------------------------------------
// airflow_line_load.slang (P4c, L61) -- the actuator line's second kernel: each element's section load and the force it gives the air,
// engine/physics/airflow/blade.cpp's section_load() and polar_at() and line.cpp's spread_flushed() term for term (the CPU twins, and
// the authority for every expression).
//
// One invocation per element, guarded by the push's count (word 2). Reads the element's record (airflow_line_elems), the air
// airflow_line_sample wrote (airflow_line_vel) and the polar (airflow_line_polar: n, the n diamond keys, the n C_l, the n C_d). Writes
// airflow_line_force, 8 floats an element: [0-2] the force on the air, (-axial * axis + tangent * motion) * (dr * share); [3] the
// blade's axial load per metre; [4] its tangent load per metre; the rest 0.
//
// PARITY. -fp-mode precise keeps the arithmetic uncontracted; the dot products are written out (a . b = ax bx + ay by + az bz, glm's);
// the square root is sqrt32_ftz and the divisions div32_ftz (exact, under the CPU's FTZ|DAZ); polar_at()'s std::upper_bound is a
// binary search for the first key above k, which gives the same index.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

struct LinePush {
    uint element_count;  // word 0
    uint unused;         // word 1
    uint count;          // word 2: the invocations (the element count)
    uint unused2;        // word 3
};

[[vk::push_constant]] ConstantBuffer<LinePush> line_push;

static const uint kLineStride = 16u + 3u * (7u + 3u * 13u);
static const uint kForceStride = 8u;

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

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_line_load(uint3 tid : SV_DispatchThreadID) {
    const uint e = tid.x;
    if (e >= line_push.count) { return; }
    const uint rec = e * kLineStride;
    const float3 axis = float3(airflow_line_elems[rec], airflow_line_elems[rec + 1u], airflow_line_elems[rec + 2u]);
    const float3 motion =
        float3(airflow_line_elems[rec + 3u], airflow_line_elems[rec + 4u], airflow_line_elems[rec + 5u]);
    const float chord = airflow_line_elems[rec + 6u];
    const float sin_beta = airflow_line_elems[rec + 7u];
    const float cos_beta = airflow_line_elems[rec + 8u];
    const float r = airflow_line_elems[rec + 9u];
    const float drs = airflow_line_elems[rec + 10u];
    const float omega = airflow_line_elems[rec + 11u];
    const float rho = airflow_line_elems[rec + 13u];
    const float3 air = float3(airflow_line_vel[e * 4u], airflow_line_vel[e * 4u + 1u], airflow_line_vel[e * 4u + 2u]);
    const float ua = -dot3(air, axis);
    const float wt = omega * r - dot3(air, motion);
    float axial = 0.0;
    float tangent = 0.0;
    const float w2 = ua * ua + wt * wt;
    if (w2 != 0.0) {
        const float w = sqrt32_ftz(w2);
        const float cphi = div32_ftz(wt, w);
        const float sphi = div32_ftz(ua, w);
        const float sin_alpha = sin_beta * cphi - cos_beta * sphi;
        const float cos_alpha = cos_beta * cphi + sin_beta * sphi;
        const uint n = uint(airflow_line_polar[0]);
        const float k = diamond_angle(cos_alpha, sin_alpha);
        float cl = 0.0;
        float cd = 0.0;
        if (n > 0u) {
            const uint keys = 1u;
            const uint cls = 1u + n;
            const uint cds = 1u + 2u * n;
            if (!isfinite(k)) {
                cl = k;
                cd = k;
            } else if (k <= airflow_line_polar[keys]) {
                cl = airflow_line_polar[cls];
                cd = airflow_line_polar[cds];
            } else if (k >= airflow_line_polar[keys + n - 1u]) {
                cl = airflow_line_polar[cls + n - 1u];
                cd = airflow_line_polar[cds + n - 1u];
            } else {
                // The first key above k (std::upper_bound), then the interval below it.
                uint a = 0u;
                uint b = n;
                while (a < b) {
                    const uint mid = (a + b) / 2u;
                    if (airflow_line_polar[keys + mid] <= k) { a = mid + 1u; } else { b = mid; }
                }
                const uint hi = a;
                const uint lo = hi - 1u;
                const float k_lo = airflow_line_polar[keys + lo];
                const float t = div32_ftz(k - k_lo, airflow_line_polar[keys + hi] - k_lo);
                cl = airflow_line_polar[cls + lo] + (airflow_line_polar[cls + hi] - airflow_line_polar[cls + lo]) * t;
                cd = airflow_line_polar[cds + lo] + (airflow_line_polar[cds + hi] - airflow_line_polar[cds + lo]) * t;
            }
        }
        const float q = 0.5 * rho * w2 * chord;
        const float lift = q * cl;
        const float drag = q * cd;
        axial = lift * cphi - drag * sphi;
        tangent = lift * sphi + drag * cphi;
    }
    const float3 force = (-axial * axis + tangent * motion) * drs;
    const uint out = e * kForceStride;
    airflow_line_force[out] = force.x;
    airflow_line_force[out + 1u] = force.y;
    airflow_line_force[out + 2u] = force.z;
    airflow_line_force[out + 3u] = axial;
    airflow_line_force[out + 4u] = tangent;
    airflow_line_force[out + 5u] = 0.0;
    airflow_line_force[out + 6u] = 0.0;
    airflow_line_force[out + 7u] = 0.0;
}
