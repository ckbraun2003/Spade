// ---------------------------------------------------------------------------
// grid_points.slang (module API stage 7, Task B3) -- trilinear interpolation of a cell-centred field at points. The device
// twin of engine/world/grid_kernels.hpp's grid_points(), in the same order of evaluation: per axis
// g = (p - origin) * inv_cell - 0.5, a non-finite g answers NaN BEFORE the clamp (a min/max fold would drop it), g clamps to
// [0, n-1], i0 = min(uint(g), n-2) (0 when n = 1), t = g - i0; then lerp(a, b, t) = a + (b - a) * t along x for the four
// (y, z) corners, y for two, z for one. Domain-neutral and Core's; Physics reviews.
//
// SET 1, THE AIRFLOW SET: airflow_u is the field (comps floats per cell, x fastest), airflow_v the points (three floats
// each), airflow_w the output (comps floats per point). One thread per point, 1D (SPADE_WG).
//
// PARITY WITH THE CPU TWIN. Every operation is a separately rounded fp32 add, sub or mul: the build's -fp-mode precise
// decorates each NoContraction, so no driver fuses (b - a) * t + a (P1), and the SPIR-V policy scan (AirflowRunnerSpirv) must
// show decorated == contractable for this kernel. inv_cell is formed on the host and multiplied. No atomics.
// ---------------------------------------------------------------------------

import bindings;

struct PointsPush {
    uint nx;
    uint ny;
    uint nz;
    uint comps;
    float ox;
    float oy;
    float oz;
    float inv_cell;
    uint point_count;
    uint unused0;
    uint unused1;
    uint unused2;
};

[[vk::push_constant]] ConstantBuffer<PointsPush> points_push;

struct Axis {
    uint i0;
    uint i1;
    float t;
    bool ok;
};

Axis axis_of(float p, float origin, uint n) {
    Axis a;
    a.i0 = 0u;
    a.i1 = 0u;
    a.t = 0.0;
    float g = (p - origin) * points_push.inv_cell - 0.5;
    // Finite means the exponent field is not all ones: tested on the bits, before the clamp.
    a.ok = (asuint(g) & 0x7F800000u) != 0x7F800000u;
    if (!a.ok) { return a; }
    const float top = float(n - 1u);
    if (g < 0.0) { g = 0.0; }
    if (g > top) { g = top; }
    if (n > 1u) {
        uint i0 = uint(g);
        if (i0 > n - 2u) { i0 = n - 2u; }
        a.i0 = i0;
        a.i1 = i0 + 1u;
        a.t = g - float(i0);
    }
    return a;
}

float lerp32(float a, float b, float t) { return a + (b - a) * t; }

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void grid_points(uint3 tid : SV_DispatchThreadID) {
    const uint p = tid.x;
    if (p >= points_push.point_count) { return; }
    const Axis ax = axis_of(airflow_v[p * 3u], points_push.ox, points_push.nx);
    const Axis ay = axis_of(airflow_v[p * 3u + 1u], points_push.oy, points_push.ny);
    const Axis az = axis_of(airflow_v[p * 3u + 2u], points_push.oz, points_push.nz);
    const uint nx = points_push.nx;
    const uint ny = points_push.ny;
    const uint comps = points_push.comps;
    for (uint c = 0u; c < comps; ++c) {
        if (!(ax.ok && ay.ok && az.ok)) {
            airflow_w[p * comps + c] = asfloat(0x7FC00000u);
            continue;
        }
        const float c000 = airflow_u[((az.i0 * ny + ay.i0) * nx + ax.i0) * comps + c];
        const float c100 = airflow_u[((az.i0 * ny + ay.i0) * nx + ax.i1) * comps + c];
        const float c010 = airflow_u[((az.i0 * ny + ay.i1) * nx + ax.i0) * comps + c];
        const float c110 = airflow_u[((az.i0 * ny + ay.i1) * nx + ax.i1) * comps + c];
        const float c001 = airflow_u[((az.i1 * ny + ay.i0) * nx + ax.i0) * comps + c];
        const float c101 = airflow_u[((az.i1 * ny + ay.i0) * nx + ax.i1) * comps + c];
        const float c011 = airflow_u[((az.i1 * ny + ay.i1) * nx + ax.i0) * comps + c];
        const float c111 = airflow_u[((az.i1 * ny + ay.i1) * nx + ax.i1) * comps + c];
        const float x00 = lerp32(c000, c100, ax.t);
        const float x10 = lerp32(c010, c110, ax.t);
        const float x01 = lerp32(c001, c101, ax.t);
        const float x11 = lerp32(c011, c111, ax.t);
        const float y0 = lerp32(x00, x10, ay.t);
        const float y1 = lerp32(x01, x11, ay.t);
        airflow_w[p * comps + c] = lerp32(y0, y1, az.t);
    }
}
