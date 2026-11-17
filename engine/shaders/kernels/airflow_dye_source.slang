// ---------------------------------------------------------------------------
// airflow_dye_source.slang (P10) -- the dye's release at the actuator discs, on the GPU: engine/physics/airflow/dye.cpp's
// emit_dye() with dye_increment(), term for term (its CPU twin and the authority for every expression). One invocation per
// CELL: it takes the advected dye (airflow_dye's next half), adds every disc's release in the disc's order,
//   d <- d + (1 - d) inc,   inc = dye_increment(disc, cell centre, 1/dx, k, r_in, r_out), only where inc > 0,
// and writes the result to the cur half: this kernel is also the step's copy of next to cur. The CPU visits disc by
// disc, each cell in order; a cell's disc sequence is the same, so the bits are.
//
// The discs are airflow_discs' records (step_chain.hpp): [0-2] the centre, [4-6] the axis, [11] the radius (the
// gather kernel reads neither word 11 nor anything past 10 but the points). The release's parameters are airflow_dye's
// header: [0] k = rate * h, [1] r_in, [2] r_out, formed on the host as the CPU forms them. The disc count is the push's
// word 0. A cell outside every disc's bounding cube (emit_dye()'s: R_out + 2 dx about the centre) has inc = 0 for it, so
// skipping it is exact.
//
// PARITY. -fp-mode precise on every kernel compile; sqrt is fp32_math's sqrt32_ftz (sqrtss under the CPU's FTZ|DAZ).
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

struct DyeSourcePush {
    uint disc_count;
    uint unused0;
    uint cell_count;
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<DyeSourcePush> dye_source_push;

static const uint kDyeHeader = 4u;
// step_chain.hpp's kDiscStride, the gather kernel's: 16 + 6 * 640 (kDiscPoints). The gather parity tests pin the layout, and the
// dye's parity test reads the radius word through this stride.
static const uint kDiscStride = 16u + 6u * 640u;

float ramp(float v, float inv_dx) {
    const float x = v * inv_dx + 0.5;
    return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x);
}

// dye.cpp's dye_increment().
float dye_increment(float3 centre, float3 axis, float radius, float3 p, float inv_dx, float k, float r_in, float r_out) {
    const float rx = p.x - centre.x;
    const float ry = p.y - centre.y;
    const float rz = p.z - centre.z;
    const float along = (rx * axis.x + ry * axis.y) + rz * axis.z;
    if (!(along >= 0.0)) return 0.0;
    const float a = along * inv_dx;
    if (!(a < 2.0)) return 0.0;
    float phi;
    if (a < 1.0) {
        phi = (3.0 - 2.0 * a + sqrt32_ftz(1.0 + 4.0 * a - 4.0 * a * a)) * 0.125;
    } else {
        phi = (5.0 - 2.0 * a - sqrt32_ftz(-7.0 + 12.0 * a - 4.0 * a * a)) * 0.125;
    }
    const float qx = rx - along * axis.x;
    const float qy = ry - along * axis.y;
    const float qz = rz - along * axis.z;
    const float r = sqrt32_ftz((qx * qx + qy * qy) + qz * qz);
    float w = ramp(r_out * radius - r, inv_dx);
    if (r_in > 0.0) w = w * ramp(r - r_in * radius, inv_dx);
    const float inc = (k * (2.0 * phi)) * w;
    return inc > 1.0 ? 1.0 : inc;
}

// emit_dye()'s cube about a disc along one axis, as the cell range [lo, hi) it visits: true when cell index c is inside.
bool in_cube(uint c, uint n, float centre, float reach, float inv_dx) {
    const float flo = floor((centre - reach) * inv_dx);
    const float fhi = ceil((centre + reach) * inv_dx);
    const uint lo = uint(0.0 < flo ? flo : 0.0);
    uint hi = uint(0.0 < fhi ? fhi : 0.0) + 1u;
    hi = hi < n ? hi : n;
    return c >= lo && c < hi;
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_dye_source(uint3 tid : SV_DispatchThreadID) {
    const uint index = tid.x;
    const uint cells = dye_source_push.cell_count;
    if (index >= cells) { return; }
    const uint3 n = grid_dims();
    const uint i = index % n.x;
    const uint j = (index / n.x) % n.y;
    const uint k = index / (n.x * n.y);
    const float dx = grid_dx();
    const float inv_dx = grid_inv_dx();
    const float3 p = float3((float(i) + 0.5) * dx, (float(j) + 0.5) * dx, (float(k) + 0.5) * dx);
    const float release_k = airflow_dye[0];
    const float r_in = airflow_dye[1];
    const float r_out = airflow_dye[2];
    float d = airflow_dye[kDyeHeader + cells + index];
    for (uint m = 0u; m < dye_source_push.disc_count; ++m) {
        const uint at = m * kDiscStride;
        const float3 centre = float3(airflow_discs[at], airflow_discs[at + 1u], airflow_discs[at + 2u]);
        const float3 axis = float3(airflow_discs[at + 4u], airflow_discs[at + 5u], airflow_discs[at + 6u]);
        const float radius = airflow_discs[at + 11u];
        const float reach = r_out * radius + 2.0 * dx;
        if (!in_cube(i, n.x, centre.x, reach, inv_dx) || !in_cube(j, n.y, centre.y, reach, inv_dx) ||
            !in_cube(k, n.z, centre.z, reach, inv_dx)) {
            continue;
        }
        const float inc = dye_increment(centre, axis, radius, p, inv_dx, release_k, r_in, r_out);
        if (inc > 0.0) d = d + (1.0 - d) * inc;
    }
    airflow_dye[kDyeHeader + index] = d;
}
