// ---------------------------------------------------------------------------
// airflow_dye_tip.slang (L86i) -- the dye's release at a line's blade tips, on the GPU: engine/physics/airflow/dye.cpp's
// emit_tip_dye(), term for term (its CPU twin and the authority for every expression). One invocation per CELL: it takes the cell's
// dye (airflow_dye's cur half, written by airflow_dye_source this step) and adds every release point's, in the points' order,
//   d <- d + (1 - d) inc,   inc = min(1, gain w),   w = (phi4(ax) phi4(ay)) phi4(az),   a = |cell centre - point| / dx per axis < 2
// only where inc > 0, and writes it back to the cur half. The CPU visits point by point, each cell in order; a cell's point sequence is
// the same, so the bits are.
//
// THE POINTS are the host's (tip_points(), the CPU's own floats), in the line elements' buffer (airflow_line_elems) after what the
// line's step packs there: the element records (the push's word 0, E, of them, kLineStride floats each), then, when the tip correction
// is on (the push's word 1), its header and groups (word 0 of the header the group count, a float, exact; kTipHeaderWords + groups *
// kTipStride floats), then [0] the point count P (a float, exact) and P records of four floats, the point's x, y, z and a spare. The
// gain is airflow_dye's header word 3. The kernel is dispatched only on a step that has lines (GpuFluid::step(discs, lines)).
//
// PARITY. -fp-mode precise on every kernel compile; sqrt is fp32_math's sqrt32_ftz (sqrtss under the CPU's FTZ|DAZ).
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

struct DyeTipPush {
    uint elements;
    uint tip_on;
    uint cell_count;
    uint unused;
};

[[vk::push_constant]] ConstantBuffer<DyeTipPush> dye_tip_push;

static const uint kDyeHeader = 4u;
// line_chain.hpp's kLineStride (16 + 3 * (7 + 3 * 13)), kTipHeaderWords and kTipStride: the line kernels' own layout, which the line
// parity tests pin and this kernel's parity test reads the points through.
static const uint kLineStride = 154u;
static const uint kTipHeaderWords = 16u;
static const uint kTipStride = 160u;

// dye.cpp's phi4(): Peskin's 4-point kernel at a cells from the point, a in [0, 2).
float phi4(float a) {
    if (a < 1.0) {
        return (3.0 - 2.0 * a + sqrt32_ftz(1.0 + 4.0 * a - 4.0 * a * a)) * 0.125;
    }
    return (5.0 - 2.0 * a - sqrt32_ftz(-7.0 + 12.0 * a - 4.0 * a * a)) * 0.125;
}

[[shader("compute")]]
[numthreads(SPADE_WG, 1, 1)]
void airflow_dye_tip(uint3 tid : SV_DispatchThreadID) {
    const uint index = tid.x;
    const uint cells = dye_tip_push.cell_count;
    if (index >= cells) { return; }
    const uint3 n = grid_dims();
    const uint i = index % n.x;
    const uint j = (index / n.x) % n.y;
    const uint k = index / (n.x * n.y);
    const float dx = grid_dx();
    const float inv_dx = grid_inv_dx();
    const float3 p = float3((float(i) + 0.5) * dx, (float(j) + 0.5) * dx, (float(k) + 0.5) * dx);
    const float gain = airflow_dye[3];

    uint at = dye_tip_push.elements * kLineStride;
    if (dye_tip_push.tip_on != 0u) {
        const uint groups = uint(airflow_line_elems[at]);
        at = at + kTipHeaderWords + groups * kTipStride;
    }
    const uint points = uint(airflow_line_elems[at]);

    float d = airflow_dye[kDyeHeader + index];
    for (uint m = 0u; m < points; ++m) {
        const uint q = at + 1u + 4u * m;
        const float az = abs(p.z - airflow_line_elems[q + 2u]) * inv_dx;
        if (!(az < 2.0)) { continue; }
        const float ay = abs(p.y - airflow_line_elems[q + 1u]) * inv_dx;
        if (!(ay < 2.0)) { continue; }
        const float ax = abs(p.x - airflow_line_elems[q]) * inv_dx;
        if (!(ax < 2.0)) { continue; }
        const float w = (phi4(ax) * phi4(ay)) * phi4(az);
        float inc = gain * w;
        inc = inc > 1.0 ? 1.0 : inc;
        if (inc > 0.0) { d = d + (1.0 - d) * inc; }
    }
    airflow_dye[kDyeHeader + index] = d;
}
