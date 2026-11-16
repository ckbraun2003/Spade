// ---------------------------------------------------------------------------
// airflow_line_sample.slang (P4c, L61) -- the actuator line's first kernel: the air each element feels, engine/physics/airflow/line.cpp's
// stencil_mean() term for term (its CPU twin, and the authority for every expression).
//
// One invocation per (element, component): t = element * 3 + component, guarded by the push's count (word 2, 3 * the element count).
// The host packed the stencil (airflow_line_elems: lo, n and the weights per component and axis, and S, the weights' sum, from
// line_stencil()), so this is a weighted sum over it in the CPU's order, k, j, i, then / S by div32_ftz (the exact division). The
// velocity read is the step's start: GpuFluid dispatches the line's sample before the advection.
//
// Writes airflow_line_vel: four floats an element, the air's x, y, z and a pad. A component whose S is not positive reads 0, as the
// CPU's (the stencil left the grid or all its weights flushed).
//
// RECORD (spike/airflow/line_chain.hpp): kLineStride floats an element; header kLineHeader = 16 floats; then per component
// kLineComponent = 7 + 3 * kLineTaps floats: lo of the three axes, n of the three, S, the weights of each axis (kLineTaps each).
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

struct LinePush {
    uint element_count;  // word 0
    uint unused;         // word 1
    uint count;          // word 2: the invocations
    uint unused2;        // word 3
};

[[vk::push_constant]] ConstantBuffer<LinePush> line_push;

static const uint kLineTaps = 13u;
static const uint kLineHeader = 16u;
static const uint kLineComponent = 7u + 3u * kLineTaps;
static const uint kLineStride = kLineHeader + 3u * kLineComponent;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_line_sample(uint3 tid : SV_DispatchThreadID) {
    const uint t = tid.x;
    if (t >= line_push.count) { return; }
    const uint e = t / 3u;
    const uint c = t - e * 3u;
    const uint comp = e * kLineStride + kLineHeader + c * kLineComponent;
    const float s = airflow_line_elems[comp + 6u];
    float mean = 0.0;
    if (s > 0.0) {
        const uint3 dims = grid_dims();
        const uint cx = c == 0u ? dims.x + 1u : dims.x;
        const uint cy = c == 1u ? dims.y + 1u : dims.y;
        const uint lo0 = uint(airflow_line_elems[comp]);
        const uint lo1 = uint(airflow_line_elems[comp + 1u]);
        const uint lo2 = uint(airflow_line_elems[comp + 2u]);
        const uint n0 = uint(airflow_line_elems[comp + 3u]);
        const uint n1 = uint(airflow_line_elems[comp + 4u]);
        const uint n2 = uint(airflow_line_elems[comp + 5u]);
        const uint w0 = comp + 7u;
        const uint w1 = comp + 7u + kLineTaps;
        const uint w2 = comp + 7u + 2u * kLineTaps;
        float acc = 0.0;
        for (uint k = 0u; k < n2; ++k) {
            for (uint j = 0u; j < n1; ++j) {
                const float wjk = airflow_line_elems[w1 + j] * airflow_line_elems[w2 + k];
                const uint row = (lo0 + cx * (lo1 + j + cy * (lo2 + k)));
                for (uint i = 0u; i < n0; ++i) {
                    const uint f = row + i;
                    const float v = c == 0u ? airflow_u[f] : (c == 1u ? airflow_v[f] : airflow_w[f]);
                    acc += airflow_line_elems[w0 + i] * wjk * v;
                }
            }
        }
        mean = div32_ftz(acc, s);
    }
    airflow_line_vel[e * 4u + c] = mean;
    if (c == 0u) { airflow_line_vel[e * 4u + 3u] = 0.0; }
}
