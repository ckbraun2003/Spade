// ---------------------------------------------------------------------------
// airflow_line_spread.slang (P4c, L61) -- the actuator line's third kernel: each element's force spread over its stencil,
// engine/physics/airflow/line.cpp's stencil_add() as a gather (its CPU twin, and the authority for every expression): each face of the
// dispatched component sums, in element order, what every element's stencil gives it, so nothing scatters and nothing is atomic. The
// CPU's scatter adds a face's contributions in the same element order, which is why the sums are bit-identical.
//
// One component per dispatch (push word 0), one invocation per face of it (1D, guarded by the push's count, word 2); word 1 is the
// element count. Each face reads its increment (airflow_increments, du, dv, dw packed like the velocity: u at 0, v at faces_u, w after),
// adds each element's contribution in element order, and writes it back. An element contributes where the face lies inside its
// stencil, in the CPU's expression: amount = force_c * (h / (rho dx^3)); skipped if S is not positive or amount is 0; coef = amount / S
// (div32_ftz); the face gets coef * (w_i * (w_j * w_k)).
//
// Cost: faces x elements, every thread of a warp reading the same record (a broadcast). P4d's tile binning of the elements is the
// optimisation for the 6144-element case; at the V2 box (about 400 elements) it is about a millisecond.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

struct LinePush {
    uint component;      // word 0: 0 u, 1 v, 2 w
    uint element_count;  // word 1
    uint count;          // word 2: the component's faces
    uint unused;         // word 3
};

[[vk::push_constant]] ConstantBuffer<LinePush> line_push;

static const uint kLineTaps = 13u;
static const uint kLineHeader = 16u;
static const uint kLineComponent = 7u + 3u * kLineTaps;
static const uint kLineStride = kLineHeader + 3u * kLineComponent;
static const uint kForceStride = 8u;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_line_spread(uint3 tid : SV_DispatchThreadID) {
    const uint index = tid.x;
    if (index >= line_push.count) { return; }
    const uint c = line_push.component;
    const uint3 dims = grid_dims();
    const uint cx = c == 0u ? dims.x + 1u : dims.x;
    const uint cy = c == 1u ? dims.y + 1u : dims.y;
    const int i = int(index % cx);
    const int j = int((index / cx) % cy);
    const int k = int(index / (cx * cy));
    const uint faces_u = (dims.x + 1u) * dims.y * dims.z;
    const uint faces_v = dims.x * (dims.y + 1u) * dims.z;
    const uint base = c == 0u ? 0u : (c == 1u ? faces_u : faces_u + faces_v);
    const uint at = base + index;
    float acc = airflow_increments[at];
    for (uint e = 0u; e < line_push.element_count; ++e) {
        const uint rec = e * kLineStride;
        const uint comp = rec + kLineHeader + c * kLineComponent;
        const int di = i - int(airflow_line_elems[comp]);
        const int dj = j - int(airflow_line_elems[comp + 1u]);
        const int dk = k - int(airflow_line_elems[comp + 2u]);
        if (di < 0 || di >= int(airflow_line_elems[comp + 3u])) { continue; }
        if (dj < 0 || dj >= int(airflow_line_elems[comp + 4u])) { continue; }
        if (dk < 0 || dk >= int(airflow_line_elems[comp + 5u])) { continue; }
        const float s = airflow_line_elems[comp + 6u];
        const float amount = airflow_line_force[e * kForceStride + c] * airflow_line_elems[rec + 12u];
        if (!(s > 0.0) || amount == 0.0) { continue; }
        const float coef = div32_ftz(amount, s);
        const float wj = airflow_line_elems[comp + 7u + kLineTaps + uint(dj)];
        const float wk = airflow_line_elems[comp + 7u + 2u * kLineTaps + uint(dk)];
        const float wjk = wj * wk;
        acc += coef * (airflow_line_elems[comp + 7u + uint(di)] * wjk);
    }
    airflow_increments[at] = acc;
}
