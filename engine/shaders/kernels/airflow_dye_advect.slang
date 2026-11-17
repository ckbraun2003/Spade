// ---------------------------------------------------------------------------
// airflow_dye_advect.slang (P10) -- the passive dye's advection on the GPU: engine/physics/airflow/advect.cpp's
// advect_scalar(), term for term (its CPU twin and the authority for every expression). A cell-centred scalar, one
// invocation per CELL: an RK2 trace of the cell centre through the face velocity, semi-Lagrangian, and the MacCormack
// correction clamped to the departure stencil's range, so a new value never leaves [min, max] of the eight cells it
// interpolated.
//
// Two passes, as airflow_advection.slang's:
//   pass 0: the forward estimate at the back-traced point and its stencil's range, into airflow_advect (hat at 0, lo
//           at cells, hi at 2 cells: the step's scratch, dead by the time the dye chain runs);
//   pass 1: the forward estimate sampled at the forward-traced point, and hat + 1/2 (d - d~) clamped to [lo, hi], into
//           the dye buffer's NEXT half.
// airflow_dye (bindings.slang, binding 48): [0, 4) the release's header (airflow_dye_source.slang), then the cells' dye
// (cur), then the same count again (next). airflow_dye_source then adds the release to next and writes cur.
//
// Reads airflow_u, airflow_v, airflow_w, airflow_params and airflow_dye's cur; writes airflow_advect and airflow_dye's
// next. The velocity sampling is airflow_advection.slang's, copied (the kernels are separate modules): the same
// expressions in the same order, so the trace is the CPU's to the bit.
//
// PARITY. -fp-mode precise on every kernel compile (cmake/SpadeSlang.cmake); 1/dx from the host; lo and hi are
// comparisons, not min() and max(); the periodic wrap's f / n is fp32_math's div32_ftz (divss under the CPU's FTZ|DAZ).
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

struct DyeAdvectPush {
    uint unused0;
    uint pass;        // 0: the forward estimate; 1: the backward one and the clamp
    uint cell_count;
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<DyeAdvectPush> dye_push;

static const uint kDyeHeader = 4u;  // airflow_dye's header floats (the source kernel's release parameters)

uint axis_cells(uint a) {
    const uint3 n = grid_dims();
    return a == 0u ? n.x : (a == 1u ? n.y : n.z);
}

uint samples_x(uint c) { return c == 0u ? grid_dims().x + 1u : grid_dims().x; }
uint samples_y(uint c) { return c == 1u ? grid_dims().y + 1u : grid_dims().y; }

struct Axis {
    uint i0;
    uint i1;
    float t;
};

Axis axis(uint i0, uint i1, float t) {
    Axis out;
    out.i0 = i0;
    out.i1 = i1;
    out.t = t;
    return out;
}

Axis locate(float f, uint n, uint count, bool periodic) {
    if (periodic) {
        if (n == 1u) return axis(0u, 0u, 0.0);
        const float nf = float(n);
        float w = f - nf * floor(div32_ftz(f, nf));
        if (!(w >= 0.0) || !(w < nf)) w = 0.0;
        const uint i0 = uint(w);
        return axis(i0, i0 + 1u == n ? 0u : i0 + 1u, w - float(i0));
    }
    if (count == 1u) return axis(0u, 0u, 0.0);
    const float top = float(count - 1u);
    const float c = f > top ? top : (f >= 0.0 ? f : 0.0);
    uint i0 = uint(c);
    if (i0 + 1u >= count) i0 = count - 2u;
    return axis(i0, i0 + 1u, c - float(i0));
}

float lerp_form(float a, float b, float t) { return a + t * (b - a); }

Axis locate_for(float3 x, uint a, uint c) {
    const bool own = a == c;
    const float f = x[a] * grid_inv_dx() - (own ? 0.0 : 0.5);
    const uint n = axis_cells(a);
    return locate(f, n, own ? n + 1u : n, axis_periodic(a));
}

struct Sample {
    float value;
    float lo;
    float hi;
};

Sample interpolate(float a000, float a100, float a010, float a110, float a001, float a101, float a011, float a111,
                   Axis x0, Axis x1, Axis x2) {
    const float c0 = lerp_form(lerp_form(a000, a100, x0.t), lerp_form(a010, a110, x0.t), x1.t);
    const float c1 = lerp_form(lerp_form(a001, a101, x0.t), lerp_form(a011, a111, x0.t), x1.t);
    Sample out;
    out.value = lerp_form(c0, c1, x2.t);
    float lo = a000;
    float hi = a000;
    lo = a100 < lo ? a100 : lo; hi = hi < a100 ? a100 : hi;
    lo = a010 < lo ? a010 : lo; hi = hi < a010 ? a010 : hi;
    lo = a110 < lo ? a110 : lo; hi = hi < a110 ? a110 : hi;
    lo = a001 < lo ? a001 : lo; hi = hi < a001 ? a001 : hi;
    lo = a101 < lo ? a101 : lo; hi = hi < a101 ? a101 : hi;
    lo = a011 < lo ? a011 : lo; hi = hi < a011 ? a011 : hi;
    lo = a111 < lo ? a111 : lo; hi = hi < a111 ? a111 : hi;
    out.lo = lo;
    out.hi = hi;
    return out;
}

float velocity_load<let C : uint>(uint index) {
    if (C == 0u) return airflow_u[index];
    if (C == 1u) return airflow_v[index];
    return airflow_w[index];
}

Sample sample_velocity<let C : uint>(Axis x0, Axis x1, Axis x2) {
    const uint sx = samples_x(C);
    const uint sy = samples_y(C);
    return interpolate(velocity_load<C>(x0.i0 + sx * (x1.i0 + sy * x2.i0)),
                       velocity_load<C>(x0.i1 + sx * (x1.i0 + sy * x2.i0)),
                       velocity_load<C>(x0.i0 + sx * (x1.i1 + sy * x2.i0)),
                       velocity_load<C>(x0.i1 + sx * (x1.i1 + sy * x2.i0)),
                       velocity_load<C>(x0.i0 + sx * (x1.i0 + sy * x2.i1)),
                       velocity_load<C>(x0.i1 + sx * (x1.i0 + sy * x2.i1)),
                       velocity_load<C>(x0.i0 + sx * (x1.i1 + sy * x2.i1)),
                       velocity_load<C>(x0.i1 + sx * (x1.i1 + sy * x2.i1)), x0, x1, x2);
}

float3 velocity_at(float3 x) {
    const Axis face0 = locate_for(x, 0u, 0u);
    const Axis face1 = locate_for(x, 1u, 1u);
    const Axis face2 = locate_for(x, 2u, 2u);
    const Axis cent0 = locate_for(x, 0u, 1u);
    const Axis cent1 = locate_for(x, 1u, 0u);
    const Axis cent2 = locate_for(x, 2u, 0u);
    return float3(sample_velocity<0>(face0, cent1, cent2).value, sample_velocity<1>(cent0, face1, cent2).value,
                  sample_velocity<2>(cent0, cent1, face2).value);
}

float3 trace_rk2(float3 x, float sign) {
    const float h = step_h();
    const float half_h = 0.5 * h;
    const float3 mid = x + (sign * half_h) * velocity_at(x);
    return x + (sign * h) * velocity_at(mid);
}

// The cell centres' axes: advect.cpp's sample_centred(), every axis against the centres, so n samples each.
Axis centred_axis(float3 x, uint a) {
    return locate(x[a] * grid_inv_dx() - 0.5, axis_cells(a), axis_cells(a), axis_periodic(a));
}

// A cell-centred field's trilinear sample, the field read through `Field`: the dye's cur (offset kDyeHeader of
// airflow_dye) or the pass-0 estimate (airflow_advect at 0). The strides are component 2's (nx, ny).
Sample sample_dye(Axis x0, Axis x1, Axis x2) {
    const uint sx = grid_dims().x;
    const uint sy = grid_dims().y;
    const uint base = kDyeHeader;
    return interpolate(airflow_dye[base + x0.i0 + sx * (x1.i0 + sy * x2.i0)],
                       airflow_dye[base + x0.i1 + sx * (x1.i0 + sy * x2.i0)],
                       airflow_dye[base + x0.i0 + sx * (x1.i1 + sy * x2.i0)],
                       airflow_dye[base + x0.i1 + sx * (x1.i1 + sy * x2.i0)],
                       airflow_dye[base + x0.i0 + sx * (x1.i0 + sy * x2.i1)],
                       airflow_dye[base + x0.i1 + sx * (x1.i0 + sy * x2.i1)],
                       airflow_dye[base + x0.i0 + sx * (x1.i1 + sy * x2.i1)],
                       airflow_dye[base + x0.i1 + sx * (x1.i1 + sy * x2.i1)], x0, x1, x2);
}

Sample sample_hat(Axis x0, Axis x1, Axis x2) {
    const uint sx = grid_dims().x;
    const uint sy = grid_dims().y;
    return interpolate(airflow_advect[x0.i0 + sx * (x1.i0 + sy * x2.i0)],
                       airflow_advect[x0.i1 + sx * (x1.i0 + sy * x2.i0)],
                       airflow_advect[x0.i0 + sx * (x1.i1 + sy * x2.i0)],
                       airflow_advect[x0.i1 + sx * (x1.i1 + sy * x2.i0)],
                       airflow_advect[x0.i0 + sx * (x1.i0 + sy * x2.i1)],
                       airflow_advect[x0.i1 + sx * (x1.i0 + sy * x2.i1)],
                       airflow_advect[x0.i0 + sx * (x1.i1 + sy * x2.i1)],
                       airflow_advect[x0.i1 + sx * (x1.i1 + sy * x2.i1)], x0, x1, x2);
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_dye_advect(uint3 tid : SV_DispatchThreadID) {
    const uint index = tid.x;
    const uint cells = dye_push.cell_count;
    if (index >= cells) { return; }
    const uint3 n = grid_dims();
    const uint i = index % n.x;
    const uint j = (index / n.x) % n.y;
    const uint k = index / (n.x * n.y);
    const float dx = grid_dx();
    const float3 x = float3((float(i) + 0.5) * dx, (float(j) + 0.5) * dx, (float(k) + 0.5) * dx);
    if (dye_push.pass == 0u) {
        const float3 traced = trace_rk2(x, -1.0);
        const Sample f = sample_dye(centred_axis(traced, 0u), centred_axis(traced, 1u), centred_axis(traced, 2u));
        airflow_advect[index] = f.value;
        airflow_advect[cells + index] = f.lo;
        airflow_advect[2u * cells + index] = f.hi;
    } else {
        const float3 traced = trace_rk2(x, 1.0);
        const float back =
            sample_hat(centred_axis(traced, 0u), centred_axis(traced, 1u), centred_axis(traced, 2u)).value;
        const float hat = airflow_advect[index];
        float v = hat + 0.5 * (airflow_dye[kDyeHeader + index] - back);
        const float lo = airflow_advect[cells + index];
        const float hi = airflow_advect[2u * cells + index];
        v = v < lo ? lo : (v > hi ? hi : v);
        airflow_dye[kDyeHeader + cells + index] = v;
    }
}
