// ---------------------------------------------------------------------------
// airflow_advection.slang (plan B, task B3) -- the fluid step's advection on
// the GPU: engine/physics/airflow/advect.cpp, term for term (its CPU twin and
// the authority for every expression). Semi-Lagrangian, an RK2 trace, and the
// MacCormack correction clamped to the forward stencil's range. (Not
// airflow_advect.slang: bindings.slang's set 1 already names a buffer that.)
//
// One component per dispatch, one invocation per face of it (1D, guarded by
// the push's face_count), in two passes:
//   pass 0, advect():  the forward estimate of component c at the face's
//                      back-traced point, and its stencil's range, into
//                      airflow_advect (hat, lo, hi);
//   pass 1, advect_back() then maccormack_clamp(): the forward estimate
//                      sampled at the face's forward-traced point (phi~), and
//                      hat + 1/2 (phi - phi~) clamped to [lo, hi], into
//                      airflow_next.
// Pass 1 of any component runs after pass 0 of all three (the chain's
// barrier): it reads every component's velocity, and hat of its own.
//
// Reads airflow_u, airflow_v, airflow_w and airflow_params; writes
// airflow_advect and airflow_next (set 1, bindings.slang section F). No
// solids, no face tags: fluid_step applies the boundaries and the solids'
// velocities after advection (Physics, 2026-10-21).
//
// PACKING. airflow_advect holds hat for u, v and w, then lo for the three,
// then hi, each block the three components' faces in order; the blocks start
// at airflow_params' offsets_advect. airflow_next holds the new u, v and w in
// the same order (ASSUMED, to confirm with Physics: their note said "writes
// airflow_next for that component").
//
// PARITY. -fp-mode precise on every kernel compile keeps the arithmetic
// uncontracted (cmake/SpadeSlang.cmake); 1/dx comes from the host. The two
// traces are one function: x + (s h/2) v is x - (h/2) v to the bit for
// s = -1, since negation is exact. lo and hi are std::min's and std::max's
// comparisons, not min() and max(), which differ on NaN and signed zero. The
// periodic wrap's f / n is a division, which SPIR-V does not round
// correctly, so like every kernel here this is banded against its twin, not
// bit-equal.
// ---------------------------------------------------------------------------

import layouts;
import bindings;

struct AdvectionPush {
    uint component;   // 0 u, 1 v, 2 w
    uint pass;        // 0: the forward estimate; 1: the backward one and the clamp
    uint face_count;  // the component's faces
    uint unused;
};

[[vk::push_constant]] ConstantBuffer<AdvectionPush> advection_push;

// --- airflow_params, as Physics' draft lays it out (uint words; a float by
// asfloat). Physics freezes it as a shared struct when B1 resumes; these
// accessors are the one place that changes then.
//   uint4  dims           {nx, ny, nz, 0}
//   float4 grid           {dx, inv_dx, h, 0}
//   uint4  periodic       {px, py, pz, 0}
//   uint4  offsets_advect {hat, lo, hi, faces_total}, in floats
uint3 grid_dims() { return uint3(airflow_params[0], airflow_params[1], airflow_params[2]); }
float grid_dx() { return asfloat(airflow_params[4]); }
float grid_inv_dx() { return asfloat(airflow_params[5]); }
float step_h() { return asfloat(airflow_params[6]); }
bool axis_periodic(uint a) { return airflow_params[8 + a] != 0u; }
uint offset_hat() { return airflow_params[12]; }
uint offset_lo() { return airflow_params[13]; }
uint offset_hi() { return airflow_params[14]; }

// --- the grid's sampling geometry (advect.cpp's Geometry) ------------------

uint axis_cells(uint a) {
    const uint3 n = grid_dims();
    return a == 0u ? n.x : (a == 1u ? n.y : n.z);
}

// Component c's samples along x and y (its own axis holds n + 1).
uint samples_x(uint c) { return c == 0u ? grid_dims().x + 1u : grid_dims().x; }
uint samples_y(uint c) { return c == 1u ? grid_dims().y + 1u : grid_dims().y; }

// Where component c's faces start in a three-component block.
uint component_base(uint c) {
    const uint3 n = grid_dims();
    const uint faces_u = (n.x + 1u) * n.y * n.z;
    const uint faces_v = n.x * (n.y + 1u) * n.z;
    return c == 0u ? 0u : (c == 1u ? faces_u : faces_u + faces_v);
}

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

// A fractional sample coordinate f along an axis of n cells holding `count`
// samples (n + 1 on the component's own axis, n on the others).
Axis locate(float f, uint n, uint count, bool periodic) {
    if (periodic) {
        if (n == 1u) return axis(0u, 0u, 0.0);
        const float nf = float(n);
        float w = f - nf * floor(f / nf);
        if (!(w >= 0.0) || !(w < nf)) w = 0.0;  // NaN, or rounding onto the period's end
        const uint i0 = uint(w);
        return axis(i0, i0 + 1u == n ? 0u : i0 + 1u, w - float(i0));
    }
    if (count == 1u) return axis(0u, 0u, 0.0);
    const float top = float(count - 1u);
    const float c = f > top ? top : (f >= 0.0 ? f : 0.0);  // NaN clamps to 0
    uint i0 = uint(c);
    if (i0 + 1u >= count) i0 = count - 2u;
    return axis(i0, i0 + 1u, c - float(i0));
}

float lerp_form(float a, float b, float t) { return a + t * (b - a); }

// The coordinate along axis a, located for component c.
Axis locate_for(float3 x, uint a, uint c) {
    const bool own = a == c;
    const float f = x[a] * grid_inv_dx() - (own ? 0.0 : 0.5);
    const uint n = axis_cells(a);
    return locate(f, n, own ? n + 1u : n, axis_periodic(a));
}

struct Sample {
    float value;
    float lo;  // the smallest of the eight values interpolated
    float hi;  // the largest
};

// The eight values' trilinear mix, x then y then z in the lerp form, and
// their range in advect.cpp's order (std::min(lo, v) is v < lo ? v : lo).
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

// Velocity component C, by face index: C is a compile-time constant, so each
// call reads one buffer with no branch.
float velocity_load<let C : uint>(uint index) {
    if (C == 0u) return airflow_u[index];
    if (C == 1u) return airflow_v[index];
    return airflow_w[index];
}

// Velocity component C's trilinear sample on located axes.
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

// Component c's forward estimate's trilinear sample on located axes (one
// buffer for the three, so c may vary).
Sample sample_hat(uint c, Axis x0, Axis x1, Axis x2) {
    const uint sx = samples_x(c);
    const uint sy = samples_y(c);
    const uint base = offset_hat() + component_base(c);
    return interpolate(airflow_advect[base + x0.i0 + sx * (x1.i0 + sy * x2.i0)],
                       airflow_advect[base + x0.i1 + sx * (x1.i0 + sy * x2.i0)],
                       airflow_advect[base + x0.i0 + sx * (x1.i1 + sy * x2.i0)],
                       airflow_advect[base + x0.i1 + sx * (x1.i1 + sy * x2.i0)],
                       airflow_advect[base + x0.i0 + sx * (x1.i0 + sy * x2.i1)],
                       airflow_advect[base + x0.i1 + sx * (x1.i0 + sy * x2.i1)],
                       airflow_advect[base + x0.i0 + sx * (x1.i1 + sy * x2.i1)],
                       airflow_advect[base + x0.i1 + sx * (x1.i1 + sy * x2.i1)], x0, x1, x2);
}

// The velocity at x: each axis located once against its faces and once
// against the centres, each component taking its own mix (Velocity::at).
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

// The RK2 trace from x over h, backward (sign -1: advect.cpp's back_trace)
// or forward (+1: forward_trace). Nothing is stored, as the CPU's
// AdvectScratch keeps no departure points.
float3 trace_rk2(float3 x, float sign) {
    const float h = step_h();
    const float half_h = 0.5 * h;
    const float3 mid = x + (sign * half_h) * velocity_at(x);
    return x + (sign * h) * velocity_at(mid);
}

// Face `index` of component c: its point in the grid's frame
// (for_each_sample's expression).
float3 face_point(uint c, uint index) {
    const uint cx = samples_x(c);
    const uint cy = samples_y(c);
    const uint i = index % cx;
    const uint j = (index / cx) % cy;
    const uint k = index / (cx * cy);
    const float ox = c == 0u ? 0.0 : 0.5;
    const float oy = c == 1u ? 0.0 : 0.5;
    const float oz = c == 2u ? 0.0 : 0.5;
    const float dx = grid_dx();
    return float3((float(i) + ox) * dx, (float(j) + oy) * dx, (float(k) + oz) * dx);
}

float velocity_of(uint c, uint index) {
    if (c == 0u) return airflow_u[index];
    if (c == 1u) return airflow_v[index];
    return airflow_w[index];
}

// Pass 0: the forward semi-Lagrangian estimate of component c at the
// back-traced point, and its stencil's range.
void advect(uint c, uint index, float3 traced) {
    const Axis x0 = locate_for(traced, 0u, c);
    const Axis x1 = locate_for(traced, 1u, c);
    const Axis x2 = locate_for(traced, 2u, c);
    Sample f;
    if (c == 0u) {
        f = sample_velocity<0>(x0, x1, x2);
    } else if (c == 1u) {
        f = sample_velocity<1>(x0, x1, x2);
    } else {
        f = sample_velocity<2>(x0, x1, x2);
    }
    const uint at = component_base(c) + index;
    airflow_advect[offset_hat() + at] = f.value;
    airflow_advect[offset_lo() + at] = f.lo;
    airflow_advect[offset_hi() + at] = f.hi;
}

// Pass 1, first half: phi~, the forward estimate at the forward-traced point.
float advect_back(uint c, float3 traced) {
    return sample_hat(c, locate_for(traced, 0u, c), locate_for(traced, 1u, c), locate_for(traced, 2u, c)).value;
}

// Pass 1, second half: hat + 1/2 (phi - phi~), clamped to the forward
// stencil's range.
void maccormack_clamp(uint c, uint index, float back) {
    const uint at = component_base(c) + index;
    const float hat = airflow_advect[offset_hat() + at];
    const float lo = airflow_advect[offset_lo() + at];
    const float hi = airflow_advect[offset_hi() + at];
    float v = hat + 0.5 * (velocity_of(c, index) - back);
    v = v < lo ? lo : (v > hi ? hi : v);
    airflow_next[at] = v;
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_advection(uint3 tid : SV_DispatchThreadID) {
    const uint index = tid.x;
    if (index >= advection_push.face_count) { return; }
    const uint c = advection_push.component;
    const bool forward = advection_push.pass == 0u;
    const float3 traced = trace_rk2(face_point(c, index), forward ? -1.0 : 1.0);
    if (forward) {
        advect(c, index, traced);
    } else {
        maccormack_clamp(c, index, advect_back(c, traced));
    }
}
