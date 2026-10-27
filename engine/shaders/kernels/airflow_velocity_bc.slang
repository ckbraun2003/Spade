// ---------------------------------------------------------------------------
// airflow_velocity_bc.slang (plan B, task B3) -- the domain's velocity
// conditions on the GPU: engine/physics/airflow/boundaries.cpp's
// apply_velocity_boundaries(), term for term (its CPU twin and the authority
// for every expression).
//
// One axis per dispatch, one invocation per line along it (the plane across
// the axis, 1D, guarded by the push's count). Only that axis' component's
// normal faces change: face 0 and face n of the line. The line's two ends are
// one invocation in apply_axis()'s order, low then high, so a one-cell axis,
// whose high end copies the low end's new value, gives the CPU's result.
//
// Reads airflow_params (the dims, the face kinds and the face block: a side's
// fixed normal velocity is face_velocity(side, axis), the host's
// fixed_normal_velocity(), and the ambient's component ambient_velocity(axis));
// reads and writes airflow_u, airflow_v or airflow_w (set 1, bindings.slang
// section F).
//
// PARITY. Copies, selects, host values and the outlets' two sign tests: no
// arithmetic, so the kernel is apply_velocity_boundaries()'s bit for bit where
// the device reads a subnormal in a comparison as the CPU step's DAZ does (the
// device of record; a denormal-keeping device refuses a subnormal outflow the
// CPU passes), and the module is scanned under the exact profile.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;

// The airflow push convention: 16 bytes, the count in word 2.
struct VelocityBcPush {
    uint axis;   // 0 x (u), 1 y (v), 2 z (w)
    uint unused1;
    uint count;  // the lines: the plane across the axis
    uint unused3;
};

[[vk::push_constant]] ConstantBuffer<VelocityBcPush> velocity_bc_push;

// Component `axis`' sample q along its own axis on the line (a, b) across it,
// as apply_velocity_boundaries()' at(q, a, b) addresses it.
uint sample_index(uint axis, uint q, uint a, uint b) {
    const uint3 n = grid_dims();
    if (axis == 0u) return q + (n.x + 1u) * (a + n.y * b);  // u_index(q, a, b)
    if (axis == 1u) return a + n.x * (q + (n.y + 1u) * b);  // v_index(a, q, b)
    return a + n.x * (b + n.y * q);                         // w_index(a, b, q)
}

float load(uint axis, uint idx) {
    if (axis == 0u) return airflow_u[idx];
    if (axis == 1u) return airflow_v[idx];
    return airflow_w[idx];
}

void store(uint axis, uint idx, float value) {
    if (axis == 0u) {
        airflow_u[idx] = value;
    } else if (axis == 1u) {
        airflow_v[idx] = value;
    } else {
        airflow_w[idx] = value;
    }
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_velocity_bc(uint3 tid : SV_DispatchThreadID) {
    const uint t = tid.x;
    if (t >= velocity_bc_push.count) { return; }
    const uint axis = velocity_bc_push.axis;
    const uint3 dims = grid_dims();
    const uint n = axis == 0u ? dims.x : (axis == 1u ? dims.y : dims.z);
    const uint m1 = axis == 0u ? dims.y : dims.x;  // apply_axis' a, then b
    const uint a = t % m1;
    const uint b = t / m1;
    const uint klo = face_kind(2u * axis);
    const uint khi = face_kind(2u * axis + 1u);
    const float amb = ambient_velocity(axis);
    if (klo == kFacePeriodic) {
        store(axis, sample_index(axis, n, a, b), load(axis, sample_index(axis, 0u, a, b)));
        return;
    }
    if (klo == kFaceFixedVelocity) {
        store(axis, sample_index(axis, 0u, a, b), face_velocity(2u * axis, axis));
    } else {
        float copy = load(axis, sample_index(axis, 1u, a, b));
        if (copy > 0.0) copy = amb;  // into the box through the minus face: refused
        store(axis, sample_index(axis, 0u, a, b), copy);
    }
    if (khi == kFaceFixedVelocity) {
        store(axis, sample_index(axis, n, a, b), face_velocity(2u * axis + 1u, axis));
    } else {
        float copy = load(axis, sample_index(axis, n - 1u, a, b));
        if (copy < 0.0) copy = amb;  // into the box through the plus face: refused
        store(axis, sample_index(axis, n, a, b), copy);
    }
}
