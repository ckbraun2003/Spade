// ---------------------------------------------------------------------------
// airflow_gradient_subtract.slang (plan B, task B2) -- the projection's last
// stage: projection.cpp's subtract_gradient() for one velocity component, its
// CPU twin and the authority for every expression. Every face that is not
// fixed subtracts g_coef (p_right - p_left), g_coef = h / (rho dx)
// (airflow_params_layout; the host's float from the CPU's own expression):
//   - a face between two cells: p of the cell above it minus the cell below;
//   - a periodic axis's face 0: p_first - p_last when the axis has more than
//     one cell, and face n takes face 0's new value;
//   - a pressure outlet: its ghost is -p (p = 0 on the face), so face 0 takes
//     p_first - (-p_first) and face n (-p_last) - p_last;
//   - a fixed-velocity face is left as it is.
// The domain's face kinds are airflow_params_layout's face_kind().
//
// One invocation per face of the dispatched component; the push's count (word
// 2) is its faces. On a periodic axis the invocation of face 0 writes face n
// too, and face n's invocation writes nothing, so no invocation reads a face
// another one writes. Every operation is a correctly rounded add or multiply,
// so the velocity is the CPU's bit for bit.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;

struct GradientPush {
    uint component;  // 0 u, 1 v, 2 w
    uint unused0;
    uint count;      // the component's faces
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<GradientPush> gradient_push;

float velocity(uint c, uint at) {
    if (c == 0u) return airflow_u[at];
    if (c == 1u) return airflow_v[at];
    return airflow_w[at];
}

void set_velocity(uint c, uint at, float value) {
    if (c == 0u) {
        airflow_u[at] = value;
    } else if (c == 1u) {
        airflow_v[at] = value;
    } else {
        airflow_w[at] = value;
    }
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_gradient_subtract(uint3 tid : SV_DispatchThreadID) {
    const uint at = tid.x;
    if (at >= gradient_push.count) { return; }
    const uint c = gradient_push.component;
    const uint3 n = grid_dims();
    const float g = g_coef();
    // The component's samples: its own axis holds n + 1.
    const uint3 m = uint3(c == 0u ? n.x + 1u : n.x, c == 1u ? n.y + 1u : n.y, n.z);
    const uint3 f = uint3(at % m.x, (at / m.x) % m.y, at / (m.x * m.y));
    const uint q = f[c];
    const uint na = n[c];
    // The pressure cell at index s along the axis, this face's row across it.
    uint3 cell = f;
    cell[c] = 0u;
    const uint first = cell.x + n.x * (cell.y + n.y * cell.z);
    const uint stride = c == 0u ? 1u : (c == 1u ? n.x : n.x * n.y);
    const float vel = velocity(c, at);
    if (q >= 1u && q < na) {
        const float p_hi = airflow_pressure[first + q * stride];
        const float p_lo = airflow_pressure[first + (q - 1u) * stride];
        set_velocity(c, at, vel - g * (p_hi - p_lo));
        return;
    }
    const float p_first = airflow_pressure[first];
    const float p_last = airflow_pressure[first + (na - 1u) * stride];
    const uint lo = face_kind(2u * c);
    const uint hi = face_kind(2u * c + 1u);
    // The face stride along the component's own axis, from face 0 to face q.
    const uint face_stride = c == 0u ? 1u : (c == 1u ? m.x : m.x * m.y);
    if (lo == kFacePeriodic) {
        if (q != 0u) return;  // face n: face 0's invocation writes it
        const float value = na > 1u ? vel - g * (p_first - p_last) : vel;
        set_velocity(c, at, value);
        set_velocity(c, at + na * face_stride, value);
        return;
    }
    if (q == 0u && lo == kFacePressureOutlet) set_velocity(c, at, vel - g * (p_first - (-p_first)));
    if (q == na && hi == kFacePressureOutlet) set_velocity(c, at, vel - g * ((-p_last) - p_last));
}
