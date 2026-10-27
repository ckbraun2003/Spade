// ---------------------------------------------------------------------------
// airflow_solid_gradient.slang (plan B, task B5.2) -- the solids
// projection's last stage: projection.cpp's
// subtract_gradient_with_apertures() for one velocity component, its CPU twin
// and the authority for every expression. A closed face (theta == 0, the
// solid field's, airflow_pcg.slang) takes its wall's velocity; every other
// face that is not fixed subtracts g_coef (p_right - p_left) as B2's
// airflow_gradient_subtract does (which stays plan A's):
//   - a face between two cells: p of the cell above it minus the cell below;
//   - a periodic axis's face 0: p_first - p_last when the axis has more than
//     one cell, and face n takes face 0's new value (its own theta unread);
//   - a pressure outlet: its ghost is -p, so face 0 takes p_first - (-p_first)
//     and face n (-p_last) - p_last;
//   - a fixed-velocity face is left as it is, closed or not.
// p is the grid's (airflow_pressure, the solve's x copied back).
//
// One invocation per face of the dispatched component; the push's count (word
// 2) is its faces. On a periodic axis the invocation of face 0 writes face n
// too, and face n's invocation writes nothing, so no invocation reads a face
// another one writes. Every operation is a correctly rounded add or multiply,
// uncontracted, or a copy, so the velocity is the CPU's bit for bit.
//
// SET 1, bindings.slang section F: reads airflow_params, airflow_pressure,
// airflow_apertures (13) and airflow_wall_velocity (14); reads and writes
// airflow_u, _v or _w.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;

struct SolidGradientPush {
    uint component;  // 0 u, 1 v, 2 w
    uint unused0;
    uint count;      // the component's faces
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<SolidGradientPush> solid_gradient_push;

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
void airflow_solid_gradient(uint3 tid : SV_DispatchThreadID) {
    const uint at = tid.x;
    if (at >= solid_gradient_push.count) { return; }
    const uint c = solid_gradient_push.component;
    const uint3 n = grid_dims();
    const float g = g_coef();
    // The component's samples: its own axis holds n + 1.
    const uint3 m = uint3(c == 0u ? n.x + 1u : n.x, c == 1u ? n.y + 1u : n.y, c == 2u ? n.z + 1u : n.z);
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
        if (face_closed(c, at)) {
            set_velocity(c, at, face_wall(c, at));
            return;
        }
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
        float value = vel;
        if (face_closed(c, at)) {
            value = face_wall(c, at);
        } else if (na > 1u) {
            value = vel - g * (p_first - p_last);
        }
        set_velocity(c, at, value);
        set_velocity(c, at + na * face_stride, value);
        return;
    }
    if (q == 0u && lo == kFacePressureOutlet) {
        set_velocity(c, at, face_closed(c, at) ? face_wall(c, at) : vel - g * (p_first - (-p_first)));
    }
    if (q == na && hi == kFacePressureOutlet) {
        set_velocity(c, at, face_closed(c, at) ? face_wall(c, at) : vel - g * ((-p_last) - p_last));
    }
}
