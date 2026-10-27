// ---------------------------------------------------------------------------
// airflow_solid_velocities.slang (plan B, task B5.5) -- WHAT A CLOSED FACE
// HOLDS on the GPU: physics/airflow/solids.cpp's apply_solid_velocities(), its
// CPU twin and the authority, in both SolidInterior modes:
//   rigid        every closed face (theta == 0) takes its wall's velocity;
//   extrapolate  the same, then the free-slip layers (solids design Task 7):
//                a layer-1 face takes the mean of its open neighbours of its
//                own component, then a layer-2 face the mean of its layer-1
//                neighbours, each mean the neighbours' velocities summed from
//                0.0 in the order -x, +x, -y, +y, -z, +z (no periodic wrap)
//                and divided by their count with div32_ftz.
//
// TWO PASSES, chosen by the push (one invocation per face, u then v then w
// from aperture_base(c); the push's count is every face):
//   pass 0  rigid: a closed face takes its wall's velocity;
//   pass 1  extrapolate's first: a closed face of layer 1 takes its mean, any
//           other closed face its wall's velocity;
//   pass 2  extrapolate's second, dispatched after pass 1: a face of layer 2
//           takes its mean of the layer-1 faces pass 1 wrote.
// Pass 1 gives the CPU's bits: a layer-1 face's sources are open faces, which
// no pass writes, so its mean is the CPU's whatever order the invocations run
// in; and the CPU's wall velocity on a layer-1 face is overwritten before it
// is read. Layer 2 reads layer 1, so it waits for pass 1's barrier.
//
// EVERY DIVISION IS fp32_math's div32_ftz (solids.cpp:796's `sum / count`,
// count 1 to 6): divss's bits under the step's FTZ|DAZ, where the device's
// own OpFDiv is not correctly rounded (0x3F800800 / 3.0: the host 0x3EAAB555,
// the 3060 Ti's division 0x3EAAB556). The adds are OpFAdd, uncontracted.
//
// SET 1, bindings.slang section F: reads airflow_params (the dims, the solid
// bit), airflow_apertures (13), airflow_wall_velocity (14) and
// airflow_face_tags (16, the free-slip layer's bytes); reads and writes
// airflow_u, _v and _w (1-3). With no solid (word 16) it writes nothing.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;
import fp32_math;

// The airflow push convention: 16 bytes, the count in word 2.
struct SolidVelocitiesPush {
    uint pass;   // 0 rigid, 1 extrapolate's layer 1, 2 extrapolate's layer 2
    uint unused0;
    uint count;  // every face: F_u + F_v + F_w
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<SolidVelocitiesPush> solid_velocities_push;

static const uint kPassRigid = 0u;
static const uint kPassLayer1 = 1u;
static const uint kPassLayer2 = 2u;

float sample_velocity(uint c, uint idx) {
    if (c == 0u) return airflow_u[idx];
    if (c == 1u) return airflow_v[idx];
    return airflow_w[idx];
}

void set_sample_velocity(uint c, uint idx, float value) {
    if (c == 0u) {
        airflow_u[idx] = value;
    } else if (c == 1u) {
        airflow_v[idx] = value;
    } else {
        airflow_w[idx] = value;
    }
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_solid_velocities(uint3 tid : SV_DispatchThreadID) {
    const uint f = tid.x;
    if (f >= solid_velocities_push.count || !has_solids()) { return; }
    const uint c = f < aperture_base(1u) ? 0u : (f < aperture_base(2u) ? 1u : 2u);
    const uint idx = f - aperture_base(c);
    if (!face_closed(c, idx)) { return; }
    const uint pass = solid_velocities_push.pass;
    const uint layer = face_layer(c, idx);
    if (pass == kPassRigid || (pass == kPassLayer1 && layer != 1u)) {
        set_sample_velocity(c, idx, face_wall(c, idx));
        return;
    }
    if (pass != kPassLayer1 && (pass != kPassLayer2 || layer != 2u)) { return; }
    // The component's samples: its own axis holds n + 1.
    const uint3 n = grid_dims();
    const uint3 m = uint3(c == 0u ? n.x + 1u : n.x, c == 1u ? n.y + 1u : n.y, c == 2u ? n.z + 1u : n.z);
    const uint3 q = uint3(idx % m.x, (idx / m.x) % m.y, idx / (m.x * m.y));
    const uint3 stride = uint3(1u, m.x, m.x * m.y);
    float sum = 0.0f;
    uint count = 0u;
    for (uint a = 0u; a < 3u; ++a) {
        for (uint up = 0u; up < 2u; ++up) {
            if (up == 0u ? q[a] == 0u : q[a] + 1u == m[a]) { continue; }
            const uint nb = up == 0u ? idx - stride[a] : idx + stride[a];
            const bool source = layer == 1u ? !face_closed(c, nb) : face_layer(c, nb) == 1u;
            if (!source) { continue; }
            sum = sum + sample_velocity(c, nb);
            count = count + 1u;
        }
    }
    set_sample_velocity(c, idx, div32_ftz(sum, float(count)));
}
