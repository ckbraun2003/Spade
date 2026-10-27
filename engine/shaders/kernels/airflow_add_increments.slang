// ---------------------------------------------------------------------------
// airflow_add_increments.slang (plan B, B7's GPU step) -- fluid_step's swap
// and add_and_clear (engine/physics/airflow/fluid_step.cpp, step_flushed):
// the advected velocity taken from airflow_next and the step's body-force
// increments added once, u = next + du, then du cleared for the next step's
// gather (airflow_actuator_gather adds into airflow_increments, as
// spread_actuator() adds into du). One invocation per face of u, v and w in
// turn: airflow_next's and airflow_increments' packing, u at 0, v at
// (nx+1) ny nz, w after it.
//
// CLEAR ONLY (push word 0 = 1): du = 0 alone, which GpuFluid dispatches once
// at create so the increments start cleared, as make_fluid_scratch()'s du
// does, without staging a device-only buffer.
//
// EXACT: one OpFAdd per face, the CPU's one `+=` (the swap is a copy), under
// -fp-mode precise; a 0 increment adds as the CPU's does. Scanned under the
// exact profile (AirflowRunnerSpirv).
//
// SET 1, bindings.slang section F: reads airflow_params (the dims) and
// airflow_next (5); writes airflow_u/v/w (1-3) and airflow_increments (7).
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;

// The airflow push convention: 16 bytes, the count in word 2.
struct AddPush {
    uint clear_only;  // 1: du = 0, and u, v and w untouched
    uint unused1;
    uint face_count;  // (nx+1) ny nz + nx (ny+1) nz + nx ny (nz+1)
    uint unused3;
};

[[vk::push_constant]] ConstantBuffer<AddPush> add_push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_add_increments(uint3 tid : SV_DispatchThreadID) {
    const uint f = tid.x;
    if (f >= add_push.face_count) { return; }
    if (add_push.clear_only != 0u) {
        airflow_increments[f] = 0.0;
        return;
    }
    const uint3 n = grid_dims();
    const uint faces_u = (n.x + 1u) * n.y * n.z;
    const uint faces_v = n.x * (n.y + 1u) * n.z;
    const float value = airflow_next[f] + airflow_increments[f];
    airflow_increments[f] = 0.0;
    if (f < faces_u) {
        airflow_u[f] = value;
    } else if (f < faces_u + faces_v) {
        airflow_v[f - faces_u] = value;
    } else {
        airflow_w[f - faces_u - faces_v] = value;
    }
}
