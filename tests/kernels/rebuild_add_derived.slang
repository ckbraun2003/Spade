// ---------------------------------------------------------------------------
// rebuild_add_derived.slang (module API stage 7, Task C3) -- a TEST kernel, the step's pass: adds the derived buffer to the
// resident grid (set 1: airflow_u += airflow_v). With the derived buffer rebuilt it adds the configured constant; without
// it, zero -- which is how a missed rebuild after a restore shows.
// ---------------------------------------------------------------------------

import bindings;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void rebuild_add_derived(uint3 tid : SV_DispatchThreadID) {
    if (tid.x >= pass_params.world_count * 4096u) { return; }
    airflow_u[tid.x] = airflow_u[tid.x] + airflow_v[tid.x];
}
