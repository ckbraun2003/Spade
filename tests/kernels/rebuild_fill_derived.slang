// ---------------------------------------------------------------------------
// rebuild_fill_derived.slang (module API stage 7, Task C3) -- a TEST kernel, the rebuild hook's GPU half: fills the DERIVED
// buffer (set 1, airflow_v: 4096 floats a world) from the module's configuration image (airflow_config word 0, a float's
// bits). The buffer is scratch, not state, so a restore must run this again to get it back.
// ---------------------------------------------------------------------------

import bindings;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void rebuild_fill_derived(uint3 tid : SV_DispatchThreadID) {
    if (tid.x >= pass_params.world_count * 4096u) { return; }
    airflow_v[tid.x] = asfloat(airflow_config[0]);
}
