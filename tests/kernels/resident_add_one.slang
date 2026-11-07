// ---------------------------------------------------------------------------
// resident_add_one.slang (module API stage 7, Task C2) -- a TEST kernel: adds 1.0 to every cell of the resident grid
// (set 1, airflow_u: 4096 floats a world, world after world). Exact in fp32, so the device and the CPU reference agree bit for
// bit, which is what lets the resident-array tests compare digests. Its layout is set 0 plus the one set-1 binding it names.
// ---------------------------------------------------------------------------

import bindings;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void resident_add_one(uint3 tid : SV_DispatchThreadID) {
    if (tid.x >= pass_params.world_count * 4096u) { return; }
    airflow_u[tid.x] = airflow_u[tid.x] + 1.0;
}
