// ---------------------------------------------------------------------------
// airflow_copy.slang (plan B, task B1) -- airflow_v[i] = airflow_u[i], one
// thread per float. One of the airflow chain runner's two test kernels (its
// design note, docs/design/physics/plans/2026-10-12-airflow-chain-runner-
// design.md §4): with airflow_add_one it shows a chain's second dispatch sees
// its first's write.
//
// SET 1, THE AIRFLOW SET: bindings.slang section F's airflow_u (1) and
// airflow_v (2), never set 0's state registry. The runner
// (spike/airflow/chain_runner.hpp) binds its buffers[i] at set 1, binding i,
// over an empty set 0, so a runner for this kernel lists at least section F's
// first three buffers.
//
// A load and a store, no arithmetic: the copy carries the float's bits.
// ---------------------------------------------------------------------------

import bindings;

// The airflow push convention (airflow_params_layout.slang): 16 bytes, the
// count in word 2.
struct CopyPush {
    uint unused0;
    uint unused1;
    uint count;  // floats
    uint unused3;
};

[[vk::push_constant]] ConstantBuffer<CopyPush> copy_push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_copy(uint3 tid : SV_DispatchThreadID) {
    const uint i = tid.x;
    if (i >= copy_push.count) { return; }
    airflow_v[i] = airflow_u[i];
}
