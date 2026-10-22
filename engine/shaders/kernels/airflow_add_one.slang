// ---------------------------------------------------------------------------
// airflow_add_one.slang (plan B, task B1) -- airflow_v[i] += 1, one thread per
// float, IN PLACE ON airflow_v: the buffer airflow_copy writes. The airflow
// chain runner's second test kernel (its design note §4):
//
//   * after airflow_copy in one chain, it reads what the copy wrote, so the
//     result is src + 1 only if the barrier between the two dispatches holds;
//   * alone in a chain submitted n times, it adds exactly n.
//
// Set 1, bindings.slang section F, as airflow_copy.slang says. One add,
// decorated NoContraction by the build's -fp-mode precise (nothing here could
// contract anyway).
// ---------------------------------------------------------------------------

import bindings;

// The airflow push convention: 16 bytes, the count in word 2.
struct AddOnePush {
    uint unused0;
    uint unused1;
    uint count;  // floats
    uint unused3;
};

[[vk::push_constant]] ConstantBuffer<AddOnePush> add_one_push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_add_one(uint3 tid : SV_DispatchThreadID) {
    const uint i = tid.x;
    if (i >= add_one_push.count) { return; }
    airflow_v[i] = airflow_v[i] + 1.0f;
}
