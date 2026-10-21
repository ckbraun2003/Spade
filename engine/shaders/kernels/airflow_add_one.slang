// ---------------------------------------------------------------------------
// airflow_add_one.slang (plan B, task B1) -- x[i] += 1, one thread per float,
// IN PLACE ON BINDING 1: the destination airflow_copy writes. The airflow
// chain runner's second test kernel (its design note §4):
//
//   * after airflow_copy in one chain, it reads what the copy wrote, so the
//     result is src + 1 only if the barrier between the two dispatches holds;
//   * alone in a chain submitted n times, it adds exactly n.
//
// Set 1, the airflow set, as airflow_copy.slang says. One add, decorated NoContraction
// by the build's -fp-mode precise (nothing here could contract anyway).
// ---------------------------------------------------------------------------

[[vk::binding(1, 1)]] RWStructuredBuffer<float> add_one_x;

struct AddOneParams {
    uint count;  // floats
};

[[vk::push_constant]] ConstantBuffer<AddOneParams> add_one_params;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_add_one(uint3 tid : SV_DispatchThreadID) {
    const uint i = tid.x;
    if (i >= add_one_params.count) { return; }
    add_one_x[i] = add_one_x[i] + 1.0f;
}
