// ---------------------------------------------------------------------------
// airflow_copy.slang (plan B, task B1) -- dst[i] = src[i], one thread per
// 32-bit word. One of the airflow chain runner's two test kernels (its design
// note, docs/design/physics/plans/2026-10-12-airflow-chain-runner-design.md
// §4): with airflow_add_one it shows a chain's second dispatch sees its
// first's write.
//
// SET 1, THE AIRFLOW SET (Core's B0 plan, decision 1), never set 0's state
// registry. The runner (spike/airflow/chain_runner.hpp) binds its buffers[i] at
// set 1, binding i, over an empty set 0; these two bindings are a runner's
// first two buffers. B0's named airflow block in bindings.slang replaces these
// declarations when it lands.
//
// Words, not floats: a copy carries every bit pattern, and does no arithmetic.
// ---------------------------------------------------------------------------

[[vk::binding(0, 1)]] StructuredBuffer<uint> copy_src;
[[vk::binding(1, 1)]] RWStructuredBuffer<uint> copy_dst;

// The runner pushes the same block to every kernel of a chain; this kernel
// reads its first word.
struct CopyParams {
    uint count;  // words
};

[[vk::push_constant]] ConstantBuffer<CopyParams> copy_params;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_copy(uint3 tid : SV_DispatchThreadID) {
    const uint i = tid.x;
    if (i >= copy_params.count) { return; }
    copy_dst[i] = copy_src[i];
}
