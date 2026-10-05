// ---------------------------------------------------------------------------
// denorm_probe.slang -- M1 of the NVIDIA fp32-denormal measurement
// (docs/design/core/plans/2026-10-04-nvidia-denorm-measurement-plan.md).
// Compiled ONLY in the measurement build (SPADE_MEASURE_UNPINNED_DENORMS), so
// it carries no DenormPreserve 32, like every kernel there: it shows what the
// device does with subnormals when no fp32 denormal mode is requested.
// tests/test_gpu_denorm_probe.cpp is the host half.
//
// THE CONTRACT is probe_runner.hpp's: set 0, binding 0 the arguments, binding
// 1 the results, push constants { arg_count, out_count }, 64 threads per
// group, one thread per result. args[0] is the mode, bit-cast from a uint.
// Case i reads three operands, args[1 + 3i .. 3 + 3i]; a unary or binary op
// ignores the ones it does not need.
//
// WHY FLOAT BUFFERS. The physics kernels load and store through float-typed
// buffers, and the pass-through class exists to test exactly that path. The
// compiler cannot fold an operand, because every one comes from a buffer. The
// two integer-valued results (floor->int and the packed compares) come back
// as small exact floats, so a store that flushed could not alter them.
//
// -fp-mode precise decorates every contractable op NoContraction, so the
// multiply-add below is a separate multiply and add: what the physics kernels
// emit (tests/test_slang_layouts.cpp checks the decoration on this module).
// ---------------------------------------------------------------------------

[[vk::binding(0, 0)]] StructuredBuffer<float> probe_args;
[[vk::binding(1, 0)]] RWStructuredBuffer<float> probe_out;

struct ProbeParams {
    uint arg_count;
    uint out_count;
};
[[vk::push_constant]] ConstantBuffer<ProbeParams> probe_params;

[shader("compute")]
[numthreads(64, 1, 1)]
void denorm_probe(uint3 tid : SV_DispatchThreadID) {
    const uint i = tid.x;
    if (i >= probe_params.out_count) {
        return;
    }
    const uint base = 1u + 3u * i;
    if (base + 2u >= probe_params.arg_count) {
        probe_out[i] = asfloat(0x7FC00001u);
        return;
    }
    // STUB: every mode answers a + b.
    probe_out[i] = probe_args[base] + probe_args[base + 1u];
}
