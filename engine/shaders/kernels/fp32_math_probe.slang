// ---------------------------------------------------------------------------
// fp32_math_probe.slang -- THE EVALUATION KERNEL for the fp32_math parity
// batteries. One thread per RESULT: read an argument out of the arg buffer,
// call one of ../fp32_math.slang's kernels, write the answer to the result
// buffer. It computes nothing of its own -- deliberately, because anything it
// computed would sit between the kernel under test and the bits the host
// compares.
//
// WHY IT HAS ITS OWN BINDINGS AND DOES NOT IMPORT bindings.slang. The state
// binding registry (shaders/shared/bindings.slang, set 0) is the ABI the host's
// descriptor writes depend on and holds only the state mirror's buffers; a
// probe that needed two scratch buffers has no business in it. This kernel
// declares its OWN set 0 with its OWN two bindings, which is a different
// descriptor-set layout on a different pipeline and touches nothing the
// registry owns. engine/compute/vulkan/probe_runner.hpp is the host half and
// its header states the same contract from the other side; the
// GpuFp32Math.ProbeKernelDeclaresTheRunnerBindings test reads the compiled
// SPIR-V and checks the two agree, so the convention is verified rather than
// asserted.
//
// THE ARGUMENT-BUFFER LAYOUT, and why the mode rides in it. probe_runner's
// signature is fixed by the task brief and is deliberately kernel-agnostic --
// `run_probe(ctx, spirv, args, out)`, no user-supplied push value -- because
// Task 7 reuses it for the sort-isolation tests. So a probe that needs to
// select between six evaluations puts the selector in the data it already
// owns: args[0] is the MODE, bit-cast from a uint, and args[1..] are the
// arguments proper. That keeps the runner free of any knowledge of this
// kernel.
//
//      mode 0  out[i] = log32(args[1 + i])
//      mode 1  out[i] = exp32(args[1 + i])
//      mode 2  out[i] = sin32(args[1 + i])
//      mode 3  out[i] = cos32(args[1 + i])
//      mode 4  out[i] = log32_div(args[1 + 2i], args[2 + 2i])
//      mode 5  out[i] = fp32_math_constant(i)          (args ignored)
//      mode 6  u64_shr, ONE OUTPUT WORD PER THREAD -- see below
//
// ---------------------------------------------------------------------------
// MODE 6 (S6 Task 8 review, minor M9) -- WHY AN INTEGER KERNEL RIDES THE
// fp32_math PROBE, since it looks like a category error and is not.
//
// engine/shaders/u64.slang is the exact 64-bit integer arithmetic
// engine/shaders/rng.slang performs on a uint2 pair, and it exists for exactly
// the reason this file's own kernels do: the correctness device cannot give the
// primitive (shaderInt64 == VK_FALSE, Task 8's probe), so the engine implements
// it and must PROVE the implementation matches the host bit for bit. That is
// the same claim, the same shape of evidence and the same harness as log32's.
//
// It rides THIS probe rather than a new one because a second probe kernel would
// mean a second CMake entry, a second kSpirvModules row and a second copy of
// probe_runner's binding contract -- for three lines of shift logic. Importing
// u64 changes nothing about this module's EXACT profile: u64.slang contains no
// floating-point arithmetic at all, so it adds no OpFDiv, no sqrt, no
// sum-of-products and nothing for NoContraction to be missing from.
//
// LAYOUT: one thread per OUTPUT WORD, because a 64-bit result is two of them.
// Thread i covers case c = i/2 and emits that case's LOW word when i is even
// and its HIGH word when i is odd. A case is three arguments, bit-cast from
// uints: args[1+3c] = a.x, args[2+3c] = a.y, args[3+3c] = n.
//
// WHAT IT IS FOR, specifically: u64_shr's `n == 0` branch and most of its
// `n >= 32` branch are unreachable from any current kernel (splitmix64 asks for
// 27, 30 and 31; rng_next_float asks for 40), so this sweeps all 64 distances
// against host-computed expectations rather than leaving them to inspection.
// ---------------------------------------------------------------------------
//
// FLOAT-CONTROLS PROFILE: EXACT (tests/test_slang_layouts.cpp holds this
// module to it). The compiled module contains ../fp32_math.slang whole, so the
// gate's NoContraction / no-OpFDiv / no-sqrt / no-sum-of-products rules are
// enforced over the ported kernels themselves, not just over this file.
// ---------------------------------------------------------------------------

import fp32_math;
import u64;

// The two buffers, in probe_runner's pinned order: binding 0 in, binding 1 out.
// StructuredBuffer (not RW) on the input is documentation with teeth -- a probe
// that tried to scribble on its own arguments would not compile.
[[vk::binding(0, 0)]] StructuredBuffer<float> probe_args;
[[vk::binding(1, 0)]] RWStructuredBuffer<float> probe_out;

// probe_runner's push-constant block. Mirrored by ProbePushConstants in
// engine/compute/vulkan/probe_runner.cpp; the sizes are pinned by a
// static_assert there.
struct ProbeParams {
    uint arg_count;
    uint out_count;
};

[[vk::push_constant]] ConstantBuffer<ProbeParams> probe_params;

static const uint kModeLog = 0u;
static const uint kModeExp = 1u;
static const uint kModeSin = 2u;
static const uint kModeCos = 3u;
static const uint kModeDiv = 4u;
static const uint kModeConstants = 5u;
static const uint kModeU64Shr = 6u;

[shader("compute")]
[numthreads(64, 1, 1)]
void fp32_math_probe(uint3 tid : SV_DispatchThreadID) {
    const uint i = tid.x;
    if (i >= probe_params.out_count) {
        return;
    }

    const uint mode = asuint(probe_args[0]);

    // The constants mode reads no argument at all, so its bounds question is
    // different from every other mode's and is answered first.
    if (mode == kModeConstants) {
        probe_out[i] = fp32_math_constant(i);
        return;
    }

    // Like the constants mode, this one's bounds question is its own shape
    // (three arguments per TWO results), so it is answered before the shared
    // one-argument-per-result path below.
    if (mode == kModeU64Shr) {
        const uint c = i / 2u;
        const uint base = 1u + 3u * c;
        if (base + 2u >= probe_params.arg_count) {
            probe_out[i] = asfloat(0x7FC00001u);
            return;
        }
        const uint2 a = uint2(asuint(probe_args[base]), asuint(probe_args[base + 1u]));
        const uint n = asuint(probe_args[base + 2u]);
        const uint2 shifted = u64_shr(a, n);
        // Even thread -> low word, odd thread -> high word. Carried through the
        // float buffer as a BIT PATTERN, never as a number: nothing on either
        // side performs arithmetic on it, so the store/load pair preserves
        // every encoding including the ones that happen to be NaNs (the same
        // property this file's edge-case battery already relies on).
        probe_out[i] = asfloat(((i & 1u) == 0u) ? shifted.x : shifted.y);
        return;
    }

    if (mode == kModeDiv) {
        const uint base = 1u + 2u * i;
        // A short dispatch would otherwise read past the buffer. Answering
        // with a NaN rather than clamping makes a host-side sizing mistake
        // LOUD -- the comparison prints the bit pattern and no argument
        // legitimately produces this one.
        if (base + 1u >= probe_params.arg_count) {
            probe_out[i] = asfloat(0x7FC00001u);
            return;
        }
        probe_out[i] = log32_div(probe_args[base], probe_args[base + 1u]);
        return;
    }

    if (1u + i >= probe_params.arg_count) {
        probe_out[i] = asfloat(0x7FC00001u);
        return;
    }
    const float x = probe_args[1u + i];

    switch (mode) {
        case kModeLog:
            probe_out[i] = log32(x);
            break;
        case kModeExp:
            probe_out[i] = exp32(x);
            break;
        case kModeSin:
            probe_out[i] = sin32(x);
            break;
        case kModeCos:
            probe_out[i] = cos32(x);
            break;
        default:
            probe_out[i] = asfloat(0x7FC00002u);
            break;
    }
}
