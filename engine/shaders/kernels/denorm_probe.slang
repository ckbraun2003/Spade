// ---------------------------------------------------------------------------
// denorm_probe.slang -- M1 of the NVIDIA fp32-denormal measurement
// (docs/design/core/plans/2026-10-04-nvidia-denorm-measurement-plan.md).
// Compiled in every build since banded-parity T6, a standing device record:
// like every kernel it requests no fp32 denormal mode (P3), so it shows what
// the device does with subnormals by default.
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

// The modes, numbered as tests/test_gpu_denorm_probe.cpp's Op numbers them.
static const uint kModePass = 0u;       // a
static const uint kModeAdd = 1u;        // a + b
static const uint kModeSub = 2u;        // a - b
static const uint kModeMul = 3u;        // a * b
static const uint kModeAbs = 4u;        // abs(a)
static const uint kModeNeg = 5u;        // -a
static const uint kModeMin = 6u;        // min(a, b)
static const uint kModeMax = 7u;        // max(a, b)
static const uint kModeMulAdd = 8u;     // a * b + c, a separate multiply and add
static const uint kModeFloor = 9u;      // floor(a), the float itself (its zero keeps a sign)
static const uint kModeFloorInt = 10u;  // int(floor(a)), as grid_build's cell index
static const uint kModeCompare = 11u;   // a <0, >0, ==0, <=0, >=0, a<b: bits 0..5
static const uint kModeClamp = 12u;     // clamp(a, b, c)
static const uint kModeSqrt = 13u;      // sqrt(a)
static const uint kModeDiv = 14u;       // a / b

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
    const uint mode = asuint(probe_args[0]);
    const float a = probe_args[base];
    const float b = probe_args[base + 1u];
    const float c = probe_args[base + 2u];
    float r = asfloat(0x7FC00002u);  // an unknown mode
    switch (mode) {
        case kModePass:
            r = a;
            break;
        case kModeAdd:
            r = a + b;
            break;
        case kModeSub:
            r = a - b;
            break;
        case kModeMul:
            r = a * b;
            break;
        case kModeAbs:
            r = abs(a);
            break;
        case kModeNeg:
            r = -a;
            break;
        case kModeMin:
            r = min(a, b);
            break;
        case kModeMax:
            r = max(a, b);
            break;
        case kModeMulAdd:
            r = a * b + c;
            break;
        case kModeFloor:
            r = floor(a);
            break;
        case kModeFloorInt:
            r = float(int(floor(a)));
            break;
        case kModeCompare: {
            uint bits = 0u;
            if (a < 0.0f) bits |= 1u;
            if (a > 0.0f) bits |= 2u;
            if (a == 0.0f) bits |= 4u;
            if (a <= 0.0f) bits |= 8u;
            if (a >= 0.0f) bits |= 16u;
            if (a < b) bits |= 32u;
            r = float(bits);
            break;
        }
        case kModeClamp:
            r = clamp(a, b, c);
            break;
        case kModeSqrt:
            r = sqrt(a);
            break;
        case kModeDiv:
            r = a / b;
            break;
        default:
            break;
    }
    probe_out[i] = r;
}
