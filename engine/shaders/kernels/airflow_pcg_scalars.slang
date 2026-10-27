// ---------------------------------------------------------------------------
// airflow_pcg_scalars.slang (plan B, task B5.1b; a first cut that B5.4
// extends with solve_pcg_to()'s decisions) -- THE CG'S QUOTIENTS on the GPU,
// from airflow_scalars to airflow_scalars (slots: airflow_pcg.slang), one
// thread, each the expression of pcg.cpp, its CPU twin and the authority:
//   op 0  alpha    = rz / dq                                   (pcg.cpp:118)
//   op 1  beta     = rz_next / rz                              (pcg.cpp:126)
//   op 2  residual = b_max > 0 ? r_max / b_max : 0             (pcg.cpp:57)
//
// EVERY DIVISION IS fp32_math's div32_ftz: divss's bits under the CPU step's
// FTZ|DAZ, NaN and infinity included, where the device's own OpFDiv is not
// correctly rounded (the 3060 Ti gives 0x3EAAB556 for 0x3F800800 / 3.0, the
// host 0x3EAAB555). The compare is exact. So the quotients are the CPU's bit
// for bit, and the module is scanned under the exact profile.
//
// SET 1, bindings.slang section F: reads and writes airflow_scalars (21);
// reads airflow_params only through the module imports.
//
// A MISUSE SHOWS AS NaN: an op past 2 writes a NaN residual.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;
import fp32_math;

// The airflow push convention: 16 bytes, the count in word 2.
struct PcgScalarsPush {
    uint op;  // 0 alpha, 1 beta, 2 residual
    uint unused0;
    uint count;  // 1: one thread
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<PcgScalarsPush> scalars_push;

static const uint kOpAlpha = 0u;
static const uint kOpBeta = 1u;
static const uint kOpResidual = 2u;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_pcg_scalars(uint3 tid : SV_DispatchThreadID) {
    if (tid.x != 0u || tid.x >= scalars_push.count) { return; }
    const uint op = scalars_push.op;
    if (op == kOpAlpha) {
        airflow_scalars[kSlotAlpha] = div32_ftz(airflow_scalars[kSlotRz], airflow_scalars[kSlotDq]);
        return;
    }
    if (op == kOpBeta) {
        airflow_scalars[kSlotBeta] = div32_ftz(airflow_scalars[kSlotRzNext], airflow_scalars[kSlotRz]);
        return;
    }
    if (op == kOpResidual) {
        const float r_max = airflow_scalars[kSlotRMax];
        const float b_max = airflow_scalars[kSlotBMax];
        airflow_scalars[kSlotResidual] = b_max > 0.0f ? div32_ftz(r_max, b_max) : 0.0f;
        return;
    }
    airflow_scalars[kSlotResidual] = asfloat(0x7FC00000u);
}
