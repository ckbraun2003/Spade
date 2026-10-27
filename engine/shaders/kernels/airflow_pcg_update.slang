// ---------------------------------------------------------------------------
// airflow_pcg_update.slang (plan B, task B5.4) -- THE CG'S VECTOR WORK on the
// GPU: pcg.cpp's solve_pcg_to(), its CPU twin and the authority, over
// airflow_pcg's vectors (airflow_pcg.slang) and level 0's p and b (the
// preconditioner's, mg_levels.slang's level_p() and level_b()), one
// invocation per cell, the op chosen by the push:
//   op 0  x <- p, masked          (projection.cpp:102's copy, pcg.cpp:70)
//   op 1  r = b - q, masked       (pcg.cpp:63-64, start()'s residual from A x)
//   op 2  level 0's b <- r, and level 0's p <- 0
//                                 (pcg.cpp:40; multigrid.cpp:256, precondition())
//   op 3  z <- level 0's p, masked                    (pcg.cpp:42-43)
//   op 4  d <- z                                       (pcg.cpp:67)
//   op 5  x += alpha d, r -= alpha q                   (pcg.cpp:119-122)
//   op 6  d = z + beta d                               (pcg.cpp:127)
//   op 7  best <- x, when the measure kept it          (pcg.cpp:103)
//   op 8  x <- best, when the finish returns it        (pcg.cpp:134)
// "Masked" is pcg.cpp's mask(): a cell marked solid (airflow_pcg.slang's
// solid_cell()) holds 0. Alpha and beta are airflow_pcg_scalars' slots, and
// keep_best and take_best its flags.
//
// THE STOP WORD: ops 0 to 7 return at once while it is set, so the
// iterations a submit runs past an exact zero (the plan's B5: "every kernel
// returns at once if the stop word is set") leave x, r, d and best as the
// stop left them. Op 8 runs only after the stop, from the finish.
//
// Every operation is a correctly rounded add, subtract or multiply,
// uncontracted (-fp-mode precise), in the CPU's order, so each vector is the
// CPU's bit for bit. No division.
//
// SET 1, bindings.slang section F: reads airflow_params (the dims), airflow_
// region_labels (18) and airflow_scalars (21); reads and writes airflow_pcg
// (20), airflow_pressure (4, level 0's p) and airflow_rhs (9, level 0's b).
//
// A MISUSE SHOWS AS NaN: an op past 8 writes a NaN x.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;
import mg_levels;

// The airflow push convention: 16 bytes, the count in word 2.
struct PcgUpdatePush {
    uint op;  // 0 x <- p ... 8 x <- best
    uint unused0;
    uint count;  // the grid's cells
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<PcgUpdatePush> update_push;

static const uint kOpXFromP = 0u;
static const uint kOpResidual = 1u;
static const uint kOpToLevel0 = 2u;
static const uint kOpZFromLevel0 = 3u;
static const uint kOpDFromZ = 4u;
static const uint kOpXR = 5u;
static const uint kOpDirection = 6u;
static const uint kOpKeepBest = 7u;
static const uint kOpTakeBest = 8u;

// pcg.cpp's mask() of one cell's value.
float masked(uint c, float v) { return solid_cell(c) ? 0.0f : v; }

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_pcg_update(uint3 tid : SV_DispatchThreadID) {
    const uint c = tid.x;
    if (c >= update_push.count) { return; }
    const uint op = update_push.op;
    if (op == kOpTakeBest) {
        if (slot_set(kSlotTakeBest)) { pcg_set(kPcgX, c, pcg_get(kPcgBest, c)); }
        return;
    }
    if (op > kOpTakeBest) {
        pcg_set(kPcgX, c, asfloat(0x7FC00000u));
        return;
    }
    if (pcg_stopped()) { return; }
    const Level L = level(0u);
    if (op == kOpXFromP) {
        pcg_set(kPcgX, c, masked(c, level_p(L, c)));
        return;
    }
    if (op == kOpResidual) {
        pcg_set(kPcgR, c, masked(c, pcg_get(kPcgB, c) - pcg_get(kPcgQ, c)));
        return;
    }
    if (op == kOpToLevel0) {
        set_level_b(L, c, pcg_get(kPcgR, c));
        set_level_p(L, c, 0.0f);
        return;
    }
    if (op == kOpZFromLevel0) {
        pcg_set(kPcgZ, c, masked(c, level_p(L, c)));
        return;
    }
    if (op == kOpDFromZ) {
        pcg_set(kPcgD, c, pcg_get(kPcgZ, c));
        return;
    }
    if (op == kOpXR) {
        const float alpha = airflow_scalars[kSlotAlpha];
        pcg_set(kPcgX, c, pcg_get(kPcgX, c) + alpha * pcg_get(kPcgD, c));
        pcg_set(kPcgR, c, pcg_get(kPcgR, c) - alpha * pcg_get(kPcgQ, c));
        return;
    }
    if (op == kOpDirection) {
        pcg_set(kPcgD, c, pcg_get(kPcgZ, c) + airflow_scalars[kSlotBeta] * pcg_get(kPcgD, c));
        return;
    }
    // kOpKeepBest
    if (slot_set(kSlotKeepBest)) { pcg_set(kPcgBest, c, pcg_get(kPcgX, c)); }
}
