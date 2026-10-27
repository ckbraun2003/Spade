// ---------------------------------------------------------------------------
// airflow_pcg_apply.slang (plan B, task B5.2) -- THE CG'S OPERATOR on the
// GPU: pressure.cpp's apply_operator() on the multigrid's level 0, its CPU
// twin and the authority, over one of airflow_pcg's vectors (x or d,
// airflow_pcg.slang), into q:
//   q[c] = sum - diag * v[c]
// with sum and diag mg_levels.slang's stencil of level 0 over v
// (stencil_from(), the same faces in the same order as B2's stencil(): one
// stencil, TD-9; ruling 4). With solids level 0 is weighted, its weights the
// faces' theta (set_weights(), multigrid.cpp), packed in airflow_mg_weights
// by B2's mg_layout(), so this is weighted_stencil()'s operator.
//
// One invocation per cell; the push's count (word 2) is the grid's cells.
// Every operation is a correctly rounded add or multiply, uncontracted
// (-fp-mode precise), in the CPU's order, so q is the CPU's bit for bit.
//
// SET 1, bindings.slang section F: reads airflow_params (the level table and
// the face kinds), airflow_mg_weights (12) and airflow_pcg's source vector;
// writes airflow_pcg's q.
//
// A MISUSE SHOWS AS NaN: a source past airflow_pcg's seven vectors, or q
// itself, writes a NaN q.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;
import mg_levels;

struct PcgApplyPush {
    uint source;  // the vector the operator reads (airflow_pcg.slang's kPcgX or kPcgD)
    uint unused0;
    uint count;   // the grid's cells
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<PcgApplyPush> pcg_apply_push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_pcg_apply(uint3 tid : SV_DispatchThreadID) {
    const uint c = tid.x;
    if (c >= pcg_apply_push.count) { return; }
    const uint v = pcg_apply_push.source;
    if (v >= kPcgVectors || v == kPcgQ) {
        pcg_set(kPcgQ, c, asfloat(0x7FC00000u));
        return;
    }
    const Level L = level(0u);
    const Stencil s = stencil_from(L, cell_of(L, c), pcg_offset(v));
    pcg_set(kPcgQ, c, s.sum - s.diag * pcg_get(v, c));
}
