// ---------------------------------------------------------------------------
// airflow_mg_residual.slang (plan B, task B2) -- one multigrid level's
// residual: pressure.cpp's residual(), its CPU twin and the authority for
// every expression. r = b - (sum - diag p) for every cell, with mg_levels' one
// stencil (the CPU's interior fast path, sum - 6 p, gives the same bits).
//
// One invocation per cell; the push's count (word 2) is the level's cells.
// Every operation is a correctly rounded add or multiply, so the residual of
// the same p and b is the CPU's bit for bit.
// ---------------------------------------------------------------------------

import bindings;
import mg_levels;

struct ResidualPush {
    uint level;
    uint unused0;
    uint count;  // the level's cells
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<ResidualPush> residual_push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_mg_residual(uint3 tid : SV_DispatchThreadID) {
    const uint at = tid.x;
    if (at >= residual_push.count) { return; }
    const Level L = level(residual_push.level);
    const Stencil st = stencil(L, cell_of(L, at));
    set_level_r(L, at, level_b(L, at) - (st.sum - st.diag * level_p(L, at)));
}
