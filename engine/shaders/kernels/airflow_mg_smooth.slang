// ---------------------------------------------------------------------------
// airflow_mg_smooth.slang (plan B, task B2) -- one colour of one red-black
// Gauss-Seidel sweep on one multigrid level: pressure.cpp's
// smooth_red_black() and smooth_one_cell(), its CPU twin and the authority for
// every expression. p = (sum - b) / diag for every cell of the colour, where
// (i + j + k) & 1 is the colour, red 0 then black 1; a cell whose diagonal is
// 0 is left as it is.
//
// A colour's cells read only the other colour's, so one invocation per cell
// of the colour gives the CPU's sequential sweep exactly: every level's
// periodic ring is even (check_shape() refuses an odd one and the hierarchy
// never makes one, multigrid.hpp), so the wrap's neighbour is the other
// colour too. The CPU's interior fast path sums the same six neighbours in
// the same order from 0 with the diagonal 6, so mg_levels' one stencil gives
// its bits.
//
// One invocation per cell of the colour: each row of the level holds
// (nx + 1) / 2 slots, slot m at i = 2 m + ((j + k + colour) & 1), and a slot
// past the row's end does nothing. The push's count (word 2) is the slots,
// (nx + 1) / 2 * ny * nz.
//
// PARITY. The sums are correctly rounded adds and multiplies (-fp-mode
// precise: no contraction), and the division is fp32_math's div32_ftz, divss
// under the CPU step's FTZ|DAZ to the bit, so the sweep is its twin's bit for
// bit (TD-14 as amended 2026-10-22, the user's "Full bit-exact"); the module
// is scanned under the exact profile, so no bare OpFDiv can return. It sits
// in smooth_divide() alone.
// ---------------------------------------------------------------------------

import bindings;
import mg_levels;
import fp32_math;

struct SmoothPush {
    uint level;
    uint colour;  // 0 red, 1 black
    uint count;   // the colour's slots
    uint unused;
};

[[vk::push_constant]] ConstantBuffer<SmoothPush> smooth_push;

// The smoothing's one division (smooth_one_cell()'s `(sum - b) / diag`).
float smooth_divide(float numerator, float diag) { return div32_ftz(numerator, diag); }

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_mg_smooth(uint3 tid : SV_DispatchThreadID) {
    const uint t = tid.x;
    if (t >= smooth_push.count) { return; }
    const Level L = level(smooth_push.level);
    const uint half = (L.n.x + 1u) / 2u;
    const uint row = t / half;
    const uint j = row % L.n.y;
    const uint k = row / L.n.y;
    const uint i = 2u * (t % half) + ((j + k + smooth_push.colour) & 1u);
    if (i >= L.n.x) { return; }
    const uint3 c = uint3(i, j, k);
    const Stencil st = stencil(L, c);
    if (st.diag == 0.0) { return; }
    const uint at = cell_at(L, c);
    set_level_p(L, at, smooth_divide(st.sum - level_b(L, at), st.diag));
}
