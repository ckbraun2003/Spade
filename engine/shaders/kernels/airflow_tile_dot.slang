// ---------------------------------------------------------------------------
// airflow_tile_dot.slang (plan B, task B5.1b) -- THE CG'S DOT PRODUCTS on the
// GPU: physics/airflow/reduce.cpp's fixed_tile_dot(), its CPU twin and the
// authority, of two of airflow_pcg's vectors (airflow_pcg.slang), in
// reduce.hpp's two-level order, so it gives the CPU's bits at every
// workgroup size.
//
// THIS KERNEL IS PASS 0 ONLY: one thread per TILE of kTile^3 cells (fewer at
// the far edges), its products a[c] * b[c], each rounded to fp32 first, summed
// from 0.0 in (k, j, i) order, written to airflow_partials[t],
// t = ti + tnx * (tj + tny * tk). Passes 1 and 2 are airflow_tile_sum's, which
// read only the partials (that kernel's header: "a later reduction that fills
// them its own way (a tile dot) can end with them"), so a dot's chain is this
// dispatch, then airflow_tile_sum's pass 1 and pass 2 with the solver's block
// (kReduceBlockTiles), and the total lands in airflow_scalars[slot]
// (spike/airflow/solids_chain.hpp's append_tile_dot()). One rule for the
// blocks, then, on both reductions (TD-9).
//
// SET 1, bindings.slang section F: reads airflow_params (the dims) and
// airflow_pcg; writes airflow_partials (23).
//
// WHAT KEEPS THE BITS FIXED: the tile is kTile cells on a side whatever
// SPADE_WG is; a tile's sum is one thread's sequential loop; no atomics, no
// groupshared memory, no barrier; the product and the add are OpFMul and
// OpFAdd, decorated NoContraction by the build's -fp-mode precise, so the
// product rounds before it is added, as fixed_tile_dot()'s does.
//
// A MISUSE SHOWS AS NaN: a vector past airflow_pcg's seven, or a cell_count
// other than nx * ny * nz, gives NaN partials and so a NaN dot.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;

// The airflow push convention: 16 bytes, the count in word 2.
struct TileDotPush {
    uint vectors;     // bits 0-7 vector a, bits 8-15 vector b (airflow_pcg.slang's kPcg*)
    uint unused0;
    uint cell_count;  // nx * ny * nz of the params' dims
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<TileDotPush> tile_dot_push;

// physics/airflow/reduce.hpp's kReduceTile, in cells.
static const uint kTile = 4u;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_tile_dot(uint3 tid : SV_DispatchThreadID) {
    const uint3 n = grid_dims();
    const uint tnx = (n.x + kTile - 1u) / kTile;
    const uint tny = (n.y + kTile - 1u) / kTile;
    const uint tnz = (n.z + kTile - 1u) / kTile;
    const uint t = tid.x;
    if (t >= tnx * tny * tnz) { return; }
    const uint va = tile_dot_push.vectors & 0xFFu;
    const uint vb = (tile_dot_push.vectors >> 8u) & 0xFFu;
    if (tile_dot_push.cell_count != n.x * n.y * n.z || va >= kPcgVectors || vb >= kPcgVectors) {
        airflow_partials[t] = asfloat(0x7FC00000u);
        return;
    }
    const uint ti = t % tnx;
    const uint tj = (t / tnx) % tny;
    const uint tk = t / (tnx * tny);
    const uint i0 = ti * kTile;
    const uint j0 = tj * kTile;
    const uint k0 = tk * kTile;
    const uint i_end = min(i0 + kTile, n.x);
    const uint j_end = min(j0 + kTile, n.y);
    const uint k_end = min(k0 + kTile, n.z);
    const uint a0 = pcg_offset(va);
    const uint b0 = pcg_offset(vb);
    float sum = 0.0f;
    for (uint k = k0; k < k_end; ++k) {
        for (uint j = j0; j < j_end; ++j) {
            for (uint i = i0; i < i_end; ++i) {
                const uint c = i + n.x * (j + n.y * k);
                const float product = airflow_pcg[a0 + c] * airflow_pcg[b0 + c];
                sum = sum + product;
            }
        }
    }
    airflow_partials[t] = sum;
}
