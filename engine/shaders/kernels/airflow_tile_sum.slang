// ---------------------------------------------------------------------------
// airflow_tile_sum.slang (plan B, task B1) -- THE FIXED-ORDER REDUCTION on the
// GPU: the sum of a cell field over the grid of airflow_params' dims
// (nx x ny x nz, i innermost) in exactly physics/airflow/reduce.hpp's
// fixed_tile_sum_of() order, so it gives the same bits as the CPU at every
// workgroup size (design note §3, A4).
//
// TWO DISPATCHES OF THIS ONE MODULE, chosen by the push's `pass`:
//
//   pass 0  one thread per TILE of kTile^3 cells (fewer at the far edges):
//           the tile's cells summed from 0.0 in (k, j, i) order, written to
//           airflow_partials[t], t = ti + tnx * (tj + tny * tk);
//   pass 1  ONE thread: the partials added from 0.0 in (tk, tj, ti) order,
//           which is increasing t, written to airflow_scalars[slot].
//
// A chain records both with the same push but for `pass`, and the runner's
// barrier between them makes pass 0's writes visible to pass 1. Pass 1 reads
// only the partials, so a later reduction that fills them its own way (a tile
// dot) can end with it. One entry point, so the workgroup-size variants differ
// in their LocalSize word only.
//
// SET 1, bindings.slang section F: reads airflow_params (the dims) and the
// field `field` names, airflow_pressure (4) or airflow_rhs (9); writes
// airflow_partials (23) and airflow_scalars (21). airflow_partials is shared
// by the reductions in turn, so a chain reads a sum from airflow_scalars, not
// from the partials.
//
// WHAT KEEPS THE BITS FIXED: the tile is kTile elements on a side whatever
// SPADE_WG is; every sum is one thread's sequential loop; there are no
// atomics, no groupshared memory and no tree; and the only arithmetic is
// OpFAdd, decorated NoContraction by the build's -fp-mode precise (with no
// multiply there is nothing to contract into, either).
//
// A MISUSE SHOWS AS NaN: a field number this kernel does not know, or a
// cell_count other than nx * ny * nz, gives NaN partials and so a NaN sum,
// never a plausible one.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;

// The airflow push convention: 16 bytes, the count in word 2.
struct TileSumPush {
    uint field;       // 0 airflow_pressure, 1 airflow_rhs (the residual and CG's vectors are appended later)
    uint pass;        // 0: the tiles into airflow_partials; 1: the partials into airflow_scalars[slot]
    uint cell_count;  // nx * ny * nz of the params' dims
    uint slot;        // where in airflow_scalars pass 1 writes the sum
};

[[vk::push_constant]] ConstantBuffer<TileSumPush> tile_sum_push;

// physics/airflow/reduce.hpp's kReduceTile, in elements.
static const uint kTile = 4u;

static const uint kTileSumPressure = 0u;
static const uint kTileSumRhs = 1u;

float field_at(uint field, uint c) {
    if (field == kTileSumPressure) { return airflow_pressure[c]; }
    if (field == kTileSumRhs) { return airflow_rhs[c]; }
    return asfloat(0x7FC00000u);  // a quiet NaN: no such field
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_tile_sum(uint3 tid : SV_DispatchThreadID) {
    const uint3 n = grid_dims();
    const uint tnx = (n.x + kTile - 1u) / kTile;
    const uint tny = (n.y + kTile - 1u) / kTile;
    const uint tnz = (n.z + kTile - 1u) / kTile;
    const uint tiles = tnx * tny * tnz;

    if (tile_sum_push.pass == 0u) {
        const uint t = tid.x;
        if (t >= tiles) { return; }
        if (tile_sum_push.cell_count != n.x * n.y * n.z) {
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
        float sum = 0.0f;
        for (uint k = k0; k < k_end; ++k) {
            for (uint j = j0; j < j_end; ++j) {
                for (uint i = i0; i < i_end; ++i) {
                    sum = sum + field_at(tile_sum_push.field, i + n.x * (j + n.y * k));
                }
            }
        }
        airflow_partials[t] = sum;
        return;
    }

    if (tid.x != 0u) { return; }
    float total = 0.0f;
    for (uint t = 0u; t < tiles; ++t) {
        total = total + airflow_partials[t];
    }
    airflow_scalars[tile_sum_push.slot] = total;
}
