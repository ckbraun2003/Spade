// ---------------------------------------------------------------------------
// airflow_tile_sum.slang (plan B, task B1; B5.1's two-level order) -- THE
// FIXED-ORDER REDUCTION on the GPU: the sum of a cell field over the grid of
// airflow_params' dims (nx x ny x nz, i innermost) in exactly
// physics/airflow/reduce.hpp's fixed_tile_sum_of() order, so it gives the same
// bits as the CPU at every workgroup size (design note §3, A4).
//
// THREE DISPATCHES OF THIS ONE MODULE, chosen by the push's pass:
//
//   pass 0  one thread per TILE of kTile^3 cells (fewer at the far edges):
//           the tile's cells summed from 0.0 in (k, j, i) order, written to
//           airflow_partials[t], t = ti + tnx * (tj + tny * tk);
//   pass 1  one thread per BLOCK b of `block` consecutive tiles (the last
//           block holds the rest): its partials added from 0.0 in increasing
//           t, written over its first tile's partial,
//           airflow_partials[b * block];
//   pass 2  ONE thread: the block sums airflow_partials[b * block] added from
//           0.0 in increasing b, written to airflow_scalars[slot].
//
// THE BLOCK COMES FROM THE PUSH, which the host fills from reduce.hpp's
// kReduceBlockTiles (Cameron, 2026-10-22: blocks of 256 tiles on both twins),
// never from SPADE_WG or a thread count.
//
// A chain records the three with the same push but for the pass, and the
// runner's barrier between them makes each pass's writes visible to the next.
// Passes 1 and 2 read only the partials, so a later reduction that fills them
// its own way (a tile dot) can end with them. The partials need one float a
// tile: pass 1 writes each block's sum in place. One entry point, so the
// workgroup-size variants differ in their LocalSize word only.
//
// SET 1, bindings.slang section F: reads airflow_params (the dims) and the
// field `field` names, airflow_pressure (4) or airflow_rhs (9); writes
// airflow_partials (23) and airflow_scalars (21). airflow_partials is shared
// by the reductions in turn, so a chain reads a sum from airflow_scalars, not
// from the partials.
//
// WHAT KEEPS THE BITS FIXED: the tile is kTile elements on a side and the
// block is the push's tiles, whatever SPADE_WG is; every sum is one thread's
// sequential loop; there are no atomics, no groupshared memory and no
// barrier; and the only arithmetic is OpFAdd, decorated NoContraction by the
// build's -fp-mode precise (with no multiply there is nothing to contract
// into, either). Passes 1 and 2 issue kAhead loads before they add them
// (B5.0b: the same adds in the same order, the fastest at the D1 box).
//
// A MISUSE SHOWS AS NaN: a field number this kernel does not know, or a
// cell_count other than nx * ny * nz, gives NaN partials and so a NaN sum;
// a block of 0 or a pass past 2 writes a NaN sum. Never a plausible one.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;

// The airflow push convention: 16 bytes, the count in word 2.
struct TileSumPush {
    uint field;       // 0 airflow_pressure, 1 airflow_rhs (the residual and CG's vectors are appended later)
    uint pass_block;  // bits 0-7 the pass (0 tiles, 1 blocks, 2 the total); bits 8-31 the block, in tiles
    uint cell_count;  // nx * ny * nz of the params' dims
    uint slot;        // where in airflow_scalars pass 2 writes the sum
};

[[vk::push_constant]] ConstantBuffer<TileSumPush> tile_sum_push;

// physics/airflow/reduce.hpp's kReduceTile, in elements.
static const uint kTile = 4u;

static const uint kTileSumPressure = 0u;
static const uint kTileSumRhs = 1u;

// Loads issued before their adds in passes 1 and 2.
static const int kAhead = 16;

float field_at(uint field, uint c) {
    if (field == kTileSumPressure) { return airflow_pressure[c]; }
    if (field == kTileSumRhs) { return airflow_rhs[c]; }
    return asfloat(0x7FC00000u);  // a quiet NaN: no such field
}

// airflow_partials[first + u * stride] for u = 0 .. count - 1, added from 0.0
// in increasing u. The next kAhead loads are issued before the current kAhead
// are added: the same adds in the same order. A load past the end repeats the
// last element and is never added.
float partials_in_order(uint first, uint stride, uint count) {
    const uint ahead = uint(kAhead);
    const uint full = count - count % ahead;
    float total = 0.0f;
    if (full > 0u) {
        float cur[kAhead];
        [ForceUnroll]
        for (int u = 0; u < kAhead; ++u) { cur[u] = airflow_partials[first + uint(u) * stride]; }
        for (uint t = 0u; t < full; t += ahead) {
            float nxt[kAhead];
            [ForceUnroll]
            for (int u = 0; u < kAhead; ++u) {
                nxt[u] = airflow_partials[first + min(t + ahead + uint(u), count - 1u) * stride];
            }
            [ForceUnroll]
            for (int u = 0; u < kAhead; ++u) { total = total + cur[u]; }
            cur = nxt;
        }
    }
    for (uint t = full; t < count; ++t) { total = total + airflow_partials[first + t * stride]; }
    return total;
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_tile_sum(uint3 tid : SV_DispatchThreadID) {
    const uint3 n = grid_dims();
    const uint tnx = (n.x + kTile - 1u) / kTile;
    const uint tny = (n.y + kTile - 1u) / kTile;
    const uint tnz = (n.z + kTile - 1u) / kTile;
    const uint tiles = tnx * tny * tnz;
    const uint pass = tile_sum_push.pass_block & 0xFFu;
    const uint block = tile_sum_push.pass_block >> 8u;

    if (pass == 0u) {
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

    if (pass == 1u) {
        if (block == 0u) { return; }
        const uint b = tid.x;
        if (b >= (tiles + block - 1u) / block) { return; }
        const uint first = b * block;
        airflow_partials[first] = partials_in_order(first, 1u, min(block, tiles - first));
        return;
    }

    if (tid.x != 0u) { return; }
    if (pass != 2u || block == 0u) {
        airflow_scalars[tile_sum_push.slot] = asfloat(0x7FC00000u);
        return;
    }
    airflow_scalars[tile_sum_push.slot] = partials_in_order(0u, block, (tiles + block - 1u) / block);
}
