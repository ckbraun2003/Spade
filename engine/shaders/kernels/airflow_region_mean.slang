// ---------------------------------------------------------------------------
// airflow_region_mean.slang (plan B, task B5.3) -- THE CLOSED REGIONS' MEANS
// on the GPU: physics/airflow/regions.cpp's remove_region_means(), its CPU
// twin and the authority, over one of airflow_pcg's vectors (airflow_pcg.slang):
// each closed region's sum in reduce.hpp's two-level order restricted to its
// cells, times the host's reciprocal of its cell count, then subtracted from
// its cells. A pinned region (an outlet reaches it) and a solid cell are left
// alone. The solids CG's gauge (projection.cpp's gauge_regions()) and the
// right side's means are this.
//
// FOUR DISPATCHES OF THIS ONE MODULE, chosen by the push's pass, C the closed
// regions (the pcg block's count), j a closed region (airflow_region_table's
// index), t a tile of kTile^3 cells (fewer at the far edges):
//
//   pass 0  one thread per (j, t): the tile's cells of region j summed from
//           0.0 in (k, j, i) order, and whether the tile met region j;
//   pass 1  one thread per (j, block b of `block` consecutive tiles): the
//           partials of the tiles that met j, from 0.0 in increasing t,
//           written over the block's first tile's, with whether any did;
//   pass 2  one thread per j: the sums of the blocks that met j, from 0.0 in
//           increasing b, times the host's reciprocal (the table's), written
//           to the table as region j's mean;
//   pass 3  one thread per cell: a cell of closed region j takes x - mean(j).
//
// A tile or a block without region j's cells adds nothing to its sum (not a
// +0.0, which would turn a sum flushed to -0.0 into +0.0), as the CPU's
// in_block and touched lists say. The partials need 2 C floats a tile: region
// j's sums at j * tiles + t, and its met flags (1.0 or 0.0) at (C + j) * tiles
// + t. THE BLOCK COMES FROM THE PUSH, filled from reduce.hpp's
// kReduceBlockTiles, never from SPADE_WG or a thread count.
//
// SET 1, bindings.slang section F: reads airflow_params (the dims and the pcg
// block's closed-region count) and airflow_region_labels (18); reads and
// writes airflow_region_table (19, the means), airflow_pcg's vector (20) and
// airflow_partials (23).
//
// WHAT KEEPS THE BITS FIXED: every sum is one thread's sequential loop in the
// CPU's order; no atomics, no groupshared memory, no barrier; the adds, the
// multiply and the subtract are OpFAdd, OpFMul and OpFSub, decorated
// NoContraction by the build's -fp-mode precise. No division: the reciprocal
// is the host's.
//
// A MISUSE SHOWS AS NaN: a vector past airflow_pcg's seven or a cell_count
// other than nx * ny * nz gives NaN partials and so NaN means (and, when the
// vector is one of the seven, NaN in the cells pass 3 gauges); a block of 0
// gives NaN means.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;

// The airflow push convention: 16 bytes, the count in word 2.
struct RegionMeanPush {
    uint vector;      // the airflow_pcg vector gauged (airflow_pcg.slang's kPcg*)
    uint pass_block;  // bits 0-7 the pass (0 tiles, 1 blocks, 2 the means, 3 the subtract); bits 8-31 the block, in tiles
    uint cell_count;  // nx * ny * nz of the params' dims
    uint unused;
};

[[vk::push_constant]] ConstantBuffer<RegionMeanPush> region_mean_push;

// physics/airflow/reduce.hpp's kReduceTile, in cells.
static const uint kTile = 4u;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_region_mean(uint3 tid : SV_DispatchThreadID) {
    const uint3 n = grid_dims();
    const uint tnx = (n.x + kTile - 1u) / kTile;
    const uint tny = (n.y + kTile - 1u) / kTile;
    const uint tnz = (n.z + kTile - 1u) / kTile;
    const uint tiles = tnx * tny * tnz;
    const uint closed = pcg_closed_regions();
    const uint pass = region_mean_push.pass_block & 0xFFu;
    const uint block = region_mean_push.pass_block >> 8u;
    const uint v = region_mean_push.vector;
    const bool misuse = region_mean_push.cell_count != n.x * n.y * n.z || v >= kPcgVectors;

    if (pass == 0u) {
        if (tid.x >= closed * tiles) { return; }
        const uint j = tid.x / tiles;
        const uint t = tid.x % tiles;
        const uint sum_at = j * tiles + t;
        const uint met_at = (closed + j) * tiles + t;
        if (misuse) {
            airflow_partials[sum_at] = asfloat(0x7FC00000u);
            airflow_partials[met_at] = 1.0f;
            return;
        }
        const uint id = region_of_closed(j);
        const uint ti = t % tnx;
        const uint tj = (t / tnx) % tny;
        const uint tk = t / (tnx * tny);
        const uint i_end = min(ti * kTile + kTile, n.x);
        const uint j_end = min(tj * kTile + kTile, n.y);
        const uint k_end = min(tk * kTile + kTile, n.z);
        const uint x0 = pcg_offset(v);
        float sum = 0.0f;
        bool met = false;
        for (uint k = tk * kTile; k < k_end; ++k) {
            for (uint jj = tj * kTile; jj < j_end; ++jj) {
                for (uint i = ti * kTile; i < i_end; ++i) {
                    const uint c = i + n.x * (jj + n.y * k);
                    if (airflow_region_labels[c] != id) { continue; }
                    sum = sum + airflow_pcg[x0 + c];
                    met = true;
                }
            }
        }
        airflow_partials[sum_at] = sum;
        airflow_partials[met_at] = met ? 1.0f : 0.0f;
        return;
    }

    if (pass == 1u) {
        if (block == 0u) { return; }
        const uint blocks = (tiles + block - 1u) / block;
        if (tid.x >= closed * blocks) { return; }
        const uint j = tid.x / blocks;
        const uint first = (tid.x % blocks) * block;
        const uint last = min(first + block, tiles);
        const uint sums = j * tiles;
        const uint mets = (closed + j) * tiles;
        float sum = 0.0f;
        bool met = false;
        for (uint t = first; t < last; ++t) {
            if (airflow_partials[mets + t] == 0.0f) { continue; }
            sum = sum + airflow_partials[sums + t];
            met = true;
        }
        airflow_partials[sums + first] = sum;
        airflow_partials[mets + first] = met ? 1.0f : 0.0f;
        return;
    }

    if (pass == 2u) {
        const uint j = tid.x;
        if (j >= closed) { return; }
        if (block == 0u) {
            set_region_mean(j, asfloat(0x7FC00000u));
            return;
        }
        const uint sums = j * tiles;
        const uint mets = (closed + j) * tiles;
        float total = 0.0f;
        for (uint first = 0u; first < tiles; first += block) {
            if (airflow_partials[mets + first] == 0.0f) { continue; }
            total = total + airflow_partials[sums + first];
        }
        set_region_mean(j, total * region_reciprocal(j));
        return;
    }

    if (pass != 3u || v >= kPcgVectors) { return; }
    const uint c = tid.x;
    if (c >= n.x * n.y * n.z) { return; }
    const uint id = airflow_region_labels[c];
    if (id == kNoRegion) { return; }
    const uint j = region_closed_index(id);
    if (j == kNoRegion) { return; }
    pcg_set(v, c, pcg_get(v, c) - region_mean(j));
}
