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
// FOUR DISPATCHES OF THIS ONE MODULE, chosen by the push's pass, over the
// region table's tile lists (airflow_pcg.slang's layout, from word L, the
// push's lists): C the closed regions (the pcg block's count), j a closed
// region (the table's index), a pair p one (j, tile t) with a cell of region
// j in tile t (kTile^3 cells, fewer at the far edges), a group g region j's
// pairs in one block of kReduceBlockTiles consecutive t:
//
//   pass 0  one thread per pair p: tile t's cells of region j summed from
//           0.0 in (k, j, i) order, into partials[p];
//   pass 1  one thread per group g: its pairs' partials from 0.0 in
//           increasing t, written over its first pair's;
//   pass 2  one thread per j: its groups' sums from 0.0 in increasing block,
//           times the host's reciprocal (the table's), written to the table
//           as region j's mean;
//   pass 3  one thread per cell: a cell of closed region j takes x - mean(j).
//
// THE LISTS ARE THE CPU'S SKIPS. remove_region_means() adds, for region j,
// only the tiles holding a cell of j and only the blocks holding such a
// tile; the pairs are exactly those tiles and the groups exactly those
// blocks, so a tile or block without j's cells adds nothing (not a +0.0,
// which would turn a sum flushed to -0.0 into +0.0). The host builds the
// lists once from the labels with reduce.hpp's kReduceTile and
// kReduceBlockTiles (solids_chain.hpp's pack_region_table()); whatever
// relabels the cells packs the table again. The partials need a float a
// pair.
//
// SET 1, bindings.slang section F: reads airflow_params (the dims and the pcg
// block's closed-region count) and airflow_region_labels (18); reads and
// writes airflow_region_table (19, the lists and the means), airflow_pcg's
// vector (20) and airflow_partials (23).
//
// WHAT KEEPS THE BITS FIXED: every sum is one thread's sequential loop in the
// CPU's order; no atomics, no groupshared memory, no barrier; the adds, the
// multiply and the subtract are OpFAdd, OpFMul and OpFSub, decorated
// NoContraction by the build's -fp-mode precise. No division: the reciprocal
// is the host's. The workgroup size only chooses which thread runs which
// loop.
//
// A MISUSE SHOWS AS NaN: a vector past airflow_pcg's seven or a cell_count
// other than nx * ny * nz gives NaN partials and so NaN means (and, when the
// vector is one of the seven, NaN in the cells pass 3 gauges).
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;

// The airflow push convention: 16 bytes, the count in word 2.
struct RegionMeanPush {
    uint vector;      // the airflow_pcg vector gauged (airflow_pcg.slang's kPcg*)
    uint pass_block;  // bits 0-7 the pass (0 pairs, 1 groups, 2 the means, 3 the subtract); bits 8-31 unread
    uint cell_count;  // nx * ny * nz of the params' dims
    uint lists;       // L: the region table's word where the tile lists start (passes 0-2)
};

[[vk::push_constant]] ConstantBuffer<RegionMeanPush> region_mean_push;

// physics/airflow/reduce.hpp's kReduceTile, in cells.
static const uint kTile = 4u;

// The tile lists' words (airflow_pcg.slang's layout).
uint list_word(uint at) { return airflow_region_table[region_mean_push.lists + at]; }

// Closed region j's first group (j <= C).
uint first_group(uint j) { return list_word(kRegionListHeader + j); }

// Group g's first pair (g <= G).
uint first_pair(uint closed, uint g) { return list_word(kRegionListHeader + closed + 1u + g); }

// Pair p's word w (kRegionPairTile or kRegionPairClosed).
uint pair_word(uint closed, uint groups, uint p, uint w) {
    return list_word(kRegionListHeader + closed + 1u + groups + 1u + kRegionPairWords * p + w);
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_region_mean(uint3 tid : SV_DispatchThreadID) {
    const uint3 n = grid_dims();
    const uint closed = pcg_closed_regions();
    const uint pass = region_mean_push.pass_block & 0xFFu;
    const uint v = region_mean_push.vector;
    const bool misuse = region_mean_push.cell_count != n.x * n.y * n.z || v >= kPcgVectors;

    if (pass == 0u) {
        const uint pairs = list_word(kRegionListPairs);
        const uint p = tid.x;
        if (p >= pairs) { return; }
        if (misuse) {
            airflow_partials[p] = asfloat(0x7FC00000u);
            return;
        }
        const uint groups = list_word(kRegionListGroups);
        const uint t = pair_word(closed, groups, p, kRegionPairTile);
        const uint id = region_of_closed(pair_word(closed, groups, p, kRegionPairClosed));
        const uint tnx = (n.x + kTile - 1u) / kTile;
        const uint tny = (n.y + kTile - 1u) / kTile;
        const uint ti = t % tnx;
        const uint tj = (t / tnx) % tny;
        const uint tk = t / (tnx * tny);
        const uint i_end = min(ti * kTile + kTile, n.x);
        const uint j_end = min(tj * kTile + kTile, n.y);
        const uint k_end = min(tk * kTile + kTile, n.z);
        const uint x0 = pcg_offset(v);
        float sum = 0.0f;
        for (uint k = tk * kTile; k < k_end; ++k) {
            for (uint jj = tj * kTile; jj < j_end; ++jj) {
                for (uint i = ti * kTile; i < i_end; ++i) {
                    const uint c = i + n.x * (jj + n.y * k);
                    if (airflow_region_labels[c] != id) { continue; }
                    sum = sum + airflow_pcg[x0 + c];
                }
            }
        }
        airflow_partials[p] = sum;
        return;
    }

    if (pass == 1u) {
        const uint g = tid.x;
        if (g >= list_word(kRegionListGroups)) { return; }
        const uint first = first_pair(closed, g);
        const uint last = first_pair(closed, g + 1u);
        float sum = 0.0f;
        for (uint p = first; p < last; ++p) { sum = sum + airflow_partials[p]; }
        airflow_partials[first] = sum;
        return;
    }

    if (pass == 2u) {
        const uint j = tid.x;
        if (j >= closed) { return; }
        const uint last = first_group(j + 1u);
        float total = 0.0f;
        for (uint g = first_group(j); g < last; ++g) { total = total + airflow_partials[first_pair(closed, g)]; }
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
