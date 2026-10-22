// ---------------------------------------------------------------------------
// airflow_tile_sum_probe.slang (plan B, B5.0b) -- TEST ONLY, a measurement of
// two ways to end the fixed-order sum faster. No chain of the solver records
// it; tests/test_gpu_airflow_runner.cpp's opt-in B5.0b test does
// (SPADE_AIRFLOW_TIMINGS=1). airflow_tile_sum.slang is the production kernel
// and is not changed.
//
//   pass 0  one thread per TILE, exactly airflow_tile_sum's pass 0 (the field
//           is airflow_pressure): airflow_partials[t], t in (tk, tj, ti) order.
//   pass 1  (i) THE ORDER KEPT: one thread adds the partials from 0.0 in
//           increasing t, as airflow_tile_sum's pass 1 does, but issues
//           `ahead` loads (8, 16, 32 or 64) before it adds them, and the next
//           `ahead` before it adds those: the same adds in the same order, so
//           the same bits as reduce.hpp. `ahead` 1 is the plain loop.
//   pass 2  (ii) THE TWO-LEVEL ORDER, level 1: one thread per BLOCK of `block`
//           consecutive tiles (the last block holds the rest), its partials
//           added from 0.0 in increasing t, written to
//           airflow_partials[tiles + b].
//   pass 3  (ii) level 2: one thread adds the block sums from 0.0 in
//           increasing b, written to airflow_scalars[kSlot].
//
// The block size comes from the push, which the host fills from ONE constant
// (the test's kTreeBlockTiles; on a scratch CPU twin, reduce.hpp's), never
// from SPADE_WG or a thread count: every workgroup size and the CPU add in
// the same order. No atomics, no groupshared memory, no barrier: every sum is
// one thread's loop, and the only arithmetic is OpFAdd.
//
// SET 1, bindings.slang section F: reads airflow_params (the dims) and
// airflow_pressure (4); writes airflow_partials (23), sized 2 x tiles floats
// for the block sums, and airflow_scalars (21).
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;

struct TileSumProbePush {
    uint pass;        // 0 tiles, 1 the kept order's chain, 2 block sums, 3 the block sums' total
    uint ahead;       // loads issued before their adds: 1, 8, 16, 32 or 64
    uint cell_count;  // nx * ny * nz of the params' dims
    uint block;       // passes 2-3: the tiles in a block
};

[[vk::push_constant]] ConstantBuffer<TileSumProbePush> probe_push;

static const uint kTile = 4u;  // reduce.hpp's kReduceTile
static const uint kSlot = 1u;  // airflow_scalars' slot for the total

// partials[base] + ... + partials[base + count - 1], added from 0.0 in
// increasing index, with N loads in flight: the next N are loaded before the
// current N are added. Loads past the end are clamped to the last element and
// never added.
float chain_ahead<let N : int>(uint base, uint count) {
    const uint n = uint(N);
    const uint full = count - count % n;
    float total = 0.0f;
    if (full > 0u) {
        float cur[N];
        [ForceUnroll]
        for (int u = 0; u < N; ++u) { cur[u] = airflow_partials[base + uint(u)]; }
        for (uint t = 0u; t < full; t += n) {
            float nxt[N];
            const uint tn = t + n;
            [ForceUnroll]
            for (int u = 0; u < N; ++u) { nxt[u] = airflow_partials[base + min(tn + uint(u), count - 1u)]; }
            [ForceUnroll]
            for (int u = 0; u < N; ++u) { total = total + cur[u]; }
            cur = nxt;
        }
    }
    for (uint t = full; t < count; ++t) { total = total + airflow_partials[base + t]; }
    return total;
}

float chain(uint base, uint count, uint ahead) {
    if (ahead == 8u) { return chain_ahead<8>(base, count); }
    if (ahead == 16u) { return chain_ahead<16>(base, count); }
    if (ahead == 32u) { return chain_ahead<32>(base, count); }
    if (ahead == 64u) { return chain_ahead<64>(base, count); }
    float total = 0.0f;
    for (uint t = 0u; t < count; ++t) { total = total + airflow_partials[base + t]; }
    return total;
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_tile_sum_probe(uint3 tid : SV_DispatchThreadID) {
    const uint3 n = grid_dims();
    const uint tnx = (n.x + kTile - 1u) / kTile;
    const uint tny = (n.y + kTile - 1u) / kTile;
    const uint tnz = (n.z + kTile - 1u) / kTile;
    const uint tiles = tnx * tny * tnz;
    const uint pass = probe_push.pass;

    if (pass == 0u) {
        const uint t = tid.x;
        if (t >= tiles) { return; }
        if (probe_push.cell_count != n.x * n.y * n.z) {
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
                    sum = sum + airflow_pressure[i + n.x * (j + n.y * k)];
                }
            }
        }
        airflow_partials[t] = sum;
        return;
    }

    const uint block = probe_push.block;
    if (pass == 2u) {
        if (block == 0u) { return; }
        const uint b = tid.x;
        const uint first = b * block;
        if (first >= tiles) { return; }
        airflow_partials[tiles + b] = chain(first, min(block, tiles - first), probe_push.ahead);
        return;
    }

    if (tid.x != 0u) { return; }
    if (pass == 1u) {
        airflow_scalars[kSlot] = chain(0u, tiles, probe_push.ahead);
        return;
    }
    if (pass == 3u) {
        if (block == 0u) {
            airflow_scalars[kSlot] = asfloat(0x7FC00000u);
            return;
        }
        airflow_scalars[kSlot] = chain(tiles, (tiles + block - 1u) / block, probe_push.ahead);
    }
}
