// ---------------------------------------------------------------------------
// airflow_tile_sum.slang (plan B, task B1) -- THE FIXED-ORDER REDUCTION on the
// GPU: the sum of an nx x ny x nz float array (i innermost) in exactly
// physics/airflow/reduce.hpp's fixed_tile_sum_of() order, so it gives the same
// bits as the CPU at every workgroup size (design note §3, A4).
//
// TWO DISPATCHES OF THIS ONE MODULE, chosen by `phase`:
//
//   phase 0  one thread per TILE of kTile^3 cells (fewer at the far edges):
//            the tile's cells summed from 0.0 in (k, j, i) order, written to
//            tile_sums[t], t = ti + tnx * (tj + tny * tk);
//   phase 1  ONE thread: the tile sums added from 0.0 in (tk, tj, ti) order,
//            which is increasing t, written to tile_total[0].
//
// The runner's barrier between the two dispatches makes phase 0's writes
// visible to phase 1. One entry point, so the workgroup-size variants differ
// in their LocalSize word only.
//
// WHAT KEEPS THE BITS FIXED: the tile is kTile elements on a side whatever
// SPADE_WG is; every sum is one thread's sequential loop; there are no
// atomics, no groupshared memory and no tree; and the only arithmetic is
// OpFAdd, decorated NoContraction by the build's -fp-mode precise (with no
// multiply there is nothing to contract into, either).
//
// Bindings in set 1, the airflow set, as airflow_copy.slang says.
// ---------------------------------------------------------------------------

[[vk::binding(0, 1)]] StructuredBuffer<float> tile_field;
[[vk::binding(1, 1)]] RWStructuredBuffer<float> tile_sums;
[[vk::binding(2, 1)]] RWStructuredBuffer<float> tile_total;

struct TileSumParams {
    uint nx;
    uint ny;
    uint nz;
    uint phase;  // 0: per-tile sums; 1: the total
};

[[vk::push_constant]] ConstantBuffer<TileSumParams> tile_params;

// physics/airflow/reduce.hpp's kReduceTile, in elements.
static const uint kTile = 4u;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_tile_sum(uint3 tid : SV_DispatchThreadID) {
    const uint nx = tile_params.nx;
    const uint ny = tile_params.ny;
    const uint nz = tile_params.nz;
    const uint tnx = (nx + kTile - 1u) / kTile;
    const uint tny = (ny + kTile - 1u) / kTile;
    const uint tnz = (nz + kTile - 1u) / kTile;
    const uint tiles = tnx * tny * tnz;

    if (tile_params.phase == 0u) {
        const uint t = tid.x;
        if (t >= tiles) { return; }
        const uint ti = t % tnx;
        const uint tj = (t / tnx) % tny;
        const uint tk = t / (tnx * tny);
        const uint i0 = ti * kTile;
        const uint j0 = tj * kTile;
        const uint k0 = tk * kTile;
        const uint i_end = min(i0 + kTile, nx);
        const uint j_end = min(j0 + kTile, ny);
        const uint k_end = min(k0 + kTile, nz);
        float sum = 0.0f;
        for (uint k = k0; k < k_end; ++k) {
            for (uint j = j0; j < j_end; ++j) {
                for (uint i = i0; i < i_end; ++i) {
                    sum = sum + tile_field[i + nx * (j + ny * k)];
                }
            }
        }
        tile_sums[t] = sum;
        return;
    }

    if (tid.x != 0u) { return; }
    float total = 0.0f;
    for (uint t = 0u; t < tiles; ++t) {
        total = total + tile_sums[t];
    }
    tile_total[0] = total;
}
