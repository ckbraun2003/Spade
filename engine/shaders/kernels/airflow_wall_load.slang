// ---------------------------------------------------------------------------
// airflow_wall_load.slang (plan B, task B5.5) -- THE PRESSURE'S LOAD ON THE
// SOLIDS on the GPU: physics/airflow/wall_load.cpp's wall_load(), its CPU twin
// and the authority (its body runs under the step's FTZ|DAZ, as this device
// flushes):
//   F = sum_c -p_c A_c,   M = sum_c (x_c - about) x (-p_c A_c),
// A_c = dx^2 (theta_hi - theta_lo) per axis on a cell with an open face, 0 on
// a solid one (whose pressure is never read), x_c = ((i, j, k) + 0.5) dx, and
// the cross product glm::cross()'s a*b - c*d. Six fixed-tile sums, one per
// component of F and M, each in reduce.hpp's two-level order (wall_load()'s
// fixed_tile_sum_of<glm::vec3>, per component), written to airflow_wall_load
// (airflow_pcg.slang's kWallLoadForce and kWallLoadMoment). With no solid
// (word 16) the six are 0, as wall_load() returns.
//
// THREE DISPATCHES OF THIS ONE MODULE, as airflow_tile_sum's:
//   pass 0  one thread per TILE: its cells' six terms each summed from 0.0 in
//           (k, j, i) order, written to airflow_partials[q * tiles + t];
//   pass 1  one thread per (q, BLOCK of `block` consecutive tiles): its tiles'
//           partials from 0.0 in increasing t, over its first tile's;
//   pass 2  one thread per q: the blocks' sums from 0.0 in increasing block.
// The partials need six floats a tile. The point the moment is taken about is
// the host's, in airflow_wall_load (kWallLoadAbout), so a chain recorded once
// takes a new point by an upload. THE BLOCK COMES FROM THE PUSH.
//
// SET 1, bindings.slang section F: reads airflow_params (the dims, dx, the
// solid bit), airflow_pressure (4), airflow_apertures (13) and
// airflow_region_labels (18, kNoRegion marks a solid cell); writes
// airflow_partials (23); reads and writes airflow_wall_load (27).
//
// WHAT KEEPS THE BITS FIXED: one thread's sequential loop per sum; no atomics,
// no groupshared memory, no barrier; every product rounds before its sum or
// difference (-fp-mode precise: NoContraction); no division.
//
// A MISUSE SHOWS AS NaN: a cell_count other than nx * ny * nz gives NaN
// partials, and a block of 0 a NaN load.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;

// The airflow push convention: 16 bytes, the count in word 2.
struct WallLoadPush {
    uint unused0;
    uint pass_block;  // as airflow_tile_sum's: bits 0-7 the pass, bits 8-31 the block, in tiles
    uint cell_count;  // nx * ny * nz of the params' dims
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<WallLoadPush> wall_load_push;

// physics/airflow/reduce.hpp's kReduceTile, in cells.
static const uint kTile = 4u;

// F's three components, then M's.
static const uint kTerms = 6u;

// wall_load.cpp's push(): -p_c A_c for cell (i, j, k), or 0 where the cell has
// no open face (its pressure unread). A_c's axis a is area (theta_hi -
// theta_lo), and -p_c times each, as glm's scalar * vec3.
float3 cell_push(uint3 n, float area, uint i, uint j, uint k) {
    const uint c = i + n.x * (j + n.y * k);
    if (solid_cell(c)) { return float3(0.0f, 0.0f, 0.0f); }
    const uint u_lo = i + (n.x + 1u) * (j + n.y * k);
    const uint v_lo = i + n.x * (j + (n.y + 1u) * k);
    const float3 a = float3(area * (face_theta(0u, u_lo + 1u) - face_theta(0u, u_lo)),
                            area * (face_theta(1u, v_lo + n.x) - face_theta(1u, v_lo)),
                            area * (face_theta(2u, c + n.x * n.y) - face_theta(2u, c)));
    const float neg_p = -airflow_pressure[c];
    return float3(neg_p * a.x, neg_p * a.y, neg_p * a.z);
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_wall_load(uint3 tid : SV_DispatchThreadID) {
    const uint3 n = grid_dims();
    const uint tnx = (n.x + kTile - 1u) / kTile;
    const uint tny = (n.y + kTile - 1u) / kTile;
    const uint tnz = (n.z + kTile - 1u) / kTile;
    const uint tiles = tnx * tny * tnz;
    const uint pass = wall_load_push.pass_block & 0xFFu;
    const uint block = wall_load_push.pass_block >> 8u;

    if (pass == 0u) {
        const uint t = tid.x;
        if (t >= tiles || !has_solids()) { return; }
        if (wall_load_push.cell_count != n.x * n.y * n.z) {
            for (uint q = 0u; q < kTerms; ++q) { airflow_partials[q * tiles + t] = asfloat(0x7FC00000u); }
            return;
        }
        const float dx = grid_dx();
        const float area = dx * dx;
        const float3 about = wall_load_about();
        const uint ti = t % tnx;
        const uint tj = (t / tnx) % tny;
        const uint tk = t / (tnx * tny);
        const uint i_end = min(ti * kTile + kTile, n.x);
        const uint j_end = min(tj * kTile + kTile, n.y);
        const uint k_end = min(tk * kTile + kTile, n.z);
        float3 force = float3(0.0f, 0.0f, 0.0f);
        float3 moment = float3(0.0f, 0.0f, 0.0f);
        for (uint k = tk * kTile; k < k_end; ++k) {
            for (uint j = tj * kTile; j < j_end; ++j) {
                for (uint i = ti * kTile; i < i_end; ++i) {
                    const float3 push = cell_push(n, area, i, j, k);
                    force = float3(force.x + push.x, force.y + push.y, force.z + push.z);
                    const float3 x = float3((float(i) + 0.5f) * dx, (float(j) + 0.5f) * dx, (float(k) + 0.5f) * dx);
                    const float3 arm = float3(x.x - about.x, x.y - about.y, x.z - about.z);
                    // glm::cross(arm, push).
                    const float3 turn = float3(arm.y * push.z - push.y * arm.z, arm.z * push.x - push.z * arm.x,
                                               arm.x * push.y - push.x * arm.y);
                    moment = float3(moment.x + turn.x, moment.y + turn.y, moment.z + turn.z);
                }
            }
        }
        airflow_partials[0u * tiles + t] = force.x;
        airflow_partials[1u * tiles + t] = force.y;
        airflow_partials[2u * tiles + t] = force.z;
        airflow_partials[3u * tiles + t] = moment.x;
        airflow_partials[4u * tiles + t] = moment.y;
        airflow_partials[5u * tiles + t] = moment.z;
        return;
    }

    if (pass == 1u) {
        if (block == 0u || !has_solids()) { return; }
        const uint blocks = (tiles + block - 1u) / block;
        if (tid.x >= kTerms * blocks) { return; }
        const uint base = (tid.x / blocks) * tiles;
        const uint first = (tid.x % blocks) * block;
        const uint last = min(first + block, tiles);
        float sum = 0.0f;
        for (uint t = first; t < last; ++t) { sum = sum + airflow_partials[base + t]; }
        airflow_partials[base + first] = sum;
        return;
    }

    const uint q = tid.x;
    if (q >= kTerms) { return; }
    if (pass != 2u || block == 0u) {
        set_wall_load(q, asfloat(0x7FC00000u));
        return;
    }
    if (!has_solids()) {
        set_wall_load(q, 0.0f);
        return;
    }
    float total = 0.0f;
    for (uint first = 0u; first < tiles; first += block) { total = total + airflow_partials[q * tiles + first]; }
    set_wall_load(q, total);
}
