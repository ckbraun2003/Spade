// ---------------------------------------------------------------------------
// airflow_plan_a_measure.slang (0fcb99f9) -- PLAN A'S RESIDUAL MEASURE on the
// GPU: projection.cpp's project() after its n_v cycles (and a closed domain's
// last remove_mean), its CPU twin and the authority:
//   residual(f); r_max = max |f.r[c]|, b_max = max |f.b[c]| over every cell
// of the finest level, each fold std::max's select, (a < x) ? x : a, from 0.0.
// The residual itself is B2's airflow_mg_residual at level 0 (dispatched just
// before, into airflow_residual); this reads that and level 0's b
// (airflow_rhs). Written to airflow_scalars[kSlotRMax] and [kSlotBMax]
// (airflow_pcg.slang); airflow_pcg_scalars' op 2 divides them (div32_ftz) into
// kSlotResidual.
//
// ORDER-FREE, as airflow_pcg_measure is: a max of non-negative floats is exact
// and commutative, and the select never lets a NaN win (a NaN compares false),
// so no partial is ever NaN and any order of folds gives the CPU's bits. It
// runs in airflow_tile_sum's three passes, for the width, not the order:
//   pass 0  one thread per TILE of kTile^3 cells: its cells' folds, written
//           to airflow_partials[2 t] (r_max) and [2 t + 1] (b_max);
//   pass 1  one thread per BLOCK of `block` consecutive tiles (the push's):
//           its tiles' pairs folded over its first tile's pair;
//   pass 2  ONE thread: the blocks' pairs folded, written to the two slots.
// The partials need two floats a tile.
//
// A COPY OF airflow_pcg_measure.slang's BODY, by choice: only the two loads differ
// (airflow_residual and airflow_rhs for pcg_measure's b - q and b, and no solid mask).
// The copy is that kernel's, and so is its proof: tests/test_gpu_airflow_solids.cpp's
// TheMaxNormIsOrderFree (a planted NaN, every workgroup size and block, the CPU's bits).
// A change to one fold or pass belongs in both. Sharing the body with the mask and the
// loads as specialisation constants was weighed and left: it would put the solids
// kernel's SPIR-V, which Physics proved, under an edit for a health word.
//
// THE DEVICE'S OWN max() IS NEVER USED: its NaN rule is not the select's
// (SPIR-V leaves FMax undefined on a NaN operand). So a NaN pressure reports
// 0 here, as on the CPU: the step's scan is what finds one (LiveRun finds it
// at the publish).
//
// SET 1, bindings.slang section F: reads airflow_params (the dims),
// airflow_residual (10) and airflow_rhs (9); writes airflow_partials (23) and
// airflow_scalars (21).
//
// A MISUSE SHOWS AS NaN: a cell_count other than nx * ny * nz, a block of 0
// or a pass past 2 writes NaN slots.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;

// The airflow push convention: 16 bytes, the count in word 2.
struct PlanAMeasurePush {
    uint unused0;
    uint pass_block;  // as airflow_tile_sum's: bits 0-7 the pass, bits 8-31 the block, in tiles
    uint cell_count;  // nx * ny * nz of the params' dims
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<PlanAMeasurePush> measure_push;

// physics/airflow/reduce.hpp's kReduceTile, in cells.
static const uint kTile = 4u;

// std::max(a, x): x when a < x, else a.
float fold(float a, float x) { return (a < x) ? x : a; }

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_plan_a_measure(uint3 tid : SV_DispatchThreadID) {
    const uint3 n = grid_dims();
    const uint tnx = (n.x + kTile - 1u) / kTile;
    const uint tny = (n.y + kTile - 1u) / kTile;
    const uint tnz = (n.z + kTile - 1u) / kTile;
    const uint tiles = tnx * tny * tnz;
    const uint pass = measure_push.pass_block & 0xFFu;
    const uint block = measure_push.pass_block >> 8u;

    if (pass == 0u) {
        const uint t = tid.x;
        if (t >= tiles) { return; }
        if (measure_push.cell_count != n.x * n.y * n.z) {
            airflow_partials[2u * t] = asfloat(0x7FC00000u);
            airflow_partials[2u * t + 1u] = asfloat(0x7FC00000u);
            return;
        }
        const uint ti = t % tnx;
        const uint tj = (t / tnx) % tny;
        const uint tk = t / (tnx * tny);
        const uint i_end = min(ti * kTile + kTile, n.x);
        const uint j_end = min(tj * kTile + kTile, n.y);
        const uint k_end = min(tk * kTile + kTile, n.z);
        float r_max = 0.0f;
        float b_max = 0.0f;
        for (uint k = tk * kTile; k < k_end; ++k) {
            for (uint j = tj * kTile; j < j_end; ++j) {
                for (uint i = ti * kTile; i < i_end; ++i) {
                    const uint c = i + n.x * (j + n.y * k);
                    r_max = fold(r_max, abs(airflow_residual[c]));
                    b_max = fold(b_max, abs(airflow_rhs[c]));
                }
            }
        }
        airflow_partials[2u * t] = r_max;
        airflow_partials[2u * t + 1u] = b_max;
        return;
    }

    if (pass == 1u) {
        if (block == 0u) { return; }
        const uint b = tid.x;
        if (b >= (tiles + block - 1u) / block) { return; }
        const uint first = b * block;
        const uint last = min(first + block, tiles);
        float r_max = 0.0f;
        float b_max = 0.0f;
        for (uint t = first; t < last; ++t) {
            r_max = fold(r_max, airflow_partials[2u * t]);
            b_max = fold(b_max, airflow_partials[2u * t + 1u]);
        }
        airflow_partials[2u * first] = r_max;
        airflow_partials[2u * first + 1u] = b_max;
        return;
    }

    if (tid.x != 0u) { return; }
    if (pass != 2u || block == 0u) {
        airflow_scalars[kSlotRMax] = asfloat(0x7FC00000u);
        airflow_scalars[kSlotBMax] = asfloat(0x7FC00000u);
        return;
    }
    float r_max = 0.0f;
    float b_max = 0.0f;
    for (uint first = 0u; first < tiles; first += block) {
        r_max = fold(r_max, airflow_partials[2u * first]);
        b_max = fold(b_max, airflow_partials[2u * first + 1u]);
    }
    airflow_scalars[kSlotRMax] = r_max;
    airflow_scalars[kSlotBMax] = b_max;
}
