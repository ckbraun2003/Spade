// ---------------------------------------------------------------------------
// airflow_remove_mean.slang (plan B, task B2) -- a closed domain's gauge:
// reduce.cpp's remove_mean(), its CPU twin and the authority for every
// expression. mean = fixed_tile_sum(x) * inv_count, then x -= mean for every
// cell. A closed domain (no pressure outlet) fixes p only up to a constant, so
// project() removes b's mean before the cycles and p's after each one.
//
// THE SUM IS airflow_tile_sum's (Physics' B1, on section F): the field the
// push names, summed in reduce.hpp's fixed 4^3 tile order into
// airflow_scalars[slot]; slot 0 is this sum (Physics, 2026-10-22). This kernel
// is the subtraction after it: mean = airflow_scalars[slot] * inv_count, with
// inv_count the CPU's own host reciprocal, float(1.0 / double(n))
// (airflow_params_layout's mean_inv_count()),
// so the mean, and every subtraction, is the CPU's bit for bit.
//
// One invocation per cell; the push's count (word 2) is the grid's cells.
// field: 0 airflow_pressure, 1 airflow_rhs (tile_sum's selectors).
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;

struct RemoveMeanPush {
    uint field;  // 0 airflow_pressure, 1 airflow_rhs
    uint unused;
    uint count;  // the grid's cells
    uint slot;   // the sum's airflow_scalars slot
};

[[vk::push_constant]] ConstantBuffer<RemoveMeanPush> remove_mean_push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_remove_mean(uint3 tid : SV_DispatchThreadID) {
    const uint c = tid.x;
    if (c >= remove_mean_push.count) { return; }
    const float mean = airflow_scalars[remove_mean_push.slot] * mean_inv_count();
    if (remove_mean_push.field == 0u) {
        airflow_pressure[c] = airflow_pressure[c] - mean;
    } else {
        airflow_rhs[c] = airflow_rhs[c] - mean;
    }
}
