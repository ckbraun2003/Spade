// ---------------------------------------------------------------------------
// points_positions.slang (module API stage 7, Task D1) -- the points.positions pass: each BODY slot's position into its
// airflow_points row, the device twin of physics/schedule.cpp's pass_points_positions(). One thread per body slot (dead
// slots included: they hold zeros, as the CPU's rows do), no arithmetic, so the rows equal the CPU's bit for bit.
//
// Row = world * rows_per_world + the slot's world-local index (rows_per_world rides in the `sweep` lane, StepShape::
// point_rows); words 0..2 the position, word 3 the reserved 0.0f. Wind and density (4..7) are the provider's, and the
// element x k and sensor rows are the declaring module's (L43): this kernel writes body rows only.
// ---------------------------------------------------------------------------
import layouts;
import bindings;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void points_positions(uint3 tid : SV_DispatchThreadID) {
    const uint slot = tid.x;
    if (slot >= pass_params.world_count * pass_params.body_capacity) {
        return;
    }
    const uint world = slot / pass_params.body_capacity;
    const uint local_body = slot - world * pass_params.body_capacity;
    const uint row = (world * pass_params.sweep + local_body) * 8u;
    const float3 pos = bodies[slot].pos;
    airflow_points[row + 0u] = pos.x;
    airflow_points[row + 1u] = pos.y;
    airflow_points[row + 2u] = pos.z;
    airflow_points[row + 3u] = 0.0f;
}
