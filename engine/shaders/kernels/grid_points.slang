// ---------------------------------------------------------------------------
// grid_points.slang (module API stage 7, Task B3) -- trilinear interpolation of a cell-centred field at points. The device
// twin of engine/world/grid_kernels.hpp's grid_points(), in the same order of evaluation (a + (b - a) * t, x then y then
// z; the index clamps to the border; a non-finite coordinate gives NaN). Domain-neutral and Core's, Physics reviews.
//
// SET 1, THE AIRFLOW SET: airflow_u is the field (comps floats per cell, x fastest), airflow_v the points (three floats
// each), airflow_w the output (comps floats per point). One thread per point, 1D (SPADE_WG).
//
// RED STUB (stage 7 Task B3): writes nothing. The interpolation arrives with the GREEN commit.
// ---------------------------------------------------------------------------

import bindings;

struct PointsPush {
    uint nx;
    uint ny;
    uint nz;
    uint comps;
    float ox;
    float oy;
    float oz;
    float inv_cell;
    uint point_count;
    uint unused0;
    uint unused1;
    uint unused2;
};

[[vk::push_constant]] ConstantBuffer<PointsPush> points_push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void grid_points(uint3 tid : SV_DispatchThreadID) {
    if (tid.x >= points_push.point_count) { return; }
}
