// ---------------------------------------------------------------------------
// grid_shift.slang (module API stage 7, Task B3) -- the integer-shift copy: dst(x, y, z) = src(x + sx, y + sy, z + sz)
// where that cell is inside the grid, `fill` where it is not. Words move as words (no float conversion). The device
// twin of engine/world/grid_kernels.hpp's grid_shift(); domain-neutral and Core's, Physics reviews.
//
// SET 1, THE AIRFLOW SET: airflow_u is the source, airflow_v the destination (one 32-bit word per cell, x fastest).
// One thread per destination cell, 3D (SPADE_WG_X, SPADE_WG_Y, SPADE_WG_Z), one world.
//
// RED STUB (stage 7 Task B3): writes nothing. The shift arrives with the GREEN commit.
// ---------------------------------------------------------------------------

import bindings;

struct ShiftPush {
    uint nx;
    uint ny;
    uint nz;
    uint fill;
    int sx;
    int sy;
    int sz;
    uint unused;
};

[[vk::push_constant]] ConstantBuffer<ShiftPush> shift_push;

[shader("compute")]
[numthreads(SPADE_WG_X, SPADE_WG_Y, SPADE_WG_Z)]
void grid_shift(uint3 tid : SV_DispatchThreadID) {
    if (tid.x >= shift_push.nx || tid.y >= shift_push.ny || tid.z >= shift_push.nz) { return; }
}
