// ---------------------------------------------------------------------------
// grid_shift.slang (module API stage 7, Task B3) -- the integer-shift copy: dst(x, y, z) = src(x + sx, y + sy, z + sz)
// where that cell is inside the grid, `fill` where it is not. Words move as words (no float conversion). The device
// twin of engine/world/grid_kernels.hpp's grid_shift(); domain-neutral and Core's, Physics reviews.
//
// SET 1, THE AIRFLOW SET: airflow_u is the source, airflow_v the destination (one 32-bit word per cell, x fastest).
// One thread per destination cell, 3D (SPADE_WG_X, SPADE_WG_Y, SPADE_WG_Z), one world per dispatch (the host folds worlds
// into the z groups with compute/dispatch_shape.hpp when it needs several).
//
// A load and a store, no arithmetic on the data: the copy carries the word's bits (the build pins NaN preservation, so a NaN
// payload, a negative zero and a denormal arrive as they left). No atomics, no reads of what another thread writes.
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
    const int sx = int(tid.x) + shift_push.sx;
    const int sy = int(tid.y) + shift_push.sy;
    const int sz = int(tid.z) + shift_push.sz;
    const uint to = (tid.z * shift_push.ny + tid.y) * shift_push.nx + tid.x;
    const bool inside = sx >= 0 && sy >= 0 && sz >= 0 && sx < int(shift_push.nx) && sy < int(shift_push.ny) &&
                        sz < int(shift_push.nz);
    if (inside) {
        airflow_v[to] = airflow_u[(uint(sz) * shift_push.ny + uint(sy)) * shift_push.nx + uint(sx)];
    } else {
        airflow_v[to] = asfloat(shift_push.fill);
    }
}
