// ---------------------------------------------------------------------------
// dispatch_3d_id.slang (module API stage 7, Task B1) -- a TEST kernel: it records which grid cell each thread of a 3D
// dispatch visited, so tests/test_gpu_dispatch_3d.cpp can check that every cell is visited exactly once, whatever the
// grid's remainder, the number of worlds or the workgroup size.
//
// The dispatch is compute/dispatch_shape.hpp's: local shape SPADE_WG_X x SPADE_WG_Y x SPADE_WG_Z, the grid cut by it, and
// the worlds folded into the z groups, each world's z groups contiguous. A thread's world is its z group divided by the
// z groups per world.
//
// Output, on the airflow set (set 1, the ChainRunner's): airflow_u[t] for EVERY thread t of the dispatch, in
// (z, y, x) order over the whole grid of threads, is 0 for a thread outside its world's extent and the cell's linear
// index + 1 otherwise. So the host checks that the non-zero values are 1..cells, each exactly once, and that nothing
// else was written. airflow_v[cell] is a function of the cell index alone (the same bits at every workgroup size).
// Plain stores, no atomics.
// ---------------------------------------------------------------------------

import bindings;

struct IdPush {
    uint nx;
    uint ny;
    uint nz;
    uint worlds;
    uint groups_x;  // the dispatch's group counts, to spell the global thread index
    uint groups_y;
    uint groups_z;
    uint unused;
};

[[vk::push_constant]] ConstantBuffer<IdPush> id_push;

[shader("compute")]
[numthreads(SPADE_WG_X, SPADE_WG_Y, SPADE_WG_Z)]
void dispatch_3d_id(uint3 tid : SV_DispatchThreadID) {
    const uint dim_x = id_push.groups_x * SPADE_WG_X;
    const uint dim_y = id_push.groups_y * SPADE_WG_Y;
    const uint thread = (tid.z * dim_y + tid.y) * dim_x + tid.x;

    const uint z_groups_per_world = (id_push.nz + SPADE_WG_Z - 1u) / SPADE_WG_Z;
    const uint group_z = tid.z / SPADE_WG_Z;
    const uint world = group_z / z_groups_per_world;
    const uint z = (group_z % z_groups_per_world) * SPADE_WG_Z + (tid.z % SPADE_WG_Z);

    if (tid.x >= id_push.nx || tid.y >= id_push.ny || z >= id_push.nz || world >= id_push.worlds) {
        airflow_u[thread] = 0.0;
        return;
    }
    const uint cell = ((world * id_push.nz + z) * id_push.ny + tid.y) * id_push.nx + tid.x;
    airflow_u[thread] = asfloat(cell + 1u);
    airflow_v[cell] = asfloat(cell * 2654435761u + 12345u);
}
