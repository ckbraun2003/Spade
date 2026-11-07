// ---------------------------------------------------------------------------
// lbm_macro.slang (L1, the GPU twin, T2) -- THE LBM'S VELOCITY PASS: lbm_flow.hpp's
// velocity_row()/compute_velocity() (the CPU twin and the authority). One thread
// a node: the node's 27 incoming populations (the parity rule and the moving
// walls, lbm_nodes.slang's incoming_walled), macro_of(), and u written to lbm_u
// (component-major: lbm_u[c N + cell]). It reads lbm_f and writes lbm_u only;
// no node reads what another writes, so there is no ordering and no atomic.
//
// SET 1, bindings.slang section F: lbm_f (40) read, lbm_u (41) written. The push
// block is lbm_params.slang's LbmPush (parity = the step counter's low bit).
// DISPATCH: (ceil(nx / SPADE_WG), ny, nz) groups, a thread an x.
// ---------------------------------------------------------------------------
import bindings;
import lbm_lattice;
import lbm_params;
import lbm_collide;
import lbm_nodes;

[[vk::push_constant]] ConstantBuffer<LbmPush> push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void lbm_macro(uint3 tid : SV_DispatchThreadID) {
    const uint x = tid.x;
    const uint y = tid.y;
    const uint z = tid.z;
    if (x >= push.nx || y >= push.ny || z >= push.nz) { return; }
    // lbm_flow.hpp lines 154-166: velocity_row().
    float f[27];
    for (uint i = 0u; i < 27u; ++i) { f[i] = lbm_incoming_walled(push, i, x, y, z); }
    const LbmMacro m = lbm_macro_of(f);
    const uint cell = lbm_cell(push, x, y, z);
    const uint cells = push.nx * push.ny * push.nz;
    lbm_u[0u * cells + cell] = m.ux;
    lbm_u[1u * cells + cell] = m.uy;
    lbm_u[2u * cells + cell] = m.uz;
}
