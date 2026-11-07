// ---------------------------------------------------------------------------
// lbm_collide_stream.slang (L1, the GPU twin, T3 and T4) -- THE LBM'S COLLIDE-AND-
// STREAM PASS: lbm_flow.hpp's collide_row() (the CPU twin and the authority). One
// thread a node: the 27 incoming populations (incoming_walled), WALE's nu_t from
// the velocity gradient of lbm_u (past a wall the ghost 2 u_wall - u_node),
// collide_rr3 with omega = 1 / (3 (nu + nu_t) + 1/2), and the esoteric-pull write:
//   EVEN step: f*_i -> slot [i-bar][x];
//   ODD  step: f*_j -> slot [j][x + c_j], or, where the target is outside the box
//              through a wall, slot [j-bar][x] plus the moving-wall term.
// THE SLOTS A NODE WRITES ARE THE SLOTS IT READ (the esoteric rule), so nodes in
// any order, or at once, give the same bits: there is no barrier, atomic, scratch
// copy or reduction. lbm_u is READ ONLY in this pass; lbm_macro wrote it before
// (the chain's barrier between the two dispatches).
//
// SET 1, bindings.slang section F: lbm_f (40) read and written, lbm_u (41) read.
// The push block is lbm_params.slang's LbmPush. DISPATCH as lbm_macro's.
// ---------------------------------------------------------------------------
import bindings;
import lbm_lattice;
import lbm_params;
import lbm_collide;
import lbm_nodes;

[[vk::push_constant]] ConstantBuffer<LbmPush> push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void lbm_collide_stream(uint3 tid : SV_DispatchThreadID) {
    const uint x = tid.x;
    const uint y = tid.y;
    const uint z = tid.z;
    if (x >= push.nx || y >= push.ny || z >= push.nz) { return; }
    // lbm_flow.hpp lines 206-233: collide_row().
    float f[27];
    for (uint i = 0u; i < 27u; ++i) { f[i] = lbm_incoming_walled(push, i, x, y, z); }
    float nu_t = 0.0f;
    if (push.cw > 0.0f) {
        float gr[9];
        lbm_velocity_gradient(push, x, y, z, gr);
        nu_t = lbm_wale_nu_t(gr, push.cw);
    }
    lbm_collide_rr3(f, lbm_omega_of(push.nu, nu_t));
    const uint here = lbm_cell(push, x, y, z);
    if (push.parity == 0u) {
        for (uint i = 0u; i < 27u; ++i) { lbm_f[lbm_slot(push, lbm_opposite(i), here)] = f[i]; }
    } else {
        for (uint j = 0u; j < 27u; ++j) {
            // The target x + c_j is outside through a wall exactly when incoming j-bar's source is.
            const LbmWallHit h = lbm_wall_hit(push, lbm_opposite(j), x, y, z);
            if (h.hit) {
                lbm_f[lbm_slot(push, lbm_opposite(j), here)] = f[j] + lbm_wall_term(lbm_opposite(j), h);
            } else {
                lbm_f[lbm_slot(push, j, lbm_neighbour_cell(push, x, y, z, kLbmCx[j], kLbmCy[j], kLbmCz[j]))] = f[j];
            }
        }
    }
}
