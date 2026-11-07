// ---------------------------------------------------------------------------
// lbm_nodes.slang -- the LBM tier's node addressing and box walls on the device
// (L1, the GPU twin, T2 to T4; docs/design/physics/plans/2026-11-06-l1-gpu-twin-
// plan.md). A LINE-CITED PORT of engine/physics/lbm/lbm_grid.hpp (wrapped,
// slot_of, neighbour_cell, incoming_slot) and lbm_flow.hpp (wall_hit, wall_term,
// incoming_walled, velocity_gradient): the same statements in the same order.
//
// The state is lbm_f (bindings.slang 40), 27 x N shifted populations,
// direction-major: the slot of direction i at node (x, y, z) is
// f[i * N + x + nx (y + ny z)]. u is lbm_u (41), component-major (3 x N).
// ONE copy, esoteric pull: an odd step's node reads exactly the slots it writes.
// ---------------------------------------------------------------------------
import bindings;
import fp32_math;
import lbm_lattice;
import lbm_params;

// lbm_grid.hpp lines 58-62: the coordinate v + d wrapped into [0, n), d in {-1, 0, 1}.
uint lbm_wrapped(uint v, int d, uint n) {
    if (d < 0) { return v == 0u ? n - 1u : v - 1u; }
    if (d > 0) { return v + 1u == n ? 0u : v + 1u; }
    return v;
}

// lbm_grid.hpp lines 36-38: cell(x, y, z).
uint lbm_cell(LbmPush p, uint x, uint y, uint z) {
    return x + p.nx * (y + p.ny * z);
}

// lbm_grid.hpp lines 65-67: slot_of(dir, cell).
uint lbm_slot(LbmPush p, uint dir, uint cell) {
    return dir * (p.nx * p.ny * p.nz) + cell;
}

// lbm_grid.hpp lines 70-72: the cell one lattice step along c from (x, y, z), periodic on all faces.
uint lbm_neighbour_cell(LbmPush p, uint x, uint y, uint z, int cx, int cy, int cz) {
    return lbm_cell(p, lbm_wrapped(x, cx, p.nx), lbm_wrapped(y, cy, p.ny), lbm_wrapped(z, cz, p.nz));
}

// lbm_grid.hpp lines 76-83: the slot incoming population `dir` of node (x, y, z) is read from in the step
// about to run (p.parity is the step counter's low bit).
uint lbm_incoming_slot(LbmPush p, uint dir, uint x, uint y, uint z) {
    if (p.parity == 0u) { return lbm_slot(p, dir, lbm_cell(p, x, y, z)); }
    return lbm_slot(p, lbm_opposite(dir), lbm_neighbour_cell(p, x, y, z, -kLbmCx[dir], -kLbmCy[dir], -kLbmCz[dir]));
}

// lbm_flow.hpp lines 60-64 (BoxWalls::any): some axis is not periodic. periodic_mask bit a set = axis a periodic.
bool lbm_any_wall(LbmPush p) {
    return (p.periodic_mask & 7u) != 7u;
}

// lbm_flow.hpp lines 107-130: wall_hit(). Whether the source cell of incoming direction `dir` at the node,
// x - c, is outside the box through a wall, and the mean velocity of the faces it crosses.
struct LbmWallHit {
    bool hit;
    float ux;
    float uy;
    float uz;
};

LbmWallHit lbm_wall_hit(LbmPush p, uint dir, uint x, uint y, uint z) {
    LbmWallHit h;
    h.hit = false;
    h.ux = 0.0f;
    h.uy = 0.0f;
    h.uz = 0.0f;
    if (!lbm_any_wall(p)) { return h; }
    const int pos[3] = {int(x), int(y), int(z)};
    const int cc[3] = {kLbmCx[dir], kLbmCy[dir], kLbmCz[dir]};
    const int n[3] = {int(p.nx), int(p.ny), int(p.nz)};
    int crossed = 0;
    for (int a = 0; a < 3; ++a) {
        if (((p.periodic_mask >> uint(a)) & 1u) != 0u) { continue; }
        const int src = pos[a] - cc[a];
        if (src >= 0 && src < n[a]) { continue; }
        const uint side = src < 0 ? 0u : 1u;
        const uint base = (uint(a) * 2u + side) * 3u;
        h.ux += p.wall_u[base + 0u];
        h.uy += p.wall_u[base + 1u];
        h.uz += p.wall_u[base + 2u];
        ++crossed;
    }
    if (crossed == 0) { return h; }
    h.hit = true;
    const float inv = div32_ftz(1.0f, float(crossed));  // CPU: 1.0f / static_cast<float>(crossed)
    h.ux *= inv;
    h.uy *= inv;
    h.uz *= inv;
    return h;
}

// lbm_flow.hpp lines 132-137: wall_term(dir, uw) = 6 w_i (c_i . u_w), the moving wall's addition to incoming i.
float lbm_wall_term(uint dir, LbmWallHit h) {
    return 6.0f * lbm_weight(dir) * (float(kLbmCx[dir]) * h.ux + float(kLbmCy[dir]) * h.uy + float(kLbmCz[dir]) * h.uz);
}

// lbm_flow.hpp lines 139-144: incoming_walled(). The logical incoming population of `dir` at the node, with the
// walls applied: an even step's slot already holds it (the last odd step put it there), an odd step's wall link
// takes its own slot plus the wall term.
float lbm_incoming_walled(LbmPush p, uint dir, uint x, uint y, uint z) {
    const LbmWallHit h = lbm_wall_hit(p, dir, x, y, z);
    if (!h.hit || p.parity == 0u) { return lbm_f[lbm_incoming_slot(p, dir, x, y, z)]; }
    return lbm_f[lbm_slot(p, dir, lbm_cell(p, x, y, z))] + lbm_wall_term(dir, h);
}

// lbm_flow.hpp lines 177-203: velocity_gradient(): g[3 i + j] = du_i / dx_j by central differences of u over
// the periodic neighbours, past a wall the ghost 2 u_wall - u_node (the wall is halfway).
void lbm_velocity_gradient(LbmPush p, uint x, uint y, uint z, out float gr[9]) {
    const uint here = lbm_cell(p, x, y, z);
    const uint cells = p.nx * p.ny * p.nz;
    const uint coord[3] = {x, y, z};
    const uint extent[3] = {p.nx, p.ny, p.nz};
    for (int j = 0; j < 3; ++j) {
        const int px = j == 0 ? 1 : 0;
        const int py = j == 1 ? 1 : 0;
        const int pz = j == 2 ? 1 : 0;
        const uint cp = lbm_neighbour_cell(p, x, y, z, px, py, pz);
        const uint cm = lbm_neighbour_cell(p, x, y, z, -px, -py, -pz);
        const bool periodic = ((p.periodic_mask >> uint(j)) & 1u) != 0u;
        const bool wall_lo = !periodic && coord[j] == 0u;
        const bool wall_hi = !periodic && coord[j] + 1u == extent[j];
        for (int i = 0; i < 3; ++i) {
            const float u_here = lbm_u[uint(i) * cells + here];
            const float up = wall_hi ? 2.0f * p.wall_u[(uint(j) * 2u + 1u) * 3u + uint(i)] - u_here
                                     : lbm_u[uint(i) * cells + cp];
            const float um = wall_lo ? 2.0f * p.wall_u[(uint(j) * 2u + 0u) * 3u + uint(i)] - u_here
                                     : lbm_u[uint(i) * cells + cm];
            gr[3 * i + j] = 0.5f * (up - um);
        }
    }
}
