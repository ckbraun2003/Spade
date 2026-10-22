// ---------------------------------------------------------------------------
// airflow_viscous.slang (plan B, task B3) -- the fluid step's implicit
// viscosity on the GPU: engine/physics/airflow/viscous.cpp's
// diffuse_component(), term for term (its CPU twin and the authority for every
// expression). (I - alpha L) x = x0 by red-black Gauss-Seidel, component by
// component, alpha = nu h / dx^2 the host's.
//
// One component per dispatch, in three modes:
//   mode 0, prepare: per face, a fixed sample (closed theta or phi < 0) takes
//           its wall velocity, and the right side is the face's value after
//           that (viscous.cpp's fill, then its copy into rhs);
//   mode 1, sweep:   one colour of one sweep, one invocation per face of the
//           colour; the chain runs colour 0 then 1, visc_sweeps times;
//   mode 2, alias:   on a periodic own axis, face n takes face 0's value.
// Reads airflow_params and, with solids, airflow_apertures, airflow_phi,
// airflow_wall_velocity, airflow_face_tags and airflow_wall_links; reads and
// writes airflow_u, airflow_v or airflow_w and airflow_viscous_rhs (set 1,
// bindings.slang section F). The solid arrays follow B5's packing (Physics,
// 2026-10-22): per face, u then v then w from component_base(c); face_tags'
// word component_base(c) + idx the wall-link index or kNoWallLinks; a link
// 12 words, inv_f[0..5] then wall[0..5]. airflow_viscous_rhs is packed the
// same way.
//
// THE COLOURS. A sample's six neighbours differ from it by one in one index,
// so they are the other colour, and across a periodic wrap too: a periodic
// ring is even or a single cell (check_shape()), and a single cell is its
// own neighbour, which the stencil skips. So one invocation per sample of the
// colour gives the CPU's in-place sweep exactly.
//
// THE STENCIL. viscous.cpp's interior fast path makes the general body's
// additions in the general body's order, with the same diagonal (1 + alpha
// six times) and a literal + 0.0 for `extra`, so its general body alone is
// ported: sum, diag and extra accumulated over a = 0..2, dir = 0..1.
//
// PARITY. -fp-mode precise keeps the arithmetic uncontracted; every add and
// multiply rounds correctly, and the one division is fp32_math's div32_ftz,
// divss under the CPU step's FTZ|DAZ to the bit, so a sweep is diffuse()'s
// bit for bit (TD-14 as amended 2026-10-22), and the module is scanned under
// the exact profile.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

struct ViscousPush {
    uint component;   // 0 u, 1 v, 2 w
    uint mode;        // 0 prepare, 1 sweep, 2 alias
    uint colour;      // mode 1: 0 red, 1 black
    uint count;       // the invocations the mode needs
    uint alpha;       // the host's nu h / dx^2 (float bits)
    uint unused0;
    uint unused1;
    uint unused2;
    // Per side (-x, +x, -y, +y, -z, +z), the ghost value a fixed-velocity
    // side gives this component: a wall's wall_velocity[side][c], an inflow's
    // ambient[c] (float bits). Scalars, not an array: no layout rule can pad
    // them.
    uint ghost0;
    uint ghost1;
    uint ghost2;
    uint ghost3;
    uint ghost4;
    uint ghost5;
};

[[vk::push_constant]] ConstantBuffer<ViscousPush> viscous_push;

static const uint kNoWallLinks = 0xFFFFFFFFu;  // physics/airflow/solids.hpp's

float ghost(uint side) {
    const uint g = side == 0u ? viscous_push.ghost0
                 : side == 1u ? viscous_push.ghost1
                 : side == 2u ? viscous_push.ghost2
                 : side == 3u ? viscous_push.ghost3
                 : side == 4u ? viscous_push.ghost4
                              : viscous_push.ghost5;
    return asfloat(g);
}

// --- component c's samples ---------------------------------------------------

uint axis_cells(uint a) {
    const uint3 n = grid_dims();
    return a == 0u ? n.x : (a == 1u ? n.y : n.z);
}

// Samples along axis a of component c: n + 1 on its own axis, n across it.
uint samples(uint c, uint a) { return axis_cells(a) + (a == c ? 1u : 0u); }

uint at(uint c, uint i, uint j, uint k) { return i + samples(c, 0u) * (j + samples(c, 1u) * k); }

// Where component c's faces start in a packed u, v, w array.
uint component_base(uint c) {
    const uint3 n = grid_dims();
    const uint faces_u = (n.x + 1u) * n.y * n.z;
    const uint faces_v = n.x * (n.y + 1u) * n.z;
    return c == 0u ? 0u : (c == 1u ? faces_u : faces_u + faces_v);
}

float load(uint c, uint idx) {
    if (c == 0u) return airflow_u[idx];
    if (c == 1u) return airflow_v[idx];
    return airflow_w[idx];
}

void store(uint c, uint idx, float value) {
    if (c == 0u) {
        airflow_u[idx] = value;
    } else if (c == 1u) {
        airflow_v[idx] = value;
    } else {
        airflow_w[idx] = value;
    }
}

// --- the solids (B5's packing) -----------------------------------------------

// viscous.cpp's fixed(): closed (theta == 0) or inside a solid (phi < 0).
bool fixed_sample(uint c, uint idx) {
    const uint f = component_base(c) + idx;
    return airflow_apertures[f] == 0.0 || airflow_phi[f] < 0.0;
}

uint wall_link(uint c, uint idx) { return airflow_face_tags[component_base(c) + idx]; }
float link_inv_f(uint link, uint side) { return asfloat(airflow_wall_links[12u * link + side]); }
float link_wall(uint link, uint side) { return asfloat(airflow_wall_links[12u * link + 6u + side]); }

// --- the modes ---------------------------------------------------------------

void prepare(uint c, uint idx) {
    float x = load(c, idx);
    if (has_solids() && fixed_sample(c, idx)) {
        x = airflow_wall_velocity[component_base(c) + idx];
        store(c, idx, x);
    }
    airflow_viscous_rhs[component_base(c) + idx] = x;
}

// One sample of the colour: viscous.cpp's general body.
void relax(uint c, uint i, uint j, uint k) {
    const uint n[3] = { grid_dims().x, grid_dims().y, grid_dims().z };
    const uint idx[3] = { i, j, k };
    const uint here = at(c, i, j, k);
    const bool walls = has_solids();
    uint link = kNoWallLinks;
    if (walls) {
        if (fixed_sample(c, here)) return;
        link = wall_link(c, here);
    }
    const uint own = idx[c];
    const bool periodic_c = axis_periodic(c);
    // The periodic alias face, or a boundary face the velocity conditions hold.
    if (periodic_c ? own == n[c] : (own == 0u || own == n[c])) return;
    const float alpha = asfloat(viscous_push.alpha);
    float sum = 0.0;
    float diag = 1.0;
    float extra = 0.0;
    for (uint a = 0u; a < 3u; ++a) {
        for (uint dir = 0u; dir < 2u; ++dir) {
            const uint side = 2u * a + dir;
            if (link != kNoWallLinks && link_inv_f(link, side) > 0.0) {
                const float w = alpha * link_inv_f(link, side);
                diag += w;
                extra += w * link_wall(link, side);
                continue;
            }
            uint nb[3] = { i, j, k };
            if (a == c) {
                if (periodic_c && n[c] == 1u) continue;  // its own neighbour
                if (dir == 0u) {
                    nb[a] = own == 0u ? n[c] - 1u : own - 1u;
                } else {
                    nb[a] = (periodic_c && own + 1u == n[c]) ? 0u : own + 1u;
                }
                sum += alpha * load(c, at(c, nb[0], nb[1], nb[2]));
                diag += alpha;
                continue;
            }
            const bool edge = dir == 0u ? idx[a] == 0u : idx[a] + 1u == n[a];
            if (!edge) {
                nb[a] = dir == 0u ? idx[a] - 1u : idx[a] + 1u;
                sum += alpha * load(c, at(c, nb[0], nb[1], nb[2]));
                diag += alpha;
                continue;
            }
            const uint kind = face_kind(side);
            if (kind == kFacePeriodic) {
                if (n[a] > 1u) {
                    nb[a] = dir == 0u ? n[a] - 1u : 0u;
                    sum += alpha * load(c, at(c, nb[0], nb[1], nb[2]));
                    diag += alpha;
                }
            } else if (kind == kFaceFixedVelocity) {  // a wall's no slip or an inflow: the ghost 2 U - x
                diag += 2.0 * alpha;
                extra += 2.0 * alpha * ghost(side);
            }  // an outlet: the zero-gradient ghost, no term
        }
    }
    store(c, here, div32_ftz((airflow_viscous_rhs[component_base(c) + here] + sum) + extra, diag));
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_viscous(uint3 tid : SV_DispatchThreadID) {
    const uint t = tid.x;
    if (t >= viscous_push.count) { return; }
    const uint c = viscous_push.component;
    const uint sx = samples(c, 0u);
    const uint sy = samples(c, 1u);
    if (viscous_push.mode == 0u) {
        prepare(c, t);
    } else if (viscous_push.mode == 1u) {
        // count = ceil(sx / 2) sy sz: the colour's samples, row by row, as the
        // CPU's i = (j + k + colour) & 1, i += 2.
        const uint half_x = (sx + 1u) / 2u;
        const uint row = t / half_x;
        const uint j = row % sy;
        const uint k = row / sy;
        const uint i = 2u * (t % half_x) + ((j + k + viscous_push.colour) & 1u);
        if (i < sx) relax(c, i, j, k);
    } else {
        // count = the plane across the own axis; face n takes face 0's value.
        const uint n = axis_cells(c);
        const uint s0 = c == 0u ? samples(c, 1u) : sx;
        const uint p = t % s0;
        const uint q = t / s0;
        if (c == 0u) {
            store(c, at(c, n, p, q), load(c, at(c, 0u, p, q)));
        } else if (c == 1u) {
            store(c, at(c, p, n, q), load(c, at(c, p, 0u, q)));
        } else {
            store(c, at(c, p, q, n), load(c, at(c, p, q, 0u)));
        }
    }
}
