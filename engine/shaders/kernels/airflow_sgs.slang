// ---------------------------------------------------------------------------
// airflow_sgs.slang (L54) -- the fluid step's Smagorinsky eddy viscosity on the GPU: engine/physics/airflow/sgs.cpp's
// sgs_diffuse(), expression for expression (its CPU twin and the authority for every expression). The CPU's four
// phases are modes 0-2 here, and its report's reduction is modes 3 and 4:
//
//   mode 0, nu_t:    one invocation per cell. The strain from the centre velocities (0.5 (face + face), recomputed
//                    where the CPU stored them: the same bits), nu_t = c2 sqrt(2 ss), 0 in a solid cell. The RAW value
//                    (before the cap) is stored; a reader holds it at the cap (cap_nut()), which is the CPU's stored
//                    value bit for bit, and the count and the peak read the raw one.
//   mode 1, change:  one invocation per face of component `component`. sgs.cpp's face_change() with the link's
//                    nu_t the mean of the fluid cells it touches (the cap applied), k times it; the wall limit's
//                    flag beside it (1.0 when the change was divided by its load).
//   mode 2, add:     every face of the component takes x += change, the boundary faces' +0 included, as the CPU's
//                    phase 4 does.
//   mode 3, reduce:  kReduceThreads invocations, each one a fixed stride over the cells (the capped count, the peak) and
//                    the faces (the limited count), its three words in a partial slot.
//   mode 4, reduce:  one invocation: the partials summed in slot order into airflow_scalars' words 28, 29 and 30
//                    (the capped count and the limited count as uint bits, the peak). Integer sums and a max do not
//                    depend on their order, so this is the CPU's sum over its planes exactly, with no atomics (the
//                    exact profile forbids them).
//
// WHERE THINGS LIVE. No new binding: airflow_advect (6), dead between the add-increments pass and the next step's
// advection, holds, in floats, the change [0, total), the limited flags [total, 2 total), the raw nu_t
// [2 total, 2 total + cells) and the partials after it, 3 kReduceThreads of them. The host sizes it for that when
// the step has Cs on. airflow_params' words 29, 30 and 31 are k = h / dx^2, c2 = cs cs dx dx and the cap
// dx dx / (8 h), the CPU's own expressions computed on the host under its flush, as the file's header says of its
// other floats.
//
// THE STEP IS d_j(nu_t d_j u_i): the Laplacian form. The transpose term d_j(nu_t d_i u_j) of 2 nu_t S_ij is not in it.
//
// PARITY. -fp-mode precise keeps the arithmetic uncontracted; every division is fp32_math's div32_ftz and the one
// square root its sqrt32_ftz, divss and sqrtss under the CPU step's FTZ|DAZ to the bit (TD-14 as amended), so the
// module is scanned under the exact profile.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

// The airflow push convention: 16 bytes, the count in word 2.
struct SgsPush {
    uint component;  // modes 1 and 2: 0 u, 1 v, 2 w
    uint mode;       // 0 nu_t, 1 change, 2 add, 3 reduce, 4 finish
    uint count;      // the invocations the mode needs
    uint unused;
};

[[vk::push_constant]] ConstantBuffer<SgsPush> sgs_push;

static const uint kNoWallLinks = 0xFFFFFFFFu;  // physics/airflow/solids.hpp's
static const uint kNoRegion = 0xFFFFFFFFu;     // regions.hpp's: a solid cell
static const uint kReduceThreads = 1024u;      // mode 3's invocations (the host dispatches that many)

// airflow_scalars' words for the report (solids_chain.hpp's kSlotSgs*).
static const uint kSlotSgsCapped = 28u;
static const uint kSlotSgsLimited = 29u;
static const uint kSlotSgsPeak = 30u;

// airflow_params' words 29-31 (airflow_params_layout.slang says so).
float sgs_k() { return asfloat(airflow_params[29]); }
float sgs_c2() { return asfloat(airflow_params[30]); }
float sgs_cap() { return asfloat(airflow_params[31]); }

// --- the grid -----------------------------------------------------------------

uint axis_cells(uint a) {
    const uint3 n = grid_dims();
    return a == 0u ? n.x : (a == 1u ? n.y : n.z);
}

uint cell_count() {
    const uint3 n = grid_dims();
    return n.x * n.y * n.z;
}

uint cell_at(uint i, uint j, uint k) {
    const uint3 n = grid_dims();
    return i + n.x * (j + n.y * k);
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

uint total_faces() {
    const uint3 n = grid_dims();
    return (n.x + 1u) * n.y * n.z + n.x * (n.y + 1u) * n.z + n.x * n.y * (n.z + 1u);
}

// --- airflow_advect's scratch ---------------------------------------------------

uint flags_at() { return total_faces(); }
uint nut_at() { return 2u * total_faces(); }
uint partials_at() { return 2u * total_faces() + cell_count(); }

float raw_nut(uint cell) { return airflow_advect[nut_at() + cell]; }

// The cap, as sgs.cpp's `if (nut > cap) nut = cap`.
float cap_nut(float raw) { return raw > sgs_cap() ? sgs_cap() : raw; }

// --- the solids (B5's packing) ----------------------------------------------------

bool cell_solid(uint cell) { return has_solids() && airflow_region_labels[cell] == kNoRegion; }

// sgs.cpp's `closed(theta) || phi < 0`.
bool fixed_sample(uint c, uint idx) {
    const uint f = component_base(c) + idx;
    return airflow_apertures[f] == 0.0 || airflow_phi[f] < 0.0;
}

uint wall_link(uint c, uint idx) { return airflow_face_tags[component_base(c) + idx]; }
float link_inv_f(uint link, uint side) { return asfloat(airflow_wall_links[12u * link + side]); }
float link_wall(uint link, uint side) { return asfloat(airflow_wall_links[12u * link + 6u + side]); }

float load(uint c, uint idx) {
    if (c == 0u) return airflow_u[idx];
    if (c == 1u) return airflow_v[idx];
    return airflow_w[idx];
}

// --- mode 0 ---------------------------------------------------------------------

// The centre velocity of component c in cell (i, j, k): 0.5 (the face below + the face above).
float centre(uint c, uint i, uint j, uint k) {
    if (c == 0u) return 0.5 * (airflow_u[at(0u, i, j, k)] + airflow_u[at(0u, i + 1u, j, k)]);
    if (c == 1u) return 0.5 * (airflow_v[at(1u, i, j, k)] + airflow_v[at(1u, i, j + 1u, k)]);
    return 0.5 * (airflow_w[at(2u, i, j, k)] + airflow_w[at(2u, i, j, k + 1u)]);
}

// sgs.cpp's grad(): d(centre of component c)/d(axis) at the cell, one-sided at the box's faces.
float grad(uint c, uint i, uint j, uint k, uint axis) {
    const uint n3[3] = { axis_cells(0u), axis_cells(1u), axis_cells(2u) };
    const uint a[3] = { i, j, k };
    const uint lo = a[axis] > 0u ? a[axis] - 1u : 0u;
    const uint hi = a[axis] + 1u < n3[axis] ? a[axis] + 1u : n3[axis] - 1u;
    uint pl[3] = { i, j, k };
    uint ph[3] = { i, j, k };
    pl[axis] = lo;
    ph[axis] = hi;
    const float span = float(hi - lo) * grid_dx();
    return div32_ftz(centre(c, ph[0], ph[1], ph[2]) - centre(c, pl[0], pl[1], pl[2]), span);
}

void make_nut(uint t) {
    const uint3 n = grid_dims();
    const uint i = t % n.x;
    const uint j = (t / n.x) % n.y;
    const uint k = t / (n.x * n.y);
    const float dx = grid_dx();
    const float sxx = div32_ftz(airflow_u[at(0u, i + 1u, j, k)] - airflow_u[at(0u, i, j, k)], dx);
    const float syy = div32_ftz(airflow_v[at(1u, i, j + 1u, k)] - airflow_v[at(1u, i, j, k)], dx);
    const float szz = div32_ftz(airflow_w[at(2u, i, j, k + 1u)] - airflow_w[at(2u, i, j, k)], dx);
    const float sxy = 0.5 * (grad(0u, i, j, k, 1u) + grad(1u, i, j, k, 0u));
    const float sxz = 0.5 * (grad(0u, i, j, k, 2u) + grad(2u, i, j, k, 0u));
    const float syz = 0.5 * (grad(1u, i, j, k, 2u) + grad(2u, i, j, k, 1u));
    const float ss = sxx * sxx + syy * syy + szz * szz + 2.0 * (sxy * sxy + sxz * sxz + syz * syz);
    float nut = sgs_c2() * sqrt32_ftz(2.0 * ss);
    if (cell_solid(t)) nut = 0.0;
    airflow_advect[nut_at() + t] = nut;
}

// --- mode 1 ---------------------------------------------------------------------

// sgs.cpp's Mean::add: a fluid cell's capped nu_t joins the sum.
void mean_add(inout float sum, inout uint count, uint i, uint j, uint k) {
    const uint cell = cell_at(i, j, k);
    if (cell_solid(cell)) return;
    sum += cap_nut(raw_nut(cell));
    count += 1u;
}

// sgs.cpp's face_change(): the change d (before the factor k) of component c's sample (i, j, kz), `limited` set when
// the wall limit divided it. x is component c's own array, picked by the caller.
float face_change(RWStructuredBuffer<float> x, uint c, uint i, uint j, uint kz, out bool limited) {
    limited = false;
    const uint n[3] = { axis_cells(0u), axis_cells(1u), axis_cells(2u) };
    const uint idx[3] = { i, j, kz };
    const uint here = at(c, i, j, kz);
    uint link = kNoWallLinks;
    if (has_solids()) {
        if (fixed_sample(c, here)) return 0.0;
        link = wall_link(c, here);
    }
    const uint own = idx[c];
    const float x0 = x[here];
    float d = 0.0;
    float weight = 0.0;  // the sum of the links' nu_t (1 / f) weights, for the bound
    for (uint a = 0u; a < 3u; ++a) {
        for (uint dir = 0u; dir < 2u; ++dir) {
            const bool up = dir == 1u;
            const uint side = 2u * a + dir;
            uint nb[3] = { i, j, kz };
            bool exists = true;
            float msum = 0.0;
            uint mcount = 0u;
            if (a == c) {
                if (axis_periodic(c) && n[c] == 1u) continue;  // its own neighbour
                // Samples own = 0 and n are boundaries; a periodic axis wraps 0 and n - 1.
                nb[a] = up ? ((axis_periodic(c) && own + 1u == n[c]) ? 0u : own + 1u)
                           : (own == 0u ? n[c] - 1u : own - 1u);
                // The cell between the two samples: own when up, own - 1 below.
                uint cc[3] = { i, j, kz };
                cc[c] = up ? own : (own == 0u ? n[c] - 1u : own - 1u);
                mean_add(msum, mcount, cc[0], cc[1], cc[2]);
            } else {
                // step_cell(): one along a, wrapped on a periodic axis, none past a non-periodic edge.
                uint q = 0u;
                if (up) {
                    if (idx[a] + 1u < n[a]) {
                        q = idx[a] + 1u;
                    } else if (axis_periodic(a)) {
                        q = 0u;
                    } else {
                        exists = false;
                    }
                } else {
                    if (idx[a] > 0u) {
                        q = idx[a] - 1u;
                    } else if (axis_periodic(a)) {
                        q = n[a] - 1u;
                    } else {
                        exists = false;
                    }
                }
                // The edge's cells: along c the cells own - 1 and own, along a the cell idx[a] and its neighbour's.
                const uint c0 = own == 0u ? n[c] - 1u : own - 1u;
                const uint c1 = own < n[c] ? own : n[c] - 1u;
                for (uint m = 0u; m < 2u; ++m) {
                    const uint cc_c = m == 0u ? c0 : c1;
                    for (uint s = 0u; s < 2u; ++s) {
                        uint cc[3] = { i, j, kz };
                        cc[c] = cc_c;
                        if (s == 1u) {
                            if (!exists) continue;
                            cc[a] = q;
                        }
                        mean_add(msum, mcount, cc[0], cc[1], cc[2]);
                    }
                }
                if (exists) nb[a] = q;
            }
            const float nu = mcount == 0u ? 0.0 : div32_ftz(msum, float(mcount));
            if (link != kNoWallLinks && link_inv_f(link, side) > 0.0) {
                const float w = nu * link_inv_f(link, side);
                weight += w;
                d += w * (link_wall(link, side) - x0);
                continue;
            }
            if (exists) {
                weight += nu;
                d += nu * (x[at(c, nb[0], nb[1], nb[2])] - x0);
                continue;
            }
            // A domain edge across the axis: a wall's no slip or an inflow takes the ghost 2 U - x; an outlet's
            // zero-gradient ghost and a periodic axis (handled by `exists`) add nothing.
            if (face_kind(side) == kFaceFixedVelocity) {
                weight += 2.0 * nu;
                d += 2.0 * nu * (face_velocity(side, c) - x0);
            }
        }
    }
    const float ld = sgs_k() * weight;
    if (ld > 1.0) {
        limited = true;
        return div32_ftz(d, ld);
    }
    return d;
}

void make_change(RWStructuredBuffer<float> x, uint c, uint t) {
    const uint sx = samples(c, 0u);
    const uint sy = samples(c, 1u);
    const uint i = t % sx;
    const uint j = (t / sx) % sy;
    const uint kz = t / (sx * sy);
    const uint idx[3] = { i, j, kz };
    const uint own = idx[c];
    const uint nc = axis_cells(c);
    // Samples own = 0 and n are boundaries; a periodic axis solves 0 and leaves n to its alias.
    const bool skip = axis_periodic(c) ? own == nc : (own == 0u || own == nc);
    float change = 0.0;
    bool limited = false;
    if (!skip) change = sgs_k() * face_change(x, c, i, j, kz, limited);
    const uint f = component_base(c) + t;
    airflow_advect[f] = change;
    airflow_advect[flags_at() + f] = limited ? 1.0 : 0.0;
}

// --- mode 2 ---------------------------------------------------------------------

void add_change(RWStructuredBuffer<float> x, uint c, uint t) {
    x[t] = x[t] + airflow_advect[component_base(c) + t];
}

// --- modes 3 and 4 ----------------------------------------------------------------

void reduce_stride(uint t) {
    uint capped = 0u;
    float peak = 0.0;
    for (uint cell = t; cell < cell_count(); cell += kReduceThreads) {
        const float v = raw_nut(cell);
        peak = peak < v ? v : peak;  // std::max(peak, nut)
        if (v > sgs_cap()) capped += 1u;
    }
    uint limited = 0u;
    for (uint f = t; f < total_faces(); f += kReduceThreads) {
        if (airflow_advect[flags_at() + f] != 0.0) limited += 1u;
    }
    const uint at_slot = partials_at() + 3u * t;
    airflow_advect[at_slot] = asfloat(capped);
    airflow_advect[at_slot + 1u] = peak;
    airflow_advect[at_slot + 2u] = asfloat(limited);
}

void finish() {
    uint capped = 0u;
    float peak = 0.0;
    uint limited = 0u;
    for (uint t = 0u; t < kReduceThreads; ++t) {
        const uint at_slot = partials_at() + 3u * t;
        capped += asuint(airflow_advect[at_slot]);
        const float v = airflow_advect[at_slot + 1u];
        peak = peak < v ? v : peak;
        limited += asuint(airflow_advect[at_slot + 2u]);
    }
    airflow_scalars[kSlotSgsCapped] = asfloat(capped);
    airflow_scalars[kSlotSgsLimited] = asfloat(limited);
    airflow_scalars[kSlotSgsPeak] = peak;
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_sgs(uint3 tid : SV_DispatchThreadID) {
    const uint t = tid.x;
    if (t >= sgs_push.count) { return; }
    const uint c = sgs_push.component;
    if (sgs_push.mode == 0u) {
        make_nut(t);
    } else if (sgs_push.mode == 1u) {
        // The component's buffer, picked once: a pick per load can compile to a load from all three.
        if (c == 0u) {
            make_change(airflow_u, c, t);
        } else if (c == 1u) {
            make_change(airflow_v, c, t);
        } else {
            make_change(airflow_w, c, t);
        }
    } else if (sgs_push.mode == 2u) {
        if (c == 0u) {
            add_change(airflow_u, c, t);
        } else if (c == 1u) {
            add_change(airflow_v, c, t);
        } else {
            add_change(airflow_w, c, t);
        }
    } else if (sgs_push.mode == 3u) {
        reduce_stride(t);
    } else {
        finish();
    }
}
