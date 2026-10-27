// ---------------------------------------------------------------------------
// airflow_publish_fields.slang (plan B, task B6) -- publish's channels on the
// GPU: engine/physics/airflow/publish.cpp's per-cell work and core/lambda2's
// Jacobi solve, term for term (their CPU twins, and the authority for every
// expression). airflow_pack_f16 then packs the channels to halves.
//
// One invocation per cell (1D, guarded by the push's cell_count). A fluid
// cell's eight channels: the face velocities averaged to the centre, dye 0,
// the pressure, |omega|, q-hat and lambda2-hat, from the velocity gradient
// G[c][a] = d u_c / d x_a (its diagonal from the cell's own faces, its off-
// diagonal from central differences of the centred velocities: wrapped on a
// periodic axis, one-sided at another axis' edge or across a closed face,
// 0 with both sides closed). A solid cell with fluid face-neighbours takes
// their channels' mean (-x, +x, -y, +y, -z, +z; across the wrap on a
// periodic axis of more than one cell); a deeper one is no data (NaN, which
// the pack writes as fp16::kNoDataBits).
//
// Reads airflow_u/v/w, airflow_pressure, airflow_apertures (theta, packed
// like the velocity, when the params say so) and airflow_region_labels (the
// solid mask); writes the eight fp32 channels per cell into airflow_advect,
// the advection's scratch, cell-major (channel c of cell n at 8 n + c).
// The header (channel ranges, no-data and extrapolated counts) is the host's,
// from the packed halves.
//
// PARITY. -fp-mode precise keeps the arithmetic uncontracted; 1/dx comes from
// the host, and 1/n for the solid mean from a table of correctly rounded
// constants, so u, v, w, dye and p match publish.cpp to the bit. |omega|,
// q-hat and lambda2-hat take fp32_math's sqrt32_ftz and div32_ftz, sqrtss and
// divss under the CPU's FTZ|DAZ to the bit, each spelled as publish.cpp and
// core/lambda2.cpp spell theirs, so they match too (TD-14 as amended
// 2026-10-22), and the module is scanned under the exact profile.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;
import fp32_math;

// The airflow push convention (airflow_params_layout.slang): the count in
// word 2.
struct PublishPush {
    uint unused0;
    uint unused1;
    uint cell_count;
    uint unused2;
};

[[vk::push_constant]] ConstantBuffer<PublishPush> publish_push;

static const uint kChannels = 8u;

// A cell is solid where its region label is kNoRegion (regions.hpp), as
// FluidRegions::label marks it: the solid mask with no buffer of its own.
static const uint kNoRegion = 0xFFFFFFFFu;
bool cell_solid(uint cell) { return has_solids() && airflow_region_labels[cell] == kNoRegion; }

// The fp32 scratch is airflow_advect: 3 x faces_total floats, at least 9 per
// cell, and dead between the step's chain and this one (Physics).
void store_channel(uint cell, uint c, float value) { airflow_advect[kChannels * cell + c] = value; }

uint axis_cells(uint a) {
    const uint3 n = grid_dims();
    return a == 0u ? n.x : (a == 1u ? n.y : n.z);
}

uint cell_index(uint i, uint j, uint k) {
    const uint3 n = grid_dims();
    return i + n.x * (j + n.y * k);
}

// mac_grid.hpp's face indices.
uint u_index(uint i, uint j, uint k) {
    const uint3 n = grid_dims();
    return i + (n.x + 1u) * (j + n.y * k);
}
uint v_index(uint i, uint j, uint k) {
    const uint3 n = grid_dims();
    return i + n.x * (j + (n.y + 1u) * k);
}
uint w_index(uint i, uint j, uint k) {
    const uint3 n = grid_dims();
    return i + n.x * (j + n.y * k);
}

// Where component c's faces start in airflow_apertures (packed u, v, w).
uint aperture_base(uint c) {
    const uint3 n = grid_dims();
    const uint faces_u = (n.x + 1u) * n.y * n.z;
    const uint faces_v = n.x * (n.y + 1u) * n.z;
    return c == 0u ? 0u : (c == 1u ? faces_u : faces_u + faces_v);
}

// Whether the face of component a at coordinate f along a, in the line
// through cell (i, j, k), is closed.
bool closed_face(uint a, uint i, uint j, uint k, uint f) {
    if (!has_apertures()) return false;
    const uint idx = a == 0u ? u_index(f, j, k) : (a == 1u ? v_index(i, f, k) : w_index(i, j, f));
    return airflow_apertures[aperture_base(a) + idx] == 0.0;
}

// The velocity component c averaged to the centre of cell (i, j, k).
float centred(uint c, uint i, uint j, uint k) {
    if (c == 0u) return 0.5 * (airflow_u[u_index(i, j, k)] + airflow_u[u_index(i + 1u, j, k)]);
    if (c == 1u) return 0.5 * (airflow_v[v_index(i, j, k)] + airflow_v[v_index(i, j + 1u, k)]);
    return 0.5 * (airflow_w[w_index(i, j, k)] + airflow_w[w_index(i, j, k + 1u)]);
}

typealias Gradient = float[3][3];

// G[c][a] = d u_c / d x_a at the centre of cell (i, j, k).
Gradient gradient(uint i, uint j, uint k) {
    const float inv_dx = grid_inv_dx();
    Gradient G = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
    G[0][0] = (airflow_u[u_index(i + 1u, j, k)] - airflow_u[u_index(i, j, k)]) * inv_dx;
    G[1][1] = (airflow_v[v_index(i, j + 1u, k)] - airflow_v[v_index(i, j, k)]) * inv_dx;
    G[2][2] = (airflow_w[w_index(i, j, k + 1u)] - airflow_w[w_index(i, j, k)]) * inv_dx;
    const uint at[3] = { i, j, k };
    for (uint a = 0u; a < 3u; ++a) {
        const uint n = axis_cells(a);
        if (n == 1u) continue;  // an axis of one cell: no difference along it
        // A neighbour is used when it exists (a periodic axis wraps at its
        // edges) and the face between is open: both, a central difference;
        // one, one-sided from the cell; none, 0.
        const bool wrap = axis_periodic(a);
        const bool lo_ok = (at[a] != 0u || wrap) && !closed_face(a, i, j, k, at[a]);
        const bool hi_ok = (at[a] + 1u != n || wrap) && !closed_face(a, i, j, k, at[a] + 1u);
        const uint lo_q = !lo_ok ? at[a] : (at[a] == 0u ? n - 1u : at[a] - 1u);
        const uint hi_q = !hi_ok ? at[a] : (at[a] + 1u == n ? 0u : at[a] + 1u);
        const float scale = lo_ok && hi_ok ? 0.5 * inv_dx : (lo_ok || hi_ok ? inv_dx : 0.0);
        uint lo[3] = { i, j, k };
        uint hi[3] = { i, j, k };
        lo[a] = lo_q;
        hi[a] = hi_q;
        for (uint c = 0u; c < 3u; ++c) {
            if (c == a) continue;
            G[c][a] = (centred(c, hi[0], hi[1], hi[2]) - centred(c, lo[0], lo[1], lo[2])) * scale;
        }
    }
    return G;
}

// --- core/lambda2.cpp ------------------------------------------------------

// std::min and std::max as they compare (not min() and max(), which differ
// on NaN and signed zero).
float std_min(float a, float b) { return b < a ? b : a; }
float std_max(float a, float b) { return a < b ? b : a; }

// One Jacobi rotation in the plane (p, q), r the third index.
void rotate(inout float a[3][3], uint p, uint q, uint r) {
    const float apq = a[p][q];
    if (apq == 0.0) return;  // already diagonal in this plane
    const float theta = div32_ftz(a[q][q] - a[p][p], 2.0 * apq);
    float t = 0.0;
    if (abs(theta) > 1.0e18) {
        t = div32_ftz(0.5, theta);
    } else {
        t = div32_ftz(1.0, abs(theta) + sqrt32_ftz(theta * theta + 1.0));
        if (theta < 0.0) t = -t;
    }
    const float c = div32_ftz(1.0, sqrt32_ftz(t * t + 1.0));
    const float s = t * c;
    const float tau = div32_ftz(s, 1.0 + c);
    a[p][p] -= t * apq;
    a[q][q] += t * apq;
    a[p][q] = 0.0;
    a[q][p] = 0.0;
    const float arp = a[r][p];
    const float arq = a[r][q];
    a[r][p] = arp - s * (arq + tau * arp);
    a[p][r] = a[r][p];
    a[r][q] = arq + s * (arp - tau * arq);
    a[q][r] = a[r][q];
}

static const int kLambda2Sweeps = 6;  // core/lambda2.hpp's

// lambda2-hat of G: sign(lambda2) sqrt|lambda2|, lambda2 the middle
// eigenvalue of S^2 + Omega^2, computed as (G^2 + (G^2)^T) / 2.
float lambda2_hat(Gradient g) {
    float p[3][3];
    for (uint i = 0u; i < 3u; ++i) {
        for (uint j = 0u; j < 3u; ++j) p[i][j] = g[i][0] * g[0][j] + g[i][1] * g[1][j] + g[i][2] * g[2][j];
    }
    const float xy = 0.5 * (p[0][1] + p[1][0]);
    const float xz = 0.5 * (p[0][2] + p[2][0]);
    const float yz = 0.5 * (p[1][2] + p[2][1]);
    float a[3][3] = { { p[0][0], xy, xz }, { xy, p[1][1], yz }, { xz, yz, p[2][2] } };
    for (int sweep = 0; sweep < kLambda2Sweeps; ++sweep) {
        rotate(a, 0u, 1u, 2u);
        rotate(a, 0u, 2u, 1u);
        rotate(a, 1u, 2u, 0u);
    }
    const float d0 = a[0][0];
    const float d1 = a[1][1];
    const float d2 = a[2][2];
    const float l = std_max(std_min(d0, d1), std_min(std_max(d0, d1), d2));
    return l < 0.0 ? -sqrt32_ftz(-l) : sqrt32_ftz(l);
}

// --- publish.cpp's fluid_channels ------------------------------------------

typealias Channels = float[8];

Channels fluid_channels(uint i, uint j, uint k) {
    const Gradient G = gradient(i, j, k);
    const float wx = G[2][1] - G[1][2];
    const float wy = G[0][2] - G[2][0];
    const float wz = G[1][0] - G[0][1];
    const float w2 = (wx * wx + wy * wy) + wz * wz;
    float s2 = 0.0;
    for (uint r = 0u; r < 3u; ++r) {
        for (uint c = 0u; c < 3u; ++c) {
            const float sym = 0.5 * (G[r][c] + G[c][r]);
            s2 += sym * sym;
        }
    }
    const float Q = 0.25 * w2 - 0.5 * s2;
    const float q_hat = Q >= 0.0 ? sqrt32_ftz(Q) : -sqrt32_ftz(-Q);
    const Channels out = { centred(0u, i, j, k), centred(1u, i, j, k), centred(2u, i, j, k), 0.0,
                           airflow_pressure[cell_index(i, j, k)], sqrt32_ftz(w2), q_hat, lambda2_hat(G) };
    return out;
}

// 1/n for the solid mean's n fluid neighbours, as the CPU's 1.0f / n rounds:
// each the exact decimal value of that float (0x3EAAAAAB, 0x3E4CCCCD,
// 0x3E2AAAAB), so no parse or fold can round it elsewhere, where a division
// on the device need not round correctly.
static const float kInverse[7] = { 0.0, 1.0, 0.5, 0.3333333432674407958984375, 0.25,
                                   0.20000000298023223876953125, 0.16666667163372039794921875 };

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_publish_fields(uint3 tid : SV_DispatchThreadID) {
    const uint cell = tid.x;
    if (cell >= publish_push.cell_count) { return; }
    const uint3 n = grid_dims();
    const uint i = cell % n.x;
    const uint j = (cell / n.x) % n.y;
    const uint k = cell / (n.x * n.y);
    if (!cell_solid(cell)) {
        const Channels ch = fluid_channels(i, j, k);
        for (uint c = 0u; c < kChannels; ++c) store_channel(cell, c, ch[c]);
        return;
    }
    // The fluid face-neighbours, -x, +x, -y, +y, -z, +z; across the wrap on
    // a periodic axis of more than one cell.
    const uint at[3] = { i, j, k };
    Channels sum = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    uint fluid = 0u;
    for (uint a = 0u; a < 3u; ++a) {
        const uint na = axis_cells(a);
        for (uint dir = 0u; dir < 2u; ++dir) {
            const bool edge = dir == 0u ? at[a] == 0u : at[a] + 1u == na;
            if (edge && !(axis_periodic(a) && na > 1u)) continue;
            uint nb[3] = { i, j, k };
            if (edge) {
                nb[a] = dir == 0u ? na - 1u : 0u;
            } else {
                nb[a] = dir == 0u ? at[a] - 1u : at[a] + 1u;
            }
            if (cell_solid(cell_index(nb[0], nb[1], nb[2]))) continue;
            const Channels f = fluid_channels(nb[0], nb[1], nb[2]);
            for (uint c = 0u; c < kChannels; ++c) sum[c] += f[c];
            ++fluid;
        }
    }
    if (fluid == 0u) {  // deeper than the first layer: no data
        for (uint c = 0u; c < kChannels; ++c) store_channel(cell, c, asfloat(0x7FC00000u));
        return;
    }
    const float inv = kInverse[fluid];
    for (uint c = 0u; c < kChannels; ++c) store_channel(cell, c, sum[c] * inv);
}
