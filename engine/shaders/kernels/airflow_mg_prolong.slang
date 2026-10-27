// ---------------------------------------------------------------------------
// airflow_mg_prolong.slang (plan B, task B2) -- level l + 1's p added onto
// level l's: multigrid.cpp's prolong_add() and taps(), its CPU twin and the
// authority for every expression. fine.p += the cell-centred trilinear
// interpolation of coarse.p.
//
// TAPS, per axis. On a coarsened axis a fine index reads coarse I = fine / 2
// at 0.75 and its neighbour at 0.25: I - 1 for an even fine index, I + 1 for
// an odd one; past the axis's end the neighbour wraps on a periodic axis and
// is I itself otherwise. On a held axis or one of one cell (nf == nc) it reads
// itself at 1 and 0. The sum is prolong_add()'s, in its order: x inside y
// inside z.
//
// One invocation per fine cell; the push's count (word 2) is level l's cells.
// Every operation is a correctly rounded add or multiply, so the sum is the
// CPU's bit for bit.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import mg_levels;

struct ProlongPush {
    uint level;  // the fine level, l
    uint unused0;
    uint count;  // level l's cells
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<ProlongPush> prolong_push;

struct Taps {
    uint i0;
    uint i1;
    float w0;
    float w1;
};

Taps make_taps(uint i0, uint i1, float w0, float w1) {
    Taps t;
    t.i0 = i0;
    t.i1 = i1;
    t.w0 = w0;
    t.w1 = w1;
    return t;
}

Taps taps(uint fine, uint nf, uint nc, bool periodic) {
    if (nf == nc) return make_taps(fine, fine, 1.0, 0.0);  // an axis not coarsened
    const uint I = fine / 2u;
    if (fine % 2u == 0u) {
        const uint other = I == 0u ? (periodic ? nc - 1u : 0u) : I - 1u;
        return make_taps(I, other, 0.75, 0.25);
    }
    const uint other = I + 1u == nc ? (periodic ? 0u : I) : I + 1u;
    return make_taps(I, other, 0.75, 0.25);
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_mg_prolong(uint3 tid : SV_DispatchThreadID) {
    const uint at = tid.x;
    if (at >= prolong_push.count) { return; }
    const Level f = level(prolong_push.level);
    const Level c = level(prolong_push.level + 1u);
    const uint3 x = cell_of(f, at);
    const Taps tx = taps(x.x, f.n.x, c.n.x, face_kind(0u) == kFacePeriodic);
    const Taps ty = taps(x.y, f.n.y, c.n.y, face_kind(2u) == kFacePeriodic);
    const Taps tz = taps(x.z, f.n.z, c.n.z, face_kind(4u) == kFacePeriodic);
    const float q000 = level_p(c, cell_at(c, uint3(tx.i0, ty.i0, tz.i0)));
    const float q100 = level_p(c, cell_at(c, uint3(tx.i1, ty.i0, tz.i0)));
    const float q010 = level_p(c, cell_at(c, uint3(tx.i0, ty.i1, tz.i0)));
    const float q110 = level_p(c, cell_at(c, uint3(tx.i1, ty.i1, tz.i0)));
    const float q001 = level_p(c, cell_at(c, uint3(tx.i0, ty.i0, tz.i1)));
    const float q101 = level_p(c, cell_at(c, uint3(tx.i1, ty.i0, tz.i1)));
    const float q011 = level_p(c, cell_at(c, uint3(tx.i0, ty.i1, tz.i1)));
    const float q111 = level_p(c, cell_at(c, uint3(tx.i1, ty.i1, tz.i1)));
    const float z0 = ty.w0 * (tx.w0 * q000 + tx.w1 * q100) + ty.w1 * (tx.w0 * q010 + tx.w1 * q110);
    const float z1 = ty.w0 * (tx.w0 * q001 + tx.w1 * q101) + ty.w1 * (tx.w0 * q011 + tx.w1 * q111);
    set_level_p(f, at, level_p(f, at) + (tz.w0 * z0 + tz.w1 * z1));
}
