// ---------------------------------------------------------------------------
// airflow_mg_restrict_full.slang (plan B, task B5.2; Physics) -- the residual
// of level l onto level l + 1 BY FULL WEIGHTING, the symmetric V(2,2)
// preconditioner's restriction: multigrid.cpp's restrict_full_weighting() and
// reach(), its CPU twin and the authority for every expression, and
// cycle_sym()'s zeroing of the coarse p.
//
// The restriction is the prolongation's transpose (airflow_mg_prolong's
// taps(), below as multigrid.cpp's): along each axis, reach() lists the fine
// indices whose interpolation reads coarse index I and the weight each reads
// it by -- 2I - 1 .. 2I + 2 on a coarsened axis, wrapped on a periodic one
// (the face kind of its low side, as prolong_add() reads it), dropped past an
// edge, a weight 0 dropped too; I alone at 1 on an axis not coarsened. Each
// coarse cell then sums
//     (wzy * w_x) * r,   wzy = w_z * w_y,
// from 0 over the z list (a), the y list (b) and the x list (d), in that
// nesting, and takes b = sum * 4 / (sx sy sz), as B2's restriction scales
// (0.5, 1, 2 or 4, exact). It sets the coarse p to 0, as B2's does.
//
// The weights are sums of 0.75 and 0.25 and their products, all exact; every
// operation is a correctly rounded add or multiply, uncontracted (-fp-mode
// precise), in the CPU's order, so the coarse b is the CPU's bit for bit. It
// reads no face weight: the face weights enter the cycle through the
// smoother and the residual (mg_levels.slang's stencil).
//
// One invocation per coarse cell; the push's count (word 2) is level l + 1's
// cells.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import mg_levels;

struct RestrictFullPush {
    uint level;  // the fine level, l
    uint unused0;
    uint count;  // level l + 1's cells
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<RestrictFullPush> restrict_full_push;

// multigrid.cpp's taps(): the two coarse indices a fine index reads, and
// their weights (airflow_mg_prolong.slang's, the same rule).
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

// multigrid.cpp's Reach: the fine indices along one axis that read coarse
// index I, in ascending d, and their weights.
struct Reach {
    uint n;
    uint idx[4];
    float w[4];
};

Reach reach(uint I, uint nf, uint nc, bool periodic) {
    Reach r;
    r.n = 0u;
    for (uint m = 0u; m < 4u; ++m) {
        r.idx[m] = 0u;
        r.w[m] = 0.0;
    }
    if (nf == nc) {
        r.n = 1u;
        r.idx[0] = I;
        r.w[0] = 1.0;
        return r;
    }
    for (int d = -1; d <= 2; ++d) {
        int i = 2 * int(I) + d;
        if (i < 0 || i >= int(nf)) {
            if (!periodic) continue;
            i = (i + int(nf)) % int(nf);
        }
        const Taps t = taps(uint(i), nf, nc, periodic);
        float w = 0.0;
        if (t.i0 == I) w += t.w0;
        if (t.i1 == I) w += t.w1;
        if (w == 0.0) continue;
        r.idx[r.n] = uint(i);
        r.w[r.n] = w;
        ++r.n;
    }
    return r;
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_mg_restrict_full(uint3 tid : SV_DispatchThreadID) {
    const uint at = tid.x;
    if (at >= restrict_full_push.count) { return; }
    const Level f = level(restrict_full_push.level);
    const Level c = level(restrict_full_push.level + 1u);
    const uint3 s = uint3(f.n.x == c.n.x ? 1u : 2u, f.n.y == c.n.y ? 1u : 2u, f.n.z == c.n.z ? 1u : 2u);
    const uint children = s.x * s.y * s.z;
    // 4 / (sx sy sz), exact for 1, 2, 4 and 8 children.
    const float scale = children == 8u ? 0.5 : (children == 4u ? 1.0 : (children == 2u ? 2.0 : 4.0));
    const uint3 C = cell_of(c, at);
    const Reach rz = reach(C.z, f.n.z, c.n.z, face_kind(4u) == kFacePeriodic);
    const Reach ry = reach(C.y, f.n.y, c.n.y, face_kind(2u) == kFacePeriodic);
    const Reach rx = reach(C.x, f.n.x, c.n.x, face_kind(0u) == kFacePeriodic);
    float sum = 0.0;
    for (uint a = 0u; a < rz.n; ++a) {
        for (uint b = 0u; b < ry.n; ++b) {
            const float wzy = rz.w[a] * ry.w[b];
            for (uint d = 0u; d < rx.n; ++d) {
                sum += (wzy * rx.w[d]) * level_r(f, cell_at(f, uint3(rx.idx[d], ry.idx[b], rz.idx[a])));
            }
        }
    }
    set_level_b(c, at, sum * scale);
    set_level_p(c, at, 0.0);
}
