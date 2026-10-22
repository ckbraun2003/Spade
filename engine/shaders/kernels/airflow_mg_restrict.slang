// ---------------------------------------------------------------------------
// airflow_mg_restrict.slang (plan B, task B2) -- the residual of level l onto
// level l + 1: multigrid.cpp's restrict_residual(), its CPU twin and the
// authority for every expression, and cycle()'s zeroing of the coarse p.
//
// Each coarse cell sums its children's r from 0 in (dk, dj, di) order, a child
// past an odd axis's end left out (its partner is closed), and takes
// b = sum * 4 / (sx sy sz): s is 2 on a coarsened axis and 1 on a held one
// (multigrid.hpp), so the scale is 0.5, 1, 2 or 4, exact. It also sets the
// coarse p to 0 (cycle()'s std::fill before the recursion), which saves a
// dispatch.
//
// One invocation per coarse cell; the push's count (word 2) is level l + 1's
// cells. Every operation is a correctly rounded add or multiply, so the coarse
// b is the CPU's bit for bit.
// ---------------------------------------------------------------------------

import bindings;
import mg_levels;

struct RestrictPush {
    uint level;  // the fine level, l
    uint unused0;
    uint count;  // level l + 1's cells
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<RestrictPush> restrict_push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_mg_restrict(uint3 tid : SV_DispatchThreadID) {
    const uint at = tid.x;
    if (at >= restrict_push.count) { return; }
    const Level f = level(restrict_push.level);
    const Level c = level(restrict_push.level + 1u);
    const uint3 s = uint3(f.n.x == c.n.x ? 1u : 2u, f.n.y == c.n.y ? 1u : 2u, f.n.z == c.n.z ? 1u : 2u);
    const uint children = s.x * s.y * s.z;
    // 4 / (sx sy sz), exact for 1, 2, 4 and 8 children.
    const float scale = children == 8u ? 0.5 : (children == 4u ? 1.0 : (children == 2u ? 2.0 : 4.0));
    const uint3 C = cell_of(c, at);
    float sum = 0.0;
    for (uint dk = 0u; dk < s.z; ++dk) {
        const uint k = C.z * s.z + dk;
        if (k >= f.n.z) continue;  // an odd axis's last cell: its partner is closed
        for (uint dj = 0u; dj < s.y; ++dj) {
            const uint j = C.y * s.y + dj;
            if (j >= f.n.y) continue;
            for (uint di = 0u; di < s.x; ++di) {
                const uint i = C.x * s.x + di;
                if (i >= f.n.x) continue;
                sum += level_r(f, cell_at(f, uint3(i, j, k)));
            }
        }
    }
    set_level_b(c, at, sum * scale);
    set_level_p(c, at, 0.0);
}
