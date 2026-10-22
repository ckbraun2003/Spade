// ---------------------------------------------------------------------------
// mg_levels.slang (plan B, task B2) -- one multigrid level as the GPU
// projection's kernels see it: where its p, b and r live, its face weights,
// and plan A's pressure stencil (engine/physics/airflow/pressure.cpp's
// stencil() and weighted_stencil(), its CPU twin and the authority for every
// expression). Imported by airflow_mg_smooth, airflow_mg_residual,
// airflow_mg_restrict and airflow_mg_prolong, so the four read a level by one
// rule (TD-9). The level table, the face kinds and every other params word
// are airflow_params_layout's.
//
// WHERE A LEVEL LIVES (Core's B0 table). Level 0 is the grid's own arrays: p
// in airflow_pressure, b in airflow_rhs, r in airflow_residual. Levels 1..L
// are packed in airflow_multigrid at the table's offsets. A weighted level
// (the levels past the halving rule, multigrid.hpp's clear_weights()) reads
// its face weights from airflow_mg_weights at off_w[a], each axis's faces
// indexed as the MAC grid of the level's shape indexes u, v and w.
//
// ONE STENCIL FOR BOTH. The CPU runs stencil() on a level without weights and
// weighted_stencil() on one with them. Here an unweighted level's weight is
// 1.0: 1.0 * p is p, and the diagonal sums the same 1.0s and 2.0s in the same
// order, so the one path gives stencil()'s bits.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;

// A level: its index (level 0's arrays are the grid's own), its cells per
// axis, and its table entry.
struct Level {
    uint index;
    uint3 n;
    MgLevel table;
};

Level level(uint l) {
    Level L;
    L.index = l;
    L.table = mg_level(l);
    L.n = L.table.dims;
    return L;
}

uint cell_at(Level L, uint3 c) { return c.x + L.n.x * (c.y + L.n.y * c.z); }

uint3 cell_of(Level L, uint index) {
    return uint3(index % L.n.x, (index / L.n.x) % L.n.y, index / (L.n.x * L.n.y));
}

float level_p(Level L, uint c) { return L.index == 0u ? airflow_pressure[c] : airflow_multigrid[L.table.off_p + c]; }
float level_b(Level L, uint c) { return L.index == 0u ? airflow_rhs[c] : airflow_multigrid[L.table.off_b + c]; }
float level_r(Level L, uint c) { return L.index == 0u ? airflow_residual[c] : airflow_multigrid[L.table.off_r + c]; }

void set_level_p(Level L, uint c, float v) {
    if (L.index == 0u) {
        airflow_pressure[c] = v;
    } else {
        airflow_multigrid[L.table.off_p + c] = v;
    }
}

void set_level_b(Level L, uint c, float v) {
    if (L.index == 0u) {
        airflow_rhs[c] = v;
    } else {
        airflow_multigrid[L.table.off_b + c] = v;
    }
}

void set_level_r(Level L, uint c, float v) {
    if (L.index == 0u) {
        airflow_residual[c] = v;
    } else {
        airflow_multigrid[L.table.off_r + c] = v;
    }
}

// The weight of axis a's face f (pressure.cpp's face_at()); 1.0 on a level
// without weights.
float face_weight(Level L, uint a, uint3 f) {
    if (!L.table.weighted) return 1.0;
    const uint cx = a == 0u ? L.n.x + 1u : L.n.x;
    const uint cy = a == 1u ? L.n.y + 1u : L.n.y;
    return airflow_mg_weights[L.table.off_w[a] + f.x + cx * (f.y + cy * f.z)];
}

struct Stencil {
    float sum;
    float diag;
};

// The neighbour sum and the diagonal of cell c, faces -x, +x, -y, +y, -z, +z
// (pressure.cpp's stencil() and weighted_stencil()). A face to another cell,
// or across a periodic axis of more than one cell, adds w p_nb and w; a
// pressure outlet adds 2 w to the diagonal; a fixed-velocity face nothing.
Stencil stencil(Level L, uint3 c) {
    Stencil s;
    s.sum = 0.0;
    s.diag = 0.0;
    for (uint axis = 0u; axis < 3u; ++axis) {
        for (uint dir = 0u; dir < 2u; ++dir) {
            uint3 f = c;
            f[axis] = c[axis] + dir;
            const float w = face_weight(L, axis, f);
            const bool edge = dir == 0u ? c[axis] == 0u : c[axis] + 1u == L.n[axis];
            uint3 nb = c;
            if (!edge) {
                nb[axis] = dir == 0u ? c[axis] - 1u : c[axis] + 1u;
                s.sum += w * level_p(L, cell_at(L, nb));
                s.diag += w;
                continue;
            }
            const uint kind = face_kind(2u * axis + dir);
            if (kind == kFacePeriodic) {
                if (L.n[axis] > 1u) {
                    nb[axis] = dir == 0u ? L.n[axis] - 1u : 0u;
                    s.sum += w * level_p(L, cell_at(L, nb));
                    s.diag += w;
                }
            } else if (kind == kFacePressureOutlet) {
                s.diag += 2.0 * w;
            }
        }
    }
    return s;
}
