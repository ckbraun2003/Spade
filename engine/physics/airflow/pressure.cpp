#include "physics/airflow/pressure.hpp"

namespace spade::physics::airflow {

namespace {

struct Stencil {
    float sum = 0.0f;
    float diag = 0.0f;
};

[[nodiscard]] inline std::size_t at(const PoissonLevel& L, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{L.nx} * (j + std::size_t{L.ny} * k);
}

// The neighbour sum and the diagonal of cell (i, j, k), faces -x, +x, -y, +y, -z, +z.
[[nodiscard]] Stencil stencil(const PoissonLevel& L, uint32_t i, uint32_t j, uint32_t k) noexcept {
    Stencil s;
    const uint32_t n[3] = {L.nx, L.ny, L.nz};
    const uint32_t c[3] = {i, j, k};
    for (uint32_t axis = 0; axis < 3; ++axis) {
        for (uint32_t dir = 0; dir < 2; ++dir) {
            const bool edge = dir == 0 ? c[axis] == 0 : c[axis] + 1u == n[axis];
            uint32_t nb[3] = {i, j, k};
            if (!edge) {
                nb[axis] = dir == 0 ? c[axis] - 1u : c[axis] + 1u;
                s.sum += L.p[at(L, nb[0], nb[1], nb[2])];
                s.diag += 1.0f;
                continue;
            }
            switch (L.kind[2u * axis + dir]) {
                case FaceKind::periodic:
                    if (n[axis] > 1) {
                        nb[axis] = dir == 0 ? n[axis] - 1u : 0u;
                        s.sum += L.p[at(L, nb[0], nb[1], nb[2])];
                        s.diag += 1.0f;
                    }
                    break;
                case FaceKind::pressure_outlet:
                    s.diag += 2.0f;
                    break;
                case FaceKind::fixed_velocity:
                    break;
            }
        }
    }
    return s;
}

// THE INTERIOR FAST PATH (the airflow CPU-performance task, 2026-10-12): a
// cell with all six neighbours inside the level takes the same six additions
// in the same order (-x, +x, -y, +y, -z, +z, from 0.0f) and the diagonal 6
// that stencil() would sum, so its bits are stencil()'s; only cells on the
// level's edges pay for the face kinds. `c` is the cell's index.
[[nodiscard]] inline float interior_sum(const float* p, std::size_t c, std::size_t sy, std::size_t sz) noexcept {
    float sum = 0.0f;
    sum += p[c - 1u];
    sum += p[c + 1u];
    sum += p[c - sy];
    sum += p[c + sy];
    sum += p[c - sz];
    sum += p[c + sz];
    return sum;
}

constexpr float kInteriorDiag = 6.0f;

// Whether index q along an axis of n cells has both neighbours inside.
[[nodiscard]] inline bool inside(uint32_t q, uint32_t n) noexcept { return q >= 1u && q + 1u < n; }

}  // namespace

PoissonLevel make_level(uint32_t nx, uint32_t ny, uint32_t nz, const std::array<FaceKind, 6>& kind) {
    PoissonLevel L;
    L.nx = nx;
    L.ny = ny;
    L.nz = nz;
    L.kind = kind;
    const std::size_t cells = std::size_t{nx} * ny * nz;
    L.p.assign(cells, 0.0f);
    L.b.assign(cells, 0.0f);
    L.r.assign(cells, 0.0f);
    return L;
}

void divergence(const MacGrid& g, std::span<float> out) noexcept {
    const GridShape& s = g.shape;
    const float inv_dx = 1.0f / s.dx;
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) {
                const float du = g.u[u_index(s, i + 1u, j, k)] - g.u[u_index(s, i, j, k)];
                const float dv = g.v[v_index(s, i, j + 1u, k)] - g.v[v_index(s, i, j, k)];
                const float dw = g.w[w_index(s, i, j, k + 1u)] - g.w[w_index(s, i, j, k)];
                out[cell_index(s, i, j, k)] = ((du + dv) + dw) * inv_dx;
            }
        }
    }
}

void smooth_one_cell(PoissonLevel& L, uint32_t i, uint32_t j, uint32_t k) noexcept {
    const Stencil st = stencil(L, i, j, k);
    if (st.diag == 0.0f) return;
    const std::size_t c = at(L, i, j, k);
    L.p[c] = (st.sum - L.b[c]) / st.diag;
}

void smooth_red_black(PoissonLevel& L, uint32_t sweeps) noexcept {
    const std::size_t sy = L.nx;
    const std::size_t sz = std::size_t{L.nx} * L.ny;
    float* p = L.p.data();
    const float* b = L.b.data();
    for (uint32_t sweep = 0; sweep < sweeps; ++sweep) {
        for (uint32_t colour = 0; colour < 2; ++colour) {
            for (uint32_t k = 0; k < L.nz; ++k) {
                for (uint32_t j = 0; j < L.ny; ++j) {
                    // The cells of this colour in the row, in ascending i, as before.
                    const bool row_inside = inside(j, L.ny) && inside(k, L.nz);
                    for (uint32_t i = (j + k + colour) & 1u; i < L.nx; i += 2u) {
                        if (row_inside && inside(i, L.nx)) {
                            const std::size_t c = at(L, i, j, k);
                            p[c] = (interior_sum(p, c, sy, sz) - b[c]) / kInteriorDiag;
                        } else {
                            smooth_one_cell(L, i, j, k);
                        }
                    }
                }
            }
        }
    }
}

void residual(PoissonLevel& L) noexcept {
    const std::size_t sy = L.nx;
    const std::size_t sz = std::size_t{L.nx} * L.ny;
    const float* p = L.p.data();
    for (uint32_t k = 0; k < L.nz; ++k) {
        for (uint32_t j = 0; j < L.ny; ++j) {
            const bool row_inside = inside(j, L.ny) && inside(k, L.nz);
            for (uint32_t i = 0; i < L.nx; ++i) {
                const std::size_t c = at(L, i, j, k);
                if (row_inside && inside(i, L.nx)) {
                    L.r[c] = L.b[c] - (interior_sum(p, c, sy, sz) - kInteriorDiag * p[c]);
                    continue;
                }
                const Stencil st = stencil(L, i, j, k);
                L.r[c] = L.b[c] - (st.sum - st.diag * L.p[c]);
            }
        }
    }
}

}  // namespace spade::physics::airflow
