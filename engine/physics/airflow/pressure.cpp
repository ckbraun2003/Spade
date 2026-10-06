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
    for (uint32_t sweep = 0; sweep < sweeps; ++sweep) {
        for (uint32_t colour = 0; colour < 2; ++colour) {
            for (uint32_t k = 0; k < L.nz; ++k) {
                for (uint32_t j = 0; j < L.ny; ++j) {
                    for (uint32_t i = 0; i < L.nx; ++i) {
                        if (((i + j + k) & 1u) == colour) smooth_one_cell(L, i, j, k);
                    }
                }
            }
        }
    }
}

void residual(PoissonLevel& L) noexcept {
    for (uint32_t k = 0; k < L.nz; ++k) {
        for (uint32_t j = 0; j < L.ny; ++j) {
            for (uint32_t i = 0; i < L.nx; ++i) {
                const Stencil st = stencil(L, i, j, k);
                const std::size_t c = at(L, i, j, k);
                L.r[c] = L.b[c] - (st.sum - st.diag * L.p[c]);
            }
        }
    }
}

}  // namespace spade::physics::airflow
