#include "physics/airflow/multigrid.hpp"

#include <algorithm>

namespace spade::physics::airflow {

namespace {

[[nodiscard]] inline std::size_t at(const PoissonLevel& L, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{L.nx} * (j + std::size_t{L.ny} * k);
}

// Along one axis: the two coarse indices a fine index reads, and their weights.
struct Taps {
    uint32_t i0;
    uint32_t i1;
    float w0;
    float w1;
};

[[nodiscard]] Taps taps(uint32_t fine, uint32_t nf, uint32_t nc, bool periodic) noexcept {
    if (nf == nc) return {fine, fine, 1.0f, 0.0f};  // an axis of size 1 is not coarsened
    const uint32_t I = fine / 2u;
    if (fine % 2u == 0u) {
        const uint32_t other = I == 0u ? (periodic ? nc - 1u : 0u) : I - 1u;
        return {I, other, 0.75f, 0.25f};
    }
    const uint32_t other = I + 1u == nc ? (periodic ? 0u : I) : I + 1u;
    return {I, other, 0.75f, 0.25f};
}

// The coarsening rule, in one place: an axis of size 1 takes no part; a level
// is coarsened when every participating axis is even and at least 4, and every
// periodic one a multiple of 4 (it halves to an even ring), halving each.
// Returns false when `n` is the coarsest.
[[nodiscard]] bool coarser(const uint32_t n[3], const std::array<bool, 3>& periodic, uint32_t out[3]) noexcept {
    bool participates = false;
    for (uint32_t a = 0; a < 3; ++a) {
        if (n[a] == 1u) continue;
        participates = true;
        if (n[a] < 4u || n[a] % 2u != 0u) return false;
        if (periodic[a] && n[a] % 4u != 0u) return false;
    }
    if (!participates) return false;
    for (uint32_t a = 0; a < 3; ++a) out[a] = n[a] == 1u ? 1u : n[a] / 2u;
    return true;
}

void cycle(Multigrid& mg, std::size_t l) noexcept {
    PoissonLevel& f = mg.levels[l];
    if (l + 1u == mg.levels.size()) {
        smooth_red_black(f, kCoarsestSweeps);
        return;
    }
    smooth_red_black(f, kPreSweeps);
    residual(f);
    PoissonLevel& c = mg.levels[l + 1u];
    restrict_residual(f, c);
    std::fill(c.p.begin(), c.p.end(), 0.0f);
    cycle(mg, l + 1u);
    prolong_add(c, f);
    smooth_red_black(f, kPostSweeps);
}

}  // namespace

Multigrid make_multigrid(uint32_t nx, uint32_t ny, uint32_t nz, std::array<bool, 3> periodic) {
    Multigrid mg;
    mg.levels.push_back(make_level(nx, ny, nz, {}));
    for (;;) {
        const uint32_t n[3] = {mg.levels.back().nx, mg.levels.back().ny, mg.levels.back().nz};
        uint32_t c[3] = {};
        if (!coarser(n, periodic, c)) break;
        mg.levels.push_back(make_level(c[0], c[1], c[2], {}));
    }
    return mg;
}

uint64_t multigrid_cells(uint32_t nx, uint32_t ny, uint32_t nz, std::array<bool, 3> periodic) noexcept {
    uint32_t n[3] = {nx, ny, nz};
    uint64_t cells = uint64_t{nx} * ny * nz;
    uint32_t c[3] = {};
    while (coarser(n, periodic, c)) {
        cells += uint64_t{c[0]} * c[1] * c[2];
        n[0] = c[0];
        n[1] = c[1];
        n[2] = c[2];
    }
    return cells;
}

void set_kinds(Multigrid& mg, const std::array<FaceKind, 6>& kind) noexcept {
    for (PoissonLevel& L : mg.levels) L.kind = kind;
}

void restrict_residual(const PoissonLevel& f, PoissonLevel& c) noexcept {
    const uint32_t sx = f.nx == c.nx ? 1u : 2u;
    const uint32_t sy = f.ny == c.ny ? 1u : 2u;
    const uint32_t sz = f.nz == c.nz ? 1u : 2u;
    const float scale = 4.0f / static_cast<float>(sx * sy * sz);
    for (uint32_t K = 0; K < c.nz; ++K) {
        for (uint32_t J = 0; J < c.ny; ++J) {
            for (uint32_t I = 0; I < c.nx; ++I) {
                float sum = 0.0f;
                for (uint32_t dk = 0; dk < sz; ++dk) {
                    for (uint32_t dj = 0; dj < sy; ++dj) {
                        for (uint32_t di = 0; di < sx; ++di) {
                            sum += f.r[at(f, I * sx + di, J * sy + dj, K * sz + dk)];
                        }
                    }
                }
                c.b[at(c, I, J, K)] = sum * scale;
            }
        }
    }
}

void prolong_add(const PoissonLevel& c, PoissonLevel& f) noexcept {
    const bool px = f.kind[kXMinus] == FaceKind::periodic;
    const bool py = f.kind[kYMinus] == FaceKind::periodic;
    const bool pz = f.kind[kZMinus] == FaceKind::periodic;
    const auto q = [&](uint32_t a, uint32_t b, uint32_t d) { return c.p[at(c, a, b, d)]; };
    for (uint32_t k = 0; k < f.nz; ++k) {
        const Taps tz = taps(k, f.nz, c.nz, pz);
        for (uint32_t j = 0; j < f.ny; ++j) {
            const Taps ty = taps(j, f.ny, c.ny, py);
            for (uint32_t i = 0; i < f.nx; ++i) {
                const Taps tx = taps(i, f.nx, c.nx, px);
                const float z0 = ty.w0 * (tx.w0 * q(tx.i0, ty.i0, tz.i0) + tx.w1 * q(tx.i1, ty.i0, tz.i0)) +
                                 ty.w1 * (tx.w0 * q(tx.i0, ty.i1, tz.i0) + tx.w1 * q(tx.i1, ty.i1, tz.i0));
                const float z1 = ty.w0 * (tx.w0 * q(tx.i0, ty.i0, tz.i1) + tx.w1 * q(tx.i1, ty.i0, tz.i1)) +
                                 ty.w1 * (tx.w0 * q(tx.i0, ty.i1, tz.i1) + tx.w1 * q(tx.i1, ty.i1, tz.i1));
                f.p[at(f, i, j, k)] += tz.w0 * z0 + tz.w1 * z1;
            }
        }
    }
}

void v_cycle(Multigrid& mg) noexcept {
    if (!mg.levels.empty()) cycle(mg, 0);
}

}  // namespace spade::physics::airflow
