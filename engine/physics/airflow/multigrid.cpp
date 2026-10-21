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

uint64_t multigrid_faces(uint32_t nx, uint32_t ny, uint32_t nz, std::array<bool, 3> periodic) noexcept {
    const auto faces = [](const uint32_t (&d)[3]) {
        const uint64_t x = d[0];
        const uint64_t y = d[1];
        const uint64_t z = d[2];
        return (x + 1u) * y * z + x * (y + 1u) * z + x * y * (z + 1u);
    };
    uint32_t n[3] = {nx, ny, nz};
    uint64_t total = faces(n);
    uint32_t c[3] = {};
    while (coarser(n, periodic, c)) {
        n[0] = c[0];
        n[1] = c[1];
        n[2] = c[2];
        total += faces(n);
    }
    return total;
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

namespace {

// Along one axis, the fine indices whose interpolation reads coarse index I,
// with the weight they read it by: taps()'s transpose. A coarsened axis:
// 2I - 1 .. 2I + 2, wrapped on a periodic axis, dropped past an edge, a weight
// 0 dropped too; an axis not coarsened: I alone, weight 1.
struct Reach {
    uint32_t n = 0;
    uint32_t idx[4] = {};
    float w[4] = {};
};

[[nodiscard]] Reach reach(uint32_t I, uint32_t nf, uint32_t nc, bool periodic) noexcept {
    Reach r;
    if (nf == nc) {
        r.n = 1;
        r.idx[0] = I;
        r.w[0] = 1.0f;
        return r;
    }
    for (int d = -1; d <= 2; ++d) {
        int64_t i = 2 * int64_t{I} + d;
        if (i < 0 || i >= int64_t{nf}) {
            if (!periodic) continue;
            i = (i + int64_t{nf}) % int64_t{nf};
        }
        const Taps t = taps(static_cast<uint32_t>(i), nf, nc, periodic);
        float w = 0.0f;
        if (t.i0 == I) w += t.w0;
        if (t.i1 == I) w += t.w1;
        if (w == 0.0f) continue;
        r.idx[r.n] = static_cast<uint32_t>(i);
        r.w[r.n] = w;
        ++r.n;
    }
    return r;
}

void cycle_sym(Multigrid& mg, std::size_t l) noexcept {
    PoissonLevel& f = mg.levels[l];
    if (l + 1u == mg.levels.size()) {
        smooth_red_black(f, kCoarsestSymSweeps, false);
        smooth_red_black(f, kCoarsestSymSweeps, true);
        return;
    }
    smooth_red_black(f, kPreSweeps, false);
    residual(f);
    PoissonLevel& c = mg.levels[l + 1u];
    restrict_full_weighting(f, c);
    std::fill(c.p.begin(), c.p.end(), 0.0f);
    cycle_sym(mg, l + 1u);
    prolong_add(c, f);
    smooth_red_black(f, kPostSweeps, true);
}

}  // namespace

void restrict_full_weighting(const PoissonLevel& f, PoissonLevel& c) noexcept {
    const bool px = f.kind[kXMinus] == FaceKind::periodic;
    const bool py = f.kind[kYMinus] == FaceKind::periodic;
    const bool pz = f.kind[kZMinus] == FaceKind::periodic;
    const uint32_t sx = f.nx == c.nx ? 1u : 2u;
    const uint32_t sy = f.ny == c.ny ? 1u : 2u;
    const uint32_t sz = f.nz == c.nz ? 1u : 2u;
    const float scale = 4.0f / static_cast<float>(sx * sy * sz);
    for (uint32_t K = 0; K < c.nz; ++K) {
        const Reach rz = reach(K, f.nz, c.nz, pz);
        for (uint32_t J = 0; J < c.ny; ++J) {
            const Reach ry = reach(J, f.ny, c.ny, py);
            for (uint32_t I = 0; I < c.nx; ++I) {
                const Reach rx = reach(I, f.nx, c.nx, px);
                float sum = 0.0f;
                for (uint32_t a = 0; a < rz.n; ++a) {
                    for (uint32_t b = 0; b < ry.n; ++b) {
                        const float wzy = rz.w[a] * ry.w[b];
                        for (uint32_t d = 0; d < rx.n; ++d) {
                            sum += (wzy * rx.w[d]) * f.r[at(f, rx.idx[d], ry.idx[b], rz.idx[a])];
                        }
                    }
                }
                c.b[at(c, I, J, K)] = sum * scale;
            }
        }
    }
}

void precondition(Multigrid& mg) noexcept {
    if (mg.levels.empty()) return;
    std::fill(mg.levels[0].p.begin(), mg.levels[0].p.end(), 0.0f);
    cycle_sym(mg, 0);
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

void set_weights(Multigrid& mg, const std::array<std::span<const float>, 3>& fine) {
    for (uint32_t a = 0; a < 3; ++a) mg.levels[0].w[a].assign(fine[a].begin(), fine[a].end());
    for (std::size_t l = 1; l < mg.levels.size(); ++l) {
        const PoissonLevel& f = mg.levels[l - 1];
        PoissonLevel& c = mg.levels[l];
        const uint32_t nf[3] = {f.nx, f.ny, f.nz};
        const uint32_t nc[3] = {c.nx, c.ny, c.nz};
        for (uint32_t a = 0; a < 3; ++a) {
            const uint32_t t1 = a == 0 ? 1u : 0u;  // the transverse axes, t1 < t2
            const uint32_t t2 = a == 2 ? 1u : 2u;
            const uint32_t r[3] = {nf[0] == nc[0] ? 1u : 2u, nf[1] == nc[1] ? 1u : 2u, nf[2] == nc[2] ? 1u : 2u};
            const uint32_t fdim[3] = {a == 0 ? nf[0] + 1u : nf[0], a == 1 ? nf[1] + 1u : nf[1], a == 2 ? nf[2] + 1u : nf[2]};
            const uint32_t cdim[3] = {a == 0 ? nc[0] + 1u : nc[0], a == 1 ? nc[1] + 1u : nc[1], a == 2 ? nc[2] + 1u : nc[2]};
            const float inv = 1.0f / static_cast<float>(r[t1] * r[t2]);
            c.w[a].assign(std::size_t{cdim[0]} * cdim[1] * cdim[2], 0.0f);
            for (uint32_t K = 0; K < cdim[2]; ++K) {
                for (uint32_t J = 0; J < cdim[1]; ++J) {
                    for (uint32_t I = 0; I < cdim[0]; ++I) {
                        const uint32_t C[3] = {I, J, K};
                        float sum = 0.0f;
                        for (uint32_t d2 = 0; d2 < r[t2]; ++d2) {
                            for (uint32_t d1 = 0; d1 < r[t1]; ++d1) {
                                uint32_t F[3];
                                F[a] = C[a] * r[a];
                                F[t1] = C[t1] * r[t1] + d1;
                                F[t2] = C[t2] * r[t2] + d2;
                                sum += f.w[a][F[0] + std::size_t{fdim[0]} * (F[1] + std::size_t{fdim[1]} * F[2])];
                            }
                        }
                        c.w[a][I + std::size_t{cdim[0]} * (J + std::size_t{cdim[1]} * K)] = sum * inv;
                    }
                }
            }
        }
    }
}

void clear_weights(Multigrid& mg) noexcept {
    for (PoissonLevel& L : mg.levels) {
        for (std::vector<float>& w : L.w) w.clear();
    }
}

}  // namespace spade::physics::airflow
