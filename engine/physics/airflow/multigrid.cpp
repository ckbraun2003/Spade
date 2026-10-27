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

// The coarsening rule, in one place. An axis of size 1 takes no part.
//   * Halving: when every participating axis is even and at least 4, and
//     every periodic one a multiple of 4 (it halves to an even ring), each
//     halves. This is plan A's rule, and every level it builds is as before.
//   * Past it (the lead's decision A, 2026-10-21): a level that halving
//     cannot coarsen but that still holds more than kCoarsestCells cells is
//     coarsened axis by axis, so an odd axis no longer leaves a grid with one
//     level (the D1 box, 240 x 177 x 168, solved on 7M cells by smoothing).
//     A non-periodic axis of 4 or more goes to ceil(n / 2): an odd axis's
//     last coarse cell covers one fine cell, and its missing partner is closed
//     (restrict_residual() and set_weights() leave it out). A periodic axis
//     that is not a multiple of 4, or any axis under 4, is held at n
//     (semi-coarsening): an odd ring's red-black colouring would make a sweep
//     depend on its order, and a closed partner would put a wall in a ring.
//     A held axis's cells are half as long as the others' on the coarser
//     level, so its faces weigh 4 times as much (set_weights()).
// Returns false when `n` is the coarsest.
[[nodiscard]] bool coarser(const uint32_t n[3], const std::array<bool, 3>& periodic, uint32_t out[3]) noexcept {
    bool participates = false;
    bool halves = true;
    for (uint32_t a = 0; a < 3; ++a) {
        if (n[a] == 1u) continue;
        participates = true;
        if (n[a] < 4u || n[a] % 2u != 0u || (periodic[a] && n[a] % 4u != 0u)) halves = false;
    }
    if (!participates) return false;
    if (halves) {
        for (uint32_t a = 0; a < 3; ++a) out[a] = n[a] == 1u ? 1u : n[a] / 2u;
        return true;
    }
    if (uint64_t{n[0]} * n[1] * n[2] <= kCoarsestCells) return false;
    bool any = false;
    for (uint32_t a = 0; a < 3; ++a) {
        const bool held = n[a] < 4u || (periodic[a] && n[a] % 4u != 0u);
        out[a] = held ? n[a] : periodic[a] ? n[a] / 2u : (n[a] + 1u) / 2u;
        any = any || out[a] != n[a];
    }
    return any;
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
    mg.periodic = periodic;
    mg.levels.push_back(make_level(nx, ny, nz, {}));
    for (;;) {
        const uint32_t n[3] = {mg.levels.back().nx, mg.levels.back().ny, mg.levels.back().nz};
        uint32_t c[3] = {};
        if (!coarser(n, periodic, c)) break;
        mg.levels.push_back(make_level(c[0], c[1], c[2], {}));
    }
    clear_weights(mg);  // plan A's weights past the halving rule, if any
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
                    const uint32_t k = K * sz + dk;
                    if (k >= f.nz) continue;  // an odd axis's last cell: its partner is closed
                    for (uint32_t dj = 0; dj < sy; ++dj) {
                        const uint32_t j = J * sy + dj;
                        if (j >= f.ny) continue;
                        for (uint32_t di = 0; di < sx; ++di) {
                            const uint32_t i = I * sx + di;
                            if (i >= f.nx) continue;
                            sum += f.r[at(f, i, j, k)];
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

namespace {

// Whether every level halves the one above on each participating axis: the
// halving rule's hierarchy, whose plan A levels need no weights.
[[nodiscard]] bool halved_throughout(const Multigrid& mg) noexcept {
    for (std::size_t l = 1; l < mg.levels.size(); ++l) {
        const PoissonLevel& f = mg.levels[l - 1];
        const PoissonLevel& c = mg.levels[l];
        const uint32_t nf[3] = {f.nx, f.ny, f.nz};
        const uint32_t nc[3] = {c.nx, c.ny, c.nz};
        for (uint32_t a = 0; a < 3; ++a) {
            if (nf[a] != 1u && nc[a] * 2u != nf[a]) return false;
        }
    }
    return true;
}

// The weight of a coarse face's path along its normal (P2c): from one coarse
// cell's centre to the next it crosses the face's own fine plane (`on`, a
// whole fine cell's length) and the fine plane inside each of the two cells
// (`lo` and `hi`, half a length each), so their conductances add in series:
// 2 / (0.5 / lo + 1 / on + 0.5 / hi). A plane that is missing (has_lo or
// has_hi false: an odd axis's partial last cell) drops out with its length,
// 1.5 / (...). Any closed plane closes the path. Weights of 1 give exactly 1.
[[nodiscard]] float series(bool has_lo, float lo, float on, bool has_hi, float hi) noexcept {
    if (on == 0.0f || (has_lo && lo == 0.0f) || (has_hi && hi == 0.0f)) return 0.0f;
    float resistance = 0.0f;
    float length = 0.0f;
    if (has_lo) {
        resistance += 0.5f / lo;
        length += 0.5f;
    }
    resistance += 1.0f / on;
    length += 1.0f;
    if (has_hi) {
        resistance += 0.5f / hi;
        length += 0.5f;
    }
    return length / resistance;
}

// Level c's face weights from level f's (an empty f.w: every weight 1).
// THE COARSE FACE RULE (P2c, the lead's order, 2026-10-21):
//   * Across the face, each fine plane its path crosses gives the mean of
//     the fine faces the coarse face covers on it (2 x 2, or 2 or 1 across an
//     axis not coarsened, summed with the second transverse axis outer; a
//     missing transverse partner is closed and counts 0).
//   * Along the face's normal those plane means are in series (series()):
//     the face's own plane and the plane inside the coarse cell on each side.
//     So a sheet or a wall on any fine plane reaches every coarser level.
//     Before, a coarse face read its own plane alone, every second fine plane,
//     and a sheet on another was invisible to it (the lead's finding 2): a
//     sphere centred on a cell centre then took 20 to 31 CG iterations where
//     one centred on a corner took 8 (measured 2026-10-21, 48^3 and 47^3 at
//     R/dx 6; now the floor of 8 for both).
//   * A face with no inner plane on either side keeps its own plane's mean:
//     a domain face (the weight enters only an outlet's ghost term), and every
//     face across a held axis (r = 1: a coarse cell is one fine cell long;
//     its weight x 4, as before). An odd axis's partial last cell has no inner
//     plane, so the face before it drops that half (series()). A periodic
//     axis wraps: face 0 reads the planes nf - 1, 0 and 1, and face nc, which
//     mirrors it, the same values in the same order.
//   * `on_plane` (plan A's levels, clear_weights()) keeps the mean of the
//     face's own plane everywhere, bit for bit as before.
// Every weight is a mean or a series of the level above's, so none is
// negative: each coarse operator stays a weighted graph Laplacian, and the
// symmetric cycle stays symmetric and definite (design test 10).
void coarsen_weights(const PoissonLevel& f, PoissonLevel& c, const std::array<bool, 3>& periodic, bool on_plane) {
    const uint32_t nf[3] = {f.nx, f.ny, f.nz};
    const uint32_t nc[3] = {c.nx, c.ny, c.nz};
    for (uint32_t a = 0; a < 3; ++a) {
        const uint32_t t1 = a == 0 ? 1u : 0u;  // the transverse axes, t1 < t2
        const uint32_t t2 = a == 2 ? 1u : 2u;
        const uint32_t r[3] = {nf[0] == nc[0] ? 1u : 2u, nf[1] == nc[1] ? 1u : 2u, nf[2] == nc[2] ? 1u : 2u};
        const uint32_t fdim[3] = {a == 0 ? nf[0] + 1u : nf[0], a == 1 ? nf[1] + 1u : nf[1], a == 2 ? nf[2] + 1u : nf[2]};
        const uint32_t cdim[3] = {a == 0 ? nc[0] + 1u : nc[0], a == 1 ? nc[1] + 1u : nc[1], a == 2 ? nc[2] + 1u : nc[2]};
        const float inv = 1.0f / static_cast<float>(r[t1] * r[t2]);
        // A held axis (not coarsened, more than one cell): its cells are half
        // as long as the coarsened axes', so its faces weigh 4 times as much.
        const bool held = r[a] == 1u && nf[a] > 1u;
        // The mean over the face's fine columns on fine plane `plane`.
        const auto plane_mean = [&](uint32_t plane, const uint32_t (&C)[3]) {
            float sum = 0.0f;
            for (uint32_t d2 = 0; d2 < r[t2]; ++d2) {
                for (uint32_t d1 = 0; d1 < r[t1]; ++d1) {
                    uint32_t F[3];
                    F[a] = plane;
                    F[t1] = C[t1] * r[t1] + d1;
                    F[t2] = C[t2] * r[t2] + d2;
                    if (F[t1] >= nf[t1] || F[t2] >= nf[t2]) continue;  // a missing partner: closed
                    sum += f.w[a].empty()
                               ? 1.0f
                               : f.w[a][F[0] + std::size_t{fdim[0]} * (F[1] + std::size_t{fdim[1]} * F[2])];
                }
            }
            return sum * inv;
        };
        c.w[a].assign(std::size_t{cdim[0]} * cdim[1] * cdim[2], 0.0f);
        for (uint32_t K = 0; K < cdim[2]; ++K) {
            for (uint32_t J = 0; J < cdim[1]; ++J) {
                for (uint32_t I = 0; I < cdim[0]; ++I) {
                    const uint32_t C[3] = {I, J, K};
                    // An odd axis's last coarse face is the domain's face.
                    const uint32_t on = std::min(C[a] * r[a], nf[a]);
                    // The inner planes: of cell C - 1 below the face, of cell C above.
                    bool has_lo = false;
                    bool has_hi = false;
                    uint32_t lo = 0;
                    uint32_t hi = 0;
                    if (!on_plane && r[a] == 2u) {
                        if (periodic[a]) {
                            has_lo = has_hi = true;
                            lo = (on == 0u ? nf[a] : on) - 1u;
                            hi = (on == nf[a] ? 0u : on) + 1u;
                        } else if (C[a] > 0u && C[a] < nc[a]) {
                            has_lo = true;
                            lo = on - 1u;
                            has_hi = on + 1u < nf[a];  // not an odd axis's partial last cell
                            hi = on + 1u;
                        }
                    }
                    const float own = plane_mean(on, C);
                    const float w = has_lo || has_hi ? series(has_lo, has_lo ? plane_mean(lo, C) : 0.0f, own, has_hi,
                                                              has_hi ? plane_mean(hi, C) : 0.0f)
                                                     : own;
                    c.w[a][I + std::size_t{cdim[0]} * (J + std::size_t{cdim[1]} * K)] = held ? 4.0f * w : w;
                }
            }
        }
    }
}

// THE PARTIAL CELL'S TRUE STENCIL (C4, the lead's ruling, 2026-10-27), on
// level c of plan A's weights. `length` is the domain's length in c's cells
// along each axis (the finest's sizes, halved at each level that coarsens the
// axis), so an odd axis's last cell covers t = length - (n - 1) of a whole
// cell (177: 0.5, 0.25, 0.125, 0.0625 down the levels), exactly: a dyadic
// fraction. The stencil took it as whole, so an outlet's ghost sat t / 2 too
// far and the cell's centre 1/2 - t/2 too far from the one before it: the
// domain face beyond it now weighs x 1 / t (its ghost term, diag += 2 w,
// sees the face t / 2 away) and the face before it x 2 / (1 + t) (the
// centres (1 + t) / 2 apart). Its transverse faces already carry t (the
// closed partner, coarsen_weights()) and its b the partial volume
// (restrict_residual()). A periodic axis halves exactly, never partial.
void true_partial_cells(PoissonLevel& c, const double (&length)[3], const std::array<bool, 3>& periodic) noexcept {
    const uint32_t nc[3] = {c.nx, c.ny, c.nz};
    for (uint32_t a = 0; a < 3; ++a) {
        const double t = length[a] - static_cast<double>(nc[a] - 1u);
        if (periodic[a] || t >= 1.0) continue;
        const auto outer = static_cast<float>(1.0 / t);
        const auto inner = static_cast<float>(2.0 / (1.0 + t));
        const uint32_t t1 = a == 0 ? 1u : 0u;  // the transverse axes, t1 < t2
        const uint32_t t2 = a == 2 ? 1u : 2u;
        const uint32_t dim[3] = {a == 0 ? nc[0] + 1u : nc[0], a == 1 ? nc[1] + 1u : nc[1], a == 2 ? nc[2] + 1u : nc[2]};
        for (uint32_t q = 0; q < nc[t2]; ++q) {
            for (uint32_t p = 0; p < nc[t1]; ++p) {
                uint32_t C[3];
                C[t1] = p;
                C[t2] = q;
                C[a] = nc[a] - 1u;  // the face before the partial cell
                c.w[a][C[0] + std::size_t{dim[0]} * (C[1] + std::size_t{dim[1]} * C[2])] *= inner;
                C[a] = nc[a];  // the domain's face
                c.w[a][C[0] + std::size_t{dim[0]} * (C[1] + std::size_t{dim[1]} * C[2])] *= outer;
            }
        }
    }
}

}  // namespace

void set_weights(Multigrid& mg, const std::array<std::span<const float>, 3>& fine) {
    for (uint32_t a = 0; a < 3; ++a) mg.levels[0].w[a].assign(fine[a].begin(), fine[a].end());
    for (std::size_t l = 1; l < mg.levels.size(); ++l) {
        coarsen_weights(mg.levels[l - 1], mg.levels[l], mg.periodic, false);
    }
}

void clear_weights(Multigrid& mg) {
    for (PoissonLevel& L : mg.levels) {
        for (std::vector<float>& w : L.w) w.clear();
    }
    if (halved_throughout(mg)) return;
    // Past the halving rule plan A's coarser levels carry the weights of
    // unit fine faces: a closed partner, a held axis and a partial cell (C4,
    // below) are weights. The finest keeps plan A's stencil and its fast
    // paths.
    for (std::size_t l = 1; l < mg.levels.size(); ++l) {
        coarsen_weights(mg.levels[l - 1], mg.levels[l], mg.periodic, true);  // the on-plane mean
    }
    // Then each level's partial cells (C4), each from its own t alone, once
    // every level is coarsened: coarsen_weights() reads the domain face of the
    // level above, so a factor applied in the loop above would compound (1 /
    // (t1 t2) on level 2). Every level here holds weights: a partial cell
    // fails halved_throughout(). The solids' set_weights() takes no part.
    double length[3] = {static_cast<double>(mg.levels[0].nx), static_cast<double>(mg.levels[0].ny),
                        static_cast<double>(mg.levels[0].nz)};
    for (std::size_t l = 1; l < mg.levels.size(); ++l) {
        const PoissonLevel& f = mg.levels[l - 1];
        PoissonLevel& c = mg.levels[l];
        if (c.nx != f.nx) length[0] *= 0.5;
        if (c.ny != f.ny) length[1] *= 0.5;
        if (c.nz != f.nz) length[2] *= 0.5;
        true_partial_cells(c, length, mg.periodic);
    }
}

std::array<uint32_t, 3> multigrid_coarsest(uint32_t nx, uint32_t ny, uint32_t nz,
                                           std::array<bool, 3> periodic) noexcept {
    uint32_t n[3] = {nx, ny, nz};
    uint32_t c[3] = {};
    while (coarser(n, periodic, c)) {
        n[0] = c[0];
        n[1] = c[1];
        n[2] = c[2];
    }
    return {n[0], n[1], n[2]};
}

uint64_t multigrid_plan_a_weight_faces(uint32_t nx, uint32_t ny, uint32_t nz, std::array<bool, 3> periodic) noexcept {
    uint32_t n[3] = {nx, ny, nz};
    uint32_t c[3] = {};
    uint64_t total = 0;
    bool halved = true;
    while (coarser(n, periodic, c)) {
        for (uint32_t a = 0; a < 3; ++a) {
            if (n[a] != 1u && c[a] * 2u != n[a]) halved = false;
        }
        total += (uint64_t{c[0]} + 1u) * c[1] * c[2] + uint64_t{c[0]} * (c[1] + 1u) * c[2] +
                 uint64_t{c[0]} * c[1] * (c[2] + 1u);
        n[0] = c[0];
        n[1] = c[1];
        n[2] = c[2];
    }
    return halved ? 0u : total;
}

}  // namespace spade::physics::airflow
