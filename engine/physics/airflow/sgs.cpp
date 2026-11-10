#include "physics/airflow/sgs.hpp"

#include <algorithm>
#include <cmath>

namespace spade::physics::airflow {

namespace {

constexpr int64_t kNone = -1;

struct Geo {
    uint32_t n[3]{};
    bool periodic[3]{};
    const float* nut = nullptr;
    const uint8_t* solid = nullptr;  // per cell, or null
    const DomainBc* bc = nullptr;
    const SolidField* sf = nullptr;  // null without walls
    float k = 0.0f;                  // h / dx^2
};

[[nodiscard]] std::size_t cell_at(const Geo& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.n[0]} * (j + std::size_t{g.n[1]} * k);
}

struct Mean {
    float sum = 0.0f;
    uint32_t count = 0;
    void add(const Geo& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
        const std::size_t c = cell_at(g, i, j, k);
        if (g.solid != nullptr && g.solid[c] != 0u) return;
        sum += g.nut[c];
        ++count;
    }
    [[nodiscard]] float value() const noexcept { return count == 0u ? 0.0f : sum / static_cast<float>(count); }
};

// The cell index one step along `axis` from q, wrapped on a periodic axis; kNone past a non-periodic edge.
[[nodiscard]] int64_t step_cell(const Geo& g, uint32_t axis, uint32_t q, bool up) noexcept {
    if (up) {
        if (q + 1u < g.n[axis]) return q + 1;
        return g.periodic[axis] ? 0 : kNone;
    }
    if (q > 0u) return q - 1;
    return g.periodic[axis] ? int64_t{g.n[axis]} - 1 : kNone;
}

// The change d (before the factor h / dx^2) of component c's sample (i, j, kz) of `x`, with the engaged-limit flag.
// c's own axis has the component's n + 1 samples; the others n.
float face_change(const Geo& g, uint32_t c, const std::vector<float>& x, const uint32_t cnt[3], uint32_t i, uint32_t j,
                  uint32_t kz, bool& limited) noexcept {
    const std::size_t here = i + std::size_t{cnt[0]} * (j + std::size_t{cnt[1]} * kz);
    const WallLinks* wl = nullptr;
    if (g.sf != nullptr) {
        if (closed(g.sf->theta[c][here]) || g.sf->phi[c][here] < 0.0f) return 0.0f;
        if (g.sf->link[c][here] != kNoWallLinks) wl = &g.sf->links[g.sf->link[c][here]];
    }
    const uint32_t idx[3] = {i, j, kz};
    const uint32_t own = idx[c];
    const float x0 = x[here];
    const auto cl = static_cast<glm::length_t>(c);
    float d = 0.0f;
    float weight = 0.0f;  // the sum of the links' nu_t (1 / f) weights, for the bound
    for (uint32_t a = 0; a < 3; ++a) {
        for (uint32_t dir = 0; dir < 2; ++dir) {
            const bool up = dir == 1u;
            // The link's neighbour sample along a, and nu_t on the link.
            uint32_t nb[3] = {i, j, kz};
            bool exists = true;
            Mean m;
            if (a == c) {
                if (g.periodic[c] && g.n[c] == 1u) continue;  // its own neighbour
                // Samples own = 0 and n are boundaries; a periodic axis wraps 0 and n - 1.
                const int64_t q = up ? (g.periodic[c] && own + 1u == g.n[c] ? 0 : int64_t{own} + 1)
                                     : (own == 0u ? int64_t{g.n[c]} - 1 : int64_t{own} - 1);
                nb[a] = static_cast<uint32_t>(q);
                // The cell between the two samples: own when up, own - 1 below.
                uint32_t cc[3] = {i, j, kz};
                cc[c] = up ? own : (own == 0u ? g.n[c] - 1u : own - 1u);
                m.add(g, cc[0], cc[1], cc[2]);
            } else {
                const int64_t q = step_cell(g, a, idx[a], up);
                exists = q != kNone;
                // The edge's cells: along c the cells own - 1 and own, along a the cell idx[a] and its neighbour's.
                const uint32_t c0 = own == 0u ? g.n[c] - 1u : own - 1u;
                const uint32_t c1 = own < g.n[c] ? own : g.n[c] - 1u;
                for (const uint32_t cc_c : {c0, c1}) {
                    for (int side = 0; side < 2; ++side) {
                        uint32_t cc[3] = {i, j, kz};
                        cc[c] = cc_c;
                        if (side == 1) {
                            if (!exists) continue;
                            cc[a] = static_cast<uint32_t>(q);
                        }
                        m.add(g, cc[0], cc[1], cc[2]);
                    }
                }
                if (exists) nb[a] = static_cast<uint32_t>(q);
            }
            const float nu = m.value();
            if (wl != nullptr && wl->inv_f[2u * a + dir] > 0.0f) {
                const float w = nu * wl->inv_f[2u * a + dir];
                weight += w;
                d += w * (wl->wall[2u * a + dir] - x0);
                continue;
            }
            if (exists) {
                weight += nu;
                d += nu * (x[nb[0] + std::size_t{cnt[0]} * (nb[1] + std::size_t{cnt[1]} * nb[2])] - x0);
                continue;
            }
            const auto side = static_cast<Side>(2u * a + dir);
            switch (g.bc->face[side]) {
                case FaceBc::periodic:
                    break;  // handled by `exists` (a wrap exists unless the axis is 1 cell)
                case FaceBc::wall:  // no slip: the ghost 2 U - x
                    weight += 2.0f * nu;
                    d += 2.0f * nu * (g.bc->wall_velocity[side][cl] - x0);
                    break;
                case FaceBc::open:
                    if (face_kind(*g.bc, side) == FaceKind::fixed_velocity) {  // an inflow
                        weight += 2.0f * nu;
                        d += 2.0f * nu * (g.bc->ambient[cl] - x0);
                    }  // an outlet: the zero-gradient ghost, no term
                    break;
            }
        }
    }
    const float load = g.k * weight;
    if (load > 1.0f) {
        limited = true;
        return d / load;
    }
    return d;
}

// d(q)/d(axis) at a cell centre by central differences, one-sided at the box's faces.
float grad(const GridShape& sh, const std::vector<float>& q, uint32_t i, uint32_t j, uint32_t k, int axis) noexcept {
    const uint32_t n3[3] = {sh.nx, sh.ny, sh.nz};
    const uint32_t a[3] = {i, j, k};
    const uint32_t lo = a[axis] > 0u ? a[axis] - 1u : 0u;
    const uint32_t hi = a[axis] + 1u < n3[axis] ? a[axis] + 1u : n3[axis] - 1u;
    uint32_t pl[3] = {i, j, k};
    uint32_t ph[3] = {i, j, k};
    pl[axis] = lo;
    ph[axis] = hi;
    const float span = static_cast<float>(hi - lo) * sh.dx;
    return (q[cell_index(sh, ph[0], ph[1], ph[2])] - q[cell_index(sh, pl[0], pl[1], pl[2])]) / span;
}

template <class F>
void each(WorkerPool* pool, uint32_t items, F&& f) {
    if (pool != nullptr) {
        pool->run(items, f);
        return;
    }
    for (uint32_t t = 0; t < items; ++t) f(t);
}

}  // namespace

bool sgs_cs_valid(float cs) noexcept { return cs == 0.0f || (std::isfinite(cs) && cs > 0.0f && cs <= kSgsCsMax); }

void sgs_size(SgsScratch& s, const GridShape& sh) {
    const std::size_t cells = std::size_t{sh.nx} * sh.ny * sh.nz;
    for (auto* a : {&s.nut, &s.uc, &s.vc, &s.wc}) a->assign(cells, 0.0f);
    s.du.assign(std::size_t{sh.nx + 1u} * sh.ny * sh.nz, 0.0f);
    s.dv.assign(std::size_t{sh.nx} * (sh.ny + 1u) * sh.nz, 0.0f);
    s.dw.assign(std::size_t{sh.nx} * sh.ny * (sh.nz + 1u), 0.0f);
    s.capped.assign(sh.nz, 0u);
    s.peak.assign(sh.nz, 0.0f);
    s.limited.assign(std::size_t{3} * (sh.nz + 1u), 0u);
}

SgsReport sgs_diffuse(MacGrid& g, const DomainBc& bc, const SolidField* solids, const uint8_t* solid_cell, float cs,
                      float h, SgsScratch& s, WorkerPool* pool) noexcept {
    const GridShape& sh = g.shape;
    const uint32_t nz = sh.nz;
    const float dx = sh.dx;
    Geo geo;
    geo.n[0] = sh.nx;
    geo.n[1] = sh.ny;
    geo.n[2] = sh.nz;
    geo.periodic[0] = bc.face[kXMinus] == FaceBc::periodic;
    geo.periodic[1] = bc.face[kYMinus] == FaceBc::periodic;
    geo.periodic[2] = bc.face[kZMinus] == FaceBc::periodic;
    geo.nut = s.nut.data();
    geo.solid = solids != nullptr && solids->any ? solid_cell : nullptr;
    geo.bc = &bc;
    geo.sf = solids != nullptr && solids->any ? solids : nullptr;
    geo.k = h / (dx * dx);
    const float cap = dx * dx / (8.0f * h);
    const float c2 = cs * cs * dx * dx;

    // Phase 1: the centre velocities, per cell plane.
    each(pool, nz, [&](uint32_t k) {
        for (uint32_t j = 0; j < sh.ny; ++j) {
            for (uint32_t i = 0; i < sh.nx; ++i) {
                const std::size_t c = cell_index(sh, i, j, k);
                s.uc[c] = 0.5f * (g.u[u_index(sh, i, j, k)] + g.u[u_index(sh, i + 1u, j, k)]);
                s.vc[c] = 0.5f * (g.v[v_index(sh, i, j, k)] + g.v[v_index(sh, i, j + 1u, k)]);
                s.wc[c] = 0.5f * (g.w[w_index(sh, i, j, k)] + g.w[w_index(sh, i, j, k + 1u)]);
            }
        }
    });
    // Phase 2: nu_t per cell, from the strain, held at the cap; the plane's count and peak go in its own slot.
    each(pool, nz, [&](uint32_t k) {
        uint32_t capped = 0;
        float peak = 0.0f;
        for (uint32_t j = 0; j < sh.ny; ++j) {
            for (uint32_t i = 0; i < sh.nx; ++i) {
                const std::size_t c = cell_index(sh, i, j, k);
                const float sxx = (g.u[u_index(sh, i + 1u, j, k)] - g.u[u_index(sh, i, j, k)]) / dx;
                const float syy = (g.v[v_index(sh, i, j + 1u, k)] - g.v[v_index(sh, i, j, k)]) / dx;
                const float szz = (g.w[w_index(sh, i, j, k + 1u)] - g.w[w_index(sh, i, j, k)]) / dx;
                const float sxy = 0.5f * (grad(sh, s.uc, i, j, k, 1) + grad(sh, s.vc, i, j, k, 0));
                const float sxz = 0.5f * (grad(sh, s.uc, i, j, k, 2) + grad(sh, s.wc, i, j, k, 0));
                const float syz = 0.5f * (grad(sh, s.vc, i, j, k, 2) + grad(sh, s.wc, i, j, k, 1));
                const float ss = sxx * sxx + syy * syy + szz * szz + 2.0f * (sxy * sxy + sxz * sxz + syz * syz);
                float nut = c2 * std::sqrt(2.0f * ss);
                if (geo.solid != nullptr && geo.solid[c] != 0u) nut = 0.0f;
                peak = std::max(peak, nut);
                if (nut > cap) {
                    nut = cap;
                    ++capped;
                }
                s.nut[c] = nut;
            }
        }
        s.capped[k] = capped;
        s.peak[k] = peak;
    });
    // Phase 3: each face's change, from the old field and nu_t. One item per component plane.
    const uint32_t planes[3] = {nz, nz, nz + 1u};
    const uint32_t items = planes[0] + planes[1] + planes[2];
    std::vector<float>* comp[3] = {&g.u, &g.v, &g.w};
    std::vector<float>* rate[3] = {&s.du, &s.dv, &s.dw};
    const uint32_t cnt[3][3] = {{sh.nx + 1u, sh.ny, sh.nz}, {sh.nx, sh.ny + 1u, sh.nz}, {sh.nx, sh.ny, sh.nz + 1u}};
    each(pool, items, [&](uint32_t item) {
        uint32_t c = 0;
        while (item >= planes[c]) item -= planes[c++];
        const uint32_t* n = cnt[c];
        const std::vector<float>& x = *comp[c];
        std::vector<float>& out = *rate[c];
        uint32_t limited = 0;
        for (uint32_t j = 0; j < n[1]; ++j) {
            for (uint32_t i = 0; i < n[0]; ++i) {
                const uint32_t idx[3] = {i, j, item};
                const std::size_t at = i + std::size_t{n[0]} * (j + std::size_t{n[1]} * item);
                // Samples own = 0 and n are boundaries; a periodic axis solves 0 and leaves n to its alias.
                const uint32_t own = idx[c];
                const bool skip = geo.periodic[c] ? own == geo.n[c] : (own == 0u || own == geo.n[c]);
                if (skip) {
                    out[at] = 0.0f;
                    continue;
                }
                bool lim = false;
                out[at] = geo.k * face_change(geo, c, x, n, i, j, item, lim);
                if (lim) ++limited;
            }
        }
        s.limited[std::size_t{c} * (nz + 1u) + item] = limited;
    });
    // Phase 4: the add.
    each(pool, items, [&](uint32_t item) {
        uint32_t c = 0;
        while (item >= planes[c]) item -= planes[c++];
        const uint32_t* n = cnt[c];
        std::vector<float>& x = *comp[c];
        const std::vector<float>& r = *rate[c];
        const std::size_t plane = std::size_t{n[0]} * n[1];
        for (std::size_t t = plane * item; t < plane * (item + 1u); ++t) x[t] += r[t];
    });

    SgsReport rep;
    for (uint32_t k = 0; k < nz; ++k) {
        rep.capped_cells += s.capped[k];
        rep.nu_t_max = std::max(rep.nu_t_max, s.peak[k]);
    }
    for (const uint32_t l : s.limited) rep.limited_faces += l;
    return rep;
}

}  // namespace spade::physics::airflow
