#include "physics/airflow/projection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "physics/airflow/reduce.hpp"

namespace spade::physics::airflow {

namespace {

// One axis' normal faces: q along the axis (0..n), (a, b) across it. `vel`
// addresses the component, `cell(q, a, b)` the pressure cell at q.
template <class Vel, class Cell>
void subtract_gradient(uint32_t n, uint32_t m1, uint32_t m2, FaceKind lo, FaceKind hi, float g_coef,
                       const std::vector<float>& p, Vel vel, Cell cell) noexcept {
    for (uint32_t b = 0; b < m2; ++b) {
        for (uint32_t a = 0; a < m1; ++a) {
            for (uint32_t q = 1; q < n; ++q) {
                vel(q, a, b) -= g_coef * (p[cell(q, a, b)] - p[cell(q - 1u, a, b)]);
            }
            const float p_first = p[cell(0, a, b)];
            const float p_last = p[cell(n - 1u, a, b)];
            if (lo == FaceKind::periodic) {
                if (n > 1u) vel(0, a, b) -= g_coef * (p_first - p_last);
                vel(n, a, b) = vel(0, a, b);
                continue;
            }
            if (lo == FaceKind::pressure_outlet) vel(0, a, b) -= g_coef * (p_first - (-p_first));
            if (hi == FaceKind::pressure_outlet) vel(n, a, b) -= g_coef * ((-p_last) - p_last);
        }
    }
}

struct GaugeCtx {
    FluidRegions* regions;
    const GridShape* shape;
};

void gauge_regions(void* ctx, std::span<float> x) noexcept {
    const auto* c = static_cast<const GaugeCtx*>(ctx);
    remove_region_means(x, *c->regions, *c->shape);
}

// subtract_gradient() with apertures: a closed face (theta = 0) takes the
// wall's velocity; an open one subtracts as plan A does. A fixed-velocity
// domain face is left as the boundaries wrote it.
template <class Vel, class Cell, class Face>
void subtract_gradient_solids(uint32_t n, uint32_t m1, uint32_t m2, FaceKind lo, FaceKind hi, float g_coef,
                              const std::vector<float>& p, Vel vel, Cell cell, Face face, const std::vector<float>& theta,
                              const std::vector<float>& wall) noexcept {
    for (uint32_t b = 0; b < m2; ++b) {
        for (uint32_t a = 0; a < m1; ++a) {
            for (uint32_t q = 1; q < n; ++q) {
                const std::size_t f = face(q, a, b);
                if (closed(theta[f])) {
                    vel(q, a, b) = wall[f];
                } else {
                    vel(q, a, b) -= g_coef * (p[cell(q, a, b)] - p[cell(q - 1u, a, b)]);
                }
            }
            const float p_first = p[cell(0, a, b)];
            const float p_last = p[cell(n - 1u, a, b)];
            const std::size_t f0 = face(0, a, b);
            const std::size_t fn = face(n, a, b);
            if (lo == FaceKind::periodic) {
                if (closed(theta[f0])) {
                    vel(0, a, b) = wall[f0];
                } else if (n > 1u) {
                    vel(0, a, b) -= g_coef * (p_first - p_last);
                }
                vel(n, a, b) = vel(0, a, b);
                continue;
            }
            if (lo == FaceKind::pressure_outlet) {
                if (closed(theta[f0])) {
                    vel(0, a, b) = wall[f0];
                } else {
                    vel(0, a, b) -= g_coef * (p_first - (-p_first));
                }
            }
            if (hi == FaceKind::pressure_outlet) {
                if (closed(theta[fn])) {
                    vel(n, a, b) = wall[fn];
                } else {
                    vel(n, a, b) -= g_coef * ((-p_last) - p_last);
                }
            }
        }
    }
}

ProjectionReport project_solids(MacGrid& g, const DomainBc& bc, float rho, float h, uint32_t n_v, uint32_t cg_cap,
                                ProjectionScratch& s, const SolidField& f, FluidRegions& regions) noexcept {
    const GridShape& sh = g.shape;
    const std::array<FaceKind, 6> kinds = face_kinds(bc);
    set_kinds(s.mg, kinds);
    const float inv_dx = 1.0f / sh.dx;
    const float b_coef = rho * sh.dx * sh.dx / h;
    const auto flux = [&](uint32_t c, std::size_t idx, const std::vector<float>& vel) {
        const float t = f.theta[c][idx];
        return t * vel[idx] + (1.0f - t) * f.wall[c][idx];
    };
    for (uint32_t k = 0; k < sh.nz; ++k) {
        for (uint32_t j = 0; j < sh.ny; ++j) {
            for (uint32_t i = 0; i < sh.nx; ++i) {
                const std::size_t c = cell_index(sh, i, j, k);
                if (regions.label[c] == kNoRegion) {
                    s.pcg.b[c] = 0.0f;
                    continue;
                }
                const float du = flux(0, u_index(sh, i + 1u, j, k), g.u) - flux(0, u_index(sh, i, j, k), g.u);
                const float dv = flux(1, v_index(sh, i, j + 1u, k), g.v) - flux(1, v_index(sh, i, j, k), g.v);
                const float dw = flux(2, w_index(sh, i, j, k + 1u), g.w) - flux(2, w_index(sh, i, j, k), g.w);
                s.pcg.b[c] = b_coef * (((du + dv) + dw) * inv_dx);
            }
        }
    }
    remove_region_means(s.pcg.b, regions, sh);
    std::copy(g.p.begin(), g.p.end(), s.pcg.x.begin());
    GaugeCtx ctx{&regions, &sh};
    const PcgResult solved = solve_pcg_to(s.mg, s.pcg, n_v, cg_cap, kResidualAnnounce,
                                          PcgGauge{&gauge_regions, &ctx, regions.solid.data()});
    std::copy(s.pcg.x.begin(), s.pcg.x.end(), g.p.begin());

    const float g_coef = h / (rho * sh.dx);
    subtract_gradient_solids(
        sh.nx, sh.ny, sh.nz, kinds[kXMinus], kinds[kXPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.u[u_index(sh, q, a, b)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, q, a, b); },
        [&](uint32_t q, uint32_t a, uint32_t b) { return u_index(sh, q, a, b); }, f.theta[0], f.wall[0]);
    subtract_gradient_solids(
        sh.ny, sh.nx, sh.nz, kinds[kYMinus], kinds[kYPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.v[v_index(sh, a, q, b)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, a, q, b); },
        [&](uint32_t q, uint32_t a, uint32_t b) { return v_index(sh, a, q, b); }, f.theta[1], f.wall[1]);
    subtract_gradient_solids(
        sh.nz, sh.nx, sh.ny, kinds[kZMinus], kinds[kZPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.w[w_index(sh, a, b, q)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, a, b, q); },
        [&](uint32_t q, uint32_t a, uint32_t b) { return w_index(sh, a, b, q); }, f.theta[2], f.wall[2]);
    return {solved.residual, solved.iterations};
}

}  // namespace

ProjectionScratch make_projection_scratch(const GridShape& shape, const DomainBc& bc) {
    ProjectionScratch s;
    s.mg = make_multigrid(shape.nx, shape.ny, shape.nz, periodic_axes(bc));
    s.div.assign(std::size_t{shape.nx} * shape.ny * shape.nz, 0.0f);
    return s;
}

ProjectionReport project(MacGrid& g, const DomainBc& bc, float rho, float h, uint32_t n_v, ProjectionScratch& s,
                         const SolidField* solids, FluidRegions* regions, uint32_t cg_cap) noexcept {
    if (solids != nullptr && solids->any) {
        // Solids without their regions cannot be projected: the grid is left
        // as it is and the residual is infinite, which a step announces (L6),
        // never plan A over the solids (P2 Task 11, the final review's finding 4).
        if (regions == nullptr) return ProjectionReport{std::numeric_limits<float>::infinity()};
        return project_solids(g, bc, rho, h, n_v, cg_cap, s, *solids, *regions);
    }
    const GridShape& sh = g.shape;
    const std::array<FaceKind, 6> kinds = face_kinds(bc);
    set_kinds(s.mg, kinds);
    divergence(g, s.div);
    PoissonLevel& f = s.mg.levels[0];
    const float b_coef = rho * sh.dx * sh.dx / h;
    for (std::size_t c = 0; c < f.b.size(); ++c) f.b[c] = b_coef * s.div[c];
    const bool closed = std::none_of(kinds.begin(), kinds.end(),
                                     [](FaceKind k) { return k == FaceKind::pressure_outlet; });
    if (closed) remove_mean(f.b, sh.nx, sh.ny, sh.nz);
    std::copy(g.p.begin(), g.p.end(), f.p.begin());
    for (uint32_t n = 0; n < n_v; ++n) {
        v_cycle(s.mg);
        if (closed) remove_mean(f.p, sh.nx, sh.ny, sh.nz);
    }
    std::copy(f.p.begin(), f.p.end(), g.p.begin());
    // The solve's own measure, as the solids path reports it (the lead,
    // 2026-10-21): max|r| / max|b| after the cycles. Reported, never iterated
    // on; the pressure is already final.
    residual(f);
    float r_max = 0.0f;
    float b_max = 0.0f;
    for (std::size_t c = 0; c < f.r.size(); ++c) {
        r_max = std::max(r_max, std::fabs(f.r[c]));
        b_max = std::max(b_max, std::fabs(f.b[c]));
    }
    const float solve_residual = b_max > 0.0f ? r_max / b_max : 0.0f;

    const float g_coef = h / (rho * sh.dx);
    subtract_gradient(
        sh.nx, sh.ny, sh.nz, kinds[kXMinus], kinds[kXPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.u[u_index(sh, q, a, b)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, q, a, b); });
    subtract_gradient(
        sh.ny, sh.nx, sh.nz, kinds[kYMinus], kinds[kYPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.v[v_index(sh, a, q, b)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, a, q, b); });
    subtract_gradient(
        sh.nz, sh.nx, sh.ny, kinds[kZMinus], kinds[kZPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.w[w_index(sh, a, b, q)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, a, b, q); });
    return {solve_residual, n_v};
}

}  // namespace spade::physics::airflow
