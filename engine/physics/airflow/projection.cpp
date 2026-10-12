#include "physics/airflow/projection.hpp"

#include <algorithm>

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

}  // namespace

ProjectionScratch make_projection_scratch(const GridShape& shape, const DomainBc& bc) {
    ProjectionScratch s;
    s.mg = make_multigrid(shape.nx, shape.ny, shape.nz, periodic_axes(bc));
    s.div.assign(std::size_t{shape.nx} * shape.ny * shape.nz, 0.0f);
    return s;
}

void project(MacGrid& g, const DomainBc& bc, float rho, float h, uint32_t n_v, ProjectionScratch& s) noexcept {
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
}

}  // namespace spade::physics::airflow
