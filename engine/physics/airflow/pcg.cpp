#include "physics/airflow/pcg.hpp"

#include <algorithm>
#include <cmath>

#include "physics/airflow/reduce.hpp"

namespace spade::physics::airflow {

PcgScratch make_pcg_scratch(std::size_t cells) {
    PcgScratch s;
    s.x.assign(cells, 0.0f);
    s.b.assign(cells, 0.0f);
    s.r.assign(cells, 0.0f);
    s.z.assign(cells, 0.0f);
    s.d.assign(cells, 0.0f);
    s.q.assign(cells, 0.0f);
    return s;
}

float solve_pcg(Multigrid& mg, PcgScratch& s, uint32_t iterations, const PcgGauge& g) noexcept {
    PoissonLevel& L = mg.levels[0];
    const std::size_t n = s.x.size();
    const auto mask = [&](std::vector<float>& v) {
        if (g.solid == nullptr) return;
        for (std::size_t i = 0; i < n; ++i) {
            if (g.solid[i] != 0u) v[i] = 0.0f;
        }
    };
    const auto gauge = [&](std::vector<float>& v) {
        if (g.remove != nullptr) g.remove(g.ctx, v);
    };
    const auto dot = [&](const std::vector<float>& a, const std::vector<float>& b) {
        return fixed_tile_dot(a, b, L.nx, L.ny, L.nz);
    };
    const auto precond = [&]() {
        std::copy(s.r.begin(), s.r.end(), L.b.begin());
        precondition(mg);
        std::copy(L.p.begin(), L.p.end(), s.z.begin());
        mask(s.z);
        gauge(s.z);
    };
    mask(s.x);
    apply_operator(L, s.x, s.q);
    for (std::size_t i = 0; i < n; ++i) s.r[i] = s.b[i] - s.q[i];
    mask(s.r);
    gauge(s.r);
    precond();
    std::copy(s.z.begin(), s.z.end(), s.d.begin());
    float rz = dot(s.r, s.z);
    for (uint32_t it = 0; it < iterations; ++it) {
        apply_operator(L, s.d, s.q);
        const float dq = dot(s.d, s.q);
        if (dq == 0.0f || rz == 0.0f) break;  // solved exactly: nothing left to reduce
        const float alpha = rz / dq;
        for (std::size_t i = 0; i < n; ++i) {
            s.x[i] += alpha * s.d[i];
            s.r[i] -= alpha * s.q[i];
        }
        gauge(s.r);
        precond();
        const float rz_next = dot(s.r, s.z);
        const float beta = rz_next / rz;
        for (std::size_t i = 0; i < n; ++i) s.d[i] = s.z[i] + beta * s.d[i];
        rz = rz_next;
    }
    gauge(s.x);
    apply_operator(L, s.x, s.q);
    float r_max = 0.0f;
    float b_max = 0.0f;
    for (std::size_t i = 0; i < n; ++i) {
        if (g.solid != nullptr && g.solid[i] != 0u) continue;
        r_max = std::max(r_max, std::fabs(s.b[i] - s.q[i]));
        b_max = std::max(b_max, std::fabs(s.b[i]));
    }
    return b_max > 0.0f ? r_max / b_max : 0.0f;
}

}  // namespace spade::physics::airflow
