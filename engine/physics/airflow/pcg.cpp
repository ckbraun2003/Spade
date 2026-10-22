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

PcgResult solve_pcg_to(Multigrid& mg, PcgScratch& s, uint32_t floor, uint32_t cap, float mark,
                       const PcgGauge& g) noexcept {
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
    // The final relative residual, over the cells not marked solid.
    const auto measure = [&]() {
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
    };
    // r = b - A x from x itself, and the first search direction from it: the
    // start, and the stall rule's one restart (pcg.hpp). Returns r . z.
    const auto start = [&]() {
        apply_operator(L, s.x, s.q);
        for (std::size_t i = 0; i < n; ++i) s.r[i] = s.b[i] - s.q[i];
        mask(s.r);
        gauge(s.r);
        precond();
        std::copy(s.z.begin(), s.z.end(), s.d.begin());
        return dot(s.r, s.z);
    };
    mask(s.x);
    float rz = start();
    const uint32_t last = std::max(floor, cap);
    PcgResult out;
    bool measured = false;
    bool exact = false;      // a step's denominator was exactly zero
    bool restarted = false;  // the stall rule's one restart is spent
    float least = 0.0f;      // the stall rule's least residual measured (pcg.hpp)
    uint32_t since = 0;      // measurements since the residual last fell below kStallGain x least
    for (;; ++out.iterations) {
        if (out.iterations >= floor) {  // the floor's done: stop at the mark, the cap or a stall
            out.residual = measure();
            measured = true;
            if (out.residual <= mark) break;
            if (out.iterations == floor || out.residual < kStallGain * least) {
                least = out.residual;
                since = 0;
            } else {
                ++since;
            }
            if (out.iterations >= last) break;  // the cap: announced as capped, not stalled
            if (since >= kStallIterations) {
                if (restarted) {
                    out.stalled = true;
                    break;
                }
                restarted = true;  // the first stall: start again once from the true residual
                since = 0;
                rz = start();
            }
        }
        apply_operator(L, s.d, s.q);
        const float dq = dot(s.d, s.q);
        if (dq == 0.0f || rz == 0.0f) {  // nothing left to reduce: solved, or (above the mark) stalled
            exact = true;
            break;
        }
        measured = false;
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
    if (!measured) out.residual = measure();
    if (exact && out.residual > mark) out.stalled = true;
    return out;
}

float solve_pcg(Multigrid& mg, PcgScratch& s, uint32_t iterations, const PcgGauge& gauge) noexcept {
    return solve_pcg_to(mg, s, iterations, iterations, 0.0f, gauge).residual;
}

}  // namespace spade::physics::airflow
