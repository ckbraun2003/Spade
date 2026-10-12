#include "physics/airflow/viscous.hpp"

#include <algorithm>
#include <cstdint>

#include "physics/airflow/boundaries.hpp"

namespace spade::physics::airflow {

namespace {

void diffuse_component(std::vector<float>& x, uint32_t c, const GridShape& s, const DomainBc& bc, float alpha,
                       uint32_t sweeps, std::vector<float>& rhs) noexcept {
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const uint32_t cnt[3] = {c == 0u ? s.nx + 1u : s.nx, c == 1u ? s.ny + 1u : s.ny, c == 2u ? s.nz + 1u : s.nz};
    const auto at = [&](uint32_t i, uint32_t j, uint32_t k) {
        return i + std::size_t{cnt[0]} * (j + std::size_t{cnt[1]} * k);
    };
    const bool periodic[3] = {bc.face[kXMinus] == FaceBc::periodic, bc.face[kYMinus] == FaceBc::periodic,
                              bc.face[kZMinus] == FaceBc::periodic};
    const auto cl = static_cast<glm::length_t>(c);
    std::copy(x.begin(), x.end(), rhs.begin());
    // THE INTERIOR FAST PATH (the airflow CPU-performance task, 2026-10-12): a
    // sample whose six neighbours are all plain samples -- along its own axis
    // neither a wrap nor a skipped boundary, across it no domain edge -- takes
    // the same additions in the same order as the general body below, and the
    // diagonal 1 + alpha summed six times in that order. `extra` stays a
    // literal + 0.0f, which turns a -0 sum into +0 exactly as the general body
    // does. Only samples on the edges pay for the face kinds.
    float diag_interior = 1.0f;
    for (int n6 = 0; n6 < 6; ++n6) diag_interior += alpha;
    // The interior per axis, inclusive, from 1: along the own axis to n - 1
    // (the samples 0 and n are boundaries) or, periodic, to n - 2 (0 and n - 1
    // wrap); across it to n - 2 (0 and n - 1 are domain edges).
    int64_t hi[3];
    for (uint32_t a = 0; a < 3; ++a) {
        hi[a] = (a == c && !periodic[c]) ? int64_t{n[a]} - 1 : int64_t{n[a]} - 2;
    }
    const auto in_range = [&](uint32_t a, uint32_t q) { return q >= 1u && int64_t{q} <= hi[a]; };
    const std::size_t sy = cnt[0];
    const std::size_t sz = std::size_t{cnt[0]} * cnt[1];
    float* xp = x.data();
    const float* rp = rhs.data();
    for (uint32_t sweep = 0; sweep < sweeps; ++sweep) {
        for (uint32_t colour = 0; colour < 2; ++colour) {
            for (uint32_t k = 0; k < cnt[2]; ++k) {
                for (uint32_t j = 0; j < cnt[1]; ++j) {
                    const bool row_inside = in_range(1, j) && in_range(2, k);
                    for (uint32_t i = (j + k + colour) & 1u; i < cnt[0]; i += 2u) {
                        if (row_inside && in_range(0, i)) {
                            const std::size_t self = at(i, j, k);
                            float sum = 0.0f;
                            sum += alpha * xp[self - 1u];
                            sum += alpha * xp[self + 1u];
                            sum += alpha * xp[self - sy];
                            sum += alpha * xp[self + sy];
                            sum += alpha * xp[self - sz];
                            sum += alpha * xp[self + sz];
                            xp[self] = ((rp[self] + sum) + 0.0f) / diag_interior;
                            continue;
                        }
                        const uint32_t idx[3] = {i, j, k};
                        const uint32_t own = idx[c];
                        // The periodic alias face, or a boundary face the velocity conditions hold.
                        if (periodic[c] ? own == n[c] : (own == 0u || own == n[c])) continue;
                        float sum = 0.0f;
                        float diag = 1.0f;
                        float extra = 0.0f;
                        for (uint32_t a = 0; a < 3; ++a) {
                            for (uint32_t dir = 0; dir < 2; ++dir) {
                                uint32_t nb[3] = {i, j, k};
                                if (a == c) {
                                    if (periodic[c] && n[c] == 1u) continue;  // its own neighbour
                                    if (dir == 0) {
                                        nb[a] = own == 0u ? n[c] - 1u : own - 1u;
                                    } else {
                                        nb[a] = (periodic[c] && own + 1u == n[c]) ? 0u : own + 1u;
                                    }
                                    sum += alpha * x[at(nb[0], nb[1], nb[2])];
                                    diag += alpha;
                                    continue;
                                }
                                const bool edge = dir == 0 ? idx[a] == 0u : idx[a] + 1u == n[a];
                                if (!edge) {
                                    nb[a] = dir == 0 ? idx[a] - 1u : idx[a] + 1u;
                                    sum += alpha * x[at(nb[0], nb[1], nb[2])];
                                    diag += alpha;
                                    continue;
                                }
                                const auto side = static_cast<Side>(2u * a + dir);
                                switch (bc.face[side]) {
                                    case FaceBc::periodic:
                                        if (n[a] > 1u) {
                                            nb[a] = dir == 0 ? n[a] - 1u : 0u;
                                            sum += alpha * x[at(nb[0], nb[1], nb[2])];
                                            diag += alpha;
                                        }
                                        break;
                                    case FaceBc::wall:  // no slip: the ghost 2 U - x
                                        diag += 2.0f * alpha;
                                        extra += 2.0f * alpha * bc.wall_velocity[side][cl];
                                        break;
                                    case FaceBc::open:
                                        if (face_kind(bc, side) == FaceKind::fixed_velocity) {  // an inflow
                                            diag += 2.0f * alpha;
                                            extra += 2.0f * alpha * bc.ambient[cl];
                                        }  // an outlet: the zero-gradient ghost, no term
                                        break;
                                }
                            }
                        }
                        const std::size_t self = at(i, j, k);
                        x[self] = ((rhs[self] + sum) + extra) / diag;
                    }
                }
            }
        }
    }
    if (periodic[c]) {  // face n is face 0
        for (uint32_t k = 0; k < cnt[2]; ++k) {
            for (uint32_t j = 0; j < cnt[1]; ++j) {
                for (uint32_t i = 0; i < cnt[0]; ++i) {
                    const uint32_t idx[3] = {i, j, k};
                    if (idx[c] != n[c]) continue;
                    uint32_t first[3] = {i, j, k};
                    first[c] = 0u;
                    x[at(i, j, k)] = x[at(first[0], first[1], first[2])];
                }
            }
        }
    }
}

}  // namespace

void diffuse(MacGrid& g, const DomainBc& bc, float nu, float h, uint32_t sweeps, std::vector<float>& rhs) noexcept {
    if (nu == 0.0f) return;
    const float alpha = nu * h / (g.shape.dx * g.shape.dx);
    diffuse_component(g.u, 0, g.shape, bc, alpha, sweeps, rhs);
    diffuse_component(g.v, 1, g.shape, bc, alpha, sweeps, rhs);
    diffuse_component(g.w, 2, g.shape, bc, alpha, sweeps, rhs);
}

}  // namespace spade::physics::airflow
