#include "physics/airflow/viscous.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

#include "physics/airflow/boundaries.hpp"

namespace spade::physics::airflow {

namespace {

// One component's solve, split so the CPU step's threads can take its planes
// (L18): prepare(), then per sweep and colour relax_plane() on every k plane
// in any order on any thread, then finish(). A sample of one colour reads
// only samples of the other: its six neighbours differ by one in one index,
// and a wrap joins 0 and n - 1 of an even n (make_grid() refuses an odd
// periodic axis; one of size 1 wraps onto nothing).
// The arithmetic is the serial solve's, sample for sample.
class ComponentSolve {
public:
    ComponentSolve(std::vector<float>& x, uint32_t c, const GridShape& s, const DomainBc& bc, float alpha,
                   std::vector<float>& rhs, const SolidField* solids) noexcept
        : x_(&x), c_(c), bc_(&bc), alpha_(alpha), rhs_(&rhs), solids_(solids) {
        n_[0] = s.nx;
        n_[1] = s.ny;
        n_[2] = s.nz;
        cnt_[0] = c == 0u ? s.nx + 1u : s.nx;
        cnt_[1] = c == 1u ? s.ny + 1u : s.ny;
        cnt_[2] = c == 2u ? s.nz + 1u : s.nz;
        periodic_[0] = bc.face[kXMinus] == FaceBc::periodic;
        periodic_[1] = bc.face[kYMinus] == FaceBc::periodic;
        periodic_[2] = bc.face[kZMinus] == FaceBc::periodic;
        walls_ = solids != nullptr && solids->any;
        if (walls_) {
            theta_ = &solids->theta[c];
            phi_ = &solids->phi[c];
            wall_v_ = &solids->wall[c];
            link_ = &solids->link[c];
        }
        // THE INTERIOR FAST PATH (the airflow CPU-performance task, 2026-10-12): a
        // sample whose six neighbours are all plain samples -- along its own axis
        // neither a wrap nor a skipped boundary, across it no domain edge -- takes
        // the same additions in the same order as the general body below, and the
        // diagonal 1 + alpha summed six times in that order. `extra` stays a
        // literal + 0.0f, which turns a -0 sum into +0 exactly as the general body
        // does. Only samples on the edges pay for the face kinds.
        diag_interior_ = 1.0f;
        for (int n6 = 0; n6 < 6; ++n6) diag_interior_ += alpha;
        // The interior per axis, inclusive, from 1: along the own axis to n - 1
        // (the samples 0 and n are boundaries) or, periodic, to n - 2 (0 and n - 1
        // wrap); across it to n - 2 (0 and n - 1 are domain edges).
        for (uint32_t a = 0; a < 3; ++a) {
            hi_[a] = (a == c && !periodic_[c]) ? int64_t{n_[a]} - 1 : int64_t{n_[a]} - 2;
        }
        sy_ = cnt_[0];
        sz_ = std::size_t{cnt_[0]} * cnt_[1];
    }

    [[nodiscard]] uint32_t planes() const noexcept { return cnt_[2]; }

    void prepare() noexcept {
        std::vector<float>& x = *x_;
        if (walls_) {
            for (std::size_t idx = 0; idx < x.size(); ++idx) {
                if (fixed(idx)) x[idx] = (*wall_v_)[idx];
            }
        }
        std::copy(x.begin(), x.end(), rhs_->begin());
    }

    void relax_plane(uint32_t k, uint32_t colour) noexcept {
        std::vector<float>& x = *x_;
        const std::vector<float>& rhs = *rhs_;
        const DomainBc& bc = *bc_;
        const uint32_t c = c_;
        const auto cl = static_cast<glm::length_t>(c);
        const float alpha = alpha_;
        float* xp = x.data();
        const float* rp = rhs.data();
        for (uint32_t j = 0; j < cnt_[1]; ++j) {
            const bool row_inside = in_range(1, j) && in_range(2, k);
            for (uint32_t i = (j + k + colour) & 1u; i < cnt_[0]; i += 2u) {
                const std::size_t here = at(i, j, k);
                const WallLinks* wl = nullptr;
                if (walls_) {
                    if (fixed(here)) continue;
                    if ((*link_)[here] != kNoWallLinks) wl = &solids_->links[(*link_)[here]];
                }
                if (wl == nullptr && row_inside && in_range(0, i)) {
                    const std::size_t self = at(i, j, k);
                    float sum = 0.0f;
                    sum += alpha * xp[self - 1u];
                    sum += alpha * xp[self + 1u];
                    sum += alpha * xp[self - sy_];
                    sum += alpha * xp[self + sy_];
                    sum += alpha * xp[self - sz_];
                    sum += alpha * xp[self + sz_];
                    xp[self] = ((rp[self] + sum) + 0.0f) / diag_interior_;
                    continue;
                }
                const uint32_t idx[3] = {i, j, k};
                const uint32_t own = idx[c];
                // The periodic alias face, or a boundary face the velocity conditions hold.
                if (periodic_[c] ? own == n_[c] : (own == 0u || own == n_[c])) continue;
                float sum = 0.0f;
                float diag = 1.0f;
                float extra = 0.0f;
                for (uint32_t a = 0; a < 3; ++a) {
                    for (uint32_t dir = 0; dir < 2; ++dir) {
                        if (wl != nullptr && wl->inv_f[2u * a + dir] > 0.0f) {
                            const float w = alpha * wl->inv_f[2u * a + dir];
                            diag += w;
                            extra += w * wl->wall[2u * a + dir];
                            continue;
                        }
                        uint32_t nb[3] = {i, j, k};
                        if (a == c) {
                            if (periodic_[c] && n_[c] == 1u) continue;  // its own neighbour
                            if (dir == 0) {
                                nb[a] = own == 0u ? n_[c] - 1u : own - 1u;
                            } else {
                                nb[a] = (periodic_[c] && own + 1u == n_[c]) ? 0u : own + 1u;
                            }
                            sum += alpha * x[at(nb[0], nb[1], nb[2])];
                            diag += alpha;
                            continue;
                        }
                        const bool edge = dir == 0 ? idx[a] == 0u : idx[a] + 1u == n_[a];
                        if (!edge) {
                            nb[a] = dir == 0 ? idx[a] - 1u : idx[a] + 1u;
                            sum += alpha * x[at(nb[0], nb[1], nb[2])];
                            diag += alpha;
                            continue;
                        }
                        const auto side = static_cast<Side>(2u * a + dir);
                        switch (bc.face[side]) {
                            case FaceBc::periodic:
                                if (n_[a] > 1u) {
                                    nb[a] = dir == 0 ? n_[a] - 1u : 0u;
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

    void finish() noexcept {
        std::vector<float>& x = *x_;
        const uint32_t c = c_;
        if (!periodic_[c]) return;  // face n is face 0
        for (uint32_t k = 0; k < cnt_[2]; ++k) {
            for (uint32_t j = 0; j < cnt_[1]; ++j) {
                for (uint32_t i = 0; i < cnt_[0]; ++i) {
                    const uint32_t idx[3] = {i, j, k};
                    if (idx[c] != n_[c]) continue;
                    uint32_t first[3] = {i, j, k};
                    first[c] = 0u;
                    x[at(i, j, k)] = x[at(first[0], first[1], first[2])];
                }
            }
        }
    }

private:
    [[nodiscard]] std::size_t at(uint32_t i, uint32_t j, uint32_t k) const noexcept {
        return i + std::size_t{cnt_[0]} * (j + std::size_t{cnt_[1]} * k);
    }
    [[nodiscard]] bool fixed(std::size_t idx) const noexcept {
        return closed((*theta_)[idx]) || (*phi_)[idx] < 0.0f;
    }
    [[nodiscard]] bool in_range(uint32_t a, uint32_t q) const noexcept { return q >= 1u && int64_t{q} <= hi_[a]; }

    std::vector<float>* x_;
    uint32_t c_;
    const DomainBc* bc_;
    float alpha_;
    std::vector<float>* rhs_;
    const SolidField* solids_;
    uint32_t n_[3]{};
    uint32_t cnt_[3]{};
    bool periodic_[3]{};
    bool walls_ = false;
    const std::vector<float>* theta_ = nullptr;
    const std::vector<float>* phi_ = nullptr;
    const std::vector<float>* wall_v_ = nullptr;
    const std::vector<uint32_t>* link_ = nullptr;
    float diag_interior_ = 1.0f;
    int64_t hi_[3]{};
    std::size_t sy_ = 0;
    std::size_t sz_ = 0;
};

void solve_serial(ComponentSolve& cs, uint32_t sweeps) noexcept {
    cs.prepare();
    for (uint32_t sweep = 0; sweep < sweeps; ++sweep) {
        for (uint32_t colour = 0; colour < 2; ++colour) {
            for (uint32_t k = 0; k < cs.planes(); ++k) cs.relax_plane(k, colour);
        }
    }
    cs.finish();
}

}  // namespace

void diffuse(MacGrid& g, const DomainBc& bc, float nu, float h, uint32_t sweeps, std::vector<float>& rhs,
             const SolidField* solids) noexcept {
    if (nu == 0.0f) return;
    const float alpha = nu * h / (g.shape.dx * g.shape.dx);
    ComponentSolve u(g.u, 0, g.shape, bc, alpha, rhs, solids);
    solve_serial(u, sweeps);
    ComponentSolve v(g.v, 1, g.shape, bc, alpha, rhs, solids);
    solve_serial(v, sweeps);
    ComponentSolve w(g.w, 2, g.shape, bc, alpha, rhs, solids);
    solve_serial(w, sweeps);
}

void diffuse(MacGrid& g, const DomainBc& bc, float nu, float h, uint32_t sweeps, std::vector<float>& rhs,
             std::array<std::vector<float>, 2>& rhs_uv, const SolidField* solids, WorkerPool* pool) noexcept {
    if (pool == nullptr || pool->threads() == 1u) {
        diffuse(g, bc, nu, h, sweeps, rhs, solids);
        return;
    }
    if (nu == 0.0f) return;
    const float alpha = nu * h / (g.shape.dx * g.shape.dx);
    // Each component its own right side: u's and v's here, w's in `rhs`, which
    // so ends the step holding w's copy, as the serial solve leaves it.
    std::array<ComponentSolve, 3> cs = {ComponentSolve(g.u, 0, g.shape, bc, alpha, rhs_uv[0], solids),
                                        ComponentSolve(g.v, 1, g.shape, bc, alpha, rhs_uv[1], solids),
                                        ComponentSolve(g.w, 2, g.shape, bc, alpha, rhs, solids)};
    pool->run(3u, [&](uint32_t c) { cs[c].prepare(); });
    // The items of one colour pass: each component's planes, one an item.
    const uint32_t per[3] = {cs[0].planes(), cs[1].planes(), cs[2].planes()};
    const uint32_t items = per[0] + per[1] + per[2];
    for (uint32_t sweep = 0; sweep < sweeps; ++sweep) {
        for (uint32_t colour = 0; colour < 2; ++colour) {
            pool->run(items, [&](uint32_t item) {
                uint32_t c = 0;
                while (item >= per[c]) item -= per[c++];
                cs[c].relax_plane(item, colour);
            });
        }
    }
    pool->run(3u, [&](uint32_t c) { cs[c].finish(); });
}

}  // namespace spade::physics::airflow
