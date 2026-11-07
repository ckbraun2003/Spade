#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/detail/flush_subnormals.hpp"
#include "physics/airflow/worker_pool.hpp"
#include "physics/lbm/lattice.hpp"
#include "physics/lbm/lbm_collide.hpp"
#include "physics/lbm/lbm_grid.hpp"

// ---------------------------------------------------------------------------
// physics/lbm/lbm_flow.hpp -- the LBM tier's flow step on a fully periodic grid
// with WALE in tau (L1; design sections 1 and 4, decision D2 = W2): two passes a
// step, both per node and reduction-free.
//   1. compute_velocity: u from the populations about to be streamed (the
//      esoteric parity rule, lbm_grid.hpp), into one array u[3][N].
//   2. step: each node gathers its 27 incoming populations, takes the velocity
//      gradient from the six face neighbours' u (central differences, periodic),
//      sets tau = 3 (nu + nu_t) + 1/2 from WALE, collides (RR3) and scatters.
//
// BOX WALLS (C1). An axis that is not periodic has a wall on each face, halfway
// between the first node and the box's edge: a population whose source cell lies
// outside the box bounces (halfway bounce-back), with the moving-wall term
// 6 w_i (c_i . u_w) added to the incoming direction i (shifted storage leaves it
// unchanged: both w's cancel). In the esoteric pull (lbm_grid.hpp):
//   ODD step, reading:  a wall-sourced incoming i is the node's own slot [i][x]
//                       (the last even step put f*_{i-bar} there), plus the term;
//   ODD step, writing:  an outgoing j whose target is outside is written to the
//                       node's own slot [j-bar][x], plus the term of j-bar, where
//                       the next (even) step reads it.
//   EVEN step:          unchanged: it reads [i][x] and writes [i-bar][x].
// A link that crosses two faces (an edge or corner of the box) uses the mean of
// the two faces' velocities; each link bounces on its own, so no special case.
//
// SUBNORMALS. The reference step is meant to run under spade::math::ScopedFlushSubnormals (FTZ|DAZ, round to nearest),
// as the device of record flushes (the GPU twin's plan, "Subnormals"): step_flow_flushed() and flow_sums_flushed() are
// the wrappers (core/detail/flush_subnormals.hpp's wrapper rule: the scope, then one call to a SPADE_NOINLINE body, no
// float work in the wrapper). The WorkerPool runs each item under the caller's word, so the split step is under it too.
// step_flow() and flow_sums() stay for the tests of the arithmetic itself and for the gradual-underflow comparison
// runs.
//
// THREADS. Both passes take an optional WorkerPool (physics/airflow/worker_pool.hpp: fixed partitions, no
// reductions). The grid's rows (y, z) are cut into fixed chunks of kRowChunk, whatever the thread count; each pass is
// one run() and the pool's return is the barrier between them. A node of the first pass writes only its own u and
// reads populations; a node of the second reads u (which no node of that pass writes) and reads and writes exactly the
// slots that only it touches (the esoteric rule, lbm_grid.hpp). So the bits cannot depend on the thread count, the
// order of the chunks or the timing, and the diagnostic sums (flow_sums) stay serial in cell order.
// ---------------------------------------------------------------------------

namespace spade::physics::lbm {

using airflow::WorkerPool;

// Rows (one y, one z, all x) per work item. A constant, so the partition does not depend on the thread count.
inline constexpr uint32_t kRowChunk = 4;

// The box's boundary: per axis, periodic or a wall on each face. wall_u[axis][side] is the velocity of the wall at
// the low (side 0) or high (side 1) face of the axis, used where the axis is not periodic.
struct BoxWalls {
    std::array<bool, 3> periodic{true, true, true};
    std::array<std::array<std::array<float, 3>, 2>, 3> wall_u{};

    [[nodiscard]] bool any() const noexcept { return !(periodic[0] && periodic[1] && periodic[2]); }
};

struct LbmFlow {
    LbmGrid grid;
    BoxWalls walls;
    std::vector<float> u;  // 3 * N, component-major
    float nu = 0.0f;       // the molecular viscosity, lattice units
    float cw = 0.5f;       // WALE's constant; 0 switches the subgrid model off

    [[nodiscard]] float& u_at(int axis, std::size_t cell) { return u[std::size_t(axis) * grid.cells() + cell]; }
    [[nodiscard]] float u_at(int axis, std::size_t cell) const { return u[std::size_t(axis) * grid.cells() + cell]; }
};

[[nodiscard]] inline LbmFlow make_flow(uint32_t nx, uint32_t ny, uint32_t nz, float nu, float cw) {
    LbmFlow fl;
    fl.grid = make_lbm_grid(nx, ny, nz);
    fl.u.assign(std::size_t{3} * fl.grid.cells(), 0.0f);
    fl.nu = nu;
    fl.cw = cw;
    return fl;
}

// Start a node at the equilibrium of (rho, u). Valid before the first step (even parity), where a slot holds the
// population it names.
inline void set_node_equilibrium(LbmFlow& fl, uint32_t x, uint32_t y, uint32_t z, float rho, float ux, float uy,
                                 float uz) {
    Macro m;
    m.rho = rho;
    m.rho_m1 = rho - 1.0f;
    m.ux = ux;
    m.uy = uy;
    m.uz = uz;
    const std::size_t cell = fl.grid.cell(x, y, z);
    for (uint32_t i = 0; i < kQ; ++i) fl.grid.f[slot_of(fl.grid, i, cell)] = equilibrium_shifted(i, m);
}

// Whether the source cell of incoming direction `dir` at the node, x - c, is outside the box through a wall, and the
// mean velocity of the faces it crosses.
struct WallHit {
    bool hit = false;
    std::array<float, 3> u{};
};

[[nodiscard]] inline WallHit wall_hit(const BoxWalls& w, const LbmGrid& g, uint32_t dir, uint32_t x, uint32_t y,
                                      uint32_t z) noexcept {
    WallHit h;
    if (!w.any()) return h;
    const Vel c = kC[dir];
    const int pos[3] = {static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)};
    const int cc[3] = {c.x, c.y, c.z};
    const int n[3] = {static_cast<int>(g.nx), static_cast<int>(g.ny), static_cast<int>(g.nz)};
    int crossed = 0;
    for (int a = 0; a < 3; ++a) {
        if (w.periodic[std::size_t(a)]) continue;
        const int src = pos[a] - cc[a];
        if (src >= 0 && src < n[a]) continue;
        const std::size_t side = src < 0 ? 0u : 1u;
        for (int k = 0; k < 3; ++k) h.u[std::size_t(k)] += w.wall_u[std::size_t(a)][side][std::size_t(k)];
        ++crossed;
    }
    if (crossed == 0) return h;
    h.hit = true;
    const float inv = 1.0f / static_cast<float>(crossed);
    for (float& v : h.u) v *= inv;
    return h;
}

// 6 w_i (c_i . u_w): the moving wall's addition to incoming direction i.
[[nodiscard]] inline float wall_term(uint32_t dir, const std::array<float, 3>& uw) noexcept {
    return 6.0f * kW[dir] *
           (static_cast<float>(kC[dir].x) * uw[0] + static_cast<float>(kC[dir].y) * uw[1] +
            static_cast<float>(kC[dir].z) * uw[2]);
}

// The logical incoming population of `dir` at the node, with the walls applied (see the header).
[[nodiscard]] inline float incoming_walled(const LbmFlow& fl, uint32_t dir, uint32_t x, uint32_t y, uint32_t z) {
    const LbmGrid& g = fl.grid;
    const WallHit h = wall_hit(fl.walls, g, dir, x, y, z);
    if (!h.hit || !g.odd()) return incoming(g, dir, x, y, z);  // even: the last odd step put it in the slot
    return g.f[slot_of(g, dir, g.cell(x, y, z))] + wall_term(dir, h.u);
}

// Calls body(r) for every row r = y + ny z, in row order; with a pool, in fixed chunks of kRowChunk rows, one run().
template <class Body>
inline void for_rows(const LbmGrid& g, WorkerPool* pool, Body&& body) {
    const uint32_t rows = g.ny * g.nz;
    if (pool == nullptr) {
        for (uint32_t r = 0; r < rows; ++r) body(r);
        return;
    }
    const uint32_t items = (rows + kRowChunk - 1u) / kRowChunk;
    pool->run(items, [&](uint32_t item) {
        const uint32_t end = (item + 1u) * kRowChunk < rows ? (item + 1u) * kRowChunk : rows;
        for (uint32_t r = item * kRowChunk; r < end; ++r) body(r);
    });
}

inline void velocity_row(LbmFlow& fl, uint32_t r) {
    const LbmGrid& g = fl.grid;
    const uint32_t y = r % g.ny;
    const uint32_t z = r / g.ny;
    Pops f{};
    for (uint32_t x = 0; x < g.nx; ++x) {
        for (uint32_t i = 0; i < kQ; ++i) f[i] = incoming_walled(fl, i, x, y, z);
        const Macro m = macro_of(f);
        const std::size_t cell = g.cell(x, y, z);
        fl.u_at(0, cell) = m.ux;
        fl.u_at(1, cell) = m.uy;
        fl.u_at(2, cell) = m.uz;
    }
}

inline void compute_velocity(LbmFlow& fl, WorkerPool* pool = nullptr) {
    for_rows(fl.grid, pool, [&](uint32_t r) { velocity_row(fl, r); });
}

// g[3 i + j] = du_i / dx_j by central differences of u over the periodic neighbours.
[[nodiscard]] inline std::array<float, 9> velocity_gradient(const LbmFlow& fl, uint32_t x, uint32_t y, uint32_t z) {
    const LbmGrid& g = fl.grid;
    std::array<float, 9> gr{};
    for (int j = 0; j < 3; ++j) {
        const Vel plus{static_cast<int8_t>(j == 0), static_cast<int8_t>(j == 1), static_cast<int8_t>(j == 2)};
        const Vel minus{static_cast<int8_t>(-plus.x), static_cast<int8_t>(-plus.y), static_cast<int8_t>(-plus.z)};
        const std::size_t here = g.cell(x, y, z);
        const std::size_t cp = neighbour_cell(g, x, y, z, plus);
        const std::size_t cm = neighbour_cell(g, x, y, z, minus);
        const uint32_t coord[3] = {x, y, z};
        const uint32_t extent[3] = {g.nx, g.ny, g.nz};
        const bool wall_lo = !fl.walls.periodic[std::size_t(j)] && coord[j] == 0u;
        const bool wall_hi = !fl.walls.periodic[std::size_t(j)] && coord[j] + 1u == extent[j];
        for (int i = 0; i < 3; ++i) {
            // Past a wall the neighbour is the ghost 2 u_wall - u_node (the wall is halfway).
            const float u_here = fl.u_at(i, here);
            const float up = wall_hi ? 2.0f * fl.walls.wall_u[std::size_t(j)][1][std::size_t(i)] - u_here
                                     : fl.u_at(i, cp);
            const float um = wall_lo ? 2.0f * fl.walls.wall_u[std::size_t(j)][0][std::size_t(i)] - u_here
                                     : fl.u_at(i, cm);
            gr[std::size_t(3 * i + j)] = 0.5f * (up - um);
        }
    }
    return gr;
}

// Collide and stream one row; u must be current (compute_velocity) and no node of the pass writes it.
inline void collide_row(LbmFlow& fl, uint32_t r, bool odd) {
    LbmGrid& g = fl.grid;
    const uint32_t y = r % g.ny;
    const uint32_t z = r / g.ny;
    Pops f{};
    for (uint32_t x = 0; x < g.nx; ++x) {
        for (uint32_t i = 0; i < kQ; ++i) f[i] = incoming_walled(fl, i, x, y, z);
        const float nu_t = fl.cw > 0.0f ? wale_nu_t(velocity_gradient(fl, x, y, z), fl.cw) : 0.0f;
        collide_rr3(f, omega_of(fl.nu, nu_t));
        if (!odd) {
            const std::size_t here = g.cell(x, y, z);
            for (uint32_t i = 0; i < kQ; ++i) g.f[slot_of(g, opposite(i), here)] = f[i];
        } else {
            for (uint32_t j = 0; j < kQ; ++j) {
                // The target x + c_j is outside through a wall exactly when incoming j-bar's source is.
                const WallHit h = wall_hit(fl.walls, g, opposite(j), x, y, z);
                if (h.hit) {
                    g.f[slot_of(g, opposite(j), g.cell(x, y, z))] = f[j] + wall_term(opposite(j), h.u);
                } else {
                    g.f[slot_of(g, j, neighbour_cell(g, x, y, z, kC[j]))] = f[j];
                }
            }
        }
    }
}

inline void step_flow(LbmFlow& fl, WorkerPool* pool = nullptr) {
    compute_velocity(fl, pool);  // the pool's return is the barrier: every u is written before any node collides
    const bool odd = fl.grid.odd();
    for_rows(fl.grid, pool, [&](uint32_t r) { collide_row(fl, r, odd); });
    ++fl.grid.step;
}

// Diagnostics, summed in double in cell order: K = mean of |u|^2 / 2, and the mean density and momentum.
struct FlowSums {
    double kinetic = 0.0;
    double mass_m1 = 0.0;  // mean of rho - 1
    double momentum[3] = {0.0, 0.0, 0.0};
    bool finite = true;
};

[[nodiscard]] inline FlowSums flow_sums(LbmFlow& fl, WorkerPool* pool = nullptr) {
    compute_velocity(fl, pool);
    const LbmGrid& g = fl.grid;
    FlowSums s;
    Pops f{};
    for (uint32_t z = 0; z < g.nz; ++z) {
        for (uint32_t y = 0; y < g.ny; ++y) {
            for (uint32_t x = 0; x < g.nx; ++x) {
                for (uint32_t i = 0; i < kQ; ++i) f[i] = incoming_walled(fl, i, x, y, z);
                const Macro m = macro_of(f);
                const double ux = m.ux;
                const double uy = m.uy;
                const double uz = m.uz;
                s.kinetic += 0.5 * (ux * ux + uy * uy + uz * uz);
                s.mass_m1 += static_cast<double>(m.rho_m1);
                s.momentum[0] += static_cast<double>(m.rho) * ux;
                s.momentum[1] += static_cast<double>(m.rho) * uy;
                s.momentum[2] += static_cast<double>(m.rho) * uz;
                if (!(ux * ux + uy * uy + uz * uz < 1.0e30)) s.finite = false;  // NaN and infinity fail the compare
            }
        }
    }
    const double n = static_cast<double>(g.cells());
    s.kinetic /= n;
    s.mass_m1 /= n;
    for (double& p : s.momentum) p /= n;
    return s;
}

// The wrappers: the scope, then one call to a body that holds every float op of the step.
SPADE_NOINLINE inline void step_flow_body(LbmFlow& fl, WorkerPool* pool) { step_flow(fl, pool); }
SPADE_NOINLINE inline FlowSums flow_sums_body(LbmFlow& fl, WorkerPool* pool) { return flow_sums(fl, pool); }

inline void step_flow_flushed(LbmFlow& fl, WorkerPool* pool = nullptr) {
    const spade::math::ScopedFlushSubnormals scope;
    step_flow_body(fl, pool);
}

[[nodiscard]] inline FlowSums flow_sums_flushed(LbmFlow& fl, WorkerPool* pool = nullptr) {
    const spade::math::ScopedFlushSubnormals scope;
    return flow_sums_body(fl, pool);
}

}  // namespace spade::physics::lbm
