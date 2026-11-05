#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/solids.hpp"
#include "physics/airflow/worker_pool.hpp"

// ---------------------------------------------------------------------------
// VISCOSITY (spec §11.3, item 1): implicit (backward Euler) diffusion of every
// velocity component, (1 + a sum) x - a sum x_nb = u*, a = nu h / dx^2, by a
// fixed number of red-black Gauss-Seidel sweeps from u*. A wall gives its
// tangential neighbour the no-slip ghost 2 U - x; an inflow the same with the
// ambient; an outlet the zero-gradient ghost; a periodic face its wrap.
// nu = 0 returns at once. `rhs` holds at least the largest component's size.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

// WITH SOLIDS (the solids design, section 3): every sample with theta = 0 or
// phi < 0 holds its wall velocity and is not solved; a solved sample's link
// that crosses a wall or a sheet at the fraction f of its length takes
// (alpha / f) (u_wall - x) in place of alpha (x_nb - x) (solids.hpp's links;
// the symmetric ghost-fluid Dirichlet condition of Gibou, Fedkiw, Cheng and
// Kang 2002, on phi = 0, where the apertures put theta = 1/2). Without:
// plan A, bit for bit.
void diffuse(MacGrid& g, const DomainBc& bc, float nu, float h, uint32_t sweeps, std::vector<float>& rhs,
             const SolidField* solids = nullptr) noexcept;

// The same solve on a pool's threads (L18): u, v and w in lockstep, each with
// its own right side (u's and v's in `rhs_uv`, w's in `rhs`), each colour pass
// split by k plane, a barrier per colour (no sample reads one of its own
// colour: make_grid() takes only even periodic axes). The bits are the
// serial solve's at any thread count; a null pool, or one thread, is the
// serial solve.
void diffuse(MacGrid& g, const DomainBc& bc, float nu, float h, uint32_t sweeps, std::vector<float>& rhs,
             std::array<std::vector<float>, 2>& rhs_uv, const SolidField* solids, WorkerPool* pool) noexcept;

}  // namespace spade::physics::airflow
