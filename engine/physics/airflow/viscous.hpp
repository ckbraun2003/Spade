#pragma once

#include <cstdint>
#include <vector>

#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/solids.hpp"

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

}  // namespace spade::physics::airflow
