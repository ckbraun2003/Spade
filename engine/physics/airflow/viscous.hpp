#pragma once

#include <cstdint>
#include <vector>

#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// VISCOSITY (spec §11.3, item 1): implicit (backward Euler) diffusion of every
// velocity component, (1 + a sum) x - a sum x_nb = u*, a = nu h / dx^2, by a
// fixed number of red-black Gauss-Seidel sweeps from u*. A wall gives its
// tangential neighbour the no-slip ghost 2 U - x; an inflow the same with the
// ambient; an outlet the zero-gradient ghost; a periodic face its wrap.
// nu = 0 returns at once. `rhs` holds at least the largest component's size.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

void diffuse(MacGrid& g, const DomainBc& bc, float nu, float h, uint32_t sweeps, std::vector<float>& rhs) noexcept;

}  // namespace spade::physics::airflow
