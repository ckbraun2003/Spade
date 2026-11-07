#pragma once

#include <array>
#include <cstdint>

#include "physics/lbm/lbm_grid.hpp"

// ---------------------------------------------------------------------------
// physics/lbm/lbm_step.hpp -- one LBM step on a fully periodic grid: gather
// each node's 27 incoming populations by the esoteric-pull rule (lbm_grid.hpp),
// collide, scatter. The collision is a callable on the node's 27 shifted
// values, in place: collide(std::array<float, kQ>& f). Boundaries beyond the
// periodic wrap (walls, open faces) are L1's later tasks.
// ---------------------------------------------------------------------------

namespace spade::physics::lbm {

template <class Collide>
void step_periodic(LbmGrid& g, Collide&& collide) {
    const bool odd = g.odd();
    std::array<float, kQ> f{};
    for (uint32_t z = 0; z < g.nz; ++z) {
        for (uint32_t y = 0; y < g.ny; ++y) {
            for (uint32_t x = 0; x < g.nx; ++x) {
                for (uint32_t i = 0; i < kQ; ++i) f[i] = incoming(g, i, x, y, z);
                collide(f);
                if (!odd) {
                    const std::size_t here = g.cell(x, y, z);
                    for (uint32_t i = 0; i < kQ; ++i) g.f[slot_of(g, opposite(i), here)] = f[i];
                } else {
                    for (uint32_t j = 0; j < kQ; ++j) g.f[slot_of(g, j, neighbour_cell(g, x, y, z, kC[j]))] = f[j];
                }
            }
        }
    }
    ++g.step;
}

}  // namespace spade::physics::lbm
