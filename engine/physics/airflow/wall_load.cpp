#include "physics/airflow/wall_load.hpp"

#include <cstddef>

#include "core/detail/flush_subnormals.hpp"
#include "physics/airflow/reduce.hpp"

namespace spade::physics::airflow {

namespace {

// wall_load()'s body, run under its wrapper's scope (the wrapper rule,
// core/detail/flush_subnormals.hpp).
SPADE_NOINLINE WallLoad wall_load_flushed(const MacGrid& g, const SolidField& f, const FluidRegions& r,
                                          glm::vec3 about) noexcept {
    const GridShape& s = g.shape;
    if (!f.any || r.solid.size() != g.p.size()) return {};
    const float area = s.dx * s.dx;
    // -p_c A_c for cell (i, j, k), or 0 where the cell has no open face.
    const auto push = [&](uint32_t i, uint32_t j, uint32_t k, std::size_t c) {
        if (r.solid[c] != 0u) return glm::vec3(0.0f);
        const glm::vec3 a(area * (f.theta[0][u_index(s, i + 1u, j, k)] - f.theta[0][u_index(s, i, j, k)]),
                          area * (f.theta[1][v_index(s, i, j + 1u, k)] - f.theta[1][v_index(s, i, j, k)]),
                          area * (f.theta[2][w_index(s, i, j, k + 1u)] - f.theta[2][w_index(s, i, j, k)]));
        return -g.p[c] * a;
    };
    WallLoad out;
    out.force = fixed_tile_sum_of<glm::vec3>(s.nx, s.ny, s.nz, push);
    out.moment = fixed_tile_sum_of<glm::vec3>(
        s.nx, s.ny, s.nz, [&](uint32_t i, uint32_t j, uint32_t k, std::size_t c) {
            const glm::vec3 x((static_cast<float>(i) + 0.5f) * s.dx, (static_cast<float>(j) + 0.5f) * s.dx,
                              (static_cast<float>(k) + 0.5f) * s.dx);
            return glm::cross(x - about, push(i, j, k, c));
        });
    return out;
}

}  // namespace

// Subnormals flushed as fluid_step() flushes them (spec section 4.5 as amended
// 2026-10-22; the GPU's twin is B5's wall_load).
WallLoad wall_load(const MacGrid& g, const SolidField& f, const FluidRegions& r, glm::vec3 about) noexcept {
    const spade::math::ScopedFlushSubnormals flush;
    return wall_load_flushed(g, f, r, about);
}

}  // namespace spade::physics::airflow
