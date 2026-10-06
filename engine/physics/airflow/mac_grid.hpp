#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"

// ---------------------------------------------------------------------------
// THE AIRFLOW SOLVER'S GRID (physics/plans/2026-10-05-airflow-design.md §1.5,
// §2). A MAC (staggered) grid of cubic cells: each velocity component lives on
// the faces normal to its axis, the pressure at the cell centres. A domain is
// [0, nx dx] x [0, ny dx] x [0, nz dx], in metres, in the grid's own frame.
//
// The six domain faces are indexed 2 * axis + (0 for the minus face, 1 for the
// plus face) everywhere in this library.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

enum Side : uint32_t { kXMinus = 0, kXPlus = 1, kYMinus = 2, kYPlus = 3, kZMinus = 4, kZPlus = 5 };

enum class FaceBc : uint8_t {
    open,      // the far field: an inflow where the ambient enters, otherwise a pressure outlet (p = 0)
    wall,      // no slip, at the face's wall velocity (a moving lid is a wall with a tangential velocity)
    periodic,  // wraps to the opposite face; both faces of an axis say so, or neither
};

struct DomainBc {
    std::array<FaceBc, 6> face{FaceBc::open, FaceBc::open, FaceBc::open,
                               FaceBc::open, FaceBc::open, FaceBc::open};
    std::array<glm::vec3, 6> wall_velocity{};  // per face, read where face[s] == wall, m/s
    glm::vec3 ambient{0.0f};                   // the far-field velocity at open faces, m/s
};

struct GridShape {
    uint32_t nx = 0;
    uint32_t ny = 0;
    uint32_t nz = 0;
    float dx = 0.0f;  // the cell size, metres
};

struct MacGrid {
    GridShape shape{};
    std::vector<float> u;  // x-faces, (nx + 1) ny nz, at (i dx, (j + 1/2) dx, (k + 1/2) dx)
    std::vector<float> v;  // y-faces, nx (ny + 1) nz
    std::vector<float> w;  // z-faces, nx ny (nz + 1)
    std::vector<float> p;  // cells, nx ny nz: gauge pressure, Pa
};

[[nodiscard]] inline std::size_t u_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx + 1u} * (j + std::size_t{g.ny} * k);
}
[[nodiscard]] inline std::size_t v_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx} * (j + std::size_t{g.ny + 1u} * k);
}
[[nodiscard]] inline std::size_t w_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx} * (j + std::size_t{g.ny} * k);
}
[[nodiscard]] inline std::size_t cell_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx} * (j + std::size_t{g.ny} * k);
}

// Every sample of velocity component `component` (0 u, 1 v, 2 w), in array
// order (k outermost, i innermost), with its position: on its own axis at
// index * dx, on the other two at (index + 1/2) dx.
template <class F>
void for_each_sample(const GridShape& s, uint32_t component, F&& f) {
    const uint32_t cx = component == 0 ? s.nx + 1u : s.nx;
    const uint32_t cy = component == 1 ? s.ny + 1u : s.ny;
    const uint32_t cz = component == 2 ? s.nz + 1u : s.nz;
    const float ox = component == 0 ? 0.0f : 0.5f;
    const float oy = component == 1 ? 0.0f : 0.5f;
    const float oz = component == 2 ? 0.0f : 0.5f;
    std::size_t idx = 0;
    for (uint32_t k = 0; k < cz; ++k) {
        for (uint32_t j = 0; j < cy; ++j) {
            for (uint32_t i = 0; i < cx; ++i, ++idx) {
                f(idx, glm::vec3((static_cast<float>(i) + ox) * s.dx, (static_cast<float>(j) + oy) * s.dx,
                                 (static_cast<float>(k) + oz) * s.dx));
            }
        }
    }
}

// A grid at rest: every face and cell 0. Refused (invalid_argument, naming
// what): a zero dimension; dx not positive and finite; a periodic face whose
// opposite face is not periodic; a periodic axis of odd size other than 1,
// since red-black colouring needs an even ring. capacity_exceeded past 2^31
// cells.
[[nodiscard]] Result<MacGrid> make_grid(const GridShape& shape, const DomainBc& bc);

}  // namespace spade::physics::airflow
