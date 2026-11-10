#pragma once

#include <cstdint>
#include <vector>

#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/solids.hpp"
#include "physics/airflow/worker_pool.hpp"

// ---------------------------------------------------------------------------
// THE SUBGRID EDDY VISCOSITY (L52, Cameron 2026-11-10; the wake probes of L48).
// The air's step has no molecular viscosity worth the name at the grid (nu =
// 1.5e-5 m^2/s reaches 1.4 um; a cell is 4 to 10 mm) and the advection's clamp
// only damps what it must, so a wake's energy piles up at the grid scale and grows
// with time (the probes: the kinetic energy above k = pi / (2 dx) tripled between
// 0.15 s and 0.6 s at 1 cm, 5 mm and 4 mm alike). Smagorinsky's model adds
//
//     du_i/dt = d_j (nu_t d_j u_i),   nu_t = (Cs dx)^2 |S|,   |S| = sqrt(2 S_ij S_ij),
//
// explicitly, from the field the step holds before it (two phases: every nu_t,
// then every face's change from the old field and the nu_t, then one add; no
// face reads another's result, so the bits are the same at any thread count).
// Cs = FluidConfig::sgs_cs; 0 is off and the step is today's, bit for bit.
//
// THE WALL RULE (the lead, 2026-11-10: the molecular diffusion's own face
// handling with nu replaced by nu + nu_t). viscous.cpp's links, term for term, with
// the viscosity of each link nu_t:
//   * a sample the solids hold (theta = 0 or phi < 0) is not changed;
//   * a link a wall crosses at the fraction f takes nu_t (1 / f) (u_wall - x)
//     in place of nu_t (x_nb - x);
//   * a domain face's ghost: a wall 2 (U - x), an inflow 2 (ambient - x), an
//     outlet the zero gradient (no term), a periodic face its wrap.
//   * nu_t is 0 in a solid cell (regions.solid: no open face), and the nu_t of
//     an edge, which three of a face's links need, is the mean over the FLUID
//     cells round it (none: 0).
//
// STABILITY. Explicit diffusion in 3-D needs nu_t h / dx^2 <= 1/6. nu_t is held
// at dx^2 / (8 h) (cap: a quarter margin) and the cells held are counted. A link
// whose weight (1 / f up to 100, kMinCrossing) would break the bound at that
// face is divided by it (the face then takes a weighted mean of its neighbours
// and the wall: it cannot overshoot); those faces are counted too. Neither
// refuses a step: they limit the model and do not make the state ill-posed.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

// The largest Cs the step takes. Lilly's value is 0.17 (the probes' choice);
// 0.1 to 0.2 is the range in use.
inline constexpr float kSgsCsMax = 0.3f;

// 0, or a finite Cs in (0, kSgsCsMax].
[[nodiscard]] bool sgs_cs_valid(float cs) noexcept;

struct SgsScratch {
    std::vector<float> nut;                     // per cell: the eddy viscosity, m^2/s
    std::vector<float> uc, vc, wc;              // per cell: the centre velocity
    std::vector<float> du, dv, dw;              // per face: the change this step
    std::vector<uint32_t> capped;               // per cell plane k: cells held at the cap
    std::vector<float> peak;                    // per cell plane k: the largest nu_t
    std::vector<uint32_t> limited;              // per face plane (3 (nz + 1)): faces whose wall weight was limited
};

// What a step reports (FluidHealth's sgs_ fields).
struct SgsReport {
    uint32_t capped_cells = 0;   // cells whose nu_t the cap held
    uint32_t limited_faces = 0;  // faces whose wall-link weight the bound limited
    float nu_t_max = 0.0f;       // the largest nu_t before the cap, m^2/s
};

// Sizes `s` for `shape` (make_fluid_scratch() does it when Cs > 0; the step does it once if Cs was switched on later).
void sgs_size(SgsScratch& s, const GridShape& shape);

// `solids` null or with no part: no walls. `solid_cell` is regions.solid (per cell, 1 where no face is open) or null.
// `cs` must be valid (sgs_cs_valid()) and above 0, `h` positive. Allocation-free once sized.
SgsReport sgs_diffuse(MacGrid& g, const DomainBc& bc, const SolidField* solids, const uint8_t* solid_cell, float cs,
                      float h, SgsScratch& s, WorkerPool* pool) noexcept;

}  // namespace spade::physics::airflow
