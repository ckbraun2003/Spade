#pragma once

#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// DYE (P10): a passive tracer carried by the air, the smoke of the website clips. One float d in [0, 1] per CELL, 0 clean air.
//
// PASSIVE: the dye never acts on the flow, so with it off every bit of the velocity and the pressure is what it was, and with it on
// they are the same bits (a test holds both). It lives in the FluidScratch (fluid_step.hpp): enable_dye() sizes it, fluid_step()
// advects it at the end of every step with the step's own final velocity (advect_scalar, advect.hpp: RK2 backtrace, MacCormack
// clamped to the departure stencil, so d stays in [0, 1], never negative, no new extremum), and the caller releases it at the discs
// after the step with emit_dye(), as it calls spread_actuators() before it.
//
// THE RELEASE: a disc releases dye in a thin sheet DOWNSTREAM of its plane (the side its axis points to) and nowhere upstream:
//   inc = min(1, rate h * 2 phi4(s / dx) * w(r)),  d <- d + (1 - d) inc
// s the cell centre's distance along the disc's axis from its plane (s >= 0 only: the release is one-sided, so it adds no halo
// above the disc where the inflow would draw it through; the 2 keeps the amount a two-sided phi4 sheet would give), phi4 Peskin's
// 4-point kernel, so the sheet is 2 cells deep. w(r) is a window in the radius r from the axis, 1 between r_in R and r_out R, its
// edges one cell wide. r_in 0 is the whole disc; the website's look is the tip ring, r_in 0.85 and r_out 1. rate is how fast an
// empty cell at the sheet's peak fills, 1/s.
//
// Every expression is dye_increment()'s, in its order: the GPU twin (airflow_dye_source.slang) is it term for term, bit for bit.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct DyeRelease {
    float rate = 120.0f;  // 1/s
    float r_in = 0.85f;   // r / R; 0: the whole disc
    float r_out = 1.0f;   // r / R
};

// invalid_argument, naming the field: a rate not finite or negative; r_in negative, r_out not above r_in, or either not finite or
// above 2.
[[nodiscard]] Result<void> check_dye_release(const DyeRelease& release);

// One cell's increment factor in [0, 1] from one disc: the cell centre p, k = rate * h and 1 / dx of the grid. 0 outside the
// sheet. The CPU's emit_dye() and the GPU kernel both take this, and nothing else, as the release.
[[nodiscard]] float dye_increment(const ActuatorDisc& disc, glm::vec3 p, float inv_dx, float k, float r_in,
                                  float r_out) noexcept;

// Release from `discs` into `dye` (nx ny nz floats, cell order), disc by disc, each cell in order: d += (1 - d) inc. h is the
// fluid step. Allocation-free. Runs with subnormals flushed as fluid_step() does.
void emit_dye(std::span<float> dye, const GridShape& shape, std::span<const ActuatorDisc> discs, float h,
              const DyeRelease& release) noexcept;

// ---------------------------------------------------------------------------
// THE TIP RELEASE (L86i): a line's blades turn through the air, so the dye can be released where the blades' tips are, and a tip's
// path through the air is a helix. For every rotor that turns (omega > 0), at each of the step's n_sub sub-azimuths (the line's own,
// line_elements()' order: rotor, sub-azimuth, blade) the point on the blade's last element's direction, `tip_frac` R from the axis
// and at the element's height along it, is a release point. Each point puts dye into the cells within two cells of it, as a 3D
// Peskin 4-point kernel,
//   w = phi4(|dx_c| / dx) phi4(|dy_c| / dx) phi4(|dz_c| / dx),  inc = min(1, gain w),  d <- d + (1 - d) inc
// the points in order, each cell visited in order; the GPU twin (airflow_dye_tip.slang) is emit_tip_dye() term for term.
// ---------------------------------------------------------------------------
struct TipRelease {
    float gain = 1.0f;      // inc = min(1, gain w): w is 1/8 at a point on a cell centre's axis crossing, so 8 fills it
    float tip_frac = 1.0f;  // r / R of the release point, the blade's end at 1
};

// invalid_argument, naming the field: a gain not finite or not above 0; tip_frac not finite or outside (0, 1.5].
[[nodiscard]] Result<void> check_tip_release(const TipRelease& release);

// The release points of `rotors` for a step of h seconds in n_sub sub-azimuths, in the grid's frame: rotor by rotor, sub-azimuth by
// sub-azimuth, blade by blade, those of a rotor with omega > 0 only. Pure function of the rotors: the GPU host packs these floats.
[[nodiscard]] std::vector<glm::vec3> tip_points(std::span<const LineRotor> rotors, float dx, float h, uint32_t n_sub,
                                                float tip_frac);

// Releases dye at `points` into `dye` (nx ny nz floats, cell order), as above; gain is TipRelease::gain. Allocation-free. Runs with
// subnormals flushed as fluid_step() does.
void emit_tip_dye(std::span<float> dye, const GridShape& shape, std::span<const glm::vec3> points, float gain) noexcept;

}  // namespace spade::physics::airflow
