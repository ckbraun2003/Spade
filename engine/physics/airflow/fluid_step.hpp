#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "physics/airflow/advect.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/projection.hpp"
#include "physics/airflow/regions.hpp"
#include "physics/airflow/solids.hpp"
#include "physics/airflow/viscous.hpp"

// ---------------------------------------------------------------------------
// ONE FLUID STEP of the real-time tier (spec §1.5): conditions, advection,
// sources, conditions, viscosity (if nu > 0) and conditions, projection.
// Allocation-free: everything it touches is sized by make_fluid_scratch().
//
// IT REFUSES RATHER THAN RUN ON (L6). A step whose domain configuration has
// turned ill-posed, or whose result holds a non-finite value, reports it as a
// FluidHealth and poisons the scratch: every later step returns the same
// fault and changes nothing, until a new scratch is made.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct FluidConfig {
    float rho = 1.225f;        // kg/m^3
    float nu = 0.0f;           // m^2/s, molecular; 0 is the inviscid lean mode
    float h = 0.0f;            // the fluid step, s
    uint32_t n_v = 2;          // V-cycles per projection; with solids, CG iterations (kSolidsCgIterations)
    uint32_t visc_sweeps = 8;  // red-black sweeps per viscous solve
    SolidInterior interior = SolidInterior::rigid;  // what closed samples hold (solids.hpp)
};

enum class FluidFault : uint8_t {
    none,
    non_finite,        // a NaN or infinity in u, v, w or p after the step
    ill_posed_domain,  // a closed domain whose prescribed wall fluxes do not balance
    ill_posed_region,  // a closed fluid region whose prescribed domain fluxes do not balance (regions.hpp)
};

struct FluidHealth {
    FluidFault fault = FluidFault::none;
    uint8_t quantity = 0;   // non_finite: 0 u, 1 v, 2 w, 3 p
    uint8_t faces = 0;      // ill_posed_domain: bit s for each wall face with a normal velocity
    std::size_t index = 0;  // non_finite: the first bad sample's index in its array (array order);
                            // ill_posed_region: the region's first cell (faces: its faces)
    float pressure_residual = 0.0f;  // with solids: this step's max|r| / max|b| (pcg.hpp)
    bool unconverged = false;        // pressure_residual > kResidualAnnounce: announced (L6), not refused
};

// The CG iteration count for a run with solids (P2 Task 9, the solids design
// section 3): the smallest n whose cold solves reach 1e-3 and warm ones 1e-4
// on S1, S3 (both parities) and the reference quad's parts. Measured: 8
// leaves every case at fp32's floor, 1e-6 to 3e-5; 4 leaves the sheet's and
// the quad's cold solves near 1e-3 to 2e-3. A run with solids sets n_v to it.
inline constexpr uint32_t kSolidsCgIterations = 8;

// A pressure solve leaving more than this relative residual is announced (the
// lead's condition, spec 1.5 as amended): ten times the warm residual the
// count above leaves (1.2e-6), held at the floor of 1e-4.
inline constexpr float kResidualAnnounce = 1.0e-4f;

// What a fault means, naming the quantity and its sample (i, j, k), or the
// faces. Allocates: call it outside the step.
[[nodiscard]] std::string describe(const FluidHealth& health, const GridShape& shape);

struct FluidScratch {
    MacGrid next;                   // advection's output, swapped in
    AdvectScratch advect;
    ProjectionScratch projection;
    std::vector<float> rhs;         // the viscous solve's right side
    std::vector<float> du, dv, dw;  // velocity increments a source wrote this step; cleared when added
    FluidHealth health;             // sticky: once faulted, every later step refuses
    SolidField solids;              // set_solids(); empty: plan A's paths
    FluidRegions regions;
};

// The bytes of a grid and of the scratch its step needs, in 64 bits (the
// multigrid's depth follows the domain's periodic axes).
[[nodiscard]] uint64_t fluid_bytes(const GridShape& shape, const DomainBc& bc) noexcept;

// A closed domain (no pressure outlet) must not take a net prescribed flux
// through its walls: the outward normal flux of every wall face, summed, must
// vanish within 1e-6 of the sum of their magnitudes (fp32's resolution).
// Refused (invalid_argument) naming every wall face with a normal velocity.
[[nodiscard]] Result<void> check_domain(const DomainBc& bc, const GridShape& shape);

// The bytes set_solids() adds for the shape, in 64 bits: the solid field, the
// regions' per-cell arrays, the PCG's vectors and every multigrid level's
// face weights. Not counted: what grows with the solid surface (the wall
// links, the fill layers) or with the region count.
[[nodiscard]] uint64_t solid_bytes(const GridShape& shape, const DomainBc& bc) noexcept;

// Whether a grid of this shape and these faces can be made and stepped, with
// solids or without: check_shape() (make_grid()'s refusals), check_domain(),
// then fluid_bytes() (plus solid_bytes() with solids) within kGridByteCap,
// refused capacity_exceeded naming the bytes and the cap. One rule for
// make_grid(), make_fluid_scratch(), set_solids() and a scene's validator
// (TD-9; the N5b agreement with Core); no allocation but a refusal's message.
[[nodiscard]] Result<void> check_grid(const GridShape& shape, const DomainBc& bc, bool with_solids);

// Refused, before anything is allocated:
//   invalid_argument, naming the field: rho or h not positive and finite; nu
//     negative or not finite; n_v = 0; visc_sweeps = 0 with nu > 0;
//   check_grid(shape, bc, false): check_domain() by face, and fluid_bytes()
//     past kGridByteCap naming the bytes and the cap.
[[nodiscard]] Result<FluidScratch> make_fluid_scratch(const MacGrid& g, const DomainBc& bc, const FluidConfig& cfg);

// One step. Returns the scratch's health: FluidFault::none, or the fault
// that stopped this step or an earlier one (the grid then is unchanged).
FluidHealth fluid_step(MacGrid& g, const DomainBc& bc, const FluidConfig& cfg, FluidScratch& s) noexcept;

// Sets the scratch's solids from `parts` (a host call: on first use it
// allocates the solid field, and every call relabels the regions and resets
// the multigrid's weights and CG's vectors to their sizes). No parts: plan A's
// paths, bit for bit. Refused as build_solids() refuses, leaving no solid.
[[nodiscard]] Result<void> set_solids(FluidScratch& s, const MacGrid& g, const DomainBc& bc,
                                      std::span<const SolidPart> parts, const SolidSampler& sample);

}  // namespace spade::physics::airflow
