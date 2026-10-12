#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "physics/airflow/advect.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/projection.hpp"
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
    uint32_t n_v = 2;          // V-cycles per projection
    uint32_t visc_sweeps = 8;  // red-black sweeps per viscous solve
};

enum class FluidFault : uint8_t {
    none,
    non_finite,        // a NaN or infinity in u, v, w or p after the step
    ill_posed_domain,  // a closed domain whose prescribed wall fluxes do not balance
};

struct FluidHealth {
    FluidFault fault = FluidFault::none;
    uint8_t quantity = 0;   // non_finite: 0 u, 1 v, 2 w, 3 p
    uint8_t faces = 0;      // ill_posed_domain: bit s for each wall face with a normal velocity
    std::size_t index = 0;  // non_finite: the first bad sample's index in its array (array order)
};

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
};

// The bytes of a grid and of the scratch its step needs, in 64 bits (the
// multigrid's depth follows the domain's periodic axes).
[[nodiscard]] uint64_t fluid_bytes(const GridShape& shape, const DomainBc& bc) noexcept;

// A closed domain (no pressure outlet) must not take a net prescribed flux
// through its walls: the outward normal flux of every wall face, summed, must
// vanish within 1e-6 of the sum of their magnitudes (fp32's resolution).
// Refused (invalid_argument) naming every wall face with a normal velocity.
[[nodiscard]] Result<void> check_domain(const DomainBc& bc, const GridShape& shape);

// Refused, before anything is allocated:
//   invalid_argument, naming the field: rho or h not positive and finite; nu
//     negative or not finite; n_v = 0; visc_sweeps = 0 with nu > 0;
//   invalid_argument, naming the faces: check_domain();
//   capacity_exceeded, naming the bytes and the cap: fluid_bytes() past
//     kGridByteCap.
[[nodiscard]] Result<FluidScratch> make_fluid_scratch(const MacGrid& g, const DomainBc& bc, const FluidConfig& cfg);

// One step. Returns the scratch's health: FluidFault::none, or the fault
// that stopped this step or an earlier one (the grid then is unchanged).
FluidHealth fluid_step(MacGrid& g, const DomainBc& bc, const FluidConfig& cfg, FluidScratch& s) noexcept;

}  // namespace spade::physics::airflow
