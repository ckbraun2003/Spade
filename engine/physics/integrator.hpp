#pragma once

#include <cstdint>
#include <span>

#include "state/layout.hpp"

// ---------------------------------------------------------------------------
// The Integrate pass (engine design spec D1; §3 "The step model"; §5 "Rigid
// bodies"). One substep of semi-implicit (symplectic) Euler over a world's
// body span, plus the specific-force capture the IMU pass consumes.
//
// WHERE THIS SITS IN THE SCHEDULE. The module schedule's six phases, per substep:
//
//     Fields -> Forces -> Constraints/Contacts (static, then dynamic) ->
//     Integrate -> Sensors -> Publish (sim/standard_modules.cpp)
//
// so by the time Integrate runs, `force_acc`/`torque_acc` hold the whole
// substep's accumulated wrench and Integrate's only job is to turn it into
// new state. Two consequences worth stating out loud:
//
//   * GRAVITY IS APPLIED HERE, NOT READ FROM force_acc. This function adds
//     `params.gravity` to the acceleration itself (step 3 below) rather than
//     expecting another pass to have folded m*g into force_acc. That is the
//     op order this task was specified against, and it is what makes the
//     specific-force capture exact rather than a subtraction. No pass may
//     also add m*g to force_acc, or gravity is applied twice. The module
//     schedule has no gravity pass (engine A9), and
//     Schedule.GravityIsAppliedExactlyOnce pins that Integrate is the one
//     site that applies it.
//   * ACCUMULATORS ARE CLEARED HERE (step 7), so the next substep's force
//     passes start from zero. A body skipped for being inactive keeps its
//     accumulators untouched -- see the flags note below.
//
// PURE FUNCTION OF (state, wrench, dt). D1 requires the integrator API to be
// a pure function of its inputs so integrator families can be swapped without
// touching the state layout. integrate_bodies() reads and writes only the
// BodyState elements it is handed plus the const WorldParams; it holds no
// state, allocates nothing, reads no clock, and draws no randomness. Running
// it twice on equal inputs produces byte-equal outputs (test_integrator.cpp
// pins that with memcmp).
//
// NO SUBSTEP LOOP HERE. `h` is the EFFECTIVE SUBSTEP dt (dt / substeps), and
// this function advances exactly one substep. Owning the substep loop, the
// step boundary, and the tick counter is the Simulation/schedule layer's job.
//
// PARITY. This op order is the CPU<->GPU parity contract (P1/P2, D11): the
// Slang implementation must perform the same operations, in the same order,
// with the same groupings, in fp32. The numbered comments in integrator.cpp
// are normative for that mirror -- including the parenthesization and the
// division-vs-reciprocal-multiply choices, which are not fp32-equivalent.
// ---------------------------------------------------------------------------

namespace spade::physics {

// ---------------------------------------------------------------------------
// BodyState::flags bit assignments.
//
// layout.hpp declares `flags` as "a bitfield reserved for per-body predicates
// (asleep, kinematic, ...)" and deliberately assigns no bits: the layout owns
// the FIELD, the passes own its MEANING. This namespace is where slot 0 of
// that convention is defined, and it lives here -- next to the only pass that
// currently reads it -- until the vehicle/model-type layer (D4) consolidates
// the per-body predicate bits in one place. Anything that spawns a body must
// set `active`; anything else that comes to read `flags` should include this
// header rather than re-spelling the constant.
// ---------------------------------------------------------------------------
namespace body_flags {

// Slot 0: this body participates in the physics passes.
//
// It is deliberately ACTIVE-HIGH so that a zeroed slot is inert. ArenaSet
// zero-fills arena storage at registration and again on free, so an
// unallocated or tombstoned slot has flags == 0 and is skipped by this pass
// without any liveness lookup -- the integrator never has to consult the
// slot_to_world map, and a freed slot cannot be resurrected by stale forces.
// The corollary is that the spawn path MUST set this bit; a body left at
// flags == 0 will silently never move.
inline constexpr uint32_t active = 1u << 0;

}  // namespace body_flags

// ---------------------------------------------------------------------------
// Advances every ACTIVE body in `bodies` by one substep of duration `h`.
//
// `bodies` is normally one world's partition (ArenaSet::world_slice), which
// is why the per-world `params` is a single record rather than per body.
// Bodies without body_flags::active are left byte-for-byte untouched --
// including their accumulators, which are NOT cleared for a skipped body.
//
// FRAMES (the contract this pass imposes on its callers):
//   pos, vel, force_acc, params.gravity   world frame
//   orient                                body -> world, unit quaternion
//   omega_body, inv_inertia_diag,
//   torque_acc, specific_force            body frame
//
// torque_acc is BODY-frame, per the ruling recorded in layout.hpp's BodyState
// frames note (which is the single source S6's Slang generator inherits).
// Step 4 sums it with gyroscopic_torque()'s body-frame result and scales by
// the body-frame `inv_inertia_diag`, so all three share the frame the inertia
// tensor is diagonal in. The accumulator asymmetry -- world-frame forces,
// body-frame torques -- is intentional, so the torque producers (the
// force passes and the contact passes) owe body-frame torques.
//
// UNITS: pos m, vel m/s, orient unitless, omega_body rad/s, mass kg,
// inv_inertia_diag 1/(kg m^2), force_acc N, torque_acc N m, specific_force
// and gravity m/s^2, h s.
//
// PRECONDITIONS on an active body: `mass` > 0 (it is divided by, and is
// stored directly rather than as an inverse, so 0 means "degenerate", not
// "infinite" -- unlike inv_inertia_diag, which stores the inverse and for
// which 0 legitimately means "infinite inertia, this axis cannot be
// angularly accelerated"); `orient` a unit quaternion (integrate_orientation
// renormalizes on the way out, so this self-corrects after the first call).
// ---------------------------------------------------------------------------
void integrate_bodies(std::span<BodyState> bodies, const WorldParams& params, float h) noexcept;

}  // namespace spade::physics
