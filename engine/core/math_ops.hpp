#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace spade::math {

// ---------------------------------------------------------------------------
// Rigid-body math primitives underneath the engine's Integrate pass (engine
// design spec D1/D2; design-specs/kat-spade-engine-design.html S5 "Rigid
// bodies (D1, D2)"). The full per-substep op order there is:
//
//   1. velocities integrated from the accumulated wrench (owned by the
//      Integrate pass itself -- a later task, not this header);
//   2. orientation advanced via the quaternion exponential map
//      (integrate_orientation), exact for constant body-frame angular
//      velocity over the substep;
//   3. the quaternion renormalized every substep -- folded INTO
//      integrate_orientation below so no caller can omit it, matching the
//      spec's "deterministic op order" requirement;
//   4. Euler's rigid-body torque term omega x (I * omega), supplied by
//      gyroscopic_torque(), added to the torque accumulator ahead of the
//      angular-velocity update that feeds step 2's omega.
//
// This op order is parity-relevant: the CPU reference twin and the GPU path
// (D11) must both fold gyroscopic_torque's result into the SAME substep's
// torque accumulator before deriving dOmega/dt, and must call
// integrate_orientation exactly once per substep with that substep's
// already-updated omega. Reordering, skipping the renormalize, or splitting
// a substep differently changes the trajectory at the fp32 rounding level --
// invisible in a single step, but it breaks CPU<->GPU parity and determinism
// replay (D11) over a run.
// ---------------------------------------------------------------------------

// Advances a body's orientation by one substep of constant body-frame
// angular velocity `omega_body`, via the quaternion exponential map:
//
//     q' = q (x) exp(0.5 * dt * omega_body)
//
// exp() of the pure quaternion (0, 0.5*dt*omega_body) is
// (cos(theta/2), sinc(theta/2) * 0.5*dt*omega_body) where theta =
// |omega_body| * dt is the total angle swept this substep and
// sinc(x) = sin(x)/x -- this is EXACT (not merely first-order accurate) for
// omega_body constant over the substep, not an approximation.
//
// Below |omega_body| * dt < 1e-6, the closed-form cos/sinc pair is replaced
// by its Taylor series (cos(x) ~= 1 - x^2/2, sinc(x) ~= 1 - x^2/6) so the
// sin(x)/x removable singularity is never evaluated as a 0/0 division near
// x == 0; the two branches agree to fp32 precision at the switchover.
//
// The result is always renormalized before return, so quaternion drift
// cannot accumulate across steps and callers never need to do it themselves.
[[nodiscard]] glm::quat integrate_orientation(glm::quat q, glm::vec3 omega_body, float dt) noexcept;

// Rotates a body-frame inertia tensor into world frame:
//
//     I_world = R * I_body * R^T,   R = mat3_cast(orientation)
//
// R^-1 == R^T holds only because `orientation` is a unit quaternion --
// integrate_orientation's per-substep renormalization is what keeps that
// true. `I_body` may carry products of inertia (general symmetric tensor).
[[nodiscard]] glm::mat3 inertia_world(glm::mat3 I_body, glm::quat orientation) noexcept;

// Convenience overload for the common case: a vehicle model parameterized
// by its three principal moments only (no products of inertia), rotated
// straight to world frame in one call.
[[nodiscard]] glm::mat3 inertia_world(glm::vec3 principal_moments_body, glm::quat orientation) noexcept;

// Euler's rigid-body term: the torque an unforced rotating body's own
// angular momentum exerts on itself,
//
//     gyroscopic_torque = -(omega_body x (I_body * omega_body))
//
// Callers add this into the external-torque accumulator ahead of deriving
// dOmega/dt = I_body^-1 * (tau_ext + gyroscopic_torque(I_body, omega_body)).
// Identically zero whenever I_body is a scalar multiple of the identity
// (spherical inertia): I_body * omega_body is then parallel to omega_body,
// so their cross product vanishes.
[[nodiscard]] glm::vec3 gyroscopic_torque(glm::mat3 I_body, glm::vec3 omega_body) noexcept;

}  // namespace spade::math
