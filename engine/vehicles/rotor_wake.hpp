#pragma once

#include <glm/glm.hpp>

// ===========================================================================
// rotor_wake_velocity() -- the air a rotor moves, as an analytic field.
//
// VISUALISATION GRADE, AND NOTHING IN THE STEP READS IT. RotorElement
// (vehicles/rotor.hpp) still models no wake: its thrust comes from the inflow
// curve at the disc alone, and no body, sensor or force element samples this
// function. It exists so a viewer can draw the downwash a rotor is already
// implied to produce. It is a pure, total CPU function: no state, no clock, no
// randomness, and no GPU twin, because it carries no parity claim.
//
// It is built ON the rotor model rather than beside it. The disc's induced
// velocity is the one RotorElement's closure already uses,
//
//     v_i = v_h . lambda(v_axial / v_h),   v_h = rotor_hover_induced_velocity(k_T w^2, rho, R)
//
// with lambda = rotor_inflow_factor() (rotor.hpp sections 1-3), so a rotor
// that is climbing, descending or sitting in the windmill-brake state shows
// the same induced velocity here that it produces thrust from there.
//
// ---------------------------------------------------------------------------
// THE MODEL: a straight, skewed, contracting actuator-disc slipstream
//
// Let n = -axis be the direction the disc pushes air, F the freestream (air
// velocity at the rotor, world frame), and s, r the downstream distance along
// the wake axis and the distance from it.
//
//   1. AXIAL PROFILE. On the axis of a uniformly loaded disc (the
//      semi-infinite vortex-cylinder model) the induced speed is
//
//          u(s) = v_i (1 + s / sqrt(s^2 + R^2)),
//
//      so v_i at the disc, 2 v_i far downstream (momentum theory's far wake)
//      and 0 far upstream. For s < 0 it is evaluated in its conjugate form
//      R^2 / (q (q - s)), q = sqrt(s^2 + R^2), because the direct form
//      subtracts two nearly equal numbers there. This is the same habit as
//      rotor.cpp's inflow roots: accurate to a rounding over the whole line,
//      not only near the disc.
//
//   2. WAKE AXIS. The wake is convected by the far-wake air velocity
//      F + 2 v_i n, so the axis runs from the hub along that direction. A
//      crosswind skews it downwind. In the windmill-brake state the flow
//      through the disc reverses and the wake lies on the THRUST side of the
//      disc, where the induced velocity slows the oncoming air. The convection
//      speed rises from v_i at the disc to 2 v_i, so the true centreline
//      curves: with no axial freestream it ends up displaced downwind of this
//      straight axis by a constant |F_t| R / (3 v_i) (integrate dz / u(z)).
//      For the sandbox drone in a 5 m/s crosswind that is about 4 cm, roughly
//      one heatmap cell, so the straight axis is kept.
//
//   3. CONTRACTION BY CONTINUITY. With U(s) = F.w + u(s) n.w, the air speed
//      along the wake axis w, the tube radius obeys
//
//          r_tube(s) = R sqrt(U(0) / U(s)).
//
//      In still air this is R sqrt(v_i / u): the classical contraction to
//      R/sqrt(2). In climb or a crosswind the freestream carries most of the
//      mass flow, so the slipstream contracts less, as momentum theory says
//      it should. In the windmill-brake state it expands, because the flow is
//      decelerating. Where U(0) or U(s) is not positive (inside the vortex-ring
//      band, where momentum theory has no steady streamtube), the still-air
//      ratio v_i / u is used instead.
//
//   4. SMOOTH EDGE. The speed is weighted by 1 / (1 + (r / r_tube)^16): about
//      1 inside the tube, 1/2 at its edge, and below 1e-4 at 1.8 r_tube. This
//      over-counts the tube's mass flow by 2.6%, which is invisible on a
//      heatmap.
//
//   5. DIRECTION. The induced velocity is along n, normal to the disc, as
//      momentum theory gives it, in axial and skewed flow alike. So far
//      downstream F + induced is parallel to the wake axis, and the plume's
//      streamlines run along the plume.
//
// NOT MODELLED, and each is a known step up rather than an oversight:
//   * swirl;
//   * the tip-vortex structure;
//   * the fore-aft inflow gradient a skewed wake produces at the disc, beyond
//     the one the rotated geometry gives for free;
//   * interaction between rotors (the caller sums them);
//   * obstruction by the airframe;
//   * the ground. A wake meeting the ground spreads; this one passes through.
// A solved flow field replaces all of this as a higher-fidelity field
// provider (restructure spec section 2).
// ===========================================================================

namespace spade::vehicles {

// One rotor, as rotor_wake_velocity() sees it. Every field is a plain value,
// so the caller builds it from a RotorRow plus its body pose (or from nothing)
// without this header naming the arena layout.
struct RotorWakeInput {
    glm::vec3 hub_world{0.0f};                      // rotor hub, world frame, m
    glm::vec3 thrust_axis_world{0.0f, 1.0f, 0.0f};  // thrust direction, world frame; normalised here. The wake flows opposite
    float radius = 0.0f;                            // R, m
    float thrust_coeff = 0.0f;                      // k_T in T_static = k_T w^2, N s^2
    float omega = 0.0f;                             // shaft speed, rad/s
    float density = 0.0f;                           // rho, kg/m^3
    glm::vec3 freestream{0.0f};                     // air velocity at the rotor, world frame, m/s
};

// Induced air velocity (world frame, m/s) at `point` from one rotor's
// slipstream. The freestream itself is NOT included; the caller adds it once,
// however many rotors it sums.
//
// TOTAL. It never returns a NaN or an infinity, and it returns exactly zero
// for:
//   * a stopped rotor;
//   * a non-positive or non-finite radius, density, k_T or omega^2 thrust;
//   * a zero or non-finite thrust axis;
//   * a non-finite freestream or point, or one so large its square overflows.
[[nodiscard]] glm::vec3 rotor_wake_velocity(const RotorWakeInput& in, glm::vec3 point) noexcept;

}  // namespace spade::vehicles
