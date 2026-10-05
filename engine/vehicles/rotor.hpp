#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "state/layout.hpp"
#include "world/medium.hpp"
#include "world/sdf.hpp"

// ===========================================================================
// RotorElement -- the first VEHICLE-LAYER force element (engine design D5:
// "Rotor aero v2 = curves + RPM lag + momentum-theory inflow + SDF-sampled
// ground effect. BEMT + flow-field coupling = roadmap behind P7"; spec S6
// "Vehicles -- the model-type layer").
//
// It runs as rotor.forces, in the Forces phase just before drag.forces
// (physics/forces.hpp's DragBody; sim/standard_modules.cpp pins that order).
// Like drag, it accumulates
// into the same two BodyState accumulators, and follows the same registered-
// state discipline. What is new here is that a rotor carries STATE of its own
// -- the lagged shaft speed omega -- so this pass both reads and writes its
// own rows, and `h` is finally load bearing (forces.hpp predicted exactly
// this: "a future kind with its own internal dynamics (e.g. motor spin-up
// lag) will need it even though DragBody does not").
//
// WHY IT LIVES UNDER vehicles/ AND NOT physics/. Spec S6 lists RotorElement
// under the model-type layer, and the coordinator's resolution gives
// engine/vehicles/ its own target (spade::vehicles). The split is by WHAT THE
// MODEL KNOWS: physics/ knows rigid bodies and fields; vehicles/ knows that
// some bodies are machines with actuators. Task 18's Quadrotor ModelType is
// the first consumer of this file and sits beside it.
//
// SCOPE, STATED UP FRONT (D5's second sentence). No BEMT, no blade elements,
// no rotor wake or downwash field, no rotor-inertia gyroscopic term. (There
// is an analytic wake in vehicles/rotor_wake.hpp, but it is for drawing: it is
// built from this element's inflow and nothing in the step reads it.) Those
// are the P7 roadmap; RotorRow's reserved lanes are where their parameters
// land so that adopting them does not move an existing field. Three more
// boundaries of what this element is answerable for:
//
//   * THE DISC'S BLUFF-BODY DRAG IS NOT ITS JOB. Deep in the windmill brake
//     state the inflow factor decays toward 0 (section 2), so a fast-descending
//     rotor stops producing thrust rather than turning into the drag plate a
//     real stalled disc becomes. That is the intended division of labour, not
//     a gap: gross body drag belongs to DragBody (physics/forces.hpp), which
//     the same vehicle carries, and double-counting it here would make the
//     element's own regime boundaries depend on how the vehicle was assembled.
//
//   * THE TORQUE CURVE IS NOT CORRECTED, ONLY THE THRUST IS. Spec S6's step
//     line applies f_inflow and f_ground to T alone and this file follows it
//     exactly -- but the consequence deserves saying out loud: at fixed omega
//     the shaft torque, and therefore the shaft power, is independent of both
//     inflow and ground proximity. So the model's thrust-per-watt rises by the
//     full 25% in saturated ground effect and falls in climb, FOR FREE. The
//     SIGN is right (a rotor in ground effect genuinely is more efficient),
//     but the efficiency arrives as an unpriced bonus rather than out of a
//     reduced induced power. A torque model sharing the inflow correction is
//     the natural companion to BEMT (P7) and would claim its own reserved
//     lane. Until then: DO NOT READ THIS ELEMENT'S Q AS A POWER BUDGET.
//
//   * DEGENERATE PARAMETERS ARE NOT DIAGNOSED, and one of them is quiet. A
//     row with radius == 0 disables BOTH corrections at once -- v_h is 0 so
//     f_inflow is 1, and there is no disc so f_ground is 1 -- leaving a rotor
//     that produces its static thrust in every flight state, at every
//     altitude, with no error anywhere. See apply_rotors()' preconditions;
//     validating it belongs to the model-type layer above (Task 18).
//
// ---------------------------------------------------------------------------
// THE MODEL, spec S6 verbatim, then this file's reading of each line:
//
//     params: body slot . local pose . spin dir . time-constant tau (RPM lag)
//             thrust curve T(w) . torque curve Q(w)
//     state:  w (RPM)
//     step:   w += (w_cmd - w).dt/tau
//             v_axial = axis . (v_body + w_b x r - medium.wind)
//             T = T(w) . f_inflow(v_axial)
//             T *= f_ground(phi_world(p_rotor))
//             apply wrench {T.axis, Q(w).spin + T.axis x r} at body
//
//   * "w (RPM)" is stored in RAD/S here. The engine is SI throughout
//     (layout.hpp, integrator.hpp: omega_body is rad/s), and a single field in
//     rev/min inside an otherwise-SI state row is the kind of unit seam that
//     produces a factor of 9.55 somewhere downstream. "RPM lag" stays the name
//     of the phenomenon; the number is rad/s.
//   * "w += (w_cmd - w).dt/tau" is evaluated EXACTLY rather than by that
//     first-order step -- see section 5 below, which also shows the two agree
//     to within theta/2.
//   * "T.axis x r" is written here as the standard moment cross(r, F): the
//     spec's operand order is the negative of the moment of a force about the
//     COM, and physics/forces.hpp already pins `cross(elem.local_pos, F_body)`
//     for the same quantity. Matching DragBody is what keeps one body's
//     torque accumulator self-consistent.
//   * "Q(w).spin" is written here as MINUS spin_dir * Q(w) along the thrust
//     axis: Q is carried as a non-negative drag-torque magnitude and the
//     reaction on the airframe opposes the rotor's own rotation. Section 6.
//
// ---------------------------------------------------------------------------
// 1. MOMENTUM THEORY, AXIAL FLIGHT
//
// An actuator disc of area A = pi R^2 in air of density rho, translating along
// its own thrust axis at climb speed V_c (V_c > 0 means moving in the
// direction the thrust points), with induced velocity v_i at the disc. The
// mass flow through the disc is mdot = rho A (V_c + v_i) and the far wake is
// accelerated to V_c + 2 v_i, so
//
//     T = mdot . 2 v_i = 2 rho A (V_c + v_i) v_i.                        (M)
//
// At hover (V_c = 0) that is T = 2 rho A v_i^2, i.e. the HOVER INDUCED
// VELOCITY for a given thrust is
//
//     v_h = sqrt(T / (2 rho A)).                                         (H)
//
// (H) is rotor_hover_induced_velocity(); it is the only place rho and R enter
// the inflow model, and it is what makes the correction below dimensionless.
//
// ---------------------------------------------------------------------------
// 2. THE INDUCED-VELOCITY CURVE AND ITS THREE BRANCHES
//
// Non-dimensionalize with x = V_c / v_h and lambda = v_i / v_h. Holding the
// thrust at 2 rho A v_h^2, (M) becomes
//
//     lambda^2 + x lambda - 1 = 0                                        (Q)
//
// whose positive root is the NORMAL WORKING STATE solution
//
//     lambda(x) = -x/2 + sqrt(x^2/4 + 1).                              (NWS)
//
// In DESCENT the streamtube reverses (air enters the disc from below) and the
// momentum balance changes sign, giving the WINDMILL BRAKE STATE root
//
//     lambda(x) = -x/2 - sqrt(x^2/4 - 1),                                (WB)
//
// which is real only for x <= -2. Between x = -2 and x = 0 NEITHER solution is
// physical: the wake recirculates through the disc and there is no steady
// streamtube at all. That is the VORTEX RING / TURBULENT WAKE STATE, and
// momentum theory is simply silent inside it.
//
// WHERE THIS FILE CUTS THE BRANCHES, and why not at the textbook x = 0.
// (NWS) and (WB) both give lambda = 1 at their own boundaries (x = 0 and
// x = -2), so interpolating endpoint-to-endpoint across the whole band would
// give lambda == 1 throughout -- i.e. NO descent thrust rise anywhere, which
// contradicts both measurement and the entire point of the correction.
// Measured induced velocities (Castles & Gray; the quartic fit in Johnson,
// "Helicopter Theory")
//
//     lambda_J(x) = 1 - 1.125 x - 1.372 x^2 - 1.718 x^3 - 0.655 x^4
//
// instead RISE to about 1.85 near x = -1.1. (NWS) is therefore continued into
// slow descent as far as the classical vortex-ring ONSET criterion
// |V_c| = v_h -- the descent rate matches the induced velocity, so the wake
// can no longer escape downward -- i.e. as far as x = kRotorVrsUpperRatio =
// -1, and only the remaining band gets an approximation:
//
//     -1 <= x            (NWS)   normal working state: climb and slow descent
//     -2 <  x < -1       LINEAR  vortex-ring band, clamped (below)
//     x <= -2            (WB)    windmill brake state
//
// THE BAND: a straight line in x from lambda = 1 at x = -2 to lambda(-1) from
// (NWS) at x = -1. Continuous at BOTH ends by construction, and exactly so:
// (WB) evaluates to 1 at x = -2 with a zero square root, and the band's upper
// endpoint is computed by calling the same (NWS) expression the branch above
// it uses, not by re-spelling the constant.
//
// "CLAMPED" is the operative word: (NWS) continued below x = -1 grows without
// bound (lambda -> |x| as x -> -inf) and would be discontinuous with (WB) at
// x = -2. The linear segment is what bounds it. The whole piecewise curve is
// consequently bounded by its value at the onset,
//
//     max lambda = lambda(-1) = (1 + sqrt(5)) / 2 = 1.6180339887...
//                = kRotorInflowFactorMax,
//
// which is why no explicit clamp appears in the implementation: the curve
// cannot exceed that, and a clamp there would be dead code. It is monotone
// decreasing in x on [-1, inf) (from 1.618 at the onset, through 1 at hover,
// to 0 as x -> inf) and decreasing as x -> -inf on (WB) (from 1 at x = -2
// toward 0), so lambda is in [0, 1.618] everywhere.
//
// CONTINUOUS BUT NOT C1, AND THE x = -2 JOIN IS THE SHARP ONE. (WB) reaches
// its boundary with an INFINITE slope: near x = -2 its square root behaves
// like sqrt(-(x+2)), so lambda drops like 1 - sqrt(eps) just below the join
// while the band rises like 1 + 0.618 eps just above it. A consumer
// differentiating thrust with respect to descent rate therefore sees an
// unbounded derivative there, and a controller linearizing about a deep
// descent should know it. That is a property of MOMENTUM THEORY -- it is
// where the windmill-brake solution ceases to exist -- not an artifact of the
// band, and smoothing it would mean leaving the momentum solution. The join
// at x = -1 is an ordinary kink (both one-sided slopes are finite: -0.724
// from (NWS), +0.618 from the band). test_rotor.cpp pins both, including the
// square-root approach rate.
//
// CONDITIONING. Both roots are written as a difference of two nearly equal
// positive terms in their textbook form, which is exactly the shape that
// destroys an fp32 answer far from the boundary: -x/2 + sqrt(x^2/4 + 1) has
// lost three digits by x = 1000 and evaluates to exactly ZERO above x ~ 8200,
// where the true value is 1.2e-4. rotor.cpp therefore evaluates each root in
// whichever of its two algebraically identical forms does not cancel (the
// conjugate form 1/(x/2 + sqrt(x^2/4 + 1)) on the climb side, and likewise on
// the windmill branch), so lambda is accurate to a rounding across the WHOLE
// domain rather than only in the interesting part of it. Same discipline, and
// the same motivation, as world/medium.hpp section 3's series thresholds.
//
// ACCURACY AGAINST THE EMPIRICAL FIT, tabulated rather than asserted
// (test_rotor.cpp recomputes all of it and pins the bounds):
//
//     x       this file   lambda_J    delta
//     0.00      1.0000     1.0000       0%
//    -0.25      1.1328     1.2198     -7.1%
//    -0.50      1.2808     1.3933     -8.1%
//    -0.75      1.4430     1.5895     -9.2%
//    -1.00      1.6180     1.8160    -10.9%     <- (NWS)/band boundary
//    -1.50      1.3090     2.0825    -37.1%     <- inside the band
//    -1.75      1.1545     1.8273    -36.8%
//    -2.00      1.0000     1.0264     -2.6%     <- band/(WB) boundary
//
// So: within 11% of measurement wherever momentum theory is used, and up to
// 37% LOW inside the band. That under-prediction is deliberate and it is the
// conservative direction -- a rotor that gains less thrust than reality while
// settling into its own wake is the pessimistic case for anything flying on
// top of this. Adopting lambda_J across the band is a one-line upgrade and is
// the roadmap item; it is not taken here because the vortex ring state is
// genuinely unsteady and hysteretic, so a smooth algebraic curve through it
// is a convenient fiction either way, and this task's brief pins the linear
// form.
//
// ---------------------------------------------------------------------------
// 3. FROM INDUCED VELOCITY TO A THRUST RATIO -- THE CLOSURE, STATED AS ONE
//
// THIS IS THE ONE STEP THAT IS NOT MOMENTUM THEORY, and it is stated plainly
// because this file's review is a re-derivation against momentum theory.
// (M) is ONE equation in TWO unknowns (T and v_i) once V_c is given. It
// cannot, by itself, say how a rotor's thrust changes with climb rate; that
// information lives in the blade, and a closure is REQUIRED. v2 takes the
// cheapest closure that needs no blade geometry at all:
//
//     THRUST SCALES WITH THE INDUCED VELOCITY THE DISC DEVELOPS AT FIXED RPM
//
//         T(w, V_c) = T_static(w) . lambda(V_c / v_h),
//         v_h = sqrt(T_static(w) / (2 rho A)).
//
// i.e. f_inflow == lambda. What it gets right, and what it costs:
//
//   * EXACT AT HOVER by construction: lambda(0) = 1, so the thrust curve T(w)
//     is used as exactly what it is measured as -- a static/hover thrust
//     curve. No calibration is silently rescaled.
//   * RIGHT IN SIGN AND IN LEADING ORDER: lambda(x) ~= 1 - x/2 for small x, so
//     thrust falls in climb and rises in descent, which is the direction the
//     blade's sectional angle of attack moves.
//   * NOT A THEOREM. Other closures are equally momentum-consistent and
//     disagree on the SLOPE:
//       - constant thrust (T fixed by w alone): lambda solves (Q) exactly and
//         f == 1 -- no variation at all;
//       - constant mass flow (mdot fixed by w): v_i = v_h - V_c, f = 1 - x,
//         slope -1, thrust crossing zero at x = 1;
//       - BEMT with uniform inflow and ideal twist: C_T = (sigma a / 4)
//         (theta_tip - lambda_total) gives f = 1 - x/(2(mu - 1)) with
//         mu = theta_tip / lambda_h the blade's pitch margin, i.e. slope
//         about -0.2 for a small multirotor (mu ~ 3.5).
//     So this closure's slope of -1/2 is roughly 2.5x more sensitive than
//     BEMT with a realistic pitch margin (and the constant-mass-flow closure
//     is 5x). BEMT is what removes the guess, it is the P7 roadmap item, and
//     the blade parameters it needs (solidity, lift-curve slope, tip pitch)
//     are exactly what RotorRow's reserved lanes are held for.
//
// THE SHARPEST WAY TO STATE WHAT THE CLOSURE COSTS, since "not a theorem" is
// vague and this is not. x is normalized by v_h(T_STATIC), and the induced
// velocity the curve reports is v_i = v_h lambda. Substitute that pair back
// into (M):
//
//     2 rho A (V_c + v_i) v_i = 2 rho A v_h^2 (x + lambda) lambda
//                             = T_static (x + lambda) lambda
//                             = T_static,
//
// because lambda (x + lambda) = 1 is the very equation lambda solves. So
// momentum theory, fed this model's own induced velocity, returns T_static --
// NOT the T_static * lambda the model publishes. THE (T, v_i) PAIR THIS
// ELEMENT REPORTS THEREFORE SITS ON THE MOMENTUM CURVE ONLY AT HOVER
// (lambda = 1); everywhere else the closure has moved the thrust off it, and
// that displacement IS the closure's entire physical content.
//
// Re-normalizing by the CORRECTED thrust instead -- v_h' = v_h sqrt(lambda),
// so x' = x / sqrt(lambda), solved as a fixed point -- restores the
// bookkeeping and changes nothing physical: (M) is then satisfied identically
// for ANY correction factor, including 1. Which is "one equation, two
// unknowns" in different clothes, and the reason the choice cannot be
// deferred to the algebra.
//
// SUMMARY OF PROVENANCE: THE BRANCH STRUCTURE AND lambda(x) ARE MOMENTUM
// THEORY; THE STEP FROM lambda TO A THRUST RATIO IS A STATED MODELLING CHOICE.
//
// ONE MORE THING THE CLOSURE DOES NOT DO: T(w) is not rescaled by air density.
// Physically T = C_T rho n^2 D^4, so a rotor at altitude loses thrust; here
// the curve IS the calibration and rho enters only through (H). A world run
// at a non-standard density therefore gets the right INFLOW behaviour and a
// thrust curve calibrated for whatever density the curve was measured at.
// Stated, not hidden; a density ratio is a one-multiply upgrade whenever a
// consumer needs it.
//
// ---------------------------------------------------------------------------
// 4. GROUND EFFECT (Cheeseman-Bennett), AND D5's "SDF DOUBLE DUTY"
//
//     f_ground(z) = 1 / (1 - (R / (4 z))^2),  clamped to <= 1.25
//
// The classical image-source result: a ground plane at distance z below the
// disc is modelled as a mirrored rotor, whose upwash reduces the real rotor's
// induced velocity, so thrust at fixed RPM rises. It diverges at z = R/4 (the
// image reaches the disc), which is why the clamp exists and is not optional;
// the clamp binds for z < R / (4 sqrt(0.2)) = 0.559 R. At z = 4 R the factor
// is 1.0039, i.e. 0.4% -- "out of ground effect" for every practical purpose.
//
// z IS THE WORLD-SDF DISTANCE AT THE ROTOR STATION (D5: "ground effect from
// the WORLD SDF -- the collision field IS the proximity query"). One eval()
// of the same program static contact tests against; no second representation
// of the ground, no ray cast, no per-world "floor height" parameter. Two
// honest consequences of that reuse:
//
//   * ANY SURFACE COUNTS, NOT JUST THE GROUND. A rotor 0.2 m from a wall gets
//     the same factor as one 0.2 m above the floor. A real wall effect exists
//     but is weaker and differently shaped than the image-source ground
//     effect, so this over-applies near vertical geometry. Accepted: it is
//     one SDF query per rotor and it is the right sign.
//   * IT IS A DISTANCE, NOT AN ALTITUDE. The field reports the nearest
//     surface in ANY direction, so a rotor under an overhang measures the
//     overhang, and a rotor inside the solid (phi <= 0) saturates at the
//     clamp. Same trade.
//
// ---------------------------------------------------------------------------
// 5. THE RPM LAG, AND WHY IT IS NOT EULER
//
// The first-order actuator model is  wdot = (w_cmd - w) / tau.  Over a substep
// of length h with w_cmd held constant the EXACT solution is
//
//     w(t+h) = w_cmd + (w(t) - w_cmd) exp(-h/tau)
//            = w + (w_cmd - w) . alpha,     alpha = 1 - exp(-theta),
//                                           theta = h / tau.
//
// Spec S6 writes the step as w += (w_cmd - w) . dt/tau, which is this same
// update with alpha replaced by its first-order approximation theta. This file
// evaluates alpha exactly, for precisely the reason world/medium.hpp section 3
// gives for the Dryden filters (the in-tree precedent for this choice): h is a
// SCHEDULE decision and tau is a CONFIG decision and nothing couples them.
// Explicit Euler on this ODE is stable only for theta < 2 -- at theta = 3 it
// overshoots the command by 50% and oscillates -- and its effective time
// constant is wrong by O(theta) even when stable. The exact alpha lies in
// [0, 1) for EVERY theta >= 0, so a coarse substep degrades gracefully to "w
// reaches w_cmd this step" instead of ringing. Where both are valid they agree
// to within theta/2 relative, i.e. 2.5% at a 1 kHz substep with a 20 ms rotor
// time constant.
//
// alpha is evaluated cancellation-safely (a Taylor series below theta = 0.25,
// the closed form above it) exactly as medium.cpp's one_minus_exp_neg does,
// so a long time constant does not lose the whole step to the 1 - (1 - eps)
// subtraction.
//
// tau <= 0, or non-finite, means NO LAG: alpha = 1 and w tracks w_cmd within
// the step. That is the honest degenerate answer (a zero time constant IS
// instant tracking) rather than a division by zero.
//
// ---------------------------------------------------------------------------
// 6. THE WRENCH, AND ITS TWO FRAMES
//
// layout.hpp's ruling: force_acc is WORLD frame, torque_acc is BODY frame.
// With `axis` the unit thrust axis and r the body-frame mount offset:
//
//     force_acc  += T . axis_world
//     torque_acc += cross(r, T . axis_body) - spin_dir . Q(w) . axis_body
//
// THE REACTION TORQUE'S SIGN. Q(w) here is the NON-NEGATIVE magnitude of the
// aerodynamic drag torque resisting the rotor's rotation. In steady state the
// motor supplies exactly that torque along the direction of rotation, so by
// reaction the AIRFRAME feels it in the opposite direction -- hence the minus
// sign and the multiplication by spin_dir (+1 for a rotor turning
// right-handed about its own thrust axis). This is why a quadrotor's
// counter-rotating pairs cancel in yaw and why a differential between them
// steers yaw; test_rotor.cpp pins both.
//
// NOT MODELLED: the rotor's own angular momentum. A spinning rotor changing
// speed exerts I_rotor . wdot on the airframe, and a rotating airframe with a
// spinning rotor exerts a gyroscopic couple. Spec S6 puts both on the roadmap
// ("gyroscopic rotor terms ... are per-rotor opt-ins"); the reserved lanes
// hold the rotor polar inertia they need.
//
// ---------------------------------------------------------------------------
// 7. PURITY, DETERMINISM, PARITY
//
// apply_rotors() is a pure function of (bodies, rotors, world, medium, params,
// h) -- it holds no state outside the spans it is handed, allocates nothing,
// reads no clock, and draws no randomness (the only stochastic input is the
// gust the Medium already carries, from its own registered stream). Rows are
// visited in slot order. Two runs on equal inputs produce byte-equal outputs.
//
// The numbered op order in rotor.cpp is the CPU<->GPU parity contract (P1/P2,
// D11) in the same sense integrator.cpp's and contacts.cpp's are: the S6 Slang
// mirror must perform the same operations, in the same order, with the same
// groupings, and with THIS FILE'S OWN series threshold -- kRotorLagSeriesThreshold,
// which is 0.25 and is deliberately NOT medium.cpp's 0.125 even though the two
// files share an identical series (rotor.cpp's note on one_minus_exp_neg
// derives why both are sound and what unifying them would cost). Mirroring the
// wrong file's constant is exactly the kind of near-miss this sentence exists
// to prevent.
//
// SINCE S5 TASK 1 THAT PARITY IS EXACT ACROSS CPU TOOLCHAINS. This file's only
// transcendental is core/fp32_math.hpp's exp32, built from IEEE-mandated
// operations alone; std::sqrt is correctly rounded by mandate. So every
// operation here is one the standard pins, and two conforming CPU
// implementations must agree bit for bit. On the GPU it is banded (TD-14):
// Vulkan's sqrt and division are not correctly rounded. The Slang mirror must
// still mirror exp32 rather than reach for HLSL's exp() intrinsic, which is
// specified only to a relative tolerance. world/medium.cpp's parity note says
// the same thing about the same kernel.
// ===========================================================================

namespace spade::vehicles {

// ---------------------------------------------------------------------------
// Pinned model constants. Changing any of these changes every rotor
// trajectory in the determinism corpus.
// ---------------------------------------------------------------------------

// The rotor's thrust axis in its OWN local frame, before RotorRow::local_orient
// rotates it into the body frame. Local +Y, matching world/sdf.hpp's local-axis
// convention for the cylinder and capsule primitives and Spade's Y-up world
// frame (world/builder.hpp) -- so an identity local_orient is a rotor thrusting
// "up" out of a level airframe, which is the case a reader expects to be free.
inline constexpr glm::vec3 kRotorLocalThrustAxis{0.0f, 1.0f, 0.0f};

// Vortex-ring ONSET, in units of v_h: the normal-working-state root (NWS) is
// used for x >= this. -1 is the classical criterion |V_c| = v_h ("the wake can
// no longer escape downward"). See section 2 for why this is not 0.
inline constexpr float kRotorVrsUpperRatio = -1.0f;

// Windmill-brake boundary, in units of v_h: (WB) is used for x <= this. -2 is
// where (WB)'s square root becomes real, i.e. where momentum theory recovers a
// solution; it is a property of the equations, not a tuning knob.
inline constexpr float kRotorVrsLowerRatio = -2.0f;

// The exact maximum of the piecewise inflow curve: lambda(-1) = (1+sqrt(5))/2,
// the golden ratio. Exposed so a consumer can size thrust headroom, and so
// test_rotor.cpp can check the implemented curve against the analytic bound
// rather than against itself. The implementation does NOT clamp to it -- the
// curve provably cannot exceed it (section 2).
inline constexpr float kRotorInflowFactorMax = 1.61803399f;

// Far-field guard on x = v_axial / v_h, in units of v_h. Beyond this ratio
// lambda is asymptotically 1/|x| on BOTH momentum branches -- below 1e-9, i.e.
// zero as far as fp32 is concerned -- so the curve reports its limit of 0
// instead of evaluating. That is also what keeps the arithmetic total: both
// branches form 0.25*x*x, which overflows fp32 above |x| ~ 3.7e19 and would
// turn the climb branch into -inf + inf = NaN. Nine orders of margin, and the
// answer at the boundary is exact rather than truncated.
inline constexpr float kRotorMaxInflowRatio = 1.0e9f;

// Cheeseman-Bennett ground-effect clamp. The model diverges at z = R/4; this
// bound binds for z < 0.559 R. See section 4.
inline constexpr float kRotorMaxGroundFactor = 1.25f;

// The clamp expressed on the DENOMINATOR of f_ground rather than on f itself:
// 1/(1-q) <= 1.25 exactly when 1-q >= 0.8. Guarding here means the divergent
// quotient is never formed, so z -> R/4 cannot produce an inf on its way to
// being clamped.
inline constexpr float kRotorGroundDenomFloor = 1.0f / kRotorMaxGroundFactor;
static_assert(1.0f / kRotorGroundDenomFloor <= kRotorMaxGroundFactor,
              "the denominator guard must not let f_ground round above the clamp; "
              "if this ever fails, clamp f directly instead");

// Below this step ratio theta = h/tau, alpha = 1 - exp(-theta) is evaluated
// from its Taylor series instead of the closed form (section 5).
//
// The SERIES is medium.cpp's one_minus_exp_neg verbatim; this SWITCH POINT is
// not (medium.cpp uses 0.125). Both sit below x = 0.260, the MEASURED crossover
// at which the truncated series stops being the more accurate branch (0.275 if
// the comparison is made on worst case rather than on the mean). rotor.cpp
// carries the model, shows why the model's own answer of 0.2843 is OPTIMISTIC
// -- it charges the series only its truncation while charging the closed form
// its total error -- and tabulates the measurement that supersedes it. The
// pre-S5 comment quoted 0.314, from that same model under the old std::exp
// assumption.
//
// So both thresholds are sound, and the difference between them is a margin
// choice: 0.125 sits comfortably clear, 0.25 sits 4% from the crossover with a
// 1.37x accuracy advantage. Do not "fix" either to match the other on the
// assumption that one drifted.
inline constexpr float kRotorLagSeriesThreshold = 0.25f;

// ---------------------------------------------------------------------------
// RotorRow -- one rotor. One row of the world-partitioned "rotors" array
// registered via ArenaSet::register_array<RotorRow>, exactly like
// physics/forces.hpp's DragBodyRow: REGISTERED, WORLD-PARTITIONED STATE, so
// it is snapshot-covered automatically (state/registry.hpp's "no unregistered
// state" invariant) and a replay resumes with the identical shaft speeds.
//
// THAT IS NOT OPTIONAL HERE THE WAY IT ARGUABLY IS FOR DragBodyRow. A DragBody
// row is pure parameters -- losing it from a snapshot would lose configuration,
// not history. `omega` is genuine dynamic state with its own time constant, so
// a snapshot that missed it would restore a vehicle whose rotors are at the
// wrong speed and whose next second of flight differs. `omega_cmd` rides in
// the row for the same reason: it is the command in force at the snapshot
// instant, and a replay that dropped it would spin every rotor down.
//
// Five 16-byte rows, following state/layout.hpp's std430 discipline. Like
// DragBodyRow this struct does not live in layout.hpp (that file is frozen by
// Task 6's static_asserts) but carries the same assert battery, and emigrates
// to the generated header with its asserts intact when S6's Slang generator
// takes over.
//
//   row 0  body_slot | enabled | tau        | radius
//   row 1  local_pos | spin_dir
//   row 2  local_orient
//   row 3  omega | omega_cmd | thrust_coeff | torque_coeff
//   row 4  _r0 | _r1 | _r2 | _r3            (reserved; see below)
//
// `body_slot` indexes the SAME world's `bodies` span apply_rotors() is handed
// -- local to that per-world slice, not a global arena slot, exactly as
// DragBodyRow::body_slot is. Both arrays are world-partitioned the same way,
// so a caller building both spans from one world id keeps the indices in
// agreement; this pass never consults ArenaSet or a world id.
//
// `enabled` is active-high (0 == inert), a uint32_t rather than a bool, so a
// zero-filled arena slot is inert by construction -- the same convention
// DragBodyRow::enabled and body_flags::active use, and the reason a fresh slot
// needs no initialization to be safe.
//
// THE RESERVED ROW is spec S6's explicit instruction: "Gyroscopic rotor terms
// and BEMT are per-rotor opt-ins on the roadmap; the param schema reserves
// their fields now so upgrading a model never changes layout." Row 4 is that
// reservation -- rotor polar inertia for the gyroscopic term, and blade
// solidity / lift-curve slope / tip pitch for BEMT (section 3) are four floats.
// They must stay 0 until a task claims them; they are NAMED rather than left
// as implicit tail padding so that every byte of this row belongs to a field.
// ---------------------------------------------------------------------------
struct alignas(kStd430StructAlignment) RotorRow {
    uint32_t body_slot;  // index into the world's `bodies` span, see above
    uint32_t enabled;    // active-high; 0 = inert (skipped, same posture as a freed slot)
    float tau;           // RPM-lag time constant, s. <= 0 means no lag (section 5)
    float radius;        // rotor radius R, m. Sets the disc area A = pi R^2 and the ground-effect scale

    glm::vec3 local_pos;  // r, BODY-frame offset of the rotor hub from the body's COM, m
    float spin_dir;       // +1 right-handed about the thrust axis, -1 the other way; 0 = no reaction torque

    glm::quat local_orient;  // local->body rotation; thrust axis = local_orient * kRotorLocalThrustAxis

    float omega;         // STATE: shaft speed, rad/s, non-negative (see the precondition note below)
    float omega_cmd;     // STATE: commanded shaft speed, rad/s, non-negative; what omega lags toward
    float thrust_coeff;  // k_T in T_static = k_T * omega^2. N s^2, i.e. N per (rad/s)^2
    float torque_coeff;  // k_Q in Q       = k_Q * omega^2. N m s^2. NON-NEGATIVE magnitude (section 6)

    float _r0;  // reserved for the gyroscopic/BEMT opt-ins (spec S6); must stay 0
    float _r1;
    float _r2;
    float _r3;
};

static_assert(std::is_standard_layout_v<RotorRow>, "RotorRow must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<RotorRow>, "RotorRow must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<RotorRow>, "arena slots are never individually destroyed");
static_assert(alignof(RotorRow) == 16, "std430 base alignment");
static_assert(sizeof(RotorRow) == 80, "std430 array stride (five 16-byte rows, no tail pad)");

static_assert(offsetof(RotorRow, body_slot) == 0);
static_assert(offsetof(RotorRow, enabled) == 4);
static_assert(offsetof(RotorRow, tau) == 8);
static_assert(offsetof(RotorRow, radius) == 12);
static_assert(offsetof(RotorRow, local_pos) == 16);
static_assert(offsetof(RotorRow, spin_dir) == 28);
static_assert(offsetof(RotorRow, local_orient) == 32);
static_assert(offsetof(RotorRow, omega) == 48);
static_assert(offsetof(RotorRow, omega_cmd) == 52);
static_assert(offsetof(RotorRow, thrust_coeff) == 56);
static_assert(offsetof(RotorRow, torque_coeff) == 60);
static_assert(offsetof(RotorRow, _r0) == 64);
static_assert(offsetof(RotorRow, _r1) == 68);
static_assert(offsetof(RotorRow, _r2) == 72);
static_assert(offsetof(RotorRow, _r3) == 76);

// Every vec3/quat starts a 16-byte row, which is what the field order buys.
static_assert(offsetof(RotorRow, local_pos) % 16 == 0);
static_assert(offsetof(RotorRow, local_orient) % 16 == 0);

// Named fields account for every byte: no implicit padding, same discipline as
// layout.hpp's BodyState and physics/forces.hpp's DragBodyRow. This is the
// property a byte-wise snapshot comparison cannot reason about otherwise.
static_assert(sizeof(RotorRow::body_slot) + sizeof(RotorRow::enabled) + sizeof(RotorRow::tau) +
                  sizeof(RotorRow::radius) + sizeof(RotorRow::local_pos) + sizeof(RotorRow::spin_dir) +
                  sizeof(RotorRow::local_orient) + sizeof(RotorRow::omega) + sizeof(RotorRow::omega_cmd) +
                  sizeof(RotorRow::thrust_coeff) + sizeof(RotorRow::torque_coeff) + sizeof(RotorRow::_r0) +
                  sizeof(RotorRow::_r1) + sizeof(RotorRow::_r2) + sizeof(RotorRow::_r3) ==
              sizeof(RotorRow),
              "RotorRow has implicit padding: every byte must belong to a named field");

// ===========================================================================
// The reviewable parts, exposed individually.
//
// Same posture as world/medium.hpp exposing the Dryden coefficients: these are
// pure scalar functions of one or two arguments, so they can be checked
// against a high-precision reference AT THE BRANCH BOUNDARIES and at step
// ratios no whole-pass test could practically reach, instead of only being
// observable through a trajectory. They are also what a reader re-deriving
// section 1-5 wants to read first.
//
// All three are TOTAL: no argument, however degenerate, produces a NaN, an
// infinity, or a trap. Each documents its own degenerate answer.
// ===========================================================================

// (H): v_h = sqrt(T / (2 rho A)), A = pi R^2. The hover induced velocity that
// non-dimensionalizes the inflow curve.
//
// Returns 0 -- "no disc loading" -- whenever the disc cannot be loaded:
// non-positive thrust, density or radius, or any non-finite argument.
// rotor_inflow_factor() reads a zero v_hover as "nothing to correct".
[[nodiscard]] float rotor_hover_induced_velocity(float thrust_static, float density, float radius) noexcept;

// f_inflow = lambda(v_axial / v_hover): the piecewise induced-velocity curve of
// section 2, used as the thrust ratio by section 3's closure.
//
// `v_axial` is the component of the rotor station's AIR-RELATIVE velocity
// along the thrust axis -- positive climbing, negative descending. Returns
// exactly 1 (the neutral factor) when v_hover <= 0 or when the ratio is a NaN,
// and exactly 0 once |ratio| reaches kRotorMaxInflowRatio (including an
// infinity): lambda tends to 0 at both ends of the curve, so that is the limit
// rather than a guess. The result lies in [0, kRotorInflowFactorMax] for every
// input.
[[nodiscard]] float rotor_inflow_factor(float v_axial, float v_hover) noexcept;

// f_ground = 1/(1 - (R/(4z))^2) clamped to kRotorMaxGroundFactor: section 4's
// Cheeseman-Bennett factor, with `distance` the WORLD-SDF distance at the
// rotor station (metres, negative inside the solid).
//
// Returns 1 for a non-positive or non-finite radius (no disc, hence no image
// source), and the clamp for a distance at or inside the surface. The result
// lies in [1, kRotorMaxGroundFactor] for every input -- ground effect can only
// ever ADD thrust in this model.
[[nodiscard]] float rotor_ground_factor(float distance, float radius) noexcept;

// alpha = 1 - exp(-h/tau): the exact one-substep fraction of the way from
// omega to omega_cmd (section 5). Returns 0 for a non-positive or non-finite
// `h` (no time passes, nothing changes) and 1 for a non-positive or non-finite
// `tau` (no lag). The result lies in [0, 1] for every input, which is what
// makes the update a contraction at any step size.
[[nodiscard]] float rotor_lag_alpha(float h, float tau) noexcept;

// ---------------------------------------------------------------------------
// apply_rotors() -- the rotor.forces pass's RotorElement contribution.
//
// For every enabled rotor whose body is active: advances that rotor's `omega`
// one substep toward `omega_cmd`, then accumulates the resulting thrust into
// the body's force_acc (WORLD frame) and the moment plus the shaft reaction
// torque into torque_acc (BODY frame).
//
// `bodies` and `rotors` must be spans over the SAME world's partitions (see
// RotorRow::body_slot); `world_sdf`, `medium` and `params` are that world's
// static SDF program, Medium and WorldParams row. The medium is sampled at the
// ROTOR STATION, not at the body origin (spec S6: "Aero force elements read
// Medium at their body-frame station") -- which is the same point as the body
// origin for every Medium that ships today, both being position-independent,
// but it is the seam P7's flow fields need and it costs nothing to get right
// now.
//
// `rotors` is MUTABLE, unlike apply_drag()'s element span: this pass owns the
// shaft-speed state and writes it back into the arena row field-wise.
//
// SKIPS ARE TOTAL AND BYTE-EXACT. A disabled rotor (`enabled == 0`) and a
// rotor whose body lacks body_flags::active are both skipped ENTIRELY --
// including the omega update, so such a rotor is frozen rather than
// free-running. That matches integrate_bodies() leaving an inactive body
// byte-for-byte untouched: an inert slot must not evolve.
//
// Allocates nothing, reads no clock, draws no randomness; safe to call every
// substep from a hot loop. `h` is the effective substep dt (dt / substeps),
// the same quantity integrate_bodies() takes -- this function does not own a
// substep loop.
//
// PRECONDITIONS, documented rather than checked -- the same posture
// physics/contacts.hpp states for its own parameters, because this is the
// physics inner loop:
//   * every `rotors[i].body_slot` is a valid index into `bodies`;
//   * `world_sdf` has passed SdfProgram::validate();
//   * `local_orient` is a unit quaternion on any ENABLED row (a zero-filled
//     row's quaternion is the degenerate all-zeros, which is exactly why a
//     zero-filled row is also disabled);
//   * `omega` and `omega_cmd` are non-negative. Both curves are even in
//     omega, so a negative shaft speed would produce the thrust and torque of
//     its magnitude while the lag drove it further negative -- not a
//     configuration this model has meaning for. `spin_dir`, not the sign of
//     omega, is what says which way a rotor turns;
//   * `spin_dir` is +1, -1 or 0. It is a DIRECTION, not a scale: 2 would
//     double the yaw torque without doubling thrust;
//   * `radius` > 0. THIS ONE FAILS QUIETLY AND IS THE ONE A MODEL AUTHOR WILL
//     ACTUALLY HIT: radius == 0 makes v_h == 0 (no disc area) so f_inflow
//     returns its neutral 1, AND leaves no image source so f_ground returns 1
//     as well. The result is a rotor that produces exactly its static thrust
//     in climb, in descent, and on the ground, with nothing anywhere
//     reporting a problem -- a plausible-looking vehicle with both aero
//     corrections silently switched off. Task 18's parameter validation
//     should reject a non-positive radius on an enabled row; this pass, like
//     every other physics inner loop in the tree, does not check.
// ---------------------------------------------------------------------------
void apply_rotors(std::span<BodyState> bodies, std::span<RotorRow> rotors, const SdfProgram& world_sdf,
                  const Medium& medium, const WorldParams& params, float h) noexcept;

}  // namespace spade::vehicles
