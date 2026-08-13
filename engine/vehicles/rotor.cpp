#include "vehicles/rotor.hpp"

#include <cmath>  // std::sqrt (IEEE-mandated), std::isnan, std::fabs

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/fp32_math.hpp"
#include "physics/integrator.hpp"  // body_flags::active

// ---------------------------------------------------------------------------
// RotorElement -- the op order. Read rotor.hpp's sections 1-7 first: the
// derivation lives there and this file is only its evaluation order, which is
// the CPU<->GPU parity contract (P1/P2, D11).
// ---------------------------------------------------------------------------

namespace spade::vehicles {
namespace {

// 1 - exp(-x) for x >= 0, evaluated without the catastrophic cancellation the
// naive form suffers for small x (fl(exp(-x)) is within an ulp of 1, so the
// subtraction keeps only ~log10(1/x) of the digits).
//
//     1 - e^-x = x(1 - x(1/2 - x(1/6 - x(1/24 - x(1/120 - x/720)))))
//
// to x^6; at this file's 0.25 threshold the first dropped term is 1.2e-8
// absolute against a value of 0.2212, i.e. 5.5e-8 relative -- under an fp32
// ulp of 1. END TO END, with BOTH branches evaluated exactly as this file
// spells them and measured against a double -expm1 reference, the series is
// 1.37x better than the closed form at 0.25 (worst case over the surrounding
// floats; 1.28x on the mean). NOT the 2.5x-7.4x an earlier revision of this
// note claimed: that figure compared the series' TRUNCATION against the closed
// form's TOTAL error, which is not a comparison of like with like. The same
// correction applies to the crossover derivation below.
//
// RELATIONSHIP TO medium.cpp's one_minus_exp_neg, STATED EXACTLY, because a
// reader will reach for it and because rotor.hpp's parity note makes this
// threshold part of the S6 Slang contract:
//
//     THE SERIES IS IDENTICAL -- same six terms, same Horner nesting, same
//     groupings. THE SWITCH POINT IS NOT: medium.cpp uses 0.125, this file
//     uses 0.25 (kRotorLagSeriesThreshold). Two-fold apart, and deliberately
//     left that way.
//
// NEITHER IS WRONG, and the arithmetic says why. Both errors are stated as
// ABSOLUTE errors on the value, which is the only way to compare them without
// tripping over the relative/ulp conversion: for every x in the interesting
// band exp(-x) lies in (0.5, 1), a single binade, where one ulp is 2^-24 =
// 5.9605e-8 flat.
//
//   * SERIES: absolute truncation x^7/5040 (alternating, so that is an upper
//     bound on the tail, not just its first term).
//   * CLOSED FORM: exactly the exponential's own absolute error E, because the
//     subtraction 1 - e is EXACT by Sterbenz while e >= 0.5, i.e. for every
//     x <= ln 2 = 0.693 -- which covers both thresholds and the whole
//     crossover band with room to spare.
//
// They cross where x^7/5040 = E, i.e. at x* = (5040 E)^(1/7).
//
// RE-DERIVED FOR exp32 (S5 Task 1; this block previously assumed std::exp).
// The old derivation used a single E of one ulp -- a fair reading of "the
// platform libm is accurate to about an ulp" -- and got x* = 0.314. exp32's
// bound is not a single number, so neither is the crossover:
//
//     E = 0.5 ulp = 2.98e-8   ->  x* = 0.2843   exp32 correctly rounded. NOT a
//                                               bound -- a convention for "the
//                                               typical case", and note that
//                                               exp32's headline 99.19%
//                                               correctly-rounded rate is over
//                                               the WHOLE float line, dominated
//                                               by near-zero arguments; over
//                                               this band specifically (-x for
//                                               x in [0.10, 0.40]) it is 86.8%.
//     E = 1.0 ulp = 5.96e-8   ->  x* = 0.3139   <- the old std::exp number
//     E = 1.5 ulp = 8.94e-8   ->  x* = 0.3326   exp32 worst case, and the only
//                                               one of the three that is a
//                                               genuine BOUND (exp32 is within
//                                               1 ulp of the correctly rounded
//                                               result, itself within 0.5)
//
// So the MODEL's crossover is a band, [0.2843, 0.3326], and the old single
// value sits inside it.
//
// BUT THE MODEL IS OPTIMISTIC ABOUT THE SERIES, AND THE MEASUREMENT SAYS SO
// (review round 1). x^7/5040 is the series' TRUNCATION only; the Horner
// evaluation has rounding of its own, six multiply-adds' worth, which the model
// omits entirely while charging the closed form its full error. Measuring BOTH
// branches as this file spells them, against a double -expm1 reference, over
// every float in each bin:
//
//     x       series rel      closed rel     series rel     closed rel
//             (max)           (max)          (mean)         (mean)
//     0.125    6.74e-8         2.92e-7        1.1e-8         6.6e-8    <- medium.cpp
//     0.25     1.33e-7         1.82e-7        5.6e-8         7.2e-8    <- this file
//     0.260    1.48e-7         1.80e-7        7.1e-8         7.1e-8    <- MEAN crossover
//     0.275    1.80e-7         1.73e-7        1.00e-7        6.8e-8    <- MAX crossover
//     0.3326   ~3.2e-7         ~1.6e-7        ~2.6e-7        ~6.3e-8
//
// THE REAL CROSSOVER IS 0.260 (mean basis) TO 0.275 (max basis) -- BELOW the
// model band's lower edge of 0.2843. The model is therefore a useful skeleton
// and an unsafe bound, and what follows is anchored on the measurement.
//
// NEITHER CONSTANT MOVES: 0.125 and 0.25 are both below 0.260, so the series is
// still the better branch where each file hands over, on both bases. But the
// margin at 0.25 is 4%, not the 14% the model suggested, and the advantage
// there is 1.37x rather than the 4x-plus the model's mismatched comparison
// implied. 0.25 really is close to optimal -- closer than anyone intended.
//
// (The pre-Task-1 version of this table had a 0.314 row reading 1.4e-7 / 1.7e-7
// under the truncation model. Those two cells were wrong even on their own
// terms -- recomputing gives 2.21e-7 and 2.21e-7, which do cross, as that
// derivation says they must.)
//
// 0.25 is nearer the optimum; 0.125 is the wider margin. test_rotor.cpp checks
// both points against a double reference AND pins the MEASURED crossover, so
// this is a verified claim rather than an asserted one.
//
// WHAT THAT COSTS THE "unify later" TICKET (deviation C4) -- spelled out so
// nobody mistakes it for a pure refactor: promoting EITHER kernel verbatim
// CHANGES THE OTHER FILE'S NUMBERS for x in [0.125, 0.25), where one file
// would switch branches (order 1e-7 relative). Dryden's outputs are covered by
// the golden determinism corpus, so unifying means moving that corpus, or
// carrying the threshold as a parameter. The duplication itself remains the
// deliberate choice it was: both are file-local numerical kernels, and
// promoting one to core/math_ops is a shared-file change this lane avoids
// while sibling lanes are in flight.
[[nodiscard]] float one_minus_exp_neg(float x) noexcept {
    if (x < kRotorLagSeriesThreshold) {
        return x * (1.0f -
                    x * (1.0f / 2.0f -
                         x * (1.0f / 6.0f -
                              x * (1.0f / 24.0f - x * (1.0f / 120.0f - x * (1.0f / 720.0f))))));
    }
    return 1.0f - math::exp32(-x);
}

// (NWS): lambda(x) = -x/2 + sqrt(x^2/4 + 1), the normal-working-state root of
// lambda^2 + x lambda - 1 = 0 (rotor.hpp section 2).
//
// Spelled once and called from BOTH the climb/slow-descent branch and the
// vortex-ring band's upper endpoint, so the two agree BIT-EXACTLY at
// x = kRotorVrsUpperRatio instead of to within however many ulps a re-spelled
// constant would cost.
//
// EVALUATED IN WHICHEVER OF TWO ALGEBRAICALLY IDENTICAL FORMS DOES NOT CANCEL
// -- the same discipline medium.cpp applies to the Dryden coefficients, and
// for the same reason (a quietly wrong answer in a regime nobody plots).
// For x <= 0 both terms of the sum are positive and the direct form is exact
// to a rounding. For x > 0 it is a DIFFERENCE of two nearly equal positives:
// by x = 1000 the fp32 answer has lost three digits, and above x ~ 8200 the
// "+1" disappears entirely and the whole expression collapses to exactly 0
// instead of 1.2e-4. The conjugate form
//
//     -x/2 + sqrt(x^2/4 + 1) = 1 / (x/2 + sqrt(x^2/4 + 1))
//
// has a denominator that is a sum of two positives and never below 1, so it
// cancels nowhere. The branch sits at x = 0 because both forms give exactly 1
// there.
[[nodiscard]] float normal_working_state_lambda(float x) noexcept {
    const float s = std::sqrt(0.25f * x * x + 1.0f);
    if (x > 0.0f) {
        return 1.0f / (0.5f * x + s);
    }
    return -0.5f * x + s;
}

}  // namespace

float rotor_hover_induced_velocity(float thrust_static, float density, float radius) noexcept {
    // (H): v_h = sqrt(T / (2 rho A)), A = pi R^2.
    //
    // Every guard is spelled !(a > 0) rather than a <= 0 so a NaN argument
    // takes the degenerate exit instead of falling through into a sqrt of a
    // NaN -- the same posture contacts.cpp uses for its distance and gradient
    // tests.
    if (!(thrust_static > 0.0f) || !(density > 0.0f) || !(radius > 0.0f)) {
        return 0.0f;  // "no disc loading"; rotor_inflow_factor() reads this as "nothing to correct"
    }
    const float area = glm::pi<float>() * radius * radius;
    const float denom = 2.0f * density * area;
    if (!(denom > 0.0f)) {
        return 0.0f;  // underflow of a tiny radius or density; same answer
    }
    return std::sqrt(thrust_static / denom);
}

float rotor_inflow_factor(float v_axial, float v_hover) noexcept {
    // Neutral factor when the curve has no meaning: a rotor developing no
    // induced velocity (omega == 0, zero radius, vacuum) produces no thrust
    // either, so the factor is never observable -- but it must not be a NaN,
    // because it multiplies a zero that would then poison force_acc.
    if (!(v_hover > 0.0f)) {
        return 1.0f;
    }

    // x = V_c / v_h, the one dimensionless group the whole curve depends on.
    const float x = v_axial / v_hover;

    // TOTALITY, and why the two exits differ. A NaN ratio means a NaN was
    // already in the state; the neutral factor keeps this pass from being the
    // thing that spreads it.
    if (std::isnan(x)) {
        return 1.0f;
    }

    // A HUGE ratio is a real (if absurd) limit -- |V_c| overwhelming v_h --
    // and lambda tends to 0 at BOTH ends of the curve (it is asymptotically
    // 1/|x| on (NWS) and on (WB) alike), so 0 is the answer rather than a
    // guess. This one guard also covers the arithmetic: both branches form
    // 0.25 * x * x, which OVERFLOWS fp32 above |x| ~ 3.7e19 and would make the
    // (NWS) branch evaluate -inf + inf = NaN, and +-inf itself compares >=
    // here so it needs no separate test. kRotorMaxInflowRatio sits nine orders
    // below the overflow point, where lambda is already below 1e-9 -- i.e. the
    // guard is exact to fp32, not a truncation.
    if (std::fabs(x) >= kRotorMaxInflowRatio) {
        return 0.0f;
    }

    // The three branches of rotor.hpp section 2.
    if (x <= kRotorVrsLowerRatio) {
        // (WB) windmill brake state, x <= -2, in its CANCELLATION-FREE
        // conjugate form (same reasoning as normal_working_state_lambda):
        //
        //     -x/2 - sqrt(x^2/4 - 1) = 1 / (-x/2 + sqrt(x^2/4 - 1))
        //
        // The direct difference subtracts two nearly equal positives and loses
        // everything for large |x| -- at x = -1e4 the fp32 answer is exactly 0
        // instead of 1e-4 -- while this denominator is a sum of two
        // non-negative terms and is never below 1.
        //
        // The square root is real here by the branch condition; at x == -2 it
        // is exactly zero, so lambda is exactly 1 and the join with the band
        // below is exact.
        return 1.0f / (-0.5f * x + std::sqrt(0.25f * x * x - 1.0f));
    }
    if (x < kRotorVrsUpperRatio) {
        // Vortex-ring band, -2 < x < -1: a straight line in x from lambda = 1
        // at x = -2 to (NWS)'s value at x = -1. Momentum theory has no
        // solution in here at all (the wake recirculates through the disc);
        // this segment is what CLAMPS (NWS)'s unbounded continuation and lands
        // continuously on (WB).
        const float t = x - kRotorVrsLowerRatio;  // in (0, 1)
        return 1.0f + t * (normal_working_state_lambda(kRotorVrsUpperRatio) - 1.0f);
    }
    // (NWS) normal working state, x >= -1: climb and slow descent.
    return normal_working_state_lambda(x);
}

float rotor_ground_factor(float distance, float radius) noexcept {
    // No disc, no image source (and the NaN exit).
    if (!(radius > 0.0f)) {
        return 1.0f;
    }
    // At or inside the surface -- and the NaN exit, since !(NaN > 0) is true.
    // Saturating rather than diverging is the whole reason the clamp exists.
    if (!(distance > 0.0f)) {
        return kRotorMaxGroundFactor;
    }

    // f = 1 / (1 - (R/(4z))^2), with the clamp applied to the DENOMINATOR (see
    // kRotorGroundDenomFloor): the divergent quotient is never formed, so
    // z -> R/4 saturates instead of producing an inf on its way to a clamp.
    const float ratio = radius / (4.0f * distance);
    const float denom = 1.0f - ratio * ratio;
    if (!(denom > kRotorGroundDenomFloor)) {
        return kRotorMaxGroundFactor;
    }
    return 1.0f / denom;
}

float rotor_lag_alpha(float h, float tau) noexcept {
    // No time passes: nothing changes. Also the NaN/negative-h exit.
    if (!(h > 0.0f)) {
        return 0.0f;
    }
    // No lag: omega tracks the command within the step. Also the NaN exit --
    // a zero time constant IS instant tracking, so this is the honest
    // degenerate answer rather than a division by zero.
    if (!(tau > 0.0f)) {
        return 1.0f;
    }
    // alpha = 1 - exp(-theta), theta = h/tau. Total for every theta in
    // [0, inf): theta = inf (a denormal tau) gives exp(-inf) = 0, i.e. alpha
    // = 1, which is the same "instant tracking" answer the tau <= 0 exit
    // gives, reached continuously.
    return one_minus_exp_neg(h / tau);
}

void apply_rotors(std::span<BodyState> bodies, std::span<RotorRow> rotors, const SdfProgram& world_sdf,
                  const Medium& medium, const WorldParams& params, float h) noexcept {
    for (RotorRow& rotor : rotors) {
        // Same skip contract as apply_drag()/integrate_bodies(): an inert row
        // -- freed, tombstoned, never spawned, or explicitly disabled -- is
        // left byte-for-byte untouched, INCLUDING its omega. A disabled rotor
        // is frozen, not free-running (rotor.hpp's "skips are total").
        if (rotor.enabled == 0u) continue;

        // Precondition (documented in rotor.hpp, not runtime-checked, same
        // posture as apply_drag()): body_slot indexes `bodies` directly.
        BodyState& body = bodies[rotor.body_slot];
        if ((body.flags & physics::body_flags::active) == 0u) continue;

        // -------------------------------------------------------------------
        // 1. MOUNT GEOMETRY. local_orient turns the local +Y convention
        //    (kRotorLocalThrustAxis) into a body-frame thrust axis -- this is
        //    the field physics/forces.hpp carried unread as forward-compat for
        //    exactly this element -- and body.orient turns that into world.
        //
        //    BOTH axes are kept. The wrench needs the world one for force_acc
        //    and the body one for torque_acc (layout.hpp's frames ruling), and
        //    deriving either from the other by a second rotation would add a
        //    rounding the other does not have.
        // -------------------------------------------------------------------
        const glm::vec3 axis_body = rotor.local_orient * kRotorLocalThrustAxis;
        const glm::vec3 axis_world = body.orient * axis_body;
        const glm::vec3 r = rotor.local_pos;                    // BODY frame, m
        const glm::vec3 p_rotor = body.pos + body.orient * r;   // WORLD frame, m

        // -------------------------------------------------------------------
        // 2. RPM LAG -- the only state this pass writes into its own row, and
        //    the reason `rotors` is mutable. Exact one-substep solution of
        //    wdot = (w_cmd - w)/tau; see rotor.hpp section 5 for why this is
        //    not the spec's explicit-Euler step.
        //
        //    FIRST, so that everything below uses THIS substep's shaft speed
        //    -- the order spec S6 states, and the semi-implicit flavour the
        //    rest of the engine uses.
        // -------------------------------------------------------------------
        const float alpha = rotor_lag_alpha(h, rotor.tau);
        rotor.omega += (rotor.omega_cmd - rotor.omega) * alpha;

        // -------------------------------------------------------------------
        // 3. THE CURVES. T_static = k_T w^2 and Q = k_Q w^2, sharing one w^2
        //    so the two cannot disagree about the shaft speed by a rounding.
        //    T_static is the STATIC (hover) thrust: what steps 5 and 6 correct.
        // -------------------------------------------------------------------
        const float omega_sq = rotor.omega * rotor.omega;
        const float thrust_static = rotor.thrust_coeff * omega_sq;
        const float torque_mag = rotor.torque_coeff * omega_sq;

        // -------------------------------------------------------------------
        // 4. AXIAL INFLOW. Spec S6: v_axial = axis . (v_body + w_b x r - wind).
        //
        //    The relative-velocity convention is DragBody's exactly (forces.cpp:
        //    velocity minus the Medium's wind at the sample point), extended by
        //    the lever-arm term w_b x r that a rotor mounted away from the COM
        //    genuinely sees -- a rolling airframe drives its rotors up and
        //    down through the air even when the COM is still. That term is
        //    body-frame, so it is rotated to world before the subtraction.
        //
        //    The Medium is sampled at the ROTOR STATION (spec S6: "at their
        //    body-frame station"). Position-independent for every Medium that
        //    ships today, so this is currently identical to sampling at
        //    body.pos -- but it is the seam P7's flow fields need.
        // -------------------------------------------------------------------
        const MediumSample sample = medium.sample(params, p_rotor);
        const glm::vec3 v_station = body.vel + body.orient * glm::cross(body.omega_body, r);
        const glm::vec3 v_rel = v_station - sample.wind;  // world frame
        const float v_axial = glm::dot(axis_world, v_rel);

        // -------------------------------------------------------------------
        // 5. MOMENTUM-THEORY INFLOW CORRECTION (rotor.hpp sections 1-3).
        //    v_h from the static thrust at this substep's shaft speed, then
        //    the piecewise induced-velocity curve at x = v_axial / v_h.
        // -------------------------------------------------------------------
        const float v_hover = rotor_hover_induced_velocity(thrust_static, sample.density, rotor.radius);
        const float f_inflow = rotor_inflow_factor(v_axial, v_hover);

        // -------------------------------------------------------------------
        // 6. GROUND EFFECT from the WORLD SDF (D5's "the collision field IS
        //    the proximity query"). eval() rather than sample(): only the
        //    distance is wanted, and sample().distance is bit-identical to
        //    eval() (world/sdf.hpp) while also walking the gradient this pass
        //    has no use for.
        // -------------------------------------------------------------------
        const float phi = spade::eval(world_sdf, p_rotor);
        const float f_ground = rotor_ground_factor(phi, rotor.radius);

        // -------------------------------------------------------------------
        // 7. THE WRENCH. One scalar thrust, applied along the world axis for
        //    force_acc and along the body axis for the moment, per layout.hpp's
        //    frames ruling.
        //
        //    The reaction torque is MINUS spin_dir * Q along the thrust axis:
        //    Q is the non-negative drag-torque magnitude the air applies to
        //    the rotor, the motor supplies it along the direction of rotation,
        //    and the airframe feels the opposite (rotor.hpp section 6).
        //
        //    cross(r, F_body), matching physics/forces.hpp's identical
        //    spelling for DragBody -- see rotor.hpp's note on spec S6's
        //    reversed operand order.
        // -------------------------------------------------------------------
        const float thrust = thrust_static * f_inflow * f_ground;
        const glm::vec3 f_body = thrust * axis_body;

        body.force_acc += thrust * axis_world;
        body.torque_acc += glm::cross(r, f_body) - (rotor.spin_dir * torque_mag) * axis_body;
    }
}

}  // namespace spade::vehicles
