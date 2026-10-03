// ---------------------------------------------------------------------------
// rotors.slang (S6 Task 8) -- the rotor.forces pass (RotorElement forces),
// ported from engine/vehicles/rotor.cpp's apply_rotors(). The op order in that
// file's numbered steps 1-7 IS the parity contract (rotor.hpp's own note:
// "this file is only its evaluation order, which is the CPU<->GPU parity
// contract (P1/P2, D11)"), and every step below cites the lines it mirrors.
//
// ===========================================================================
// IT RUNS BEFORE drag.forces (forces_drag.slang), AND THE ORDER IS A
// NUMERICAL CONTRACT
//
// The schedule runs, per world, rotor.forces before drag.forces:
//
//     vehicles::apply_rotors(...);   // <- this kernel (pass_rotor_forces)
//     apply_drag(...);               // <- forces_drag.slang (pass_drag)
//
// and drag declares `after rotor.forces` because that order is not a
// preference: both accumulate into the same two float accumulators, so the
// order is a numerical contract (sim/standard_modules.cpp). The recorded chain
// follows the schedule -- rotors, barrier, drag -- and the barrier between them
// is what makes drag's read of `force_acc` see this kernel's writes, exactly as
// the barriers order the broad phase's three stages.
//
// ===========================================================================
// ONE THREAD PER BODY, NOT PER ROTOR -- forces_drag.slang's decision, for
// forces_drag.slang's three reasons, and they apply here unchanged: two rotors
// on the same body would read-modify-write the same two float3 accumulators
// (a race), atomics would make the result depend on arrival order (fp addition
// is not associative), and a per-element dispatch is exactly the shape the
// standing kernel-design constraint excludes.
//
// So each thread owns ONE BODY and performs the FIXED-ORDER SERIAL FOLD that
// rule sanctions: it walks its world's rotor partition in ascending slot order
// -- the CPU's own visit order -- accumulating in registers, and stores the two
// accumulators exactly once.
//
// THE ONE THING THAT DIFFERS FROM DRAG: THIS PASS WRITES ITS ELEMENT ROWS.
// `rotor.omega` is STATE (rotor.hpp: "the only state this pass writes into its
// own row, and the reason `rotors` is mutable"), so the fold stores each
// handled rotor's new omega as it goes. That is race-free for the same reason
// the accumulators are: a rotor row belongs to exactly one body, so exactly one
// thread ever touches it -- the thread that owns `rotor.body_slot`.
//
// THAT COVERS THE WRITE. THE FOLD'S READ IS WIDER, AND IS WORTH STATING
// SEPARATELY (S6 hygiene: T8 review M4). Every thread's loop below walks the
// WHOLE world rotor partition, not just the rows it owns -- ownership is
// what `if (rotor.body_slot != local_body) continue;` is FILTERING for,
// which means `rotor = rotors[element_base + e]` (the whole-row read) always
// happens FIRST, before that filter can discard it. So a thread CAN read a
// row another thread is concurrently WRITING -- that other thread's own
// `.omega` store at the bottom of this same loop, for its own matching `e`.
// This is a genuine, unsynchronized read-during-write at the memory level:
// it is harmless rather than absent. The only use a non-owning thread ever
// makes of a row it reads is the `body_slot` equality test that immediately
// discards it -- `omega` and every other field of a possibly-torn read are
// NEVER folded into force_acc/torque_acc, and NEVER written anywhere, by any
// thread, for any row it does not own. A race whose only possible outcome is
// discarded is not a correctness hazard, but it is still a race, and it is
// why this kernel -- unlike forces_drag.slang's purely read-only pass --
// cannot claim "no concurrent access to any byte", only the weaker (and
// sufficient) "no concurrent access whose result is ever used."
//
// A ROTOR WHOSE BODY IS INACTIVE IS LEFT BYTE-FOR-BYTE UNTOUCHED, INCLUDING ITS
// OMEGA, because that body's thread returns before the fold begins. rotor.hpp
// calls this out explicitly -- "A disabled rotor is frozen, not free-running
// (skips are total)" -- and it is a real behavioural claim, not an optimization:
// a lagging rotor left to run would keep integrating toward its command.
//
// ===========================================================================
// THE MEDIUM IS THE REAL ONE NOW (S6 Task 8's whole point for this file). The
// CPU samples the medium at the ROTOR STATION; since module-API stage 3 both
// backends read it from the world's field row, written once per substep in
// Fields and shared with forces_drag.slang, so the two force kernels cannot
// disagree about the air in a substep -- as both CPU passes read the row
// through SampledMedium. Position is not a parameter because
// it is not one on the CPU either (a frozen-field POINT model, medium.hpp
// section 4): rotor.cpp samples at `p_rotor` and forces.cpp at `body.pos`, and
// both return the identical value today. The seam is kept on the CPU side for
// P7's flow fields; reproducing an argument that provably cannot change the
// answer would be inventing a difference between the backends.
//
// ===========================================================================
// GROUND EFFECT USES THE T6 SDF EVALUATOR. rotor.cpp calls `spade::eval()` --
// distance only -- and notes that "sample().distance is bit-identical to
// eval()". sdf_eval.slang ports sample(), not eval(), so this kernel calls
// sdf_sample() and reads `.distance`. THE ANSWER IS THE SAME BITS; the cost is
// the gradient walk it then discards, which for the corpus's rotor scenario
// (quad_hover, whose world has NO SDF geometry at all -- an empty program
// answers kSdfEmptyDistance without evaluating anything) is exactly zero. A
// distance-only walk would be a second copy of the evaluator to keep in step
// with the first, which is the trade this port declines.
//
// ===========================================================================
// DIVISION AND SQRT SITES, each against the CPU line it mirrors:
//   SQRT  sqrt(thrust_static / denom)        (rotor.cpp:185, v_h)
//         sqrt(0.25 x^2 + 1)                 (rotor.cpp:161, NWS)
//         sqrt(0.25 x^2 - 1)                 (rotor.cpp:235, WB)
//   DIV   thrust_static / denom              (rotor.cpp:185)
//         v_axial / v_hover                  (rotor.cpp:198)
//         1 / (0.5 x + s)                    (rotor.cpp:157, NWS conjugate form)
//         1 / (-0.5 x + sqrt(...))           (rotor.cpp:235, WB conjugate form)
//         radius / (4 distance)              (rotor.cpp:264)
//         1 / denom                          (rotor.cpp:269, f_ground)
//         h / tau                            (rotor.cpp:287, the lag ratio)
// Downstream kernels MAY use both (global constraint); this is part of why
// CPU<->GPU is banded rather than bit-identical.
//
// NO WORKGROUP-SHARED ANYTHING: one body per thread, no cross-lane traffic, so
// T9's workgroup_size {32, 64, 128} invariance holds by construction.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import fp32_math;
import mirrors;
import sdf_eval;
import dryden;

// state/arenas.hpp's kInvalidWorld; physics/integrator.hpp's
// body_flags::active. Sentinels, not layouts -- see integrate.slang.
static const uint kInvalidWorld = 0xFFFFFFFFu;
static const uint kBodyFlagActive = 1u;

// ---------------------------------------------------------------------------
// vehicles/rotor.hpp's pinned model constants, each with its own line number.
// "Changing any of these changes every rotor trajectory in the determinism
// corpus" -- that header's own words, and the reason they are transcribed
// individually rather than folded into the expressions that use them.
// ---------------------------------------------------------------------------

// rotor.hpp:426 -- the thrust axis in the rotor's OWN local frame, before
// local_orient rotates it into the body frame. Local +Y, matching Spade's Y-up
// world frame, so an identity local_orient is a rotor thrusting "up".
static const float3 kRotorLocalThrustAxis = float3(0.0f, 1.0f, 0.0f);

static const float kRotorVrsUpperRatio = -1.0f;    // rotor.hpp:431
static const float kRotorVrsLowerRatio = -2.0f;    // rotor.hpp:436
static const float kRotorMaxInflowRatio = 1.0e9f;  // rotor.hpp:452
static const float kRotorMaxGroundFactor = 1.25f;  // rotor.hpp:456

// rotor.hpp:462 -- 1/1.25, spelled as the quotient the CPU spells (a constant
// expression both sides fold to 0.8 exactly; 1.25 is a power-of-two-scaled
// integer, so the division is exact on any conforming implementation).
static const float kRotorGroundDenomFloor = 1.0f / kRotorMaxGroundFactor;

// rotor.hpp:484 -- THIS FILE'S OWN series threshold, and it is deliberately
// NOT dryden.slang's 0.125. Both are measured; rotor.cpp's long note has the
// error model, shows why the model's own crossover of 0.2843 is OPTIMISTIC
// (it charges the series only its truncation while charging the closed form its
// total error), and tabulates the measurement that supersedes it -- the real
// crossover is 0.260 on the mean and 0.275 on the worst case, so both
// thresholds sit below it and both are the better branch where each file hands
// over. DO NOT "fix" either to match the other: promoting one verbatim changes
// the OTHER file's numbers for x in [0.125, 0.25), and Dryden's outputs are
// covered by the golden determinism corpus.
static const float kRotorLagSeriesThreshold = 0.25f;

// glm::pi<float>() -- rotor.cpp:180's disc area. Spelled with the same decimal
// text the CPU's glm constant carries so both sides round one literal.
static const float kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// rotor.cpp:128-136 -- 1 - exp(-x) for x >= 0, without the catastrophic
// cancellation the naive form suffers for small x. The SERIES IS IDENTICAL to
// dryden.slang's (same six Horner terms, same groupings); only the switch point
// differs. See kRotorLagSeriesThreshold above.
// ---------------------------------------------------------------------------
float rotor_one_minus_exp_neg(float x) {
    if (x < kRotorLagSeriesThreshold) {
        return x * (1.0f -
                    x * (1.0f / 2.0f -
                         x * (1.0f / 6.0f -
                              x * (1.0f / 24.0f - x * (1.0f / 120.0f - x * (1.0f / 720.0f))))));
    }
    return 1.0f - exp32(-x);
}

// ---------------------------------------------------------------------------
// rotor.cpp:160-166 -- (NWS): lambda(x) = -x/2 + sqrt(x^2/4 + 1).
//
// EVALUATED IN WHICHEVER OF TWO ALGEBRAICALLY IDENTICAL FORMS DOES NOT CANCEL.
// For x <= 0 both terms of the sum are positive and the direct form is exact to
// a rounding. For x > 0 it is a DIFFERENCE of two nearly equal positives -- by
// x = 1000 the fp32 answer has lost three digits, and above x ~ 8200 the "+1"
// disappears and the expression collapses to exactly 0 instead of 1.2e-4. The
// conjugate form's denominator is a sum of two positives and is never below 1.
// The branch sits at x = 0 because both forms give exactly 1 there.
//
// Spelled ONCE and called from BOTH the climb branch and the vortex-ring band's
// upper endpoint, so the two agree BIT-EXACTLY at x = kRotorVrsUpperRatio
// instead of to within however many ulps a re-spelled constant would cost --
// the CPU's own reason for factoring it out.
// ---------------------------------------------------------------------------
float rotor_nws_lambda(float x) {
    const float s = sqrt(0.25f * x * x + 1.0f);
    if (x > 0.0f) {
        return 1.0f / (0.5f * x + s);
    }
    return -0.5f * x + s;
}

// std::isnan(x), spelled against the BIT PATTERN. A NaN is exactly an encoding
// with all eight exponent bits set and a non-zero significand, so this IS the
// predicate rather than an approximation of it -- and it does not depend on how
// a particular Slang version lowers its own intrinsic. Same reasoning as
// dryden.slang's dryden_is_finite().
bool rotor_is_nan(float x) { return (asuint(x) & 0x7FFFFFFFu) > 0x7F800000u; }

// ---------------------------------------------------------------------------
// rotor.cpp:170-186 -- (H): v_h = sqrt(T / (2 rho A)), A = pi R^2.
//
// EVERY GUARD IS SPELLED !(a > 0) RATHER THAN a <= 0 so a NaN argument takes
// the degenerate exit instead of falling through into a sqrt of a NaN -- the
// same posture contacts.cpp uses for its distance and gradient tests.
// ---------------------------------------------------------------------------
float rotor_hover_induced_velocity(float thrust_static, float density, float radius) {
    if (!(thrust_static > 0.0f) || !(density > 0.0f) || !(radius > 0.0f)) {
        return 0.0f;  // "no disc loading"; the inflow curve reads this as "nothing to correct"
    }
    const float area = kPi * radius * radius;
    const float denom = 2.0f * density * area;
    if (!(denom > 0.0f)) {
        return 0.0f;  // underflow of a tiny radius or density; same answer
    }
    return sqrt(thrust_static / denom);
}

// ---------------------------------------------------------------------------
// rotor.cpp:188-248 -- the piecewise momentum-theory inflow factor, all three
// branches (rotor.hpp section 2).
// ---------------------------------------------------------------------------
float rotor_inflow_factor(float v_axial, float v_hover) {
    // Neutral factor when the curve has no meaning: a rotor developing no
    // induced velocity produces no thrust either, so the factor is never
    // observable -- but it must not be a NaN, because it multiplies a zero that
    // would then poison force_acc.
    if (!(v_hover > 0.0f)) {
        return 1.0f;
    }

    // x = V_c / v_h, the one dimensionless group the whole curve depends on.
    const float x = v_axial / v_hover;

    // A NaN ratio means a NaN was already in the state; the neutral factor
    // keeps this pass from being the thing that spreads it.
    if (rotor_is_nan(x)) {
        return 1.0f;
    }

    // A HUGE ratio is a real (if absurd) limit and lambda tends to 0 at BOTH
    // ends of the curve, so 0 is the answer rather than a guess. This one guard
    // also covers the arithmetic: both branches form 0.25*x*x, which OVERFLOWS
    // fp32 above |x| ~ 3.7e19 and would make the (NWS) branch evaluate
    // -inf + inf = NaN. +-inf itself compares >= here, so it needs no separate
    // test.
    if (abs(x) >= kRotorMaxInflowRatio) {
        return 0.0f;
    }

    if (x <= kRotorVrsLowerRatio) {
        // (WB) windmill brake state, x <= -2, in its CANCELLATION-FREE
        // conjugate form: -x/2 - sqrt(x^2/4 - 1) = 1 / (-x/2 + sqrt(x^2/4 - 1)).
        // The square root is real here by the branch condition; at x == -2 it is
        // exactly zero, so lambda is exactly 1 and the join with the band below
        // is exact.
        return 1.0f / (-0.5f * x + sqrt(0.25f * x * x - 1.0f));
    }
    if (x < kRotorVrsUpperRatio) {
        // Vortex-ring band, -2 < x < -1: a straight line in x from lambda = 1
        // at x = -2 to (NWS)'s value at x = -1. Momentum theory has no solution
        // in here at all; this segment is what CLAMPS (NWS)'s unbounded
        // continuation and lands continuously on (WB).
        const float t = x - kRotorVrsLowerRatio;  // in (0, 1)
        return 1.0f + t * (rotor_nws_lambda(kRotorVrsUpperRatio) - 1.0f);
    }
    // (NWS) normal working state, x >= -1: climb and slow descent.
    return rotor_nws_lambda(x);
}

// ---------------------------------------------------------------------------
// rotor.cpp:250-270 -- Cheeseman-Bennett ground effect from the world SDF.
//
// THE CLAMP IS APPLIED TO THE DENOMINATOR, not to the quotient: the divergent
// quotient is never formed, so z -> R/4 saturates instead of producing an inf
// on its way to a clamp.
// ---------------------------------------------------------------------------
float rotor_ground_factor(float distance, float radius) {
    if (!(radius > 0.0f)) {
        return 1.0f;  // no disc, no image source (and the NaN exit)
    }
    if (!(distance > 0.0f)) {
        // At or inside the surface -- and the NaN exit, since !(NaN > 0) is
        // true. Saturating rather than diverging is the whole reason the clamp
        // exists.
        return kRotorMaxGroundFactor;
    }

    const float ratio = radius / (4.0f * distance);
    const float denom = 1.0f - ratio * ratio;
    if (!(denom > kRotorGroundDenomFloor)) {
        return kRotorMaxGroundFactor;
    }
    return 1.0f / denom;
}

// ---------------------------------------------------------------------------
// rotor.cpp:272-288 -- the RPM-lag coefficient, the exact one-substep solution
// of wdot = (w_cmd - w)/tau (rotor.hpp section 5), NOT the spec's explicit
// Euler step.
// ---------------------------------------------------------------------------
float rotor_lag_alpha(float h, float tau) {
    if (!(h > 0.0f)) {
        return 0.0f;  // no time passes: nothing changes. Also the NaN/negative-h exit.
    }
    if (!(tau > 0.0f)) {
        return 1.0f;  // no lag: omega tracks the command within the step. Also the NaN exit.
    }
    // theta = inf (a denormal tau) gives exp(-inf) = 0, i.e. alpha = 1 -- the
    // same "instant tracking" answer the tau <= 0 exit gives, reached
    // continuously.
    return rotor_one_minus_exp_neg(h / tau);
}

// THE ENTRY POINT IS NOT CALLED `rotors`, and the reason is mechanical: this
// kernel WRITES the `rotors` buffer (bindings.slang binding 7), and Slang
// resolves a global function name and a global shader parameter in the same
// scope -- naming both `rotors` makes every use of the buffer an "ambiguous
// reference" and fails the compile. Every OTHER kernel in this directory names
// its entry point after its file because no such collision exists there; this
// one cannot. It changes nothing on the host side: slangc's direct-to-SPIR-V
// compile names the sole compute entry point "main" regardless of the Slang
// function's name, which is what step_recorder.cpp's
// VkPipelineShaderStageCreateInfo::pName matches (and what
// SlangSpirv.FloatControlsPinned asserts per module).
[shader("compute")]
// LOCAL SIZE COMES FROM THE BUILD, NOT FROM THIS LINE (S6 Task 9b).
// cmake/SpadeSlang.cmake's spade_slang_kernel_variants() compiles this kernel
// once per size in SPADE_SLANG_WORKGROUP_SIZES (= compute/backend.hpp's
// kSupportedWorkgroupSizes), passing -DSPADE_WG=<size>, so that
// BackendDesc::workgroup_size selects a module that was actually compiled for
// it. There is DELIBERATELY no #ifndef fallback: a missing -D must fail the
// build loudly, not silently produce a 64-wide kernel that step_recorder.cpp
// then dispatches on a grid divided by some other number.
[numthreads(SPADE_WG, 1, 1)]
void rotor_forces(uint3 tid : SV_DispatchThreadID) {
    const uint slot = tid.x;
    if (slot >= pass_params.world_count * pass_params.body_capacity) {
        return;
    }

    const uint world = bodies_slot_to_world[slot];
    if (world == kInvalidWorld) {
        return;
    }

    // rotor.cpp:302 -- a rotor whose body is not active contributes nothing AND
    // is not advanced, so an inactive body is skipped whole here. That is what
    // keeps a disabled rotor FROZEN rather than free-running.
    if ((bodies[slot].flags & kBodyFlagActive) == 0u) {
        return;
    }

    // RotorRow::body_slot is WORLD-LOCAL (vehicles/rotor.hpp): it indexes the
    // same world's `bodies` partition, not a global arena slot. Partitions are
    // contiguous and equally sized, so the world-local index of a global slot is
    // exactly this subtraction -- forces_drag.slang's identical derivation.
    const uint local_body = slot - world * pass_params.body_capacity;

    const float4 orient = bodies[slot].orient;
    const float3 body_pos = bodies[slot].pos;
    const float3 body_vel = bodies[slot].vel;
    const float3 body_omega = bodies[slot].omega_body;
    const float h = pass_params.h;

    // physics/schedule.cpp's pass_rotor_forces() -- ONE medium per world per
    // substep: this world's field samples, written in Fields by
    // field_environment and field_dryden (module-API stage 3), with a barrier
    // between. They are the bits dryden_medium_sample() would compute here.
    MediumSample medium;
    medium.density = field_samples[world].density;
    medium.wind = field_samples[world].wind;

    float3 force_acc = bodies[slot].force_acc;
    float3 torque_acc = bodies[slot].torque_acc;
    bool touched = false;

    // The fixed-order serial fold: ascending rotor slot, the CPU's own visit
    // order over this world's `rotors` partition.
    const uint element_base = world * pass_params.element_capacity;
    for (uint e = 0; e < pass_params.element_capacity; ++e) {
        RotorRow rotor = rotors[element_base + e];
        if (rotor.enabled == 0u) continue;             // rotor.cpp:297
        if (rotor.body_slot != local_body) continue;   // this body's rotors only

        // 1. MOUNT GEOMETRY (rotor.cpp:315-318). BOTH axes are kept: the wrench
        //    needs the world one for force_acc and the body one for torque_acc
        //    (state/layout.hpp's frames ruling), and deriving either from the
        //    other by a second rotation would add a rounding the other does not
        //    have.
        const float3 axis_body = glm_quat_rotate(rotor.local_orient, kRotorLocalThrustAxis);
        const float3 axis_world = glm_quat_rotate(orient, axis_body);
        const float3 r = rotor.local_pos;                                  // BODY frame, m
        const float3 p_rotor = body_pos + glm_quat_rotate(orient, r);      // WORLD frame, m

        // 2. RPM LAG (rotor.cpp:330-331). FIRST, so everything below uses THIS
        //    substep's shaft speed -- the order spec S6 states, and the
        //    semi-implicit flavour the rest of the engine uses. This is the only
        //    state this pass writes into its own row.
        const float alpha = rotor_lag_alpha(h, rotor.tau);
        rotor.omega = rotor.omega + (rotor.omega_cmd - rotor.omega) * alpha;

        // 3. THE CURVES (rotor.cpp:338-340). T_static = k_T w^2 and Q = k_Q w^2
        //    share ONE w^2 so the two cannot disagree about the shaft speed by a
        //    rounding.
        const float omega_sq = rotor.omega * rotor.omega;
        const float thrust_static = rotor.thrust_coeff * omega_sq;
        const float torque_mag = rotor.torque_coeff * omega_sq;

        // 4. AXIAL INFLOW (rotor.cpp:357-360). v_axial = axis . (v_body +
        //    w_b x r - wind). The lever-arm term w_b x r is BODY-frame, so it is
        //    rotated to world before the subtraction. glm_dot3 rather than a
        //    dot(): OpDot is forbidden outright (spirv_scan.hpp P2).
        const float3 v_station = body_vel + glm_quat_rotate(orient, glm_cross(body_omega, r));
        const float3 v_rel = v_station - medium.wind;  // world frame
        const float v_axial = glm_dot3(axis_world, v_rel);

        // 5. MOMENTUM-THEORY INFLOW CORRECTION (rotor.cpp:367-368).
        const float v_hover = rotor_hover_induced_velocity(thrust_static, medium.density, rotor.radius);
        const float f_inflow = rotor_inflow_factor(v_axial, v_hover);

        // 6. GROUND EFFECT from the WORLD SDF (rotor.cpp:377-378), D5's "the
        //    collision field IS the proximity query". `.distance` of the T6
        //    evaluator's sample is bit-identical to the CPU's eval(); see this
        //    file's header.
        const float phi = sdf_sample(world, p_rotor).distance;
        const float f_ground = rotor_ground_factor(phi, rotor.radius);

        // 7. THE WRENCH (rotor.cpp:394-398). One scalar thrust, applied along
        //    the WORLD axis for force_acc and along the BODY axis for the
        //    moment. The reaction torque is MINUS spin_dir * Q along the thrust
        //    axis: Q is the non-negative drag-torque magnitude the air applies
        //    to the rotor, the motor supplies it along the direction of
        //    rotation, and the airframe feels the opposite.
        const float thrust = thrust_static * f_inflow * f_ground;
        const float3 f_body = thrust * axis_body;

        force_acc = force_acc + thrust * axis_world;
        torque_acc = torque_acc + (glm_cross(r, f_body) - (rotor.spin_dir * torque_mag) * axis_body);

        // The row's own state, stored as the fold goes. Exactly one thread ever
        // reaches this row (the one owning `rotor.body_slot`), so there is no
        // race and no ordering question -- see the header.
        rotors[element_base + e].omega = rotor.omega;
        touched = true;
    }

    if (touched) {
        bodies[slot].force_acc = force_acc;
        bodies[slot].torque_acc = torque_acc;
    }
}
