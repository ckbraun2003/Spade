// ---------------------------------------------------------------------------
// dryden.slang (S6 Task 8) -- THE SLANG PORT of engine/world/medium.cpp's
// Dryden gust filter: the three cancellation-prone coefficient primitives, the
// step ratio and its clamp, the first- and second-order coefficient sets, the
// per-substep advance, and the normalized-state -> gust-velocity output.
//
// A MODULE, NOT A KERNEL, exactly like fp32_math.slang and sdf_eval.slang: it
// declares no entry point and reaches SPIR-V only through the kernels that
// import it. THREE do, and the split is the CPU's own:
//
//     kernels/medium_update.slang  dryden_advance()      -- the MediumUpdate pass
//     kernels/forces_drag.slang    dryden_turbulence()   -- the medium SAMPLE the
//     kernels/rotors.slang         dryden_turbulence()      force passes read
//
// That is physics/schedule.cpp's own arrangement: pass_medium_update() advances
// the filter once per world per substep, and pass_force_elements() then
// constructs a DrydenMedium over the ALREADY-ADVANCED row and samples it. The
// recorded dispatch chain preserves the order (MediumUpdate is slot 0,
// ForceElements slot 1, with a memory barrier between every adjacent pair), so
// the gust a force element reads on the device is the same substep's gust the
// CPU reads.
//
// ===========================================================================
// PARITY: medium.cpp's op order IS the contract, and this file is its
// transcription. That file says so itself -- "Everything below is fp32, and the
// op order is the CPU<->GPU parity contract in the same sense integrator.cpp's
// is (P1/P2, D11). The Slang mirror must perform the same operations, in the
// same order, with the same groupings and the same series thresholds." Every
// function below cites the medium.cpp lines it reproduces.
//
// ===========================================================================
// THE exp32 SITES: FIVE PORTED, TWO CORPUS-LIVE. Stated precisely, because
// "port all five" and "five are exercised" are different claims and only the
// first is true:
//
//     medium.cpp:82    one_minus_exp_neg's closed form   (x >= 0.125)
//     medium.cpp:99    dryden_q00's closed form          (t >= 0.25)
//     medium.cpp:119   dryden_q11's closed form          (t >= 0.25)
//     medium.cpp:218   dryden_first_order_coeffs, phi = exp(-theta)   <- LIVE
//     medium.cpp:228   dryden_second_order_coeffs, e = exp(-theta)    <- LIVE
//
// ALL FIVE ARE PORTED. Only the last two are reached by any committed corpus
// scenario, and the reason is structural rather than lucky: theta = h*V/L, and
// at the corpus's step sizes with the default scale lengths and reference
// airspeed theta is of order 1e-3 to 1e-2 -- far below BOTH the 0.125 threshold
// in one_minus_exp_neg and the 0.25 threshold in dryden_q00/dryden_q11 -- so
// those three functions always take their TAYLOR SERIES branch and their
// exp32 calls are never evaluated. The two unconditional ones at :218 and :228
// run on every world on every substep.
//
// The three unreached calls are ported anyway, and must be: a scenario with a
// coarser step or a shorter scale length crosses those thresholds, and a branch
// that exists on one backend and not the other is exactly the silent divergence
// this program exists to prevent. tests/test_dryden.cpp sweeps theta from 1e-6
// to 32 on the CPU side and pins both branches of each; the GPU side inherits
// the same expressions verbatim.
//
// ===========================================================================
// DIVISION AND SQRT SITES, each against the CPU line it mirrors:
//   DIV   (h * V) / L                       (medium.cpp:211, dryden_step_ratio)
//         q01 / c.l00                       (medium.cpp:250, the Cholesky)
//   SQRT  sqrt(one_minus_exp_neg(2 theta))  (medium.cpp:222)
//         sqrt(q00)                         (medium.cpp:249)
//         sqrt(l11_sq)                      (medium.cpp:252)
// Downstream modules MAY use both (global constraint); this is part of why
// CPU<->GPU is banded rather than bit-identical. rng.slang's own sqrt (inside
// next_gauss) is the other contributor on this path.
//
// NO WORKGROUP-SHARED ANYTHING: every function here is pure per-thread
// arithmetic over a caller's local, so T9's workgroup_size invariance holds by
// construction.
// ---------------------------------------------------------------------------

import layouts;
import fp32_math;
import rng;

// world/medium.hpp:309 -- sqrt(3) in fp32, spelled with the SAME decimal text
// the CPU spells it with so both sides round one literal rather than two
// approximations of it.
static const float kSqrt3 = 1.73205080756887729353;

// world/medium.hpp:319 -- the upper clamp on theta = h/tau. Anti-overflow
// hygiene, not a stability condition: at theta = 32, exp(-theta) ~ 1.3e-14 and
// the process is already white noise as far as fp32 can tell.
static const float kDrydenMaxStepRatio = 32.0f;

// ---------------------------------------------------------------------------
// 1 - exp(-x), for x >= 0 -- medium.cpp:74-83.
//
// THE THRESHOLD IS 0.125 IN THIS FILE AND 0.25 IN rotors.slang, and the two
// MUST NOT be "unified". medium.cpp and vehicles/rotor.cpp carry the same six
// Horner terms with two different switch points, both measured, and rotor.cpp's
// note spells out what promoting either verbatim would cost: it "CHANGES THE
// OTHER FILE'S NUMBERS for x in [0.125, 0.25), where one file would switch
// branches (order 1e-7 relative)", and Dryden's outputs are covered by the
// golden determinism corpus. So the duplication is deliberate on the CPU side
// and is mirrored deliberately here -- rotors.slang has its own copy with its
// own threshold, and a comment at each site saying why.
// ---------------------------------------------------------------------------
float dryden_one_minus_exp_neg(float x) {
    if (x < 0.125f) {
        // x - x^2/2 + x^3/6 - x^4/24 + x^5/120 - x^6/720
        return x * (1.0f -
                    x * (1.0f / 2.0f -
                         x * (1.0f / 6.0f -
                              x * (1.0f / 24.0f - x * (1.0f / 120.0f - x * (1.0f / 720.0f))))));
    }
    return 1.0f - exp32(-x);
}

// Q00(t) = 1 - exp(-2t)(1 + 2t + 2t^2) -- medium.cpp:89-101. Leading term
// (4/3)t^3: the catastrophic cancellation medium.hpp section 3 documents.
float dryden_q00(float t) {
    if (t < 0.25f) {
        // (4/3)t^3 - 2t^4 + (8/5)t^5 - (8/9)t^6 + (8/21)t^7 - (2/15)t^8 + (16/405)t^9
        return t * t * t *
               (4.0f / 3.0f -
                t * (2.0f -
                     t * (8.0f / 5.0f -
                          t * (8.0f / 9.0f -
                               t * (8.0f / 21.0f - t * (2.0f / 15.0f - t * (16.0f / 405.0f)))))));
    }
    const float e2 = exp32(-2.0f * t);
    return 1.0f - e2 * (1.0f + 2.0f * t + 2.0f * t * t);
}

// Q11(t) = 1 - exp(-2t)(1 - 2t + 2t^2) -- medium.cpp:107-121. Leading term 4t,
// so the cancellation is mild rather than fatal; the series is still the better
// branch below the threshold.
float dryden_q11(float t) {
    if (t < 0.25f) {
        // 4t - 8t^2 + (28/3)t^3 - (22/3)t^4 + (64/15)t^5 - (88/45)t^6
        //   + (232/315)t^7 - (74/315)t^8
        return t * (4.0f -
                    t * (8.0f -
                         t * (28.0f / 3.0f -
                              t * (22.0f / 3.0f -
                                   t * (64.0f / 15.0f -
                                        t * (88.0f / 45.0f -
                                             t * (232.0f / 315.0f - t * (74.0f / 315.0f))))))));
    }
    const float e2 = exp32(-2.0f * t);
    return 1.0f - e2 * (1.0f - 2.0f * t + 2.0f * t * t);
}

// medium.cpp:129-133 -- the shared domain guard. Written as POSITIVE tests so a
// NaN (which fails every comparison) lands on 0, i.e. a frozen field, rather
// than propagating into registered state. Without the upper clamp, theta == inf
// would make Q01 = 2 t^2 exp(-2t) evaluate as inf * 0 = NaN.
float dryden_clamped_theta(float theta) {
    if (!(theta > 0.0f)) return 0.0f;
    if (!(theta < kDrydenMaxStepRatio)) return kDrydenMaxStepRatio;
    return theta;
}

// ---------------------------------------------------------------------------
// medium.cpp:199-212 -- theta = h V / L, computed as (h * V) / L, with the
// pinned grouping.
//
// THE FINITENESS TEST IS SPELLED AGAINST THE BIT PATTERN, not through a
// library predicate, and the substitution is exact rather than approximate:
// `std::isfinite(x)` is false exactly for the infinities and the NaNs, which
// are exactly the fp32 encodings whose 8 exponent bits are all 1. So
// `(asuint(x) & 0x7F800000) != 0x7F800000` IS std::isfinite, with no dependence
// on how a particular Slang version lowers its own intrinsic. (The NaN half is
// already unreachable here -- a NaN fails the positivity tests on the line
// above -- so this guard's live job is rejecting +infinity, which medium.cpp
// rejects rather than clamps: "an infinite substep or scale length is a broken
// configuration, not a very large one".)
// ---------------------------------------------------------------------------
bool dryden_is_finite(float x) { return (asuint(x) & 0x7F800000u) != 0x7F800000u; }

float dryden_step_ratio(float h, float reference_airspeed, float scale_length) {
    if (!(h > 0.0f) || !(reference_airspeed > 0.0f) || !(scale_length > 0.0f)) return 0.0f;
    if (!dryden_is_finite(h) || !dryden_is_finite(reference_airspeed) ||
        !dryden_is_finite(scale_length)) {
        return 0.0f;
    }
    return dryden_clamped_theta((h * reference_airspeed) / scale_length);
}

// medium.hpp:489-499's two coefficient records. Declared HERE rather than in
// layouts.slang because nothing uploads or downloads one -- they never leave a
// thread's registers, exactly like sdf_eval.slang's SdfSample.
struct DrydenFirstOrderCoeffs {
    float phi;
    float l;
};

struct DrydenSecondOrderCoeffs {
    float phi00;
    float phi01;
    float phi10;
    float phi11;
    float l00;
    float l10;
    float l11;
};

// medium.cpp:214-224.
DrydenFirstOrderCoeffs dryden_first_order_coeffs(float theta_in) {
    const float theta = dryden_clamped_theta(theta_in);

    DrydenFirstOrderCoeffs c;
    c.phi = exp32(-theta);
    // 1 - phi^2 = 1 - exp(-2 theta), evaluated WITHOUT forming phi*phi: the
    // subtraction 1 - phi^2 loses ~log10(1/(2 theta)) digits, and this is the
    // scalar channel's entire variance normalization.
    c.l = sqrt(dryden_one_minus_exp_neg(2.0f * theta));
    return c;
}

// medium.cpp:226-254.
DrydenSecondOrderCoeffs dryden_second_order_coeffs(float theta_in) {
    const float theta = dryden_clamped_theta(theta_in);
    const float e = exp32(-theta);

    DrydenSecondOrderCoeffs c;
    // Phi = exp(-theta) [[1 + theta, theta], [-theta, 1 - theta]] -- exact,
    // because A has a double eigenvalue at -1/tau and (A + I/tau) is nilpotent,
    // so the matrix exponential terminates after two terms.
    c.phi00 = e * (1.0f + theta);
    c.phi01 = e * theta;
    c.phi10 = -c.phi01;
    c.phi11 = e * (1.0f - theta);

    // Q = I - Phi Phi^T. Q01 is written in its closed form because it never
    // cancels -- it is a product, not a difference.
    const float q00 = dryden_q00(theta);
    const float q01 = 2.0f * theta * theta * e * e;
    const float q11 = dryden_q11(theta);

    // Lower Cholesky. The guards are for theta == 0 EXACTLY (a frozen field),
    // where Q is the zero matrix; they are NOT papering over the small-theta
    // conditioning -- dryden_q00's series is what makes l00 correct there.
    c.l00 = sqrt(q00);
    c.l10 = (c.l00 > 0.0f) ? (q01 / c.l00) : 0.0f;
    const float l11_sq = q11 - c.l10 * c.l10;
    c.l11 = (l11_sq > 0.0f) ? sqrt(l11_sq) : 0.0f;
    return c;
}

// ---------------------------------------------------------------------------
// medium.cpp:140-145 -- one substep of a normalized second-order channel.
//
// PINNED OP ORDER: both new states are formed from the OLD pair before either
// is written back, and n0 feeds BOTH rows. That is what the Cholesky factor's
// off-diagonal means -- the two state increments are correlated, and drawing an
// independent normal per state instead would silently change the process into a
// different (and plausible-looking) one.
// ---------------------------------------------------------------------------
void dryden_advance_pair(inout float z0, inout float z1, DrydenSecondOrderCoeffs c, float n0,
                         float n1) {
    const float next0 = c.phi00 * z0 + c.phi01 * z1 + c.l00 * n0;
    const float next1 = c.phi10 * z0 + c.phi11 * z1 + c.l10 * n0 + c.l11 * n1;
    z0 = next0;
    z1 = next1;
}

// ---------------------------------------------------------------------------
// medium.cpp:256-280 -- ONE substep of one world's filter, in place.
//
// FIVE GAUSSIANS, DRAWN UP FRONT, IN THE PINNED ORDER n_u, n_v0, n_v1, n_w0,
// n_w1, UNCONDITIONALLY -- including at turbulence level `none`, where they are
// multiplied by sigma = 0. medium.hpp's determinism note is explicit about why:
// "Skipping the draws would make the stream position depend on the turbulence
// level, so two runs of the same scenario at different levels would diverge in
// every OTHER stochastic system too." The GPU must pay for them too, or the two
// backends' stream positions part company on the first substep.
//
// Drawn UP FRONT rather than per channel so the order cannot be perturbed by a
// later reordering of the three channel updates -- the CPU's own reason.
// ---------------------------------------------------------------------------
void dryden_advance(inout DrydenState state, DrydenParams params, float h) {
    const float n_u = rng_next_gauss(state.stream);
    const float n_v0 = rng_next_gauss(state.stream);
    const float n_v1 = rng_next_gauss(state.stream);
    const float n_w0 = rng_next_gauss(state.stream);
    const float n_w1 = rng_next_gauss(state.stream);

    const float v_ref = params.reference_airspeed;

    const DrydenFirstOrderCoeffs cu =
        dryden_first_order_coeffs(dryden_step_ratio(h, v_ref, params.scale_u));
    state.u = cu.phi * state.u + cu.l * n_u;

    const DrydenSecondOrderCoeffs cv =
        dryden_second_order_coeffs(dryden_step_ratio(h, v_ref, params.scale_v));
    dryden_advance_pair(state.v0, state.v1, cv, n_v0, n_v1);

    const DrydenSecondOrderCoeffs cw =
        dryden_second_order_coeffs(dryden_step_ratio(h, v_ref, params.scale_w));
    dryden_advance_pair(state.w0, state.w1, cw, n_w0, n_w1);
}

// ---------------------------------------------------------------------------
// medium.cpp:282-294 -- normalized states -> gust velocities, world frame.
//
// GROUPING PINNED: the bracket first, then the half, then sigma. The transverse
// output sigma * (z0 + sqrt(3) z1) / 2 has variance sigma^2 because the two
// normalized states are uncorrelated with unit variance (medium.hpp section 2).
//
// AXIS MAPPING: u -> +X, w -> +Y (up), v -> +Z. Note the SWAP -- this returns
// (u, w, v), not (u, v, w) -- which is medium.hpp's axis-mapping note and is
// exactly the sort of thing a "tidy" port gets wrong silently, since all three
// components are gusts of similar magnitude.
// ---------------------------------------------------------------------------
float3 dryden_turbulence(DrydenState state, DrydenParams params) {
    const float u = params.sigma_u * state.u;
    const float v = params.sigma_v * (0.5f * (state.v0 + kSqrt3 * state.v1));
    const float w = params.sigma_w * (0.5f * (state.w0 + kSqrt3 * state.w1));
    return float3(u, w, v);
}

// ---------------------------------------------------------------------------
// The MEDIUM SAMPLE the force passes read -- world/medium.cpp:296-301's
// DrydenMedium::sample(), which is `{params.air_density, params.wind +
// dryden_turbulence(*state_, *params_)}`.
//
// SPELLED ONCE, HERE, rather than twice in forces_drag.slang and rotors.slang:
// both force kernels sample the same medium in the same substep and must agree
// bit for bit about what the air is doing, exactly as both CPU force passes are
// handed ONE DrydenMedium constructed once per world by
// physics/schedule.cpp's pass_force_elements().
//
// POSITION IS NOT A PARAMETER, deliberately -- and that is the CPU's signature
// too: both ConstantMedium::sample and DrydenMedium::sample take `glm::vec3`
// UNNAMED, because this is a FROZEN-FIELD POINT MODEL (medium.hpp section 4).
// rotor.cpp samples at the rotor station and forces.cpp at the body position,
// and today those return the identical value; the seam is kept on the CPU side
// for P7's flow fields, and reproducing an argument that provably cannot change
// the answer would be inventing a difference between the two backends.
// ---------------------------------------------------------------------------
struct MediumSample {
    float density;
    float3 wind;
};

MediumSample dryden_medium_sample(WorldParams params, DrydenState state, DrydenParams turbulence) {
    MediumSample out;
    out.density = params.air_density;
    out.wind = params.wind + dryden_turbulence(state, turbulence);
    return out;
}
