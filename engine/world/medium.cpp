#include "world/medium.hpp"

#include <cmath>  // std::sqrt (IEEE-mandated) and std::isfinite; no transcendental left

#include "core/fp32_math.hpp"

namespace spade {

// Verbatim, by contract: v0's whole claim is that sampling the medium at a
// point is indistinguishable from reading the per-world constants, so a caller
// written against Medium today behaves identically to one that read
// WorldParams directly -- and gains turbulence for free when a stateful
// implementation replaces this one. `pos` is unnamed rather than
// [[maybe_unused]] so the "position is ignored" fact is visible in the
// signature itself.
MediumSample ConstantMedium::sample(const WorldParams& params, glm::vec3 /*pos*/) const {
    return MediumSample{params.air_density, params.wind};
}

// ===========================================================================
// Dryden turbulence. Read medium.hpp's section 1-4 note first: the derivation
// lives there, and this file is only its op order.
//
// PARITY. Everything below is fp32, and the op order is the CPU<->GPU parity
// contract in the same sense integrator.cpp's is (P1/P2, D11). The Slang
// mirror must perform the same operations, in the same order, with the same
// groupings and the same series thresholds.
//
// AND, SINCE S5 TASK 1, PARITY HERE IS AN EXACT CLAIM RATHER THAN A TOLERANCE
// ONE. The exponential below is core/fp32_math.hpp's exp32, built from
// IEEE-mandated operations only, and std::sqrt is correctly rounded by
// mandate -- so every operation in this file is one the standard pins, and
// two conforming implementations must agree on all of them bit for bit. The
// Slang mirror inherits that the moment it mirrors exp32 rather than calling
// HLSL's own exp() intrinsic, which is specified to a relative tolerance and
// would put this file straight back where it started.
// ===========================================================================

namespace {

// ---------------------------------------------------------------------------
// The three cancellation-prone coefficient primitives.
//
// Each is "closed form above a pinned threshold, Taylor series below it", and
// the thresholds are chosen where the two branches are equally accurate --
// below the threshold the closed form loses digits to cancellation, above it
// the truncated series loses digits to truncation. Every series is written in
// Horner form so the evaluation cost is one multiply-add per term and no power
// is formed explicitly.
//
// These are the numbers test_dryden.cpp checks against a cancellation-free
// double-precision reference; see medium.hpp section 3 for WHY the naive
// expressions are not usable at the step ratios this engine actually runs at.
// ---------------------------------------------------------------------------

// 1 - exp(-x), for x >= 0. Cancels for small x (the answer is x, the operands
// are both ~1). Series 6 terms.
//
// AT THE 0.125 THRESHOLD, with BOTH branches measured as this file spells them
// against a double -expm1 reference: series 6.7e-8 relative worst case, closed
// form 2.9e-7, so the series is 4.3x better (6.0x on the mean). That is the
// like-for-like comparison; an earlier revision of this note quoted the
// series' TRUNCATION (8e-10) against the closed form's TOTAL error, which
// overstated the margin by two orders of magnitude and was not a comparison of
// two comparable things.
//
// The measured crossover -- where that advantage reaches 1 -- is x = 0.260 on
// the mean and 0.275 on the worst case, so 0.125 sits comfortably clear of it.
//
// vehicles/rotor.cpp's note on its own copy of this kernel carries the full
// error model, the reason the model overstates the series' case, and the
// measurement that supersedes it. It also explains why THIS file's threshold is
// 0.125 while that one's is 0.25, and why neither should be "fixed" to match.
float one_minus_exp_neg(float x) noexcept {
    if (x < 0.125f) {
        // x - x^2/2 + x^3/6 - x^4/24 + x^5/120 - x^6/720
        return x * (1.0f -
                    x * (1.0f / 2.0f -
                         x * (1.0f / 6.0f -
                              x * (1.0f / 24.0f - x * (1.0f / 120.0f - x * (1.0f / 720.0f))))));
    }
    return 1.0f - math::exp32(-x);
}

// Q00(t) = 1 - exp(-2t) (1 + 2t + 2t^2). Leading term (4/3) t^3: the
// catastrophic one (medium.hpp section 3). Series to t^9; at the 0.25
// threshold the next term is ~1e-8 against a value of 0.0144 (~7e-7 relative),
// matching the closed form's cancellation loss there.
float dryden_q00(float t) noexcept {
    if (t < 0.25f) {
        // (4/3)t^3 - 2t^4 + (8/5)t^5 - (8/9)t^6 + (8/21)t^7 - (2/15)t^8 + (16/405)t^9
        return t * t * t *
               (4.0f / 3.0f -
                t * (2.0f -
                     t * (8.0f / 5.0f -
                          t * (8.0f / 9.0f -
                               t * (8.0f / 21.0f - t * (2.0f / 15.0f - t * (16.0f / 405.0f)))))));
    }
    const float e2 = math::exp32(-2.0f * t);
    return 1.0f - e2 * (1.0f + 2.0f * t + 2.0f * t * t);
}

// Q11(t) = 1 - exp(-2t) (1 - 2t + 2t^2). Leading term 4t, so the cancellation
// is mild (about log10(1/(4t)) digits) rather than fatal -- but it is free to
// do it right, and at a 200 Hz substep with tau = 40 s the closed form is
// already down to ~4 significant digits. Series to t^8.
float dryden_q11(float t) noexcept {
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
    const float e2 = math::exp32(-2.0f * t);
    return 1.0f - e2 * (1.0f - 2.0f * t + 2.0f * t * t);
}

// The step-ratio domain guard, shared by dryden_step_ratio() and by both
// coefficient functions so that EVERY entry point into the filter math sees a
// theta in [0, kDrydenMaxStepRatio]. Written as positive tests so a NaN --
// which fails every comparison -- lands on 0 (a frozen field) rather than
// propagating into the state. Without the upper clamp, theta == inf would make
// Q01 = 2 t^2 exp(-2t) evaluate as inf * 0 = NaN.
float clamped_theta(float theta) noexcept {
    if (!(theta > 0.0f)) return 0.0f;
    if (!(theta < kDrydenMaxStepRatio)) return kDrydenMaxStepRatio;
    return theta;
}

// One substep of a normalized second-order channel. Pinned op order: both new
// states are formed from the OLD pair before either is written back, and n0
// feeds both rows (that is what the Cholesky factor's off-diagonal means --
// the two state increments are correlated, and drawing an independent normal
// per state instead would silently change the process).
void advance_pair(float& z0, float& z1, const DrydenSecondOrderCoeffs& c, float n0, float n1) noexcept {
    const float next0 = c.phi00 * z0 + c.phi01 * z1 + c.l00 * n0;
    const float next1 = c.phi10 * z0 + c.phi11 * z1 + c.l10 * n0 + c.l11 * n1;
    z0 = next0;
    z1 = next1;
}

}  // namespace

DrydenParams dryden_params(TurbulenceLevel level) noexcept {
    // The sigma table, derived in medium.hpp's TurbulenceLevel note:
    // sigma_w = 0.1 W_20 with W_20 = 15/30/45 kt, and
    // sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 * 164.042)^0.4
    //                   = sigma_w * 1.5934358.
    // Only the sigmas vary with level; scale lengths and reference airspeed
    // are the defaults DrydenParams already declares. test_dryden.cpp
    // recomputes all six numbers from the standard's constants rather than
    // copying them, so a transcription slip here is a test failure.
    DrydenParams params{};
    switch (level) {
        case TurbulenceLevel::light:
            params.sigma_u = 1.2296011f;
            params.sigma_v = 1.2296011f;
            params.sigma_w = 0.7716667f;
            break;
        case TurbulenceLevel::moderate:
            params.sigma_u = 2.4592023f;
            params.sigma_v = 2.4592023f;
            params.sigma_w = 1.5433333f;
            break;
        case TurbulenceLevel::severe:
            params.sigma_u = 3.6888034f;
            params.sigma_v = 3.6888034f;
            params.sigma_w = 2.3150000f;
            break;
        case TurbulenceLevel::none:
        default:
            // EXACTLY zero, not merely small: "level none is
            // indistinguishable from ConstantMedium" is a bit-for-bit claim.
            break;
    }
    return params;
}

void dryden_init(DrydenState& state, const WorldParams& world) noexcept {
    state.stream = rng::make_stream(world.seed, kDrydenDomainTag, 0);

    // The stationary distribution in normalized coordinates is N(0, I) -- see
    // medium.hpp section 2 -- so this is five independent standard normals and
    // nothing else. Draw order pinned, and deliberately the SAME order
    // dryden_advance() uses.
    state.u = state.stream.next_gauss();
    state.v0 = state.stream.next_gauss();
    state.v1 = state.stream.next_gauss();
    state.w0 = state.stream.next_gauss();
    state.w1 = state.stream.next_gauss();
    state._p0 = 0.0f;
}

float dryden_step_ratio(float h, float reference_airspeed, float scale_length) noexcept {
    // theta = h / tau = h V / L, computed as (h * V) / L -- pinned grouping.
    // The guards are written as positive tests so NaN inputs (which fail every
    // comparison) fall through to 0 rather than propagating. Infinities are
    // rejected here rather than clamped: an infinite substep or scale length
    // is a broken configuration, not a very large one, and reading it as "the
    // largest supported step ratio" would hide the mistake behind plausible
    // white noise.
    if (!(h > 0.0f) || !(reference_airspeed > 0.0f) || !(scale_length > 0.0f)) return 0.0f;
    if (!std::isfinite(h) || !std::isfinite(reference_airspeed) || !std::isfinite(scale_length)) {
        return 0.0f;
    }
    return clamped_theta((h * reference_airspeed) / scale_length);
}

DrydenFirstOrderCoeffs dryden_first_order_coeffs(float theta_in) noexcept {
    const float theta = clamped_theta(theta_in);

    DrydenFirstOrderCoeffs c;
    c.phi = math::exp32(-theta);
    // 1 - phi^2 = 1 - exp(-2 theta), evaluated WITHOUT forming phi*phi: the
    // subtraction 1 - phi^2 loses ~log10(1/(2 theta)) digits, and this is the
    // scalar channel's entire variance normalization.
    c.l = std::sqrt(one_minus_exp_neg(2.0f * theta));
    return c;
}

DrydenSecondOrderCoeffs dryden_second_order_coeffs(float theta_in) noexcept {
    const float theta = clamped_theta(theta_in);
    const float e = math::exp32(-theta);

    DrydenSecondOrderCoeffs c;
    // Phi = exp(-theta) [[1 + theta, theta], [-theta, 1 - theta]] -- exact,
    // because A has a double eigenvalue at -1/tau and (A + I/tau) is
    // nilpotent, so the matrix exponential terminates after two terms.
    c.phi00 = e * (1.0f + theta);
    c.phi01 = e * theta;
    c.phi10 = -c.phi01;
    c.phi11 = e * (1.0f - theta);

    // Q = I - Phi Phi^T (medium.hpp section 3). Q01 is written in its closed
    // form because it never cancels -- it is a product, not a difference.
    const float q00 = dryden_q00(theta);
    const float q01 = 2.0f * theta * theta * e * e;
    const float q11 = dryden_q11(theta);

    // Lower Cholesky. The guards are for theta == 0 exactly (a frozen field),
    // where Q is the zero matrix; they are NOT papering over the small-theta
    // conditioning -- dryden_q00's series is what makes l00 correct there, and
    // if this guard ever fires for a non-zero theta the series has failed.
    c.l00 = std::sqrt(q00);
    c.l10 = (c.l00 > 0.0f) ? (q01 / c.l00) : 0.0f;
    const float l11_sq = q11 - c.l10 * c.l10;
    c.l11 = (l11_sq > 0.0f) ? std::sqrt(l11_sq) : 0.0f;
    return c;
}

void dryden_advance(DrydenState& state, const DrydenParams& params, float h) noexcept {
    // DRAW ORDER, PINNED, and drawn UP FRONT so it cannot be perturbed by a
    // later reordering of the three channel updates: n_u, n_v0, n_v1, n_w0,
    // n_w1. Five per substep, unconditionally -- see medium.hpp's determinism
    // note for why level `none` still pays for them.
    const float n_u = state.stream.next_gauss();
    const float n_v0 = state.stream.next_gauss();
    const float n_v1 = state.stream.next_gauss();
    const float n_w0 = state.stream.next_gauss();
    const float n_w1 = state.stream.next_gauss();

    const float v_ref = params.reference_airspeed;

    const DrydenFirstOrderCoeffs cu =
        dryden_first_order_coeffs(dryden_step_ratio(h, v_ref, params.scale_u));
    state.u = cu.phi * state.u + cu.l * n_u;

    const DrydenSecondOrderCoeffs cv =
        dryden_second_order_coeffs(dryden_step_ratio(h, v_ref, params.scale_v));
    advance_pair(state.v0, state.v1, cv, n_v0, n_v1);

    const DrydenSecondOrderCoeffs cw =
        dryden_second_order_coeffs(dryden_step_ratio(h, v_ref, params.scale_w));
    advance_pair(state.w0, state.w1, cw, n_w0, n_w1);
}

glm::vec3 dryden_turbulence(const DrydenState& state, const DrydenParams& params) noexcept {
    // Normalized states -> gust velocities. The transverse output
    // sigma * (z0 + sqrt(3) z1) / 2 has variance sigma^2 (1/4 + 3/4) because
    // the two normalized states are uncorrelated with unit variance -- see
    // medium.hpp section 2. Grouping pinned: the bracket first, then the
    // half, then sigma.
    const float u = params.sigma_u * state.u;
    const float v = params.sigma_v * (0.5f * (state.v0 + kSqrt3 * state.v1));
    const float w = params.sigma_w * (0.5f * (state.w0 + kSqrt3 * state.w1));

    // u -> +X, w -> +Y (up), v -> +Z. See medium.hpp's axis-mapping note.
    return glm::vec3(u, w, v);
}

MediumSample DrydenMedium::sample(const WorldParams& params, glm::vec3 /*pos*/) const {
    // Position ignored, exactly as ConstantMedium ignores it: this is a
    // frozen-field POINT model (medium.hpp section 4). `pos` is unnamed so
    // that fact is visible in the signature.
    return MediumSample{params.air_density, params.wind + dryden_turbulence(*state_, *params_)};
}

}  // namespace spade
