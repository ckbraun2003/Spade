#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "core/rng.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"
#include "world/medium.hpp"

// ---------------------------------------------------------------------------
// Task 16 -- Dryden turbulence in the Medium.
//
// Three burdens of proof, and they are deliberately carried by different
// kinds of test, because no one kind can carry all three:
//
//   * THE FILTER MATH. Checked ALGEBRAICALLY, against a cancellation-free
//     double-precision reference, over a step-ratio sweep from 1e-6 to the
//     clamp. This is not a stylistic preference: the step ratios this engine
//     actually runs at (theta = h/tau ~ 1e-4 for a 200 Hz substep and a 40 s
//     correlation time) are where the naive coefficient expressions collapse,
//     and a statistical test at theta = 1e-4 would need ~1e9 substeps to
//     resolve a 10% variance error. The coefficient tests reach those step
//     ratios in microseconds and with no sampling error at all.
//
//   * THE PROCESS STATISTICS. Checked by SAMPLING -- variance and
//     autocorrelation over 1e6 substeps -- at a step ratio chosen so the
//     sampling error is small enough for the claim to mean something. See
//     kStatSteps for the effective-sample-size arithmetic.
//
//   * THE DETERMINISM AND SNAPSHOT PROPERTIES. Checked structurally, by the
//     same registry-walk round trip Task 8 uses for rng streams
//     (test_rng_medium.cpp) -- because the claim is about the LAYOUT (all of
//     the state is in the registered row), not about the physics.
//
// All references below are computed in double INSIDE THE TEST. The engine
// itself is fp32 throughout (global constraint); a test is allowed a more
// accurate oracle than the thing it is testing, and here it needs one.
// ---------------------------------------------------------------------------

namespace {

using spade::DrydenParams;
using spade::DrydenState;
using spade::TurbulenceLevel;

// --- doubles-only reference implementations of the discrete coefficients ----
//
// Written to avoid cancellation rather than to look like the formulas: the
// point of the reference is to be right where the fp32 code is hard, so it
// cannot itself be "1 - exp(-2t)(1 + 2t + 2t^2)" in double.
//
//   Q00 = 1 - e^{-2t}(1 + 2t + 2t^2)
//       = -(2t + 2t^2) - expm1(-2t) (1 + 2t + 2t^2)
//
// expm1 is exact for small arguments, so the only cancellation left is between
// two terms of size ~2t producing a result of size (4/3)t^3 -- a loss of
// log10(1.5/t^2) digits, which at t = 1e-5 still leaves ~5 significant digits
// out of double's 16. That is the reason the sweep below starts at 1e-6 for
// the shape check but only claims tight relative accuracy from 1e-5 up.
double ref_q00(double t) {
    const double p = 1.0 + 2.0 * t + 2.0 * t * t;
    return -(2.0 * t + 2.0 * t * t) - std::expm1(-2.0 * t) * p;
}

// Q11 = 1 - e^{-2t}(1 - 2t + 2t^2) = (2t - 2t^2) - expm1(-2t)(1 - 2t + 2t^2).
// Leading term 4t, so this one barely cancels at all.
double ref_q11(double t) {
    const double p = 1.0 - 2.0 * t + 2.0 * t * t;
    return (2.0 * t - 2.0 * t * t) - std::expm1(-2.0 * t) * p;
}

// 1 - e^{-x}.
double ref_one_minus_exp_neg(double x) { return -std::expm1(-x); }

// Q01 = 2 t^2 e^{-2t}. No cancellation; here for symmetry.
double ref_q01(double t) { return 2.0 * t * t * std::exp(-2.0 * t); }

// The step ratios every coefficient test sweeps. Spans six decades below the
// physically interesting band (a 200 Hz substep against tau = L/V of 10-40 s
// puts theta at 1.25e-4 to 5e-4) up through theta ~ 1, where h and tau are
// comparable, to the clamp.
constexpr std::array<double, 22> kThetaSweep{{
    1e-6, 3e-6, 1e-5, 3e-5, 1e-4, 1.25e-4, 3e-4, 5e-4, 1e-3, 3e-3, 1e-2,
    3e-2, 0.1,  0.2,  0.249, 0.25, 0.2501, 0.5,  1.0,  2.0,  8.0,  32.0,
}};

double rel_error(double actual, double expected) {
    if (expected == 0.0) return std::abs(actual);
    return std::abs(actual - expected) / std::abs(expected);
}

// How accurate the DOUBLE reference for Q00 can honestly claim to be at step
// ratio t, and therefore the tightest tolerance this test may assert.
//
// ref_q00 cancels two terms of size ~2t down to a result of size (4/3)t^3, so
// it loses a factor (2/3)t^2 of precision: double's 2.2e-16 becomes
// ~3.3e-16/t^2 relative in Q00, half that in sqrt(Q00), and roughly 1.5x that
// again in l11 (which inherits l10's error amplified by q11 - l10^2). 3e-15/t^2
// covers all three with margin. Below t ~ 1e-5 the fp32 SERIES is the more
// accurate of the two -- which is why the asymptotic check below exists as a
// second, cancellation-free oracle for the deep-small-theta end.
double q00_reference_tolerance(double t) { return std::max(2e-5, 3e-15 / (t * t)); }

// --- statistical-run parameters --------------------------------------------
//
// V = 50 m/s with the default scale lengths gives tau_u = tau_v = 4 s and
// tau_w = 1 s; a 0.05 s substep therefore runs the filters at
// theta_u = 0.0125 and theta_w = 0.05. Those step ratios are chosen for the
// TEST, not because they are typical -- a typical theta of 1.25e-4 would make
// the statistics below meaningless at any affordable sample count, which is
// exactly why the coefficient tests exist.
//
// EFFECTIVE SAMPLE COUNT. 1e6 substeps of a process with correlation time tau
// are not 1e6 independent samples. For a variance estimate over a
// first-order Gauss-Markov process the estimator's relative standard
// deviation is sqrt(2 / (N theta)) to leading order in theta (the sum of
// rho_k^2 over lags contributes a factor 1/theta), so:
//
//     u channel: sqrt(2 / (1e6 * 0.0125)) = 1.3%
//     w channel: sqrt(2 / (1e6 * 0.05  )) = 0.63%
//
// against a 10% tolerance -- roughly 8 and 16 sigma of margin. The transverse
// channels decorrelate faster than a first-order process of the same tau
// (their autocorrelation crosses zero at 2 tau), so their true margin is
// wider still. And the run is SEEDED, so the outcome is not a coin flip
// anyway: these numbers say the tolerance is honest, not that we got lucky.
constexpr int kStatSteps = 1'000'000;
constexpr float kStatSubstep = 0.05f;
constexpr float kStatAirspeed = 50.0f;

DrydenParams stat_params() {
    DrydenParams params = spade::dryden_params(TurbulenceLevel::moderate);
    params.reference_airspeed = kStatAirspeed;
    return params;
}

spade::WorldParams make_world_params(uint64_t seed) {
    spade::WorldParams params{};
    params.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
    params.air_density = 1.225f;
    params.wind = glm::vec3(3.5f, -0.25f, -1.75f);
    params.body_capacity = 8;
    params.body_count = 1;
    params.seed = seed;
    return params;
}

// A snapshot blob in the only form this task assumes: whatever a StateRegistry
// walk hands out, keyed by the array's stable name. Same helpers as
// test_rng_medium.cpp; Task 7 owns the real versioned format and nothing here
// depends on it.
using Blob = std::map<std::string, std::vector<std::byte>>;

Blob save_walk(const spade::StateRegistry& registry) {
    Blob blob;
    registry.for_each_array([&blob](const spade::RegisteredArray& array) {
        blob[array.name].assign(array.data, array.data + array.byte_size());
    });
    return blob;
}

void restore_walk(const spade::StateRegistry& registry, const Blob& blob) {
    registry.for_each_array([&blob](const spade::RegisteredArray& array) {
        const auto it = blob.find(array.name);
        ASSERT_NE(it, blob.end()) << array.name;
        ASSERT_EQ(it->second.size(), array.byte_size()) << array.name;
        std::memcpy(array.data, it->second.data(), it->second.size());
    });
}

std::vector<glm::vec3> run_gusts(DrydenState& state, const DrydenParams& params, float h, int steps) {
    std::vector<glm::vec3> out;
    out.reserve(static_cast<std::size_t>(steps));
    for (int i = 0; i < steps; ++i) {
        spade::dryden_advance(state, params, h);
        out.push_back(spade::dryden_turbulence(state, params));
    }
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Layout and configuration.
// ---------------------------------------------------------------------------

TEST(DrydenLayout, StateIsAPodWithPinnedOffsets) {
    // Restates the header's static_asserts at runtime so a layout regression
    // reads as a named failure with the numbers in the output -- same
    // convention as test_state.cpp and test_rng_medium.cpp.
    EXPECT_EQ(sizeof(DrydenState), 40u);
    EXPECT_EQ(alignof(DrydenState), 8u);
    EXPECT_EQ(offsetof(DrydenState, stream), 0u);
    EXPECT_EQ(offsetof(DrydenState, u), 16u);
    EXPECT_EQ(offsetof(DrydenState, v0), 20u);
    EXPECT_EQ(offsetof(DrydenState, v1), 24u);
    EXPECT_EQ(offsetof(DrydenState, w0), 28u);
    EXPECT_EQ(offsetof(DrydenState, w1), 32u);
    EXPECT_EQ(offsetof(DrydenState, _p0), 36u);
    EXPECT_TRUE(std::is_trivially_copyable_v<DrydenState>);
    EXPECT_TRUE(std::is_standard_layout_v<DrydenState>);

    // fp32 only (global constraint): a double that crept into the filter
    // state or the gust vector would show up here.
    static_assert(std::is_same_v<decltype(DrydenState::u), float>);
    static_assert(std::is_same_v<decltype(DrydenParams::sigma_u), float>);
    static_assert(std::is_same_v<decltype(spade::MediumSample::wind), glm::vec3>);
}

TEST(DrydenLevels, MapTheDocumentedMilF8785CIntensities) {
    // The mapping in medium.hpp's TurbulenceLevel note, recomputed here from
    // the standard's own constants rather than copied from the table -- so
    // this test would catch a transcription error in either direction.
    //
    //   sigma_w = 0.1 * W_20,  W_20 = 15 / 30 / 45 knots
    //   sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h_ft)^0.4, h = 50 m
    const double h_ft = 50.0 / 0.3048;
    const double ratio = 1.0 / std::pow(0.177 + 0.000823 * h_ft, 0.4);
    const double kt = 1852.0 / 3600.0;

    const std::array<std::pair<TurbulenceLevel, double>, 3> cases{{
        {TurbulenceLevel::light, 15.0},
        {TurbulenceLevel::moderate, 30.0},
        {TurbulenceLevel::severe, 45.0},
    }};

    for (const auto& [level, w20_kt] : cases) {
        const double sigma_w = 0.1 * w20_kt * kt;
        const double sigma_u = sigma_w * ratio;
        const DrydenParams params = spade::dryden_params(level);
        EXPECT_NEAR(params.sigma_w, static_cast<float>(sigma_w), 1e-5f);
        EXPECT_NEAR(params.sigma_u, static_cast<float>(sigma_u), 1e-5f);
        EXPECT_EQ(params.sigma_u, params.sigma_v) << "MIL-F-8785C sets sigma_u == sigma_v";

        // The scale lengths and reference airspeed are level-independent.
        EXPECT_EQ(params.scale_u, 200.0f);
        EXPECT_EQ(params.scale_v, 200.0f);
        EXPECT_EQ(params.scale_w, 50.0f);
        EXPECT_EQ(params.reference_airspeed, 5.0f);
    }

    // Level none is EXACTLY zero on all three axes, which is what makes the
    // bit-for-bit ConstantMedium equivalence below possible.
    const DrydenParams none = spade::dryden_params(TurbulenceLevel::none);
    EXPECT_EQ(none.sigma_u, 0.0f);
    EXPECT_EQ(none.sigma_v, 0.0f);
    EXPECT_EQ(none.sigma_w, 0.0f);
}

TEST(DrydenLevels, DefaultScaleLengthsAreTheStandardsValuesAtFiftyMetres) {
    // The claim in medium.hpp: L_u = L_v = 200 m and L_w = 50 m are not
    // arbitrary, they are MIL-F-8785C's low-altitude relations at h = 50 m.
    // L_w = h exactly; L_u = h / (0.177 + 0.000823 h_ft)^1.2 comes out at
    // 202 m, and the default rounds that to 200.
    const double h_m = 50.0;
    const double h_ft = h_m / 0.3048;
    const double l_u = h_m / std::pow(0.177 + 0.000823 * h_ft, 1.2);

    const DrydenParams params{};
    EXPECT_NEAR(params.scale_w, static_cast<float>(h_m), 1e-3f);
    EXPECT_NEAR(params.scale_u, static_cast<float>(l_u), 3.0f) << "L_u = " << l_u << " m at h = 50 m";
    EXPECT_LT(std::abs(params.scale_u - l_u) / l_u, 0.02) << "the rounding to 200 m must stay under 2%";
}

// ---------------------------------------------------------------------------
// The filter coefficients -- the part a statistical test cannot reach.
// ---------------------------------------------------------------------------

TEST(DrydenCoefficients, FirstOrderIsTheExactOrnsteinUhlenbeckDiscretization) {
    // phi = exp(-theta) and l = sqrt(1 - phi^2). The second is the whole
    // variance normalization of the scalar channel, and it is the coefficient
    // that a naive `1.0f - phi*phi` gets wrong by a factor of two or more at
    // small theta.
    for (const double theta : kThetaSweep) {
        const spade::DrydenFirstOrderCoeffs c =
            spade::dryden_first_order_coeffs(static_cast<float>(theta));

        EXPECT_LT(rel_error(c.phi, std::exp(-theta)), 1e-6) << "phi at theta=" << theta;

        const double expect_l = std::sqrt(ref_one_minus_exp_neg(2.0 * theta));
        EXPECT_LT(rel_error(c.l, expect_l), 2e-5) << "l at theta=" << theta << " (got " << c.l
                                                  << ", want " << expect_l << ")";

        // Stationarity, restated as the property rather than the formula:
        // phi^2 + l^2 == 1 is exactly "one step of this recursion maps unit
        // variance to unit variance".
        const double var_next = static_cast<double>(c.phi) * c.phi + static_cast<double>(c.l) * c.l;
        EXPECT_NEAR(var_next, 1.0, 1e-6) << "theta=" << theta;
    }
}

TEST(DrydenCoefficients, SecondOrderTransitionIsTheExactMatrixExponential) {
    // Phi = exp(-theta) [[1+theta, theta], [-theta, 1-theta]], which is exact
    // because A has a double eigenvalue and (A + I/tau) is nilpotent. Also
    // pins the antisymmetry phi10 == -phi01, which is what makes Q's
    // off-diagonal come out as a clean 2 t^2 e^{-2t}.
    for (const double theta : kThetaSweep) {
        const spade::DrydenSecondOrderCoeffs c =
            spade::dryden_second_order_coeffs(static_cast<float>(theta));
        const double e = std::exp(-theta);

        EXPECT_LT(rel_error(c.phi00, e * (1.0 + theta)), 3e-6) << "phi00 at theta=" << theta;
        EXPECT_LT(rel_error(c.phi01, e * theta), 3e-6) << "phi01 at theta=" << theta;
        EXPECT_EQ(c.phi10, -c.phi01) << "phi10 must be exactly -phi01 at theta=" << theta;
        if (std::abs(1.0 - theta) > 1e-3) {
            // Skipped at theta == 1 only: (1 - theta) is exactly zero there,
            // so there is no relative error to speak of.
            EXPECT_LT(rel_error(c.phi11, e * (1.0 - theta)), 1e-5) << "phi11 at theta=" << theta;
        }

        // Contraction at EVERY step ratio: the double eigenvalue is exp(-theta),
        // so there is no stability condition on h/tau (medium.hpp section 3).
        // Checked as the characteristic polynomial rather than by an eigen
        // solver: trace = 2 exp(-theta), det = exp(-2 theta).
        const double trace = static_cast<double>(c.phi00) + c.phi11;
        const double det = static_cast<double>(c.phi00) * c.phi11 -
                           static_cast<double>(c.phi01) * c.phi10;
        EXPECT_NEAR(trace, 2.0 * e, 1e-6) << "theta=" << theta;
        EXPECT_NEAR(det, e * e, 1e-6) << "theta=" << theta;
        EXPECT_LT(det, 1.0) << "|eigenvalue| must be < 1 at theta=" << theta;
    }
}

TEST(DrydenCoefficients, SecondOrderCholeskyMatchesTheExactProcessNoise) {
    // THE conditioning test. Q00 is O(theta^3) -- at theta = 1.25e-4 its true
    // value is 2.6e-12 -- so the closed form 1 - e^{-2t}(1 + 2t + 2t^2)
    // evaluates in fp32 to exactly zero, which would make l00 zero, l10 an
    // infinity, and the filter NaN on its first step. Relative tolerances (not
    // absolute) are the point: a zeroed l00 fails by 100%, not by 2.6e-12.
    for (const double theta : kThetaSweep) {
        const spade::DrydenSecondOrderCoeffs c =
            spade::dryden_second_order_coeffs(static_cast<float>(theta));

        const double q00 = ref_q00(theta);
        const double q01 = ref_q01(theta);
        const double q11 = ref_q11(theta);
        const double l00 = std::sqrt(q00);
        const double l10 = q01 / l00;
        const double l11 = std::sqrt(std::max(q11 - l10 * l10, 0.0));

        // The tolerance is what the ORACLE can claim, not what the
        // implementation achieves -- see q00_reference_tolerance().
        const double tol = q00_reference_tolerance(theta);
        EXPECT_LT(rel_error(c.l00, l00), tol) << "l00 at theta=" << theta << " (got " << c.l00
                                              << ", want " << l00 << ")";
        EXPECT_LT(rel_error(c.l10, l10), tol) << "l10 at theta=" << theta;
        EXPECT_LT(rel_error(c.l11, l11), tol) << "l11 at theta=" << theta;

        EXPECT_TRUE(std::isfinite(c.l00) && std::isfinite(c.l10) && std::isfinite(c.l11))
            << "non-finite Cholesky factor at theta=" << theta;
        EXPECT_GT(c.l00, 0.0f) << "l00 collapsed to zero at theta=" << theta
                               << " -- the small-theta series is not being used";
    }
}

TEST(DrydenCoefficients, SmallStepRatiosMatchTheAsymptoticCholesky) {
    // The second, INDEPENDENT oracle for the end of the range where even a
    // double-precision evaluation of Q00 cancels. As theta -> 0,
    //
    //     l00 -> sqrt(4/3) theta^1.5,   l10 -> sqrt(3) sqrt(theta),
    //     l11 -> sqrt(theta),
    //
    // which follow from Q00 ~ (4/3)t^3, Q01 ~ 2t^2, Q11 ~ 4t with no
    // subtraction anywhere -- so this test cannot fail for the same reason the
    // reference-based one could. The leading-order forms are good to O(theta),
    // hence the 2% band at theta = 1e-3 and a much tighter one below.
    //
    // This is also the test that actually pins the physically interesting
    // regime: theta = 1.25e-4 is a 200 Hz substep against tau_u = 40 s, i.e.
    // the default configuration.
    for (const double theta : {1e-6, 1e-5, 1.25e-4, 1e-3}) {
        const spade::DrydenSecondOrderCoeffs c =
            spade::dryden_second_order_coeffs(static_cast<float>(theta));
        const double band = 2.0 * theta + 1e-5;  // the O(theta) truncation, plus fp32 slack

        EXPECT_LT(rel_error(c.l00, std::sqrt(4.0 / 3.0) * std::pow(theta, 1.5)), band)
            << "l00 at theta=" << theta;
        EXPECT_LT(rel_error(c.l10, std::sqrt(3.0 * theta)), band) << "l10 at theta=" << theta;
        EXPECT_LT(rel_error(c.l11, std::sqrt(theta)), band) << "l11 at theta=" << theta;
    }

    // And the scalar channel's asymptote, l -> sqrt(2 theta).
    for (const double theta : {1e-6, 1e-5, 1.25e-4, 1e-3}) {
        const spade::DrydenFirstOrderCoeffs c =
            spade::dryden_first_order_coeffs(static_cast<float>(theta));
        EXPECT_LT(rel_error(c.l, std::sqrt(2.0 * theta)), theta + 1e-5) << "l at theta=" << theta;
    }
}

TEST(DrydenCoefficients, StationaryCovarianceIsPreservedByOneStep) {
    // The variance-normalization claim, stated as the identity it is derived
    // from: Phi Phi^T + L L^T == I. medium.hpp section 3 DEFINES
    // Q = I - Phi Phi^T, so the discrete filter's stationary state covariance
    // is exactly the identity at every step ratio -- and therefore the gust
    // variance is exactly sigma^2 -- as algebra, not as a small-h limit.
    //
    // Tolerances are ABSOLUTE and split by entry because the residual is not
    // uniform: the (0,1) entry's true value is a cancellation of two products
    // of size ~theta down to zero, so its floor is set by the fp32
    // coefficients' own rounding (~1e-7 * theta) rather than by anything the
    // implementation could improve.
    for (const double theta : kThetaSweep) {
        const spade::DrydenSecondOrderCoeffs c =
            spade::dryden_second_order_coeffs(static_cast<float>(theta));

        const double p00 = c.phi00, p01 = c.phi01, p10 = c.phi10, p11 = c.phi11;
        const double l00 = c.l00, l10 = c.l10, l11 = c.l11;

        const double m00 = p00 * p00 + p01 * p01 + l00 * l00;
        const double m01 = p00 * p10 + p01 * p11 + l00 * l10;
        const double m11 = p10 * p10 + p11 * p11 + l10 * l10 + l11 * l11;

        EXPECT_NEAR(m00, 1.0, 1e-5) << "theta=" << theta;
        EXPECT_NEAR(m11, 1.0, 1e-5) << "theta=" << theta;
        EXPECT_NEAR(m01, 0.0, 1e-5) << "theta=" << theta;
    }
}

TEST(DrydenCoefficients, DegenerateInputsFreezeTheFieldInsteadOfProducingNaN) {
    // h <= 0, V <= 0, L <= 0, and every flavour of non-finite must land on
    // theta = 0: Phi = I, Q = 0. A frozen gust is a defensible answer to a
    // nonsense configuration; a NaN that propagates into every body's force
    // accumulator is not.
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();

    for (const float bad : {0.0f, -1.0f, -inf, nan}) {
        EXPECT_EQ(spade::dryden_step_ratio(bad, 5.0f, 200.0f), 0.0f);
        EXPECT_EQ(spade::dryden_step_ratio(0.005f, bad, 200.0f), 0.0f);
        EXPECT_EQ(spade::dryden_step_ratio(0.005f, 5.0f, bad), 0.0f);
    }

    // Above the clamp the step ratio saturates rather than overflowing; that
    // is what keeps Q01 = 2 t^2 exp(-2t) from evaluating as inf * 0.
    EXPECT_EQ(spade::dryden_step_ratio(1e6f, 5.0f, 1e-6f), spade::kDrydenMaxStepRatio);
    EXPECT_EQ(spade::dryden_step_ratio(inf, 5.0f, 200.0f), 0.0f) << "h = inf is nonsense, not a big step";

    const spade::DrydenSecondOrderCoeffs frozen = spade::dryden_second_order_coeffs(0.0f);
    EXPECT_EQ(frozen.phi00, 1.0f);
    EXPECT_EQ(frozen.phi01, 0.0f);
    EXPECT_EQ(frozen.phi10, 0.0f);
    EXPECT_EQ(frozen.phi11, 1.0f);
    EXPECT_EQ(frozen.l00, 0.0f);
    EXPECT_EQ(frozen.l10, 0.0f);
    EXPECT_EQ(frozen.l11, 0.0f);

    for (const float t : {inf, nan, -1.0f, 1e30f}) {
        const spade::DrydenSecondOrderCoeffs c = spade::dryden_second_order_coeffs(t);
        EXPECT_TRUE(std::isfinite(c.phi00) && std::isfinite(c.phi01) && std::isfinite(c.phi10) &&
                    std::isfinite(c.phi11) && std::isfinite(c.l00) && std::isfinite(c.l10) &&
                    std::isfinite(c.l11))
            << "theta=" << t;
        const spade::DrydenFirstOrderCoeffs f = spade::dryden_first_order_coeffs(t);
        EXPECT_TRUE(std::isfinite(f.phi) && std::isfinite(f.l)) << "theta=" << t;
    }
}

TEST(DrydenFilter, StaysBoundedAndCorrectlyScaledAtAbsurdStepRatios) {
    // h/tau >> 1 is not a configuration this engine should be in, but it is
    // one a badly authored world file can produce, and exact discretization
    // has a specific promise about it: as theta grows the filter degrades to
    // WHITE NOISE OF THE RIGHT VARIANCE rather than oscillating or blowing up.
    // (An explicit-Euler Gauss-Markov update would be unstable past h = 2 tau,
    // which is precisely why medium.hpp does not use one.)
    DrydenParams params = spade::dryden_params(TurbulenceLevel::severe);
    params.reference_airspeed = 5.0f;

    DrydenState state{};
    spade::dryden_init(state, make_world_params(0xB16B00B5ULL));

    constexpr int kSteps = 200'000;
    constexpr float kHugeSubstep = 400.0f;  // theta_u = 10, theta_w = 40 -> clamped
    double sum_sq_u = 0.0;
    double sum_sq_w = 0.0;
    for (int i = 0; i < kSteps; ++i) {
        spade::dryden_advance(state, params, kHugeSubstep);
        const glm::vec3 g = spade::dryden_turbulence(state, params);
        ASSERT_TRUE(std::isfinite(g.x) && std::isfinite(g.y) && std::isfinite(g.z)) << "step " << i;
        sum_sq_u += static_cast<double>(g.x) * g.x;
        sum_sq_w += static_cast<double>(g.y) * g.y;
    }

    // At theta >= 10 the samples are effectively independent, so N_eff = N and
    // the estimator's relative stddev is sqrt(2/2e5) = 0.45%.
    EXPECT_NEAR(std::sqrt(sum_sq_u / kSteps), params.sigma_u, 0.05f * params.sigma_u);
    EXPECT_NEAR(std::sqrt(sum_sq_w / kSteps), params.sigma_w, 0.05f * params.sigma_w);
}

// ---------------------------------------------------------------------------
// Zero intensity.
// ---------------------------------------------------------------------------

TEST(DrydenZeroIntensity, ProducesExactlyZeroPerturbationAndMatchesConstantMediumBitForBit) {
    // Two separable claims, and both matter. (1) The gust vector is EXACTLY
    // zero -- not 1e-30 -- because sigma is exactly zero and the states, while
    // still evolving, are only ever multiplied by it. (2) The medium is
    // therefore bit-identical to ConstantMedium, which is what lets a scenario
    // turn turbulence off and keep its recorded traces.
    const spade::WorldParams world = make_world_params(0x5EED0000FEEDULL);
    const DrydenParams params = spade::dryden_params(TurbulenceLevel::none);

    DrydenState state{};
    spade::dryden_init(state, world);

    const spade::ConstantMedium constant;
    const spade::DrydenMedium dryden(state, params);
    const spade::Medium& via_interface = dryden;

    for (int i = 0; i < 5000; ++i) {
        spade::dryden_advance(state, params, 0.005f);

        const glm::vec3 gust = spade::dryden_turbulence(state, params);
        ASSERT_EQ(gust, glm::vec3(0.0f)) << "step " << i;

        for (const glm::vec3 pos : {glm::vec3(0.0f), glm::vec3(1e4f, -1e4f, 250.0f)}) {
            const spade::MediumSample a = via_interface.sample(world, pos);
            const spade::MediumSample b = constant.sample(world, pos);
            ASSERT_EQ(a.density, b.density) << "step " << i;
            ASSERT_EQ(a.wind, b.wind) << "step " << i;
        }
    }

    // The filter states are NOT frozen -- they evolved, they were just scaled
    // by zero. That is the property that keeps the rng stream position
    // independent of the turbulence level.
    EXPECT_NE(state.u, 0.0f);
    EXPECT_NE(state.v0, 0.0f);
    EXPECT_NE(state.w1, 0.0f);
}

TEST(DrydenZeroIntensity, DrawsTheSameNumberOfGaussiansAsAnyOtherLevel) {
    // Skipping the draws at level none would make every OTHER stochastic
    // system in the engine diverge between a "turbulence off" and a
    // "turbulence on" run of the same scenario. Same seed, different level,
    // identical stream position after the same number of substeps.
    const spade::WorldParams world = make_world_params(0x1234ULL);

    DrydenState off{};
    DrydenState on{};
    spade::dryden_init(off, world);
    spade::dryden_init(on, world);

    const DrydenParams params_off = spade::dryden_params(TurbulenceLevel::none);
    const DrydenParams params_on = spade::dryden_params(TurbulenceLevel::severe);

    for (int i = 0; i < 64; ++i) {
        spade::dryden_advance(off, params_off, 0.005f);
        spade::dryden_advance(on, params_on, 0.005f);
    }

    EXPECT_EQ(off.stream, on.stream) << "the level must not move the stream";
    // ...and the filter states are identical too, because sigma is applied at
    // the OUTPUT, not inside the recursion.
    EXPECT_EQ(off.u, on.u);
    EXPECT_EQ(off.v0, on.v0);
    EXPECT_EQ(off.w1, on.w1);
}

// ---------------------------------------------------------------------------
// The process statistics.
// ---------------------------------------------------------------------------

TEST(DrydenStatistics, ComponentVariancesMatchSigmaSquared) {
    // 1e6 substeps; see kStatSteps for the effective-sample-count reasoning
    // (1.3% estimator stddev on the tightest channel against a 10% tolerance).
    //
    // Also checks the MEAN, which is a different failure: a filter whose
    // stationary distribution is off-centre (a sign error in the Cholesky's
    // off-diagonal, say) can still have the right variance.
    const spade::WorldParams world = make_world_params(0xDEADBEEFCAFEF00DULL);
    const DrydenParams params = stat_params();

    DrydenState state{};
    spade::dryden_init(state, world);

    // double accumulators over fp32 samples: summing a million fp32 values in
    // fp32 would measure the summation, not the filter. Same convention as
    // test_rng_medium.cpp's moments tests.
    glm::dvec3 sum(0.0);
    glm::dvec3 sum_sq(0.0);
    for (int i = 0; i < kStatSteps; ++i) {
        spade::dryden_advance(state, params, kStatSubstep);
        const glm::vec3 g = spade::dryden_turbulence(state, params);
        sum += glm::dvec3(g);
        sum_sq += glm::dvec3(g) * glm::dvec3(g);
    }

    const glm::dvec3 mean = sum / static_cast<double>(kStatSteps);
    const glm::dvec3 variance = sum_sq / static_cast<double>(kStatSteps) - mean * mean;

    // Axis mapping: x = u, y = w, z = v (medium.hpp's axis-mapping note).
    const double sigma_u = params.sigma_u;
    const double sigma_v = params.sigma_v;
    const double sigma_w = params.sigma_w;

    // Measured on the pinned seed at the time of writing (m^2/s^2), against
    // sigma^2 of 6.0477 / 6.0477 / 2.3819: u 6.0113 (-0.60%), v 5.9538
    // (-1.55%), w 2.3980 (+0.68%) -- comfortably inside the 1.3% / 0.63%
    // estimator stddevs derived above, and an order of magnitude inside the
    // 10% tolerance.
    EXPECT_NEAR(variance.x, sigma_u * sigma_u, 0.10 * sigma_u * sigma_u) << "u (world X)";
    EXPECT_NEAR(variance.z, sigma_v * sigma_v, 0.10 * sigma_v * sigma_v) << "v (world Z)";
    EXPECT_NEAR(variance.y, sigma_w * sigma_w, 0.10 * sigma_w * sigma_w) << "w (world Y)";

    // Zero mean. The standard error of the mean of a correlated process is
    // sigma sqrt(2 tau / (N h)): 0.017 m/s for u, 0.005 for w. 5% of sigma is
    // several times that on every channel.
    EXPECT_NEAR(mean.x, 0.0, 0.05 * sigma_u) << "u (world X)";
    EXPECT_NEAR(mean.z, 0.0, 0.05 * sigma_v) << "v (world Z)";
    EXPECT_NEAR(mean.y, 0.0, 0.05 * sigma_w) << "w (world Y)";
}

TEST(DrydenStatistics, AutocorrelationTimeConstantsMatchScaleLengthOverAirspeed) {
    // METHOD, stated because "the autocorrelation time constant" is not one
    // number for two different model orders:
    //
    //   1. estimate rho(k h) for k = 0..K by accumulating lagged products
    //      through a ring buffer (no 1e6-sample array is retained);
    //   2. find the first lag where rho crosses e^-1 and linearly interpolate
    //      between the bracketing lags to get the crossing time t_c;
    //   3. divide t_c by the MODEL's analytic crossing point in units of tau,
    //      which is 1 for the first-order channel (rho = e^{-t/tau} crosses
    //      e^-1 at exactly tau) but 0.6252 for the transverse channels
    //      (rho = e^{-x}(1 - x/2) crosses e^-1 earlier). That constant is not
    //      hard-coded -- it is bisected below from the analytic rho -- so the
    //      test cannot be tuned into agreement by fudging it;
    //   4. compare the resulting tau-hat against L/V.
    //
    // Using the e^-1 crossing rather than a log-linear fit is deliberate: the
    // transverse rho goes NEGATIVE at t > 2 tau, so its logarithm does not
    // exist over the range a fit would want.
    const spade::WorldParams world = make_world_params(0x00C0FFEE12345678ULL);
    const DrydenParams params = stat_params();

    DrydenState state{};
    spade::dryden_init(state, world);

    // tau_u = tau_v = 4 s = 80 substeps; tau_w = 1 s = 20 substeps. 200 lags
    // reaches 10 s, past every channel's crossing and past 1.5 tau_u.
    //
    // The lagged products are accumulated every kLagStride-th sample rather
    // than every sample. That is a 5x cost cut for essentially no statistical
    // cost: the estimator's precision is set by the RUN LENGTH in units of tau
    // (50000 s / 4 s ~ 12500 correlation times), not by how densely a
    // correlation time is sampled -- five consecutive samples 0.05 s apart in a
    // 4 s process are very nearly the same measurement. Every sample still
    // contributes to the mean and to the ring buffer.
    constexpr int kMaxLag = 200;
    constexpr int kLagStride = 5;
    std::array<double, 3> sum{};
    std::array<std::array<double, kMaxLag + 1>, 3> lag_sum{};
    std::array<glm::vec3, kMaxLag + 1> ring{};
    int ring_head = 0;
    int filled = 0;
    int64_t lag_pairs = 0;

    for (int i = 0; i < kStatSteps; ++i) {
        spade::dryden_advance(state, params, kStatSubstep);
        const glm::vec3 g = spade::dryden_turbulence(state, params);

        for (int axis = 0; axis < 3; ++axis) sum[static_cast<std::size_t>(axis)] += g[axis];

        ring[static_cast<std::size_t>(ring_head)] = g;
        if (filled >= kMaxLag) {
            if (i % kLagStride == 0) {
                ++lag_pairs;
                for (int lag = 0; lag <= kMaxLag; ++lag) {
                    const int idx = (ring_head - lag + (kMaxLag + 1)) % (kMaxLag + 1);
                    const glm::vec3 past = ring[static_cast<std::size_t>(idx)];
                    for (int axis = 0; axis < 3; ++axis) {
                        lag_sum[static_cast<std::size_t>(axis)][static_cast<std::size_t>(lag)] +=
                            static_cast<double>(g[axis]) * past[axis];
                    }
                }
            }
        } else {
            ++filled;
        }
        ring_head = (ring_head + 1) % (kMaxLag + 1);
    }
    ASSERT_GT(lag_pairs, 0);

    // The analytic e^-1 crossing of the transverse autocorrelation, in units
    // of tau: the root of exp(-x)(1 - x/2) = exp(-1). Bisected rather than
    // pinned as a literal.
    const double target = std::exp(-1.0);
    auto transverse_rho = [](double x) { return std::exp(-x) * (1.0 - 0.5 * x); };
    double lo = 0.0;
    double hi = 2.0;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (transverse_rho(mid) > target) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    const double transverse_crossing = 0.5 * (lo + hi);
    EXPECT_NEAR(transverse_crossing, 0.6252, 1e-3) << "the analytic transverse crossing moved";

    struct Channel {
        const char* name;
        int axis;             // world axis this Dryden component maps to
        double scale_length;  // L
        double crossing;      // analytic e^-1 crossing, in units of tau
    };
    const std::array<Channel, 3> channels{{
        {"u (world X, first order)", 0, params.scale_u, 1.0},
        {"w (world Y, transverse)", 1, params.scale_w, transverse_crossing},
        {"v (world Z, transverse)", 2, params.scale_v, transverse_crossing},
    }};

    for (const Channel& ch : channels) {
        const std::size_t a = static_cast<std::size_t>(ch.axis);
        const double mean = sum[a] / static_cast<double>(kStatSteps);
        const double pairs = static_cast<double>(lag_pairs);

        // rho(lag) from the lagged products, mean-corrected and normalized by
        // the lag-0 term rather than by an independently computed variance --
        // so rho(0) is 1 by construction and the shape claims below are not
        // contaminated by a 1% disagreement between two estimates of the same
        // variance. The variance itself is the OTHER test's business.
        const double var = lag_sum[a][0] / pairs - mean * mean;
        ASSERT_GT(var, 0.0) << ch.name;

        std::array<double, kMaxLag + 1> rho{};
        for (int lag = 0; lag <= kMaxLag; ++lag) {
            const double cov = lag_sum[a][static_cast<std::size_t>(lag)] / pairs - mean * mean;
            rho[static_cast<std::size_t>(lag)] = cov / var;
        }
        ASSERT_EQ(rho[0], 1.0) << ch.name;

        // First crossing of e^-1, linearly interpolated.
        int k = 1;
        while (k <= kMaxLag && rho[static_cast<std::size_t>(k)] > target) ++k;
        ASSERT_LE(k, kMaxLag) << ch.name << ": rho never fell to e^-1 within " << kMaxLag << " lags";
        const double r_hi = rho[static_cast<std::size_t>(k - 1)];
        const double r_lo = rho[static_cast<std::size_t>(k)];
        const double frac = (r_hi - target) / (r_hi - r_lo);
        const double t_c = (static_cast<double>(k - 1) + frac) * kStatSubstep;

        const double tau_hat = t_c / ch.crossing;
        const double tau_expected = ch.scale_length / kStatAirspeed;
        // Measured on the pinned seed at the time of writing: u -0.95%,
        // w +0.96%, v -0.35% against L/V -- roughly 15x inside the tolerance.
        // The transverse crossings land at t_c = 0.631 s and 2.492 s, i.e.
        // 0.625 tau, which is the whole reason step 3 exists.
        EXPECT_NEAR(tau_hat, tau_expected, 0.15 * tau_expected)
            << ch.name << ": measured tau " << tau_hat << " s vs L/V = " << tau_expected << " s";

        // Shape, not just the one crossing: the whole analytic curve out to
        // 1.5 tau. This is what separates "a first-order process with the
        // right time constant" from "the Dryden transverse process" -- the two
        // agree at the crossing by construction and nowhere else.
        const int check_to = std::min(kMaxLag, static_cast<int>(1.5 * tau_expected / kStatSubstep));
        for (int lag = 1; lag <= check_to; ++lag) {
            const double x = static_cast<double>(lag) * kStatSubstep / tau_expected;
            const double expected =
                (ch.crossing == 1.0) ? std::exp(-x) : transverse_rho(x);
            EXPECT_NEAR(rho[static_cast<std::size_t>(lag)], expected, 0.05)
                << ch.name << " at lag " << lag << " (x = " << x << " tau)";
        }
    }
}

// ---------------------------------------------------------------------------
// Determinism and snapshot safety -- the same structural claims Task 8 makes
// about rng streams, restated for the filter row.
// ---------------------------------------------------------------------------

TEST(DrydenDeterminism, SameSeedReplaysExactlyAndDifferentSeedsDiverge) {
    const DrydenParams params = spade::dryden_params(TurbulenceLevel::moderate);

    DrydenState a{};
    DrydenState b{};
    DrydenState other{};
    spade::dryden_init(a, make_world_params(0xA5A5A5A5A5A5A5A5ULL));
    spade::dryden_init(b, make_world_params(0xA5A5A5A5A5A5A5A5ULL));
    spade::dryden_init(other, make_world_params(0xA5A5A5A5A5A5A5A6ULL));

    const std::vector<glm::vec3> from_a = run_gusts(a, params, 0.005f, 4096);
    const std::vector<glm::vec3> from_b = run_gusts(b, params, 0.005f, 4096);
    const std::vector<glm::vec3> from_other = run_gusts(other, params, 0.005f, 4096);

    EXPECT_EQ(from_a, from_b) << "the same world seed must replay bit-for-bit";
    EXPECT_EQ(a, b) << "...and land on the identical row bytes";
    EXPECT_NE(from_a, from_other) << "a different world seed must produce a different gust sequence";

    // Not merely "different somewhere": the sequences must be uncorrelated,
    // which a seed derivation that only perturbs a low bit would fail.
    double dot = 0.0;
    double norm_a = 0.0;
    double norm_o = 0.0;
    for (std::size_t i = 0; i < from_a.size(); ++i) {
        dot += glm::dot(glm::dvec3(from_a[i]), glm::dvec3(from_other[i]));
        norm_a += glm::dot(glm::dvec3(from_a[i]), glm::dvec3(from_a[i]));
        norm_o += glm::dot(glm::dvec3(from_other[i]), glm::dvec3(from_other[i]));
    }
    const double correlation = dot / std::sqrt(norm_a * norm_o);
    EXPECT_LT(std::abs(correlation), 0.1) << "two world seeds produced correlated gusts: " << correlation;
}

TEST(DrydenSnapshot, TheRowRegistersAsStateAndAppearsInTheWalk) {
    spade::ArenaSet arenas(2);
    const auto rows = arenas.register_array<DrydenState>("world.dryden", 1);
    ASSERT_TRUE(rows.has_value());

    const spade::RegisteredArray* entry = arenas.registry().find("world.dryden");
    ASSERT_NE(entry, nullptr) << "unregistered filter state is state a snapshot would miss";
    EXPECT_EQ(entry->elem_size, sizeof(DrydenState));
    EXPECT_EQ(entry->world_count, 2u);
    EXPECT_EQ(entry->capacity_per_world, 1u);
    EXPECT_EQ(entry->byte_size(), sizeof(DrydenState) * 2u);
}

TEST(DrydenSnapshot, ARegistryWalkRoundTripMidSequenceResumesIdentically) {
    // The claim: EVERYTHING the gust sequence depends on is in the registered
    // row. Save mid-sequence (and specifically mid-Box-Muller-pair, which is
    // the case a `static float spare` implementation gets wrong), restore into
    // a FRESH ArenaSet, and demand the identical continuation.
    constexpr uint32_t kWorlds = 2;
    constexpr uint64_t kSeedA = 0xBEEF0001ULL;
    constexpr uint64_t kSeedB = 0xBEEF0002ULL;
    constexpr int kBefore = 4;  // even -> 5*(4+1) = 25 gaussians drawn -> cache live
    constexpr int kAfter = 512;

    const DrydenParams params = spade::dryden_params(TurbulenceLevel::severe);

    spade::ArenaSet saved(kWorlds);
    const auto saved_id = saved.register_array<DrydenState>("world.dryden", 1);
    ASSERT_TRUE(saved_id.has_value());
    ASSERT_TRUE(saved.alloc_slot(*saved_id, 0).has_value());
    ASSERT_TRUE(saved.alloc_slot(*saved_id, 1).has_value());

    {
        const auto rows = saved.array(*saved_id);
        ASSERT_TRUE(rows.has_value());
        spade::dryden_init((*rows)[0], make_world_params(kSeedA));
        spade::dryden_init((*rows)[1], make_world_params(kSeedB));
        for (int i = 0; i < kBefore; ++i) {
            spade::dryden_advance((*rows)[0], params, 0.005f);
            spade::dryden_advance((*rows)[1], params, 0.005f);
        }
        ASSERT_EQ((*rows)[0].stream.has_cached, 1u)
            << "the Box-Muller cache must be live for this test to mean anything";
    }

    // The control: what the untouched rows would produce next. Copies, so the
    // originals are not advanced.
    std::vector<std::vector<glm::vec3>> expected;
    {
        const auto rows = saved.array(*saved_id);
        ASSERT_TRUE(rows.has_value());
        for (DrydenState row : *rows) expected.push_back(run_gusts(row, params, 0.005f, kAfter));
    }

    const Blob blob = save_walk(saved.registry());

    spade::ArenaSet restored(kWorlds);
    const auto restored_id = restored.register_array<DrydenState>("world.dryden", 1);
    ASSERT_TRUE(restored_id.has_value());
    restore_walk(restored.registry(), blob);
    ASSERT_TRUE(restored.resync_from_slot_to_world(*restored_id).has_value());

    const auto restored_rows = restored.array(*restored_id);
    ASSERT_TRUE(restored_rows.has_value());
    ASSERT_EQ(restored_rows->size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(run_gusts((*restored_rows)[i], params, 0.005f, kAfter), expected[i]) << "world " << i;
    }
}

TEST(DrydenSnapshot, TheFortyBytesAreTheWholeFilter) {
    // The converse framing of the round trip, without an arena in the way: a
    // memcpy of the row is a complete description of its future. Anything held
    // in a static, a thread_local, or a hidden engine-global would survive the
    // copy and break this.
    const DrydenParams params = spade::dryden_params(TurbulenceLevel::light);

    DrydenState original{};
    spade::dryden_init(original, make_world_params(0x2026080900000010ULL));
    for (int i = 0; i < 7; ++i) spade::dryden_advance(original, params, 0.004f);

    std::array<std::byte, sizeof(DrydenState)> bytes{};
    std::memcpy(bytes.data(), &original, sizeof(DrydenState));

    const std::vector<glm::vec3> from_original = run_gusts(original, params, 0.004f, 64);

    DrydenState reconstituted{};
    std::memcpy(&reconstituted, bytes.data(), sizeof(DrydenState));
    EXPECT_EQ(run_gusts(reconstituted, params, 0.004f, 64), from_original);
}

// ---------------------------------------------------------------------------
// The Medium seam.
// ---------------------------------------------------------------------------

TEST(DrydenMediumSeam, AddsTheGustToTheMeanWindAndLeavesDensityAlone) {
    const spade::WorldParams world = make_world_params(0x77777777ULL);
    const DrydenParams params = spade::dryden_params(TurbulenceLevel::moderate);

    DrydenState state{};
    spade::dryden_init(state, world);

    const spade::DrydenMedium medium(state, params);
    const spade::Medium& via_interface = medium;

    for (int i = 0; i < 128; ++i) {
        spade::dryden_advance(state, params, 0.005f);
        const spade::MediumSample sample = via_interface.sample(world, glm::vec3(0.0f));
        EXPECT_EQ(sample.density, world.air_density) << "step " << i;
        EXPECT_EQ(sample.wind, world.wind + spade::dryden_turbulence(state, params)) << "step " << i;
    }

    // The gust really is non-zero at this level -- otherwise the assertions
    // above would pass for a broken filter that returns nothing.
    EXPECT_NE(via_interface.sample(world, glm::vec3(0.0f)).wind, world.wind);
}

TEST(DrydenMediumSeam, IsPositionIndependentAndDoesNotAdvanceOnSample) {
    // Two properties of the FROZEN-FIELD POINT model, both of which a future
    // spatial field will deliberately break -- so they are pinned here as the
    // v1 contract rather than left implicit.
    const spade::WorldParams world = make_world_params(0x88888888ULL);
    const DrydenParams params = spade::dryden_params(TurbulenceLevel::severe);

    DrydenState state{};
    spade::dryden_init(state, world);
    for (int i = 0; i < 16; ++i) spade::dryden_advance(state, params, 0.005f);

    const spade::DrydenMedium medium(state, params);
    const spade::MediumSample reference = medium.sample(world, glm::vec3(0.0f));

    const DrydenState before = state;
    for (const glm::vec3 pos : {glm::vec3(1000.0f, -500.0f, 250.0f), glm::vec3(-1e6f, 1e6f, 0.0f),
                                glm::vec3(0.001f, 0.0f, -0.001f)}) {
        const spade::MediumSample sample = medium.sample(world, pos);
        EXPECT_EQ(sample.density, reference.density);
        EXPECT_EQ(sample.wind, reference.wind);
    }
    EXPECT_EQ(state, before) << "sample() must be a pure read -- advancing is dryden_advance()'s job";
}

TEST(DrydenMediumSeam, DifferentWorldsGetIndependentRowsThroughOneInterface) {
    // A DrydenMedium is a per-world VIEW (unlike ConstantMedium's single
    // shared instance), and the reason is visible here: two worlds sampled
    // through the same abstract interface must see their own filter rows.
    const spade::WorldParams world_a = make_world_params(0x1111ULL);
    spade::WorldParams world_b = make_world_params(0x2222ULL);
    world_b.air_density = 0.4135f;
    world_b.wind = glm::vec3(-12.0f, 0.0f, 4.0f);

    const DrydenParams params = spade::dryden_params(TurbulenceLevel::moderate);

    DrydenState state_a{};
    DrydenState state_b{};
    spade::dryden_init(state_a, world_a);
    spade::dryden_init(state_b, world_b);
    for (int i = 0; i < 32; ++i) {
        spade::dryden_advance(state_a, params, 0.005f);
        spade::dryden_advance(state_b, params, 0.005f);
    }

    const spade::DrydenMedium medium_a(state_a, params);
    const spade::DrydenMedium medium_b(state_b, params);
    const spade::Medium& a = medium_a;
    const spade::Medium& b = medium_b;

    const spade::MediumSample sa = a.sample(world_a, glm::vec3(1.0f, 2.0f, 3.0f));
    const spade::MediumSample sb = b.sample(world_b, glm::vec3(1.0f, 2.0f, 3.0f));

    EXPECT_EQ(sa.density, world_a.air_density);
    EXPECT_EQ(sb.density, world_b.air_density);
    EXPECT_NE(sa.wind, sb.wind);
    EXPECT_NE(sa.wind - world_a.wind, sb.wind - world_b.wind) << "the two worlds' gusts must differ";
}
