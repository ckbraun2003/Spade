#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

#include "core/fp32_math.hpp"
#include "core/rng.hpp"

// ---------------------------------------------------------------------------
// core/fp32_math.hpp -- the engine's own log/sin/cos.
//
// TWO BURDENS OF PROOF, and they are different burdens carried by different
// tests, because neither implies the other:
//
//   * ACCURACY. These must be as good as a libm, or replacing the libm has
//     traded a correctness problem for a numerics problem. Measured in ULPS
//     against a double-precision reference, and measured EXHAUSTIVELY over the
//     domain that actually matters rather than by sampling -- the whole input
//     space of Stream::next_float() is 2^24 values, which is small enough to
//     enumerate in full. A sampled bound would be an estimate; an enumerated
//     one is a fact.
//
//   * DETERMINISM. This is the reason the file exists, and it is the burden a
//     single-platform test CANNOT discharge -- proving it needs the CI matrix
//     (.github/workflows/spade.yml runs MSVC and gcc-13 on the same commit)
//     plus the golden digest corpus, which is exactly the mechanism that
//     caught the original defect. What CAN be checked here, and is, are the
//     structural properties the determinism argument rests on: that the
//     constants have the exactness the Cody-Waite reduction claims, and that
//     nothing in the implementation reads a rounding mode or a libm.
//
// The ULP bounds below are asserted as `<= 1` rather than `== 0`. Correct
// rounding is NOT the goal and never was: the goal is that both platforms
// produce the SAME bits, which a shared implementation gives by construction
// at any accuracy. One ulp is simply the quality bar, and it is the same bar
// the two libms this replaces meet.
// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] float float_of(uint32_t u) {
    float x = 0.0f;
    std::memcpy(&x, &u, sizeof(x));
    return x;
}

// Signed ULP distance between a float and a double-precision reference value.
// Computed by walking the float lattice: the reference is rounded to float,
// then the two bit patterns are compared as monotone integers (which they are,
// for same-signed finite floats). Returns a large sentinel if either side is
// non-finite, so a NaN can never be mistaken for a zero-ulp match.
[[nodiscard]] double ulp_error(float got, double reference) {
    if (!std::isfinite(got) || !std::isfinite(reference)) return 1e30;
    const float ref = static_cast<float>(reference);
    if (got == ref) return 0.0;
    // Distance in units of the local ulp -- exact enough at the 1-2 ulp scale
    // this test cares about, and immune to the exponent-boundary awkwardness
    // an integer bit difference would have.
    const float ulp = std::nextafter(ref, std::numeric_limits<float>::infinity()) - ref;
    if (ulp == 0.0f) return 1e30;
    return std::abs((static_cast<double>(got) - static_cast<double>(ref)) / static_cast<double>(ulp));
}

}  // namespace

// ===========================================================================
// 1. log32 -- exhaustive over the ENTIRE domain Box-Muller can present
// ===========================================================================

// next_float() returns k * 2^-24 for integer k in [0, 2^24), and next_gauss
// clamps k == 0 up to 1. So the complete set of arguments log32 can ever see
// inside the engine's gaussian is exactly {k * 2^-24 : k in [1, 2^24)} -- all
// 16,777,215 of them, enumerated here. There is no sampling and no seed: this
// is the whole input space, so the bound it reports is a maximum, not an
// estimate.
TEST(Fp32Log, IsWithinOneUlpAcrossEveryValueNextFloatCanProduce) {
    double worst = 0.0;
    uint32_t worst_k = 0;
    for (uint32_t k = 1; k < (1u << 24); ++k) {
        const float u = static_cast<float>(k) * spade::rng::kUniformQuantum;
        const double err = ulp_error(spade::math::log32(u), std::log(static_cast<double>(u)));
        if (err > worst) {
            worst = err;
            worst_k = k;
        }
    }
    EXPECT_LE(worst, 1.0) << "worst at k=" << worst_k << " (u=" << worst_k * 0x1.0p-24 << ")";
    std::printf("log32 worst ulp over all 2^24 next_float values: %.3f (k=%u)\n", worst, worst_k);
}

// The rest of the normal range, on a stride that still covers every binade and
// every mantissa region. log32 is used only by next_gauss today, but it is a
// general routine and a general routine that is only ever tested on [2^-24, 1)
// would be a trap for its second caller.
TEST(Fp32Log, IsWithinOneUlpAcrossTheWholeNormalRange) {
    double worst = 0.0;
    uint32_t worst_bits = 0;
    // Every 4096th representable positive normal float: ~2.1M samples spread
    // uniformly over the bit space, hence uniformly over every exponent.
    for (uint32_t b = 0x00800000u; b < 0x7F800000u; b += 4096u) {
        const float x = float_of(b);
        const double err = ulp_error(spade::math::log32(x), std::log(static_cast<double>(x)));
        if (err > worst) {
            worst = err;
            worst_bits = b;
        }
    }
    EXPECT_LE(worst, 1.0) << "worst at bits 0x" << std::hex << worst_bits;
}

TEST(Fp32Log, MatchesTheLibraryOnExactPowersOfTwo) {
    // ln(2^n) = n*ln2 is where the reconstruction's exact-product argument
    // (e * kLn2Hi carries no rounding) is doing all the work, so these are
    // checked directly rather than left to the sweep.
    for (int n = -126; n <= 127; ++n) {
        const float x = std::ldexp(1.0f, n);
        EXPECT_LE(ulp_error(spade::math::log32(x), std::log(static_cast<double>(x))), 1.0) << "2^" << n;
    }
    EXPECT_EQ(spade::math::log32(1.0f), 0.0f) << "ln(1) must be exactly zero, not an epsilon";
}

TEST(Fp32Log, HandlesSubnormalsAndEdgeCasesLikeTheStandardLogarithm) {
    EXPECT_EQ(spade::math::log32(0.0f), -std::numeric_limits<float>::infinity());
    EXPECT_EQ(spade::math::log32(-0.0f), -std::numeric_limits<float>::infinity());
    EXPECT_TRUE(std::isnan(spade::math::log32(-1.0f)));
    EXPECT_TRUE(std::isnan(spade::math::log32(-std::numeric_limits<float>::infinity())));
    EXPECT_TRUE(std::isnan(spade::math::log32(std::numeric_limits<float>::quiet_NaN())));
    EXPECT_EQ(spade::math::log32(std::numeric_limits<float>::infinity()),
              std::numeric_limits<float>::infinity());

    // Subnormals are scaled into the normal range rather than being allowed to
    // decompose into a garbage exponent.
    const float tiny = std::numeric_limits<float>::denorm_min();
    EXPECT_LE(ulp_error(spade::math::log32(tiny), std::log(static_cast<double>(tiny))), 1.0);
    const float small = std::numeric_limits<float>::min() * 0.5f;
    EXPECT_LE(ulp_error(spade::math::log32(small), std::log(static_cast<double>(small))), 1.0);
}

// ===========================================================================
// 2. sin32 / cos32 -- exhaustive over the ENTIRE Box-Muller angle domain
// ===========================================================================

// theta = kTwoPi * u2 with u2 = k * 2^-24, so the complete set of angles the
// engine's gaussian can present is these 2^24 products. Enumerated in full,
// same argument as the log sweep above.
TEST(Fp32SinCos, AreWithinOneUlpAcrossEveryBoxMullerAngle) {
    double worst_sin = 0.0;
    double worst_cos = 0.0;
    uint32_t worst_sin_k = 0;
    uint32_t worst_cos_k = 0;
    for (uint32_t k = 0; k < (1u << 24); ++k) {
        const float u2 = static_cast<float>(k) * spade::rng::kUniformQuantum;
        const float theta = spade::rng::kTwoPi * u2;
        const double t = static_cast<double>(theta);

        const double es = ulp_error(spade::math::sin32(theta), std::sin(t));
        if (es > worst_sin) {
            worst_sin = es;
            worst_sin_k = k;
        }
        const double ec = ulp_error(spade::math::cos32(theta), std::cos(t));
        if (ec > worst_cos) {
            worst_cos = ec;
            worst_cos_k = k;
        }
    }
    EXPECT_LE(worst_sin, 1.0) << "worst sin at k=" << worst_sin_k;
    EXPECT_LE(worst_cos, 1.0) << "worst cos at k=" << worst_cos_k;
    std::printf("sin32/cos32 worst ulp over all 2^24 Box-Muller angles: %.3f / %.3f\n", worst_sin,
                worst_cos);
}

// The documented accuracy domain, |x| <= kMaxAccurateAngle, is wider than the
// Box-Muller interval; the claim is checked across all of it and on both signs.
TEST(Fp32SinCos, AreWithinOneUlpAcrossTheWholeDocumentedAccuracyDomain) {
    double worst = 0.0;
    float worst_x = 0.0f;
    constexpr int kSteps = 500'000;
    for (int i = -kSteps; i <= kSteps; ++i) {
        const float x = spade::math::kMaxAccurateAngle * (static_cast<float>(i) / kSteps);
        const double t = static_cast<double>(x);
        for (const double err : {ulp_error(spade::math::sin32(x), std::sin(t)),
                                 ulp_error(spade::math::cos32(x), std::cos(t))}) {
            if (err > worst) {
                worst = err;
                worst_x = x;
            }
        }
    }
    EXPECT_LE(worst, 1.0) << "worst at x=" << worst_x;
}

TEST(Fp32SinCos, HitTheExactIdentitiesAtZeroAndAgreeWithPythagoras) {
    EXPECT_EQ(spade::math::sin32(0.0f), 0.0f) << "sin(0) must be exactly zero";
    EXPECT_EQ(spade::math::cos32(0.0f), 1.0f) << "cos(0) must be exactly one";
    EXPECT_EQ(spade::math::sin32(-0.0f), -0.0f);

    // sin^2 + cos^2 == 1 to fp32 across the quadrant reconstruction. This is
    // the property a botched quadrant table breaks while every individual ulp
    // check still passes.
    for (int i = 0; i <= 4000; ++i) {
        const float x = 6.2831853f * (static_cast<float>(i) / 4000.0f);
        const float s = spade::math::sin32(x);
        const float c = spade::math::cos32(x);
        EXPECT_NEAR(s * s + c * c, 1.0f, 2e-7f) << "x=" << x;
    }
}

TEST(Fp32SinCos, AreOddAndEvenExactly) {
    // Sign symmetry is structural here (the reduction carries the sign of x
    // through n), so it is exact rather than approximate -- and a quadrant
    // table that got a sign wrong would fail this before any ulp test noticed.
    for (int i = 1; i <= 20000; ++i) {
        const float x = 6.2831853f * (static_cast<float>(i) / 20000.0f);
        EXPECT_EQ(spade::math::sin32(-x), -spade::math::sin32(x)) << "x=" << x;
        EXPECT_EQ(spade::math::cos32(-x), spade::math::cos32(x)) << "x=" << x;
    }
}

TEST(Fp32SinCos, ReturnNaNForNonFiniteArguments) {
    EXPECT_TRUE(std::isnan(spade::math::sin32(std::numeric_limits<float>::infinity())));
    EXPECT_TRUE(std::isnan(spade::math::cos32(-std::numeric_limits<float>::infinity())));
    EXPECT_TRUE(std::isnan(spade::math::sin32(std::numeric_limits<float>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(spade::math::cos32(std::numeric_limits<float>::quiet_NaN())));
}

// ===========================================================================
// 3. The structural facts the determinism argument rests on
// ===========================================================================

// The Cody-Waite split's whole correctness argument is "n * part is an EXACT
// product for every quadrant index the accuracy domain can produce". That is a
// property of the constants' trailing zero bits, so it is checkable directly
// -- and if a future edit retunes a constant without preserving it, the
// reduction silently loses precision instead of failing to compile.
TEST(Fp32SinCos, TheCodyWaiteConstantsMultiplyExactlyAcrossTheAccuracyDomain) {
    // |x| <= 100 => |n| <= round(100 * 2/pi) = 64.
    constexpr int32_t kMaxN = 64;
    // Re-declared here rather than exported: a test that reads the same
    // constant object the implementation uses proves nothing about the
    // constant, so these are transcribed and the transcription is the check.
    const float parts[3] = {0x1.921f8p+0f, 0x1.aa22p-19f, 0x1.68cp-39f};

    for (int32_t n = -kMaxN; n <= kMaxN; ++n) {
        for (const float part : parts) {
            const float product = static_cast<float>(n) * part;
            const double exact = static_cast<double>(n) * static_cast<double>(part);
            EXPECT_EQ(static_cast<double>(product), exact)
                << "n=" << n << " part=" << part << " -- the product must be exact";
        }
    }

    // And the three parts really do sum to pi/2 to well beyond fp32.
    const double sum = static_cast<double>(parts[0]) + static_cast<double>(parts[1]) +
                       static_cast<double>(parts[2]);
    EXPECT_LT(std::abs(sum - 1.57079632679489661923), 1e-18);
}

// The same check for the logarithm's ln2 split: e * kLn2Hi must be exact for
// every exponent a float can carry.
TEST(Fp32Log, TheLn2SplitMultipliesExactlyAcrossEveryFloatExponent) {
    const float ln2_hi = 0x1.62e4p-1f;
    const float ln2_lo = 0x1.7f7d1cp-20f;
    for (int32_t e = -150; e <= 128; ++e) {
        const float product = static_cast<float>(e) * ln2_hi;
        EXPECT_EQ(static_cast<double>(product), static_cast<double>(e) * static_cast<double>(ln2_hi))
            << "e=" << e;
    }
    EXPECT_LT(std::abs((static_cast<double>(ln2_hi) + static_cast<double>(ln2_lo)) -
                       0.69314718055994530942),
              1e-13);
}
