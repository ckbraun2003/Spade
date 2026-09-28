#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "core/fp32_math.hpp"
#include "core/rng.hpp"

// ---------------------------------------------------------------------------
// core/fp32_math.hpp -- the engine's own log/exp/sin/cos.
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
//
// A NOTE ON THE DOUBLE REFERENCE, since it is a libm and this whole file
// exists because libms disagree. std::log/std::exp/std::sin in DOUBLE are
// accurate to well under a double ulp, i.e. under 2.2e-16 relative, which is
// 2^29 times finer than a float ulp's 1.2e-7. Two platforms' doubles may
// differ from each other in their last bit, and that difference is half a
// billion times too small to move any assertion here. So the reference is
// portable for this purpose even though it would not be portable inside the
// engine -- the engine's problem is bit-EQUALITY, this file's is accuracy.
// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] float float_of(uint32_t u) {
    float x = 0.0f;
    std::memcpy(&x, &u, sizeof(x));
    return x;
}

[[nodiscard]] uint32_t bits_of(float x) {
    uint32_t u = 0;
    std::memcpy(&u, &x, sizeof(u));
    return u;
}

// Is an environment gate set? Used by exactly one test, the opt-in full-domain
// sweep at the end of the exp32 section.
//
// The MSVC pragma is scoped to this one call rather than defined away globally:
// C4996 on std::getenv is a CRT deprecation aimed at the buffer-handling
// hazards of getenv_s's predecessors, and none of them apply to a read-only
// null check performed once, before the suite has started any thread. Saying
// that here is cheaper than a _dupenv_s dance that would have to free().
[[nodiscard]] bool env_gate_is_set(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name) != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// Signed ULP distance between a float and a double-precision reference value.
// Computed by walking the float lattice: the reference is rounded to float,
// then the two bit patterns are compared as monotone integers (which they are,
// for same-signed finite floats). Returns a large sentinel if either side is
// non-finite, so a NaN can never be mistaken for a zero-ulp match.
//
// SATURATED REFERENCES ARE AN EXACT-MATCH REQUIREMENT, not a sentinel. exp32
// legitimately answers +infinity above 88.7228394 and +0 below -103.972084,
// and at those arguments the correctly rounded float answer IS the infinity or
// the zero -- so "did it saturate in exactly the same place the reference
// does" is the right question, and the answer is 0 ulp or a failure. Without
// this branch every argument in the saturated tails would report the sentinel
// and a sweep across them could not distinguish correct saturation from a
// broken guard. (log32/sin32/cos32 never present such a reference, so this
// changes nothing for them.)
[[nodiscard]] double ulp_error(float got, double reference) {
    const float saturated = static_cast<float>(reference);
    if (!std::isfinite(saturated) || saturated == 0.0f) return got == saturated ? 0.0 : 1e30;
    if (!std::isfinite(got)) return 1e30;
    const float ref = saturated;
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
// 2. exp32 -- the three windows that DECIDE the bound, at full float density
// ===========================================================================
//
// WHY THIS SECTION IS SHAPED DIFFERENTLY FROM THE OTHER TWO, stated up front
// because "exhaustive" is the standard this file set for itself and these
// tests do not, quite, meet it.
//
// log32 and sin32/cos32 each have a FINITE argument set inside the engine --
// the 2^24 values next_float() can return, and the 2^24 Box-Muller angles --
// so enumerating that set IS enumerating the whole problem. exp32 has no such
// set: world/medium.cpp evaluates it at -theta and -2*theta for any step ratio
// in [0, 32], vehicles/rotor.cpp at -h/tau for any positive ratio at all, and
// there are 1.12 billion floats in [0, 104] before either sign is counted.
//
// The whole 2,241,855,490-argument sweep WAS run, during development, against
// the same double reference these tests use: worst 1.000 ulp, zero arguments
// above it, 99.19% correctly rounded. It takes just over two minutes -- past
// this suite's 60-second per-test timeout in Release and hopelessly past it in
// Debug -- so what is COMMITTED is the three windows in which that bound is
// decided, each at stride 1, plus a strided pass over everything else:
//
//   * THE REDUCTION BOUNDARY, [0.25, 0.5) and its negation. |r| reaches its
//     maximum ln(2)/2 = 0.34657 inside this window and the quotient n flips
//     between 0 and +-1 at exactly that point, so both the polynomial's worst
//     case and the reduction's discontinuity are enumerated here.
//   * THE LARGEST QUOTIENTS AND THE SUBNORMAL TAIL, [64, 104] and its
//     negation. |n| runs to 150 here, which is where n*kLn2Lo -- the
//     reduction's only rounding -- is largest; the negative half is also the
//     entire subnormal-result region and both saturation boundaries.
//   * EVERYTHING ELSE, strided. Same posture, and the same reasoning, as the
//     whole-normal-range sweep in the log32 section above.
//
// The engine's own arguments are a subset of the second and third: Dryden's
// live in [-64, 0] and the rotor lag's in (-inf, -0.25].
// ===========================================================================

// [0.25, 0.5) and [-0.5, -0.25], every float, both signs: 2^24 arguments, the
// same order of magnitude as the log32 and Box-Muller sweeps above.
TEST(Fp32Exp, IsWithinOneUlpAcrossTheReductionBoundaryAtFullFloatDensity) {
    double worst = 0.0;
    float worst_x = 0.0f;
    for (uint32_t b = 0x3E800000u; b < 0x3F000000u; ++b) {
        for (const float x : {float_of(b), -float_of(b)}) {
            const double err = ulp_error(spade::math::exp32(x), std::exp(static_cast<double>(x)));
            if (err > worst) {
                worst = err;
                worst_x = x;
            }
        }
    }
    EXPECT_LE(worst, 1.0) << "worst at x=" << worst_x;
    std::printf("exp32 worst ulp over all 2^24 floats spanning the ln2/2 reduction boundary: %.3f\n",
                worst);
}

// [64, 104] and its negation, every float: the largest quotients the reduction
// can produce, the whole subnormal-result tail, and both saturation
// boundaries -- about 10.5 million arguments.
TEST(Fp32Exp, IsWithinOneUlpAcrossTheLargestQuotientsAndTheSubnormalTail) {
    double worst = 0.0;
    float worst_x = 0.0f;
    for (uint32_t b = 0x42800000u; b <= 0x42D00000u; ++b) {
        for (const float x : {float_of(b), -float_of(b)}) {
            const double err = ulp_error(spade::math::exp32(x), std::exp(static_cast<double>(x)));
            if (err > worst) {
                worst = err;
                worst_x = x;
            }
        }
    }
    EXPECT_LE(worst, 1.0) << "worst at x=" << worst_x;
    std::printf("exp32 worst ulp over every float in +-[64, 104]: %.3f\n", worst);
}

// Everything else, on a stride that still covers every binade and every
// mantissa region -- and, at the same time, the TOTALITY check: no argument
// anywhere may produce a NaN or a negative result, because exp32's callers
// (world/medium.cpp's Cholesky, vehicles/rotor.cpp's lag coefficient) treat a
// non-negative answer as a precondition rather than checking for one.
TEST(Fp32Exp, IsWithinOneUlpAndTotalAcrossTheWholeFiniteDomain) {
    double worst = 0.0;
    float worst_x = 0.0f;
    for (uint32_t b = 0u; b <= 0x42D00000u; b += 256u) {
        for (const float x : {float_of(b), -float_of(b)}) {
            const float got = spade::math::exp32(x);
            ASSERT_FALSE(std::isnan(got)) << "exp32 must be total; x=" << x;
            ASSERT_GE(got, 0.0f) << "exp32 must never be negative; x=" << x;
            const double err = ulp_error(got, std::exp(static_cast<double>(x)));
            if (err > worst) {
                worst = err;
                worst_x = x;
            }
        }
    }
    EXPECT_LE(worst, 1.0) << "worst at x=" << worst_x;
}

// Both saturation boundaries, walked one float at a time from inside the
// finite range out into the saturated one. This is where a guard placed an ulp
// out of position shows up: the assertion is not "it saturates eventually" but
// "it saturates on exactly the float the correctly rounded exponential does".
TEST(Fp32Exp, SaturatesOnExactlyTheSameFloatTheCorrectlyRoundedExponentialDoes) {
    // Overflow: the last finite result is at 88.7228317, the first +infinity
    // at 88.7228394. 4096 floats either side of the boundary.
    for (uint32_t b = 0x42B17218u - 4096u; b <= 0x42B17218u + 4096u; ++b) {
        const float x = float_of(b);
        const float reference = static_cast<float>(std::exp(static_cast<double>(x)));
        const float got = spade::math::exp32(x);
        ASSERT_EQ(std::isinf(got), std::isinf(reference)) << "x=" << x;
        if (std::isfinite(reference)) {
            EXPECT_LE(ulp_error(got, std::exp(static_cast<double>(x))), 1.0) << "x=" << x;
        }
    }

    // Underflow: the last non-zero result is at -103.972076, the first +0 at
    // -103.972084.
    for (uint32_t b = 0x42CFF1B5u - 4096u; b <= 0x42CFF1B5u + 4096u; ++b) {
        const float x = -float_of(b);
        const float reference = static_cast<float>(std::exp(static_cast<double>(x)));
        const float got = spade::math::exp32(x);
        ASSERT_EQ(got == 0.0f, reference == 0.0f) << "x=" << x;
        EXPECT_LE(ulp_error(got, std::exp(static_cast<double>(x))), 1.0) << "x=" << x;
    }

    // The two guard constants themselves, and the header's quoted boundaries.
    EXPECT_TRUE(std::isinf(spade::math::exp32(89.0f)));
    EXPECT_TRUE(std::isinf(spade::math::exp32(88.7228394f)));
    EXPECT_TRUE(std::isfinite(spade::math::exp32(88.7228317f)));
    EXPECT_EQ(spade::math::exp32(-104.0f), 0.0f);
    EXPECT_EQ(spade::math::exp32(-103.972084f), 0.0f);
    EXPECT_GT(spade::math::exp32(-103.972076f), 0.0f);
}

TEST(Fp32Exp, HandlesTheIdentityAtZeroAndTheNonFiniteArgumentsLikeTheStandardExponential) {
    EXPECT_EQ(spade::math::exp32(0.0f), 1.0f) << "exp(0) must be exactly one, not an epsilon";
    EXPECT_EQ(spade::math::exp32(-0.0f), 1.0f);

    EXPECT_EQ(spade::math::exp32(std::numeric_limits<float>::infinity()),
              std::numeric_limits<float>::infinity());
    EXPECT_EQ(spade::math::exp32(-std::numeric_limits<float>::infinity()), 0.0f);
    // +0, not -0: std::exp underflows toward positive zero and so must this.
    EXPECT_FALSE(std::signbit(spade::math::exp32(-std::numeric_limits<float>::infinity())));
    EXPECT_TRUE(std::isnan(spade::math::exp32(std::numeric_limits<float>::quiet_NaN())));

    // The subnormal-result region is documented as reduced precision but is
    // still held to the same 1-ulp bound on its own coarser lattice, and the
    // two-step reconstruction that produces it is the part a reader will
    // doubt. denorm_min itself is the extreme case.
    for (const float x : {-87.3365479f, -88.0f, -95.0f, -100.0f, -103.0f, -103.9f}) {
        EXPECT_LE(ulp_error(spade::math::exp32(x), std::exp(static_cast<double>(x))), 1.0)
            << "subnormal result at x=" << x;
    }

    // Totality ABOVE the saturation guards, which the sweeps above stop at 104
    // and would otherwise leave to inspection. Every one of these is answered
    // by an early return rather than by arithmetic, and that is the claim.
    for (const float x : {104.0f, 1.0e6f, 1.0e30f, std::numeric_limits<float>::max()}) {
        EXPECT_EQ(spade::math::exp32(x), std::numeric_limits<float>::infinity()) << "x=" << x;
        EXPECT_EQ(spade::math::exp32(-x), 0.0f) << "x=" << -x;
        EXPECT_FALSE(std::signbit(spade::math::exp32(-x))) << "underflow must reach +0, not -0";
    }
}

// The headline claim, RE-RUNNABLE. It takes 90 s in Release on the box this was
// written on and several times that in Debug, against a suite whose every other
// case is under two seconds -- so it cannot gate, and it is opt-in. But a
// number quoted in a header, a commit message and a review must be reproducible
// FROM THE REPOSITORY, not from a scratch directory somebody has since deleted.
//
//     $env:SPADE_FULL_EXP_SWEEP = 1
//     powershell -File scripts\test.ps1 -Preset release -Filter FullDomainSweep
//
// WHY AN ENVIRONMENT GATE AND NOT THE DISABLED_ PREFIX, which is the idiom a
// reader will expect and which was tried first: gtest_discover_tests sets
// CTest's own DISABLED property on any test whose name begins with DISABLED_,
// and a CTest-DISABLED test is skipped by CTest itself before the binary is
// ever launched. No flag overrides it, and --gtest_also_run_disabled_tests is
// not a CTest option at all -- so the DISABLED_ form would have been reachable
// only by invoking the gtest executable directly, which scripts/test.ps1
// exists to forbid. GTEST_SKIP costs the same nothing on a normal run (it
// reports as skipped in ~0.02 s) and stays reachable through the sanctioned
// path, which is the whole point of committing the sweep.
//
// ORIGINAL RUN, the provenance for the figures quoted in core/fp32_math.hpp and
// in the Task 1 commit message: 2026-08-11, MSVC 19.44 x64, /fp:precise,
// Release, Windows 11. n = 2,241,855,490 (every float in [-104, 104], both
// signs -- outside that range exp32 answers with the saturated limit, and the
// test above covers those); worst 1.000 ulp; ZERO arguments above 1 ulp;
// 18,062,900 not correctly rounded, i.e. 99.194% correctly rounded; 127 s.
//
// READ THE 99.19% FOR WHAT IT IS: a whole-line figure, dominated by near-zero
// arguments where exp32 returns 1 + x trivially. It is not the local rate
// anywhere in particular -- over the band the rotor lag hands over at it is
// 86.8% (vehicles/rotor.cpp quotes that one, in the place it matters).
TEST(Fp32Exp, FullDomainSweepEveryFloatArgument) {
    if (!env_gate_is_set("SPADE_FULL_EXP_SWEEP")) {
        GTEST_SKIP() << "set SPADE_FULL_EXP_SWEEP=1 to run the ~2 minute full-domain sweep";
    }

    double worst = 0.0;
    float worst_x = 0.0f;
    uint64_t n = 0;
    uint64_t missed = 0;
    for (uint32_t b = 0u;; ++b) {
        for (const float x : {float_of(b), -float_of(b)}) {
            const double err = ulp_error(spade::math::exp32(x), std::exp(static_cast<double>(x)));
            ++n;
            if (err > 0.0) ++missed;
            if (err > worst) {
                worst = err;
                worst_x = x;
            }
        }
        if (b == 0x42D00000u) break;
    }
    EXPECT_LE(worst, 1.0) << "worst at x=" << worst_x;
    std::printf("exp32 FULL DOMAIN: n=%llu  worst=%.6f  not-correctly-rounded=%llu (%.3f%%)\n",
                static_cast<unsigned long long>(n), worst, static_cast<unsigned long long>(missed),
                100.0 * static_cast<double>(missed) / static_cast<double>(n));
}

// ===========================================================================
// 3. sin32 / cos32 -- exhaustive over the ENTIRE Box-Muller angle domain
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

// ---------------------------------------------------------------------------
// THE HUGE-ARGUMENT DOMAIN -- the coverage gap that let a real defect through.
//
// Until S5 Task 1 the only caller of these functions was rng.hpp's Box-Muller
// angle, always in [0, 2*pi], so nothing ever asked what they did at 1e8. The
// answer, measured, was: |cos32| exceeds 1 from x = 27,107,084 and |sin32| from
// 35,136,480, the magnitude reaches 3.2e20 by x ~ 3.2e9, and at
// x >= 3,373,259,520 the reduction's float->int32 cast leaves the destination's
// range -- UNDEFINED BEHAVIOUR, resolving to INT_MIN on x86 and INT_MAX on
// ARM64. A file whose entire purpose is that two platforms agree bit for bit
// cannot contain that construct, and world/sdf.cpp's heightfield swap put
// unbounded arguments in front of it.
//
// kMaxReducibleAngle and this test are the fix and its proof. The properties
// asserted are the ones a caller can actually rely on out there -- BOUNDED,
// TOTAL, and PINNED -- and deliberately not accuracy, which is not on offer.
// ---------------------------------------------------------------------------
TEST(Fp32SinCos, StayBoundedTotalAndPinnedAtEveryMagnitudeIncludingTheUndefinedOnes) {
    // A stride over the whole float range above the accuracy domain, up to and
    // well past both the boundedness breach and the former UB point.
    for (uint32_t b = bits_of(spade::math::kMaxAccurateAngle); b <= bits_of(3.4e38f); b += 97u) {
        for (const float x : {float_of(b), -float_of(b)}) {
            const float s = spade::math::sin32(x);
            const float c = spade::math::cos32(x);
            ASSERT_FALSE(std::isnan(s)) << "sin32 must be total; x=" << x;
            ASSERT_FALSE(std::isnan(c)) << "cos32 must be total; x=" << x;
            ASSERT_LE(std::abs(s), 1.0f) << "sin32 must be bounded by 1; x=" << x;
            ASSERT_LE(std::abs(c), 1.0f) << "cos32 must be bounded by 1; x=" << x;
        }
    }

    // At and beyond the guard the answer is PINNED, which is the property that
    // makes it portable: every platform returns these exact two floats, so no
    // digest can depend on an ISA's out-of-range conversion behaviour.
    for (const float x : {spade::math::kMaxReducibleAngle, 2.0e7f, 3.373259e9f, 3.5e9f, 1.0e30f,
                          std::numeric_limits<float>::max()}) {
        EXPECT_EQ(spade::math::sin32(x), 0.0f) << "x=" << x;
        EXPECT_EQ(spade::math::cos32(x), 1.0f) << "x=" << x;
        EXPECT_EQ(spade::math::sin32(-x), 0.0f) << "x=" << -x;
        EXPECT_EQ(spade::math::cos32(-x), 1.0f) << "x=" << -x;
        // sin^2 + cos^2 == 1 survives the guard exactly, which is why phase
        // zero was chosen over any other pinned pair.
        EXPECT_EQ(spade::math::sin32(x) * spade::math::sin32(x) +
                      spade::math::cos32(x) * spade::math::cos32(x),
                  1.0f);
    }

    // And the guard does not encroach: the float immediately BELOW it is still
    // computed, not pinned. (cos32 there is not 1 and sin32 is not 0 -- if the
    // guard ever slid down onto real arguments, this is what would notice.)
    const float just_inside = std::nextafter(spade::math::kMaxReducibleAngle, 0.0f);
    EXPECT_NE(spade::math::sin32(just_inside), 0.0f);
    EXPECT_NE(spade::math::cos32(just_inside), 1.0f);
}

TEST(Fp32SinCos, ReturnNaNForNonFiniteArguments) {
    EXPECT_TRUE(std::isnan(spade::math::sin32(std::numeric_limits<float>::infinity())));
    EXPECT_TRUE(std::isnan(spade::math::cos32(-std::numeric_limits<float>::infinity())));
    EXPECT_TRUE(std::isnan(spade::math::sin32(std::numeric_limits<float>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(spade::math::cos32(std::numeric_limits<float>::quiet_NaN())));
}

// ===========================================================================
// 4. The structural facts the determinism argument rests on
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

// The same check for the ln2 split that log32 AND exp32 share: the product of
// the split's high part with an integer multiplier must be EXACT, or both
// functions' leading terms quietly acquire a rounding.
//
// The multiplier ranges are different and the test covers their union with a
// margin either side:
//   * log32 multiplies by a float's EXPONENT, e in [-149, 127] (the -149 end
//     is a subnormal after the 2^24 pre-scale);
//   * exp32 multiplies by the QUOTIENT round(x/ln2), n in [-150, 128] over
//     its whole non-saturating domain -- the guards at +-89/-104 are what
//     bound it, which is the second thing those guards are for.
// [-151, 129] is both, plus one on each side.
TEST(Fp32Log, TheLn2SplitMultipliesExactlyAcrossEveryExponentAndEveryExpQuotient) {
    const float ln2_hi = 0x1.62e4p-1f;
    const float ln2_lo = 0x1.7f7d1cp-20f;
    for (int32_t e = -151; e <= 129; ++e) {
        const float product = static_cast<float>(e) * ln2_hi;
        EXPECT_EQ(static_cast<double>(product), static_cast<double>(e) * static_cast<double>(ln2_hi))
            << "e=" << e;
    }
    EXPECT_LT(std::abs((static_cast<double>(ln2_hi) + static_cast<double>(ln2_lo)) -
                       0.69314718055994530942),
              1e-13);
}
