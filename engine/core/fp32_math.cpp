#include "core/fp32_math.hpp"

#include <cstdint>
#include <cstring>
#include <limits>

namespace spade::math {

namespace {

// Bit-level access. std::memcpy rather than a union or a reinterpret_cast: it
// is the only spelling that is not strict-aliasing UB, and every compiler
// lowers it to a register move.
[[nodiscard]] uint32_t bits_of(float x) noexcept {
    uint32_t u = 0;
    std::memcpy(&u, &x, sizeof(u));
    return u;
}

[[nodiscard]] float float_of(uint32_t u) noexcept {
    float x = 0.0f;
    std::memcpy(&x, &u, sizeof(x));
    return x;
}

// ===========================================================================
// log32
// ===========================================================================

// ln(2), split so that `e * kLn2Hi` is EXACT for every exponent this function
// can produce. kLn2Hi carries 15 significant bits (its low 9 mantissa bits are
// zero, which is what the hex spelling makes visible); |e| <= 149 needs 8; and
// 15 + 8 = 23 fits float's 24-bit significand with a bit to spare. So the
// dominant term of the reconstruction below carries NO rounding error at all,
// and kLn2Lo carries the remainder (the pair reproduces ln 2 to 5.5e-14, i.e.
// far below fp32's reach).
constexpr float kLn2Hi = 0x1.62e4p-1f;      // 0.693145751953125
constexpr float kLn2Lo = 0x1.7f7d1cp-20f;   // ln2 - kLn2Hi

// sqrt(2), the mantissa split point. Folding m into [sqrt(2)/2, sqrt(2)) rather
// than leaving it in [1, 2) halves the series argument and is what lets five
// terms suffice: |s| <= 0.1716 instead of 0.3334.
constexpr float kSqrt2 = 0x1.6a09e6p+0f;

// ===========================================================================
// sin32 / cos32
// ===========================================================================

// 2/pi, the quadrant-index multiplier.
constexpr float kTwoOverPi = 0x1.45f306p-1f;

// pi/2 in THREE parts, Cody-Waite. The point of the split is the trailing
// zeros: kPio2A carries 18 significant bits and kPio2B carries 18, so for any
// quadrant index |n| <= 63 (6 bits) both `n * kPio2A` and `n * kPio2B` are
// EXACT products -- 18 + 6 = 24. kMaxAccurateAngle (100) is well inside that
// bound (|n| <= 64), which is what makes the domain claim in the header a
// property of these constants rather than an assertion.
//
// The three parts sum to pi/2 with a residual below 1e-19, so the reduction
// loses nothing to the split itself.
constexpr float kPio2A = 0x1.921f8p+0f;    // 1.5707931518554688
constexpr float kPio2B = 0x1.aa22p-19f;    // 3.1749368645250797e-06
constexpr float kPio2C = 0x1.68cp-39f;     // 2.5632829192545614e-12

// sin(r) / r and cos(r) Taylor coefficients, |r| <= pi/4. Spelled as exact
// rationals (see the header's note on constant spelling): IEEE division is
// correctly rounded, so these fold identically everywhere.
//
// Both series are carried far enough that TRUNCATION is not the error term --
// the first dropped term is ~2e-9 relative for sine and ~1.4e-10 for cosine,
// an order of magnitude under fp32's own 6e-8 resolution -- so what the
// exhaustive test in tests/test_fp32_math.cpp measures is rounding, which is
// the only thing left to measure.
constexpr float kS1 = -1.0f / 6.0f;
constexpr float kS2 = 1.0f / 120.0f;
constexpr float kS3 = -1.0f / 5040.0f;
constexpr float kS4 = 1.0f / 362880.0f;
constexpr float kS5 = -1.0f / 39916800.0f;

constexpr float kC1 = -1.0f / 2.0f;
constexpr float kC2 = 1.0f / 24.0f;
constexpr float kC3 = -1.0f / 720.0f;
constexpr float kC4 = 1.0f / 40320.0f;
constexpr float kC5 = -1.0f / 3628800.0f;

// sin(r) for |r| <= pi/4, Horner in r^2 with the linear term kept OUT of the
// polynomial -- `r + r*(r2*poly)` rather than `r*(1 + r2*poly)` -- so that
// sin32(r) is exactly r for small enough r and the relative error near zero is
// the relative error of the correction, not of the whole value.
[[nodiscard]] float sin_kernel(float r) noexcept {
    const float r2 = r * r;
    const float poly = kS1 + r2 * (kS2 + r2 * (kS3 + r2 * (kS4 + r2 * kS5)));
    return r + r * (r2 * poly);
}

// cos(r) for |r| <= pi/4. Same shape, with the constant 1 kept out of the
// polynomial for the same reason.
[[nodiscard]] float cos_kernel(float r) noexcept {
    const float r2 = r * r;
    const float poly = kC1 + r2 * (kC2 + r2 * (kC3 + r2 * (kC4 + r2 * kC5)));
    return 1.0f + r2 * poly;
}

// The reduced argument r = x - n*(pi/2) and its quadrant index n (mod 4).
struct Reduction {
    float r = 0.0f;
    uint32_t quadrant = 0;
};

// Cody-Waite reduction. The three subtractions are spelled left to right and
// each is its own statement, because that grouping IS the accuracy argument:
// `x - n*kPio2A` is exact (Sterbenz -- the two operands are within a factor of
// two of each other whenever n != 0, and the n == 0 case is a no-op), so the
// only rounding in the whole reduction is the two tiny corrections.
//
// ROUNDING TO THE NEAREST QUADRANT is done with a truncating cast of y + 0.5
// rather than std::rint or std::nearbyint. Three reasons, all load-bearing:
// the cast does not consult the dynamic rounding mode, it is not a libm call
// (which is the entire subject of this file), and integer conversion of a
// float is exactly specified by the language.
[[nodiscard]] Reduction reduce_quadrant(float x) noexcept {
    const float y = x * kTwoOverPi;
    // Round-half-away-from-zero. Which way a tie breaks is immaterial: both
    // choices leave |r| <= pi/4 + eps, which is all the kernels require.
    const float n = static_cast<float>(static_cast<int32_t>(y >= 0.0f ? y + 0.5f : y - 0.5f));

    float r = x - n * kPio2A;
    r = r - n * kPio2B;
    r = r - n * kPio2C;

    // The quadrant index, mod 4, as an unsigned. Adding a multiple of 4 before
    // the mask keeps a negative n's index in [0, 3] without a branch and
    // without relying on the sign of C++'s integer remainder.
    const int32_t ni = static_cast<int32_t>(n);
    return Reduction{r, static_cast<uint32_t>(ni) & 3u};
}

}  // namespace

float log32(float x) noexcept {
    const uint32_t ix = bits_of(x);

    // Edge cases first, spelled against the BIT PATTERN so that NaN -- which
    // fails every comparison -- is classified rather than falling through.
    if (ix == 0x00000000u || ix == 0x80000000u) {
        return -std::numeric_limits<float>::infinity();  // +-0
    }
    if ((ix & 0x80000000u) != 0u) {
        return std::numeric_limits<float>::quiet_NaN();  // negative (incl. -inf)
    }
    if (ix >= 0x7F800000u) {
        // +infinity returns +infinity; a NaN returns itself.
        return x;
    }

    // Decompose x = m * 2^e with m in [1, 2). A subnormal has a zero exponent
    // field, so it is scaled into the normal range first by an EXACT multiply
    // by 2^24 and the exponent is paid back afterwards.
    uint32_t mx = ix;
    int32_t e = 0;
    if ((ix >> 23) == 0u) {
        mx = bits_of(x * 0x1.0p24f);
        e = -24;
    }
    e += static_cast<int32_t>(mx >> 23) - 127;
    const float m = float_of((mx & 0x007FFFFFu) | 0x3F800000u);

    // Fold into [sqrt(2)/2, sqrt(2)) so |s| <= 0.1716. The halving is exact
    // (a power of two) and the exponent absorbs it.
    float mm = m;
    if (mm > kSqrt2) {
        mm = mm * 0.5f;
        e += 1;
    }

    // THE FORMULATION, AND WHY IT IS NOT THE OBVIOUS ONE.
    //
    // The textbook route is ln(mm) = 2*atanh(s) with s = (mm-1)/(mm+1),
    // evaluated as 2s plus a series correction. It is accurate in exact
    // arithmetic and it is WRONG HERE, for a reason an exhaustive sweep makes
    // very concrete: 2s is the LEADING term, and s is a quotient, so s carries
    // a rounding of its own that arrives in the answer at full size. Measured
    // over all 2^24 arguments that formulation peaks at 2 ulp, and no amount
    // of compensating the sum that follows can remove an error that is already
    // in the leading term.
    //
    // So the leading term is chosen to be one that is EXACT instead. With
    //     f = mm - 1                (exact: Sterbenz, since 0.5 <= mm <= 2)
    //     s = f / (2 + f)           (== (mm-1)/(mm+1), same s as before)
    //     R = 2*(s^2/3 + s^4/5 + s^6/7 + s^8/9)
    // the identity
    //     ln(1 + f) = f - f^2/2 + s*(f^2/2 + R)
    // holds exactly -- substituting f = 2s/(1-s) collapses the right-hand side
    // to 2s + s*R, which is 2*atanh(s) -- and now f, not 2s, is what carries
    // the answer. Everything else is a correction of relative size ~1e-2 or
    // smaller, so s's own rounding is diluted by that factor and contributes
    // ~0.01 ulp instead of ~1. This is fdlibm's arrangement, and this is the
    // reason for it.
    const float f = mm - 1.0f;  // EXACT -- the whole point
    const float s = f / (2.0f + f);
    const float z = s * s;

    // R, Horner in z. Four terms: the first dropped one is 2*z^5/11 ~ 4e-9
    // RELATIVE TO A CORRECTION that is itself ~1e-2 of the answer, i.e. ~4e-11
    // of the answer -- three orders below fp32's reach.
    const float R = z * ((2.0f / 3.0f) +
                         z * ((2.0f / 5.0f) + z * ((2.0f / 7.0f) + z * (2.0f / 9.0f))));
    const float hfsq = 0.5f * f * f;

    // RECONSTRUCTION. ef * kLn2Hi is exact (see kLn2Hi) and f is exact, and
    // the grouping is written so those two meet in ONE addition, with every
    // small quantity already folded into the subtrahend beforehand. That
    // single rounding at the answer's scale is the theoretical minimum for a
    // result that is a sum of two exact terms.
    const float ef = static_cast<float>(e);
    return ef * kLn2Hi - ((hfsq - (s * (hfsq + R) + ef * kLn2Lo)) - f);
}

float sin32(float x) noexcept {
    const uint32_t ix = bits_of(x) & 0x7FFFFFFFu;
    if (ix >= 0x7F800000u) return std::numeric_limits<float>::quiet_NaN();  // +-inf, NaN

    const Reduction red = reduce_quadrant(x);
    switch (red.quadrant) {
        case 0:
            return sin_kernel(red.r);
        case 1:
            return cos_kernel(red.r);
        case 2:
            return -sin_kernel(red.r);
        default:
            return -cos_kernel(red.r);
    }
}

float cos32(float x) noexcept {
    const uint32_t ix = bits_of(x) & 0x7FFFFFFFu;
    if (ix >= 0x7F800000u) return std::numeric_limits<float>::quiet_NaN();  // +-inf, NaN

    const Reduction red = reduce_quadrant(x);
    switch (red.quadrant) {
        case 0:
            return cos_kernel(red.r);
        case 1:
            return -sin_kernel(red.r);
        case 2:
            return -cos_kernel(red.r);
        default:
            return sin_kernel(red.r);
    }
}

}  // namespace spade::math
