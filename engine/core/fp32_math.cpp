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
//
// THE SAME PAIR SERVES exp32, and that is not a coincidence worth hiding: the
// two functions reduce against ln 2 in opposite directions, log32 with a
// multiplier |e| <= 149 (a float's exponent) and exp32 with a multiplier
// |n| <= 150 (the quotient x/ln2 over exp32's whole non-saturating domain).
// One 8-bit bound covers both, so one split covers both -- and the structural
// test in tests/test_fp32_math.cpp that checks the exactness over [-151, 129]
// covers both at once.
constexpr float kLn2Hi = 0x1.62e4p-1f;      // 0.693145751953125
constexpr float kLn2Lo = 0x1.7f7d1cp-20f;   // ln2 - kLn2Hi

// sqrt(2), the mantissa split point. Folding m into [sqrt(2)/2, sqrt(2)) rather
// than leaving it in [1, 2) halves the series argument and is what lets five
// terms suffice: |s| <= 0.1716 instead of 0.3334.
constexpr float kSqrt2 = 0x1.6a09e6p+0f;

// ===========================================================================
// exp32
// ===========================================================================

// 1/ln(2), the quotient multiplier for the range reduction. Hex, like every
// other irrational split in this file: what matters about it is a bit pattern,
// not a decimal a reader would have to trust.
//
// It is NOT part of any exactness argument, and the reason is worth stating
// carefully rather than waving at. n is rounded to an integer immediately
// afterwards, and the identity x = n*ln2 + r holds for ANY integer n -- so a
// rounding here does not make the reduction wrong, it only decides WHICH of two
// adjacent quotients an argument near a half-way point receives. Both are
// legitimate: either way |r| stays within ln2/2 plus the same rounding, well
// inside the kernel's validated range, and the reconstruction's power of two
// moves by one to match.
//
// It does NOT follow that the two choices give bit-identical answers -- they
// evaluate the polynomial at arguments ln2 apart and round differently. What
// follows is that both are inside the bound, which is all that is claimed. The
// affected window is narrow (order 1e-5 in x at the domain edge, set mostly by
// ulp(y) at |y| ~ 150, not by this constant's own 1.3e-8 relative error), and
// the exhaustive sweep in tests/test_fp32_math.cpp enumerates both sides of it
// rather than reasoning about it.
//
// That tolerance is the contrast worth drawing: the reduction accepts a rounded
// multiplier here, and the SUBTRACTION below accepts none at all.
constexpr float kInvLn2 = 0x1.715476p+0f;

// exp(r) - 1 - r, over |r| <= ln(2)/2 = 0.34658, as r^2 * P(r). Exact
// rationals, same spelling and same reason as the sine/cosine coefficients
// below: IEEE division is correctly rounded, so `1.0f / 5040.0f` folds
// identically on every implementation.
//
// SEVEN COEFFICIENTS, i.e. the series carried through r^8/8!. That is not
// "one more for luck" -- it is the same standard the trigonometric kernels
// below hold themselves to, that TRUNCATION MUST NOT BE THE ERROR TERM. The
// first dropped term is r^9/362880, which at the reduction's worst case is
// 1.99e-10 absolute against a value of 0.707, i.e. 2.8e-10 relative: 0.005 ulp,
// two and a half orders of magnitude under fp32's own 6e-8 resolution. Stopping
// one term earlier (at r^7/5040) would leave 5.2e-9 absolute, 0.12 ulp -- still
// a passing kernel, but one whose worst case is 12% truncation instead of ~0%,
// which is exactly the bookkeeping this file refuses to carry.
constexpr float kE2 = 1.0f / 2.0f;
constexpr float kE3 = 1.0f / 6.0f;
constexpr float kE4 = 1.0f / 24.0f;
constexpr float kE5 = 1.0f / 120.0f;
constexpr float kE6 = 1.0f / 720.0f;
constexpr float kE7 = 1.0f / 5040.0f;
constexpr float kE8 = 1.0f / 40320.0f;

// exp(r) for |r| <= ln(2)/2, Horner in r with the CONSTANT AND THE LINEAR TERM
// BOTH KEPT OUT of the polynomial -- `1 + (r + r2*poly)` rather than
// `1 + r*(1 + r*poly)` or `(1 + r) + r2*poly`. The grouping is the accuracy
// argument, and it is the same one sin_kernel() makes below, so it is worth
// being explicit about which of the three shapes wins and why:
//
//   * The final `1 + u` costs half an ulp OF THE RESULT no matter what. That
//     rounding is unavoidable for any implementation that forms 1 + expm1(r),
//     which is every implementation that does not carry a second limb.
//   * So the only thing left to minimise is the error already sitting in u,
//     and u should therefore be assembled at the SMALLEST scale available.
//     `r + r2*poly` has |u| <= 0.414, whose ulp is 3.0e-8; `(1 + r) + r2*poly`
//     instead rounds the intermediate 1 + r at a scale up to 1.35, whose ulp
//     is 1.2e-7 -- four times coarser, for the same number of operations.
//
// That is not a theoretical preference. Measured over the whole domain the
// two shapes differ by nearly 3x in how often they miss the correctly rounded
// answer: 0.81% for the form below, 2.37% for `(1 + r) + r2*poly`. Neither
// exceeds 1 ulp; the one below is simply the better of two passing kernels,
// and the cheaper one to defend.
[[nodiscard]] float exp_kernel(float r) noexcept {
    const float r2 = r * r;
    const float poly = kE2 + r * (kE3 + r * (kE4 + r * (kE5 + r * (kE6 + r * (kE7 + r * kE8)))));
    return 1.0f + (r + r2 * poly);
}

// 2^k as a float, from the exponent field directly. Valid for k in
// [-126, 127] (the normal exponents); exp32 splits the two tails itself.
[[nodiscard]] float pow2(int32_t k) noexcept {
    return float_of(static_cast<uint32_t>(k + 127) << 23);
}

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
    // ---- THE BOUNDEDNESS GUARD (S5 Task 1, review round 1) ----------------
    //
    // WHY IT EXISTS. Everything below degrades GRACEFULLY with |x| -- right up
    // until it does not. The reduction's phase error is about half an ulp of
    // n*(pi/2), i.e. ~|x| * 2^-24 radians, so it reaches ONE RADIAN at
    // |x| ~ 2^24. Past that, r can leave the kernels' |r| <= pi/4 validity and
    // the polynomials stop being bounded by 1 at all: measured, |cos32| first
    // exceeds 1 at x = 27,107,084 and |sin32| at 35,136,480, and by x ~ 3.2e9
    // the magnitude has reached 3.2e20.
    //
    // AND THEN IT GETS WORSE, IN THE ONE WAY THIS MODULE CANNOT TOLERATE. At
    // |x| >= 3,373,259,520 (INT32_MAX / kTwoOverPi) the float->int32 cast below
    // is out of the destination type's range, which is UNDEFINED BEHAVIOUR
    // ([conv.fpint]/1) -- and undefined in the platform-DIVERGENT way: x86
    // yields INT_MIN, ARM64 saturates to INT_MAX. A file whose entire purpose
    // is bit-portability cannot host a construct whose answer depends on the
    // instruction set.
    //
    // WHY IT WAS LATENT UNTIL NOW. Before this task the only caller was
    // rng.hpp's Box-Muller angle, always in [0, 2*pi]. world/sdf.cpp's
    // heightfield -- swapped to sin32 in this same commit -- passes
    // freq * coordinate, which is unbounded. This guard is what makes that swap
    // sound instead of an exchange of one portability defect for another.
    //
    // WHAT IT RETURNS, AND THE HONEST COST. Phase zero: sin32 -> 0, cos32 -> 1.
    // Defined, bounded, identical on every conforming platform, and
    // sin^2 + cos^2 == 1 still holds. It is NOT accurate -- it is not an
    // approximation of anything -- and there IS a discontinuity at the
    // boundary. Both of those are strict improvements on an unbounded,
    // ISA-dependent value. An accurate answer out here would need a
    // Payne-Hanek reduction against a multi-word 2/pi, which is a great deal of
    // machinery for a regime where THE ARGUMENT ITSELF has stopped carrying a
    // phase: at 2^24 a float's ulp is 2 radians, so consecutive representable
    // inputs are already a third of a period apart.
    //
    // The threshold sits BELOW both measured breach points (16.8e6 against
    // 27.1e6) rather than at them, because it marks where the REASONING stops
    // being sound, not where the symptom happens to become visible.
    //
    // Spelled as a negated range test so a NaN would take this exit too --
    // sin32/cos32 reject non-finite arguments before calling, so it cannot
    // happen today, and that is precisely why this should not depend on them.
    if (!(x > -kMaxReducibleAngle && x < kMaxReducibleAngle)) {
        return Reduction{0.0f, 0u};
    }

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

float exp32(float x) noexcept {
    // Edge cases first, and the NaN test is spelled against the BIT PATTERN
    // for the same reason log32's is: a NaN fails every comparison, so it has
    // to be classified rather than allowed to fall through. A NaN returns
    // ITSELF, not a fresh quiet NaN -- std::exp propagates the payload and so
    // does this.
    const uint32_t ix = bits_of(x);
    if ((ix & 0x7FFFFFFFu) > 0x7F800000u) return x;

    // THE TWO SATURATION GUARDS, and why they are at +-89/-104 rather than at
    // the true thresholds 88.7228394 and -103.972084.
    //
    // They are not the thresholds; they are the range in which the arithmetic
    // below is TRUSTED TO SATURATE ON ITS OWN. Between 88.7228394 and 89 the
    // reconstruction genuinely overflows to +infinity, and between -103.972084
    // and -104 it genuinely rounds to +0 -- correctly, and by the same single
    // rounding a correctly rounded exp would use, so routing those arguments
    // through the arithmetic is not a concession, it is what makes the
    // boundary EXACT instead of a hand-placed cutoff one ulp out of place.
    //
    // What the guards actually buy is the bound |n| <= 150 that the whole
    // exactness argument rests on, plus totality on the infinities (+inf
    // compares >= 89, -inf compares <= -104, so neither reaches the cast).
    if (x >= 89.0f) return std::numeric_limits<float>::infinity();
    if (x <= -104.0f) return 0.0f;

    // RANGE REDUCTION: x = n*ln2 + r, |r| <= ln2/2, n an integer.
    //
    // Rounded to the nearest integer with a truncating cast of y +- 0.5 rather
    // than std::rint, for the three reasons reduce_quadrant() gives below: the
    // cast does not consult the dynamic rounding mode, it is not a libm call,
    // and integer conversion of a float is exactly specified by the language.
    const float y = x * kInvLn2;
    const float n = static_cast<float>(static_cast<int32_t>(y >= 0.0f ? y + 0.5f : y - 0.5f));

    // The two subtractions are separate statements because that grouping IS
    // the accuracy argument, exactly as in reduce_quadrant():
    //
    //   * `x - n*kLn2Hi` is EXACT, twice over. n*kLn2Hi is an exact product
    //     (kLn2Hi's 15 significant bits plus |n| <= 150's 8 = 23 <= 24), and
    //     the subtraction itself is exact by Sterbenz: x lies within 0.347 of
    //     n*ln2, so for |n| >= 1 the two operands differ by less than a factor
    //     of 1.5, and for n == 0 the subtrahend is zero.
    //   * `- n*kLn2Lo` is the only rounding in the reduction, at the scale of
    //     r itself (<= 1.5e-8 absolute), where it costs about a fifth of an
    //     ulp of the answer rather than the full ulp it would cost if the
    //     correction were folded in at the answer's scale instead.
    float r = x - n * kLn2Hi;
    r = r - n * kLn2Lo;

    // RECONSTRUCTION: exp(x) = 2^n * exp(r). Multiplying by a power of two is
    // EXACT whenever the product is normal, so the kernel's error is the whole
    // error -- there is no reconstruction rounding to account for at all in
    // the normal range. That is the entire reason for reducing modulo ln 2:
    // the reduction's quotient lands on the one operation IEEE gives away free.
    const float m = exp_kernel(r);
    const int32_t k = static_cast<int32_t>(n);

    if (k >= -126 && k <= 127) {
        return m * pow2(k);
    }
    if (k > 127) {
        // k == 128 only, and the result may still be finite: m < 1 whenever
        // r < 0, and 1.0 * 2^128 is the first value that is not. Split so the
        // intermediate stays representable and let the second multiply
        // overflow to +infinity on its own if the answer really is above
        // FLT_MAX -- a hand-rolled overflow test here would be a second
        // opinion about where FLT_MAX is.
        return (m * pow2(127)) * pow2(k - 127);
    }
    // k in [-150, -127]: the result is subnormal or zero. Scaled in two exact
    // steps so the SINGLE rounding happens in the last multiply, where it is
    // the correct rounding to the subnormal lattice. m * 2^(k+64) is normal
    // for every k in range (its exponent lands in [-87, -62]), so that first
    // multiply is exact and the second is the only one that rounds.
    return (m * pow2(k + 64)) * 0x1.0p-64f;
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
