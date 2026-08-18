// ---------------------------------------------------------------------------
// fp32_math.slang -- the Slang port of engine/core/fp32_math.cpp. THE PARITY
// FOUNDATION: every later GPU kernel that needs a logarithm, an exponential or
// a sine calls these, never a vendor transcendental intrinsic (global
// constraint, "GPU corollary": "kernels never call GPU-vendor transcendental
// intrinsics on parity paths -- they call the Slang ports of
// log32/exp32/sin32/cos32").
//
// THE STANDARD THIS FILE IS HELD TO IS BIT-IDENTITY, NOT A TOLERANCE. Every
// argument the CPU twin can be handed must produce the SAME 32 BITS here.
// tests/test_gpu_fp32_math.cpp is what says so, on the real device, over the
// deciding windows of tests/test_fp32_math.cpp, all 2^24 Box-Muller angles,
// the corpus-live exp32 arguments and every edge case -- compared as uint32
// bit patterns, zero mismatches tolerated.
//
// ===========================================================================
// HOW THE PORT IS WRITTEN, AND THE FOUR RULES IT OBEYS
//
//   1. EXACT OP ORDER. Every statement below is the same statement, in the
//      same order, with the same parenthesisation, as the C++ line it cites.
//      A C++ expression written as three separate statements is three separate
//      statements here; a grouping that IS an accuracy argument over there is
//      the identical grouping over here. Each function carries the
//      fp32_math.cpp LINE RANGE it was ported from, so a reviewer can diff the
//      two side by side rather than trust this sentence.
//
//   2. NO CONTRACTION. cmake/SpadeSlang.cmake compiles every kernel
//      -fp-mode precise, which decorates every contractable float op in the
//      module NoContraction -- the module-level equivalent of the CPU side's
//      /fp:precise + -ffp-contract=off. tests/test_slang_layouts.cpp's
//      SlangSpirv.FloatControlsPinned holds THIS module to the EXACT profile
//      (spirv_scan.hpp): NoContraction everywhere, no sum-of-products opcode,
//      no OpFDiv, no sqrt.
//
//   3. NO OpFDiv, NO SQRT. Vulkan's division and square root are <= 2.5 ulp,
//      NOT correctly rounded, so neither may appear on a path that claims
//      bit-identity with an IEEE-correct CPU. The DIVISION AUDIT is recorded
//      in full below -- it is the one place this port could not simply follow
//      the CPU text, and it is the finding this file most wants a reader to
//      see.
//
//      SQRT AUDIT: there is NO square root in any of the four kernels. Checked
//      by reading fp32_math.cpp end to end -- `kSqrt2` is a LITERAL CONSTANT
//      (the mantissa split point), not a call, and core/fp32_math.hpp's own
//      "WHAT IS *NOT* HERE" note states the policy ("std::sqrt STAYS ... IEEE
//      754 mandates a correctly rounded square root"). rng.hpp's
//      `std::sqrt(-2 * log32(u1))` is Box-Muller's, OUTSIDE these kernels, and
//      is a later task's problem. So rule E2 is satisfied by absence, not by
//      respelling, and that is stated rather than left to be inferred.
//
//   4. CONSTANTS ARE SPELLED EXACTLY AS THE CPU SPELLS THEM -- exact rationals
//      (`1.0f / 5040.0f`) for the polynomial coefficients, hex float literals
//      (`0x1.62e4p-1f`) for the irrational splits, matching core/fp32_math.hpp's
//      "CONSTANTS, AND WHY THEY ARE SPELLED THE WAY THEY ARE" note verbatim.
//      This is a MEASURED decision, not an assumption that slangc folds like
//      MSVC: slang v2026.14.1 folds `1.0f / 5040.0f` to 0x39500D01, which is
//      the correctly rounded float and the same bit pattern MSVC produces, and
//      emits NO OpFDiv for it (transcript in this task's report). The
//      GpuFp32Math.KernelConstantsAreBitIdenticalToTheHostConstants battery
//      re-checks every one of them ON THE DEVICE against the C++ expression
//      transcribed independently in the test, so the fold is proved per build
//      rather than remembered.
//
// ===========================================================================
// THE DIVISION AUDIT -- READ THIS BEFORE THE CODE
//
// The task brief's instruction was "respell any `x / c` as `x * (1.0f/c)` with
// the inverse as a correctly-rounded literal". Across the four kernels there
// is EXACTLY ONE division, and it is not of that shape:
//
//      fp32_math.cpp:337   const float s = f / (2.0f + f);
//
// The divisor is a RUNTIME VALUE, not a constant, so there is no inverse to
// precompute and no multiply that reproduces it. (The only other `/` tokens in
// the file are the constant expressions `1.0f / 6.0f` and friends, which are
// compile-time folds on both sides -- rule 4 above -- and emit no OpFDiv.)
//
// That division is also LOAD-BEARING FOR ACCURACY in a way that forbids
// approximating it. fp32_math.cpp:313-335 records why log32 uses this exact
// formulation: the textbook `2*atanh(s)` route peaks at 2 ulp precisely
// BECAUSE s carries a division's rounding into the leading term, and fdlibm's
// arrangement -- the one ported here -- survives that only because s's rounding
// arrives diluted by ~1e-2. A 2.5-ulp OpFDiv would put ~2.5x that error back
// into s. And, more decisively for this program: an OpFDiv is not merely less
// accurate, it is DEVICE-DEPENDENT, so log32 would stop being bit-identical at
// all.
//
// SO THE DIVISION IS IMPLEMENTED EXACTLY, in integer arithmetic: log32_div()
// below is a restoring long division on the significands that produces the
// correctly-rounded (round-to-nearest, ties-to-even) quotient IEEE 754 requires
// -- i.e. bit-for-bit what MSVC's `divss` returns. It contains no float
// arithmetic at all, so it adds nothing for the NoContraction rule to check and
// nothing for the OpFDiv rule to find. The GPU battery
// GpuFp32Math.CorrectlyRoundedDivisionMatchesTheHostDivide proves the claim
// against the host `/` over every (f, 2+f) pair log32 can form, and the four
// log32 batteries prove it again end to end.
//
// THIS IS A DEVIATION FROM THE BRIEF'S LETTER AND AN OBEDIENCE TO ITS INTENT,
// and it is flagged here rather than buried: the brief anticipated only
// divisions by constants.
// ===========================================================================

// ---------------------------------------------------------------------------
// Bit-level access. The C++ side uses std::memcpy (fp32_math.cpp:11-24) because
// it is the only spelling that is not strict-aliasing UB there; Slang's
// asuint/asfloat are the same reinterpretation, compile to OpBitcast, and have
// no aliasing question to answer.
// ---------------------------------------------------------------------------

// Non-finite bit patterns, spelled as patterns rather than as expressions.
// `1.0f / 0.0f` would be an OpFDiv (rule 3) and `0.0f / 0.0f` a second one;
// these are what std::numeric_limits<float>::infinity() and ::quiet_NaN()
// ARE on the host, and the edge-case battery compares them bit for bit.
static const uint kBitsPosInf = 0x7F800000u;
static const uint kBitsNegInf = 0xFF800000u;
static const uint kBitsQNaN = 0x7FC00000u;  // std::numeric_limits<float>::quiet_NaN()

// ===========================================================================
// log32_div -- correctly-rounded fp32 division, without OpFDiv
// ===========================================================================
//
// THE NAME CARRIES THE PURPOSE ON PURPOSE. This is not a general-purpose
// `div32` and must not be reached for as one: it is log32's divide, with a
// PRECONDITION its caller satisfies and which IT DOES NOT CHECK. Out of domain
// it fails SILENTLY, not loudly -- measured: 1.0/0.0 returns 1.7e38 rather than
// +infinity, and a NaN numerator returns a finite number. A name like `div32`
// would advertise a contract this function does not honour, and this module is
// importable by every later kernel, so the name is the fence.
//
//      *** PRECONDITION -- THE CALLER MUST GUARANTEE ALL FOUR ***
//        1. b is finite, NORMAL and non-zero;
//        2. a is either exactly +-0, or finite and NORMAL;
//        3. neither operand is an infinity or a NaN;
//        4. the exact quotient is in the NORMAL range -- no overflow, no
//           subnormal result.
//      Outside that domain the answer is meaningless, not merely inaccurate.
//
// WHAT IT COMPUTES INSIDE THE DOMAIN: fl(a / b) under round-to-nearest,
// ties-to-even -- the result IEEE 754 mandates and every conforming `/`
// produces, including the host's. Not an approximation of it: the long division
// below produces the infinitely-precise quotient's leading 25 bits (24-bit
// significand + round bit) plus an exact sticky flag, so the rounding decision
// it makes is the same decision the hardware makes.
//
// WHY log32'S CALL SITE IS INSIDE THE DOMAIN, derived rather than asserted: mm
// lies in [sqrt(2)/2, sqrt(2)), so f = mm - 1 is either exactly 0 or has
// |f| >= 2^-24 (normal -- a float's smallest normal is 2^-126), and the divisor
// 2 + f lies in [1.707, 2.415] (normal, non-zero, finite). log32 has already
// returned for every non-finite and non-positive x before reaching here, so (3)
// holds too. The quotient's magnitude is therefore either 0 or within
// [2^-24 / 2.415, 0.4142 / 1.707] = [2.4e-8, 0.2426] -- comfortably normal.
//
// THE TIES-TO-EVEN LEG IS UNREACHABLE FOR DIVISION, and that is a theorem, not
// an untested corner: a tie needs the exact quotient to have exactly 25
// significant bits, i.e. ma * 2^j = mb * N with N odd and N >= 2^24; taking odd
// parts gives odd(ma) = odd(mb) * N >= 2^24 > ma, a contradiction. So no
// division of two normal floats ever lands halfway, the `(mant & 1u)` term
// below never decides anything, and the battery owes it no coverage. It is kept
// because removing it would make the routine wrong for any other use and right
// only by an argument the next reader would have to reconstruct.
//
// WHY LONG DIVISION AND NOT A NEWTON-RAPHSON REFINEMENT. The Newton route
// needs an exactly-computed residual to correct its last bit, which without a
// trusted FMA means Dekker two-products (contraction is pinned OFF here, and
// GLSL.std.450 Fma's precision is inherited rather than mandated, so neither
// is available as a primitive to build a proof on). The integer form needs no
// float primitive at all, and its correctness is one invariant plus one
// pre-step -- both of which are stated in the code below, because the pre-step
// is exactly what a first draft of this function got wrong.
float log32_div(float a, float b) {
    const uint ua = asuint(a);
    const uint ub = asuint(b);
    const uint sign = (ua ^ ub) & 0x80000000u;

    // +-0 / b -> +-0 with the sign of the quotient. The only special case the
    // validated domain admits, and log32 reaches it whenever x is an exact
    // power of two (mm == 1, f == 0).
    if ((ua & 0x7FFFFFFFu) == 0u) {
        return asfloat(sign);
    }

    const int ea = int((ua >> 23) & 0xFFu);
    const int eb = int((ub >> 23) & 0xFFu);
    const uint ma = (ua & 0x007FFFFFu) | 0x00800000u;  // 24-bit significand
    const uint mb = (ub & 0x007FFFFFu) | 0x00800000u;

    // ---- THE PRE-STEP, AND THE INVARIANT IT ESTABLISHES -------------------
    //
    // Restoring long division subtracts the divisor AT MOST ONCE per step, so
    // it is only correct while the running remainder is SMALLER than the
    // divisor. Both significands live in [2^23, 2^24), so ma/mb lands in
    // (0.5, 2) and `rem = ma` violates that precondition for exactly the half
    // of all inputs with ma >= mb.
    //
    // THIS IS NOT A HYPOTHETICAL. A first draft of this function started the
    // loop at `rem = ma` unconditionally; on the ma >= mb half the remainder
    // grew without bound instead of staying bounded, and the GPU battery
    // reported 3,714,649 mismatches out of 8,388,608 -- 44%, i.e. precisely
    // the fraction of log32's call-site pairs with ma >= mb. The battery is in
    // this task's report with that number, because a test that only ever
    // printed zeros would not have earned the claim it makes.
    //
    // The fix is the classic one: take the leading quotient bit BEFORE the
    // loop, which both restores the invariant (rem < mb from then on, so
    // `rem << 1 < 2^25` and the uint never overflows) and tells the
    // reconstruction which binade the quotient is in.
    //
    //   ma >= mb  ->  quotient in [1, 2):   leading bit 1, 24 more bits needed
    //   ma <  mb  ->  quotient in (0.5, 1): leading bit 0, 25 bits needed
    //
    // Either way q ends up with exactly 25 significant bits -- the 24-bit
    // significand plus the round bit -- and `rem != 0` is the exact sticky.
    uint rem = ma;
    uint q = 0u;
    int bias = 126;
    int steps = 25;
    if (ma >= mb) {
        rem = ma - mb;
        q = 1u;
        bias = 127;
        steps = 24;
    }

    for (int i = 0; i < steps; ++i) {
        rem = rem << 1;
        q = q << 1;
        if (rem >= mb) {
            rem -= mb;
            q |= 1u;
        }
    }

    uint mant = q >> 1;
    const uint guard = q & 1u;
    const uint sticky = (rem != 0u) ? 1u : 0u;
    int bexp = ea - eb + bias;

    // Round to nearest, ties to even.
    if (guard != 0u && (sticky != 0u || (mant & 1u) != 0u)) {
        mant += 1u;
        if (mant == (1u << 24)) {
            mant = mant >> 1;
            bexp += 1;
        }
    }

    return asfloat(sign | (uint(bexp) << 23) | (mant & 0x007FFFFFu));
}

// ===========================================================================
// log32  -- ported from fp32_math.cpp:26-51 (constants) and :277-354 (body)
// ===========================================================================

// ln(2), split so that `e * kLn2Hi` is EXACT for every exponent log32 can
// produce and every quotient exp32 can produce (fp32_math.cpp:30-46). Same
// pair serves both functions, exactly as it does on the host.
static const float kLn2Hi = 0x1.62e4p-1;     // fp32_math.cpp:45  0.693145751953125
static const float kLn2Lo = 0x1.7f7d1cp-20;  // fp32_math.cpp:46  ln2 - kLn2Hi

// sqrt(2), the mantissa split point (fp32_math.cpp:48-51). A LITERAL, not a
// call -- see this file's sqrt audit.
static const float kSqrt2 = 0x1.6a09e6p+0;  // fp32_math.cpp:51

// ===========================================================================
// exp32 -- ported from fp32_math.cpp:53-134 (constants + kernels) and
//          :356-432 (body)
// ===========================================================================

// 1/ln(2), the quotient multiplier for the range reduction (fp32_math.cpp:80).
static const float kInvLn2 = 0x1.715476p+0;

// exp(r) - 1 - r over |r| <= ln(2)/2, as r^2 * P(r) (fp32_math.cpp:96-102).
// Exact rationals, folded by slangc to the same bits MSVC folds them to.
static const float kE2 = 1.0f / 2.0f;
static const float kE3 = 1.0f / 6.0f;
static const float kE4 = 1.0f / 24.0f;
static const float kE5 = 1.0f / 120.0f;
static const float kE6 = 1.0f / 720.0f;
static const float kE7 = 1.0f / 5040.0f;
static const float kE8 = 1.0f / 40320.0f;

// fp32_math.cpp:124-128. The grouping `1 + (r + r2*poly)` is the accuracy
// argument (see :104-123 there); it is reproduced statement for statement.
float exp_kernel(float r) {
    const float r2 = r * r;
    const float poly = kE2 + r * (kE3 + r * (kE4 + r * (kE5 + r * (kE6 + r * (kE7 + r * kE8)))));
    return 1.0f + (r + r2 * poly);
}

// 2^k as a float, from the exponent field directly (fp32_math.cpp:130-134).
// Valid for k in [-126, 127]; exp32 splits the two tails itself.
float pow2(int k) {
    return asfloat(uint(k + 127) << 23);
}

// ===========================================================================
// sin32 / cos32 -- ported from fp32_math.cpp:136-193 (constants + kernels),
//                  :195-273 (reduction) and :434-466 (bodies)
// ===========================================================================

// 2/pi, the quadrant-index multiplier (fp32_math.cpp:141).
static const float kTwoOverPi = 0x1.45f306p-1;

// pi/2 in THREE parts, Cody-Waite (fp32_math.cpp:143-154). The trailing zeros
// are the whole correctness argument; hex is what makes them visible.
static const float kPio2A = 0x1.921f8p+0;  // 1.5707931518554688
static const float kPio2B = 0x1.aa22p-19;  // 3.1749368645250797e-06
static const float kPio2C = 0x1.68cp-39;   // 2.5632829192545614e-12

// sin(r)/r and cos(r) Taylor coefficients, |r| <= pi/4 (fp32_math.cpp:165-175).
static const float kS1 = -1.0f / 6.0f;
static const float kS2 = 1.0f / 120.0f;
static const float kS3 = -1.0f / 5040.0f;
static const float kS4 = 1.0f / 362880.0f;
static const float kS5 = -1.0f / 39916800.0f;

static const float kC1 = -1.0f / 2.0f;
static const float kC2 = 1.0f / 24.0f;
static const float kC3 = -1.0f / 720.0f;
static const float kC4 = 1.0f / 40320.0f;
static const float kC5 = -1.0f / 3628800.0f;

// core/fp32_math.hpp:198 -- the DEFINEDNESS bound. Mirrored here because it is
// a behavioural pin (sin32 -> 0, cos32 -> 1 at and beyond it) that the GPU port
// must land on the same float.
static const float kMaxReducibleAngle = 0x1.0p24;  // 16777216

// fp32_math.cpp:177-185. The linear term is kept OUT of the polynomial so that
// sin32(r) is exactly r for small enough r.
float sin_kernel(float r) {
    const float r2 = r * r;
    const float poly = kS1 + r2 * (kS2 + r2 * (kS3 + r2 * (kS4 + r2 * kS5)));
    return r + r * (r2 * poly);
}

// fp32_math.cpp:187-193.
float cos_kernel(float r) {
    const float r2 = r * r;
    const float poly = kC1 + r2 * (kC2 + r2 * (kC3 + r2 * (kC4 + r2 * kC5)));
    return 1.0f + r2 * poly;
}

// fp32_math.cpp:195-199.
struct Reduction {
    float r;
    uint quadrant;
};

// Cody-Waite reduction -- fp32_math.cpp:201-273. The boundedness guard
// (:212-257 there) is spelled as the same negated range test, so a NaN takes
// the same exit; the three subtractions stay three separate statements because
// that grouping IS the accuracy argument.
//
// THE float->int CAST. On the host this is `static_cast<int32_t>`, which
// truncates toward zero and is UB out of range -- which is exactly what the
// guard above exists to prevent. In SPIR-V it is OpConvertFToS, which also
// truncates toward zero (and whose out-of-range behaviour is likewise
// undefined); the guard bounds |y| by 2^24 * 0.6366 = 1.07e7, four orders
// inside int32's range, so neither side ever reaches the undefined case.
Reduction reduce_quadrant(float x) {
    Reduction out;
    if (!(x > -kMaxReducibleAngle && x < kMaxReducibleAngle)) {
        out.r = 0.0f;
        out.quadrant = 0u;
        return out;
    }

    const float y = x * kTwoOverPi;
    const float n = float(int(y >= 0.0f ? y + 0.5f : y - 0.5f));

    float r = x - n * kPio2A;
    r = r - n * kPio2B;
    r = r - n * kPio2C;

    const int ni = int(n);
    out.r = r;
    out.quadrant = uint(ni) & 3u;
    return out;
}

// ===========================================================================
// THE FOUR PUBLIC KERNELS
// ===========================================================================

// fp32_math.cpp:277-354.
float log32(float x) {
    const uint ix = asuint(x);

    // Edge cases first, spelled against the BIT PATTERN so that NaN -- which
    // fails every comparison -- is classified rather than falling through.
    if (ix == 0x00000000u || ix == 0x80000000u) {
        return asfloat(kBitsNegInf);  // +-0
    }
    if ((ix & 0x80000000u) != 0u) {
        return asfloat(kBitsQNaN);  // negative (incl. -inf)
    }
    if (ix >= 0x7F800000u) {
        return x;  // +infinity returns +infinity; a NaN returns itself
    }

    // Decompose x = m * 2^e with m in [1, 2). A subnormal is scaled into the
    // normal range first by an EXACT multiply by 2^24.
    //
    // THIS MULTIPLY IS THE ONE PLACE THIS MODULE DEPENDS ON DENORMAL
    // BEHAVIOUR, and it is the reason
    // GpuFp32Math.EdgeCasesAreBitIdenticalToTheHost enumerates the subnormal
    // boundary explicitly: a device that flushed fp32 denormals to zero would
    // turn this into 0 * 2^24 and answer -infinity for every subnormal. The
    // Iris preserves them (measured -- this task's report carries the
    // VkPhysicalDeviceFloatControlsProperties line and the battery's zero
    // mismatches over the subnormal window).
    uint mx = ix;
    int e = 0;
    if ((ix >> 23) == 0u) {
        mx = asuint(x * 0x1.0p24);
        e = -24;
    }
    e += int(mx >> 23) - 127;
    const float m = asfloat((mx & 0x007FFFFFu) | 0x3F800000u);

    // Fold into [sqrt(2)/2, sqrt(2)) so |s| <= 0.1716.
    float mm = m;
    if (mm > kSqrt2) {
        mm = mm * 0.5f;
        e += 1;
    }

    // The fdlibm arrangement: f, not 2s, carries the answer. See
    // fp32_math.cpp:313-335 for why the textbook 2*atanh(s) route peaks at
    // 2 ulp and this one does not.
    const float f = mm - 1.0f;  // EXACT -- the whole point
    const float s = log32_div(f, 2.0f + f);  // fp32_math.cpp:337 -- the one division; see the audit
    const float z = s * s;

    const float R = z * ((2.0f / 3.0f) +
                         z * ((2.0f / 5.0f) + z * ((2.0f / 7.0f) + z * (2.0f / 9.0f))));
    const float hfsq = 0.5f * f * f;

    const float ef = float(e);
    return ef * kLn2Hi - ((hfsq - (s * (hfsq + R) + ef * kLn2Lo)) - f);
}

// fp32_math.cpp:356-432.
float exp32(float x) {
    const uint ix = asuint(x);
    if ((ix & 0x7FFFFFFFu) > 0x7F800000u) {
        return x;  // a NaN returns ITSELF, payload and all
    }

    // The two saturation guards at +-89/-104 (fp32_math.cpp:365-380): they are
    // not the true thresholds, they are the range in which the arithmetic
    // below is trusted to saturate on its own -- which is what makes the
    // boundary exact instead of hand-placed. They also bound |n| <= 150, which
    // is what the kLn2Hi exactness argument rests on, and they make the
    // infinities total without reaching the cast.
    if (x >= 89.0f) {
        return asfloat(kBitsPosInf);
    }
    if (x <= -104.0f) {
        return 0.0f;
    }

    const float y = x * kInvLn2;
    const float n = float(int(y >= 0.0f ? y + 0.5f : y - 0.5f));

    // Two separate statements: `x - n*kLn2Hi` is exact twice over, and
    // `- n*kLn2Lo` is the reduction's only rounding, taken at r's scale.
    float r = x - n * kLn2Hi;
    r = r - n * kLn2Lo;

    const float m = exp_kernel(r);
    const int k = int(n);

    if (k >= -126 && k <= 127) {
        return m * pow2(k);
    }
    if (k > 127) {
        // k == 128 only, and the result may still be finite. Split so the
        // intermediate stays representable and the second multiply is allowed
        // to overflow to +infinity on its own.
        return (m * pow2(127)) * pow2(k - 127);
    }
    // k in [-150, -127]: subnormal or zero. Two exact steps so the SINGLE
    // rounding happens in the last multiply, where it is the correct rounding
    // to the subnormal lattice. (Second denormal-behaviour dependency; see
    // log32's note above.)
    return (m * pow2(k + 64)) * 0x1.0p-64;
}

// fp32_math.cpp:434-449.
float sin32(float x) {
    const uint ix = asuint(x) & 0x7FFFFFFFu;
    if (ix >= 0x7F800000u) {
        return asfloat(kBitsQNaN);  // +-inf, NaN
    }

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

// fp32_math.cpp:451-466.
float cos32(float x) {
    const uint ix = asuint(x) & 0x7FFFFFFFu;
    if (ix >= 0x7F800000u) {
        return asfloat(kBitsQNaN);  // +-inf, NaN
    }

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

// ===========================================================================
// THE CONSTANT ROSTER -- what the constant-parity battery reads back
// ===========================================================================
//
// Every constant this module carries, in a pinned order, reachable from a
// kernel so the DEVICE can report the bits it actually compiled. The test
// transcribes the same 32 expressions from fp32_math.cpp independently and
// compares bit patterns; a slangc fold that differed from an MSVC fold by one
// ulp -- the failure mode rule 4 above cannot rule out by reasoning -- fails
// there, naming the index.
//
// APPEND ONLY. The indices are the test's contract; inserting in the middle
// silently re-points every later assertion.
static const uint kFp32MathConstantCount = 32u;

float fp32_math_constant(uint i) {
    switch (i) {
        case 0u: return kLn2Hi;
        case 1u: return kLn2Lo;
        case 2u: return kSqrt2;
        case 3u: return kInvLn2;
        case 4u: return kE2;
        case 5u: return kE3;
        case 6u: return kE4;
        case 7u: return kE5;
        case 8u: return kE6;
        case 9u: return kE7;
        case 10u: return kE8;
        case 11u: return kTwoOverPi;
        case 12u: return kPio2A;
        case 13u: return kPio2B;
        case 14u: return kPio2C;
        case 15u: return kS1;
        case 16u: return kS2;
        case 17u: return kS3;
        case 18u: return kS4;
        case 19u: return kS5;
        case 20u: return kC1;
        case 21u: return kC2;
        case 22u: return kC3;
        case 23u: return kC4;
        case 24u: return kC5;
        case 25u: return 2.0f / 3.0f;   // log32's R, fp32_math.cpp:343
        case 26u: return 2.0f / 5.0f;
        case 27u: return 2.0f / 7.0f;
        case 28u: return 2.0f / 9.0f;
        case 29u: return kMaxReducibleAngle;  // core/fp32_math.hpp:198
        case 30u: return 0x1.0p24;            // log32's subnormal pre-scale
        case 31u: return 0x1.0p-64;           // exp32's subnormal post-scale
        default: return 0.0f;
    }
}
