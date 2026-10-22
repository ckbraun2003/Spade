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
// IT ALSO HOLDS THE EXACT DIVISION AND SQUARE ROOT, div32_ftz and sqrt32_ftz
// (Core's plan of 2026-10-22, docs/design/core/plans/). Their host twin is not
// a function in fp32_math.cpp but the hardware's own divss and sqrtss under
// MXCSR FTZ|DAZ (engine/core/detail/flush_subnormals.hpp), NaN bits included,
// and they are held to it bit for bit: tests/test_fp32_exact.cpp proves the
// correction on a line-cited C++ port (tests/fp32_exact_battery.hpp), and
// tests/test_gpu_fp32_math.cpp runs these functions on the device. log32
// divides with div32_ftz.
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
//      two side by side rather than trust this sentence. (The exact division
//      and square root run the other way: fp32_exact_battery.hpp cites THIS
//      file's lines.)
//
//   2. NO CONTRACTION. cmake/SpadeSlang.cmake compiles every kernel
//      -fp-mode precise, which decorates every contractable float op in the
//      module NoContraction -- the module-level equivalent of the CPU side's
//      /fp:precise + -ffp-contract=off. tests/test_slang_layouts.cpp's
//      SlangSpirv.FloatControlsPinned holds THIS module to the EXACT profile
//      (spirv_scan.hpp): NoContraction everywhere, no sum-of-products opcode,
//      and rule 3's E1/E2.
//
//   3. OpFDiv AND InverseSqrt ONLY AS AN ESTIMATE, AND NO Sqrt (E1/E2, as the
//      user restated them on 2026-10-22). Vulkan's division and inverse square
//      root are <= 2.5 and <= 2 ulp, NOT correctly rounded, so neither result
//      may reach a path that claims bit-identity with an IEEE-correct CPU.
//      Each may appear only as the ESTIMATE inside an exact operation whose
//      integer correction is proven for ANY estimate within that bound, and no
//      result leaves uncorrected. A bare Sqrt stays forbidden.
//
//      The scan proves REACH (spirv_scan.hpp, pass 3): an estimate may pass
//      only through FMul, FAdd, FSub, FNegate, Floor, Trunc, Round and
//      CopyObject, and leave only by a conversion to an integer. It cannot see
//      whether the integer that leaves is corrected -- `uint(a / b)` passes
//      too -- so the guarantee covers exactly the functions listed under
//      "Functions allowed to hold an estimate (E1/E2)" below, each with its
//      battery tests. An estimate anywhere else is a review finding.
//
//      SQRT AUDIT: there is NO square root in any of the four kernels. Checked
//      by reading fp32_math.cpp end to end -- `kSqrt2` is a LITERAL CONSTANT
//      (the mantissa split point), not a call, and core/fp32_math.hpp's own
//      "WHAT IS *NOT* HERE" note states the policy ("std::sqrt STAYS ... IEEE
//      754 mandates a correctly rounded square root"). rng.hpp's
//      `std::sqrt(-2 * log32(u1))` is Box-Muller's, OUTSIDE these kernels.
//      This module's one square root is sqrt32_ftz below: an InverseSqrt
//      estimate inside the exact correction, which rule 3 allows. None of the
//      four kernels calls it.
//
//      R3 (2026-09-24) STANDS FOR THE GAUSSIAN. It deleted the deferral that
//      used to end this paragraph ("and is a later task's problem") rather
//      than give it an owner, because Vulkan does not guarantee a
//      correctly-rounded sqrt, and A DEFERRAL ASSERTS THAT A THING IS STILL
//      COMING. sqrt32_ftz does not lean on Vulkan's sqrt -- its correction
//      holds for any estimate within the bound -- but R3 was lifted on
//      2026-10-22 only for the airflow solver (backlog, "Bit-exact GPU
//      airflow"). So the gaussian path is still BANDED BY DECLARATION -- see
//      rng.slang's header for the measured band, its n and its device
//      provenance, and design-specs/spade/01-charter.md section 4 for where
//      that grade sits among the three -- and moving rng_next_gauss onto
//      sqrt32_ftz is Cameron's call, not this file's.
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
// SO THE DIVISION IS EXACT: log32 calls div32_ftz below, which returns what the
// host's divss returns under MXCSR FTZ|DAZ, bit for bit. At log32's call site
// that is IEEE's own quotient, the host's `/`, because the domain is all
// normal: f is +0 or normal (|f| >= 2^-24), 2 + f lies in [1.71, 2.42] and
// |s| in [2.4e-8, 0.1716] (log32's fold, below), so the flush never applies and
// log32's bits do not move on any device.
// GpuFp32Math.CorrectlyRoundedDivisionMatchesTheHostDivide
// proves it against the host `/` over every (f, 2+f) pair log32 can form, and
// the four log32 batteries prove it again end to end.
//
// UNTIL 2026-10-22 THIS WAS log32_div: a restoring integer long division with
// no float op at all, fenced by its name and by a four-part precondition (no
// subnormal, infinite or NaN operand; a normal quotient) that it did not check.
// div32_ftz answers over the whole domain, so log32_div is deleted rather than
// kept beside it (TD-9: one division).
//
// THIS WAS A DEVIATION FROM THE BRIEF'S LETTER AND AN OBEDIENCE TO ITS INTENT,
// and it is flagged here rather than buried: the brief anticipated only
// divisions by constants.
// ===========================================================================
//
// ===========================================================================
// Functions allowed to hold an estimate (E1/E2)
// ===========================================================================
//
// The only functions of this module that may hold an OpFDiv or InverseSqrt
// result, and so the only ones in an EXACT-profile module that imports it
// (E1/E2 as the user restated them on 2026-10-22). A banded kernel that
// imports it keeps its own bare OpFDivs (rotors.slang, integrate.slang).
// engine/testing/spirv_scan.hpp's pass 3 proves where an estimate can go -- through FMul/FAdd/FSub/FNegate/Floor/Trunc/Round/
// CopyObject to an integer conversion -- but it also passes an uncorrected
// `uint(a / b)`. So the guarantee covers exactly these functions, and these
// batteries prove that the integer each estimate becomes is corrected, with
// estimates pushed +-3 ulps off (past the Vulkan bound) and far past it. An
// estimate in any other function is a review finding (01-verification,
// "SPIR-V scanner"). No FAbs or other op outside the scan's carrying list is
// applied to an estimate; one would need a dated amendment there.
//
//   div32_ftz_with (and div32_ftz, which supplies y)    OpFDiv 1/bm
//       Fp32Exact.DivisionRoundsExactlyWithEstimatesThreeUlpsOff
//       Fp32Exact.TheCorrectionHoldsFarPastTheVulkanBound
//       GpuFp32Math.ExactCorrectionRoundsInjectedEstimatesOnTheDevice
//
//   sqrt32_ftz_with (and sqrt32_ftz, which supplies r)  InverseSqrt xm
//       Fp32Exact.SqrtRoundsExactlyWithEstimatesThreeUlpsOffOverItsWholeInputSpace
//       Fp32Exact.TheCorrectionHoldsFarPastTheVulkanBound
//       GpuFp32Math.ExactCorrectionRoundsInjectedEstimatesOnTheDevice
// ===========================================================================

// ---------------------------------------------------------------------------
// Bit-level access. The C++ side uses std::memcpy (fp32_math.cpp:11-24) because
// it is the only spelling that is not strict-aliasing UB there; Slang's
// asuint/asfloat are the same reinterpretation, compile to OpBitcast, and have
// no aliasing question to answer.
// ---------------------------------------------------------------------------

// Non-finite bit patterns, spelled as patterns rather than as expressions.
// `1.0f / 0.0f` would be a bare OpFDiv (rule 3) and `0.0f / 0.0f` a second
// one; these are what std::numeric_limits<float>::infinity() and ::quiet_NaN()
// ARE on the host, and the edge-case battery compares them bit for bit.
static const uint kBitsPosInf = 0x7F800000u;
static const uint kBitsNegInf = 0xFF800000u;
static const uint kBitsQNaN = 0x7FC00000u;  // std::numeric_limits<float>::quiet_NaN()
static const uint kBitsDefaultNaN = 0xFFC00000u;  // x86's QNaN indefinite: divss/sqrtss's invalid result

// ===========================================================================
// div32_ftz / sqrt32_ftz -- divss and sqrtss under MXCSR FTZ|DAZ, bit for bit
// ===========================================================================
//
// WHAT THEY COMPUTE, over the WHOLE domain: IEEE 754 binary32, round to
// nearest even, under x86's flush -- a subnormal operand reads as a zero of
// its sign (DAZ), and a result below 2^-126 is a zero of its sign (FTZ) --
// with x86's NaN bits:
//
//   input                 div32_ftz                       sqrt32_ftz
//   NaN                   a's quieted (a | 0x00400000),   x | 0x00400000
//                         else b's (divss returns its
//                         first source)
//   invalid               0/0, inf/inf -> 0xFFC00000      negative normal, -inf
//                                                         -> 0xFFC00000
//   inf                   inf/finite -> +-inf;            +inf
//                         finite/inf -> +-0
//   0 or subnormal (DAZ)  x/0 -> +-inf; 0/x -> +-0        +-0, its sign kept
//   overflow              biased e >= 255 -> +-inf        none
//   underflow (FTZ)       biased e <= 0 -> +-0            none
//
// The `_ftz` suffix is the warning: unlike log32 and exp32, which keep
// subnormals, these flush. The canary is 0x00FFFFFF / 2, FLT_MIN (1 - 2^-24):
// +0 here and from divss under FTZ, where IEEE's gradual underflow rounds it
// up to FLT_MIN.
//
// THE METHOD is Rendering's exact_div and exact_sqrt (rendering/exact-div-sqrt
// @ f71252b, the estimate forms): the device's own OpFDiv or InverseSqrt gives
// an ESTIMATE of the significand's reciprocal or inverse root, and an integer
// correction on the 24-bit significands makes the answer exact. The estimate
// enters as the `_with` form's argument, so the op and the probe's
// injected-estimate modes (fp32_math_probe.slang, 8 and 9) run the one
// correction (TD-9). [ForceInline] keeps the estimate inside one function
// body: it never crosses an OpFunctionCall, which the scan would refuse.
//
// DIVISION. q* = A 2^s / B lies in [2^23, 2^24) (s = 23 when A >= B, else
// 24), and y = (1/bm)(1 + eps):
//   Q0 = uint(am * y * 2^s)       |q* - Q0| < 2^24 |eps| + 2.
//   R0 = A 2^s - B Q0 = B d0      exact in 32-bit wrapping arithmetic iff
//                                 |R0| < 2^31: |d0| < 128, as B < 2^24.
//   k = round(R0 y 2^-23)         t = d0 (1 + eps)(1 + 3 delta), |t - d0| <
//                                 2^-9, so |q* - Q1| < 1/2 + 2^-9.
//   2 R1 against +-B              decides |q* - Q| < 1/2 exactly; one +-1 is
//                                 enough, as |d1| < 3/2.
// At Vulkan's 2.5 ulp, |d0| <= 7. |d0| < 128 holds to about +-63 ulps of y
// (+-32 gives |d0| about 66); Rendering's numpy model had 0 misses through
// +-62 and its first at +-64.
//
// SQUARE ROOT. S* = sqrt(X), X = M 2^(23 + odd) in [2^46, 2^48), odd =
// (ex + 1) & 1, xm in [1, 4) the significand with the odd exponent folded in,
// and r = (1/sqrt(xm))(1 + eps):
//   S0 = uint(xm * r * 2^23)      |e0| = |S* - S0| < 2^24 |eps| + 2.
//   R0 = X - S0^2 = e0 (S* + S0)  exact in the wrap while |e0| < 63:
//                                 |R0| < 63 (2^25 + 63) < 2^31 (the true
//                                 limit is just under 64).
//   k = round(R0 r 2^-24)         R0 / 2 S*: lands within 1/2 + 2^-10.
//   R1 > S1                       <=>  sqrt(X) > S1 + 1/2, and
//   R1 <= -S1                     <=>  sqrt(X) < S1 - 1/2, in integers.
// At Vulkan's 2 ulp, |e0| <= 6. The wrap's limit is about +-31 ulps of r at
// worst, where xm -> 4 (r -> 1/2, S* -> 2^24, |e0| about 2k); near xm = 1 it
// is about four times that. A numpy model of this correction had 0 misses over
// xm in [3.9, 4) through +-31 and its first at +-32.
//
// NO TIES, AND NO ROUND-UP ACROSS A BINADE (Rendering's proofs). A quotient
// tie needs A 2^(s+1) = B (2Q + 1), whose 2-adic valuations differ (at least
// 24 against at most 23); the square root of a 24-bit significand cannot have
// exactly 25 significant bits. So every comparison above is strict and "to
// even" never decides anything. A quotient of two significands below a power
// of two is at most 1 - 2^-24 or 2 - 2^-23, both floats, so the binade is
// known from A >= B alone, tininess before and after rounding agree, and FTZ
// is "the biased exponent is 0 or less".
//
// NO FMA. The proof spends only the slack in those bounds, so it holds whether
// a driver fuses or contracts the estimate's FMul and FAdd or not.

// a / b, given y ~ 1/bm (bm = b's significand in [1, 2)). See above.
[ForceInline]  // the estimate never crosses an OpFunctionCall (the taint scan)
float div32_ftz_with(float a, float b, float y) {
    const uint ua = asuint(a);
    const uint ub = asuint(b);
    const uint sign = (ua ^ ub) & 0x80000000u;
    const uint ea = (ua >> 23) & 0xFFu;
    const uint eb = (ub >> 23) & 0xFFu;
    const uint fa = ua & 0x007FFFFFu;
    const uint fb = ub & 0x007FFFFFu;

    // The specials, in divss's order (the table above).
    if (ea == 0xFFu && fa != 0u) {
        return asfloat(ua | 0x00400000u);  // a is a NaN: a's, quieted
    }
    if (eb == 0xFFu && fb != 0u) {
        return asfloat(ub | 0x00400000u);  // b is a NaN: b's, quieted
    }
    if (ea == 0xFFu) {
        return asfloat(eb == 0xFFu ? kBitsDefaultNaN : (sign | kBitsPosInf));  // inf / inf, inf / finite
    }
    if (eb == 0xFFu) {
        return asfloat(sign);  // finite / inf
    }
    if (eb == 0u) {
        return asfloat(ea == 0u ? kBitsDefaultNaN : (sign | kBitsPosInf));  // 0 / 0, x / 0 (DAZ)
    }
    if (ea == 0u) {
        return asfloat(sign);  // 0 / normal (DAZ)
    }

    // The correction, Rendering's exact_div unchanged.
    const uint A = fa | 0x00800000u;
    const uint B = fb | 0x00800000u;
    const uint s = A >= B ? 23u : 24u;  // the quotient in [1, 2) or (1/2, 1)
    const float am = asfloat(fa | 0x3F800000u);
    uint Q = uint(am * y * asfloat((127u + s) << 23));
    int R = asint((A << s) - B * Q);  // A 2^s - B Q, in the wrap
    const int k = int(floor(float(R) * y * asfloat(104u << 23) + 0.5f));  // R / B, B = bm 2^23
    Q = asuint(asint(Q) + k);
    R = R - k * asint(B);
    if (2 * R > asint(B)) {
        Q = Q + 1u;
    } else if (2 * R < -asint(B)) {
        Q = Q - 1u;
    }

    const int e = int(ea) - int(eb) + (s == 23u ? 127 : 126);
    if (e >= 255) {
        return asfloat(sign | kBitsPosInf);  // overflow
    }
    if (e <= 0) {
        return asfloat(sign);  // FTZ
    }
    return asfloat(sign | (uint(e) << 23) | (Q & 0x007FFFFFu));
}

// a / b: divss under MXCSR FTZ|DAZ, NaN bits included.
float div32_ftz(float a, float b) {
    // The E1 estimate: OpFDiv, <= 2.5 ulp.
    return div32_ftz_with(a, b, 1.0f / asfloat((asuint(b) & 0x007FFFFFu) | 0x3F800000u));
}

// sqrt(x), given r ~ 1/sqrt(xm). See above.
[ForceInline]  // the estimate never crosses an OpFunctionCall (the taint scan)
float sqrt32_ftz_with(float x, float r) {
    const uint ux = asuint(x);
    const uint ex = (ux >> 23) & 0xFFu;
    const uint fx = ux & 0x007FFFFFu;

    // The specials, in sqrtss's order (the table above).
    if (ex == 0xFFu && fx != 0u) {
        return asfloat(ux | 0x00400000u);  // a NaN, quieted
    }
    if (ex == 0u) {
        return asfloat(ux & 0x80000000u);  // +-0, and a subnormal reads as one (DAZ)
    }
    if ((ux >> 31) != 0u) {
        return asfloat(kBitsDefaultNaN);  // a negative normal, -inf
    }
    if (ex == 0xFFu) {
        return asfloat(kBitsPosInf);  // +inf
    }

    // The correction, Rendering's exact_sqrt with the exponent in integers:
    // x = xm 2^(ex - 127 - odd), ex - 127 - odd even, sqrt(xm) in [1, 2).
    const uint odd = (ex + 1u) & 1u;
    const uint M = fx | 0x00800000u;
    const float xm = asfloat(fx | ((127u + odd) << 23));
    uint S = uint(xm * r * asfloat(150u << 23));
    const uint X = M << (23u + odd);  // its low 32 bits: all the wrap needs
    int R = asint(X - S * S);  // X - S^2, in the wrap
    const int k = int(floor(float(R) * r * asfloat(103u << 23) + 0.5f));  // R / 2S
    S = asuint(asint(S) + k);
    R = asint(X - S * S);
    // sqrt(X) > S + 1/2  <=>  R > S;  sqrt(X) < S - 1/2  <=>  R <= -S.
    if (R > asint(S)) {
        S = S + 1u;
    } else if (R <= -asint(S)) {
        S = S - 1u;
    }
    return asfloat((((ex + 127u - odd) >> 1) << 23) | (S & 0x007FFFFFu));
}

// sqrt(x): sqrtss under MXCSR FTZ|DAZ, NaN bits included.
float sqrt32_ftz(float x) {
    const uint ux = asuint(x);
    const uint odd = ((ux >> 23) + 1u) & 1u;  // (ex + 1) & 1: the sign bit is not bit 0
    // The E2 estimate: InverseSqrt, <= 2 ulp.
    return sqrt32_ftz_with(x, rsqrt(asfloat((ux & 0x007FFFFFu) | ((127u + odd) << 23))));
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

// THE INTEGER HELPERS for the subnormal paths (fp32_math.cpp's twins). No
// kernel requests an fp32 denormal mode (SPIR-V rule P3), so a device may
// flush subnormal inputs and outputs of every float op (the RTX 3060 Ti does).
// Every subnormal is handled in integers, by shifts only: no extended
// instruction, and the same steps as the host.

// The index of v's highest set bit, for v != 0.
uint msb_index(uint v) {
    uint p = 0u;
    if (v >= 0x00010000u) {
        v >>= 16u;
        p += 16u;
    }
    if (v >= 0x00000100u) {
        v >>= 8u;
        p += 8u;
    }
    if (v >= 0x00000010u) {
        v >>= 4u;
        p += 4u;
    }
    if (v >= 0x00000004u) {
        v >>= 2u;
        p += 2u;
    }
    if (v >= 0x00000002u) {
        p += 1u;
    }
    return p;
}

// v * 2^-shift, rounded to nearest with ties to even, for shift in [0, 31].
uint shift_right_nearest_even(uint v, uint shift) {
    if (shift == 0u) {
        return v;
    }
    const uint kept = v >> shift;
    const uint rest = v & ((1u << shift) - 1u);
    const uint half = 1u << (shift - 1u);
    return kept + ((rest > half || (rest == half && (kept & 1u) != 0u)) ? 1u : 0u);
}

// fp32_math.cpp's log32.
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
    // normal range first, by 2^24, in integers.
    //
    // NO PATH IN THIS MODULE DEPENDS ON DENORMAL BEHAVIOUR. A float multiply
    // here once did: on a device that flushes subnormal inputs (the RTX 3060
    // Ti, measured in docs/design/core/2026-10-04-nvidia-denorm-report.md) it
    // read x * 2^24 as 0. exp32's subnormal results and sin32's subnormal
    // arguments had the same dependency, and all three are integer now.
    // Fp32Math.SubnormalPathsDoNotDependOnTheFlushMode pins them on the host
    // under FTZ and DAZ, and GpuFp32Math.EdgeCasesAreBitIdenticalToTheHost
    // enumerates the subnormal boundary on every device.
    uint mx = ix;
    int e = 0;
    if ((ix >> 23) == 0u) {
        // In integers, never x * 2^24, which a flushing device reads as 0
        // (fp32_math.cpp explains the bits: exponent p + 2, f's bits below p).
        const uint p = msb_index(ix);
        mx = ((p + 2u) << 23) | ((ix << (23u - p)) & 0x007FFFFFu);
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
    const float s = div32_ftz(f, 2.0f + f);  // fp32_math.cpp:337 -- the one division; see the audit
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

    // k == -126 belongs to the tail below: with m < 1 its result is subnormal.
    if (k >= -125 && k <= 127) {
        return m * pow2(k);
    }
    if (k > 127) {
        // k == 128 only, and the result may still be finite. Split so the
        // intermediate stays representable and the second multiply is allowed
        // to overflow to +infinity on its own.
        return (m * pow2(127)) * pow2(k - 127);
    }
    // k in [-150, -126]: subnormal or zero (or FLT_MIN's binade for k == -126
    // and m >= 1). m * 2^(k+64) is normal and exact; the scaling by 2^-64, the
    // ONE rounding, is in integers, because a flushing device would return 0
    // from a float multiply: the significand shifted right by 65 - E, rounded
    // to nearest even (fp32_math.cpp's exp32 gives the arithmetic).
    const uint scaled = asuint(m * pow2(k + 64));
    const uint sig = (scaled & 0x007FFFFFu) | 0x00800000u;
    return asfloat(shift_right_nearest_even(sig, 65u - (scaled >> 23)));
}

// fp32_math.cpp:434-449.
float sin32(float x) {
    const uint ix = asuint(x) & 0x7FFFFFFFu;
    if (ix >= 0x7F800000u) {
        return asfloat(kBitsQNaN);  // +-inf, NaN
    }
    // A non-zero subnormal returns itself, before any float op, so a device
    // that flushes subnormal inputs gives the host's bits, not 0.
    if (ix != 0u && ix < 0x00800000u) {
        return x;
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
