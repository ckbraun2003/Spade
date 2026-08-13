#pragma once

// ---------------------------------------------------------------------------
// spade::math -- the engine's OWN fp32 logarithm, exponential and sine/cosine.
//
// WHY THIS FILE EXISTS, IN ONE SENTENCE: IEEE 754 requires +, -, *, /, sqrt
// and the comparisons to be CORRECTLY ROUNDED, and requires nothing whatsoever
// of log, sin and cos -- so an engine whose determinism contract is
// bit-exactness cannot call the platform's libm and still call itself
// deterministic.
//
// ---------------------------------------------------------------------------
// THE DEFECT THIS FILE CLOSES
//
// Every arithmetic operation in the physics passes is an IEEE-mandated one, so
// two conforming implementations compiled without contraction (-ffp-contract=off,
// engine/CMakeLists.txt's spade_fp_strict) MUST agree on them bit for bit. That
// is what makes the pinned op orders in integrator.cpp, contacts.cpp, grid.cpp
// and medium.cpp meaningful: pin the order, and the answer is pinned.
//
// std::log, std::sin and std::cos are the exception, and rng.hpp's
// Stream::next_gauss() called all three. The MSVC CRT and glibc both implement
// them to well under one ulp and both are perfectly respectable -- and they
// disagree, on roughly one input in a hundred, by one ulp. The determinism
// corpus therefore encoded the MSVC CRT, and `bounce` was the first scenario
// unlucky enough to notice:
// its four Dryden streams' INITIAL draws (dryden_init's five gaussians per
// world, which land in the state unattenuated) landed on an input where the
// two libms differ. The other three scenarios agreed by luck, not by contract.
//
// This header is the contract. log32/exp32/sin32/cos32 are built exclusively
// from IEEE-mandated operations over pinned groupings, so they return the same
// bits on every conforming implementation -- and, at S6, on the GPU, which is
// the second half of the same problem (rng.hpp's own note conceded that "GPU
// parity for gaussians is a tolerance claim"; with these it is an exact one).
//
// ---------------------------------------------------------------------------
// THE SCOPE OF THE CLAIM, AS OF S5 TASK 1
//
// THERE ARE NO LIBM TRANSCENDENTALS LEFT ON ANY ENGINE PARITY PATH. Not one:
// no std::exp, std::log, std::sin, std::cos, std::pow or std::tan executes on
// a path that feeds registered state, a force accumulator, an SDF distance or
// the golden digest. The five call sites that survived the original fix --
// world/medium.cpp's Dryden coefficients (five), core/math_ops.cpp's exp-map
// (two), world/sdf.cpp's heightfield (two) and vehicles/rotor.cpp's RPM lag
// (one) -- now call exp32/sin32/cos32 instead, and a sweep of engine/ is what
// says so rather than a memory of having looked.
//
// (An earlier revision of this note listed only medium.cpp and math_ops.cpp as
// the remaining exposure. That list was WRONG TWICE OVER: it never mentioned
// sdf.cpp or rotor.cpp, both of which were calling libm on parity paths the
// whole time. That is the argument for stating an INVARIANT -- "there are
// none" -- rather than an inventory: an inventory has to be re-checked to be
// worth anything, and nobody re-checked this one for four tasks.)
//
// THE ONE EXEMPTION IS engine/tools/. Nothing under tools/ feeds registered
// state or the digest -- the viewer's camera orbit and its debug geometry are
// presentation, evaluated after the fact -- so tools/viewer/bridge.cpp's
// std::sin/std::cos stay, deliberately and by the same rule that keeps
// std::random_device out of the engine but allows it there.
//
// ---------------------------------------------------------------------------
// WHAT IS *NOT* HERE, AND WHY
//
// std::sqrt STAYS. IEEE 754 mandates a correctly rounded square root, so it is
// already bit-identical everywhere and re-implementing it would trade a proof
// for a polynomial. The rule this file follows is exactly "replace the
// operations the standard leaves free, keep the ones it pins".
//
// ---------------------------------------------------------------------------
// CONSTANTS, AND WHY THEY ARE SPELLED THE WAY THEY ARE
//
// Two spellings appear in the .cpp, and the choice between them is not style:
//
//   * EXACT RATIONALS (`1.0f / 6.0f`) for every polynomial coefficient. IEEE
//     division is correctly rounded, so a compiler folding `1.0f / 6.0f` must
//     produce the same float as one that evaluates it -- there is no
//     folding-versus-runtime divergence to have. This is the spelling
//     world/medium.cpp already uses for its Taylor series, and it keeps the
//     coefficient readable as the mathematical object it is.
//   * HEX FLOAT LITERALS (`0x1.62e4p-1f`) for the irrational splits (ln 2,
//     pi/2). These denote an exact bit pattern with no rounding step at all,
//     which is what a Cody-Waite split needs: its whole correctness argument
//     is about which mantissa bits are zero.
//
// A DECIMAL literal for an irrational would be neither -- it would be a
// rounding the reader cannot check by eye.
//
// ---------------------------------------------------------------------------
// ROUNDING MODE. Everything here assumes the default round-to-nearest,
// ties-to-even, which is the mode every other fp32 expression in the engine
// already assumes and which nothing in spade/ ever changes. The
// round-to-integer step in the sin/cos reduction is written as a truncating
// cast rather than std::rint precisely so it does not read the mode at all.
// ---------------------------------------------------------------------------

namespace spade::math {

// ---------------------------------------------------------------------------
// Natural logarithm, fp32, deterministic.
//
// ACCURACY: <= 1 ulp over the whole normal range, and tests/test_fp32_math.cpp
// proves it EXHAUSTIVELY over the domain that actually matters -- all 2^24
// values Stream::next_float() can return -- rather than by sampling.
//
// EDGE CASES, matching std::log so a reader is never surprised:
//   x  > 0, finite -> ln(x)
//   x == 0         -> -infinity
//   x  < 0         -> NaN
//   x == +infinity -> +infinity
//   x is NaN       -> NaN
// Subnormal inputs are scaled into the normal range first, so they are as
// accurate as any other argument.
// ---------------------------------------------------------------------------
[[nodiscard]] float log32(float x) noexcept;

// ---------------------------------------------------------------------------
// Natural exponential, fp32, deterministic.
//
// ACCURACY: <= 1 ulp, and -- unlike sin32/cos32 below -- WITH NO ACCURACY
// DOMAIN TO STAY INSIDE. The claim is unconditional over the entire float
// line, which is a property of the reduction: exp's argument reduction is
// modulo ln 2 with a quotient bounded by |n| <= 150 for EVERY argument that
// does not already saturate, so the Cody-Waite exactness that bounds
// kMaxAccurateAngle for the trigonometric pair can never be exceeded here.
// See fp32_math.cpp's exp32 section for the construction and the error
// argument, and tests/test_fp32_math.cpp for what is enumerated.
//
// The bound was verified OFFLINE by evaluating all 2,241,855,490 float
// arguments in [-104, 104] -- every argument whose result is not already the
// saturated limit -- against a double std::exp reference: worst 1.000 ulp,
// zero arguments above it, 99.19% correctly rounded. That sweep takes two
// minutes and cannot live in a 60-second T0 test, so the committed tests
// enumerate the three windows that DECIDE the bound (the reduction boundary,
// the largest quotients, the subnormal tail) at full float density and sample
// the rest; test_fp32_math.cpp's exp section says which is which.
//
// EDGE CASES, matching std::exp so a reader is never surprised:
//   x finite, exp(x) representable -> exp(x)
//   x >=  88.7228394  (overflow)   -> +infinity
//   x <= -103.972084  (underflow)  -> +0
//   x == +infinity                 -> +infinity
//   x == -infinity                 -> +0
//   x is NaN                       -> that same NaN
// Arguments at or below -87.3365479 produce SUBNORMAL results. Those carry fewer
// than 24 significand bits by construction, so "1 ulp" is 1 ulp of a coarser
// lattice there -- the bound still holds, and it is still the same bound on
// every platform, which is the property this file exists for.
// ---------------------------------------------------------------------------
[[nodiscard]] float exp32(float x) noexcept;

// ---------------------------------------------------------------------------
// Sine and cosine, fp32, deterministic.
//
// ACCURACY DOMAIN: |x| <= kMaxAccurateAngle. Inside it the Cody-Waite argument
// reduction below is EXACT (the three pi/2 parts are chosen so that n * part
// is exact for every quadrant index the domain can produce), and the result is
// within ~1 ulp; tests/test_fp32_math.cpp proves that exhaustively over
// [0, 2*pi], the interval rng.hpp's Box-Muller angle actually lives in.
//
// OUTSIDE it, BUT INSIDE kMaxReducibleAngle, the functions stay TOTAL and stay
// DETERMINISTIC -- still built only from IEEE-mandated operations, so two
// platforms still agree bit for bit -- while accuracy degrades with |x|,
// exactly as every Cody-Waite implementation does: the reduction's integer
// products stop being exact and the phase error grows like |x| * 2^-24 radians.
//
// AT OR BEYOND kMaxReducibleAngle THE ANSWER IS PINNED RATHER THAN COMPUTED:
// sin32 returns 0 and cos32 returns 1, the values at phase zero. This is a
// DEFINED answer, not an accurate one, and there is a discontinuity where it
// takes over -- both stated plainly because the alternative is worse in two
// specific ways the .cpp documents in full: past |x| ~ 2.7e7 the reduced
// argument escapes the kernels' validity and the results stop being bounded by
// 1 (reaching 3.2e20 by |x| ~ 3.2e9), and past |x| = 3,373,259,520 the
// reduction's float->int32 cast is out of range, i.e. UNDEFINED BEHAVIOUR that
// resolves differently on x86 and ARM64. A pinned, bounded, portable answer is
// the only one this file is entitled to give out there.
//
// A non-finite argument returns NaN.
//
// Determinism is therefore unconditional -- at EVERY magnitude, now including
// the ones that used to be undefined -- and accuracy is what the domain bounds.
// That split is deliberate: a caller outside the domain gets a wrong-ish
// answer, never a platform-dependent one.
// ---------------------------------------------------------------------------

// The ACCURACY bound: <= 1 ulp for |x| <= this. A property of the Cody-Waite
// constants' trailing zeros (see the .cpp), not a tuning choice.
inline constexpr float kMaxAccurateAngle = 100.0f;

// The DEFINEDNESS bound, six orders further out and a different kind of claim:
// at or beyond this magnitude the reduction is not attempted at all. 2^24 is
// where the reduction's own phase error reaches one radian, which is where the
// argument reduction stops being reasoning and starts being noise -- and it is
// also the magnitude at which a float's ulp reaches 2 radians, so consecutive
// representable arguments are a third of a period apart and no phase survives
// the input's own quantization. Below it, nothing about these functions
// changed when this guard was added; the in-domain arithmetic is untouched.
inline constexpr float kMaxReducibleAngle = 0x1.0p24f;  // 16777216

[[nodiscard]] float sin32(float x) noexcept;
[[nodiscard]] float cos32(float x) noexcept;

}  // namespace spade::math
