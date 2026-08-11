#pragma once

// ---------------------------------------------------------------------------
// spade::math -- the engine's OWN fp32 logarithm and sine/cosine.
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
// corpus (tests/golden/*.digest) therefore encoded the MSVC CRT, and
// tests/golden/bounce.digest was the first scenario unlucky enough to notice:
// its four Dryden streams' INITIAL draws (dryden_init's five gaussians per
// world, which land in the state unattenuated) landed on an input where the
// two libms differ. The other three scenarios agreed by luck, not by contract.
//
// This header is the contract. log32/sin32/cos32 are built exclusively from
// IEEE-mandated operations over pinned groupings, so they return the same bits
// on every conforming implementation -- and, at S6, on the GPU, which is the
// second half of the same problem (rng.hpp's own note conceded that "GPU parity
// for gaussians is a tolerance claim"; with these it is an exact one).
//
// ---------------------------------------------------------------------------
// WHAT IS *NOT* HERE, AND WHY
//
// std::sqrt STAYS. IEEE 754 mandates a correctly rounded square root, so it is
// already bit-identical everywhere and re-implementing it would trade a proof
// for a polynomial. The rule this file follows is exactly "replace the
// operations the standard leaves free, keep the ones it pins".
//
// std::exp (world/medium.cpp's Dryden coefficients) and the std::sin/std::cos
// in core/math_ops.cpp's exp-map are the same CLASS of exposure and are NOT
// changed here: they are not what diverged, and this file is the minimum
// intervention that restores bit parity. See docs in those files and the
// investigation record for the disposition.
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
// Sine and cosine, fp32, deterministic.
//
// ACCURACY DOMAIN: |x| <= kMaxAccurateAngle. Inside it the Cody-Waite argument
// reduction below is EXACT (the three pi/2 parts are chosen so that n * part
// is exact for every quadrant index the domain can produce), and the result is
// within ~1 ulp; tests/test_fp32_math.cpp proves that exhaustively over
// [0, 2*pi], the interval rng.hpp's Box-Muller angle actually lives in.
//
// OUTSIDE the domain the functions stay TOTAL and stay DETERMINISTIC -- they
// are still built only from IEEE-mandated operations, so two platforms still
// agree bit for bit -- but the reduction's integer products stop being exact
// and accuracy degrades with |x|, exactly as every Cody-Waite implementation
// does. A non-finite argument returns NaN.
//
// Determinism is therefore unconditional; accuracy is what the domain bounds.
// That split is deliberate: a caller outside the domain gets a wrong-ish
// answer, never a platform-dependent one.
// ---------------------------------------------------------------------------
inline constexpr float kMaxAccurateAngle = 100.0f;

[[nodiscard]] float sin32(float x) noexcept;
[[nodiscard]] float cos32(float x) noexcept;

}  // namespace spade::math
