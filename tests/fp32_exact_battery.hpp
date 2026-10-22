// fp32_exact_battery.hpp -- the shared half of the exact division and square
// root batteries (Core's plan, docs/design/core/plans/
// 2026-10-22-exact-div-sqrt-plan.md): tests/test_fp32_exact.cpp's Fp32Exact.*
// (the CPU proof) and tests/test_gpu_fp32_math.cpp's GpuFp32Math.Exact* (the
// device) both use it.
//
//   THE PORT. port_div and port_sqrt are engine/shaders/fp32_math.slang's
//   div32_ftz_with and sqrt32_ftz_with in C++, statement for statement, each
//   line citing the Slang line it ports. uint32_t carries every Slang int, and
//   std::bit_cast is asint/asfloat, so no wrap is signed-overflow UB; the float
//   ops keep the Slang order and grouping, and spade_tests compiles under
//   spade_fp_strict (-ffp-contract=off, /fp:precise), so no product fuses.
//   `stop` ends the correction early, for
//   Fp32Exact.InjectedErrorIsVisibleWithoutTheCorrection.
//
//   C1 (RED): THE PORT STOPS AT Q0 / S0, the estimate's truncation alone, for
//   every `stop`: no k-step and no final +-1, so the batteries miss even at
//   k = 0. The Slang ops are stubs in C1, so there is no line to cite yet; the
//   statements follow the plan's sections 1.1 and 1.2. C2 completes the
//   correction and cites fp32_math.slang line by line.
//
//   THE REFERENCE. divss and sqrtss under MXCSR FTZ|DAZ: _mm_div_ps and
//   _mm_sqrt_ps (each lane rounded as the scalar form rounds; _mm_div_ss and
//   _mm_sqrt_ss for a tail), over arrays in memory, in a SPADE_NOINLINE body
//   behind a thin scoped wrapper (core/detail/flush_subnormals.hpp's rule). So
//   nothing folds to IEEE at compile time and no float op moves across the
//   MXCSR write. Fp32Exact.TheHostReferenceFlushes' canary fails if the scope
//   is not in effect.
//
//   THE ESTIMATES. The correctly rounded 1/bm (the host's divss) and
//   1/sqrt(xm) (via double), and nudge(), which moves one k floats along the
//   lattice; lowest_k() sizes the +-ulps window.
//
//   THE INPUTS. splitmix64 from fixed seeds (the same stream on every
//   compiler), the specials, Rendering's near_ties() for division, and
//   sqrt_near_ties(), its twin for the square root.
//
// x86-64 only, as core/detail/flush_subnormals.hpp is.
#pragma once

#include <xmmintrin.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "core/detail/flush_subnormals.hpp"

namespace spade::testing::exact {

inline constexpr uint32_t kDefaultNaN = 0xFFC00000u;  // x86's QNaN indefinite (fp32_math.slang's kBitsDefaultNaN)
inline constexpr uint32_t kPosInf = 0x7F800000u;

[[nodiscard]] inline float from_bits(uint32_t u) noexcept { return std::bit_cast<float>(u); }
[[nodiscard]] inline uint32_t bits(float x) noexcept { return std::bit_cast<uint32_t>(x); }
[[nodiscard]] inline int32_t as_int(uint32_t u) noexcept { return std::bit_cast<int32_t>(u); }

// Where the port's correction stops: at the estimate's truncation (Q0, S0),
// after the rounded k-step (Q1, S1), or nowhere.
enum class Stop { at_estimate, after_step, full };

// ---------------------------------------------------------------------------
// THE PORT. Results and operands are bit patterns, as the device test sees
// them. C1 (RED): stopped at Q0 / S0 (see the header).
// ---------------------------------------------------------------------------

// div32_ftz_with: a / b from y ~ 1/bm (the plan's section 1.1 and 1.3).
[[nodiscard]] inline uint32_t port_div(uint32_t ua, uint32_t ub, float y, Stop stop = Stop::full) noexcept {
    static_cast<void>(stop);  // RED: every stop is Q0
    const uint32_t sign = (ua ^ ub) & 0x80000000u;
    const uint32_t ea = (ua >> 23) & 0xFFu;
    const uint32_t eb = (ub >> 23) & 0xFFu;
    const uint32_t fa = ua & 0x007FFFFFu;
    const uint32_t fb = ub & 0x007FFFFFu;

    if (ea == 0xFFu && fa != 0u) return ua | 0x00400000u;
    if (eb == 0xFFu && fb != 0u) return ub | 0x00400000u;
    if (ea == 0xFFu) return eb == 0xFFu ? kDefaultNaN : (sign | kPosInf);
    if (eb == 0xFFu) return sign;
    if (eb == 0u) return ea == 0u ? kDefaultNaN : (sign | kPosInf);
    if (ea == 0u) return sign;

    const uint32_t A = fa | 0x00800000u;
    const uint32_t B = fb | 0x00800000u;
    const uint32_t s = A >= B ? 23u : 24u;
    const float am = from_bits(fa | 0x3F800000u);
    const uint32_t Q = static_cast<uint32_t>(am * y * from_bits((127u + s) << 23));  // Q0; RED stops here

    const int32_t e = static_cast<int32_t>(ea) - static_cast<int32_t>(eb) + (s == 23u ? 127 : 126);
    if (e >= 255) return sign | kPosInf;
    if (e <= 0) return sign;
    return sign | (static_cast<uint32_t>(e) << 23) | (Q & 0x007FFFFFu);
}

// sqrt32_ftz_with: sqrt(x) from r ~ 1/sqrt(xm) (the plan's section 1.2 and 1.3).
[[nodiscard]] inline uint32_t port_sqrt(uint32_t ux, float r, Stop stop = Stop::full) noexcept {
    static_cast<void>(stop);  // RED: every stop is S0
    const uint32_t ex = (ux >> 23) & 0xFFu;
    const uint32_t fx = ux & 0x007FFFFFu;

    if (ex == 0xFFu && fx != 0u) return ux | 0x00400000u;
    if (ex == 0u) return ux & 0x80000000u;
    if ((ux >> 31) != 0u) return kDefaultNaN;
    if (ex == 0xFFu) return kPosInf;

    const uint32_t odd = (ex + 1u) & 1u;
    const float xm = from_bits(fx | ((127u + odd) << 23));
    const uint32_t S = static_cast<uint32_t>(xm * r * from_bits(150u << 23));  // S0; RED stops here
    return (((ex + 127u - odd) >> 1) << 23) | (S & 0x007FFFFFu);
}

// ---------------------------------------------------------------------------
// THE ESTIMATES.
// ---------------------------------------------------------------------------

// The correctly rounded 1/bm for b's bits, bm = b's significand in [1, 2) as
// div32_ftz forms it (fp32_math.slang:312). The host's `/` is divss; bm and
// 1/bm are normal, so the flush does not enter.
[[nodiscard]] inline float div_estimate(uint32_t ub) noexcept {
    return 1.0f / from_bits((ub & 0x007FFFFFu) | 0x3F800000u);
}

// 1/sqrt(xm) for x's bits, xm in [1, 4) as sqrt32_ftz forms it
// (fp32_math.slang:322-324), rounded once to float from double. Two roundings
// in double can leave it a float off the correctly rounded value only when the
// exact one is within about 2^-52 of a float midpoint; the windows below reach
// at least 3 floats either side, so they still hold every estimate within
// InverseSqrt's 2 ulp.
[[nodiscard]] inline float sqrt_estimate(uint32_t ux) noexcept {
    const uint32_t odd = ((ux >> 23) + 1u) & 1u;
    const double xm = static_cast<double>(from_bits((ux & 0x007FFFFFu) | ((127u + odd) << 23)));
    return static_cast<float>(1.0 / std::sqrt(xm));
}

// e moved k floats along the lattice: its bit pattern plus k. e is a positive
// normal near 1, so no k used here leaves its sign or the normal range.
[[nodiscard]] inline float nudge(float e, int k) noexcept { return from_bits(bits(e) + static_cast<uint32_t>(k)); }

// The window "within `ulps` ulps of e" is every float within ulps times e's
// own binade's spacing: k from lowest_k(e, ulps) to +ulps. Above e a bit step
// is at least that spacing. Below a power of two the steps halve, so a window
// that crosses one takes more of them: from -6 at e = 1 (bm = 1, xm = 1, where
// Vulkan's ulp is the gap above), from -5 one step above a power of two
// (bm = 2 - 2^-23, xm just under 4), from -ulps everywhere else.
[[nodiscard]] inline int lowest_k(float e, int ulps) noexcept {
    const int m = static_cast<int>(bits(e) & 0x007FFFFFu);  // steps down to e's binade's floor
    return m >= ulps ? -ulps : -(2 * ulps - m);
}

// ---------------------------------------------------------------------------
// THE REFERENCE: divss and sqrtss.
// ---------------------------------------------------------------------------

// a[i] / b[i] -> out[i], as bits, under whatever MXCSR word the caller set.
SPADE_NOINLINE inline void host_divide(const uint32_t* a, const uint32_t* b, uint32_t* out, std::size_t n) {
    std::size_t i = 0;
    for (; i + 4 <= n; i += 4) {
        const __m128 x = _mm_loadu_ps(reinterpret_cast<const float*>(a + i));
        const __m128 y = _mm_loadu_ps(reinterpret_cast<const float*>(b + i));
        _mm_storeu_ps(reinterpret_cast<float*>(out + i), _mm_div_ps(x, y));
    }
    for (; i < n; ++i) {
        out[i] = bits(_mm_cvtss_f32(_mm_div_ss(_mm_set_ss(from_bits(a[i])), _mm_set_ss(from_bits(b[i])))));
    }
}

// sqrt(x[i]) -> out[i], as bits, under whatever MXCSR word the caller set.
SPADE_NOINLINE inline void host_sqrt(const uint32_t* x, uint32_t* out, std::size_t n) {
    std::size_t i = 0;
    for (; i + 4 <= n; i += 4) {
        const __m128 v = _mm_loadu_ps(reinterpret_cast<const float*>(x + i));
        _mm_storeu_ps(reinterpret_cast<float*>(out + i), _mm_sqrt_ps(v));
    }
    for (; i < n; ++i) {
        out[i] = bits(_mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ss(from_bits(x[i])))));
    }
}

// The reference: divss under MXCSR FTZ|DAZ. The wrapper is the scope and one
// call, and does no float work.
inline void reference_divide(std::span<const uint32_t> a, std::span<const uint32_t> b, std::span<uint32_t> out) {
    const spade::math::ScopedFlushSubnormals flush;
    host_divide(a.data(), b.data(), out.data(), out.size());
}

// The reference: sqrtss under MXCSR FTZ|DAZ, likewise.
inline void reference_sqrt(std::span<const uint32_t> x, std::span<uint32_t> out) {
    const spade::math::ScopedFlushSubnormals flush;
    host_sqrt(x.data(), out.data(), out.size());
}

// ---------------------------------------------------------------------------
// THE INPUTS.
// ---------------------------------------------------------------------------

// splitmix64: a fixed-seed stream, the same on every compiler (Rendering's
// SplitMix, tests/test_gpu_exact_fp32.cpp @ f71252b).
struct SplitMix {
    uint64_t state = 0;
    uint64_t next() noexcept {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
};

// Zeros, subnormals, the normal range's edges, infinities, NaNs (quiet and
// signalling, both signs, several payloads) and ordinary values. The first 40
// are Rendering's special_pairs() list (@ f71252b); the last 8 add NaN payloads
// of both signs and kinds, a negative subnormal and sqrt(2).
inline constexpr std::array<uint32_t, 48> kSpecials = {
    0x00000000u, 0x80000000u, 0x00000001u, 0x80000001u, 0x007FFFFFu, 0x807FFFFFu, 0x00400000u, 0x00800000u,
    0x80800000u, 0x00800001u, 0x00FFFFFFu, 0x00C00000u, 0x01000000u, 0x01800000u, 0x7F7FFFFFu, 0xFF7FFFFFu,
    0x7F7FFFFEu, 0x7F000000u, 0x7E800000u, 0x72800000u, 0x0D800000u, 0x7F800000u, 0xFF800000u, 0x7FC00000u,
    0xFFC00000u, 0x7F800001u, 0x7FBFFFFFu, 0xFFFFFFFFu, 0x3F800000u, 0xBF800000u, 0x3F800001u, 0x3FFFFFFFu,
    0x40000000u, 0x40400000u, 0x3DCCCCCDu, 0x3EAAAAABu, 0x34000000u, 0x33800000u, 0x4B000000u, 0x4B800000u,
    0x7FC00001u, 0x7FFFFFFFu, 0xFF800001u, 0xFFBFFFFFu, 0x7FA5A5A5u, 0xFFE00000u, 0x80400000u, 0x3FB504F3u,
};

// The hardest quotients two significands make: A / B within 1 or 3 parts in
// B 2^(s+1) of a midpoint (2Q + 1) / 2^(s+1), s = 23 for a quotient in [1, 2)
// and 24 for one in (1/2, 1). For odd B, 2Q + 1 = -N / B mod 2^(s+1); a pair
// whose A falls outside its binade is dropped. Exponents put the result
// anywhere in the normal range, the edges included; signs random. Appends
// until a and b hold n pairs. Rendering's near_ties(), verbatim
// (tests/test_gpu_exact_fp32.cpp @ f71252b).
inline void near_ties(std::size_t n, uint64_t seed, std::vector<uint32_t>& a, std::vector<uint32_t>& b) {
    SplitMix rng{seed};
    constexpr int64_t kNs[] = {1, -1, 3, -3};
    while (a.size() < n) {
        const uint64_t r = rng.next();
        const uint64_t B = ((r & 0x7FFFFFu) | 0x800000u) | 1u;
        const int64_t N = kNs[(r >> 23) & 3u];
        const uint32_t s = ((r >> 25) & 1u) != 0u ? 23u : 24u;
        uint64_t inv = B;  // B^-1 mod 2^64, Newton's doubling from 3 bits
        for (int k = 0; k < 5; ++k) inv *= 2u - B * inv;
        const uint64_t mod = uint64_t{1} << (s + 1u);
        const uint64_t c = (static_cast<uint64_t>(-N) * inv) & (mod - 1u);
        uint64_t odd = c;
        if (s == 23u) {
            odd = c + (uint64_t{1} << 24);  // 2Q + 1 in [2^24, 2^25)
        } else if (c < (uint64_t{1} << 24)) {
            continue;
        }
        const uint64_t top = B * odd + static_cast<uint64_t>(N);
        const uint64_t A = top >> (s + 1u);
        if ((A << (s + 1u)) != top) continue;  // not a near-tie after all (cannot happen)
        if (s == 23u ? (A < B || A >= (uint64_t{1} << 24)) : (A >= B || A < (uint64_t{1} << 23))) continue;
        const uint64_t e = rng.next();
        const int er = 1 + static_cast<int>(e % 254u);  // the result's biased exponent
        const int eb = 1 + static_cast<int>((e >> 8) % 254u);
        const int ea = er - (s == 23u ? 127 : 126) + eb;
        if (ea < 1 || ea > 254) continue;
        const uint32_t sa = static_cast<uint32_t>(e >> 63) << 31;
        const uint32_t sb = static_cast<uint32_t>((e >> 62) & 1u) << 31;
        a.push_back(sa | (static_cast<uint32_t>(ea) << 23) | (static_cast<uint32_t>(A) & 0x7FFFFFu));
        b.push_back(sb | (static_cast<uint32_t>(eb) << 23) | (static_cast<uint32_t>(B) & 0x7FFFFFu));
    }
}

// The hardest square roots: x in [1, 4) whose sqrt(X), X = M 2^(23 + odd),
// lies within |c| / 4T of a midpoint T / 2 = S + 1/2, |c| <= c_max. In
// integers that is T^2 = 4X + c, so T is an odd square root of c mod
// 2^(25 + odd) (c = 1 mod 8; Hensel lifting from 1), and M = (T^2 - c) /
// 2^(25 + odd) must be a 24-bit significand. Every such x, ascending; with
// c_max = 256, 96 of them, each within 2^-18 of a midpoint.
[[nodiscard]] inline std::vector<uint32_t> sqrt_near_ties(int64_t c_max) {
    std::vector<uint32_t> out;
    for (int64_t c = -c_max; c <= c_max; ++c) {
        if (((c % 8) + 8) % 8 != 1) continue;
        const uint64_t cu = static_cast<uint64_t>(c);  // T^2 - c, mod 2^64, below
        for (uint32_t odd = 0; odd <= 1u; ++odd) {
            const uint32_t width = 25u + odd;
            const uint64_t mod = uint64_t{1} << width;
            uint64_t t = 1;  // t^2 = c mod 2^j, lifted one bit at a time
            for (uint32_t j = 3; j < width; ++j) {
                if ((((t * t - cu) >> j) & 1u) != 0u) t += uint64_t{1} << (j - 1u);
            }
            const uint64_t half = mod >> 1;
            const uint64_t roots[] = {t, mod - t, (t + half) & (mod - 1u), (half - t) & (mod - 1u)};
            for (const uint64_t T : roots) {
                if (T < (uint64_t{1} << 24) || T >= (uint64_t{1} << 25)) continue;
                const uint64_t top = T * T - cu;
                const uint64_t M = top >> width;
                if ((M << width) != top || M < (uint64_t{1} << 23) || M >= (uint64_t{1} << 24)) continue;
                out.push_back(((127u + odd) << 23) | (static_cast<uint32_t>(M) & 0x7FFFFFu));
            }
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

// Every float with xm in [3.9, 4) (biased exponent 128, so odd = 1): where the
// square root's wrap bound is tightest, about +-31 ulps of r.
inline constexpr uint32_t kBoundEdgeFirst = 0x4079999Au;  // 3.9f
inline constexpr uint32_t kBoundEdgeCount = 0x407FFFFFu - kBoundEdgeFirst + 1u;

// A random pair whose quotient has the biased exponent `er` before any flush
// or overflow (port_div's e): random significands and signs, eb
// redrawn until ea lands in [1, 254]. er from -1 to 256 reaches both of
// div32_ftz's edges, e <= 0 (FTZ) and e >= 255 (overflow).
[[nodiscard]] inline std::pair<uint32_t, uint32_t> pair_with_result_exponent(SplitMix& rng, int er) {
    for (;;) {
        const uint64_t r = rng.next();
        const uint32_t fa = static_cast<uint32_t>(r) & 0x7FFFFFu;
        const uint32_t fb = static_cast<uint32_t>(r >> 23) & 0x7FFFFFu;
        const int bias = (fa | 0x800000u) >= (fb | 0x800000u) ? 127 : 126;
        const uint64_t e = rng.next();
        const int eb = 1 + static_cast<int>(e % 254u);
        const int ea = er - bias + eb;
        if (ea < 1 || ea > 254) continue;
        const uint32_t sa = static_cast<uint32_t>(e >> 63) << 31;
        const uint32_t sb = static_cast<uint32_t>((e >> 62) & 1u) << 31;
        return {sa | (static_cast<uint32_t>(ea) << 23) | fa, sb | (static_cast<uint32_t>(eb) << 23) | fb};
    }
}

}  // namespace spade::testing::exact
