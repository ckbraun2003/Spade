#pragma once

// ---------------------------------------------------------------------------
// fp16 -- IEEE 754 binary16 conversions, as a numeric primitive beside
// fp32_math (TD-9: one rule in one place). First user: the airflow flow
// volume (core/flow_volume.hpp), whose halves Physics' Publish writes and
// Rendering reads; the GPU staging (plan C) packs to the same rule.
//
// PURE INTEGER BIT MANIPULATION. No F16C or _cvtss_sh intrinsics, no
// _Float16 or std::float16_t (MSVC and gcc differ on both), and no float
// arithmetic in the rounding. So the result is bit-exact on every toolchain
// (TD-12) and does not depend on FTZ or DAZ, the lesson of fp32_math's
// subnormal paths.
//
// f32 -> f16 rounds to nearest, ties to even, and keeps subnormals and the
// sign of zero. NO-DATA, exactly: a NaN, an infinity, or any finite value
// whose rounded result would be infinite, that is |x| >= 65520 (values in
// [65504, 65520) round to 65504). A no-data sample is never saturated: it is
// written as kNoDataBits, a quiet NaN, and reported, so a reader draws it as
// no data and never as a value (L6).
// ---------------------------------------------------------------------------

#include <bit>
#include <cstdint>

namespace spade::fp16 {

inline constexpr uint16_t kNoDataBits = 0x7E00u;  // a quiet NaN, positive

struct Half {
    uint16_t bits = 0;
    bool no_data = false;  // true when `bits` is kNoDataBits for one of the reasons above
};

namespace detail {

// `m >> s`, rounded to nearest with ties to even, for 1 <= s <= 31.
[[nodiscard]] constexpr uint32_t shift_right_rne(uint32_t m, uint32_t s) noexcept {
    const uint32_t kept = m >> s;
    const uint32_t rest = m & ((1u << s) - 1u);
    const uint32_t half = 1u << (s - 1u);
    return kept + ((rest > half || (rest == half && (kept & 1u) != 0u)) ? 1u : 0u);
}

}  // namespace detail

[[nodiscard]] constexpr Half from_f32(float x) noexcept {
    const uint32_t f = std::bit_cast<uint32_t>(x);
    const auto sign = static_cast<uint16_t>((f >> 16) & 0x8000u);
    const uint32_t exp = (f >> 23) & 0xFFu;
    const uint32_t mant = f & 0x7FFFFFu;
    if (exp == 0xFFu) {
        return Half{kNoDataBits, true};  // NaN or infinity
    }
    if (exp < 102u) {
        // Below 2^-25, under half the smallest subnormal (f32 subnormals
        // included): rounds to zero, keeping the sign.
        return Half{sign, false};
    }
    // The 24-bit significand with its implicit bit, worth sig * 2^(exp - 150).
    const uint32_t sig = mant | 0x800000u;
    uint32_t magnitude = 0;
    if (exp < 113u) {
        // A half subnormal: units of 2^-24, so shift by 126 - exp (14 to 24).
        // A carry out of the mantissa lands on the smallest normal, 0x0400.
        magnitude = detail::shift_right_rne(sig, 126u - exp);
    } else {
        // A half normal: rebias the exponent and round the mantissa to 10
        // bits. A carry rolls into the exponent, and past 30 into infinity.
        magnitude = ((exp - 112u) << 10) + detail::shift_right_rne(sig, 13u) - 0x400u;
    }
    if (magnitude >= 0x7C00u) {
        return Half{kNoDataBits, true};  // |x| >= 65520: would round to infinity
    }
    return Half{static_cast<uint16_t>(sign | magnitude), false};
}

// Exact: every half is a float. A NaN half decodes to a quiet NaN.
[[nodiscard]] constexpr float to_f32(uint16_t h) noexcept {
    const uint32_t sign = static_cast<uint32_t>(h & 0x8000u) << 16;
    const uint32_t exp = (h >> 10) & 0x1Fu;
    uint32_t mant = h & 0x3FFu;
    if (exp == 0x1Fu) {
        return std::bit_cast<float>(sign | 0x7F800000u | (mant != 0u ? 0x400000u | (mant << 13) : 0u));
    }
    if (exp == 0u) {
        if (mant == 0u) {
            return std::bit_cast<float>(sign);  // +-0
        }
        // A subnormal half, mant * 2^-24: normalise the leading bit up to
        // bit 10, lowering the exponent once per shift.
        uint32_t e = 113u;  // the f32 exponent field of 2^-14
        while ((mant & 0x400u) == 0u) {
            mant <<= 1;
            --e;
        }
        return std::bit_cast<float>(sign | (e << 23) | ((mant & 0x3FFu) << 13));
    }
    return std::bit_cast<float>(sign | ((exp + 112u) << 23) | (mant << 13));
}

}  // namespace spade::fp16
