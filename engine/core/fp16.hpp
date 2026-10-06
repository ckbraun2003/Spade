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

// RED: not yet implemented.
[[nodiscard]] constexpr Half from_f32(float) noexcept { return Half{}; }

// Exact: every half is a float. A NaN half decodes to a quiet NaN.
// RED: not yet implemented.
[[nodiscard]] constexpr float to_f32(uint16_t) noexcept { return 0.0f; }

}  // namespace spade::fp16
