// core/fp16.hpp: binary16 conversions by pure integer bit manipulation.
// Checked exhaustively on the 65 536 halves, at the boundaries Core named,
// and on a strided f32 sweep against an independent reference that rounds
// in double (test-only; this file feeds no golden).

#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <vector>

#include "core/fp16.hpp"

namespace {

using spade::fp16::from_f32;
using spade::fp16::Half;
using spade::fp16::kNoDataBits;
using spade::fp16::to_f32;

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

// The half's value by the format's definition, in double: the independent
// reference for decoding.
[[nodiscard]] double half_value(uint16_t h) {
    const int sign = (h & 0x8000u) != 0u ? -1 : 1;
    const int exp = (h >> 10) & 0x1F;
    const int mant = h & 0x3FF;
    if (exp == 0) return sign * std::ldexp(static_cast<double>(mant), -24);
    return sign * std::ldexp(1.0 + mant / 1024.0, exp - 15);
}

// Every finite non-negative half, ascending, with its value.
struct HalfTable {
    std::vector<uint16_t> bits;
    std::vector<double> value;
};

[[nodiscard]] const HalfTable& finite_halves() {
    static const HalfTable table = [] {
        HalfTable t;
        for (uint32_t h = 0; h < 0x7C00u; ++h) {
            t.bits.push_back(static_cast<uint16_t>(h));
            t.value.push_back(half_value(static_cast<uint16_t>(h)));
        }
        return t;
    }();
    return table;
}

// The reference f32 -> f16: the nearest half by exhaustive search, ties to
// the even mantissa, no-data past the largest half's rounding boundary.
[[nodiscard]] Half reference_from_f32(float x) {
    if (std::isnan(x) || std::isinf(x)) return Half{kNoDataBits, true};
    const double a = std::fabs(static_cast<double>(x));
    const uint16_t sign = std::signbit(x) ? 0x8000u : 0u;
    if (a >= 65520.0) return Half{kNoDataBits, true};
    const HalfTable& t = finite_halves();
    size_t lo = 0, hi = t.value.size() - 1;
    while (hi - lo > 1) {
        const size_t mid = (lo + hi) / 2;
        (t.value[mid] <= a ? lo : hi) = mid;
    }
    if (a >= t.value[hi]) lo = hi;
    uint16_t pick = t.bits[lo];
    if (lo + 1 < t.value.size() && a > t.value[lo]) {
        const double below = a - t.value[lo], above = t.value[lo + 1] - a;
        if (above < below || (above == below && (t.bits[lo + 1] & 1u) == 0u)) pick = t.bits[lo + 1];
    }
    return Half{static_cast<uint16_t>(sign | pick), false};
}

[[nodiscard]] bool is_nan_half(uint16_t h) { return (h & 0x7C00u) == 0x7C00u && (h & 0x3FFu) != 0u; }
[[nodiscard]] bool is_inf_half(uint16_t h) { return (h & 0x7FFFu) == 0x7C00u; }

// Every half decodes to its value, and every finite one encodes back to
// itself. NaN and infinite halves decode to NaN and infinity, and encode
// back to no-data.
TEST(CoreFp16, EveryHalfRoundTrips) {
    for (uint32_t i = 0; i <= 0xFFFFu; ++i) {
        const auto h = static_cast<uint16_t>(i);
        const float f = to_f32(h);
        const Half back = from_f32(f);
        if (is_nan_half(h)) {
            ASSERT_TRUE(std::isnan(f)) << std::hex << i;
            ASSERT_TRUE(back.no_data) << std::hex << i;
            ASSERT_EQ(back.bits, kNoDataBits) << std::hex << i;
        } else if (is_inf_half(h)) {
            ASSERT_TRUE(std::isinf(f)) << std::hex << i;
            ASSERT_EQ(std::signbit(f), (h & 0x8000u) != 0u) << std::hex << i;
            ASSERT_TRUE(back.no_data) << std::hex << i;
        } else {
            ASSERT_EQ(static_cast<double>(f), half_value(h)) << std::hex << i;
            ASSERT_EQ(std::signbit(f), (h & 0x8000u) != 0u) << std::hex << i;
            ASSERT_FALSE(back.no_data) << std::hex << i;
            ASSERT_EQ(back.bits, h) << std::hex << i;
        }
    }
}

// The boundaries Core named.
TEST(CoreFp16, BoundariesRoundAsNamed) {
    EXPECT_EQ(from_f32(65504.0f).bits, 0x7BFFu) << "the largest half";
    EXPECT_FALSE(from_f32(65504.0f).no_data);
    EXPECT_EQ(from_f32(65519.99f).bits, 0x7BFFu) << "under the boundary rounds down to 65504";
    EXPECT_FALSE(from_f32(65519.99f).no_data);
    EXPECT_TRUE(from_f32(65520.0f).no_data) << "the tie at 65520 rounds to even, which is infinity";
    EXPECT_TRUE(from_f32(-65520.0f).no_data);
    EXPECT_EQ(from_f32(-65520.0f).bits, kNoDataBits) << "never saturated, whatever the sign";
    EXPECT_TRUE(from_f32(1.0e9f).no_data);

    const float smallest = std::ldexp(1.0f, -24);  // the smallest subnormal half
    EXPECT_EQ(from_f32(smallest).bits, 0x0001u);
    EXPECT_EQ(from_f32(smallest * 0.5f).bits, 0x0000u) << "half the smallest is a tie, and 0 is even";
    EXPECT_EQ(from_f32(smallest * 0.75f).bits, 0x0001u) << "above the tie rounds up";
    EXPECT_EQ(from_f32(smallest * 1.5f).bits, 0x0002u) << "a tie between 1 and 2 goes to 2, the even one";
    EXPECT_EQ(from_f32(smallest * 2.5f).bits, 0x0002u) << "a tie between 2 and 3 goes to 2";
    EXPECT_EQ(from_f32(-0.0f).bits, 0x8000u) << "-0 stays -0";
    EXPECT_EQ(from_f32(std::numeric_limits<float>::denorm_min()).bits, 0x0000u) << "an f32 subnormal is 0 in f16";

    // Normal ties: 1 + 2^-11 lies halfway between 1 (even) and 1 + 2^-10 (odd).
    EXPECT_EQ(from_f32(1.0f + std::ldexp(1.0f, -11)).bits, 0x3C00u) << "a tie to the even mantissa, down";
    EXPECT_EQ(from_f32(1.0f + 3.0f * std::ldexp(1.0f, -11)).bits, 0x3C02u) << "a tie to the even mantissa, up";
    EXPECT_EQ(from_f32(std::ldexp(1.0f, -14) * (2.0f - std::ldexp(1.0f, -11))).bits, 0x0800u)
        << "rounding carries into the exponent";
    EXPECT_EQ(from_f32(std::ldexp(1.0f, -14) * (1.0f - std::ldexp(1.0f, -11))).bits, 0x0400u)
        << "a subnormal rounding up becomes the smallest normal";

    EXPECT_TRUE(from_f32(std::numeric_limits<float>::infinity()).no_data);
    EXPECT_TRUE(from_f32(std::numeric_limits<float>::quiet_NaN()).no_data);
}

// A strided sweep over every f32 bit pattern's sign, exponent and mantissa,
// against the reference.
TEST(CoreFp16, AStridedF32SweepMatchesTheReference) {
    uint64_t checked = 0;
    for (uint64_t i = 0; i <= 0xFFFFFFFFull; i += 4099u) {
        const auto x = std::bit_cast<float>(static_cast<uint32_t>(i));
        const Half got = from_f32(x);
        const Half want = reference_from_f32(x);
        ASSERT_EQ(got.no_data, want.no_data) << "x = " << x << " (bits " << std::hex << i << ")";
        if (!want.no_data) {
            ASSERT_EQ(got.bits, want.bits) << "x = " << x << " (bits " << std::hex << i << ")";
        }
        ++checked;
    }
    EXPECT_GT(checked, 1000000u);
}

// Every f32 bit pattern: about six minutes on release (348 s measured,
// 2026-10-05); opt-in, like Fp32Exp's full sweep.
TEST(CoreFp16, FullDomainSweepEveryFloat) {
    if (!env_gate_is_set("SPADE_FULL_FP16_SWEEP")) {
        GTEST_SKIP() << "set SPADE_FULL_FP16_SWEEP=1 to run the full 2^32 sweep";
    }
    for (uint64_t i = 0; i <= 0xFFFFFFFFull; ++i) {
        const auto x = std::bit_cast<float>(static_cast<uint32_t>(i));
        const Half got = from_f32(x);
        const Half want = reference_from_f32(x);
        ASSERT_EQ(got.no_data, want.no_data) << std::hex << i;
        if (!want.no_data) {
            ASSERT_EQ(got.bits, want.bits) << std::hex << i;
        }
    }
}

}  // namespace
