// test_srgb.cpp -- core/srgb.hpp: the linear to sRGB byte, rounded to the nearest integer with no libm call (the engine has none: BitPortability). It
// moved here from world/mesh_import/colour.hpp (internal to the readers) so that Rendering's camera shading can call it by a public name; the camera's exact
// bytes are now a contract, so the function is pinned at the name callers use. This test uses libm (std::pow, std::nextafter): tests may, engine/ may not.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "core/srgb.hpp"

namespace {

// The sRGB decoding of a code (s in 0..1), the definition the table was made from.
double linear_from_srgb(double s) { return s <= 0.04045 ? s / 12.92 : std::pow((s + 0.055) / 1.055, 2.4); }

}  // namespace

TEST(Srgb, EveryCodeRoundTripsThroughItsLinearValue) {
    for (int k = 0; k <= 255; ++k) {
        EXPECT_EQ(static_cast<int>(spade::srgb_from_linear(linear_from_srgb(k / 255.0))), k) << "code " << k;
    }
}

TEST(Srgb, TheThresholdTableIsStrictlyIncreasingAndInsideTheUnitRange) {
    ASSERT_EQ(spade::kSrgbThreshold.size(), 255u);
    EXPECT_GT(spade::kSrgbThreshold.front(), 0.0);
    EXPECT_LT(spade::kSrgbThreshold.back(), 1.0);
    for (std::size_t k = 1; k < spade::kSrgbThreshold.size(); ++k) {
        EXPECT_GT(spade::kSrgbThreshold[k], spade::kSrgbThreshold[k - 1]) << "entry " << k;
    }
}

TEST(Srgb, EachEntryIsTheLinearValueWhereTheCodeTurnsOverAndMatchesItsDefinition) {
    for (std::size_t k = 0; k < spade::kSrgbThreshold.size(); ++k) {
        const double t = spade::kSrgbThreshold[k];
        const double expected = linear_from_srgb((static_cast<double>(k) + 0.5) / 255.0);
        EXPECT_NEAR(t, expected, expected * 1e-12) << "entry " << k << " drifted from the formula it was made from";
        EXPECT_EQ(static_cast<int>(spade::srgb_from_linear(t)), static_cast<int>(k) + 1) << "at the threshold the code is the next one up";
        EXPECT_EQ(static_cast<int>(spade::srgb_from_linear(std::nextafter(t, 0.0))), static_cast<int>(k)) << "just below it, the lower one";
    }
}

TEST(Srgb, OutOfRangeAndNonFiniteInputsClamp) {
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_EQ(spade::srgb_from_linear(0.0), 0);
    EXPECT_EQ(spade::srgb_from_linear(-1.0), 0);
    EXPECT_EQ(spade::srgb_from_linear(-inf), 0);
    EXPECT_EQ(spade::srgb_from_linear(std::numeric_limits<double>::quiet_NaN()), 0) << "a NaN is no colour";
    EXPECT_EQ(spade::srgb_from_linear(1.0), 255);
    EXPECT_EQ(spade::srgb_from_linear(2.0), 255);
    EXPECT_EQ(spade::srgb_from_linear(inf), 255);
}
