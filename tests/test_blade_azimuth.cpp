// P11, the blade azimuth (physics/blade_azimuth.hpp): psi <- psi + omega h,
// wrapped by subtracting a pinned float 2 pi, held on the host; the bits of a
// sequence are pinned (computed with IEEE float32 + - *, round to nearest).

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <vector>

#include "physics/blade_azimuth.hpp"

namespace {

using spade::physics::advance_blade_azimuth;
using spade::physics::advance_blade_azimuths;
using spade::physics::kAzimuthTwoPi;
using spade::vehicles::RotorRow;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

RotorRow row(uint32_t enabled, float omega) {
    RotorRow r{};
    r.enabled = enabled;
    r.omega = omega;
    return r;
}

TEST(BladeAzimuth, TheTwoPiIsTheFloatNearestTwoPi) {
    EXPECT_EQ(std::bit_cast<uint32_t>(kAzimuthTwoPi), 0x40C90FDBu);
}

TEST(BladeAzimuth, ASequenceWrapsByThePinnedTwoPiBitForBit) {
    // omega 1000 rad/s, h 2 ms: a step of exactly 2.0f; the 4th, 7th and 10th
    // steps wrap.
    constexpr std::array<uint32_t, 10> kExpected = {0x40000000u, 0x40800000u, 0x40c00000u, 0x3fdbc094u, 0x406de04au,
                                                    0x40b6f025u, 0x3fb78128u, 0x405bc094u, 0x40ade04au, 0x3f9341bcu};
    float psi = 0.0f;
    for (std::size_t i = 0; i < kExpected.size(); ++i) {
        ASSERT_TRUE(advance_blade_azimuth(psi, 1000.0f, 0.002f));
        EXPECT_EQ(std::bit_cast<uint32_t>(psi), kExpected[i]) << "step " << i + 1;
    }
}

TEST(BladeAzimuth, ALongRunStaysInOneTurn) {
    // omega 4000 rad/s, h 4 ms: 16 rad a step, two or three wraps each.
    float psi = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        ASSERT_TRUE(advance_blade_azimuth(psi, 4000.0f, 0.004f));
        ASSERT_GE(psi, 0.0f);
        ASSERT_LT(psi, kAzimuthTwoPi);
    }
    EXPECT_EQ(std::bit_cast<uint32_t>(psi), 0x404091DAu);
}

TEST(BladeAzimuth, ExactlyTwoPiWrapsToZero) {
    float psi = 0.0f;
    ASSERT_TRUE(advance_blade_azimuth(psi, kAzimuthTwoPi, 1.0f));
    EXPECT_EQ(std::bit_cast<uint32_t>(psi), 0u);
}

TEST(BladeAzimuth, ZeroOmegaHoldsBitForBit) {
    float psi = 1.25f;
    ASSERT_TRUE(advance_blade_azimuth(psi, 0.0f, 0.002f));
    EXPECT_EQ(psi, 1.25f);
}

TEST(BladeAzimuth, TheLargestStepIsTakenAndALargerOneRefused) {
    float psi = 0.5f;
    ASSERT_TRUE(advance_blade_azimuth(psi, 64.0f, 1.0f));
    EXPECT_GE(psi, 0.0f);
    EXPECT_LT(psi, kAzimuthTwoPi);
    float held = 0.5f;
    EXPECT_FALSE(advance_blade_azimuth(held, 100.0f, 1.0f));
    EXPECT_EQ(held, 0.5f);
}

TEST(BladeAzimuth, EveryRefusalHoldsPsi) {
    struct Case {
        float psi, omega, h;
    };
    const std::vector<Case> cases = {
        {0.5f, kNaN, 0.002f},  {0.5f, kInf, 0.002f},  {0.5f, -1.0f, 0.002f}, {0.5f, 1000.0f, -0.002f},
        {0.5f, 1000.0f, kNaN}, {0.5f, 1.0e30f, 1.0f}, {7.0f, 1000.0f, 0.002f}, {-0.1f, 1000.0f, 0.002f},
        {kNaN, 1000.0f, 0.002f}, {kAzimuthTwoPi, 1000.0f, 0.002f},
    };
    for (const Case& c : cases) {
        float psi = c.psi;
        EXPECT_FALSE(advance_blade_azimuth(psi, c.omega, c.h)) << c.psi << " " << c.omega << " " << c.h;
        EXPECT_EQ(std::bit_cast<uint32_t>(psi), std::bit_cast<uint32_t>(c.psi));
    }
}

TEST(BladeAzimuths, ADisabledRotorHoldsWhateverItsStaleOmega) {
    const std::array<RotorRow, 2> rows = {row(1u, 1000.0f), row(0u, 1000.0f)};
    std::array<float, 2> psi = {0.0f, 0.75f};
    ASSERT_TRUE(advance_blade_azimuths(psi, rows, 0.002f));
    EXPECT_EQ(psi[0], 2.0f);
    EXPECT_EQ(psi[1], 0.75f);
}

TEST(BladeAzimuths, ARefusedRotorHoldsAndTheOthersAdvance) {
    const std::array<RotorRow, 3> rows = {row(1u, 1000.0f), row(1u, kNaN), row(1u, 500.0f)};
    std::array<float, 3> psi = {0.0f, 0.25f, 0.0f};
    EXPECT_FALSE(advance_blade_azimuths(psi, rows, 0.002f));
    EXPECT_EQ(psi[0], 2.0f);
    EXPECT_EQ(psi[1], 0.25f);
    EXPECT_EQ(psi[2], 1.0f);
}

TEST(BladeAzimuths, SpansOfDifferentLengthsMoveNothing) {
    const std::array<RotorRow, 2> rows = {row(1u, 1000.0f), row(1u, 1000.0f)};
    std::array<float, 1> psi = {0.0f};
    EXPECT_FALSE(advance_blade_azimuths(psi, rows, 0.002f));
    EXPECT_EQ(psi[0], 0.0f);
}

}  // namespace
