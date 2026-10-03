#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "vehicles/propeller.hpp"
#include "vehicles/propulsion_flags.hpp"

// ===========================================================================
// The drone builder's propeller coefficient tier (vehicles/propeller.hpp).
// Every expectation is the closed form, computed here in double (TD-4).
// ===========================================================================

using namespace spade::vehicles;

namespace {

namespace pf = spade::vehicles::propulsion_flags;

constexpr double kPi = 3.14159265358979323846;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kD = 0.1524;  // a 6-inch propeller, m

}  // namespace

TEST(Propeller, AdvanceRatioIsAxialSpeedOverNTimesD) {
    const double v = 5.0;
    const double w = 1500.0;
    const double n = w / (2.0 * kPi);
    EXPECT_NEAR(propeller_advance_ratio(v, w, kD), v / (n * kD), 1e-14);
    EXPECT_NEAR(propeller_advance_ratio(5.0f, 1500.0f, 0.1524f), v / (n * kD), 1e-6);
}

TEST(Propeller, ThrustAndTorqueFollowTheUiucConvention) {
    const double ct = 0.11;
    const double cq = 0.0075;
    const double rho = 1.225;
    const double w = 2000.0;
    const double n = w / (2.0 * kPi);
    EXPECT_NEAR(propeller_thrust(ct, rho, w, kD), ct * rho * n * n * std::pow(kD, 4.0), 1e-12);
    EXPECT_NEAR(propeller_torque(cq, rho, w, kD), cq * rho * n * n * std::pow(kD, 5.0), 1e-14);
}

TEST(Propeller, AStoppedPropellerMakesNothing) {
    EXPECT_EQ(propeller_advance_ratio(5.0, 0.0, kD), 0.0);
    EXPECT_EQ(propeller_thrust(0.11, 1.225, 0.0, kD), 0.0);
    EXPECT_EQ(propeller_torque(0.0075, 1.225, 0.0, kD), 0.0);
}

TEST(Propeller, CoefficientInterpolatesTheUniformGrid) {
    const std::vector<float> table = {0.12f, 0.10f, 0.07f, 0.03f, -0.01f};  // J = 0, 0.2, 0.4, 0.6, 0.8
    uint32_t flags = 0;
    EXPECT_NEAR(propeller_coefficient(table, 0.0, 0.8, 0.3, flags), 0.10 + (0.07 - 0.10) * 0.5, 1e-7);
    EXPECT_EQ(propeller_coefficient(table, 0.0, 0.8, 0.0, flags), double{0.12f});
    EXPECT_EQ(propeller_coefficient(table, 0.0, 0.8, 0.8, flags), double{-0.01f});
    EXPECT_EQ(flags, 0u);
}

TEST(Propeller, OutsideTheTableTheEndValueHoldsAndIsFlagged) {
    const std::vector<float> table = {0.12f, 0.10f, 0.07f};
    uint32_t flags = 0;
    EXPECT_EQ(propeller_coefficient(table, 0.0, 0.4, -0.2, flags), double{0.12f}) << "descent, J < 0";
    EXPECT_NE(flags & pf::out_of_table, 0u);
    flags = 0;
    EXPECT_EQ(propeller_coefficient(table, 0.0, 0.4, 0.9, flags), double{0.07f});
    EXPECT_NE(flags & pf::out_of_table, 0u);
    flags = 0;
    EXPECT_EQ(propeller_coefficient(table, 0.0, 0.4, kNaN, flags), 0.0);
    EXPECT_NE(flags & pf::out_of_table, 0u);
}

TEST(Propeller, ResampleBuildsOneGridOverBothTables) {
    const std::array<double, 2> ct[] = {{0.0, 0.12}, {0.2, 0.10}, {0.6, 0.04}};
    const std::array<double, 2> cq[] = {{0.0, 0.008}, {0.4, 0.006}};
    const auto t = propeller_resample_table(ct, cq, 7u);
    ASSERT_TRUE(t.has_value()) << t.error().context;
    EXPECT_EQ(t->j_min, 0.0f);
    EXPECT_EQ(t->j_max, static_cast<float>(0.6));
    ASSERT_EQ(t->ct.size(), 7u);
    ASSERT_EQ(t->cq.size(), 7u);
    for (uint32_t g = 0; g < 7u; ++g) {
        const double j = 0.0 + (0.6 - 0.0) * g / 6.0;
        const double c_t = j <= 0.2 ? 0.12 + (0.10 - 0.12) * (j - 0.0) / (0.2 - 0.0)
                                    : 0.10 + (0.04 - 0.10) * (j - 0.2) / (0.6 - 0.2);
        const double c_q = j >= 0.4 ? 0.006 : 0.008 + (0.006 - 0.008) * (j - 0.0) / (0.4 - 0.0);
        EXPECT_EQ(t->ct[g], static_cast<float>(c_t)) << "grid point " << g;
        EXPECT_EQ(t->cq[g], static_cast<float>(c_q)) << "grid point " << g << " (C_Q holds its end)";
    }
}

TEST(Propeller, ResampleRefusesBadPoints) {
    const std::array<double, 2> good[] = {{0.0, 0.1}, {0.5, 0.05}};
    EXPECT_FALSE(propeller_resample_table(good, good, 1u).has_value());
    const std::array<double, 2> one[] = {{0.0, 0.1}};
    EXPECT_FALSE(propeller_resample_table(one, good, 5u).has_value());
    const std::array<double, 2> back[] = {{0.5, 0.1}, {0.2, 0.05}};
    EXPECT_FALSE(propeller_resample_table(good, back, 5u).has_value());
    const std::array<double, 2> nan[] = {{0.0, kNaN}, {0.5, 0.05}};
    EXPECT_FALSE(propeller_resample_table(nan, good, 5u).has_value());
}

TEST(Propeller, DegenerateInputsReturnDefinedFiniteValues) {
    uint32_t flags = 0;
    EXPECT_EQ(propeller_advance_ratio(kNaN, 1500.0, kD), 0.0);
    EXPECT_EQ(propeller_advance_ratio(5.0, -10.0, kD), 0.0);
    EXPECT_EQ(propeller_advance_ratio(5.0, 1500.0, 0.0), 0.0);
    EXPECT_EQ(propeller_thrust(kNaN, 1.225, 2000.0, kD), 0.0);
    EXPECT_EQ(propeller_torque(0.0075, 1.225, std::numeric_limits<double>::infinity(), kD), 0.0);
    EXPECT_EQ(propeller_thrust(0.1f, 1.225f, 1e30f, 0.1524f), 0.0f) << "an overflowing thrust reads as 0";
    EXPECT_EQ(propeller_coefficient(std::vector<float>{}, 0.0, 1.0, 0.5, flags), 0.0);
    EXPECT_EQ(propeller_coefficient(std::vector<float>{0.1f, 0.2f}, 1.0, 1.0, 1.0, flags), 0.0) << "an empty range";
}
