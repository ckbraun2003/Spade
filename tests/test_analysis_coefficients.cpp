// The analysis's coefficients (Q6; DBC-030, DBC-031): rotorcraft and
// propeller conventions, each with its definition, against values computed
// by hand from the definitions in double precision.

#include <gtest/gtest.h>

#include <string>
#include <string_view>

#include "analysis/coefficients.hpp"

namespace {

namespace an = spade::analysis;

// T 5 N, Q 0.08 N m, Omega 1000 rad/s, R 0.13 m, rho 1.225 kg/m3.
const an::RotorLoads kLoads{.thrust = 5.0f, .torque = 0.08f, .omega = 1000.0f, .radius = 0.13f};
constexpr float kRho = 1.225f;

[[nodiscard]] const an::Coefficient* find(const std::vector<an::Coefficient>& list, std::string_view symbol) {
    for (const an::Coefficient& c : list) {
        if (c.symbol == symbol) return &c;
    }
    return nullptr;
}

}  // namespace

TEST(AnalysisCoefficients, TheDefaultConventionFollowsTheFlight) {
    EXPECT_EQ(an::default_convention(an::FlightKind::hover), an::Convention::rotorcraft);
    EXPECT_EQ(an::default_convention(an::FlightKind::forward), an::Convention::propeller);
    EXPECT_EQ(an::default_convention(an::FlightKind::hover_in_wind), an::Convention::propeller);
}

TEST(AnalysisCoefficients, RotorcraftInHover) {
    const auto c = an::rotor_coefficients(an::Convention::rotorcraft, kLoads, kRho, 0.0f);
    ASSERT_TRUE(c.has_value()) << c.error();
    const an::Coefficient* ct = find(*c, "C_T");
    const an::Coefficient* cq = find(*c, "C_Q");
    const an::Coefficient* cp = find(*c, "C_P");
    const an::Coefficient* fm = find(*c, "FM");
    ASSERT_TRUE(ct && cq && cp && fm);
    EXPECT_NEAR(ct->value, 0.00454894445, 1e-9);
    EXPECT_NEAR(cq->value, 0.000559870087, 1e-11);
    EXPECT_NEAR(cp->value, 0.000559870087, 1e-11);  // C_P = C_Q in this convention
    EXPECT_NEAR(fm->value, 0.387492846, 1e-6);
    // Each carries the definition a reader checks it against (DBC-031).
    EXPECT_NE(ct->definition.find("(Omega R)^2"), std::string_view::npos) << ct->definition;
    EXPECT_NE(fm->definition.find("C_T^(3/2)"), std::string_view::npos) << fm->definition;
}

// The figure of merit is a hover measure: none in a wind.
TEST(AnalysisCoefficients, RotorcraftUnderWindHasNoFigureOfMerit) {
    const auto c = an::rotor_coefficients(an::Convention::rotorcraft, kLoads, kRho, 4.0f);
    ASSERT_TRUE(c.has_value()) << c.error();
    EXPECT_EQ(find(*c, "FM"), nullptr);
    EXPECT_NE(find(*c, "C_T"), nullptr);
}

TEST(AnalysisCoefficients, PropellerUnderWind) {
    const auto c = an::rotor_coefficients(an::Convention::propeller, kLoads, kRho, 4.0f);
    ASSERT_TRUE(c.has_value()) << c.error();
    const an::Coefficient* ct = find(*c, "C_T");
    const an::Coefficient* cq = find(*c, "C_Q");
    const an::Coefficient* cp = find(*c, "C_P");
    const an::Coefficient* j = find(*c, "J");
    const an::Coefficient* eta = find(*c, "eta");
    ASSERT_TRUE(ct && cq && cp && j && eta);
    EXPECT_NEAR(ct->value, 0.0352614576, 1e-8);
    EXPECT_NEAR(cq->value, 0.00216993585, 1e-9);
    EXPECT_NEAR(cp->value, 0.0136341091, 1e-8);
    EXPECT_NEAR(j->value, 0.0966643893, 1e-8);
    EXPECT_NEAR(eta->value, 0.25, 1e-6);  // T V / P = 5 x 4 / (0.08 x 1000)
    EXPECT_NE(ct->definition.find("n^2 D^4"), std::string_view::npos) << ct->definition;
}

// With no wind a propeller has J = 0 and no efficiency to speak of.
TEST(AnalysisCoefficients, PropellerInStillAirHasNoEfficiency) {
    const auto c = an::rotor_coefficients(an::Convention::propeller, kLoads, kRho, 0.0f);
    ASSERT_TRUE(c.has_value()) << c.error();
    EXPECT_EQ(find(*c, "eta"), nullptr);
    ASSERT_NE(find(*c, "J"), nullptr);
    EXPECT_EQ(find(*c, "J")->value, 0.0);
}

TEST(AnalysisCoefficients, AStoppedOrShapelessRotorHasNone) {
    an::RotorLoads stopped = kLoads;
    stopped.omega = 0.0f;
    const auto a = an::rotor_coefficients(an::Convention::rotorcraft, stopped, kRho, 0.0f);
    ASSERT_FALSE(a.has_value());
    EXPECT_NE(a.error().find("stopped"), std::string::npos) << a.error();
    an::RotorLoads flat = kLoads;
    flat.radius = 0.0f;
    EXPECT_FALSE(an::rotor_coefficients(an::Convention::propeller, flat, kRho, 0.0f).has_value());
    EXPECT_FALSE(an::rotor_coefficients(an::Convention::propeller, kLoads, 0.0f, 0.0f).has_value());
}

TEST(AnalysisCoefficients, PressureCoefficientAndItsReference) {
    const auto cp = an::pressure_coefficient(101425.0f, 101325.0f, kRho, 4.0f);
    ASSERT_TRUE(cp.has_value()) << cp.error();
    EXPECT_NEAR(*cp, 10.2040816, 1e-5);
    EXPECT_FALSE(an::pressure_coefficient(1.0f, 0.0f, kRho, 0.0f).has_value());  // no reference speed
    // The reference speed: the tip speed in the rotorcraft convention, the freestream in the propeller one.
    EXPECT_FLOAT_EQ(an::reference_speed(an::Convention::rotorcraft, 1000.0f, 0.13f, 4.0f), 130.0f);
    EXPECT_FLOAT_EQ(an::reference_speed(an::Convention::propeller, 1000.0f, 0.13f, 4.0f), 4.0f);
}
