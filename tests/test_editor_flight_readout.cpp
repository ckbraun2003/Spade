// Run air's flight readout (sandbox/editor_flight_readout.hpp), display-free: the formulas against hand values, and
// the labels that say what the numbers are.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <string_view>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../sandbox/editor_flight_readout.hpp"

using namespace spade::sandbox::editor;

namespace {

constexpr float kPi = 3.14159265358979f;

FlightInput level_in(glm::vec3 ambient) {
    FlightInput in;
    in.ambient = ambient;
    in.mass = 0.05f;
    return in;
}

bool contains(std::string_view text, std::string_view part) { return text.find(part) != std::string_view::npos; }

std::string row_text(const std::vector<FlightRow>& rows) {
    std::string all;
    for (const FlightRow& r : rows) all += r.label + " | " + r.value + "\n";
    return all;
}

}  // namespace

// A calm hover has no relative wind: the angles are not defined, so the readout says there is no flow and shows none.
TEST(EditorFlightReadout, ACalmHoverHasNoFlowAndNoAngles) {
    const FlightReadout r = flight_readout(level_in(glm::vec3(0.0f)));
    EXPECT_FALSE(r.flow);
    EXPECT_EQ(r.airspeed_mps, 0.0f);
    EXPECT_EQ(r.aoa_deg, 0.0f);
    EXPECT_EQ(r.sideslip_deg, 0.0f);
    EXPECT_TRUE(contains(row_text(flight_rows(r)), "no relative wind"));
}

// The case's air moves toward -X for a head-on flow (heading 0: it comes from the nose), so a level drone meets 10 m/s
// at no angle.
TEST(EditorFlightReadout, AHeadOnFlowIsAirspeedAtNoAngle) {
    const FlightReadout r = flight_readout(level_in(glm::vec3(-10.0f, 0.0f, 0.0f)));
    EXPECT_TRUE(r.flow);
    EXPECT_NEAR(r.airspeed_mps, 10.0f, 1e-5f);
    EXPECT_NEAR(r.aoa_deg, 0.0f, 1e-4f);
    EXPECT_NEAR(r.sideslip_deg, 0.0f, 1e-4f);
}

// Nose up by 10 degrees (a rotation about +Z) in the same flow: the air meets the underside, so the angle of attack is
// +10 degrees and nothing else changes.
TEST(EditorFlightReadout, NoseUpAgainstAHeadOnFlowIsPositiveAngleOfAttack) {
    FlightInput in = level_in(glm::vec3(-10.0f, 0.0f, 0.0f));
    in.orient = glm::angleAxis(10.0f * kPi / 180.0f, glm::vec3(0.0f, 0.0f, 1.0f));
    const FlightReadout r = flight_readout(in);
    EXPECT_NEAR(r.airspeed_mps, 10.0f, 1e-5f);
    EXPECT_NEAR(r.aoa_deg, 10.0f, 1e-3f);
    EXPECT_NEAR(r.sideslip_deg, 0.0f, 1e-3f);
    EXPECT_NEAR(r.pitch_deg, 10.0f, 1e-3f);
}

// Heading 30 (from the left, -Z): the case's air is -s * (cos 30, 0, -sin 30). The air comes from the left of the nose,
// so the sideslip is -30 degrees (+ is from the right) and the angle of attack stays 0.
TEST(EditorFlightReadout, AFlowFromTheLeftIsNegativeSideslip) {
    const float s = 8.0f, h = 30.0f * kPi / 180.0f;
    const FlightReadout r = flight_readout(level_in(-s * glm::vec3(std::cos(h), 0.0f, -std::sin(h))));
    EXPECT_NEAR(r.airspeed_mps, 8.0f, 1e-5f);
    EXPECT_NEAR(r.sideslip_deg, -30.0f, 1e-3f);
    EXPECT_NEAR(r.aoa_deg, 0.0f, 1e-3f);
}

// A pure crosswind from the left meets the drone at -90 degrees of sideslip and, with no component along the nose or
// the vertical, a defined angle of attack of 0.
TEST(EditorFlightReadout, ACrosswindIsMinusNinetyDegreesOfSideslip) {
    const FlightReadout r = flight_readout(level_in(glm::vec3(0.0f, 0.0f, 5.0f)));
    EXPECT_NEAR(r.sideslip_deg, -90.0f, 1e-3f);
    EXPECT_NEAR(r.aoa_deg, 0.0f, 1e-3f);
}

// Rotors: rpm = w * 60 / 2 pi, the thrust the model's static curve k_T w^2, the total against m g.
TEST(EditorFlightReadout, EachRotorsRpmAndStaticThrustAndTheTotalAgainstTheWeight) {
    const std::array<float, 2> omega{100.0f, 200.0f};
    const std::array<float, 2> kt{1.0e-5f, 1.0e-5f};
    FlightInput in = level_in(glm::vec3(0.0f));
    in.rotor_omega = omega;
    in.thrust_coeff = kt;
    const FlightReadout r = flight_readout(in);
    ASSERT_EQ(r.rotors.size(), 2u);
    EXPECT_NEAR(r.rotors[0].rpm, 954.9297f, 1e-2f);
    EXPECT_NEAR(r.rotors[1].rpm, 1909.8593f, 1e-2f);
    EXPECT_NEAR(r.rotors[0].thrust_n, 0.1f, 1e-6f);
    EXPECT_NEAR(r.rotors[1].thrust_n, 0.4f, 1e-6f);
    EXPECT_NEAR(r.thrust_n, 0.5f, 1e-6f);
    EXPECT_NEAR(r.weight_n, 0.05f * 9.80665f, 1e-6f);
}

// A rotor with no k_T on record shows its speed and a thrust of 0 rather than reading past the end.
TEST(EditorFlightReadout, ARotorWithNoThrustCoefficientShowsNoThrust) {
    const std::array<float, 2> omega{100.0f, 200.0f};
    const std::array<float, 1> kt{1.0e-5f};
    FlightInput in = level_in(glm::vec3(0.0f));
    in.rotor_omega = omega;
    in.thrust_coeff = kt;
    const FlightReadout r = flight_readout(in);
    ASSERT_EQ(r.rotors.size(), 2u);
    EXPECT_EQ(r.rotors[1].thrust_n, 0.0f);
    EXPECT_GT(r.rotors[1].rpm, 0.0f);
}

TEST(EditorFlightReadout, AttitudeAndRatesAreInDegrees) {
    FlightInput in = level_in(glm::vec3(0.0f));
    in.omega_body = glm::vec3(1.0f, 0.0f, 0.0f);
    const FlightReadout r = flight_readout(in);
    EXPECT_NEAR(r.rates_deg_s.x, 57.29578f, 1e-3f);
    EXPECT_EQ(r.yaw_deg, 0.0f);
    EXPECT_EQ(r.roll_deg, 0.0f);
}

// The labels say what the numbers are (the lead's honesty rule): the thrust is the static curve, the angles are
// against the case's flow direction, and the panel names the loads it does not show.
TEST(EditorFlightReadout, TheLabelsSayWhatTheNumbersAre) {
    const std::array<float, 1> omega{100.0f};
    const std::array<float, 1> kt{1.0e-5f};
    FlightInput in = level_in(glm::vec3(-10.0f, 0.0f, 0.0f));
    in.rotor_omega = omega;
    in.thrust_coeff = kt;
    const std::string rows = row_text(flight_rows(flight_readout(in)));
    EXPECT_TRUE(contains(rows, "Airspeed")) << rows;
    EXPECT_TRUE(contains(rows, "Angle of attack")) << rows;
    EXPECT_TRUE(contains(rows, "Sideslip")) << rows;
    EXPECT_TRUE(contains(rows, "against the case's flow")) << rows;
    EXPECT_TRUE(contains(rows, "static curve")) << rows;
    EXPECT_TRUE(contains(rows, "weight")) << rows;
    // The row labels are elided in the panel's label column, so the wrapped note under them says it whole.
    EXPECT_TRUE(contains(kFlightNotShown, "static curve")) << kFlightNotShown;
    EXPECT_TRUE(contains(kFlightNotShown, "case's flow direction")) << kFlightNotShown;
    EXPECT_TRUE(contains(kFlightNotShown, "aero loads on the body"));
    EXPECT_TRUE(contains(kFlightNotShown, "stage 7"));
}

// A held-level drone's yaw is a hair under zero from rounding: the panel shows 0.0, never -0.0 (the snapshot read
// "yaw -0.0"), and a flow at no angle reads +0.0.
TEST(EditorFlightReadout, ANegativeZeroNeverShows) {
    FlightInput in = level_in(glm::vec3(-10.0f, 0.0f, 0.0f));
    in.orient = glm::angleAxis(-1.0e-8f, glm::vec3(0.0f, 1.0f, 0.0f));
    in.omega_body = glm::vec3(-1.0e-9f, 0.0f, 0.0f);
    const std::string rows = row_text(flight_rows(flight_readout(in)));
    EXPECT_FALSE(contains(rows, "-0.0")) << rows;
}

// The panel's value column is about 26 characters wide (the editor_airflow snapshot clipped the old Attitude value
// at "yaw -0.0\xC2\xB0, pitch 0.0\xC2\xB0, roll"): every value fits it, at the largest figures the rows show.
TEST(EditorFlightReadout, EveryValueFitsTheValueColumn) {
    const std::array<float, 2> omega{1300.0f, 1300.0f};
    const std::array<float, 2> kt{1.2e-5f, 1.2e-5f};
    FlightInput in = level_in(glm::vec3(-30.0f, 0.0f, 20.0f));
    in.orient = glm::angleAxis(2.5f, glm::normalize(glm::vec3(0.3f, 0.5f, 0.2f)));
    in.omega_body = glm::vec3(-6.0f, 5.0f, -4.0f);
    in.mass = 12.0f;
    in.rotor_omega = omega;
    in.thrust_coeff = kt;
    for (const FlightRow& r : flight_rows(flight_readout(in))) {
        EXPECT_LE(r.value.size(), 26u) << r.label << ": '" << r.value << "'";
    }
}
