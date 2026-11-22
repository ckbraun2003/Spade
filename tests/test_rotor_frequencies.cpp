#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "vehicles/rotor_frequencies.hpp"

// ===========================================================================
// The frequencies a rotor makes (vehicles/rotor_frequencies.hpp, O169). Every expectation is the closed form, written here in double; none
// calls the function under test (TD-4).
// ===========================================================================

using namespace spade::vehicles;

namespace {

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// 155 revolutions a second (9,300 rpm), a 3-blade propeller, a 7-pole-pair motor, a 24 kHz ESC.
[[nodiscard]] FrequencyInputs film() {
    FrequencyInputs in;
    in.omega = kTwoPi * 155.0;
    in.blades = 3;
    in.blades_assumed = false;
    in.pole_pairs = 7;
    in.pole_pairs_assumed = true;
    in.pwm_hz = 24000.0;
    in.pwm_assumed = true;
    return in;
}

[[nodiscard]] bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

}  // namespace

TEST(RotorFrequencies, TheLinesFollowFromTheShaftSpeed) {
    const auto f = derive_frequencies(film());
    ASSERT_TRUE(f.has_value());
    EXPECT_NEAR(f->shaft_hz, 155.0, 1e-9);
    EXPECT_NEAR(f->blade_pass_hz, 3.0 * 155.0, 1e-9);
    EXPECT_NEAR(f->electrical_hz, 7.0 * 155.0, 1e-9);
    EXPECT_NEAR(f->commutation_hz, 6.0 * 7.0 * 155.0, 1e-8);
    EXPECT_EQ(f->pwm_hz, 24000.0);
    EXPECT_EQ(f->blades, 3u);
    EXPECT_EQ(f->pole_pairs, 7u);
}

TEST(RotorFrequencies, AStoppedRotorMakesNoLineButTheSwitchingOne) {
    FrequencyInputs in = film();
    in.omega = 0.0;
    const auto f = derive_frequencies(in);
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(f->shaft_hz, 0.0);
    EXPECT_EQ(f->blade_pass_hz, 0.0);
    EXPECT_EQ(f->electrical_hz, 0.0);
    EXPECT_EQ(f->commutation_hz, 0.0);
    EXPECT_EQ(f->pwm_hz, 24000.0);
}

TEST(RotorFrequencies, ABrushedMotorHasNoElectricalFrequency) {
    FrequencyInputs in = film();
    in.pole_pairs = 0;
    const auto f = derive_frequencies(in);
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(f->electrical_hz, 0.0);
    EXPECT_EQ(f->commutation_hz, 0.0);
    EXPECT_NEAR(f->blade_pass_hz, 465.0, 1e-9);
    const std::string text = frequency_readout(*f);
    EXPECT_TRUE(contains(text, "electrical none (a brushed motor)")) << text;
    EXPECT_FALSE(contains(text, "commutation")) << text;
}

TEST(RotorFrequencies, EachDerivedValueCarriesTheLabelOfItsInputs) {
    FrequencyInputs in = film();
    auto f = derive_frequencies(in);
    ASSERT_TRUE(f.has_value());
    EXPECT_FALSE(f->blade_pass_assumed);  // the blade count came from a graded part file
    EXPECT_TRUE(f->electrical_assumed);   // the 7 pole pairs are the library's assumption
    EXPECT_TRUE(f->pwm_assumed);
    in.blades_assumed = true;
    in.pole_pairs_assumed = false;
    in.pwm_assumed = false;
    f = derive_frequencies(in);
    ASSERT_TRUE(f.has_value());
    EXPECT_TRUE(f->blade_pass_assumed);
    EXPECT_FALSE(f->electrical_assumed);
    EXPECT_FALSE(f->pwm_assumed);
}

TEST(RotorFrequencies, TheReadoutNamesEveryAssumedValueOnItsOwnLine) {
    const auto f = derive_frequencies(film());
    ASSERT_TRUE(f.has_value());
    const std::string expected =
        "shaft 155.00 Hz\n"
        "blade pass 465.00 Hz (3 blades)\n"
        "electrical 1085.00 Hz (7 pole pairs, assumed)\n"
        "commutation 6510.00 Hz (6 per electrical cycle, assumed)\n"
        "pwm 24000 Hz (assumed)\n";
    EXPECT_EQ(frequency_readout(*f), expected);
}

TEST(RotorFrequencies, AnUnsetSwitchingFrequencyIsSaidSo) {
    FrequencyInputs in = film();
    in.pwm_hz = 0.0;
    const auto f = derive_frequencies(in);
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(f->pwm_hz, 0.0);
    EXPECT_TRUE(contains(frequency_readout(*f), "pwm not set\n"));
}

TEST(RotorFrequencies, HarmonicsAreTheMultiples) {
    std::array<double, 4> h{};
    harmonics(465.0, h);
    EXPECT_EQ(h[0], 465.0);
    EXPECT_EQ(h[1], 930.0);
    EXPECT_EQ(h[2], 1395.0);
    EXPECT_EQ(h[3], 1860.0);
    std::array<double, 0> none{};
    harmonics(465.0, none);  // an empty span writes nothing
}

TEST(RotorFrequencies, BadInputsAreRefusedByName) {
    const auto refused = [](FrequencyInputs in, const char* field) {
        const auto f = derive_frequencies(in);
        ASSERT_FALSE(f.has_value()) << field;
        EXPECT_EQ(f.error().code, spade::Code::invalid_argument) << field;
        EXPECT_NE(f.error().context.find(field), std::string::npos) << field << ": " << f.error().context;
    };
    FrequencyInputs in = film();
    in.omega = kNaN;
    refused(in, "omega");
    in = film();
    in.omega = kInf;
    refused(in, "omega");
    in = film();
    in.omega = -1.0;
    refused(in, "omega");
    in = film();
    in.blades = 0;
    refused(in, "blades");
    in = film();
    in.blades = 17;
    refused(in, "blades");
    in = film();
    in.pole_pairs = 65;
    refused(in, "pole_pairs");
    in = film();
    in.pwm_hz = kNaN;
    refused(in, "pwm_hz");
    in = film();
    in.pwm_hz = -1.0;
    refused(in, "pwm_hz");
}
