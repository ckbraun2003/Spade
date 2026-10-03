#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "vehicles/motor.hpp"
#include "vehicles/propulsion_flags.hpp"

// ===========================================================================
// The drone builder's motor and ESC (vehicles/motor.hpp). Every expectation is
// the closed form, computed here in double -- never a call to the function
// under test (TD-4).
// ===========================================================================

using namespace spade::vehicles;

namespace {

namespace pf = spade::vehicles::propulsion_flags;

constexpr double kPi = 3.14159265358979323846;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr float kNaNf = std::numeric_limits<float>::quiet_NaN();
constexpr float kInff = std::numeric_limits<float>::infinity();

// A 2207-class motor: 2450 rpm/V, 70 mOhm plus 5 mOhm of ESC, 1.2 A no-load.
constexpr double kKvRpm = 2450.0;
constexpr double kR = 0.075;
constexpr double kI0 = 1.2;

[[nodiscard]] double kv() { return kKvRpm * kPi / 30.0; }

}  // namespace

TEST(Motor, KvConvertsRpmPerVoltToRadiansPerSecondPerVolt) {
    EXPECT_NEAR(motor_kv_si(kKvRpm), kv(), 1e-12 * kv());
    EXPECT_NEAR(motor_kv_si(static_cast<float>(kKvRpm)), kv(), 1e-6 * kv());
}

TEST(Motor, NoLoadSpeedDrawsNoCurrentAndStallDrawsVOverR) {
    const double v = 16.8;
    EXPECT_NEAR(motor_current(1.0, v, 0.0, kv(), kR), v / kR, 1e-12 * v / kR);
    EXPECT_NEAR(motor_current(1.0, v, kv() * v, kv(), kR), 0.0, 1e-9);
    // Stall torque, Kt (V/R - I0) with Kt = 1/Kv.
    const double stall = (v / kR - kI0) / kv();
    EXPECT_NEAR(motor_torque(v / kR, kI0, kv(), 0.0), stall, 1e-12 * stall);
}

TEST(Motor, EffectiveResistanceIsRPlusReactanceSquaredOverR) {
    const double p = 7.0;
    const double l = 15e-6;
    const double w = 1900.0;
    const double x = p * w * l;
    EXPECT_NEAR(motor_effective_resistance(kR, p, l, w), kR + x * x / kR, 1e-12);
    EXPECT_EQ(motor_effective_resistance(kR, p, 0.0, w), kR) << "L = 0 is the resistive model exactly";
    EXPECT_EQ(motor_effective_resistance(kR, 0.0, l, w), kR) << "p = 0 is the resistive model exactly";
    EXPECT_EQ(motor_effective_resistance(kR, p, l, 0.0), kR) << "a stopped motor has no reactance";
}

TEST(Motor, SpeedTargetIsTheSteadyStateOfTheShaftEquation) {
    const double d = 0.6;
    const double v = 16.2;
    const double load = 0.08;
    const double w_inf = motor_speed_target(d, v, kv(), kR, kI0, load);
    // At omega_inf the motor's torque equals the load: Kt (I - I0) = Q_load.
    const double i = (d * v - w_inf / kv()) / kR;
    EXPECT_NEAR((i - kI0) / kv(), load, 1e-12);
}

TEST(Motor, TimeConstantIsInertiaTimesResistanceTimesKvSquared) {
    const double j = 2.5e-5;
    EXPECT_NEAR(motor_time_constant(j, kR, kv()), j * kR * kv() * kv(), 1e-15);
}

TEST(Motor, ExponentialStepMatchesTheClosedFormSolution) {
    // dw/dt = (w_inf - w)/tau, exactly w_inf + (w - w_inf) e^(-h/tau).
    const float h = 0.001f;
    const float tau = 0.12f;
    const float w0 = 500.0f;
    const float w_inf = 2000.0f;
    uint32_t flags = 0;
    const float alpha = motor_alpha(h, tau);
    EXPECT_NEAR(alpha, 1.0 - std::exp(-static_cast<double>(h) / static_cast<double>(tau)), 1e-7);
    const float w1 = motor_speed_step(w0, w_inf, alpha, flags);
    const double ref = w_inf + (w0 - w_inf) * std::exp(-static_cast<double>(h) / static_cast<double>(tau));
    EXPECT_NEAR(w1, ref, 1e-6 * ref);
    EXPECT_EQ(flags, 0u);
}

TEST(Motor, LimitedStepIsTheLinearUpdate) {
    uint32_t flags = 0;
    const double w = motor_speed_step_limited(800.0, 0.2, 0.05, 2.5e-5, 0.001, flags);
    EXPECT_NEAR(w, 800.0 + 0.001 * (0.2 - 0.05) / 2.5e-5, 1e-9);
    EXPECT_EQ(flags, 0u);
}

TEST(Motor, TheShaftNeverTurnsBackwards) {
    uint32_t flags = 0;
    EXPECT_EQ(motor_speed_step(100.0f, -5000.0f, 1.0f, flags), 0.0f);
    EXPECT_NE(flags & pf::speed_clamped, 0u);
    flags = 0;
    EXPECT_EQ(motor_speed_step_limited(1.0, -10.0, 0.0, 2.5e-5, 0.01, flags), 0.0);
    EXPECT_NE(flags & pf::speed_clamped, 0u);
}

TEST(Motor, FrictionHoldsAStoppedShaftButBrakesAMovingOne) {
    // Below the no-load current, a stopped shaft stays put ...
    EXPECT_EQ(motor_torque(0.5, kI0, kv(), 0.0), 0.0);
    // ... while a moving one decelerates by Kt (I - I0).
    EXPECT_NEAR(motor_torque(0.5, kI0, kv(), 100.0), (0.5 - kI0) / kv(), 1e-15);
}

TEST(Motor, CurrentClampsAtItsLimitsAndFlags) {
    uint32_t flags = 0;
    EXPECT_EQ(motor_clamp_current(60.0, 0.0, 50.0, flags), 50.0);
    EXPECT_NE(flags & pf::current_limited, 0u);
    flags = 0;
    EXPECT_EQ(motor_clamp_current(-3.0, 0.0, 50.0, flags), 0.0) << "no braking: the ESC sources only";
    EXPECT_NE(flags & pf::current_limited, 0u);
    flags = 0;
    EXPECT_EQ(motor_clamp_current(-3.0, -50.0, 50.0, flags), -3.0) << "active braking passes it";
    EXPECT_EQ(flags, 0u);
}

TEST(Motor, TheEscClampsItsDutyAndFlags) {
    uint32_t flags = 0;
    EXPECT_EQ(esc_clamp_duty(0.5, flags), 0.5);
    EXPECT_EQ(flags, 0u);
    EXPECT_EQ(esc_clamp_duty(-0.1, flags), 0.0);
    EXPECT_NE(flags & pf::duty_clamped, 0u);
    flags = 0;
    EXPECT_EQ(esc_clamp_duty(1.2f, flags), 1.0f);
    EXPECT_NE(flags & pf::duty_clamped, 0u);
    flags = 0;
    EXPECT_EQ(esc_clamp_duty(kNaN, flags), 0.0);
    EXPECT_NE(flags & pf::duty_clamped, 0u);
}

TEST(Motor, FloatAndDoubleAreTheSameOperationsToFloatRounding) {
    // Inputs exact in float, so any gap is the arithmetic's rounding.
    const float d = 0.5f;
    const float v = 16.0f;
    const float w = 1024.0f;
    const float k = 256.0f;
    const float r = 0.0625f;
    const double cur_d = motor_current(double{d}, double{v}, double{w}, double{k}, double{r});
    const float cur_f = motor_current(d, v, w, k, r);
    EXPECT_NEAR(cur_f, cur_d, 4.0 * std::numeric_limits<float>::epsilon() * std::fabs(cur_d));
}

TEST(Motor, DegenerateInputsReturnDefinedFiniteValues) {
    uint32_t flags = 0;
    EXPECT_EQ(motor_kv_si(kNaN), 0.0);
    EXPECT_EQ(motor_kv_si(-1.0), 0.0);
    EXPECT_EQ(motor_kv_si(kInff), 0.0f);

    EXPECT_EQ(motor_effective_resistance(0.0, 7.0, 1e-5, 100.0), 0.0);
    EXPECT_EQ(motor_effective_resistance(kNaN, 7.0, 1e-5, 100.0), 0.0);
    EXPECT_EQ(motor_effective_resistance(kR, 7.0, 1e-5, kNaN), kR) << "a non-finite reactance input drops L";
    EXPECT_EQ(motor_effective_resistance(0.075f, 7.0f, 1.0f, 1e30f), std::numeric_limits<float>::max())
        << "an overflowing reactance is the largest finite resistance";

    EXPECT_EQ(motor_current(1.0, 16.0, 0.0, 0.0, kR), 0.0) << "Kv = 0";
    EXPECT_EQ(motor_current(1.0, 16.0, 0.0, kv(), 0.0), 0.0) << "R = 0";
    EXPECT_EQ(motor_current(kNaN, 16.0, 0.0, kv(), kR), 0.0);
    EXPECT_EQ(motor_current(1.0f, kInff, 0.0f, 256.0f, 0.075f), 0.0f);

    EXPECT_EQ(motor_torque(10.0, kI0, 0.0, 100.0), 0.0);
    EXPECT_EQ(motor_torque(kNaN, kI0, kv(), 100.0), 0.0);

    EXPECT_EQ(motor_time_constant(0.0, kR, kv()), 0.0);
    EXPECT_EQ(motor_time_constant(2.5e-5, -1.0, kv()), 0.0);
    EXPECT_EQ(motor_time_constant(2.5e-5f, kNaNf, 256.0f), 0.0f);

    EXPECT_EQ(motor_alpha(0.0f, 0.1f), 0.0f) << "no time passes";
    EXPECT_EQ(motor_alpha(0.001f, 0.0f), 1.0f) << "no time constant: instant tracking";
    EXPECT_EQ(motor_alpha(kNaNf, 0.1f), 0.0f);

    EXPECT_EQ(motor_speed_target(1.0, 16.0, 0.0, kR, kI0, 0.1), 0.0);
    EXPECT_EQ(motor_speed_target(1.0, kInf, kv(), kR, kI0, 0.1), 0.0);

    EXPECT_EQ(motor_speed_step(kNaN, 100.0, 0.5, flags), 50.0) << "a non-finite speed reads as 0";
    EXPECT_EQ(motor_speed_step(100.0, kNaN, 0.5, flags), 100.0) << "a non-finite target leaves the speed";
    EXPECT_EQ(motor_speed_step_limited(100.0, 1.0, 0.0, 0.0, 0.001, flags), 100.0) << "J_r = 0";
    EXPECT_EQ(motor_speed_step_limited(100.0, 1.0, 0.0, 2.5e-5, -0.001, flags), 100.0) << "h < 0";
    EXPECT_EQ(motor_clamp_current(kNaN, 0.0, 50.0, flags), 0.0);
}
