#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "vehicles/battery.hpp"
#include "vehicles/propulsion_flags.hpp"

// ===========================================================================
// The drone builder's battery and shared bus (vehicles/battery.hpp). Every
// expectation is the closed form, computed here in double (TD-4).
// ===========================================================================

using namespace spade::vehicles;

namespace {

namespace pf = spade::vehicles::propulsion_flags;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr float kNaNf = std::numeric_limits<float>::quiet_NaN();

// The bus voltage with every motor free, from section 4's closed form.
[[nodiscard]] double free_bus_voltage(double v_source, double r0, const std::vector<BusMotorD>& motors) {
    double a = 0.0;
    double b = 0.0;
    for (const BusMotorD& m : motors) {
        a += m.duty * m.duty / m.r_eff;
        b += m.duty * m.back_emf / m.r_eff;
    }
    return (v_source + r0 * b) / (1.0 + r0 * a);
}

}  // namespace

TEST(Battery, PackResistanceIsSeriesOverParallel) {
    EXPECT_NEAR(battery_pack_resistance(4u, 1u, 0.005), 0.02, 1e-15);
    EXPECT_NEAR(battery_pack_resistance(6u, 2u, 0.004), 0.012, 1e-15);
    EXPECT_EQ(battery_pack_resistance(0u, 1u, 0.005), 0.0);
    EXPECT_EQ(battery_pack_resistance(4u, 0u, 0.005), 0.0);
    EXPECT_EQ(battery_pack_resistance(4u, 1u, kNaN), 0.0);
}

TEST(Battery, OcvInterpolatesTheUniformTable) {
    const std::vector<float> table = {3.0f, 3.5f, 3.7f, 4.2f};  // SoC 0, 1/3, 2/3, 1
    EXPECT_NEAR(battery_ocv(0.5, table), 3.5 + (3.7 - 3.5) * 0.5, 1e-6);
    EXPECT_EQ(battery_ocv(0.0, table), double{3.0f});
    EXPECT_EQ(battery_ocv(1.0, table), double{4.2f});
    EXPECT_EQ(battery_ocv(-1.0, table), double{3.0f}) << "SoC is held to [0, 1]";
    EXPECT_EQ(battery_ocv(2.0, table), double{4.2f});
    EXPECT_EQ(battery_ocv(kNaN, table), double{3.0f}) << "a non-finite SoC reads as 0";
    EXPECT_EQ(battery_ocv(0.5, std::vector<float>{}), 0.0);
    EXPECT_EQ(battery_ocv(0.5f, std::vector<float>{3.9f}), 3.9f);
}

TEST(Battery, ResampleIsLinearInterpolationInDoubleRoundedOnce) {
    const std::array<double, 2> pts[] = {{0.0, 3.0}, {0.1, 3.6}, {0.5, 3.8}, {1.0, 4.2}};
    const auto table = battery_resample_ocv(pts, 11u);
    ASSERT_TRUE(table.has_value()) << table.error().context;
    ASSERT_EQ(table->size(), 11u);
    for (uint32_t g = 0; g < 11u; ++g) {
        const double s = g / 10.0;
        double v = 0.0;
        // v0 + (v1 - v0) (s - s0) / (s1 - s0) on the segment holding s.
        if (s <= 0.1) {
            v = 3.0 + (3.6 - 3.0) * (s - 0.0) / (0.1 - 0.0);
        } else if (s <= 0.5) {
            v = 3.6 + (3.8 - 3.6) * (s - 0.1) / (0.5 - 0.1);
        } else {
            v = 3.8 + (4.2 - 3.8) * (s - 0.5) / (1.0 - 0.5);
        }
        EXPECT_EQ((*table)[g], static_cast<float>(v)) << "grid point " << g;
    }
}

TEST(Battery, ResampleHoldsTheEndsOutsideTheAuthoredRange) {
    const std::array<double, 2> pts[] = {{0.2, 3.5}, {0.8, 4.0}};
    const auto table = battery_resample_ocv(pts, 6u);
    ASSERT_TRUE(table.has_value());
    EXPECT_EQ((*table)[0], 3.5f);
    EXPECT_EQ((*table)[5], 4.0f);
}

TEST(Battery, ResampleRefusesBadPoints) {
    const std::array<double, 2> good[] = {{0.0, 3.0}, {1.0, 4.2}};
    EXPECT_FALSE(battery_resample_ocv(good, 1u).has_value()) << "count < 2";
    const std::array<double, 2> one[] = {{0.0, 3.0}};
    EXPECT_FALSE(battery_resample_ocv(one, 5u).has_value());
    const std::array<double, 2> flat[] = {{0.0, 3.0}, {0.0, 4.2}};
    EXPECT_FALSE(battery_resample_ocv(flat, 5u).has_value()) << "SoC must strictly increase";
    const std::array<double, 2> wide[] = {{0.0, 3.0}, {1.5, 4.2}};
    EXPECT_FALSE(battery_resample_ocv(wide, 5u).has_value()) << "SoC outside [0, 1]";
    const std::array<double, 2> dead[] = {{0.0, 0.0}, {1.0, 4.2}};
    EXPECT_FALSE(battery_resample_ocv(dead, 5u).has_value()) << "a voltage that is not positive";
    const std::array<double, 2> nan[] = {{0.0, kNaN}, {1.0, 4.2}};
    EXPECT_FALSE(battery_resample_ocv(nan, 5u).has_value());
}

TEST(Battery, RcBetaIsExpOfMinusHOverTau) {
    const float beta = battery_rc_beta(0.002f, 0.01f, 5.0f);
    const double ref = std::exp(-static_cast<double>(0.002f) / (static_cast<double>(0.01f) * 5.0));
    EXPECT_NEAR(beta, ref, 2.0 * std::numeric_limits<float>::epsilon());
    EXPECT_EQ(battery_rc_beta(0.0f, 0.01f, 5.0f), 1.0f) << "no time passes";
    EXPECT_EQ(battery_rc_beta(0.002f, 0.0f, 5.0f), 0.0f) << "no RC branch: no memory";
    EXPECT_EQ(battery_rc_beta(0.002f, kNaNf, 5.0f), 0.0f);
}

TEST(Battery, PolarizationSettlesAtR1TimesTheCurrent) {
    const double r1 = 0.01;
    const double i = 40.0;
    const double beta = 0.9;
    EXPECT_NEAR(battery_polarization_step(0.1, i, r1, beta), 0.1 * beta + r1 * i * (1.0 - beta), 1e-15);
    double v1 = 0.0;
    for (int k = 0; k < 2000; ++k) v1 = battery_polarization_step(v1, i, r1, beta);
    EXPECT_NEAR(v1, r1 * i, 1e-12);
    EXPECT_EQ(battery_polarization_step(kNaN, i, r1, beta), 0.0);
}

TEST(Battery, StateOfChargeCountsChargeAndHoldsAtEmpty) {
    uint32_t flags = 0;
    EXPECT_NEAR(battery_soc_step(0.5, 10.0, 1.0, 1.0, flags), 0.5 - 10.0 / 3600.0, 1e-15);
    EXPECT_EQ(flags, 0u);
    EXPECT_EQ(battery_soc_step(0.001, 100.0, 1.0, 1.0, flags), 0.0);
    EXPECT_NE(flags & pf::battery_empty, 0u);
    flags = 0;
    EXPECT_EQ(battery_soc_step(0.9999, -100.0, 1.0, 1.0, flags), 1.0) << "regeneration holds at full";
    EXPECT_EQ(flags, 0u);
    EXPECT_EQ(battery_soc_step(0.5, 10.0, 1.0, 0.0, flags), 0.5) << "no capacity: SoC does not move";
    EXPECT_EQ(battery_soc_step(kNaN, 10.0, 1.0, 1.0, flags), 0.0) << "a non-finite SoC reads as 0";
}

TEST(Battery, TerminalVoltageIsSourceLessBothDrops) {
    EXPECT_NEAR(battery_terminal_voltage(16.4, 0.3, 0.02, 60.0), 16.4 - 0.3 - 0.02 * 60.0, 1e-12);
    EXPECT_EQ(battery_terminal_voltage(kNaN, 0.3, 0.02, 60.0), 0.0);
}

TEST(BusSolve, FreeMotorsMatchTheClosedForm) {
    const std::vector<BusMotorD> motors = {{0.6, 7.8, 0.075, 0.0, 50.0}, {0.4, 6.1, 0.08, 0.0, 50.0}};
    std::vector<double> currents(2);
    const BusResultD r = bus_solve(16.4, 0.02, 0.0, 0.0, motors, currents);
    const double v = free_bus_voltage(16.4, 0.02, motors);
    EXPECT_NEAR(r.bus_voltage, v, 1e-12);
    double ib = 0.0;
    for (std::size_t k = 0; k < motors.size(); ++k) {
        const double i = (motors[k].duty * v - motors[k].back_emf) / motors[k].r_eff;
        EXPECT_NEAR(currents[k], i, 1e-9) << "motor " << k;
        ib += motors[k].duty * i;
    }
    EXPECT_NEAR(r.battery_current, ib, 1e-9);
    EXPECT_EQ(r.duty_scale, 1.0);
    EXPECT_EQ(r.flags, 0u);
}

TEST(BusSolve, AMotorAtItsLimitMovesToTheFixedSum) {
    // Motor 0 wants about 27 A against a 20 A limit.
    const std::vector<BusMotorD> motors = {{0.6, 7.8, 0.075, 0.0, 20.0}, {0.4, 6.1, 0.08, 0.0, 50.0}};
    std::vector<double> currents(2);
    const BusResultD r = bus_solve(16.4, 0.02, 0.0, 0.0, motors, currents);
    EXPECT_EQ(currents[0], 20.0);
    EXPECT_NE(r.flags & pf::current_limited, 0u);
    // V from the fixed-sum form: motor 1 free, motor 0 held at 20 A.
    const BusMotorD& f = motors[1];
    const double v = (16.4 + 0.02 * f.duty * f.back_emf / f.r_eff - 0.02 * 0.6 * 20.0) /
                     (1.0 + 0.02 * f.duty * f.duty / f.r_eff);
    EXPECT_NEAR(r.bus_voltage, v, 1e-12);
    EXPECT_NEAR(currents[1], (f.duty * v - f.back_emf) / f.r_eff, 1e-9);
}

TEST(BusSolve, ThePackCurrentLimitScalesEveryDutyByOneFactor) {
    const std::vector<BusMotorD> motors(4, BusMotorD{0.9, 2.0, 0.075, 0.0, 200.0});
    std::vector<double> currents(4);
    const double limit = 60.0;
    const BusResultD r = bus_solve(16.4, 0.02, limit, 0.0, motors, currents);
    EXPECT_NE(r.flags & pf::battery_current_limited, 0u);
    EXPECT_LT(r.duty_scale, 1.0);
    EXPECT_LE(r.battery_current, limit);
    EXPECT_GT(r.battery_current, limit * (1.0 - 1e-5)) << "24 bisection steps land just under the limit";
    for (std::size_t k = 1; k < 4; ++k) EXPECT_EQ(currents[k], currents[0]) << "one common scale";
}

TEST(BusSolve, CutoffIsHeldByTheSameScale) {
    const std::vector<BusMotorD> motors(4, BusMotorD{1.0, 2.0, 0.075, 0.0, 200.0});
    std::vector<double> currents(4);
    const double cutoff = 14.0;
    const BusResultD r = bus_solve(16.4, 0.05, 0.0, cutoff, motors, currents);
    EXPECT_NE(r.flags & pf::battery_cutoff, 0u);
    EXPECT_GE(r.bus_voltage, cutoff);
    EXPECT_LT(r.bus_voltage, cutoff + 1e-4);
}

TEST(BusSolve, AFlatPackIsAtCutoffWithNoCurrent) {
    const std::vector<BusMotorD> motors(2, BusMotorD{0.5, 0.0, 0.075, 0.0, 50.0});
    std::vector<double> currents(2);
    const BusResultD r = bus_solve(12.0, 0.02, 0.0, 13.2, motors, currents);
    EXPECT_EQ(r.duty_scale, 0.0);
    EXPECT_EQ(r.battery_current, 0.0);
    EXPECT_NE(r.flags & pf::battery_cutoff, 0u);
}

TEST(BusSolve, FloatAndDoubleAgreeToFloatRounding) {
    const std::vector<BusMotor> mf(4, BusMotor{0.5f, 6.0f, 0.0625f, 0.0f, 50.0f});
    const std::vector<BusMotorD> md(4, BusMotorD{0.5, 6.0, 0.0625, 0.0, 50.0});
    std::vector<float> cf(4);
    std::vector<double> cd(4);
    const BusResult rf = bus_solve(16.0f, 0.0625f, 0.0f, 0.0f, mf, cf);
    const BusResultD rd = bus_solve(16.0, 0.0625, 0.0, 0.0, md, cd);
    ASSERT_GT(rd.bus_voltage, 1.0) << "non-vacuity: the closed form gives 14 V, not 0";
    EXPECT_NEAR(rf.bus_voltage, rd.bus_voltage, 8.0 * std::numeric_limits<float>::epsilon() * rd.bus_voltage);
    EXPECT_NEAR(cf[0], cd[0], 1e-4 * std::fabs(cd[0]));
}

// DBP-26: a 4-in-1 ESC's total motor-side current, held through the same
// common duty scale as the pack's limits.
TEST(BusSolve, TheEscTotalLimitScalesEveryDutyByOneFactor) {
    const std::vector<BusMotorD> motors(4, BusMotorD{0.9, 2.0, 0.075, 0.0, 200.0});
    std::vector<double> currents(4);
    const double total = 120.0;
    const BusResultD r = bus_solve(16.4, 0.02, 0.0, 0.0, total, motors, currents);
    EXPECT_NE(r.flags & pf::esc_total_limited, 0u);
    EXPECT_EQ(r.flags & pf::battery_current_limited, 0u) << "no pack limit was given";
    EXPECT_LT(r.duty_scale, 1.0);
    double sum = 0.0;
    for (const double i : currents) sum += std::fabs(i);
    EXPECT_LE(sum, total);
    EXPECT_GT(sum, total * (1.0 - 1e-5)) << "24 bisection steps land just under the limit";
    for (std::size_t k = 1; k < 4; ++k) EXPECT_EQ(currents[k], currents[0]) << "one common scale";
}

TEST(BusSolve, TheSixArgumentFormIsTheSevenWithNoEscLimit) {
    const std::vector<BusMotorD> motors(4, BusMotorD{0.9, 2.0, 0.075, 0.0, 200.0});
    std::vector<double> a(4);
    std::vector<double> b(4);
    const BusResultD six = bus_solve(16.4, 0.02, 60.0, 0.0, motors, a);
    const BusResultD seven = bus_solve(16.4, 0.02, 60.0, 0.0, 0.0, motors, b);
    EXPECT_EQ(six.bus_voltage, seven.bus_voltage);
    EXPECT_EQ(six.duty_scale, seven.duty_scale);
    EXPECT_EQ(six.flags, seven.flags);
    EXPECT_EQ(a, b);
}

TEST(BusSolve, DegenerateInputsDrawNothing) {
    std::vector<double> currents(2, 99.0);
    const std::vector<BusMotorD> motors(2, BusMotorD{0.5, 6.0, 0.075, 0.0, 50.0});
    BusResultD r = bus_solve(kNaN, 0.02, 0.0, 0.0, motors, currents);
    EXPECT_EQ(r.battery_current, 0.0);
    EXPECT_EQ(currents[0], 0.0);

    std::vector<double> too_short(1);
    r = bus_solve(16.0, 0.02, 0.0, 0.0, motors, too_short);
    EXPECT_EQ(r.battery_current, 0.0);

    const std::vector<BusMotorD> many(kMaxBusMotors + 1, BusMotorD{0.5, 6.0, 0.075, 0.0, 50.0});
    std::vector<double> many_currents(kMaxBusMotors + 1);
    r = bus_solve(16.0, 0.02, 0.0, 0.0, many, many_currents);
    EXPECT_EQ(r.battery_current, 0.0) << "more than kMaxBusMotors is refused";

    const std::vector<BusMotorD> dead = {{0.5, 6.0, 0.0, 0.0, 50.0}, {0.5, 6.0, 0.075, 0.0, 50.0}};
    std::vector<double> dead_currents(2);
    r = bus_solve(16.0, 0.02, 0.0, 0.0, dead, dead_currents);
    EXPECT_EQ(dead_currents[0], 0.0) << "R_eff = 0 draws nothing";
    EXPECT_GT(dead_currents[1], 0.0);
}
