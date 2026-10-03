#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <span>

#include "vehicles/battery.hpp"
#include "vehicles/motor.hpp"
#include "vehicles/propeller.hpp"
#include "vehicles/propulsion_flags.hpp"
#include "vehicles/propulsion_steady.hpp"

// ===========================================================================
// The drone builder's steady-state solver (vehicles/propulsion_steady.hpp),
// and DBP-52: the stepped chain settles to the solver's answer.
// ===========================================================================

using namespace spade::vehicles;

namespace {

namespace pf = spade::vehicles::propulsion_flags;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRho = 1.225;
constexpr double kSoc = 0.8;  // an exact point of the 21-point OCV grid (index 16)

// A 5-inch-class quad's chain: 2207 2450 KV, 70 + 5 mOhm, 6-inch prop, 4S1P.
[[nodiscard]] PropulsionChain make_chain(double inductance = 0.0) {
    PropulsionChain c;
    c.kv = motor_kv_si(2450.0);
    c.resistance = 0.075;
    c.no_load_current = 1.2;
    c.current_min = 0.0;
    c.current_max = 50.0;
    c.pole_pairs = 7.0;
    c.inductance = inductance;
    c.diameter = 0.1524;
    const std::array<double, 2> ct[] = {{0.0, 0.12}, {0.8, 0.04}};
    const std::array<double, 2> cq[] = {{0.0, 0.008}, {0.8, 0.004}};
    c.table = *propeller_resample_table(ct, cq, 17u);
    c.cells_series = 4;
    c.cells_parallel = 1;
    c.cell_resistance = 0.005;
    c.cell_cutoff_voltage = 3.0;
    c.pack_polarization_resistance = 0.0;
    c.pack_current_max = 120.0;
    const std::array<double, 2> ocv[] = {{0.0, 3.3}, {0.2, 3.6}, {0.5, 3.75}, {0.8, 3.95}, {1.0, 4.2}};
    c.ocv_table = *battery_resample_ocv(ocv, 21u);
    c.motors_on_bus = 4;
    return c;
}

}  // namespace

TEST(PropulsionSteady, ForwardModeBalancesMotorAndPropellerTorque) {
    const PropulsionChain c = make_chain();
    const double d = 0.6;
    const SteadyPoint p = steady_state_at_duty(c, d, kRho, 0.0, kSoc);
    ASSERT_EQ(p.flags, 0u) << "no limit is reached at duty 0.6";

    // The balance, from the closed forms: 4 identical free motors on one bus.
    const double vs = 4.0 * double{c.ocv_table[16]};
    const double r0 = 4.0 * 0.005;
    const double e = p.omega / c.kv;
    const double v = (vs + r0 * 4.0 * d * e / c.resistance) / (1.0 + r0 * 4.0 * d * d / c.resistance);
    const double i = (d * v - e) / c.resistance;
    const double q_motor = (i - c.no_load_current) / c.kv;
    const double n = p.omega / (2.0 * kPi);
    const double q_prop = double{c.table.cq[0]} * kRho * n * n * std::pow(c.diameter, 5.0);
    EXPECT_NEAR(q_motor, q_prop, 1e-9 * q_prop);
    EXPECT_NEAR(p.thrust, double{c.table.ct[0]} * kRho * n * n * std::pow(c.diameter, 4.0), 1e-9 * p.thrust);
    EXPECT_NEAR(p.bus_voltage, v, 1e-9 * v);
    EXPECT_NEAR(p.current, i, 1e-9 * i);
    EXPECT_GT(p.efficiency, 0.0);
    EXPECT_LT(p.efficiency, 1.0);
}

TEST(PropulsionSteady, InverseModeFindsTheHoverDuty) {
    const PropulsionChain c = make_chain();
    const double per_motor = 0.7 * 9.80665 / 4.0;
    const SteadyPoint p = steady_state_for_thrust(c, per_motor, kRho, 0.0, kSoc);
    EXPECT_EQ(p.flags & pf::unreachable, 0u);
    EXPECT_NEAR(p.thrust, per_motor, 1e-9 * per_motor);
    EXPECT_GT(p.duty, 0.2);
    EXPECT_LT(p.duty, 0.35) << "a 0.7 kg quad on this chain hovers near a quarter throttle";
}

TEST(PropulsionSteady, ATargetAboveFullThrottleIsUnreachable) {
    const SteadyPoint p = steady_state_for_thrust(make_chain(), 100.0, kRho, 0.0, kSoc);
    EXPECT_NE(p.flags & pf::unreachable, 0u);
    EXPECT_EQ(p.duty, 1.0);
}

TEST(PropulsionSteady, AFallingLoadTableHasNoBracket) {
    PropulsionChain c = make_chain();
    const std::array<double, 2> ct[] = {{0.0, 0.12}, {0.8, 0.04}};
    const std::array<double, 2> cq[] = {{0.0, -0.01}, {0.8, -0.01}};
    c.table = *propeller_resample_table(ct, cq, 17u);
    const SteadyPoint p = steady_state_at_duty(c, 0.6, kRho, 0.0, kSoc);
    EXPECT_NE(p.flags & pf::no_bracket, 0u);
}

TEST(PropulsionSteady, AnInvalidChainReturnsAZeroPointFlagged) {
    const SteadyPoint p = steady_state_at_duty(PropulsionChain{}, 0.6, kRho, 0.0, kSoc);
    EXPECT_EQ(p.omega, 0.0);
    EXPECT_EQ(p.thrust, 0.0);
    EXPECT_NE(p.flags & pf::no_bracket, 0u);
    const SteadyPoint q = steady_state_at_duty(make_chain(), 0.6, std::nan(""), 0.0, kSoc);
    EXPECT_NE(q.flags & pf::no_bracket, 0u);
}

TEST(PropulsionSteady, InductanceDroopGrowsWithSpeed) {
    // Measured in a Python port before this test was written: 7.4% at duty
    // 0.2 and 28% at 0.6 for 15 uH and 7 pole pairs.
    const PropulsionChain plain = make_chain();
    const PropulsionChain inductive = make_chain(15e-6);
    const auto drop = [&](double d) {
        const double w0 = steady_state_at_duty(plain, d, kRho, 0.0, kSoc).omega;
        const double wl = steady_state_at_duty(inductive, d, kRho, 0.0, kSoc).omega;
        return (w0 - wl) / w0;
    };
    const double low = drop(0.2);
    const double high = drop(0.6);
    EXPECT_GT(low, 0.0);
    EXPECT_GT(high, low) << "R_eff grows with the square of speed";
}

// ===========================================================================
// DBP-52. The float step functions, driven at constant duty, settle to the
// double solver's speed. The exponential step's fixed point IS the torque
// balance, so after the transient the gap is fp32 rounding alone.
//
// THE BOUND. fp32 carries 1.2e-7 relative. The balance is reached through
// about 25 float operations, with cancellation of up to about 6 in
// (d V - e), so the gap can reach a few hundred epsilons: 5e-5 is about 400.
// A float32 Python port of this loop measured 3.6e-6 (L = 0) and 7.8e-6
// (L = 15 uH), the same at 3 s and 10 s, so the transient is gone well before
// the 10 s run here (the inductive chain's effective time constant is about
// 0.25 s).
// ===========================================================================
TEST(PropulsionSteady, TheSteppedChainSettlesToTheSolver) {
    for (const double inductance : {0.0, 15e-6}) {
        const PropulsionChain c = make_chain(inductance);
        const double d = 0.6;
        const SteadyPoint target = steady_state_at_duty(c, d, kRho, 0.0, kSoc);
        ASSERT_EQ(target.flags, 0u);

        const float kv = static_cast<float>(c.kv);
        const float r = static_cast<float>(c.resistance);
        const float poles = static_cast<float>(c.pole_pairs);
        const float l = static_cast<float>(c.inductance);
        const float i0 = static_cast<float>(c.no_load_current);
        const float dia = static_cast<float>(c.diameter);
        const float rho = static_cast<float>(kRho);
        const float duty = static_cast<float>(d);
        const float inertia = 2.5e-5f;
        const float h = 0.001f;
        const float v_source = 4.0f * battery_ocv(static_cast<float>(kSoc), std::span<const float>(c.ocv_table));
        const float r0 = battery_pack_resistance(4u, 1u, 0.005f);

        float omega = 0.0f;
        for (int step = 0; step < 10000; ++step) {
            uint32_t flags = 0;
            const float r_eff = motor_effective_resistance(r, poles, l, omega);
            const float back_emf = omega / kv;
            std::array<BusMotor, 4> motors;
            motors.fill(BusMotor{duty, back_emf, r_eff, 0.0f, 50.0f});
            std::array<float, 4> currents{};
            const BusResult bus = bus_solve(v_source, r0, 120.0f, 12.0f, motors, currents);
            const float j = propeller_advance_ratio(0.0f, omega, dia);
            const float cq = propeller_coefficient(c.table.cq, c.table.j_min, c.table.j_max, j, flags);
            const float load = propeller_torque(cq, rho, omega, dia);
            if ((bus.flags & pf::current_limited) != 0u) {
                const float drive = motor_torque(currents[0], i0, kv, omega);
                omega = motor_speed_step_limited(omega, drive, load, inertia, h, flags);
            } else {
                const float w_inf = motor_speed_target(duty * bus.duty_scale, bus.bus_voltage, kv, r_eff, i0, load);
                const float alpha = motor_alpha(h, motor_time_constant(inertia, r_eff, kv));
                omega = motor_speed_step(omega, w_inf, alpha, flags);
            }
        }
        EXPECT_NEAR(omega, target.omega, 5e-5 * target.omega) << "L = " << inductance;
    }
}
