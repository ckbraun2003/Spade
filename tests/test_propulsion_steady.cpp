#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "core/error.hpp"
#include "core/rng.hpp"
#include "vehicles/battery.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/motor.hpp"
#include "vehicles/propeller.hpp"
#include "vehicles/propulsion_flags.hpp"
#include "vehicles/propulsion_steady.hpp"

// ===========================================================================
// The drone builder's steady-state solver (vehicles/propulsion_steady.hpp),
// and DBP-52: the stepped chain settles to the solver's answer. Then the
// throttle map (rotor_omega_for_duty), the arm rule (armed_duty) and the
// chain's own identity (propulsion_identity, vehicles/model_identity.hpp).
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

// The small-signal time constant, against the closed form. With no pack
// resistance the bus holds V = V_source, so d(balance)/d(omega) is
// -1/(Kv^2 R) - 2 Q_load/omega at J = 0, and tau = J_r over its negation.
TEST(PropulsionSteady, TimeConstantMatchesTheClosedForm) {
    PropulsionChain c = make_chain();
    c.cell_resistance = 0.0;
    const double inertia = 2.5e-5;
    const double d = 0.6;
    const SteadyPoint p = steady_state_at_duty(c, d, kRho, 0.0, kSoc);
    ASSERT_EQ(p.flags, 0u);
    const double n = p.omega / (2.0 * kPi);
    const double q_load = double{c.table.cq[0]} * kRho * n * n * std::pow(c.diameter, 5.0);
    const double slope = 1.0 / (c.kv * c.kv * c.resistance) + 2.0 * q_load / p.omega;
    EXPECT_NEAR(steady_state_time_constant(c, d, kRho, 0.0, kSoc, inertia), inertia / slope, 1e-6 * inertia / slope);
    EXPECT_EQ(steady_state_time_constant(c, d, kRho, 0.0, kSoc, 0.0), 0.0) << "no inertia, no time constant";
    EXPECT_EQ(steady_state_time_constant(c, 0.0, kRho, 0.0, kSoc, inertia), 0.0) << "at rest";
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
        ASSERT_GT(target.omega, 100.0) << "non-vacuity: the motor must be turning, or 0 = 0 would pass";

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

// ===========================================================================
// THE THROTTLE MAP (rotor_omega_for_duty): a duty to the rotor row's
// omega_cmd, the equilibrium at zero inflow.
// ===========================================================================

namespace {

[[nodiscard]] bool names(const spade::Error& e, std::string_view what) {
    return e.code == spade::Code::invalid_argument && e.context.find(what) != std::string::npos;
}

// The pack's open-circuit voltage at kSoc, from the closed form: 4 cells at
// the OCV grid's point for 0.8.
[[nodiscard]] double source_voltage(const PropulsionChain& c) { return 4.0 * double{c.ocv_table[16]}; }

}  // namespace

TEST(ThrottleMap, IsTheEquilibriumAtZeroInflow) {
    const PropulsionChain c = make_chain();
    float last = 0.0f;
    for (const double d : {0.2, 0.6, 0.9}) {
        const SteadyPoint p = steady_state_at_duty(c, d, kRho, 0.0, kSoc);
        const auto r = rotor_omega_for_duty(c, d, kRho, kSoc);
        ASSERT_TRUE(r.has_value()) << r.error().context;
        EXPECT_EQ(r->omega_cmd, static_cast<float>(p.omega)) << "rounded once from double, at duty " << d;
        EXPECT_EQ(r->flags, p.flags) << "duty " << d;
        EXPECT_GT(r->omega_cmd, last) << "a higher duty spins faster";
        last = r->omega_cmd;
    }
    EXPECT_GT(last, 1000.0f) << "non-vacuity: the motor must be turning";
}

TEST(ThrottleMap, AtRestIsZeroNotAnError) {
    const PropulsionChain c = make_chain();
    // Duty 0: the bracket collapses.
    const auto zero = rotor_omega_for_duty(c, 0.0, kRho, kSoc);
    ASSERT_TRUE(zero.has_value()) << zero.error().context;
    EXPECT_EQ(zero->omega_cmd, 0.0f);
    EXPECT_EQ(zero->flags, 0u) << "at rest is not no_bracket";

    // Below breakaway: at rest the drive current d V / R (V at most the
    // source voltage) cannot beat the no-load current I0, so the balance at
    // omega = 0 is not positive. The breakaway duty is I0 R / V_source.
    const double breakaway = c.no_load_current * c.resistance / source_voltage(c);
    const double below = 0.9 * breakaway;
    ASSERT_GT(below, 0.0) << "a real duty, not the collapsed bracket";
    const auto still = rotor_omega_for_duty(c, below, kRho, kSoc);
    ASSERT_TRUE(still.has_value()) << still.error().context;
    EXPECT_EQ(still->omega_cmd, 0.0f);
    EXPECT_EQ(still->flags & pf::no_bracket, 0u) << "below breakaway is not no_bracket";

    // Non-vacuity: just above breakaway the motor turns.
    const auto above = rotor_omega_for_duty(c, 1.1 * breakaway, kRho, kSoc);
    ASSERT_TRUE(above.has_value()) << above.error().context;
    EXPECT_GT(above->omega_cmd, 0.0f) << "the breakaway estimate is where the motor starts";
}

TEST(ThrottleMap, LimitsAreOperatingPointsItAnnounces) {
    // At duty 0.6 each motor draws about 22 A and the pack about 53 A (the
    // forward-mode closed form). A 10 A motor rating and a 30 A pack rating
    // are both reached; the chain settles there, so each is an answer with
    // its flag, not a refusal.
    PropulsionChain motor_limited = make_chain();
    motor_limited.current_max = 10.0;
    PropulsionChain pack_limited = make_chain();
    pack_limited.pack_current_max = 30.0;
    const std::pair<const PropulsionChain*, uint32_t> cases[] = {{&motor_limited, pf::current_limited},
                                                                  {&pack_limited, pf::battery_current_limited}};
    for (const auto& [c, flag] : cases) {
        const SteadyPoint p = steady_state_at_duty(*c, 0.6, kRho, 0.0, kSoc);
        ASSERT_NE(p.flags & flag, 0u) << "the fixture must reach the limit: flag " << flag;
        const auto r = rotor_omega_for_duty(*c, 0.6, kRho, kSoc);
        ASSERT_TRUE(r.has_value()) << r.error().context;
        EXPECT_NE(r->flags & flag, 0u) << "announced: flag " << flag;
        EXPECT_EQ(r->omega_cmd, static_cast<float>(p.omega)) << "flag " << flag;
        EXPECT_GT(r->omega_cmd, 100.0f) << "flag " << flag;
    }
}

TEST(ThrottleMap, RefusesBadInputsByNameBeforeSolving) {
    const PropulsionChain c = make_chain();
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const double d : {nan, inf, -inf, -0.001, 1.001, std::nextafter(0.0, -1.0), std::nextafter(1.0, 2.0)}) {
        const auto r = rotor_omega_for_duty(c, d, kRho, kSoc);
        ASSERT_FALSE(r.has_value()) << "duty " << d << " is refused, not clamped";
        EXPECT_TRUE(names(r.error(), "duty must")) << r.error().context;
    }
    for (const double rho : {0.0, -0.0, -kRho, nan, inf}) {
        const auto r = rotor_omega_for_duty(c, 0.6, rho, kSoc);
        ASSERT_FALSE(r.has_value()) << "density " << rho;
        EXPECT_TRUE(names(r.error(), "density must")) << r.error().context;
    }
    for (const double soc : {-0.01, 1.01, nan, inf, -inf}) {
        const auto r = rotor_omega_for_duty(c, 0.6, kRho, soc);
        ASSERT_FALSE(r.has_value()) << "soc " << soc;
        EXPECT_TRUE(names(r.error(), "soc must")) << r.error().context;
    }
    // The ends are in range.
    for (const double d : {0.0, -0.0, 1.0}) {
        const auto r = rotor_omega_for_duty(c, d, kRho, kSoc);
        EXPECT_TRUE(r.has_value()) << "duty " << d << ": " << r.error().context;
    }
    for (const double soc : {0.0, 1.0}) {
        const auto r = rotor_omega_for_duty(c, 0.6, kRho, soc);
        EXPECT_TRUE(r.has_value()) << "soc " << soc << ": " << r.error().context;
    }
    // In order, and before the solve: an invalid chain with a bad duty is
    // refused for the duty, a bad density beside a bad soc for the density.
    const auto first = rotor_omega_for_duty(PropulsionChain{}, 2.0, nan, nan);
    ASSERT_FALSE(first.has_value());
    EXPECT_TRUE(names(first.error(), "duty must")) << first.error().context;
    const auto second = rotor_omega_for_duty(PropulsionChain{}, 0.5, nan, nan);
    ASSERT_FALSE(second.has_value());
    EXPECT_TRUE(names(second.error(), "density must")) << second.error().context;
    const auto third = rotor_omega_for_duty(PropulsionChain{}, 0.5, kRho, nan);
    ASSERT_FALSE(third.has_value());
    EXPECT_TRUE(names(third.error(), "soc must")) << third.error().context;
}

TEST(ThrottleMap, RefusesAChainWithNoTorqueBalance) {
    // A load that falls with speed (AFallingLoadTableHasNoBracket's table),
    // and an invalid chain: steady_state_at_duty() flags both no_bracket.
    PropulsionChain falling = make_chain();
    const std::array<double, 2> ct[] = {{0.0, 0.12}, {0.8, 0.04}};
    const std::array<double, 2> cq[] = {{0.0, -0.01}, {0.8, -0.01}};
    falling.table = *propeller_resample_table(ct, cq, 17u);
    for (const PropulsionChain& c : {falling, PropulsionChain{}}) {
        ASSERT_NE(steady_state_at_duty(c, 0.6, kRho, 0.0, kSoc).flags & pf::no_bracket, 0u);
        const auto r = rotor_omega_for_duty(c, 0.6, kRho, kSoc);
        ASSERT_FALSE(r.has_value());
        EXPECT_TRUE(names(r.error(), "no_bracket")) << r.error().context;
    }
}

// ===========================================================================
// THE ARM RULE (armed_duty): disarmed 0; armed idle + (1 - idle) throttle.
// ===========================================================================

TEST(ArmRule, DisarmedIsDutyZeroAndOmegaCmdZero) {
    for (const double throttle : {0.0, 0.5, 1.0}) {
        for (const double idle : {0.0, 0.1}) {
            const auto d = armed_duty(false, throttle, idle);
            ASSERT_TRUE(d.has_value()) << d.error().context;
            EXPECT_EQ(*d, 0.0) << "throttle " << throttle << ", idle " << idle;
        }
    }
    // Disarm is omega_cmd 0: the row spins down on its own tau.
    const auto d = armed_duty(false, 1.0, 0.1);
    ASSERT_TRUE(d.has_value()) << d.error().context;
    const auto r = rotor_omega_for_duty(make_chain(), *d, kRho, kSoc);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(r->omega_cmd, 0.0f);
}

TEST(ArmRule, ArmedSpansIdleToFullDuty) {
    // Zero throttle is exactly idle, and full throttle exactly 1, for every
    // idle in [0, 1): idle + (1 - idle) can round only to 1 (the rounding
    // error of 1 - idle is at most a quarter of 1's ulp).
    for (const double idle : {0.0, 0.05, 0.1, 0.3, 1.0 / 3.0, 0.7, std::nextafter(1.0, 0.0)}) {
        const auto lo = armed_duty(true, 0.0, idle);
        const auto hi = armed_duty(true, 1.0, idle);
        ASSERT_TRUE(lo.has_value() && hi.has_value()) << "idle " << idle;
        EXPECT_EQ(*lo, idle);
        EXPECT_EQ(*hi, 1.0) << "idle " << idle;
    }
    // Between, the closed form, rising with throttle and inside [idle, 1].
    const auto mid = armed_duty(true, 0.5, 0.1);
    ASSERT_TRUE(mid.has_value()) << mid.error().context;
    EXPECT_DOUBLE_EQ(*mid, 0.55);
    double last = -1.0;
    for (int k = 0; k <= 64; ++k) {
        const double throttle = static_cast<double>(k) / 64.0;
        const auto d = armed_duty(true, throttle, 0.1);
        ASSERT_TRUE(d.has_value()) << d.error().context;
        EXPECT_GT(*d, last) << "throttle " << throttle;
        EXPECT_GE(*d, 0.1);
        EXPECT_LE(*d, 1.0);
        last = *d;
    }
    // With no idle, the duty is the throttle.
    const auto plain = armed_duty(true, 0.3, 0.0);
    ASSERT_TRUE(plain.has_value()) << plain.error().context;
    EXPECT_EQ(*plain, 0.3);
    // Full throttle feeds the throttle map without a refusal, and spins.
    const auto full = armed_duty(true, 1.0, 0.05);
    ASSERT_TRUE(full.has_value()) << full.error().context;
    const auto r = rotor_omega_for_duty(make_chain(), *full, kRho, kSoc);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_GT(r->omega_cmd, 1000.0f);
}

TEST(ArmRule, RefusesBadInputsByNameWhateverArmedIs) {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const bool armed : {false, true}) {
        for (const double throttle : {nan, inf, -inf, -0.01, 1.01, std::nextafter(1.0, 2.0)}) {
            const auto d = armed_duty(armed, throttle, 0.1);
            ASSERT_FALSE(d.has_value()) << "armed " << armed << ", throttle " << throttle;
            EXPECT_TRUE(names(d.error(), "throttle must")) << d.error().context;
        }
        for (const double idle : {nan, inf, -inf, -0.01, 1.0, 1.5}) {
            const auto d = armed_duty(armed, 0.5, idle);
            ASSERT_FALSE(d.has_value()) << "armed " << armed << ", idle " << idle;
            EXPECT_TRUE(names(d.error(), "idle must")) << d.error().context;
        }
        // Both bad: the throttle is named first.
        const auto both = armed_duty(armed, nan, nan);
        ASSERT_FALSE(both.has_value());
        EXPECT_TRUE(names(both.error(), "throttle must")) << both.error().context;
    }
}

// ===========================================================================
// propulsion_identity(): the chain's own identity (TD-9: the one rule), in
// vehicles/model_identity.hpp beside parts_identity().
// ===========================================================================

namespace {

// A fixed chain: every field a literal, so its identity is the same on every
// toolchain (no resample, no libm). make_chain()'s motor and pack, with
// braking off (current_min +0), a 4-in-1 ESC total, 15 uH, a 4S2P pack with
// R1, and short tables.
[[nodiscard]] PropulsionChain fixed_chain() {
    PropulsionChain c;
    c.kv = 256.56340004316644;  // 2450 rpm/V
    c.resistance = 0.075;
    c.no_load_current = 1.2;
    c.current_min = 0.0;
    c.current_max = 50.0;
    c.esc_current_total_max = 160.0;
    c.pole_pairs = 7.0;
    c.inductance = 15e-6;
    c.diameter = 0.1524;
    c.table.j_min = 0.0f;
    c.table.j_max = 0.8f;
    c.table.ct = {0.12f, 0.1f, 0.08f, 0.06f, 0.04f};
    c.table.cq = {0.008f, 0.007f, 0.006f, 0.005f, 0.004f};
    c.cells_series = 4;
    c.cells_parallel = 2;
    c.cell_resistance = 0.005;
    c.cell_cutoff_voltage = 3.0;
    c.pack_polarization_resistance = 0.002;
    c.pack_current_max = 120.0;
    c.ocv_table = {3.3f, 3.6f, 3.75f, 3.95f, 4.2f};
    c.motors_on_bus = 4;
    return c;
}

// fixed_chain()'s propulsion_identity: PINNED, and the cross-toolchain check
// (the gcc leg must reproduce it, TD-12). Taken first from an independent
// Python port of the fold, written byte by byte from model_identity.hpp's
// comment before the C++ fold existed (TD-4), and measured on MSVC
// (msvc-ninja-release) to the same value.
constexpr uint64_t kFixedPropulsionIdentity = 0x5668d33c828d7d22ull;

}  // namespace

TEST(PropulsionIdentity, ATinyChainIsTheDocumentedFold) {
    // Spelt here independently: FNV-1a 64 over the domain tag, then every
    // field in declaration order, doubles as their 64-bit patterns and
    // floats as their 32-bit ones (written as hex), counts and u32s LE.
    PropulsionChain c;  // every double +0, empty tables, motors_on_bus 1
    c.kv = 1.5;
    c.table.j_max = 0.5f;
    c.table.ct = {0.25f};
    c.table.cq = {0.125f, -0.0f};
    c.cells_series = 3;
    c.ocv_table = {4.0f};

    uint64_t h = spade::rng::kFnv1aOffsetBasis;
    const auto fold = [&h](uint8_t b) {
        h ^= b;
        h *= spade::rng::kFnv1aPrime;
    };
    const auto le = [&fold](uint64_t v, int bytes) {
        for (int i = 0; i < bytes; ++i) fold(static_cast<uint8_t>(v >> (8 * i)));
    };
    fold(0xFF);
    for (const char ch : std::string_view("propulsion_identity")) fold(static_cast<uint8_t>(ch));
    le(0x3FF8000000000000ull, 8);          // kv 1.5
    for (int k = 0; k < 8; ++k) le(0, 8);  // resistance .. diameter, +0
    le(0, 4);                              // j_min +0
    le(0x3F000000u, 4);                    // j_max 0.5
    le(1, 4);                              // ct count
    le(0x3E800000u, 4);                    //   0.25
    le(2, 4);                              // cq count
    le(0x3E000000u, 4);                    //   0.125
    le(0x80000000u, 4);                    //   -0
    le(3, 4);                              // cells_series
    le(0, 4);                              // cells_parallel
    for (int k = 0; k < 4; ++k) le(0, 8);  // cell_resistance .. pack_current_max, +0
    le(1, 4);                              // ocv count
    le(0x40800000u, 4);                    //   4.0
    le(1, 4);                              // motors_on_bus
    EXPECT_EQ(propulsion_identity(c), h) << std::hex << propulsion_identity(c) << " vs " << h;
    EXPECT_NE(h, 0u);
}

TEST(PropulsionIdentity, AFixedChainHasAPinnedValue) {
    const PropulsionChain c = fixed_chain();
    EXPECT_EQ(propulsion_identity(c), kFixedPropulsionIdentity) << std::hex << propulsion_identity(c);
    EXPECT_NE(propulsion_identity(c), propulsion_identity(PropulsionChain{})) << "not the default chain's";
}

TEST(PropulsionIdentity, EveryFieldMovesIt) {
    const std::vector<std::pair<const char*, std::function<void(PropulsionChain&)>>> changes = {
        {"kv by one ulp (a double: all 64 bits)", [](PropulsionChain& c) { c.kv = std::nextafter(c.kv, 0.0); }},
        {"resistance", [](PropulsionChain& c) { c.resistance = 0.07; }},
        {"no_load_current", [](PropulsionChain& c) { c.no_load_current = 1.1; }},
        {"current_min, -0 for +0", [](PropulsionChain& c) { c.current_min = -0.0; }},
        {"current_min, braking", [](PropulsionChain& c) { c.current_min = -50.0; }},
        {"current_max", [](PropulsionChain& c) { c.current_max = 45.0; }},
        {"esc_current_total_max", [](PropulsionChain& c) { c.esc_current_total_max = 0.0; }},
        {"pole_pairs", [](PropulsionChain& c) { c.pole_pairs = 6.0; }},
        {"inductance", [](PropulsionChain& c) { c.inductance = 16e-6; }},
        {"diameter", [](PropulsionChain& c) { c.diameter = 0.127; }},
        {"table.j_min, -0 for +0", [](PropulsionChain& c) { c.table.j_min = -0.0f; }},
        {"table.j_max", [](PropulsionChain& c) { c.table.j_max = 0.9f; }},
        {"a ct value by one ulp", [](PropulsionChain& c) { c.table.ct[2] = std::nextafter(c.table.ct[2], 1.0f); }},
        {"a cq value", [](PropulsionChain& c) { c.table.cq[4] = 0.0041f; }},
        {"ct one longer", [](PropulsionChain& c) { c.table.ct.push_back(0.02f); }},
        {"cq one shorter", [](PropulsionChain& c) { c.table.cq.pop_back(); }},
        {"ct and cq split elsewhere (the counts are folded)",
         [](PropulsionChain& c) {
             // The same value bytes in the same order; only the counts differ.
             c.table.cq.insert(c.table.cq.begin(), c.table.ct.back());
             c.table.ct.pop_back();
         }},
        {"cells_series", [](PropulsionChain& c) { c.cells_series = 6; }},
        {"cells_parallel", [](PropulsionChain& c) { c.cells_parallel = 1; }},
        {"cell_resistance", [](PropulsionChain& c) { c.cell_resistance = 0.004; }},
        {"cell_cutoff_voltage", [](PropulsionChain& c) { c.cell_cutoff_voltage = 3.2; }},
        {"pack_polarization_resistance", [](PropulsionChain& c) { c.pack_polarization_resistance = 0.0; }},
        {"pack_current_max", [](PropulsionChain& c) { c.pack_current_max = 100.0; }},
        {"an ocv value", [](PropulsionChain& c) { c.ocv_table[3] = 3.9f; }},
        {"ocv one longer", [](PropulsionChain& c) { c.ocv_table.push_back(4.25f); }},
        {"motors_on_bus", [](PropulsionChain& c) { c.motors_on_bus = 6; }},
        {"two doubles swapped (the order is folded)",
         [](PropulsionChain& c) { std::swap(c.current_max, c.pack_current_max); }},
    };
    const uint64_t base = propulsion_identity(fixed_chain());
    for (const auto& [why, change] : changes) {
        PropulsionChain c = fixed_chain();
        change(c);
        EXPECT_NE(propulsion_identity(c), base) << why;
    }
}

TEST(PropulsionIdentity, ACopyKeepsIt) {
    const PropulsionChain c = fixed_chain();
    const PropulsionChain copied = c;
    PropulsionChain assigned = make_chain();
    assigned = c;
    PropulsionChain moved = fixed_chain();
    const PropulsionChain moved_into = std::move(moved);
    for (const PropulsionChain* k : std::vector<const PropulsionChain*>{&c, &copied, &assigned, &moved_into}) {
        EXPECT_EQ(propulsion_identity(*k), kFixedPropulsionIdentity) << std::hex << propulsion_identity(*k);
    }
}

TEST(PropulsionIdentity, ChainsEqualFieldForFieldAreEqual) {
    // The same chain built twice, and once field by field in reverse order
    // with its tables made another way: equal fields, equal identities.
    const PropulsionChain a = fixed_chain();
    PropulsionChain b;
    b.motors_on_bus = 4;
    b.ocv_table.assign({3.3f, 3.6f, 3.75f, 3.95f, 4.2f});
    b.pack_current_max = 120.0;
    b.pack_polarization_resistance = 0.002;
    b.cell_cutoff_voltage = 3.0;
    b.cell_resistance = 0.005;
    b.cells_parallel = 2;
    b.cells_series = 4;
    b.table.cq = std::vector<float>{0.008f, 0.007f, 0.006f, 0.005f, 0.004f};
    b.table.ct.reserve(16);  // capacity is not a field
    for (const float v : {0.12f, 0.1f, 0.08f, 0.06f, 0.04f}) b.table.ct.push_back(v);
    b.table.j_max = 0.8f;
    b.table.j_min = 0.0f;
    b.diameter = 0.1524;
    b.inductance = 15e-6;
    b.pole_pairs = 7.0;
    b.esc_current_total_max = 160.0;
    b.current_max = 50.0;
    b.current_min = 0.0;
    b.no_load_current = 1.2;
    b.resistance = 0.075;
    b.kv = 256.56340004316644;
    EXPECT_EQ(propulsion_identity(a), propulsion_identity(fixed_chain())) << "built twice";
    EXPECT_EQ(propulsion_identity(b), propulsion_identity(a)) << std::hex << propulsion_identity(b);
    // Non-vacuity: a chain that differs does not compare equal.
    EXPECT_NE(propulsion_identity(make_chain()), propulsion_identity(a));
}
