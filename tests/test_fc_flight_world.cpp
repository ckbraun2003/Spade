// The flight film's world (tests/fc_flight_world.hpp) flown WITHOUT PX4: a made-up altitude controller (PI-D on the world's own truth, one common duty
// for the four motors, converted to rotor speeds through the model's own propulsion chain) takes the X500 from its feet to 2 m and holds it. This
// answers, before any autopilot is involved, whether Physics' kit model can hold 2 m at a sane duty and what duty that is; it does not tune the model.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "fc_flight_world.hpp"
#include "fc_link/autopilot_map.hpp"

namespace {

using namespace fc_flight_test;
namespace fcl = spade::fc_link;

constexpr uint64_t kDtNs = 500'000;
constexpr uint32_t kImuDivider = 8;  // 4 ms

TEST(FcFlightWorld, TheX500StandsOnItsFeetAndALevelUnpoweredWorldHoldsStill) {
    std::optional<FlightWorld> w = make_flight_world();
    ASSERT_TRUE(w.has_value());
    ASSERT_TRUE(w->sim.step(2000).has_value());  // one second, no power
    EXPECT_NEAR(w->altitude_m(), 0.0, 0.002) << "resting on its feet: the centre of mass where the ground contact test says it rests";
    ASSERT_EQ(w->bindings.size(), 4u) << "IMU, barometer, magnetometer, GNSS";
}

TEST(FcFlightWorld, ACommonDutyControllerTakesTheX500ToTwoMetersAndHoldsItAtASaneDuty) {
    std::optional<FlightWorld> w = make_flight_world();
    ASSERT_TRUE(w.has_value());
    auto host = spade::fc_host::LockstepHost::open(w->sim, w->vehicle, w->bindings);
    ASSERT_TRUE(host.has_value()) << host.error().context;
    ASSERT_TRUE(w->model.model.propulsion.has_value());
    const auto& chain = *w->model.model.propulsion;

    const double dt = kImuDivider * kDtNs * 1e-9;  // the controller's period, 4 ms
    double integral = 0.0, last_alt = 0.0, duty = 0.0;
    double duty_sum = 0.0, alt_min = 1e9, alt_max = -1e9;
    uint32_t held = 0;
    const double target = 2.0;
    const uint32_t periods = static_cast<uint32_t>(25.0 / dt);  // 25 s
    for (uint32_t k = 1; k <= periods; ++k) {
        const double alt = w->altitude_m();
        const double vel = (alt - last_alt) / dt;
        last_alt = alt;
        const double err = target - alt;
        integral = std::clamp(integral + err * dt, -2.0, 2.0);
        duty = std::clamp(0.45 + 0.15 * err - 0.25 * vel + 0.05 * integral, 0.0, 1.0);
        const auto omega = fcl::autopilot_duties_to_rotor_commands(chain, std::array<double, 4>{duty, duty, duty, duty}, 1.225, 1.0);
        ASSERT_TRUE(omega.has_value()) << omega.error().context;
        ASSERT_TRUE(host->command(*omega).has_value());
        ASSERT_TRUE(host->advance_to(static_cast<uint64_t>(k) * kImuDivider * kDtNs).has_value());
        ASSERT_TRUE(host->take_sensors().has_value());
        if (k % 250 == 0) {  // a line a second of simulated time: what the controller saw and sent
            const auto b = w->sim.body(w->vehicle.body);
            std::cout << "t " << k * dt << " s  altitude " << alt << " m  vy " << (b ? static_cast<double>((*b)->vel.y) : 0.0) << "  duty " << duty << "  omega0 " << (*omega)[0] << " rad/s
";
        }
        if (k * dt > 20.0) {  // the last five seconds
            duty_sum += duty;
            alt_min = std::min(alt_min, alt);
            alt_max = std::max(alt_max, alt);
            ++held;
        }
    }
    ASSERT_GT(held, 0u);
    const double hover_duty = duty_sum / held;
    std::cout << "x500 hover: duty " << hover_duty << ", altitude " << alt_min << " .. " << alt_max << " m over the last 5 s\n";
    EXPECT_GT(alt_min, target - 0.05);
    EXPECT_LT(alt_max, target + 0.05);
    EXPECT_GT(hover_duty, 0.25) << "a sane hover duty is between a quarter and nine tenths";
    EXPECT_LT(hover_duty, 0.90);
}

}  // namespace
