// The flight film's world (tests/fc_flight_world.hpp) flown WITHOUT PX4: a made-up altitude controller (PI-D on the world's own truth, the throttle
// for the four motors, converted to rotor speeds through the model's own propulsion chain) takes the X500 from its feet to 2 m and holds it. This
// answers, before any autopilot is involved, whether Physics' kit model can hold 2 m at a sane duty and what duty that is; it does not tune the model.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "../sandbox/flight_controller.hpp"
#include "analysis/case.hpp"
#include "fc_flight_world.hpp"

namespace {

using namespace fc_flight_test;


TEST(FcFlightWorld, TheX500StandsOnItsFeetAndALevelUnpoweredWorldHoldsStill) {
    std::optional<FlightWorld> w = make_flight_world();
    ASSERT_TRUE(w.has_value());
    ASSERT_TRUE(w->sim.step(2000).has_value());  // one second, no power
    EXPECT_NEAR(w->altitude_m(), 0.0, 0.002) << "resting on its feet: the centre of mass where the ground contact test says it rests";
    ASSERT_EQ(w->bindings.size(), 4u) << "IMU, barometer, magnetometer, GNSS";
}

// THE NUMBER THIS PINS: the pressure altitude Spade's barometer SENDS (BaroSample::pressure_alt_m) at rest, which must be the ground's 488 m above sea level
// (the datum the GNSS says through the link's geo frame) plus the centre of mass' few centimetres, not 0 m. It does not pin either of PX4's own figures;
// those are what dry run 4 measures after this fix. (Dry run 3, before it: PX4's BARO altitude read 2.060 m for the truth's 1.990, 3.5% over, and its
// FUSED height 2.032 m for the truth's 1.987, 4.5 cm or 2.26% over.)
TEST(FcFlightWorld, TheBarometerReadsTheGroundAtTheSameAltitudeAboveSeaLevelAsTheGnss) {
    std::optional<FlightWorld> w = make_flight_world();
    ASSERT_TRUE(w.has_value());
    auto host = spade::fc_host::LockstepHost::open(w->sim, w->vehicle, w->bindings);
    ASSERT_TRUE(host.has_value()) << host.error().context;
    ASSERT_TRUE(host->advance_to(200 * kDtNs).has_value());  // 0.1 s: past the barometer's first period (20 ms)
    const auto samples = host->take_sensors();
    ASSERT_TRUE(samples.has_value());
    bool seen = false;
    for (const spade::fc_host::Sample& s : *samples) {
        if (s.kind != spade::fc_host::SensorKind::baro) continue;
        spade::sensors::BaroSample b{};
        ASSERT_EQ(s.size, sizeof(b));
        std::memcpy(&b, s.bytes.data(), sizeof(b));
        EXPECT_NEAR(b.pressure_alt_m, kGroundAmslM + w->rest_com_y, 1.0) << "the sensor sits at the centre of mass, a few centimetres over the ground";
        seen = true;
    }
    EXPECT_TRUE(seen) << "a barometer sample in the first 0.1 s";
}

// The exchange is told where each sensor sits in the bindings (ExchangeConfig); those indices live in the hashed world header next to the order the world builds
// them in, not as literals in the live test.
TEST(FcFlightWorld, TheBindingIndicesNameTheSensorsInTheOrderTheWorldBuildsThem) {
    std::optional<FlightWorld> w = make_flight_world();
    ASSERT_TRUE(w.has_value());
    ASSERT_EQ(w->bindings.size(), 4u);
    EXPECT_EQ(w->bindings[0].kind, spade::fc_host::SensorKind::imu);
    EXPECT_EQ(w->bindings[kBaroBinding].kind, spade::fc_host::SensorKind::baro);
    EXPECT_EQ(w->bindings[kMagBinding].kind, spade::fc_host::SensorKind::mag);
    EXPECT_EQ(w->bindings[kGnssBinding].kind, spade::fc_host::SensorKind::gnss);
}

TEST(FcFlightWorld, ACommonDutyControllerTakesTheX500ToTwoMetersAndHoldsItAtASaneDuty) {
    std::optional<FlightWorld> w = make_flight_world();
    ASSERT_TRUE(w.has_value());
    auto host = spade::fc_host::LockstepHost::open(w->sim, w->vehicle, w->bindings);
    ASSERT_TRUE(host.has_value()) << host.error().context;
    ASSERT_TRUE(w->model.model.propulsion.has_value());
    // The attitude loop is the sandbox flight controller (the editor tour's; its own tests pin it): a quad cannot hold a height on a common duty alone, it
    // tumbles (the first run of this test did: it rose to 0.9 m and fell back). Only the throttle comes from this test.
    const auto controller = spade::sandbox::flight::FlightController::make(w->model.model, 1.225f);
    ASSERT_TRUE(controller.has_value()) << controller.error().context;

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
        const double throttle = std::clamp(0.36 + 0.15 * err - 0.25 * vel + 0.10 * integral, 0.0, 1.0);  // 0.36: about the hover throttle the first run's trace showed
        duty = spade::analysis::kDefaultIdleDuty + (1.0 - spade::analysis::kDefaultIdleDuty) * throttle;  // the arm rule's duty for the throttle
        spade::analysis::Controls controls;
        controls.armed = true;
        controls.throttle = static_cast<float>(throttle);
        const auto body = w->sim.body(w->vehicle.body);
        ASSERT_TRUE(body.has_value());
        std::array<float, 4> omega{};
        ASSERT_TRUE(controller->command(controls, **body, omega).has_value());
        ASSERT_TRUE(host->command(omega).has_value());
        ASSERT_TRUE(host->advance_to(static_cast<uint64_t>(k) * kImuDivider * kDtNs).has_value());
        ASSERT_TRUE(host->take_sensors().has_value());
        if (k % 250 == 0) {  // a line a second of simulated time: what the controller saw and sent
            const auto b = w->sim.body(w->vehicle.body);
            std::cout << "t " << k * dt << " s  altitude " << alt << " m  vy " << (b ? static_cast<double>((*b)->vel.y) : 0.0) << "  duty " << duty << "  omega0 " << omega[0] << " rad/s\n";
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
