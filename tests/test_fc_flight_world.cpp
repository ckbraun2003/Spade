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
#include <vector>

#include "../sandbox/flight_controller.hpp"
#include "analysis/case.hpp"
#include "fc_flight_world.hpp"
#include "fc_link/autopilot_link.hpp"
#include "fc_link/sensor_profiles.hpp"

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

// PX4's sensor validators read a reading that repeats exactly as a frozen sensor and fail it as STALE. Both flight logs of dry runs 3 and 4b carry such failures
// (MAG #0 at about 3.5 s, BARO #0 and #1 later) and the barometer's logged pressure sat identical for 22 s at rest: the flight world's barometer and
// magnetometer had no noise. At rest for 6 s (300 barometer and 600 magnetometer samples) neither may repeat a reading more than a handful of times in a row.
TEST(FcFlightWorld, TheBarometerAndMagnetometerNeverRepeatAReadingBitForBitAtRest) {
    std::optional<FlightWorld> w = make_flight_world();
    ASSERT_TRUE(w.has_value());
    auto host = spade::fc_host::LockstepHost::open(w->sim, w->vehicle, w->bindings);
    ASSERT_TRUE(host.has_value()) << host.error().context;
    std::vector<float> baro;
    std::vector<std::array<float, 3>> mag;
    for (uint64_t step = 1; step <= 60; ++step) {  // 6 s in 100 ms steps
        ASSERT_TRUE(host->advance_to(step * 200 * kDtNs).has_value());
        const auto samples = host->take_sensors();
        ASSERT_TRUE(samples.has_value());
        for (const spade::fc_host::Sample& s : *samples) {
            if (s.kind == spade::fc_host::SensorKind::baro) {
                spade::sensors::BaroSample b{};
                std::memcpy(&b, s.bytes.data(), sizeof(b));
                baro.push_back(b.pressure_pa);
            } else if (s.kind == spade::fc_host::SensorKind::mag) {
                spade::sensors::MagSample m{};
                std::memcpy(&m, s.bytes.data(), sizeof(m));
                mag.push_back({m.field_body_ut.x, m.field_body_ut.y, m.field_body_ut.z});
            }
        }
    }
    ASSERT_GE(baro.size(), 290u);
    ASSERT_GE(mag.size(), 590u);
    const auto longest_run = [](const auto& v) {
        std::size_t best = 1, run = 1;
        for (std::size_t i = 1; i < v.size(); ++i) {
            run = (v[i] == v[i - 1]) ? run + 1 : 1;
            best = std::max(best, run);
        }
        return best;
    };
    EXPECT_LT(longest_run(baro), 10u) << "of " << baro.size() << " barometer samples";
    EXPECT_LT(longest_run(mag), 10u) << "of " << mag.size() << " magnetometer samples";
}

// PX4 fails a sensor that sends no new sample within its time-out. What WE send is fixed by construction (the barometer's period is 40 ticks, exactly five
// 8-tick IMU messages; the magnetometer's 20 ticks, alternately 2 and 3 messages), and this pins it through the real AutopilotLink over 12 s: a barometer group on
// every fifth message (20 ms apart, never more), a magnetometer group never more than 12 ms apart. A gap PX4's log shows around the STALE lines is therefore not ours.
TEST(FcFlightWorld, TheBarometerAndMagnetometerGroupsReachHilSensorOnScheduleWithNoGapOverTwelveSeconds) {
    std::optional<FlightWorld> w = make_flight_world();
    ASSERT_TRUE(w.has_value());
    auto host = spade::fc_host::LockstepHost::open(w->sim, w->vehicle, w->bindings);
    ASSERT_TRUE(host.has_value()) << host.error().context;
    ASSERT_TRUE(w->model.model.propulsion.has_value());

    constexpr uint32_t kBaroBits = 0x1A00u, kMagBits = 0x01C0u;
    std::vector<uint64_t> baro_at, mag_at;
    spade::fc_link::ExchangeConfig cfg;
    cfg.chain = &*w->model.model.propulsion;
    cfg.baro_binding = kBaroBinding;
    cfg.mag_binding = kMagBinding;
    auto link = spade::fc_link::AutopilotLink::create(
        [&](const spade::fc_link::mavlink::HilSensor& m, const std::optional<spade::fc_link::mavlink::HilGps>&) -> spade::Result<spade::fc_link::ActuatorReply> {
            if ((m.fields_updated & kBaroBits) == kBaroBits) baro_at.push_back(m.time_usec);
            if ((m.fields_updated & kMagBits) == kMagBits) mag_at.push_back(m.time_usec);
            spade::fc_link::ActuatorReply r;
            for (std::size_t i = 0; i < 4; ++i) r.controls[i] = 0.45f;
            r.armed = true;
            r.lockstep = true;
            return r;
        },
        cfg);
    ASSERT_TRUE(link.has_value()) << link.error().context;
    for (int i = 0; i < 3000; ++i) {
        const auto step = link->exchange(*host);
        ASSERT_TRUE(step.has_value()) << "exchange " << i << ": " << step.error().context;
    }
    ASSERT_GE(baro_at.size(), 590u);
    ASSERT_GE(mag_at.size(), 1180u);
    uint64_t baro_gap = 0, mag_gap = 0;
    for (std::size_t i = 1; i < baro_at.size(); ++i) baro_gap = std::max(baro_gap, baro_at[i] - baro_at[i - 1]);
    for (std::size_t i = 1; i < mag_at.size(); ++i) mag_gap = std::max(mag_gap, mag_at[i] - mag_at[i - 1]);
    EXPECT_EQ(baro_gap, 20000u) << "us between barometer groups, the worst of " << baro_at.size();
    EXPECT_LE(mag_gap, 12000u) << "us between magnetometer groups, the worst of " << mag_at.size();
}

// One source for the kit's sensor figures: the flight world's barometer and magnetometer noise are the kit profile's white sigmas, not numbers of their own.
TEST(FcFlightWorld, TheBarometerAndMagnetometerNoiseAreTheKitProfilesWhiteSigmas) {
    const auto kit = spade::fc_link::sensor_set(spade::fc_link::SensorProfile::kit, kDtNs, glm::vec3(20.0f, -40.0f, 5.0f), static_cast<float>(kGroundAmslM));
    ASSERT_TRUE(kit.has_value()) << kit.error().context;
    EXPECT_GT(kit->baro.sigma_p_pa, 0.0f);
    EXPECT_GT(kit->mag.sigma_white_ut, 0.0f);
    EXPECT_EQ(kBaroSigmaPa, kit->baro.sigma_p_pa);
    EXPECT_EQ(kMagSigmaUt, kit->mag.sigma_white_ut);
}

// One noise source: the flight run's boot messages use the same barometer and magnetometer sigmas as its world, which are the kit profile's.
TEST(FcFlightWorld, TheFlightRunsBootNoiseForTheBarometerAndMagnetometerIsTheKitProfilesToo) {
    const auto kit = spade::fc_link::sensor_set(spade::fc_link::SensorProfile::kit, kDtNs, glm::vec3(20.0f, -40.0f, 5.0f), static_cast<float>(kGroundAmslM));
    ASSERT_TRUE(kit.has_value()) << kit.error().context;
    const spade::fc_host::BootConfig boot = flight_boot_config();
    const spade::fc_host::BootConfig core_defaults{};
    EXPECT_EQ(boot.sigma_pressure_pa, kit->baro.sigma_p_pa);
    EXPECT_EQ(boot.sigma_mag_ut, kit->mag.sigma_white_ut);
    EXPECT_EQ(boot.sigma_accel, core_defaults.sigma_accel) << "Core's accelerometer boot sigma is not touched here";
    EXPECT_EQ(boot.sigma_gyro, core_defaults.sigma_gyro);
    EXPECT_EQ(boot.seed, core_defaults.seed);
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
