// test_sensor_profiles.cpp -- the sensor profiles of the autopilot link (tools/fc_link/sensor_profiles.hpp; docs/design/physics/plans/2026-11-21-fc-physics.md
// section 1.4): the graded numbers of the X500 kit's sensors as spawn records for the IMU, barometer, magnetometer and GNSS rows.
//
// EVERY EXPECTED VALUE HERE IS WRITTEN OUT FROM ITS SOURCE, not read back from the header: the datasheet density and the sample rate give the per-sample
// sigma (the shipped rule, sigma = density x sqrt(f)), the CEP gives the per-axis sigma (1.5 m / 1.1774), the bias instability gives the walk density
// (Allan deviation of a random walk at tau: K sqrt(tau / 3)), and so on. A test that re-typed the header's own constants would pass whatever they were.
// Libm is used for the references: tests may, engine/ may not.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>

#include <glm/glm.hpp>

#include "fc_link/sensor_profiles.hpp"
#include "sensors/baro.hpp"
#include "sensors/mag.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "world/builder.hpp"

namespace {

using spade::fc_link::SensorProfile;
using spade::fc_link::SensorSet;
using spade::fc_link::sensor_set;

constexpr double kPi = 3.14159265358979323846;
constexpr double kG0 = 9.80665;
constexpr uint64_t kTickNs = 500'000;  // the link's tick (fc-physics 5.1)

// The rates (Hz), and what each is a rate OF: the IMU message 250 Hz (the lead's figure), the barometer 50 Hz and the magnetometer 100 Hz (the PX4 reference
// rates of fc-physics 1.4), the GNSS 10 Hz (the module's rate in the Holybro page).
constexpr double kImuHz = 250.0;
constexpr double kBaroHz = 50.0;
constexpr double kMagHz = 100.0;
constexpr double kGnssHz = 10.0;

const glm::vec3 kField(20.0f, -43.0f, 1.0f);  // an arbitrary site's field (x north, y up, z east, uT): the profile takes the field from the caller

[[nodiscard]] SensorSet make(SensorProfile profile, uint64_t dt_ns = kTickNs, float altitude = 0.0f) {
    const spade::Result<SensorSet> set = sensor_set(profile, dt_ns, kField, altitude);
    EXPECT_TRUE(set.has_value()) << (set ? "" : set.error().context);
    return set ? *set : SensorSet{};
}

}  // namespace

// ===========================================================================
// THE RATES, AS DIVIDERS OF THE TICK
// ===========================================================================

TEST(SensorProfiles, TheDividersFollowTheTick) {
    for (const SensorProfile p : {SensorProfile::reference, SensorProfile::kit}) {
        const SensorSet at500 = make(p, 500'000);
        EXPECT_EQ(at500.imu.rate_divider, 8u);   // 4 ms
        EXPECT_EQ(at500.baro.rate_divider, 40u);   // 20 ms
        EXPECT_EQ(at500.mag.rate_divider, 20u);    // 10 ms
        EXPECT_EQ(at500.gnss.rate_divider, 200u);  // 100 ms
        const SensorSet at1000 = make(p, 1'000'000);
        EXPECT_EQ(at1000.imu.rate_divider, 4u);
        EXPECT_EQ(at1000.baro.rate_divider, 20u);
        EXPECT_EQ(at1000.mag.rate_divider, 10u);
        EXPECT_EQ(at1000.gnss.rate_divider, 100u);
    }
}

// A tick that does not divide a sensor's period is refused by name, never rounded: a 3 ms tick cannot carry a 4 ms IMU period.
TEST(SensorProfiles, ATickThatDoesNotDivideAPeriodIsRefusedByName) {
    const spade::Result<SensorSet> bad = sensor_set(SensorProfile::kit, 3'000'000, kField);
    ASSERT_FALSE(bad.has_value()) << "a 3 ms tick was accepted";
    EXPECT_EQ(bad.error().code, spade::Code::invalid_argument);
    EXPECT_NE(bad.error().context.find("imu"), std::string::npos) << bad.error().context;
    EXPECT_NE(bad.error().context.find("250"), std::string::npos) << bad.error().context;
    const spade::Result<SensorSet> zero = sensor_set(SensorProfile::kit, 0, kField);
    ASSERT_FALSE(zero.has_value());
    EXPECT_EQ(zero.error().code, spade::Code::invalid_argument);
}

// ===========================================================================
// THE IMU
// ===========================================================================

// The kit's IMU, from the ICM-42688-P figures (recalled, not read: fc-physics 1.4 grades them so): the per-sample sigma is the density times sqrt(250 Hz); the
// walk step is the random-walk density over sqrt(250 Hz), the density the one whose Allan deviation at tau = 100 s equals the ASSUMED bias instability (5 deg/h
// and 40 ug): K = sigma_allan x sqrt(3 / tau); the turn-on sigma is the recalled initial tolerance (1 deg/s, 20 mg) taken as 3 sigma.
TEST(SensorProfiles, TheKitImuIsTheDatasheetFiguresAtTheMessageRate) {
    const SensorSet kit = make(SensorProfile::kit);
    const double gyro_density = 4.9e-5;     // rad/s/sqrt(Hz), 2.8 mdps/sqrt(Hz)
    const double accel_density = 6.9e-4;    // m/s^2/sqrt(Hz), 70 ug/sqrt(Hz)
    EXPECT_NEAR(kit.imu.sigma_g, gyro_density * std::sqrt(kImuHz), 1.0e-6 * 1.0e-1);
    EXPECT_NEAR(kit.imu.sigma_a, accel_density * std::sqrt(kImuHz), 1.0e-6 * 1.0e1);

    const double gyro_instability = 5.0 * (kPi / 180.0) / 3600.0;  // 5 deg/h in rad/s
    const double accel_instability = 40.0e-6 * kG0;                // 40 ug in m/s^2
    const double tau = 100.0;
    const double gyro_walk = gyro_instability * std::sqrt(3.0 / tau);
    const double accel_walk = accel_instability * std::sqrt(3.0 / tau);
    EXPECT_NEAR(kit.imu.sigma_bg, gyro_walk / std::sqrt(kImuHz), 1.0e-8);
    EXPECT_NEAR(kit.imu.sigma_ba, accel_walk / std::sqrt(kImuHz), 1.0e-7);

    EXPECT_NEAR(kit.imu.turn_on_sigma_g, (1.0 * kPi / 180.0) / 3.0, 1.0e-5);
    EXPECT_NEAR(kit.imu.turn_on_sigma_a, (20.0e-3 * kG0) / 3.0, 1.0e-4);

    // PX4's HIL path decodes the accelerometer and gyro at +-16 g and +-2000 deg/s, 16 bits (fc-physics 1.3, 4).
    EXPECT_NEAR(kit.imu.range_a, 16.0 * kG0, 1.0e-3);
    EXPECT_NEAR(kit.imu.range_g, 2000.0 * kPi / 180.0, 1.0e-4);
    EXPECT_EQ(kit.imu.adc_bits, 16.0f);
}

// The reference profile is PX4's own Gazebo reference: its bridge forwards the IMU with NO noise, so every noise term is 0; the converter clamps at the full
// scales PX4's path assumes and does not quantise (the reference has no converter noise either).
TEST(SensorProfiles, TheReferenceImuIsNoiseFreeButClampedAtPx4sFullScale) {
    const SensorSet ref = make(SensorProfile::reference);
    EXPECT_EQ(ref.imu.sigma_a, 0.0f);
    EXPECT_EQ(ref.imu.sigma_g, 0.0f);
    EXPECT_EQ(ref.imu.sigma_ba, 0.0f);
    EXPECT_EQ(ref.imu.sigma_bg, 0.0f);
    EXPECT_EQ(ref.imu.turn_on_sigma_a, 0.0f);
    EXPECT_EQ(ref.imu.turn_on_sigma_g, 0.0f);
    EXPECT_NEAR(ref.imu.range_a, 16.0 * kG0, 1.0e-3);
    EXPECT_NEAR(ref.imu.range_g, 2000.0 * kPi / 180.0, 1.0e-4);
    EXPECT_EQ(ref.imu.adc_bits, 0.0f);
}

// ===========================================================================
// THE BAROMETER AND THE MAGNETOMETER
// ===========================================================================

// The MS5611: white 1.2 Pa (recalled, 0.012 mbar), a Gauss-Markov bias of 5 Pa over 300 s and 1 Pa/K (both ASSUMED), the temperature's white 0.01 K (the part's
// resolution, recalled). The site's altitude is the caller's.
TEST(SensorProfiles, TheKitBarometerIsTheMs5611) {
    const SensorSet kit = make(SensorProfile::kit, kTickNs, 488.0f);
    EXPECT_FLOAT_EQ(kit.baro.sigma_p_pa, 1.2f);
    EXPECT_FLOAT_EQ(kit.baro.sigma_bias_pa, 5.0f);
    EXPECT_FLOAT_EQ(kit.baro.bias_tau_s, 300.0f);
    EXPECT_FLOAT_EQ(kit.baro.temp_coeff_pa_per_k, 1.0f);
    EXPECT_FLOAT_EQ(kit.baro.sigma_t_k, 0.01f);
    EXPECT_FLOAT_EQ(kit.baro.world_origin_altitude_m, 488.0f);
    EXPECT_NEAR(1.0 / (kit.baro.rate_divider * (kTickNs * 1e-9)), kBaroHz, 1.0e-9);
}

// The IST8310: white 0.3 uT (one LSB, ASSUMED equal), a Gauss-Markov bias with the zero offset of +-0.3 uT taken as 3 sigma (0.1 uT) over 600 s (ASSUMED), no
// hard or soft iron (PX4 calibrates it away; the profile adds one when a calibration scenario needs it). The field is the caller's.
TEST(SensorProfiles, TheKitMagnetometerIsTheIst8310) {
    const SensorSet kit = make(SensorProfile::kit);
    EXPECT_FLOAT_EQ(kit.mag.sigma_white_ut, 0.3f);
    EXPECT_FLOAT_EQ(kit.mag.sigma_bias_ut, 0.1f);
    EXPECT_FLOAT_EQ(kit.mag.bias_tau_s, 600.0f);
    EXPECT_EQ(kit.mag.hard_iron_ut, glm::vec3(0.0f));
    EXPECT_EQ(kit.mag.soft_iron, glm::mat3(1.0f));
    EXPECT_EQ(kit.mag.field_world_ut, kField);
    EXPECT_NEAR(1.0 / (kit.mag.rate_divider * (kTickNs * 1e-9)), kMagHz, 1.0e-9);
}

// The reference's barometer and magnetometer carry no noise (PX4's bridge adds none) but keep their rate, site and field.
TEST(SensorProfiles, TheReferenceBarometerAndMagnetometerAreNoiseFree) {
    const SensorSet ref = make(SensorProfile::reference, kTickNs, 488.0f);
    EXPECT_EQ(ref.baro.sigma_p_pa, 0.0f);
    EXPECT_EQ(ref.baro.sigma_t_k, 0.0f);
    EXPECT_EQ(ref.baro.sigma_bias_pa, 0.0f);
    EXPECT_EQ(ref.baro.temp_coeff_pa_per_k, 0.0f);
    EXPECT_EQ(ref.baro.world_origin_altitude_m, 488.0f);
    EXPECT_EQ(ref.baro.rate_divider, 40u);
    EXPECT_EQ(ref.mag.sigma_white_ut, 0.0f);
    EXPECT_EQ(ref.mag.sigma_bias_ut, 0.0f);
    EXPECT_EQ(ref.mag.field_world_ut, kField);
    EXPECT_EQ(ref.mag.rate_divider, 20u);
}

// ===========================================================================
// THE GNSS (the same in both profiles: PX4's own reference has a noisy GNSS)
// ===========================================================================

// 1.5 m CEP (the M9N, published) is 1.1774 sigma for a circular Gaussian: sigma_h = 1.5 / 1.1774 per axis; the vertical is 1.5 times it and the velocity
// 0.05 m/s (recalled), as PX4's own Gazebo bridge scales its down axis (published). No correlated bias (the table gives none).
TEST(SensorProfiles, TheGnssIsTheCepAsAPerAxisSigmaInBothProfiles) {
    for (const SensorProfile p : {SensorProfile::reference, SensorProfile::kit}) {
        const SensorSet set = make(p);
        const double sigma_h = 1.5 / 1.1774;
        EXPECT_NEAR(set.gnss.sigma_h, sigma_h, 1.0e-5);
        EXPECT_NEAR(set.gnss.sigma_v, 1.5 * sigma_h, 1.0e-5);
        EXPECT_FLOAT_EQ(set.gnss.sigma_vel, 0.05f);
        EXPECT_EQ(set.gnss.sigma_bias, 0.0f);
        EXPECT_EQ(set.gnss.turn_on_bias, 0u);
        EXPECT_NEAR(1.0 / (set.gnss.rate_divider * (kTickNs * 1e-9)), kGnssHz, 1.0e-9);
    }
}

// ===========================================================================
// THE ROWS TAKE THEM, AND EMIT AT THE RATES
// ===========================================================================

namespace {
// One world with gravity off, a body at rest, and all four sensors of `profile` on it, the module set with Core's opt-in barometer and magnetometer.
struct SensorBench {
    spade::Result<spade::Simulation> sim = std::unexpected(spade::Error{});
    spade::BodyRef body{};
    spade::ImuSensorRef imu{};
    spade::GnssSensorRef gnss{};
    spade::RowRef baro{};
    spade::RowRef mag{};
};

[[nodiscard]] bool make_bench(SensorBench& b, SensorProfile profile, std::string* why) {
    spade::Environment env;
    env.gravity = glm::vec3(0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    spade::Capacities caps;
    caps.bodies = 2;
    caps.force_elements = 1;
    caps.sensors = 8;
    caps.contacts = 1;
    const spade::Result<spade::WorldDesc> world = spade::WorldBuilder().name("bench").environment(env).capacities(caps).build();
    if (!world) return false;
    spade::physics::ContactParams contacts;
    contacts.restitution_e = 0.0f;
    contacts.friction_mu = 0.0f;
    contacts.proxy_radius = 0.0f;
    spade::physics::GridParams grid;
    grid.cell_size = 1.0f;
    spade::WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0x5E5E5E5EULL;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts = contacts;
    instance.grid = grid;
    const spade::WorldSetDesc set{{instance}};

    spade::modules::ModuleSet modules = spade::modules::standard_modules();
    modules.push_back(spade::modules::baro_module());
    modules.push_back(spade::modules::mag_module());
    b.sim = spade::Simulation::create(set, kTickNs, 1, {}, modules);
    if (!b.sim) {
        *why = b.sim.error().context;
        return false;
    }
    spade::BodySpawn body;
    body.pos = glm::vec3(0.0f, 488.0f, 0.0f);
    const spade::Result<spade::BodyRef> ref = b.sim->spawn(0, body);
    if (!ref) return false;
    b.body = *ref;

    const spade::Result<SensorSet> sensors = sensor_set(profile, kTickNs, kField, 0.0f);
    if (!sensors) {
        *why = sensors.error().context;
        return false;
    }
    const spade::Result<spade::ImuSensorRef> imu = b.sim->add_imu_sensor(b.body, sensors->imu);
    const spade::Result<spade::GnssSensorRef> gnss = b.sim->add_gnss_sensor(b.body, sensors->gnss);
    const spade::Result<spade::RowRef> baro = b.sim->attach_row(b.body, "baro_sensors", sensors->baro);
    const spade::Result<spade::RowRef> mag = b.sim->attach_row(b.body, "mag_sensors", sensors->mag);
    if (!imu || !gnss || !baro || !mag) {
        *why = !imu ? imu.error().context : !gnss ? gnss.error().context : !baro ? baro.error().context : mag.error().context;
        return false;
    }
    b.imu = *imu;
    b.gnss = *gnss;
    b.baro = *baro;
    b.mag = *mag;
    return static_cast<bool>(b.sim->flush_structural());
}
}  // namespace

// Every record of both profiles passes the rows' own validation, and one second of ticks (2,000 at 500 us) emits exactly the rate times one second: 250 IMU
// samples, 50 barometer, 100 magnetometer and 10 GNSS fixes.
TEST(SensorProfiles, BothProfilesAreAcceptedByTheRowsAndEmitAtTheirRates) {
    for (const SensorProfile p : {SensorProfile::reference, SensorProfile::kit}) {
        SensorBench b;
        std::string why;
        ASSERT_TRUE(make_bench(b, p, &why)) << why;
        ASSERT_TRUE(b.sim->step(2000).has_value());
        EXPECT_EQ((*b.sim->imu_sensor(b.imu))->last_index, 250u) << "profile " << static_cast<int>(p);
        EXPECT_EQ((*b.sim->gnss_sensor(b.gnss))->last_index, 10u);
        const auto baro_rows = b.sim->module_rows<spade::sensors::BaroSensorRow>("baro_sensors", b.baro.world_index);
        const auto mag_rows = b.sim->module_rows<spade::sensors::MagSensorRow>("mag_sensors", b.mag.world_index);
        ASSERT_TRUE(baro_rows.has_value() && mag_rows.has_value());
        EXPECT_EQ((*baro_rows)[b.baro.slot].last_index, 50u);
        EXPECT_EQ((*mag_rows)[b.mag.slot].last_index, 100u);
    }
}
