#pragma once

#include <cmath>
#include <cstdint>
#include <expected>
#include <string>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "sensors/baro.hpp"
#include "sensors/mag.hpp"
#include "sim/simulation.hpp"

// ===========================================================================
// fc_link/sensor_profiles.hpp -- the X500 kit's sensors as spawn records for the IMU, barometer, magnetometer and GNSS rows (O140 phase 1;
// docs/design/physics/plans/2026-11-21-fc-physics.md section 1.4 holds the table and the grades; this is its code). Header-only, host-side, in double
// and float arithmetic, never in the step. It lives in the link (tools/fc_link), like the actuator map: a table named for the autopilot's sensor set is
// link domain (L7).
//
// TWO PROFILES.
//   reference  PX4's own Gazebo reference: its bridge forwards the IMU, barometer and magnetometer with NO noise, so every noise term of those three is 0.
//              The IMU's converter CLAMPS at the full scales PX4's HIL path decodes (+-16 g, +-2000 deg/s) and does not quantise. The GNSS is noisy (PX4's
//              reference has a noisy GNSS). Use it for the like-for-like comparison with PX4's Gazebo flight.
//   kit        the real kit's sensors from their datasheets, every figure graded below.
// Both take the site from the caller: the world magnetic field (a magnetometer has no meaning without one) and the height of the world's y = 0 above the
// ISA datum (the barometer's). Neither is a default here, because a default would be a hidden place.
//
// THE RATES are the figures of fc-physics 1.4: IMU message 250 Hz (the lead's figure; PX4's docs do not state one), barometer 50 Hz and magnetometer 100 Hz
// (PX4's reference rates), GNSS 10 Hz (the module's rate, Holybro's GPS page). Each becomes a rate divider of the tick, and a tick that does not divide a
// sensor's period is REFUSED by name, never rounded (a 500 us tick carries them as 8, 40, 20 and 200; a 3 ms tick cannot carry a 4 ms IMU period).
//
// GRADES (published = read from the vendor's page; recalled = from memory of the datasheet, NOT re-read; assumed = a stated choice; derived = arithmetic
// from the others):
//   IMU gyro density 4.9e-5 rad/s/sqrt(Hz) (2.8 mdps/sqrt(Hz))       recalled (ICM-42688-P)
//   IMU accel density 6.9e-4 m/s^2/sqrt(Hz) (70 ug/sqrt(Hz))         recalled (same)
//   per-sample white sigma = density x sqrt(250 Hz)                  derived (the shipped IMU rule, sensors/imu.hpp section 4)
//   IMU bias instability 5 deg/h and 40 ug                           assumed
//   walk density K = instability x sqrt(3 / tau) at tau = 100 s      derived (the Allan deviation of a random walk is K sqrt(tau / 3)); the step per
//                                                                    sample is K / sqrt(250 Hz)
//   IMU turn-on offsets 1 deg/s and 20 mg                            recalled (initial tolerance), taken as 3 sigma: assumed
//   IMU converter +-16 g, +-2000 deg/s                               published as what PX4's HIL path decodes (fc-physics 1.3); 16 bits recalled
//   barometer white 1.2 Pa (0.012 mbar)                              recalled (MS5611, the part is published)
//   barometer bias 5 Pa, tau 300 s, 1 Pa/K, temperature white 0.01 K  assumed (the last is the part's resolution, recalled)
//   magnetometer white 0.3 uT                                        assumed equal to one LSB (the IST8310's resolution, published)
//   magnetometer bias 0.1 uT, tau 600 s                              the zero offset +-0.3 uT is published; taken as 3 sigma and tau assumed
//   hard iron, soft iron                                             none: PX4 calibrates them away; add when a calibration scenario needs them
//   GNSS 1.5 m CEP -> 1.5 / 1.1774 m per axis                        CEP published (M9N); the 1.1774 conversion is a circular Gaussian's: derived
//   GNSS vertical 1.5 x horizontal                                   published (PX4's own Gazebo bridge scales its down axis by 1.5)
//   GNSS velocity 0.05 m/s                                           recalled (u-blox); the Holybro page states none
//   GNSS correlated bias                                             none: the table gives none
// ===========================================================================

namespace spade::fc_link {

enum class SensorProfile { reference, kit };

// All four records of one profile, ready for add_imu_sensor, attach_row("baro_sensors"), attach_row("mag_sensors") and add_gnss_sensor. The mount is the
// body's centre of mass and the mount orientation the identity in every record: where the real sensors sit is the model's to say.
struct SensorSet {
    ImuSensorSpawn imu;
    sensors::BaroSensorSpawn baro;
    sensors::MagSensorSpawn mag;
    GnssSensorSpawn gnss;
};

namespace detail {

inline constexpr uint32_t kImuHz = 250;
inline constexpr uint32_t kBaroHz = 50;
inline constexpr uint32_t kMagHz = 100;
inline constexpr uint32_t kGnssHz = 10;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kG0 = 9.80665;

// The divider of `hz` on a tick of `dt_ns`, or an invalid_argument naming the sensor and its rate.
[[nodiscard]] inline Result<uint32_t> divider(const char* sensor, uint32_t hz, uint64_t dt_ns) {
    const uint64_t period_ns = 1'000'000'000ull / hz;
    if (dt_ns == 0 || period_ns % dt_ns != 0) {
        return std::unexpected(Error{Code::invalid_argument, std::string("sensor_set: the ") + sensor + "'s period of " + std::to_string(period_ns) + " ns (" +
                                                                 std::to_string(hz) + " Hz) is not a whole number of ticks of " + std::to_string(dt_ns) + " ns"});
    }
    return static_cast<uint32_t>(period_ns / dt_ns);
}

}  // namespace detail

[[nodiscard]] inline Result<SensorSet> sensor_set(SensorProfile profile, uint64_t dt_ns, const glm::vec3& mag_field_world_ut,
                                                  float world_origin_altitude_m = 0.0f) {
    using namespace detail;
    const Result<uint32_t> imu_div = divider("imu", kImuHz, dt_ns);
    if (!imu_div) return std::unexpected(imu_div.error());
    const Result<uint32_t> baro_div = divider("barometer", kBaroHz, dt_ns);
    if (!baro_div) return std::unexpected(baro_div.error());
    const Result<uint32_t> mag_div = divider("magnetometer", kMagHz, dt_ns);
    if (!mag_div) return std::unexpected(mag_div.error());
    const Result<uint32_t> gnss_div = divider("gnss", kGnssHz, dt_ns);
    if (!gnss_div) return std::unexpected(gnss_div.error());

    const bool kit = profile == SensorProfile::kit;
    SensorSet set;

    // --- the IMU
    set.imu.rate_divider = *imu_div;
    set.imu.range_a = static_cast<float>(16.0 * kG0);
    set.imu.range_g = static_cast<float>(2000.0 * kPi / 180.0);
    if (kit) {
        const double sqrt_f = std::sqrt(static_cast<double>(kImuHz));
        const double tau = 100.0;  // s: the instability's averaging time
        set.imu.sigma_g = static_cast<float>(4.9e-5 * sqrt_f);
        set.imu.sigma_a = static_cast<float>(6.9e-4 * sqrt_f);
        set.imu.sigma_bg = static_cast<float>(5.0 * (kPi / 180.0) / 3600.0 * std::sqrt(3.0 / tau) / sqrt_f);
        set.imu.sigma_ba = static_cast<float>(40.0e-6 * kG0 * std::sqrt(3.0 / tau) / sqrt_f);
        set.imu.turn_on_sigma_g = static_cast<float>((1.0 * kPi / 180.0) / 3.0);
        set.imu.turn_on_sigma_a = static_cast<float>((20.0e-3 * kG0) / 3.0);
        set.imu.adc_bits = 16.0f;
    }

    // --- the barometer
    set.baro.rate_divider = *baro_div;
    set.baro.world_origin_altitude_m = world_origin_altitude_m;
    if (kit) {
        set.baro.sigma_p_pa = 1.2f;
        set.baro.sigma_t_k = 0.01f;
        set.baro.sigma_bias_pa = 5.0f;
        set.baro.bias_tau_s = 300.0f;
        set.baro.temp_coeff_pa_per_k = 1.0f;
    }

    // --- the magnetometer
    set.mag.rate_divider = *mag_div;
    set.mag.field_world_ut = mag_field_world_ut;
    if (kit) {
        set.mag.sigma_white_ut = 0.3f;
        set.mag.sigma_bias_ut = 0.1f;
        set.mag.bias_tau_s = 600.0f;
    }

    // --- the GNSS (the same in both profiles)
    const double sigma_h = 1.5 / 1.1774;
    set.gnss.rate_divider = *gnss_div;
    set.gnss.sigma_h = static_cast<float>(sigma_h);
    set.gnss.sigma_v = static_cast<float>(1.5 * sigma_h);
    set.gnss.sigma_vel = 0.05f;
    return set;
}

}  // namespace spade::fc_link
