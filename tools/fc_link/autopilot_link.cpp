#include "fc_link/autopilot_link.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <string>

#include <glm/glm.hpp>

#include "fc_link/autopilot_map.hpp"
#include "sensors/baro.hpp"
#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "sensors/mag.hpp"

namespace spade::fc_link {

namespace {

// HIL_SENSOR's accelerometer and gyroscope groups (bits 0 to 5 of fields_updated).
constexpr uint32_t kImuFields = 0x003Fu;

void put_imu(mavlink::HilSensor& m, const frames::FrameMap& frame, const glm::vec3& accel_body, const glm::vec3& gyro_body) {
    const glm::dvec3 a = frames::body_to_frd(frame, glm::dvec3(accel_body));
    const glm::dvec3 g = frames::body_to_frd(frame, glm::dvec3(gyro_body));
    m.xacc = static_cast<float>(a.x);
    m.yacc = static_cast<float>(a.y);
    m.zacc = static_cast<float>(a.z);
    m.xgyro = static_cast<float>(g.x);
    m.ygyro = static_cast<float>(g.y);
    m.zgyro = static_cast<float>(g.z);
    m.fields_updated = kImuFields;
}

// HIL_SENSOR's magnetometer group (bits 6 to 8) and barometer group (abs_pressure bit 9, pressure_alt bit 11, temperature bit 12; no differential
// pressure), marked updated only for a NEW sample.
constexpr uint32_t kMagFields = 0x01C0u;
constexpr uint32_t kBaroFields = 0x1A00u;

void put_mag(mavlink::HilSensor& m, const frames::FrameMap& frame, const glm::vec3& field_body_ut) {
    const glm::dvec3 f = frames::body_to_frd(frame, glm::dvec3(field_body_ut)) / 100.0;  // microtesla to gauss
    m.xmag = static_cast<float>(f.x);
    m.ymag = static_cast<float>(f.y);
    m.zmag = static_cast<float>(f.z);
    m.fields_updated |= kMagFields;
}

void put_baro(mavlink::HilSensor& m, float pressure_pa, float temperature_c, float pressure_alt_m) {
    m.abs_pressure = pressure_pa / 100.0f;  // hPa
    m.temperature = temperature_c;
    m.pressure_alt = pressure_alt_m;
    m.fields_updated |= kBaroFields;
}

// The ISA altitude of a pressure (the barometer sample carries its own; a boot message has only the held pressure): the troposphere formula.
float isa_altitude_m(float pressure_pa) {
    return 44330.77f * (1.0f - std::pow(pressure_pa / 101325.0f, 0.190263f));
}

constexpr uint64_t kBootGpsEveryUsec = 200000u;  // a boot message carries a still fix every 200 ms of link time

// A GNSS fix (Spade's world frame) as HIL_GPS: the position world -> NED about the home point -> geodetic through the config's local frame; the
// velocity to NED in cm/s with the ground speed and the course (65535 below 5 cm/s, where it is not defined). HDOP and VDOP are 1.0 (100): Spade's
// receiver model carries sigmas, not dilutions of precision. A height is the anchor's ellipsoid height plus -Down (frames.hpp).
mavlink::HilGps make_gps(const ExchangeConfig& config, const glm::vec3& position, const glm::vec3& velocity, uint64_t time_usec) {
    const glm::dvec3 ned = frames::position_to_ned(config.frame, glm::dvec3(position), config.home_world);
    const geo::Geodetic g = frames::ned_to_geodetic(*config.geo_frame, ned);
    const glm::dvec3 v = frames::world_to_ned(config.frame, glm::dvec3(velocity));
    mavlink::HilGps out;
    out.time_usec = time_usec;
    out.lat = static_cast<int32_t>(std::llround(g.lat_deg * 1.0e7));
    out.lon = static_cast<int32_t>(std::llround(g.lon_deg * 1.0e7));
    out.alt = static_cast<int32_t>(std::llround(g.height_m * 1000.0));
    out.vn = static_cast<int16_t>(std::llround(v.x * 100.0));
    out.ve = static_cast<int16_t>(std::llround(v.y * 100.0));
    out.vd = static_cast<int16_t>(std::llround(v.z * 100.0));
    const double speed = std::hypot(v.x, v.y);
    out.vel = static_cast<uint16_t>(std::llround(speed * 100.0));
    out.cog = 65535;
    if (speed >= 0.05) {
        double deg = std::atan2(v.y, v.x) * 180.0 / 3.14159265358979323846;
        if (deg < 0.0) deg += 360.0;
        out.cog = static_cast<uint16_t>(std::llround(deg * 100.0) % 36000);
    }
    out.eph = 100;
    out.epv = 100;
    out.fix_type = 3;
    out.satellites_visible = 10;
    return out;
}

}  // namespace

Result<ExchangeResult> AutopilotLink::exchange(fc_host::LockstepHost& host) {
    if (config_.chain == nullptr) return std::unexpected(Error{Code::invalid_argument, "AutopilotLink::exchange: no propulsion chain in the config"});
    const std::span<const uint64_t> periods = host.sensor_periods_ticks();
    if (config_.imu_binding >= periods.size()) {
        return std::unexpected(Error{Code::invalid_argument, "AutopilotLink::exchange: imu_binding " + std::to_string(config_.imu_binding) +
                                                              " is not one of the host's " + std::to_string(periods.size()) + " sensors"});
    }
    const uint64_t period_ticks = periods[config_.imu_binding];
    const uint64_t dt_ns = host.dt_ns();
    const uint64_t period_usec = period_ticks * dt_ns / 1000u;

    mavlink::HilSensor message;
    std::optional<mavlink::HilGps> gps;
    const bool can_send_gps = config_.gnss_binding.has_value() && config_.geo_frame.has_value();
    if (!boot_over_) {
        // The autopilot is booting, or has not said so yet: the world is held and the message is built from the held state.
        const uint64_t step = boot_steps_ + 1u;
        const Result<fc_host::BootReadings> held = host.boot_readings(step, config_.boot);
        if (!held) return std::unexpected(held.error());
        put_imu(message, config_.frame, held->accel_body, held->gyro_body);
        message.time_usec = step * period_usec;
        if (config_.baro_binding && held->has_baro) put_baro(message, held->pressure_pa, held->temperature_c, isa_altitude_m(held->pressure_pa));
        if (config_.mag_binding && held->has_mag) put_mag(message, config_.frame, held->mag_body_ut);
        const uint64_t every = std::max<uint64_t>(1u, kBootGpsEveryUsec / std::max<uint64_t>(1u, period_usec));
        if (can_send_gps && (step - 1u) % every == 0u) gps = make_gps(config_, held->position, held->velocity, message.time_usec);
    } else {
        const Result<void> advanced = host.advance_to(host.sim_time_ns() + period_ticks * dt_ns);
        if (!advanced) return std::unexpected(advanced.error());
        const Result<std::vector<fc_host::Sample>> samples = host.take_sensors();
        if (!samples) return std::unexpected(samples.error());
        const fc_host::Sample* imu = nullptr;
        for (const fc_host::Sample& s : *samples) {
            if (s.kind == fc_host::SensorKind::imu && s.binding == config_.imu_binding) imu = &s;
        }
        if (imu == nullptr) {
            return std::unexpected(Error{Code::internal, "AutopilotLink::exchange: the host gave no IMU sample for a whole IMU period"});
        }
        sensors::ImuSample sample{};
        std::memcpy(&sample, imu->bytes.data(), sizeof(sample));
        put_imu(message, config_.frame, sample.accel, sample.gyro);
        message.time_usec = boot_end_usec_ + imu->time_ns / 1000u;
        // The newest sample of each other bound sensor taken in this period, if any.
        const fc_host::Sample* baro = nullptr;
        const fc_host::Sample* mag = nullptr;
        for (const fc_host::Sample& s : *samples) {
            if (s.kind == fc_host::SensorKind::baro && config_.baro_binding && s.binding == *config_.baro_binding) baro = &s;
            if (s.kind == fc_host::SensorKind::mag && config_.mag_binding && s.binding == *config_.mag_binding) mag = &s;
        }
        if (baro != nullptr) {
            sensors::BaroSample b{};
            std::memcpy(&b, baro->bytes.data(), sizeof(b));
            put_baro(message, b.pressure_pa, b.temperature_c, b.pressure_alt_m);
        }
        if (mag != nullptr) {
            sensors::MagSample g{};
            std::memcpy(&g, mag->bytes.data(), sizeof(g));
            put_mag(message, config_.frame, g.field_body_ut);
        }
        if (can_send_gps) {
            const fc_host::Sample* fix_sample = nullptr;
            for (const fc_host::Sample& s : *samples) {
                if (s.kind == fc_host::SensorKind::gnss && s.binding == *config_.gnss_binding) fix_sample = &s;
            }
            if (fix_sample != nullptr) {
                sensors::GnssFix fix{};
                std::memcpy(&fix, fix_sample->bytes.data(), sizeof(fix));
                gps = make_gps(config_, fix.position, fix.velocity, message.time_usec);
            }
        }
    }

    // A group with no new sample carries its last values and is not marked updated.
    if ((message.fields_updated & kBaroFields) == 0u) {
        message.abs_pressure = last_.abs_pressure;
        message.temperature = last_.temperature;
        message.pressure_alt = last_.pressure_alt;
    }
    if ((message.fields_updated & kMagFields) == 0u) {
        message.xmag = last_.xmag;
        message.ymag = last_.ymag;
        message.zmag = last_.zmag;
    }
    last_ = message;

    const Result<ActuatorReply> reply = transport_(message, gps);
    if (!reply) return std::unexpected(reply.error());

    ExchangeResult out;
    out.time_usec = message.time_usec;
    if (reply->booting) {
        ++boot_steps_;
        out.booting = true;
        out.tick = host.tick();
        return out;
    }
    if (!boot_over_) {
        const Result<void> noted = host.note_boot(boot_steps_, message.time_usec, config_.boot);
        if (!noted) return std::unexpected(noted.error());
        boot_end_usec_ = message.time_usec;
        boot_over_ = true;
    }
    std::array<double, kAutopilotQuadXMotors> duties{};
    for (std::size_t i = 0; i < kAutopilotQuadXMotors; ++i) {
        const Result<double> duty = px4_output_to_duty(reply->controls[i]);
        if (!duty) return std::unexpected(duty.error());
        duties[i] = *duty;
    }
    const Result<std::array<float, kAutopilotQuadXMotors>> omega =
        autopilot_duties_to_rotor_commands(*config_.chain, duties, config_.density, config_.soc);
    if (!omega) return std::unexpected(omega.error());
    const Result<void> commanded = host.command(*omega);
    if (!commanded) return std::unexpected(commanded.error());
    out.tick = host.tick();
    return out;
}

}  // namespace spade::fc_link
