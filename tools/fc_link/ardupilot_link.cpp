#include "fc_link/ardupilot_link.hpp"

#include <array>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "fc_link/autopilot_map.hpp"
#include "sensors/imu.hpp"

namespace spade::fc_link {

namespace {

[[nodiscard]] Error refusal(Code code, std::string text) { return Error{code, "ArduPilotLink::respond: " + std::move(text)}; }

}  // namespace

Result<ArduPilotLink> ArduPilotLink::create(const ArduPilotConfig& config) {
    if (config.chain == nullptr) return std::unexpected(Error{Code::invalid_argument, "ArduPilotLink::create: no propulsion chain in the config"});
    return ArduPilotLink(config);
}

Result<ArduPilotStep> ArduPilotLink::respond(fc_host::LockstepHost& host, const ardupilot_json::ServoPacket& packet) {
    const std::span<const uint64_t> periods = host.sensor_periods_ticks();
    if (config_.imu_binding >= periods.size()) {
        return std::unexpected(refusal(Code::invalid_argument, "imu_binding " + std::to_string(config_.imu_binding) + " is not one of the host's " +
                                                                std::to_string(periods.size()) + " sensors"));
    }
    if (packet.pwm.size() < kAutopilotQuadXMotors) {
        return std::unexpected(refusal(Code::invalid_argument, "the servo packet carries " + std::to_string(packet.pwm.size()) + " channels, the motors need 4"));
    }
    const uint64_t period_ticks = periods[config_.imu_binding];
    const uint64_t dt_ns = host.dt_ns();
    const uint64_t period_ns = period_ticks * dt_ns;
    // The IMU sample of the period is the frame's gyro and accelerometer, so the frame is exactly one IMU period: frame_rate * period == 1 s.
    if (packet.frame_rate == 0 || static_cast<uint64_t>(packet.frame_rate) * period_ns != 1'000'000'000ULL) {
        return std::unexpected(refusal(Code::invalid_argument, "the packet's frame rate of " + std::to_string(packet.frame_rate) + " Hz is not the IMU period (" +
                                                                std::to_string(period_ticks) + " ticks of " + std::to_string(dt_ns) + " ns = " + std::to_string(period_ns) +
                                                                " ns)"));
    }

    const ardupilot_json::FrameKind kind = tracker_.observe(packet.frame_count);
    if (kind == ardupilot_json::FrameKind::restart) {
        return std::unexpected(refusal(Code::invalid_argument, "ArduPilot restart: frame_count fell to " + std::to_string(packet.frame_count) +
                                                                ", and the world cannot be rewound, so the run is over"));
    }
    if (kind == ardupilot_json::FrameKind::resend) {
        if (!last_) return std::unexpected(refusal(Code::internal, "a resend with no reply to repeat"));
        const Result<void> ok = clock_.accept(last_->timestamp_s, /*resend=*/true);
        if (!ok) return std::unexpected(ok.error());
        return ArduPilotStep{*last_, kind, host.tick()};
    }

    std::array<double, kAutopilotQuadXMotors> duties{};
    for (std::size_t i = 0; i < kAutopilotQuadXMotors; ++i) {
        const Result<double> duty = ardupilot_pwm_to_duty(packet.pwm[i]);
        if (!duty) return std::unexpected(duty.error());
        duties[i] = *duty;
    }
    const Result<std::array<float, kAutopilotQuadXMotors>> omega = autopilot_duties_to_rotor_commands(*config_.chain, duties, config_.density, config_.soc);
    if (!omega) return std::unexpected(omega.error());
    const Result<void> commanded = host.command(*omega);
    if (!commanded) return std::unexpected(commanded.error());
    const Result<void> advanced = host.advance_to(host.sim_time_ns() + period_ns);
    if (!advanced) return std::unexpected(advanced.error());

    const Result<std::vector<fc_host::Sample>> samples = host.take_sensors();
    if (!samples) return std::unexpected(samples.error());
    const fc_host::Sample* imu = nullptr;
    for (const fc_host::Sample& s : *samples) {
        if (s.kind == fc_host::SensorKind::imu && s.binding == config_.imu_binding) imu = &s;
    }
    if (imu == nullptr) return std::unexpected(refusal(Code::internal, "the host gave no IMU sample for a whole IMU period"));
    sensors::ImuSample sample{};
    std::memcpy(&sample, imu->bytes.data(), sizeof(sample));
    const Result<fc_host::Truth> truth = host.truth();
    if (!truth) return std::unexpected(truth.error());

    ardupilot_json::StateReply state;
    state.timestamp_s = static_cast<double>(host.sim_time_ns()) / 1.0e9;
    const glm::dvec3 gyro = frames::body_to_frd(config_.frame, glm::dvec3(sample.gyro));
    const glm::dvec3 accel = frames::body_to_frd(config_.frame, glm::dvec3(sample.accel));
    const glm::dvec3 position = frames::position_to_ned(config_.frame, glm::dvec3(truth->pos), config_.home_world);
    const glm::dvec3 velocity = frames::world_to_ned(config_.frame, glm::dvec3(truth->vel));
    const glm::dquat attitude = frames::attitude_to_ned_frd(config_.frame, glm::dquat(truth->orient));
    state.gyro_frd = {gyro.x, gyro.y, gyro.z};
    state.accel_frd = {accel.x, accel.y, accel.z};
    state.position_ned = {position.x, position.y, position.z};
    state.velocity_ned = {velocity.x, velocity.y, velocity.z};
    state.quaternion_wxyz = {attitude.w, attitude.x, attitude.y, attitude.z};

    const Result<void> ok = clock_.accept(state.timestamp_s);
    if (!ok) return std::unexpected(ok.error());
    last_ = state;
    return ArduPilotStep{state, kind, host.tick()};
}

}  // namespace spade::fc_link
