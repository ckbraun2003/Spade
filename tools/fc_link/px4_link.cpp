#include "px4_link.hpp"

#include <algorithm>
#include <string>
#include <variant>

namespace spade::fc_link {

namespace {

Error closed(const std::string& why) { return Error{Code::io_error, "px4_link_closed: " + why}; }

}  // namespace

Result<void> Px4Link::fill(std::chrono::milliseconds wait) {
    auto got = channel_->read_some(wait);
    if (!got) return std::unexpected(closed(got.error().context));
    if (!got->empty()) parser_.feed(*got);
    report_.parser = parser_.stats();
    return {};
}

Px4Link::Taken Px4Link::take() {
    Taken t;
    while (auto frame = parser_.next()) {
        const Result<mavlink::Message> m = mavlink::decode(*frame);
        if (!m) continue;
        if (std::holds_alternative<mavlink::Heartbeat>(*m)) {
            ++report_.heartbeats_seen;
        } else if (std::holds_alternative<mavlink::CommandLong>(*m)) {
            ++report_.command_longs_seen;
        } else if (const auto* a = std::get_if<mavlink::HilActuatorControls>(&*m)) {
            t.actuator = *a;  // the last of them is the freshest output
            ++t.actuator_count;
        }
    }
    report_.parser = parser_.stats();
    return t;
}

Result<Px4Link> Px4Link::open(std::unique_ptr<ByteChannel> channel, const Px4LinkConfig& config) {
    if (!channel) return std::unexpected(Error{Code::invalid_argument, "Px4Link::open: no channel"});
    Px4Link link(std::move(channel), config);
    const auto deadline = std::chrono::steady_clock::now() + config.heartbeat_timeout;
    while (link.report_.heartbeats_seen == 0) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
            return std::unexpected(Error{Code::io_error, "px4_no_heartbeat: PX4 sent no HEARTBEAT within " + std::to_string(config.heartbeat_timeout.count()) +
                                                             " ms of connecting (is PX4 running with the simulator link pointed at this port?)"});
        const auto step = std::min<std::chrono::milliseconds>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now) + std::chrono::milliseconds(1),
                                                              std::chrono::milliseconds(50));
        if (auto r = link.fill(step); !r) return std::unexpected(r.error());
        (void)link.take();
    }
    return link;
}

Result<ActuatorReply> Px4Link::exchange(const mavlink::HilSensor& sensor, const std::optional<mavlink::HilGps>& gps) {
    if (last_sensor_time_usec_ && sensor.time_usec <= *last_sensor_time_usec_)
        return std::unexpected(Error{Code::invalid_argument, "Px4Link::exchange: time_usec " + std::to_string(sensor.time_usec) +
                                                                 " does not increase (last " + std::to_string(*last_sensor_time_usec_) +
                                                                 "); PX4 sets its whole clock from it"});

    const auto send = [&](const mavlink::Message& m) -> Result<void> {
        const std::vector<uint8_t> bytes = mavlink::encode(mavlink::Header{seq_++, config_.sender.sysid, config_.sender.compid}, m);
        if (auto w = channel_->write(bytes); !w) return std::unexpected(closed(w.error().context));
        return {};
    };
    ++report_.sensor_messages;
    last_sensor_time_usec_ = sensor.time_usec;
    report_.last_time_usec = sensor.time_usec;
    if (auto r = send(sensor); !r) return std::unexpected(r.error());
    if (gps) {
        if (auto r = send(*gps); !r) return std::unexpected(r.error());
        ++report_.gps_messages;
    }

    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + config_.reply_timeout;
    const auto note_wait = [&] {
        const auto waited = std::chrono::steady_clock::now() - start;
        report_.wall_wait_total += std::chrono::duration_cast<std::chrono::nanoseconds>(waited);
        report_.wall_wait_max = std::max(report_.wall_wait_max, std::chrono::duration_cast<std::chrono::nanoseconds>(waited));
    };

    Taken got;
    for (;;) {
        got = take();
        if (got.actuator) break;
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            note_wait();
            return std::unexpected(Error{Code::io_error, "px4_no_reply: no HIL_ACTUATOR_CONTROLS within " + std::to_string(config_.reply_timeout.count()) +
                                                             " ms after HIL_SENSOR time_usec " + std::to_string(sensor.time_usec) + " (sensor message " +
                                                             std::to_string(report_.sensor_messages) +
                                                             "); PX4 has no lockstep timeout, so a silent PX4 stalls both sides"});
        }
        const auto step = std::min<std::chrono::milliseconds>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now) + std::chrono::milliseconds(1),
                                                              std::chrono::milliseconds(50));
        if (auto r = fill(step); !r) {
            note_wait();
            return std::unexpected(r.error());
        }
    }
    note_wait();

    if (got.actuator->time_usec < sensor.time_usec)
        return std::unexpected(Error{Code::io_error, "px4_stale_reply: HIL_ACTUATOR_CONTROLS time_usec " + std::to_string(got.actuator->time_usec) +
                                                         " is before the HIL_SENSOR it should answer (" + std::to_string(sensor.time_usec) + ")"});
    ++report_.replies;
    report_.max_replies_per_sensor = std::max(report_.max_replies_per_sensor, got.actuator_count);

    ActuatorReply out;
    out.time_usec = got.actuator->time_usec;
    out.controls = got.actuator->controls;
    out.armed = (got.actuator->mode & 128) != 0;
    out.lockstep = (got.actuator->flags & 1) != 0;
    return out;
}

}  // namespace spade::fc_link
