#include "ardupilot_json.hpp"

#include <charconv>
#include <cmath>
#include <system_error>

namespace spade::fc_link::ardupilot_json {

namespace {

[[nodiscard]] uint16_t le16(std::span<const uint8_t> b, std::size_t at) { return static_cast<uint16_t>(b[at] | (b[at + 1] << 8)); }
[[nodiscard]] uint32_t le32(std::span<const uint8_t> b, std::size_t at) {
    return static_cast<uint32_t>(b[at]) | (static_cast<uint32_t>(b[at + 1]) << 8) | (static_cast<uint32_t>(b[at + 2]) << 16) | (static_cast<uint32_t>(b[at + 3]) << 24);
}

[[nodiscard]] Error refusal(std::string text) { return Error{Code::invalid_argument, std::move(text)}; }

// Shortest text that reads back as the same double, in plain decimal (never an exponent): to_chars does not look at the locale.
void append_number(std::string& out, double v) {
    char buf[400];
    const auto [end, ec] = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::fixed);
    out.append(buf, ec == std::errc() ? end : buf);
}

template <std::size_t N>
[[nodiscard]] bool append_array(std::string& out, const char* name, const std::array<double, N>& values, std::string& bad) {
    out += '"';
    out += name;
    out += "\":[";
    for (std::size_t i = 0; i < N; ++i) {
        if (!std::isfinite(values[i])) {
            bad = std::string(name) + "[" + std::to_string(i) + "] is not finite";
            return false;
        }
        if (i) out += ',';
        append_number(out, values[i]);
    }
    out += ']';
    return true;
}

}  // namespace

Result<ServoPacket> decode_servo_packet(std::span<const uint8_t> bytes) {
    std::size_t channels = 0;
    uint16_t want_magic = 0;
    if (bytes.size() == kPacket16Bytes) {
        channels = 16;
        want_magic = kMagic16;
    } else if (bytes.size() == kPacket32Bytes) {
        channels = 32;
        want_magic = kMagic32;
    } else {
        return std::unexpected(refusal("servo packet length " + std::to_string(bytes.size()) + " is neither 40 (16 channels) nor 72 (32 channels)"));
    }
    const uint16_t magic = le16(bytes, 0);
    if (magic != want_magic) {
        return std::unexpected(refusal("servo packet magic " + std::to_string(magic) + " is not " + std::to_string(want_magic) + ", the magic of a " + std::to_string(bytes.size()) +
                                       "-byte packet"));
    }
    ServoPacket p;
    p.frame_rate = le16(bytes, 2);
    p.frame_count = le32(bytes, 4);
    p.pwm.reserve(channels);
    for (std::size_t i = 0; i < channels; ++i) p.pwm.push_back(le16(bytes, 8 + 2 * i));
    return p;
}

Result<std::string> encode_state_reply(const StateReply& s) {
    if (!std::isfinite(s.timestamp_s)) return std::unexpected(refusal("timestamp is not finite"));
    std::string out;
    out.reserve(256);
    std::string bad;
    out += "{\"timestamp\":";
    append_number(out, s.timestamp_s);
    out += ",\"imu\":{";
    if (!append_array(out, "gyro", s.gyro_frd, bad)) return std::unexpected(refusal("imu " + bad));
    out += ',';
    if (!append_array(out, "accel_body", s.accel_frd, bad)) return std::unexpected(refusal("imu accel_body: " + bad));
    out += "},";
    if (!append_array(out, "position", s.position_ned, bad)) return std::unexpected(refusal(bad));
    out += ',';
    if (!append_array(out, "quaternion", s.quaternion_wxyz, bad)) return std::unexpected(refusal(bad));
    out += ',';
    if (!append_array(out, "velocity", s.velocity_ned, bad)) return std::unexpected(refusal(bad));
    out += "}\n";
    return out;
}

FrameKind FrameTracker::observe(uint32_t frame_count) {
    FrameKind kind = FrameKind::fresh;
    if (last_) {
        if (frame_count == *last_) {
            kind = FrameKind::resend;
        } else if (frame_count < *last_) {
            kind = FrameKind::restart;
        } else {
            skipped_ += static_cast<uint64_t>(frame_count) - *last_ - 1;
        }
    }
    last_ = frame_count;
    return kind;
}

Result<void> ReplyClock::accept(double timestamp_s, bool resend) {
    if (!std::isfinite(timestamp_s)) return std::unexpected(refusal("reply timestamp is not finite"));
    if (last_) {
        const double dt = timestamp_s - *last_;
        const bool ok = (resend && dt == 0.0) || (dt > 0.0 && dt < kMaxReplyStepS);
        if (!ok) return std::unexpected(refusal("reply timestamp step dt = " + std::to_string(dt) + " s is outside (0, " + std::to_string(kMaxReplyStepS) + ") s"));
    }
    last_ = timestamp_s;
    return {};
}

}  // namespace spade::fc_link::ardupilot_json
