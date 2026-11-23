#pragma once
// ---------------------------------------------------------------------------
// fc_link/ardupilot_json.hpp -- the wire layer of ArduPilot's SITL "JSON" physics-backend interface (docs/design/interface/plans/2026-11-21-fc-links.md section 6),
// as pure functions and small state machines, tested without ArduPilot.
//
// ArduPilot sends a SERVO PACKET on UDP (Spade binds 9002 and replies to the address each packet came from); Spade replies with ONE JSON object per packet. Time
// advances in ArduPilot only from the reply's `timestamp`, and only when 0 < dt < 0.1 s. This is written from the documented layout (ardupilot.org's SITL JSON page,
// the JSON example's readme, SIM_JSON.h and SIM_JSON.cpp, as the plan's section 6 read them); NO ardupilot code is copied (those files are GPL-3.0), and NO real
// ArduPilot capture has been compared yet: the golden bytes in the tests are built from the written layout. The first real packet is the phase that runs it.
//
//   decode_servo_packet  the 40-byte (16 channels, magic 18458) or 72-byte (32 channels, magic 29569) little-endian packet, refusing by name.
//   encode_state_reply   the JSON object, newline-terminated; numbers by std::to_chars (shortest round-trip, plain decimal), so the bytes do not depend on the locale.
//   FrameTracker         a repeated frame_count is a resend (answer it again), a smaller one is a restart of ArduPilot (reset the clock).
//   ReplyClock           the rule on reply timestamps: 0 < dt < 0.1 s from the previous reply, except a resend (dt == 0) and the first reply after a reset.
//
// The servo packet's pwm values go to the rotors through autopilot_map.hpp's ardupilot_pwm_to_duty and its motor map: not duplicated here.
// ---------------------------------------------------------------------------
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "core/error.hpp"

namespace spade::fc_link::ardupilot_json {

inline constexpr uint16_t kMagic16 = 18458;  // the servo packet with 16 channels
inline constexpr uint16_t kMagic32 = 29569;  // with 32 channels (SERVO_32_ENABLE)
inline constexpr std::size_t kPacket16Bytes = 40;
inline constexpr std::size_t kPacket32Bytes = 72;
inline constexpr double kMaxReplyStepS = 0.1;  // ArduPilot ignores a reply whose timestamp is this far or farther past the last

struct ServoPacket {
    uint16_t frame_rate = 0;   // SIM_RATE_HZ
    uint32_t frame_count = 0;  // rises per frame; resets when SITL restarts; a repeat is a resend
    std::vector<uint16_t> pwm;  // 16 or 32 values, microseconds (1000 to 2000)
};

// Refuses, naming what: a length that is neither 40 nor 72, a magic that is not the one that length carries.
[[nodiscard]] Result<ServoPacket> decode_servo_packet(std::span<const uint8_t> bytes);

// What Spade says back: NED and FRD, SI units.
struct StateReply {
    double timestamp_s = 0.0;                     // absolute seconds
    std::array<double, 3> gyro_frd{};             // rad/s, body
    std::array<double, 3> accel_frd{};            // m/s2, body: specific force
    std::array<double, 3> position_ned{};         // m from home
    std::array<double, 3> velocity_ned{};         // m/s
    std::array<double, 4> quaternion_wxyz{1, 0, 0, 0};  // body to NED
};

// One JSON object and a newline. Refuses a value that is not finite, naming the field.
[[nodiscard]] Result<std::string> encode_state_reply(const StateReply& state);

enum class FrameKind { fresh, resend, restart };

class FrameTracker {
public:
    // fresh: a larger frame_count than the last (a gap is counted in skipped()); resend: the same; restart: a smaller one.
    [[nodiscard]] FrameKind observe(uint32_t frame_count);
    [[nodiscard]] uint64_t skipped() const { return skipped_; }

private:
    std::optional<uint32_t> last_;
    uint64_t skipped_ = 0;
};

class ReplyClock {
public:
    // The next reply's timestamp. Refused (and nothing changes) when dt = timestamp - previous is not in (0, kMaxReplyStepS), except dt == 0 for a resend. The first
    // reply, and the first after reset(), has no previous and is accepted.
    [[nodiscard]] Result<void> accept(double timestamp_s, bool resend = false);
    void reset() { last_.reset(); }

private:
    std::optional<double> last_;
};

}  // namespace spade::fc_link::ardupilot_json
