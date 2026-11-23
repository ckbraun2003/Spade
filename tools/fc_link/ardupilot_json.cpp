#include "ardupilot_json.hpp"

namespace spade::fc_link::ardupilot_json {

// RED: not written yet.
Result<ServoPacket> decode_servo_packet(std::span<const uint8_t>) { return std::unexpected(Error{Code::unavailable, "decode_servo_packet: not written yet"}); }
Result<std::string> encode_state_reply(const StateReply&) { return std::unexpected(Error{Code::unavailable, "encode_state_reply: not written yet"}); }
FrameKind FrameTracker::observe(uint32_t) { return FrameKind::fresh; }
Result<void> ReplyClock::accept(double, bool) { return {}; }

}  // namespace spade::fc_link::ardupilot_json
