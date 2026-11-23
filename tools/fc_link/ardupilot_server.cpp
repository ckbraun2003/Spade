#include "fc_link/ardupilot_server.hpp"

#include <span>
#include <string>
#include <vector>

#include "fc_link/ardupilot_json.hpp"

namespace spade::fc_link {

Result<ArduPilotServer> ArduPilotServer::create(ByteChannel& channel, ArduPilotLink link, std::chrono::milliseconds packet_timeout) {
    if (packet_timeout.count() <= 0) {
        return std::unexpected(Error{Code::invalid_argument, "ArduPilotServer::create: the packet timeout must be positive, got " + std::to_string(packet_timeout.count()) + " ms"});
    }
    return ArduPilotServer(channel, std::move(link), packet_timeout);
}

Result<ArduPilotStep> ArduPilotServer::serve_one(fc_host::LockstepHost& host) {
    const Result<std::vector<uint8_t>> got = channel_->read_some(timeout_);
    if (!got) return std::unexpected(got.error());
    if (got->empty()) {
        return std::unexpected(Error{Code::io_error, "ardupilot_no_packet: no servo packet from ArduPilot within " + std::to_string(timeout_.count()) + " ms"});
    }
    report_.bytes_in += got->size();

    const Result<ardupilot_json::ServoPacket> packet = ardupilot_json::decode_servo_packet(*got);
    if (!packet) return std::unexpected(packet.error());
    ++report_.packets;

    const Result<ArduPilotStep> step = link_.respond(host, *packet);
    if (!step) return std::unexpected(step.error());
    const Result<std::string> text = ardupilot_json::encode_state_reply(step->state);
    if (!text) return std::unexpected(text.error());

    const std::span<const uint8_t> bytes(reinterpret_cast<const uint8_t*>(text->data()), text->size());
    const Result<void> written = channel_->write(bytes);
    if (!written) return std::unexpected(written.error());
    ++report_.replies;
    report_.bytes_out += text->size();
    report_.last_frame_count = packet->frame_count;
    if (step->kind == ardupilot_json::FrameKind::resend) ++report_.resends;
    return *step;
}

}  // namespace spade::fc_link
