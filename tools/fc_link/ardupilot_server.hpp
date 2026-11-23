#pragma once
// ---------------------------------------------------------------------------
// fc_link/ardupilot_server.hpp -- the UDP loop of the ArduPilot link (docs/design/interface/plans/2026-11-21-fc-links.md sections 6 and 15): one servo packet in
// from ArduPilot, one JSON line out. Spade is the server: the channel (udp.hpp's UdpChannel, or any ByteChannel) learns ArduPilot's address from the first
// datagram and every reply goes back there. One datagram is one packet and one reply; the loop is the caller's (it runs serve_one until its own end).
//
// serve_one(host): wait at most `packet_timeout` of WALL time for a datagram (a stall detector, as the PX4 link's reply timeout: nothing here reads a clock into
// Spade's state), decode it (ardupilot_json), let ArduPilotLink::respond step the world, encode the state, write it. Every refusal names what and writes nothing:
// no packet in time (code io_error, "ardupilot_no_packet"), a datagram that is not a servo packet (the decoder's refusal), the link's own refusals (a frame
// period that is not the IMU's, a restart), a closed channel.
//
// Bound on 127.0.0.1 by UdpChannel: ArduPilot in a container reaches it only through a published or forwarded address, which is the phase that runs ArduPilot.
// ---------------------------------------------------------------------------
#include <chrono>
#include <cstdint>

#include "core/error.hpp"
#include "fc_host/lockstep.hpp"
#include "fc_link/ardupilot_link.hpp"
#include "fc_link/px4_link.hpp"  // ByteChannel

namespace spade::fc_link {

struct ArduPilotServeReport {
    uint64_t packets = 0;    // servo packets decoded
    uint64_t replies = 0;    // replies written
    uint64_t resends = 0;    // packets that repeated the last frame_count (answered again, the world not stepped)
    uint64_t bytes_in = 0;
    uint64_t bytes_out = 0;
    uint32_t last_frame_count = 0;
};

class ArduPilotServer {
public:
    // `channel` must outlive the server. A packet_timeout of zero or less is refused.
    [[nodiscard]] static Result<ArduPilotServer> create(ByteChannel& channel, ArduPilotLink link, std::chrono::milliseconds packet_timeout);

    [[nodiscard]] Result<ArduPilotStep> serve_one(fc_host::LockstepHost& host);
    [[nodiscard]] const ArduPilotServeReport& report() const { return report_; }

private:
    ArduPilotServer(ByteChannel& channel, ArduPilotLink link, std::chrono::milliseconds timeout) : channel_(&channel), link_(std::move(link)), timeout_(timeout) {}

    ByteChannel* channel_;
    ArduPilotLink link_;
    std::chrono::milliseconds timeout_;
    ArduPilotServeReport report_;
};

}  // namespace spade::fc_link
