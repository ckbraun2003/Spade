#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "core/error.hpp"
#include "px4_link.hpp"

// ---------------------------------------------------------------------------
// fc_link/udp.hpp -- the companion (offboard) link's byte channel: one UDP socket on this machine's loopback (docs/design/interface/plans/
// 2026-11-21-fc-links.md section 13). PX4 takes arm, take-off and land as MAVLink commands over UDP, not over the lockstep TCP link.
//
// A datagram is a unit: one write is one datagram and one read_some is ONE received datagram (never two joined, never one split), so a frame
// is never cut by the transport. The channel has a peer address: given at open, or learned from the first datagram received (PX4 answers
// the address it hears from). Until it has one, write refuses by name. Every wait is a poll with a timeout, as in tcp.hpp; nothing here reads a
// clock into Spade's state.
// ---------------------------------------------------------------------------

namespace spade::fc_link {

// Binds `local_port` (0 = the system picks; the returned channel's local_port() says which) on 127.0.0.1. With `peer_port` non-zero the peer is
// 127.0.0.1:peer_port from the start; with 0 it is learned from the first datagram received.
class UdpChannel : public ByteChannel {
public:
    [[nodiscard]] static Result<std::unique_ptr<UdpChannel>> open(uint16_t local_port, uint16_t peer_port = 0);

    [[nodiscard]] uint16_t local_port() const { return local_port_; }
    // The peer's port, or 0 while none is known.
    [[nodiscard]] uint16_t peer_port() const { return peer_port_; }

    [[nodiscard]] Result<void> write(std::span<const uint8_t> bytes) override;
    [[nodiscard]] Result<std::vector<uint8_t>> read_some(std::chrono::milliseconds timeout) override;

    ~UdpChannel() override;
    UdpChannel(const UdpChannel&) = delete;
    UdpChannel& operator=(const UdpChannel&) = delete;

private:
    UdpChannel(intptr_t socket, uint16_t local_port, uint16_t peer_port) : socket_(socket), local_port_(local_port), peer_port_(peer_port) {}
    intptr_t socket_ = -1;
    uint16_t local_port_ = 0;
    uint16_t peer_port_ = 0;
};

}  // namespace spade::fc_link
