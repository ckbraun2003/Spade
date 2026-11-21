#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "core/error.hpp"
#include "px4_link.hpp"

// ---------------------------------------------------------------------------
// fc_link/tcp.hpp -- the real byte channel under Px4Link: a TCP listener (Spade is the server; PX4 SITL connects to it, docs/design/
// interface/plans/2026-11-21-fc-links.md section 1.2) and a TCP client (the test peers, and a simulator that must dial out).
//
// Winsock on Windows, BSD sockets elsewhere. Blocking sockets with a poll() for every wait, so every wait has a timeout and nothing here
// can hang. TCP_NODELAY on every connection: a lockstep exchange is one small message each way per period, and Nagle would add its delay
// to every one. Nothing in this file reads a clock into Spade's state; the timeouts are I/O waits only.
// ---------------------------------------------------------------------------

namespace spade::fc_link {

class TcpListener {
public:
    // Binds `port` (0 = the system picks one; port() says which) on the loopback interface, or on every interface when `any_interface`
    // (needed only when the autopilot is in a container that reaches the host through a bridge: the caller says so explicitly, because
    // that exposes the port beyond this machine).
    [[nodiscard]] static Result<TcpListener> listen(uint16_t port, bool any_interface = false);

    [[nodiscard]] uint16_t port() const { return port_; }

    // The next connection, waiting at most `timeout`; an error named "no connection" when none came.
    [[nodiscard]] Result<std::unique_ptr<ByteChannel>> accept(std::chrono::milliseconds timeout);

    TcpListener(TcpListener&&) noexcept;
    TcpListener& operator=(TcpListener&&) noexcept;
    TcpListener(const TcpListener&) = delete;
    TcpListener& operator=(const TcpListener&) = delete;
    ~TcpListener();

private:
    TcpListener(intptr_t socket, uint16_t port) : socket_(socket), port_(port) {}
    intptr_t socket_ = -1;
    uint16_t port_ = 0;
};

// A client connection (blocking connect bounded by `timeout`).
[[nodiscard]] Result<std::unique_ptr<ByteChannel>> tcp_connect(const std::string& host, uint16_t port, std::chrono::milliseconds timeout);

}  // namespace spade::fc_link
