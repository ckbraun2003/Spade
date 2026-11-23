#include "udp.hpp"

#include <algorithm>
#include <cerrno>
#include <string>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace spade::fc_link {

namespace {

// The platform seams are the same few lines as tcp.cpp's, kept here so the two transports stay independent files.
#ifdef _WIN32
using Sock = SOCKET;
constexpr Sock kInvalid = INVALID_SOCKET;
int last_error() { return WSAGetLastError(); }
void close_socket(Sock s) { closesocket(s); }
int poll_one(Sock s, short events, int timeout_ms) {
    WSAPOLLFD p{};
    p.fd = s;
    p.events = events;
    const int r = WSAPoll(&p, 1, timeout_ms);
    if (r <= 0) return r;
    return (p.revents & (events | POLLERR | POLLHUP | POLLNVAL)) ? 1 : 0;
}
struct WsaGuard {
    WsaGuard() {
        WSADATA d;
        ok = WSAStartup(MAKEWORD(2, 2), &d) == 0;
    }
    ~WsaGuard() {
        if (ok) WSACleanup();
    }
    bool ok = false;
};
bool winsock_ready() {
    static const WsaGuard g;
    return g.ok;
}
// Windows reports an earlier datagram's ICMP "port unreachable" as an error on the NEXT receive (WSAECONNRESET), which would read as a
// broken channel when the peer simply was not listening yet. UDP has no connection to reset: turn it off.
void ignore_connection_reset(Sock s) {
    BOOL off = FALSE;
    DWORD returned = 0;
    WSAIoctl(s, SIO_UDP_CONNRESET, &off, sizeof(off), nullptr, 0, &returned, nullptr, nullptr);
}
#else
using Sock = int;
constexpr Sock kInvalid = -1;
int last_error() { return errno; }
void close_socket(Sock s) { ::close(s); }
int poll_one(Sock s, short events, int timeout_ms) {
    pollfd p{};
    p.fd = s;
    p.events = events;
    int r;
    do {
        r = ::poll(&p, 1, timeout_ms);
    } while (r < 0 && errno == EINTR);
    if (r <= 0) return r;
    return (p.revents & (events | POLLERR | POLLHUP | POLLNVAL)) ? 1 : 0;
}
bool winsock_ready() { return true; }
void ignore_connection_reset(Sock) {}
#endif

Error io_error(const std::string& what) { return Error{Code::io_error, what + " (os error " + std::to_string(last_error()) + ")"}; }

int ms_of(std::chrono::milliseconds t) { return static_cast<int>(std::clamp<int64_t>(t.count(), 0, 1000000)); }

constexpr size_t kMaxDatagram = 2048;  // a MAVLink v2 frame is at most 280 bytes

sockaddr_in loopback(uint16_t port) {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return a;
}

}  // namespace

Result<std::unique_ptr<UdpChannel>> UdpChannel::open(uint16_t local_port, uint16_t peer_port) {
    if (!winsock_ready()) return std::unexpected(Error{Code::io_error, "udp: Winsock did not start"});
    const Sock s = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (s == kInvalid) return std::unexpected(io_error("udp socket failed"));
    const sockaddr_in a = loopback(local_port);  // 127.0.0.1 only: this link never leaves the machine
    if (::bind(s, reinterpret_cast<const sockaddr*>(&a), sizeof(a)) != 0) {
        Error e = io_error("udp bind failed on port " + std::to_string(local_port));
        close_socket(s);
        return std::unexpected(e);
    }
    ignore_connection_reset(s);
    sockaddr_in got{};
#ifdef _WIN32
    int len = sizeof(got);
#else
    socklen_t len = sizeof(got);
#endif
    if (getsockname(s, reinterpret_cast<sockaddr*>(&got), &len) != 0) {
        Error e = io_error("udp getsockname failed");
        close_socket(s);
        return std::unexpected(e);
    }
    return std::unique_ptr<UdpChannel>(new UdpChannel(static_cast<intptr_t>(s), ntohs(got.sin_port), peer_port));
}

Result<void> UdpChannel::write(std::span<const uint8_t> bytes) {
    if (bytes.empty()) return std::unexpected(Error{Code::invalid_argument, "udp: nothing to send"});
    if (peer_port_ == 0) return std::unexpected(Error{Code::invalid_argument, "udp: no peer known yet (none given, and nothing heard)"});
    const sockaddr_in to = loopback(peer_port_);
    const int n = static_cast<int>(::sendto(static_cast<Sock>(socket_), reinterpret_cast<const char*>(bytes.data()), static_cast<int>(bytes.size()), 0,
                                            reinterpret_cast<const sockaddr*>(&to), sizeof(to)));
    if (n != static_cast<int>(bytes.size())) return std::unexpected(io_error("udp send failed"));
    return {};
}

Result<std::vector<uint8_t>> UdpChannel::read_some(std::chrono::milliseconds timeout) {
    const Sock s = static_cast<Sock>(socket_);
    const int ready = poll_one(s, POLLIN, ms_of(timeout));
    if (ready < 0) return std::unexpected(io_error("udp poll failed"));
    if (ready == 0) return std::vector<uint8_t>{};
    std::vector<uint8_t> buf(kMaxDatagram);
    sockaddr_in from{};
#ifdef _WIN32
    int len = sizeof(from);
#else
    socklen_t len = sizeof(from);
#endif
    const int n = static_cast<int>(::recvfrom(s, reinterpret_cast<char*>(buf.data()), static_cast<int>(buf.size()), 0, reinterpret_cast<sockaddr*>(&from), &len));
    if (n < 0) return std::unexpected(io_error("udp receive failed"));
    buf.resize(static_cast<size_t>(n));
    if (peer_port_ == 0) peer_port_ = ntohs(from.sin_port);  // the first datagram heard names the peer
    return buf;
}

UdpChannel::~UdpChannel() {
    if (socket_ != -1) close_socket(static_cast<Sock>(socket_));
}

}  // namespace spade::fc_link
