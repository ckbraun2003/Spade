#include "tcp.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace spade::fc_link {

namespace {

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
void set_nonblocking(Sock s, bool on) {
    u_long v = on ? 1 : 0;
    ioctlsocket(s, FIONBIO, &v);
}
// Winsock needs WSAStartup before any call; once per process.
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
void set_nonblocking(Sock s, bool on) {
    const int f = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, on ? (f | O_NONBLOCK) : (f & ~O_NONBLOCK));
}
bool winsock_ready() { return true; }
#endif

Error io_error(const std::string& what) { return Error{Code::io_error, what + " (os error " + std::to_string(last_error()) + ")"}; }

void set_nodelay(Sock s) {
    const int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));
}

int ms_of(std::chrono::milliseconds t) { return static_cast<int>(std::clamp<int64_t>(t.count(), 0, 1000000)); }

class TcpChannel final : public ByteChannel {
public:
    explicit TcpChannel(Sock s) : s_(s) {}
    ~TcpChannel() override {
        if (s_ != kInvalid) close_socket(s_);
    }
    TcpChannel(const TcpChannel&) = delete;
    TcpChannel& operator=(const TcpChannel&) = delete;

    Result<void> write(std::span<const uint8_t> bytes) override {
        size_t sent = 0;
        while (sent < bytes.size()) {
#ifdef _WIN32
            const int n = ::send(s_, reinterpret_cast<const char*>(bytes.data() + sent), static_cast<int>(std::min<size_t>(bytes.size() - sent, 1 << 20)), 0);
#else
            const ssize_t n = ::send(s_, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
#endif
            if (n < 0) {
#ifndef _WIN32
                if (errno == EINTR) continue;
#endif
                return std::unexpected(io_error("tcp send failed"));
            }
            sent += static_cast<size_t>(n);
        }
        return {};
    }

    Result<std::vector<uint8_t>> read_some(std::chrono::milliseconds timeout) override {
        const int ready = poll_one(s_, POLLIN, ms_of(timeout));
        if (ready < 0) return std::unexpected(io_error("tcp poll failed"));
        if (ready == 0) return std::vector<uint8_t>{};
        std::vector<uint8_t> buf(65536);
#ifdef _WIN32
        const int n = ::recv(s_, reinterpret_cast<char*>(buf.data()), static_cast<int>(buf.size()), 0);
#else
        const ssize_t n = ::recv(s_, buf.data(), buf.size(), 0);
#endif
        if (n == 0) return std::unexpected(Error{Code::io_error, "tcp peer closed the connection"});
        if (n < 0) return std::unexpected(io_error("tcp recv failed"));
        buf.resize(static_cast<size_t>(n));
        return buf;
    }

private:
    Sock s_;
};

}  // namespace

Result<TcpListener> TcpListener::listen(uint16_t port, bool any_interface) {
    if (!winsock_ready()) return std::unexpected(Error{Code::io_error, "Winsock did not start"});
    const Sock s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s == kInvalid) return std::unexpected(io_error("tcp socket failed"));
#ifndef _WIN32
    const int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));  // not on Windows, where it would let another process share the port
#endif
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(any_interface ? INADDR_ANY : INADDR_LOOPBACK);
    if (::bind(s, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) {
        Error e = io_error("tcp bind failed on port " + std::to_string(port));
        close_socket(s);
        return std::unexpected(e);
    }
    if (::listen(s, 4) != 0) {
        Error e = io_error("tcp listen failed");
        close_socket(s);
        return std::unexpected(e);
    }
    sockaddr_in got{};
#ifdef _WIN32
    int len = sizeof(got);
#else
    socklen_t len = sizeof(got);
#endif
    if (getsockname(s, reinterpret_cast<sockaddr*>(&got), &len) != 0) {
        Error e = io_error("tcp getsockname failed");
        close_socket(s);
        return std::unexpected(e);
    }
    return TcpListener(static_cast<intptr_t>(s), ntohs(got.sin_port));
}

Result<std::unique_ptr<ByteChannel>> TcpListener::accept(std::chrono::milliseconds timeout) {
    const Sock s = static_cast<Sock>(socket_);
    const int ready = poll_one(s, POLLIN, ms_of(timeout));
    if (ready < 0) return std::unexpected(io_error("tcp poll failed"));
    if (ready == 0)
        return std::unexpected(Error{Code::io_error, "no connection on port " + std::to_string(port_) + " within " + std::to_string(timeout.count()) + " ms"});
    const Sock c = ::accept(s, nullptr, nullptr);
    if (c == kInvalid) return std::unexpected(io_error("tcp accept failed"));
    set_nodelay(c);
    return std::unique_ptr<ByteChannel>(new TcpChannel(c));
}

TcpListener::TcpListener(TcpListener&& o) noexcept : socket_(o.socket_), port_(o.port_) { o.socket_ = -1; }
TcpListener& TcpListener::operator=(TcpListener&& o) noexcept {
    if (this != &o) {
        if (socket_ != -1) close_socket(static_cast<Sock>(socket_));
        socket_ = o.socket_;
        port_ = o.port_;
        o.socket_ = -1;
    }
    return *this;
}
TcpListener::~TcpListener() {
    if (socket_ != -1) close_socket(static_cast<Sock>(socket_));
}

Result<std::unique_ptr<ByteChannel>> tcp_connect(const std::string& host, uint16_t port, std::chrono::milliseconds timeout) {
    if (!winsock_ready()) return std::unexpected(Error{Code::io_error, "Winsock did not start"});
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 || !res)
        return std::unexpected(Error{Code::io_error, "cannot resolve " + host});
    const Sock s = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s == kInvalid) {
        freeaddrinfo(res);
        return std::unexpected(io_error("tcp socket failed"));
    }
    set_nonblocking(s, true);
    const int rc = ::connect(s, res->ai_addr, static_cast<int>(res->ai_addrlen));
    freeaddrinfo(res);
    if (rc != 0) {
        const int ready = poll_one(s, POLLOUT, ms_of(timeout));
        int err = 0;
#ifdef _WIN32
        int len = sizeof(err);
#else
        socklen_t len = sizeof(err);
#endif
        if (ready > 0) getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&err), &len);
        if (ready <= 0 || err != 0) {
            close_socket(s);
            return std::unexpected(Error{Code::io_error, "tcp connect to " + host + ":" + std::to_string(port) + (ready == 0 ? " timed out" : " failed (os error " + std::to_string(err) + ")")});
        }
    }
    set_nonblocking(s, false);
    set_nodelay(s);
    return std::unique_ptr<ByteChannel>(new TcpChannel(s));
}

}  // namespace spade::fc_link
