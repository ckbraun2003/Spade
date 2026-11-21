#include "tcp.hpp"

// RED stub: every entry point refuses; tests/test_fc_link_tcp.cpp fails against it.
namespace spade::fc_link {

Result<TcpListener> TcpListener::listen(uint16_t, bool) { return std::unexpected(Error{Code::internal, "stub"}); }
Result<std::unique_ptr<ByteChannel>> TcpListener::accept(std::chrono::milliseconds) { return std::unexpected(Error{Code::internal, "stub"}); }
TcpListener::TcpListener(TcpListener&& o) noexcept : socket_(o.socket_), port_(o.port_) { o.socket_ = -1; }
TcpListener& TcpListener::operator=(TcpListener&& o) noexcept {
    socket_ = o.socket_;
    port_ = o.port_;
    o.socket_ = -1;
    return *this;
}
TcpListener::~TcpListener() = default;
Result<std::unique_ptr<ByteChannel>> tcp_connect(const std::string&, uint16_t, std::chrono::milliseconds) {
    return std::unexpected(Error{Code::internal, "stub"});
}

}  // namespace spade::fc_link
