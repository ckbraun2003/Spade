#include "udp.hpp"

namespace spade::fc_link {

// RED: not written yet; every call refuses by name.
Result<std::unique_ptr<UdpChannel>> UdpChannel::open(uint16_t, uint16_t) { return std::unexpected(Error{Code::io_error, "udp: not written"}); }
Result<void> UdpChannel::write(std::span<const uint8_t>) { return std::unexpected(Error{Code::io_error, "udp: not written"}); }
Result<std::vector<uint8_t>> UdpChannel::read_some(std::chrono::milliseconds) { return std::unexpected(Error{Code::io_error, "udp: not written"}); }
UdpChannel::~UdpChannel() = default;

}  // namespace spade::fc_link
