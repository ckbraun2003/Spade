#include "fc_link/mavlink.hpp"

// RED stub: every entry point compiles and returns nothing useful; the tests in tests/test_fc_link_mavlink.cpp fail against it.
namespace spade::fc_link::mavlink {

uint16_t crc16(std::span<const uint8_t>, uint16_t) { return 0; }
std::optional<uint8_t> crc_extra(uint32_t) { return std::nullopt; }
std::optional<size_t> max_payload_len(uint32_t) { return std::nullopt; }
uint32_t msgid_of(const Message&) { return 0xFFFFFFFFu; }
std::vector<uint8_t> encode(const Header&, const Message&) { return {}; }
Result<Message> decode(const Frame&) { return std::unexpected(Error{Code::internal, "stub"}); }
void Parser::feed(std::span<const uint8_t>) {}
std::optional<Frame> Parser::next() { return std::nullopt; }

}  // namespace spade::fc_link::mavlink
