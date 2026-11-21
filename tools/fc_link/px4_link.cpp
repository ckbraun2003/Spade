#include "fc_link/px4_link.hpp"

// RED stub: every entry point refuses; tests/test_fc_link_px4.cpp fails against it.
namespace spade::fc_link {

Result<Px4Link> Px4Link::open(std::unique_ptr<ByteChannel>, const Px4LinkConfig&) { return std::unexpected(Error{Code::internal, "stub"}); }
Result<ActuatorReply> Px4Link::exchange(const mavlink::HilSensor&, const std::optional<mavlink::HilGps>&) {
    return std::unexpected(Error{Code::internal, "stub"});
}
Result<void> Px4Link::fill(std::chrono::milliseconds) { return std::unexpected(Error{Code::internal, "stub"}); }
Px4Link::Taken Px4Link::take() { return {}; }

}  // namespace spade::fc_link
