#include "fc_link/ardupilot_server.hpp"

namespace spade::fc_link {

// RED: create() accepts anything and serve_one() is not written yet.
Result<ArduPilotServer> ArduPilotServer::create(ByteChannel& channel, ArduPilotLink link, std::chrono::milliseconds packet_timeout) {
    return ArduPilotServer(channel, std::move(link), packet_timeout);
}

Result<ArduPilotStep> ArduPilotServer::serve_one(fc_host::LockstepHost&) {
    return std::unexpected(Error{Code::unavailable, "ArduPilotServer::serve_one: not written yet"});
}

}  // namespace spade::fc_link
