#include "fc_link/ardupilot_link.hpp"

namespace spade::fc_link {

// RED: create() accepts anything and respond() is not written yet.
Result<ArduPilotLink> ArduPilotLink::create(const ArduPilotConfig& config) { return ArduPilotLink(config); }

Result<ArduPilotStep> ArduPilotLink::respond(fc_host::LockstepHost&, const ardupilot_json::ServoPacket&) {
    return std::unexpected(Error{Code::unavailable, "ArduPilotLink::respond: not written yet"});
}

}  // namespace spade::fc_link
