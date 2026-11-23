#include "flight_mission.hpp"

namespace spade::fc_link {

// RED: the mission does nothing yet.
std::vector<mavlink::CommandLong> FlightMission::step(const FlightObservation&) { return {}; }

}  // namespace spade::fc_link
