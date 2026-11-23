#pragma once

// ---------------------------------------------------------------------------
// fc_link/companion.hpp -- what the host says to PX4 on the companion (offboard) link, apart from the lockstep link (docs/design/interface/
// plans/2026-11-21-fc-links.md section 13, the flight film). PX4 takes arm, take-off and land as MAVLink commands on its UDP API, and it wants to
// hear a HEARTBEAT from the peer about once a second.
//
//   HeartbeatSchedule  when a heartbeat is due, in SIMULATED time (the autopilot runs on the host's clock in lockstep, so a wall-clock second
//                      would be a different number of its seconds on every run: this keeps the run deterministic and replayable).
//   companion_heartbeat()  the heartbeat itself: a ground-control-station type, no autopilot, active.
// ---------------------------------------------------------------------------

#include <cstdint>

#include "fc_link/mavlink.hpp"

namespace spade::fc_link {

class HeartbeatSchedule {
public:
    explicit HeartbeatSchedule(uint64_t period_usec = 1000000u) : period_usec_(period_usec) {}

    // True when a heartbeat is due at simulated time `time_usec`: the first call, then once each period. A time that jumps far ahead is one
    // heartbeat, not a catch-up of the ones in between; a time that has not advanced past the last due one is never due.
    [[nodiscard]] bool due(uint64_t time_usec) {
        (void)time_usec;
        return false;  // RED: nothing is scheduled yet
    }

private:
    uint64_t period_usec_;
    bool started_ = false;
    uint64_t next_usec_ = 0;
};

// MAV_TYPE_GCS (6), MAV_AUTOPILOT_INVALID (8), no base mode, MAV_STATE_ACTIVE (4), MAVLink version 3.
[[nodiscard]] inline mavlink::Heartbeat companion_heartbeat() { return mavlink::Heartbeat{}; }  // RED

}  // namespace spade::fc_link
