#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "fc_link/mavlink.hpp"

// ---------------------------------------------------------------------------
// fc_link/flight_mission.hpp -- the flight film's script as a state machine with no I/O (docs/design/interface/plans/2026-11-21-fc-links.md section
// 13): boot until PX4 is ready, arm, take off to a height, hover, land, wait for the disarm. Each tick it is given what was observed (simulated time,
// whether PX4 says it is armed, Spade's own altitude above the ground, the COMMAND_ACK that arrived) and it answers with the commands to send now.
// It judges every step against a band and writes each judgement into a journal (name, measured value, the limit, simulated time, pass or fail);
// the first failure ends the mission, and a failed take is not kept.
//
// The bands in FlightConfig are the film's; their numbers are chosen from the dry runs' measured values with a stated margin (the live test passes
// them in), so this file has no opinion about PX4: it only holds a mission to what it was told.
// ---------------------------------------------------------------------------

namespace spade::fc_link {

struct FlightConfig {
    double takeoff_height_m = 2.0;     // above the ground
    double home_amsl_m = 0.0;          // the ground's altitude above mean sea level: the takeoff command's param7 is AMSL
    double ready_s = 20.0;             // simulated seconds after boot before the first arm (PX4's estimator needs this to be ready)
    double arm_timeout_s = 10.0;       // armed within this long of the first arm command
    double arm_retry_s = 1.0;          // a temporarily rejected or unanswered arm is sent again after this long
    double takeoff_band_m = 0.2;       // the takeoff height +- this, reached within takeoff_timeout_s of the takeoff command
    double takeoff_timeout_s = 20.0;
    double hover_s = 10.0;             // how long the hover is held
    double hover_band_m = 0.1;         // the altitude stays within this of where the hover began
    double landed_height_m = 0.05;     // landed: below this and disarmed
    double land_timeout_s = 30.0;      // landed within this long of the land command
};

// What the autopilot answered to a command (MAV_RESULT: 0 accepted, 1 temporarily rejected, 2 denied, 3 unsupported, 4 failed).
struct AckObservation {
    uint16_t command = 0;
    uint8_t result = 0;
};

struct FlightObservation {
    double time_s = 0.0;               // simulated time
    bool armed = false;                // PX4's own lockstep reply
    double altitude_m = 0.0;           // Spade's altitude above the ground
    std::optional<AckObservation> ack; // the COMMAND_ACK that arrived this tick, if any
};

struct FlightCheck {
    std::string name;
    bool pass = false;
    double measured = 0.0;
    double limit = 0.0;
    double time_s = 0.0;
};

class FlightMission {
public:
    enum class Phase { boot, arming, takeoff, hover, land, done, failed };

    explicit FlightMission(const FlightConfig& config = {}) : config_(config) {}

    // The commands to send at this observation (none, one, or an arm repeat).
    [[nodiscard]] std::vector<mavlink::CommandLong> step(const FlightObservation& observation);

    [[nodiscard]] Phase phase() const { return phase_; }
    [[nodiscard]] bool finished() const { return phase_ == Phase::done || phase_ == Phase::failed; }
    [[nodiscard]] bool passed() const { return phase_ == Phase::done; }
    [[nodiscard]] const std::vector<FlightCheck>& journal() const { return journal_; }

private:
    FlightConfig config_;
    Phase phase_ = Phase::boot;
    std::vector<FlightCheck> journal_;
    double first_arm_s_ = 0.0, last_arm_s_ = 0.0, takeoff_s_ = 0.0, hover_start_s_ = 0.0, hover_ref_m_ = 0.0, max_dev_m_ = 0.0, land_s_ = 0.0;
};

}  // namespace spade::fc_link
