#pragma once
// ---------------------------------------------------------------------------
// fc_flight_report.hpp -- the text of flight-report.txt that the live flight test (test_fc_live_flight.cpp) leaves in its run folder, apart from the test so that
// what a report says can be tested without PX4 (test_fc_flight_report.cpp).
//
// After a stall the report must answer, from its own numbers, which end paused and which way the last message went: the exchange count and the wall time of
// the slowest (and when), every exchange over slow_exchange_ms listed with its phase, the time of the first datagram from PX4, and the link's own last_time_usec
// (the last sensor message sent) against last_reply_time_usec (the last reply accepted).
// ---------------------------------------------------------------------------
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "fc_link/flight_mission.hpp"
#include "fc_link/px4_link.hpp"

namespace fc_flight_test {

inline constexpr double kSlowExchangeMs = 100.0;  // an exchange whose wait or host gap is over this wall time is listed
inline constexpr std::size_t kSlowLinesMax = 40;  // the report lists at most this many, then says how many more there were

// An exchange with a long WAIT (its send to its reply accepted: the path or PX4) or a long HOST GAP (the previous reply accepted to its send: the host).
struct SlowExchange {
    uint64_t index = 0;    // the exchange's number, from 1
    double sim_s = 0.0;    // simulated time when it ended
    double wall_s = 0.0;   // wall time since the loop began, when it ended
    double wait_ms = 0.0;  // its wait
    double gap_ms = 0.0;   // the host gap before it
    bool boot = false;     // before PX4's first reply
};

struct AckLine {
    double t = 0.0;
    uint16_t command = 0;
    uint8_t result = 0;
};

struct FlightRunFacts {
    std::string mission_phase;
    bool mission_passed = false;
    bool last_armed = false;
    double last_altitude_m = 0.0;
    bool landed = false;
    std::string abort_outcome;
    double sim_s = 0.0;
    double wall_s = 0.0;
    uint64_t boot_steps = 0;
    double first_datagram_wall_s = -1.0;  // wall time since the loop began of the first datagram from PX4 on the companion link; negative: none arrived
    uint64_t px4_no_reply = 0, px4_stale_reply = 0, px4_link_closed = 0, other_refusals = 0, send_failures = 0, datagrams_back = 0;
    std::string first_refusal;
    spade::fc_link::Px4LinkReport link{};
    uint64_t exchanges = 0;
    double wait_total_ms = 0.0, wait_max_ms = 0.0, wait_max_at_sim_s = 0.0, wait_max_at_wall_s = 0.0;
    double gap_total_ms = 0.0, gap_max_ms = 0.0, gap_max_at_sim_s = 0.0, gap_max_at_wall_s = 0.0;
    std::vector<SlowExchange> slow;
    std::vector<spade::fc_link::FlightCheck> journal;
    std::vector<std::string> sent_log;
    std::vector<AckLine> acks;
    std::map<uint32_t, uint64_t> by_msgid;
};

// The report text. RED: not written yet.
[[nodiscard]] inline std::string format_flight_report(const FlightRunFacts&) { return std::string(); }

}  // namespace fc_flight_test
