#pragma once
// ---------------------------------------------------------------------------
// fc_flight_report.hpp -- the text of flight-report.txt that the live flight test (test_fc_live_flight.cpp) leaves in its run folder, apart from the test so that
// what a report says can be tested without PX4 (test_fc_flight_report.cpp).
//
// After a stall the report must answer, from its own numbers, which end paused and which way the last message went: the exchange count and the wall time of
// the slowest (and when), every exchange over slow_exchange_ms listed with its phase, the time of the first datagram from PX4, and the link's own last_time_usec
// (the last sensor message sent) against last_reply_time_usec (the last reply accepted).
// ---------------------------------------------------------------------------
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <sstream>
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

// The report text, one `key: value` per line.
[[nodiscard]] inline std::string format_flight_report(const FlightRunFacts& f) {
    std::ostringstream t;
    const auto yes = [](bool v) { return v ? "yes" : "no"; };
    const auto ms = [](std::chrono::nanoseconds ns) { return std::chrono::duration<double, std::milli>(ns).count(); };
    const auto mean = [&](double total) { return f.exchanges == 0 ? 0L : std::lround(total / static_cast<double>(f.exchanges)); };
    t << "mission_phase: " << f.mission_phase << "\n"
      << "mission_passed: " << yes(f.mission_passed) << "\n"
      << "last_armed: " << yes(f.last_armed) << "\n"
      << "last_altitude_m: " << f.last_altitude_m << "\n"
      << "landed: " << yes(f.landed) << "\n"
      << "abort_outcome: " << f.abort_outcome << "\n"
      << "sim_s: " << f.sim_s << "\n"
      << "wall_s: " << f.wall_s << "\n"
      << "boot_steps: " << f.boot_steps << "\n";
    if (f.first_datagram_wall_s < 0.0) t << "first_datagram_wall_s: none\n";
    else t << "first_datagram_wall_s: " << f.first_datagram_wall_s << "\n";
    t << "px4_no_reply: " << f.px4_no_reply << "\npx4_stale_reply: " << f.px4_stale_reply << "\npx4_link_closed: " << f.px4_link_closed
      << "\nother_refusals: " << f.other_refusals << "\nfirst_refusal: " << f.first_refusal << "\nsend_failures: " << f.send_failures
      << "\ndatagrams_back: " << f.datagrams_back << "\n";
    t << "link_sensor_messages: " << f.link.sensor_messages << "\nlink_replies: " << f.link.replies << "\nlink_last_time_usec: " << f.link.last_time_usec
      << "\nlink_last_reply_time_usec: " << f.link.last_reply_time_usec << "\nlink_wall_wait_max_ms: " << ms(f.link.wall_wait_max)
      << "\nlink_wall_send_max_ms: " << ms(f.link.wall_send_max) << "\nlink_wall_gap_max_ms: " << ms(f.link.wall_gap_max) << "\n";
    t << "exchanges: " << f.exchanges << "\n"
      << "wait_mean_ms: " << mean(f.wait_total_ms) << "\n"
      << "wait_max_ms: " << f.wait_max_ms << " at sim " << f.wait_max_at_sim_s << " s, wall " << f.wait_max_at_wall_s << " s\n"
      << "host_gap_mean_ms: " << mean(f.gap_total_ms) << "\n"
      << "host_gap_max_ms: " << f.gap_max_ms << " at sim " << f.gap_max_at_sim_s << " s, wall " << f.gap_max_at_wall_s << " s\n"
      << "slow_exchanges: " << f.slow.size() << "\n";
    for (std::size_t i = 0; i < f.slow.size() && i < kSlowLinesMax; ++i) {
        const SlowExchange& e = f.slow[i];
        t << "slow_exchange " << e.index << ": " << (e.boot ? "boot" : "strict") << ", sim " << e.sim_s << " s, wall " << e.wall_s << " s, waited " << e.wait_ms
          << " ms, host gap before " << e.gap_ms << " ms\n";
    }
    if (f.slow.size() > kSlowLinesMax) t << "slow_exchanges_not_listed: " << f.slow.size() - kSlowLinesMax << "\n";
    for (const spade::fc_link::FlightCheck& c : f.journal)
        t << "check " << c.name << ": " << (c.pass ? "PASS" : "FAIL") << " measured " << c.measured << " limit " << c.limit << " at " << c.time_s << " s\n";
    for (const std::string& s : f.sent_log) t << "sent " << s << "\n";
    for (const AckLine& a : f.acks) t << "ack at " << a.t << " s: command " << a.command << " result " << static_cast<int>(a.result) << "\n";
    for (const auto& [id, n] : f.by_msgid) t << "msgid_" << id << "_count: " << n << "\n";
    return t.str();
}

}  // namespace fc_flight_test
