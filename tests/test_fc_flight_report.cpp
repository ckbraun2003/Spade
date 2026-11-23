// What flight-report.txt says (tests/fc_flight_report.hpp), checked without PX4 or a world: the fields a stalled run needs to be read from its own numbers.
#include <gtest/gtest.h>

#include <string>

#include "fc_flight_report.hpp"

namespace {

using namespace fc_flight_test;

bool has_line(const std::string& text, const std::string& line) {
    std::size_t at = 0;
    while (at <= text.size()) {
        const std::size_t end = text.find('\n', at);
        const std::string one = text.substr(at, end == std::string::npos ? std::string::npos : end - at);
        if (one == line) return true;
        if (end == std::string::npos) break;
        at = end + 1;
    }
    return false;
}

FlightRunFacts stalled_run() {
    FlightRunFacts f;
    f.mission_phase = "boot";
    f.sim_s = 1.292;
    f.wall_s = 42.65;
    f.boot_steps = 51;
    f.first_datagram_wall_s = 31.4;
    f.px4_no_reply = 1;
    f.first_refusal = "px4_no_reply: no HIL_ACTUATOR_CONTROLS within 5000 ms";
    f.datagrams_back = 88;
    f.link.sensor_messages = 375;
    f.link.replies = 374;
    f.link.last_time_usec = 1'500'000;
    f.link.last_reply_time_usec = 1'496'000;
    f.link.wall_wait_total = std::chrono::milliseconds(37'000);
    f.link.wall_wait_max = std::chrono::milliseconds(5'000);
    f.link.wall_send_max = std::chrono::milliseconds(3);
    f.exchanges = 375;
    f.wait_total_ms = 40'000.0;
    f.wait_max_ms = 5'001.5;
    f.wait_max_at_sim_s = 1.5;
    f.wait_max_at_wall_s = 42.6;
    f.gap_total_ms = 2'000.0;
    f.gap_max_ms = 3'210.0;
    f.gap_max_at_sim_s = 0.012;
    f.gap_max_at_wall_s = 6.5;
    f.slow.push_back({3, 0.012, 6.5, 12.0, 3'210.0, true});
    f.slow.push_back({375, 1.5, 42.6, 5'001.5, 2.0, false});
    return f;
}

TEST(FcFlightReport, ATerminalStateAndTheCountersKeepTheirLines) {
    const std::string t = format_flight_report(stalled_run());
    EXPECT_TRUE(has_line(t, "mission_phase: boot")) << t;
    EXPECT_TRUE(has_line(t, "mission_passed: no")) << t;
    EXPECT_TRUE(has_line(t, "boot_steps: 51")) << t;
    EXPECT_TRUE(has_line(t, "px4_no_reply: 1")) << t;
    EXPECT_TRUE(has_line(t, "datagrams_back: 88")) << t;
    EXPECT_TRUE(has_line(t, "first_refusal: px4_no_reply: no HIL_ACTUATOR_CONTROLS within 5000 ms")) << t;
}

TEST(FcFlightReport, TheLinksLastSentAndLastAcceptedTimesAreThereToTellWhichWayTheLastMessageWent) {
    const std::string t = format_flight_report(stalled_run());
    EXPECT_TRUE(has_line(t, "link_sensor_messages: 375")) << t;
    EXPECT_TRUE(has_line(t, "link_replies: 374")) << t;
    EXPECT_TRUE(has_line(t, "link_last_time_usec: 1500000")) << t;
    EXPECT_TRUE(has_line(t, "link_last_reply_time_usec: 1496000")) << t;
    EXPECT_TRUE(has_line(t, "link_wall_wait_max_ms: 5000")) << t;
    EXPECT_TRUE(has_line(t, "link_wall_send_max_ms: 3")) << t;
}

TEST(FcFlightReport, TheFirstDatagramsWallTimeAndTheExchangeTimingAreThere) {
    const std::string t = format_flight_report(stalled_run());
    EXPECT_TRUE(has_line(t, "first_datagram_wall_s: 31.4")) << t;
    EXPECT_TRUE(has_line(t, "exchanges: 375")) << t;
    EXPECT_TRUE(has_line(t, "wait_max_ms: 5001.5 at sim 1.5 s, wall 42.6 s")) << t;
    EXPECT_TRUE(has_line(t, "wait_mean_ms: 107")) << "40000 ms over 375 exchanges, to a whole millisecond: " << t;
    EXPECT_TRUE(has_line(t, "host_gap_max_ms: 3210 at sim 0.012 s, wall 6.5 s")) << t;
    EXPECT_TRUE(has_line(t, "host_gap_mean_ms: 5")) << "2000 ms over 375 exchanges: " << t;
}

TEST(FcFlightReport, NoDatagramAtAllIsSaidNotPrintedAsANumber) {
    FlightRunFacts f = stalled_run();
    f.first_datagram_wall_s = -1.0;
    const std::string t = format_flight_report(f);
    EXPECT_TRUE(has_line(t, "first_datagram_wall_s: none")) << t;
}

TEST(FcFlightReport, EveryExchangeOverTheThresholdIsListedWithItsPhaseAndTheListIsCapped) {
    FlightRunFacts f = stalled_run();
    std::string t = format_flight_report(f);
    EXPECT_TRUE(has_line(t, "slow_exchange 3: boot, sim 0.012 s, wall 6.5 s, waited 12 ms, host gap before 3210 ms")) << "a long HOST GAP is the host's: " << t;
    EXPECT_TRUE(has_line(t, "slow_exchange 375: strict, sim 1.5 s, wall 42.6 s, waited 5001.5 ms, host gap before 2 ms")) << "a long WAIT is the path's or PX4's: " << t;
    f.slow.clear();
    for (uint64_t i = 1; i <= kSlowLinesMax + 7; ++i) f.slow.push_back({i, 0.0, 1.0, 200.0, 1.0, false});
    t = format_flight_report(f);
    EXPECT_TRUE(has_line(t, "slow_exchange " + std::to_string(kSlowLinesMax) + ": strict, sim 0 s, wall 1 s, waited 200 ms, host gap before 1 ms")) << t;
    EXPECT_FALSE(has_line(t, "slow_exchange " + std::to_string(kSlowLinesMax + 1) + ": strict, sim 0 s, wall 1 s, waited 200 ms, host gap before 1 ms")) << t;
    EXPECT_TRUE(has_line(t, "slow_exchanges_not_listed: 7")) << t;
    EXPECT_TRUE(has_line(t, "slow_exchanges: " + std::to_string(kSlowLinesMax + 7))) << t;
}

}  // namespace
