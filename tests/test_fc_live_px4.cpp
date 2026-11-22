// ---------------------------------------------------------------------------
// test_fc_live_px4.cpp -- the first lockstep run against the real PX4 SITL image (O140 phase 1; docs/design/interface/plans/2026-11-21-fc-links.md).
//
// Label `fc`, run through scripts/fc-run.ps1 (see test_fc_live_harness.cpp): fc-run starts ONE container spade-px4-<run id> from scripts/px4-sitl.ps1, PX4
// connects out to this machine on TCP 4560, and this test is the simulator it dials. With no autopilot run (SPADE_FC_RUN_ID unset) it SKIPS by name.
//
// What it does: listens on 4560 (every interface: a container reaches the host through the Docker bridge, which the loopback does not serve; the port is
// open only for the length of the run), opens Px4Link, then steps a vehicle that sits still on the ground: gravity on the accelerometer, no rotation, a
// fixed magnetic field and sea-level pressure, a GPS fix every 200 ms of simulated time, at the 4 ms link step. It counts every refusal by its name, asks
// PX4's commander for its preflight verdict from outside the lockstep thread, and writes lockstep-report.txt into the run folder.
//
// It does not prove flight (nothing here moves); it proves the link: the lockstep advances, PX4 answers every sensor message exactly once, and PX4 reaches
// a state its own commander calls ready. SPADE_FC_STEPS (default 10000 = 40 s of simulated time) sets the length.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include "../tools/fc_link/mavlink.hpp"
#include "../tools/fc_link/px4_link.hpp"
#include "../tools/fc_link/tcp.hpp"

namespace {

using namespace spade::fc_link;
using namespace std::chrono_literals;

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)  // getenv
#endif
std::string env_or_empty(const char* name) {
    const char* v = std::getenv(name);
    return v != nullptr ? std::string(v) : std::string();
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif

// Runs the commander's own preflight check inside the container, away from the lockstep thread, and keeps what it printed.
int docker_exec_to_file(const std::string& container, const std::string& command, const std::filesystem::path& out) {
    const std::string line = "docker exec " + container + " " + command + " > \"" + out.string() + "\" 2>&1";
    return std::system(line.c_str());
}

std::string reason_of(const spade::Error& e) { return e.context; }

}  // namespace

TEST(FcLivePx4, TheLockstepRunsAgainstTheRealImageAndTheCommanderSaysReady) {
    const std::string run = env_or_empty("SPADE_FC_RUN_ID");
    if (run.empty()) GTEST_SKIP() << "no autopilot run: run the fc tests through scripts/fc-run.ps1";
    const std::string out_dir = env_or_empty("SPADE_FC_OUT_DIR");
    ASSERT_FALSE(out_dir.empty());
    const std::string steps_text = env_or_empty("SPADE_FC_STEPS");
    const uint64_t steps = steps_text.empty() ? 10000 : std::strtoull(steps_text.c_str(), nullptr, 10);
    const std::string container = "spade-px4-" + run;
    const std::filesystem::path out(out_dir);

    auto listener = TcpListener::listen(4560, /*any_interface=*/true);
    ASSERT_TRUE(listener.has_value()) << reason_of(listener.error());
    auto channel = listener->accept(120s);
    ASSERT_TRUE(channel.has_value()) << reason_of(channel.error());
    Px4LinkConfig cfg;
    cfg.reply_timeout = 5s;
    cfg.heartbeat_timeout = 60s;
    auto link = Px4Link::open(std::move(*channel), cfg);
    ASSERT_TRUE(link.has_value()) << reason_of(link.error());

    uint64_t no_reply = 0, stale_reply = 0, link_closed = 0, other_refusal = 0;
    std::string first_refusal;
    uint64_t done = 0;
    float max_motor = 0.F;
    bool ever_armed = false;
    std::future<int> commander;
    const auto wall0 = std::chrono::steady_clock::now();

    for (uint64_t k = 1; k <= steps; ++k) {
        mavlink::HilSensor s;
        s.time_usec = k * 4000;
        s.zacc = -9.80665F;                       // body at rest, FRD: gravity reads as -g on the down axis
        s.xmag = 0.21F, s.ymag = 0.F, s.zmag = 0.43F;  // gauss, heading north at about 47 degrees latitude
        s.abs_pressure = 955.F;                    // hPa, about 488 m
        s.pressure_alt = 488.F;
        s.temperature = 15.F;
        s.fields_updated = 0x1BFF;                 // accel, gyro, mag, baro; no differential pressure
        std::optional<mavlink::HilGps> gps;
        if (k % 50 == 0) {                         // every 200 ms
            mavlink::HilGps g;
            g.time_usec = s.time_usec;
            g.lat = 473977420;
            g.lon = 85455940;
            g.alt = 488000;
            g.eph = 100, g.epv = 100;
            g.cog = 65535;
            g.fix_type = 3, g.satellites_visible = 10;
            gps = g;
        }
        const auto r = link->exchange(s, gps);
        if (!r) {
            const std::string why = reason_of(r.error());
            if (first_refusal.empty()) first_refusal = why;
            if (why.find("px4_no_reply") != std::string::npos) ++no_reply;
            else if (why.find("px4_stale_reply") != std::string::npos) ++stale_reply;
            else if (why.find("px4_link_closed") != std::string::npos) ++link_closed;
            else ++other_refusal;
            break;  // a refused exchange ends the run: nothing is retried or guessed
        }
        ++done;
        for (int m = 0; m < 4; ++m) max_motor = std::max(max_motor, r->controls[m]);
        ever_armed = ever_armed || r->armed;
        if (k % 250 == 0 || k == steps) {  // a progress line for whoever watches the run (the run film tails this file)
            const double wall_now = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
            std::ofstream(out / "lockstep-progress.txt", std::ios::app)
                << "step " << k << " of " << steps << "   sim " << (static_cast<double>(k) * 0.004) << " s   wall " << wall_now << " s   replies " << link->report().replies
                << "   px4_no_reply 0   px4_stale_reply 0   px4_link_closed 0   motors armed " << (r->armed ? "yes" : "no") << "
";
        }
        if (k == (steps * 4) / 5) {
            commander = std::async(std::launch::async, [&] {
                return docker_exec_to_file(container, "/src/PX4-Autopilot/build/px4_sitl_spade/bin/px4-commander check", out / "commander-check.txt");
            });
        }
    }
    const auto wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    int commander_rc = -1;
    if (commander.valid()) commander_rc = commander.get();

    const auto& rep = link->report();
    std::ostringstream text;
    text << "steps_requested: " << steps << "\n"
         << "steps_done: " << done << "\n"
         << "sensor_messages: " << rep.sensor_messages << "\n"
         << "gps_messages: " << rep.gps_messages << "\n"
         << "replies: " << rep.replies << "\n"
         << "max_replies_per_sensor: " << rep.max_replies_per_sensor << "\n"
         << "px4_no_reply: " << no_reply << "\n"
         << "px4_stale_reply: " << stale_reply << "\n"
         << "px4_link_closed: " << link_closed << "\n"
         << "other_refusals: " << other_refusal << "\n"
         << "first_refusal: " << first_refusal << "\n"
         << "heartbeats_seen: " << rep.heartbeats_seen << "\n"
         << "command_longs_seen: " << rep.command_longs_seen << "\n"
         << "last_time_usec: " << rep.last_time_usec << "\n"
         << "wall_s: " << wall << "\n"
         << "wall_wait_max_ms: " << std::chrono::duration<double, std::milli>(rep.wall_wait_max).count() << "\n"
         << "parser_frames: " << rep.parser.frames << "\n"
         << "parser_bad_crc: " << rep.parser.bad_crc << "\n"
         << "parser_unknown_msgid: " << rep.parser.unknown_msgid << "\n"
         << "parser_garbage_bytes: " << rep.parser.garbage_bytes << "\n"
         << "armed_seen: " << (ever_armed ? "yes" : "no") << "\n"
         << "max_motor_output: " << max_motor << "\n"
         << "commander_check_exit: " << commander_rc << "\n";
    std::ofstream(out / "lockstep-report.txt") << text.str();
    std::cout << text.str();

    EXPECT_EQ(done, steps) << first_refusal;
    EXPECT_EQ(no_reply + stale_reply + link_closed + other_refusal, 0u) << first_refusal;
    EXPECT_EQ(rep.max_replies_per_sensor, 1u);
}
