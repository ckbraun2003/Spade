// ---------------------------------------------------------------------------
// test_fc_live_companion.cpp -- DRY RUN 1 of the flight film (video #3; docs/design/interface/plans/2026-11-21-fc-links.md section 13): does the
// companion (offboard) link reach the real PX4 image, and what does PX4 say back?
//
// Label `fc` (the FcLive* name prefix), run through scripts/fc-run.ps1 -PublishOffboard, which publishes the container's UDP 14580 on this
// machine's loopback only (PX4 LISTENS on 14580 inside the container; 14540 is where it sends, inside the container). With no autopilot run
// (SPADE_FC_RUN_ID unset) it SKIPS by name.
//
// What it does: the same boot as FcLivePx4 (TCP 4560, Px4Link, a still vehicle with a little noise from its own fixed stream), and once PX4
// has answered, a UdpChannel to 127.0.0.1:14580 sends companion_heartbeat() once per SIMULATED second while the lockstep keeps stepping. Every
// datagram that comes back is parsed and counted by MAVLink message id. It sends NO command: nothing arms, nothing takes off, no motor can
// spin. It RECORDS, it does not assert a fact it is there to find out: it asserts only that the lockstep ran and the three link failures are 0.
// SPADE_FC_STEPS (default 5000 = 20 s of simulated time after boot) sets the length. The record is companion-report.txt in the run folder.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <variant>

#include "../tools/fc_link/companion.hpp"
#include "../tools/fc_link/mavlink.hpp"
#include "../tools/fc_link/px4_link.hpp"
#include "../tools/fc_link/tcp.hpp"
#include "../tools/fc_link/udp.hpp"

namespace {

using namespace spade::fc_link;
using namespace std::chrono_literals;
namespace mav = spade::fc_link::mavlink;

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

constexpr uint16_t kPx4ListenPort = 14580;  // PX4's local UDP port for the onboard/offboard instance, published on the host's loopback by fc-run

}  // namespace

TEST(FcLiveCompanion, TheCompanionHeartbeatReachesPx4AndWhatComesBackIsRecorded) {
    const std::string run = env_or_empty("SPADE_FC_RUN_ID");
    if (run.empty()) GTEST_SKIP() << "no autopilot run: run the fc tests through scripts/fc-run.ps1 -PublishOffboard";
    const std::string out_dir = env_or_empty("SPADE_FC_OUT_DIR");
    ASSERT_FALSE(out_dir.empty());
    const std::string steps_text = env_or_empty("SPADE_FC_STEPS");
    const uint64_t steps = steps_text.empty() ? 5000 : std::strtoull(steps_text.c_str(), nullptr, 10);
    const std::filesystem::path out(out_dir);

    auto listener = TcpListener::listen(4560, /*any_interface=*/true);
    ASSERT_TRUE(listener.has_value()) << listener.error().context;
    auto channel = listener->accept(120s);
    ASSERT_TRUE(channel.has_value()) << channel.error().context;
    Px4LinkConfig cfg;
    cfg.reply_timeout = 5s;
    cfg.heartbeat_timeout = 60s;
    auto link = Px4Link::open(std::move(*channel), cfg);
    ASSERT_TRUE(link.has_value()) << link.error().context;

    auto udp = UdpChannel::open(0, kPx4ListenPort);
    ASSERT_TRUE(udp.has_value()) << udp.error().context;

    uint64_t noise_state = 0x5ADE0000F00DULL;
    const auto noise = [&noise_state](double sigma) {  // the same fixed-seed still feed as FcLivePx4 (a constant signal reads as a stale sensor)
        double sum = 0.0;
        for (int i = 0; i < 4; ++i) {
            noise_state += 0x9E3779B97F4A7C15ULL;
            uint64_t z = noise_state;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
            z ^= z >> 31;
            sum += static_cast<double>(z >> 11) * (1.0 / 9007199254740992.0) - 0.5;
        }
        return static_cast<float>(sum * sigma * 1.7320508);
    };

    HeartbeatSchedule schedule;
    mav::Parser parser;
    uint8_t seq = 0;
    uint64_t heartbeats_sent = 0, send_failures = 0, datagrams = 0, datagram_bytes = 0;
    std::map<uint32_t, uint64_t> by_msgid;
    std::optional<double> first_back_sim_s, first_px4_heartbeat_sim_s, last_px4_heartbeat_sim_s, first_back_wall_s;
    std::optional<mav::Header> px4_header;
    std::optional<mav::Heartbeat> px4_heartbeat;
    uint64_t no_reply = 0, stale_reply = 0, link_closed = 0, other_refusal = 0, done = 0;
    std::string first_refusal;
    const auto wall0 = std::chrono::steady_clock::now();

    for (uint64_t k = 1; done < steps; ++k) {
        if (k > 5000000) break;
        mav::HilSensor s;
        s.time_usec = k * 4000;
        s.xacc = noise(0.05), s.yacc = noise(0.05), s.zacc = -9.80665F + noise(0.05);
        s.xgyro = noise(0.001), s.ygyro = noise(0.001), s.zgyro = noise(0.001);
        s.xmag = 0.21F + noise(0.002), s.ymag = noise(0.002), s.zmag = 0.43F + noise(0.002);
        s.abs_pressure = 955.F + noise(0.1);
        s.pressure_alt = 488.F;
        s.temperature = 15.F;
        s.fields_updated = 0x1BFF;
        std::optional<mav::HilGps> gps;
        if (k % 50 == 0) {
            mav::HilGps g;
            g.time_usec = s.time_usec;
            g.lat = 473977420, g.lon = 85455940, g.alt = 488000;
            g.eph = 100, g.epv = 100, g.cog = 65535;
            g.fix_type = 3, g.satellites_visible = 10;
            gps = g;
        }
        const auto r = link->exchange(s, gps);
        if (!r) {
            const std::string why = r.error().context;
            if (first_refusal.empty()) first_refusal = why;
            if (why.find("px4_no_reply") != std::string::npos) ++no_reply;
            else if (why.find("px4_stale_reply") != std::string::npos) ++stale_reply;
            else if (why.find("px4_link_closed") != std::string::npos) ++link_closed;
            else ++other_refusal;
            break;
        }
        if (r->booting) continue;
        ++done;
        const double sim_s = static_cast<double>(done - 1) * 0.004;  // Spade's own tick time, as FcLivePx4 counts it
        if (schedule.due((done - 1) * 4000u)) {
            const auto frame = mav::encode(mav::Header{seq++, kCompanionSystemId, kCompanionComponentId}, mav::Message{companion_heartbeat()});  // 191 = MAV_COMP_ID_ONBOARD_COMPUTER
            if ((*udp)->write(frame)) ++heartbeats_sent;
            else ++send_failures;
        }
        for (;;) {  // whatever has arrived: every datagram, never waiting
            auto d = (*udp)->read_some(0ms);
            if (!d || d->empty()) break;
            ++datagrams;
            datagram_bytes += d->size();
            if (!first_back_sim_s) {
                first_back_sim_s = sim_s;
                first_back_wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
            }
            parser.feed(*d);
            while (auto f = parser.next()) {
                ++by_msgid[f->msgid];
                if (f->msgid == static_cast<uint32_t>(mav::MsgId::heartbeat)) {
                    if (auto m = mav::decode(*f); m && std::holds_alternative<mav::Heartbeat>(*m)) {
                        if (!first_px4_heartbeat_sim_s) {
                            first_px4_heartbeat_sim_s = sim_s;
                            px4_header = f->header;
                            px4_heartbeat = std::get<mav::Heartbeat>(*m);
                        }
                        last_px4_heartbeat_sim_s = sim_s;
                    }
                }
            }
        }
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();

    const auto opt = [](const std::optional<double>& v) { return v ? std::to_string(*v) : std::string("none"); };
    const auto& rep = link->report();
    std::ostringstream text;
    text << "published_container_port: udp " << kPx4ListenPort << " (host 127.0.0.1 only)\n"
         << "steps_requested: " << steps << "\n"
         << "steps_done: " << done << "\n"
         << "boot_steps: " << rep.boot_steps << "\n"
         << "px4_no_reply: " << no_reply << "\n"
         << "px4_stale_reply: " << stale_reply << "\n"
         << "px4_link_closed: " << link_closed << "\n"
         << "other_refusals: " << other_refusal << "\n"
         << "first_refusal: " << first_refusal << "\n"
         << "companion_heartbeats_sent: " << heartbeats_sent << "\n"
         << "companion_send_failures: " << send_failures << "\n"
         << "datagrams_back: " << datagrams << "\n"
         << "datagram_bytes_back: " << datagram_bytes << "\n"
         << "first_datagram_back_sim_s: " << opt(first_back_sim_s) << "\n"
         << "first_datagram_back_wall_s: " << opt(first_back_wall_s) << "\n"
         << "px4_heartbeat_first_sim_s: " << opt(first_px4_heartbeat_sim_s) << "\n"
         << "px4_heartbeat_last_sim_s: " << opt(last_px4_heartbeat_sim_s) << "\n";
    if (px4_header && px4_heartbeat)
        text << "px4_heartbeat_sysid_compid: " << static_cast<int>(px4_header->sysid) << "/" << static_cast<int>(px4_header->compid) << "\n"
             << "px4_heartbeat_type_autopilot_base_custom_state: " << static_cast<int>(px4_heartbeat->type) << "/" << static_cast<int>(px4_heartbeat->autopilot)
             << "/" << static_cast<int>(px4_heartbeat->base_mode) << "/" << px4_heartbeat->custom_mode << "/" << static_cast<int>(px4_heartbeat->system_status) << "\n";
    for (const auto& [id, n] : by_msgid) text << "msgid_" << id << "_count: " << n << "\n";
    text << "parser_bad_crc: " << parser.stats().bad_crc << "\n"
         << "parser_unknown_msgid: " << parser.stats().unknown_msgid << "\n"
         << "wall_s: " << wall << "\n";
    std::ofstream(out / "companion-report.txt") << text.str();
    std::cout << text.str();

    EXPECT_EQ(done, steps) << first_refusal;
    EXPECT_EQ(no_reply + stale_reply + link_closed + other_refusal, 0u) << first_refusal;
    EXPECT_EQ(send_failures, 0u) << "a heartbeat the host could not send";
}
