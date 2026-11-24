// ---------------------------------------------------------------------------
// test_fc_live_flight.cpp -- the flight film's mission against the real PX4 image (video #3; docs/design/interface/plans/2026-11-21-fc-links.md section 13):
// arm, take off to 2 m, hover 10 s, land, and the autopilot disarms on its own.
//
// Label `fc` (the FcLive* name prefix), run through scripts/fc-run.ps1 -PublishOffboard. With no autopilot run (SPADE_FC_RUN_ID unset) it SKIPS by name.
//
// The world is Physics' X500 on a ground plane (tests/fc_flight_world.hpp); PX4 connects to TCP 4560 and runs in lockstep with it through
// AutopilotLink::exchange (the IMU, barometer, magnetometer and GNSS, and PX4's motor outputs through the model's own chain). The commands go to PX4
// over the companion link (UDP 14580 on the loopback, fc_link/udp.hpp) as COMMAND_LONG from the companion's ids (companion.hpp), a heartbeat once a
// simulated second; PX4's COMMAND_ACKs come back on it. FlightMission (fc_link/flight_mission.hpp) decides what to send and judges every step.
//
// What it records in the run folder: flight-report.txt (the journal, the acks, the counters), flight-trace.csv (ten lines a simulated second: time,
// altitude, armed, the four motor outputs, the mission's phase). The bands are FlightConfig's defaults, PROVISIONAL until the dry runs measure them.
// SPADE_FC_FLIGHT_MAX_S (default 150) caps the simulated time.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "fc_flight_record.hpp"
#include "fc_flight_report.hpp"
#include "fc_flight_world.hpp"
#include "fc_link/autopilot_link.hpp"
#include "fc_link/companion.hpp"
#include "fc_link/flight_mission.hpp"
#include "fc_link/mavlink.hpp"
#include "fc_link/px4_link.hpp"
#include "fc_link/tcp.hpp"
#include "fc_link/udp.hpp"

namespace {

using namespace spade::fc_link;
using namespace std::chrono_literals;
namespace mav = spade::fc_link::mavlink;
using fc_flight_test::FlightRunFacts;
using fc_flight_test::format_flight_report;
using fc_flight_test::kSlowExchangeMs;

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

constexpr uint16_t kPx4ListenPort = 14580;  // PX4's local UDP port for the offboard instance, published on the host's loopback by fc-run

const char* phase_name(FlightMission::Phase p) {
    switch (p) {
        case FlightMission::Phase::boot: return "boot";
        case FlightMission::Phase::arming: return "arming";
        case FlightMission::Phase::takeoff: return "takeoff";
        case FlightMission::Phase::settle: return "settle";
        case FlightMission::Phase::hover: return "hover";
        case FlightMission::Phase::land: return "land";
        case FlightMission::Phase::aborting: return "aborting";
        case FlightMission::Phase::done: return "done";
        case FlightMission::Phase::failed: return "failed";
    }
    return "?";
}

const char* abort_name(FlightMission::AbortOutcome a) {
    switch (a) {
        case FlightMission::AbortOutcome::none: return "none";
        case FlightMission::AbortOutcome::landed_and_disarmed: return "landed_and_disarmed";
        case FlightMission::AbortOutcome::expired: return "bound_expired";
    }
    return "?";
}

struct AckRecord {
    double t;
    uint16_t command;
    uint8_t result;
};

}  // namespace

TEST(FcLiveFlight, Px4FliesTheX500ThroughArmTakeoffHoverAndLand) {
    const std::string run = env_or_empty("SPADE_FC_RUN_ID");
    if (run.empty()) GTEST_SKIP() << "no autopilot run: run the fc tests through scripts/fc-run.ps1 -PublishOffboard";
    const std::string out_dir = env_or_empty("SPADE_FC_OUT_DIR");
    ASSERT_FALSE(out_dir.empty());
    const std::string max_text = env_or_empty("SPADE_FC_FLIGHT_MAX_S");
    const double max_s = max_text.empty() ? 150.0 : std::strtod(max_text.c_str(), nullptr);
    const std::filesystem::path out(out_dir);

    std::optional<fc_flight_test::FlightWorld> world = fc_flight_test::make_flight_world();
    ASSERT_TRUE(world.has_value());
    auto host = spade::fc_host::LockstepHost::open(world->sim, world->vehicle, world->bindings);
    ASSERT_TRUE(host.has_value()) << host.error().context;
    ASSERT_TRUE(world->model.model.propulsion.has_value());

    auto listener = TcpListener::listen(4560, /*any_interface=*/true);
    ASSERT_TRUE(listener.has_value()) << listener.error().context;
    auto channel = listener->accept(120s);
    ASSERT_TRUE(channel.has_value()) << channel.error().context;
    Px4LinkConfig link_cfg;
    link_cfg.reply_timeout = 5s;
    link_cfg.heartbeat_timeout = 60s;
    auto link = Px4Link::open(std::move(*channel), link_cfg);
    ASSERT_TRUE(link.has_value()) << link.error().context;
    auto udp = UdpChannel::open(0, kPx4ListenPort);
    ASSERT_TRUE(udp.has_value()) << udp.error().context;

    std::optional<ActuatorReply> last_reply;
    ExchangeConfig cfg;
    cfg.chain = &*world->model.model.propulsion;
    cfg.baro_binding = fc_flight_test::kBaroBinding;  // where each sensor sits, and the home, live in the hashed world header (tests/fc_flight_world.hpp)
    cfg.mag_binding = fc_flight_test::kMagBinding;
    cfg.gnss_binding = fc_flight_test::kGnssBinding;
    cfg.boot = fc_flight_test::flight_boot_config();  // the boot messages' baro and mag noise is the world's: one source
    cfg.geo_frame = spade::geo::make_local_frame(spade::geo::Geodetic{fc_flight_test::kHomeLat, fc_flight_test::kHomeLon, fc_flight_test::kGroundAmslM});
    cfg.home_world = glm::dvec3(0.0);  // the ground under the vehicle's start is the autopilot's NED origin
    auto alink = AutopilotLink::create(
        [&](const mav::HilSensor& s, const std::optional<mav::HilGps>& g) -> spade::Result<ActuatorReply> {
            auto r = link->exchange(s, g);
            if (r) last_reply = *r;
            return r;
        },
        cfg);
    ASSERT_TRUE(alink.has_value()) << alink.error().context;

    FlightConfig fcfg;  // the defaults: PROVISIONAL bands
    fcfg.home_amsl_m = fc_flight_test::kGroundAmslM + world->rest_com_y;  // altitude_m() is the centre of mass above its rest height; param7 is AMSL
    FlightMission mission(fcfg);
    HeartbeatSchedule schedule;
    mav::Parser parser;
    uint8_t seq = 0;
    std::map<uint32_t, uint64_t> by_msgid;
    std::vector<AckRecord> acks;
    std::vector<std::string> sent_log;
    uint64_t no_reply = 0, stale_reply = 0, link_closed = 0, other_refusal = 0, send_failures = 0, datagrams = 0;
    std::string first_refusal;
    std::ofstream trace(out / "flight-trace.csv");
    trace << "t_s,altitude_m,armed,motor0,motor1,motor2,motor3,phase\n";
    double next_trace_s = 0.0;
    const auto wall0 = std::chrono::steady_clock::now();
    const auto wall_since = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count(); };
    FlightRunFacts facts;  // the report: first datagram, the wait and the host gap of every exchange
    uint64_t seen_exchanges = 0;
    std::vector<double> wait_samples_ns, gap_samples_ns;  // per exchange, for the run record's stages

    for (uint64_t guard = 0; guard < 100000000ull; ++guard) {
        const auto res = alink->exchange(*host);
        if (const auto& rep = link->report(); rep.sensor_messages != seen_exchanges) {  // one exchange of the link happened: its wait and the host gap before it
            seen_exchanges = rep.sensor_messages;
            const double wait_ms = std::chrono::duration<double, std::milli>(rep.wall_wait_last).count();
            const double gap_ms = std::chrono::duration<double, std::milli>(rep.wall_gap_last).count();
            const double sim_now = static_cast<double>(host->sim_time_ns()) * 1e-9;
            const double wall_now = wall_since();
            ++facts.exchanges;
            wait_samples_ns.push_back(static_cast<double>(rep.wall_wait_last.count()));
            gap_samples_ns.push_back(static_cast<double>(rep.wall_gap_last.count()));
            facts.wait_total_ms += wait_ms;
            facts.gap_total_ms += gap_ms;
            if (wait_ms > facts.wait_max_ms) { facts.wait_max_ms = wait_ms; facts.wait_max_at_sim_s = sim_now; facts.wait_max_at_wall_s = wall_now; }
            if (gap_ms > facts.gap_max_ms) { facts.gap_max_ms = gap_ms; facts.gap_max_at_sim_s = sim_now; facts.gap_max_at_wall_s = wall_now; }
            if (wait_ms > kSlowExchangeMs || gap_ms > kSlowExchangeMs)
                facts.slow.push_back({rep.sensor_messages, sim_now, wall_now, wait_ms, gap_ms, rep.boot_end_time_usec == 0});
        }
        if (!res) {
            const std::string why = res.error().context;
            if (first_refusal.empty()) first_refusal = why;
            if (why.find("px4_no_reply") != std::string::npos) ++no_reply;
            else if (why.find("px4_stale_reply") != std::string::npos) ++stale_reply;
            else if (why.find("px4_link_closed") != std::string::npos) ++link_closed;
            else ++other_refusal;
            break;
        }
        if (res->booting || !last_reply) continue;
        const double t = static_cast<double>(host->sim_time_ns()) * 1e-9;
        const uint64_t t_us = host->sim_time_ns() / 1000;

        if (schedule.due(t_us)) {
            const auto frame = mav::encode(mav::Header{seq++, kCompanionSystemId, kCompanionComponentId}, mav::Message{companion_heartbeat()});
            if (!(*udp)->write(frame)) ++send_failures;
        }
        std::optional<AckObservation> ack;
        for (;;) {  // every datagram that has arrived, never waiting
            auto d = (*udp)->read_some(0ms);
            if (!d || d->empty()) break;
            if (++datagrams == 1) facts.first_datagram_wall_s = wall_since();
            parser.feed(*d);
            while (auto f = parser.next()) {
                ++by_msgid[f->msgid];
                if (f->msgid != static_cast<uint32_t>(mav::MsgId::command_ack)) continue;
                if (auto m = mav::decode(*f); m && std::holds_alternative<mav::CommandAck>(*m)) {
                    const auto& a = std::get<mav::CommandAck>(*m);
                    acks.push_back({t, a.command, a.result});
                    ack = AckObservation{a.command, a.result};
                }
            }
        }
        const double altitude = world->altitude_m();
        const auto commands = mission.step(FlightObservation{t, last_reply->armed, altitude, ack});
        for (const mav::CommandLong& c : commands) {
            const auto frame = mav::encode(mav::Header{seq++, kCompanionSystemId, kCompanionComponentId}, mav::Message{c});
            if ((*udp)->write(frame)) {
                std::ostringstream line;
                line << "t " << t << " s: command " << c.command << " param1 " << c.param[0] << " param7 " << c.param[6];
                sent_log.push_back(line.str());
            } else {
                ++send_failures;
            }
        }
        if (t >= next_trace_s) {
            trace << t << ',' << altitude << ',' << (last_reply->armed ? 1 : 0) << ',' << last_reply->controls[0] << ',' << last_reply->controls[1] << ','
                  << last_reply->controls[2] << ',' << last_reply->controls[3] << ',' << phase_name(mission.phase()) << '\n';
            next_trace_s = t + 0.1;
        }
        if (mission.finished() || t > max_s) break;
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    const double sim_s = static_cast<double>(host->sim_time_ns()) * 1e-9;

    facts.mission_phase = phase_name(mission.phase());
    facts.mission_passed = mission.passed();
    facts.last_armed = mission.last_armed();
    facts.last_altitude_m = mission.last_altitude_m();
    facts.landed = mission.landed();
    facts.abort_outcome = abort_name(mission.abort_outcome());
    facts.sim_s = sim_s;
    facts.wall_s = wall;
    facts.boot_steps = link->report().boot_steps;
    facts.px4_no_reply = no_reply;
    facts.px4_stale_reply = stale_reply;
    facts.px4_link_closed = link_closed;
    facts.other_refusals = other_refusal;
    facts.first_refusal = first_refusal;
    facts.send_failures = send_failures;
    facts.datagrams_back = datagrams;
    facts.link = link->report();
    facts.journal = mission.journal();
    facts.sent_log = sent_log;
    for (const AckRecord& a : acks) facts.acks.push_back({a.t, a.command, a.result});
    facts.by_msgid = by_msgid;
    const std::string text = format_flight_report(facts);
    std::ofstream(out / "flight-report.txt") << text;
    std::cout << text;

    // The run record (tests/fc_flight_record.hpp): the image id arrives from the wrapper (SPADE_FC_IMAGE_ID, from `docker image inspect --format {{.Id}}`);
    // without it the record is REFUSED, not filled in, and the test fails saying so.
    fc_flight_test::FlightRecordInputs record_in;
    record_in.run_id = run;
    record_in.package_hash = env_or_empty("SPADE_FC_IMAGE_ID");
    record_in.link_version = "v1.17.0";
    record_in.dockerfile = std::filesystem::path(SPADE_REPO_DIR) / "tools/fc_link/px4/Dockerfile";
    record_in.model_file = std::filesystem::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500/x500.model.yaml";
    record_in.world_file = std::filesystem::path(SPADE_REPO_DIR) / "tests/fc_flight_world.hpp";
    record_in.profile_board_yaml = "board: px4_sitl_airframe_10090\nlink: docker_desktop_forward\nlockstep: hil_sensor_4ms\n";
    record_in.wait_ns = wait_samples_ns;
    record_in.gap_ns = gap_samples_ns;
    const auto record = fc_flight_test::build_flight_record(*host, world->sim, record_in);
    EXPECT_TRUE(record.has_value()) << "the run record is refused: " << (record ? "" : record.error().context);
    if (record) {
        const auto written = spade::fc_host::write_run_folder(*record, out / "run-record");
        EXPECT_TRUE(written.has_value()) << (written ? "" : written.error().context);
    }

    EXPECT_EQ(no_reply + stale_reply + link_closed + other_refusal, 0u) << first_refusal;
    EXPECT_EQ(send_failures, 0u);
    EXPECT_TRUE(mission.passed()) << "phase " << phase_name(mission.phase());
}
