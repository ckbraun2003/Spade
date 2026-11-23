// The ArduPilot link's UDP loop (tools/fc_link/ardupilot_server.hpp) over real loopback sockets: a fake ArduPilot (a UdpChannel that sends servo packets and reads
// the JSON lines) in the same process, a real LockstepHost on the flight world. No ArduPilot, no container. The packet layout is the documented one, not a capture.
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "fc_flight_world.hpp"
#include "fc_host/lockstep.hpp"
#include "fc_link/ardupilot_json.hpp"
#include "fc_link/ardupilot_link.hpp"
#include "fc_link/ardupilot_server.hpp"
#include "fc_link/udp.hpp"

namespace {

using namespace std::chrono_literals;
using namespace spade::fc_link;

std::vector<uint8_t> wire_packet(uint32_t count, uint16_t rate, uint16_t pwm, std::size_t channels = 16) {
    std::vector<uint8_t> b;
    const uint16_t magic = channels == 16 ? ardupilot_json::kMagic16 : ardupilot_json::kMagic32;
    b.push_back(static_cast<uint8_t>(magic & 0xFF));
    b.push_back(static_cast<uint8_t>(magic >> 8));
    b.push_back(static_cast<uint8_t>(rate & 0xFF));
    b.push_back(static_cast<uint8_t>(rate >> 8));
    for (int i = 0; i < 4; ++i) b.push_back(static_cast<uint8_t>((count >> (8 * i)) & 0xFF));
    for (std::size_t i = 0; i < channels; ++i) {
        b.push_back(static_cast<uint8_t>(pwm & 0xFF));
        b.push_back(static_cast<uint8_t>(pwm >> 8));
    }
    return b;
}

struct Rig {
    std::optional<fc_flight_test::FlightWorld> world;
    std::optional<spade::fc_host::LockstepHost> host;
    std::unique_ptr<UdpChannel> server_channel;  // Spade's side: binds, learns ArduPilot's address
    std::unique_ptr<UdpChannel> peer;            // the fake ArduPilot
    std::optional<ArduPilotServer> server;
};

void open_rig(Rig& r, std::chrono::milliseconds timeout = 2000ms) {
    r.world = fc_flight_test::make_flight_world();
    ASSERT_TRUE(r.world.has_value());
    auto host = spade::fc_host::LockstepHost::open(r.world->sim, r.world->vehicle, r.world->bindings);
    ASSERT_TRUE(host.has_value()) << host.error().context;
    r.host.emplace(std::move(*host));
    auto sc = UdpChannel::open(0);
    ASSERT_TRUE(sc.has_value()) << sc.error().context;
    r.server_channel = std::move(*sc);
    auto pc = UdpChannel::open(0, r.server_channel->local_port());
    ASSERT_TRUE(pc.has_value()) << pc.error().context;
    r.peer = std::move(*pc);
    ArduPilotConfig cfg;
    cfg.chain = &*r.world->model.model.propulsion;
    auto link = ArduPilotLink::create(cfg);
    ASSERT_TRUE(link.has_value()) << link.error().context;
    auto server = ArduPilotServer::create(*r.server_channel, std::move(*link), timeout);
    ASSERT_TRUE(server.has_value()) << server.error().context;
    r.server.emplace(std::move(*server));
}

std::string reply_text(UdpChannel& peer) {
    const auto got = peer.read_some(2000ms);
    if (!got || got->empty()) return {};
    return std::string(got->begin(), got->end());
}

TEST(FcLinkArdupilotUdp, AServerWithNoTimeoutIsRefused) {
    auto sc = UdpChannel::open(0);
    ASSERT_TRUE(sc.has_value());
    spade::fc_link::ArduPilotConfig cfg;
    auto world = fc_flight_test::make_flight_world();
    ASSERT_TRUE(world.has_value());
    cfg.chain = &*world->model.model.propulsion;
    auto link = ArduPilotLink::create(cfg);
    ASSERT_TRUE(link.has_value());
    const auto server = ArduPilotServer::create(**sc, std::move(*link), 0ms);
    ASSERT_FALSE(server.has_value());
    EXPECT_NE(server.error().context.find("timeout"), std::string::npos) << server.error().context;
}

TEST(FcLinkArdupilotUdp, OneServoPacketGetsOneJsonLineBackOnTheAddressItCameFrom) {
    Rig r;
    ASSERT_NO_FATAL_FAILURE(open_rig(r));
    ASSERT_TRUE(r.peer->write(wire_packet(1, 250, 1000)).has_value());
    const auto step = r.server->serve_one(*r.host);
    ASSERT_TRUE(step.has_value()) << step.error().context;
    const std::string text = reply_text(*r.peer);
    ASSERT_FALSE(text.empty()) << "no reply reached the fake ArduPilot";
    EXPECT_EQ(text.rfind("{\"timestamp\":0.004,", 0), 0u) << text;
    EXPECT_EQ(text.back(), '\n');
    EXPECT_EQ(text.find('\n'), text.size() - 1) << "one datagram, one line";
    EXPECT_EQ(r.host->tick(), 8u);
    EXPECT_EQ(r.server->report().packets, 1u);
    EXPECT_EQ(r.server->report().replies, 1u);
    EXPECT_EQ(r.server->report().bytes_in, 40u);
    EXPECT_EQ(r.server->report().bytes_out, text.size());
    EXPECT_EQ(r.server->report().last_frame_count, 1u);
}

TEST(FcLinkArdupilotUdp, TenPacketsGiveTenRepliesWithRisingTimestamps) {
    Rig r;
    ASSERT_NO_FATAL_FAILURE(open_rig(r));
    std::string previous;
    for (uint32_t i = 1; i <= 10; ++i) {
        ASSERT_TRUE(r.peer->write(wire_packet(i, 250, 1200)).has_value());
        const auto step = r.server->serve_one(*r.host);
        ASSERT_TRUE(step.has_value()) << "frame " << i << ": " << step.error().context;
        const std::string text = reply_text(*r.peer);
        ASSERT_FALSE(text.empty()) << "frame " << i;
        EXPECT_NE(text, previous);
        previous = text;
        EXPECT_NEAR(step->state.timestamp_s, 0.004 * i, 1e-9);
    }
    EXPECT_EQ(r.server->report().replies, 10u);
}

TEST(FcLinkArdupilotUdp, AResentPacketIsAnsweredAgainAndCountedAndTheWorldDoesNotMove) {
    Rig r;
    ASSERT_NO_FATAL_FAILURE(open_rig(r));
    ASSERT_TRUE(r.peer->write(wire_packet(4, 250, 1500)).has_value());
    ASSERT_TRUE(r.server->serve_one(*r.host).has_value());
    const std::string first = reply_text(*r.peer);
    const uint64_t tick = r.host->tick();
    ASSERT_TRUE(r.peer->write(wire_packet(4, 250, 1500)).has_value());
    const auto again = r.server->serve_one(*r.host);
    ASSERT_TRUE(again.has_value()) << again.error().context;
    EXPECT_EQ(reply_text(*r.peer), first);
    EXPECT_EQ(r.host->tick(), tick);
    EXPECT_EQ(r.server->report().resends, 1u);
    EXPECT_EQ(r.server->report().replies, 2u);
}

TEST(FcLinkArdupilotUdp, NoPacketInTimeIsARefusalNamingItAndTheWorldDoesNotMove) {
    Rig r;
    ASSERT_NO_FATAL_FAILURE(open_rig(r, 100ms));
    const auto step = r.server->serve_one(*r.host);
    ASSERT_FALSE(step.has_value());
    EXPECT_EQ(step.error().code, spade::Code::io_error);
    EXPECT_NE(step.error().context.find("ardupilot_no_packet"), std::string::npos) << step.error().context;
    EXPECT_NE(step.error().context.find("100"), std::string::npos) << "the timeout is named: " << step.error().context;
    EXPECT_EQ(r.host->tick(), 0u);
    EXPECT_EQ(r.server->report().packets, 0u);
}

TEST(FcLinkArdupilotUdp, ADatagramThatIsNotAServoPacketIsRefusedAndNothingIsWritten) {
    Rig r;
    ASSERT_NO_FATAL_FAILURE(open_rig(r));
    ASSERT_TRUE(r.peer->write(std::vector<uint8_t>(12, 0x55)).has_value());
    const auto step = r.server->serve_one(*r.host);
    ASSERT_FALSE(step.has_value());
    EXPECT_NE(step.error().context.find("length"), std::string::npos) << step.error().context;
    EXPECT_EQ(r.host->tick(), 0u);
    const auto none = r.peer->read_some(100ms);
    ASSERT_TRUE(none.has_value());
    EXPECT_TRUE(none->empty()) << "a refused datagram gets no reply";
    EXPECT_EQ(r.server->report().replies, 0u);
}

TEST(FcLinkArdupilotUdp, ALinkRefusalComesBackUnchangedAndNothingIsWritten) {
    Rig r;
    ASSERT_NO_FATAL_FAILURE(open_rig(r));
    ASSERT_TRUE(r.peer->write(wire_packet(1, 400, 1000)).has_value());  // 2.5 ms: not the IMU's 4 ms
    const auto step = r.server->serve_one(*r.host);
    ASSERT_FALSE(step.has_value());
    EXPECT_NE(step.error().context.find("IMU period"), std::string::npos) << step.error().context;
    const auto none = r.peer->read_some(100ms);
    ASSERT_TRUE(none.has_value());
    EXPECT_TRUE(none->empty());
}

TEST(FcLinkArdupilotUdp, TheSamePacketsGiveTheSameReplyBytesOverTheWireTwice) {
    Rig a, b;
    ASSERT_NO_FATAL_FAILURE(open_rig(a));
    ASSERT_NO_FATAL_FAILURE(open_rig(b));
    for (uint32_t i = 1; i <= 30; ++i) {
        const uint16_t pwm = static_cast<uint16_t>(1100 + (i * 29) % 800);
        ASSERT_TRUE(a.peer->write(wire_packet(i, 250, pwm)).has_value());
        ASSERT_TRUE(b.peer->write(wire_packet(i, 250, pwm)).has_value());
        ASSERT_TRUE(a.server->serve_one(*a.host).has_value());
        ASSERT_TRUE(b.server->serve_one(*b.host).has_value());
        const std::string ta = reply_text(*a.peer);
        const std::string tb = reply_text(*b.peer);
        ASSERT_FALSE(ta.empty());
        ASSERT_EQ(ta, tb) << "frame " << i;
    }
}

}  // namespace
