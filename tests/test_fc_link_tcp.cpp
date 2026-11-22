// tests/test_fc_link_tcp.cpp -- the real TCP channel under the PX4 link (tools/fc_link/tcp.hpp), on the loopback interface with a
// system-picked port, and the whole lockstep (Px4Link) over it against a PX4-like peer thread.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "../tools/fc_link/mavlink.hpp"
#include "../tools/fc_link/px4_link.hpp"
#include "../tools/fc_link/tcp.hpp"

namespace {

using namespace spade::fc_link;
using namespace std::chrono_literals;
namespace mav = spade::fc_link::mavlink;

// Reads from `ch` until `n` bytes arrived or `limit` passed.
std::vector<uint8_t> read_n(ByteChannel& ch, size_t n, std::chrono::milliseconds limit = 5s) {
    std::vector<uint8_t> out;
    const auto end = std::chrono::steady_clock::now() + limit;
    while (out.size() < n && std::chrono::steady_clock::now() < end) {
        auto r = ch.read_some(50ms);
        if (!r) break;
        out.insert(out.end(), r->begin(), r->end());
    }
    return out;
}

}  // namespace

TEST(FcLinkTcp, AListenerOnPortZeroSaysWhichPortItGot) {
    auto l = TcpListener::listen(0);
    ASSERT_TRUE(l.has_value()) << l.error().context;
    EXPECT_GT(l->port(), 0);
}

TEST(FcLinkTcp, AcceptingWithNobodyConnectingTimesOutAndNamesIt) {
    auto l = TcpListener::listen(0);
    ASSERT_TRUE(l.has_value());
    const auto t0 = std::chrono::steady_clock::now();
    auto c = l->accept(80ms);
    const auto waited = std::chrono::steady_clock::now() - t0;
    ASSERT_FALSE(c.has_value());
    EXPECT_NE(c.error().context.find("no connection"), std::string::npos) << c.error().context;
    EXPECT_GE(waited, 70ms);
    EXPECT_LT(waited, 3s);
}

TEST(FcLinkTcp, BytesGoBothWaysExactly) {
    auto l = TcpListener::listen(0);
    ASSERT_TRUE(l.has_value());
    auto client = tcp_connect("127.0.0.1", l->port(), 2s);
    ASSERT_TRUE(client.has_value()) << client.error().context;
    auto server = l->accept(2s);
    ASSERT_TRUE(server.has_value()) << server.error().context;

    std::vector<uint8_t> blob(300000);
    for (size_t i = 0; i < blob.size(); ++i) blob[i] = static_cast<uint8_t>((i * 31 + 7) & 0xFF);
    std::thread writer([&] { EXPECT_TRUE((*client)->write(blob).has_value()); });  // larger than a socket buffer: the reader must keep up
    const std::vector<uint8_t> got = read_n(**server, blob.size());
    writer.join();
    EXPECT_EQ(got, blob) << "client to server";

    const std::vector<uint8_t> small = {1, 2, 3, 4, 5};
    ASSERT_TRUE((*server)->write(small).has_value());
    EXPECT_EQ(read_n(**client, small.size()), small) << "server to client";
}

TEST(FcLinkTcp, ReadingWithNothingSentReturnsEmptyAfterTheTimeout) {
    auto l = TcpListener::listen(0);
    ASSERT_TRUE(l.has_value());
    auto client = tcp_connect("127.0.0.1", l->port(), 2s);
    ASSERT_TRUE(client.has_value());
    auto server = l->accept(2s);
    ASSERT_TRUE(server.has_value());
    const auto t0 = std::chrono::steady_clock::now();
    auto r = (*server)->read_some(60ms);
    const auto waited = std::chrono::steady_clock::now() - t0;
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_TRUE(r->empty());
    EXPECT_GE(waited, 50ms);
    EXPECT_LT(waited, 2s);
}

TEST(FcLinkTcp, ThePeerClosingIsAnErrorOnTheNextRead) {
    auto l = TcpListener::listen(0);
    ASSERT_TRUE(l.has_value());
    auto client = tcp_connect("127.0.0.1", l->port(), 2s);
    ASSERT_TRUE(client.has_value());
    auto server = l->accept(2s);
    ASSERT_TRUE(server.has_value());
    client->reset();  // closes
    bool failed = false;
    for (int i = 0; i < 40 && !failed; ++i) {
        auto r = (*server)->read_some(50ms);
        failed = !r.has_value();
    }
    EXPECT_TRUE(failed) << "a closed peer must surface, not look like silence forever";
}

TEST(FcLinkTcp, ConnectingToNothingFailsWithinTheTimeout) {
    uint16_t dead_port = 0;
    {
        auto l = TcpListener::listen(0);
        ASSERT_TRUE(l.has_value());
        dead_port = l->port();
    }  // closed again: nobody listens there
    const auto t0 = std::chrono::steady_clock::now();
    auto c = tcp_connect("127.0.0.1", dead_port, 1s);
    EXPECT_FALSE(c.has_value());
    EXPECT_LT(std::chrono::steady_clock::now() - t0, 5s);
}

// ---- the whole lockstep over a real socket, against a PX4-like peer thread ----

TEST(FcLinkTcp, TheLockstepRunsOverARealSocketAgainstAPx4LikePeer) {
    auto l = TcpListener::listen(0);
    ASSERT_TRUE(l.has_value());
    const uint16_t port = l->port();
    std::atomic<uint64_t> answered{0};
    std::thread peer([&] {  // plays PX4: connects, sends its HEARTBEAT, answers every HIL_SENSOR with an actuator message
        auto ch = tcp_connect("127.0.0.1", port, 3s);
        if (!ch) return;
        const auto hb = mav::encode(mav::Header{0, 1, 1}, mav::Heartbeat{0, 2, 12, 0, 3, 3});
        if (!(*ch)->write(hb)) return;
        mav::Parser parser;
        uint8_t seq = 1;
        int idle = 0;
        while (idle < 100) {  // 100 x 50 ms of silence ends the peer
            auto r = (*ch)->read_some(50ms);
            if (!r) return;
            idle = r->empty() ? idle + 1 : 0;
            parser.feed(*r);
            while (auto f = parser.next()) {
                const auto m = mav::decode(*f);
                if (!m) continue;
                if (const auto* s = std::get_if<mav::HilSensor>(&*m)) {
                    mav::HilActuatorControls a;
                    a.time_usec = s->time_usec;
                    a.controls = {0.5f, 0.5f, 0.5f, 0.5f};
                    a.mode = 129;
                    a.flags = 1;
                    if (!(*ch)->write(mav::encode(mav::Header{seq++, 1, 1}, a))) return;
                    ++answered;
                }
            }
        }
    });

    auto server = l->accept(3s);
    ASSERT_TRUE(server.has_value()) << server.error().context;
    Px4LinkConfig cfg;
    cfg.reply_timeout = 2s;
    auto link = Px4Link::open(std::move(*server), cfg);
    ASSERT_TRUE(link.has_value()) << link.error().context;
    for (uint64_t k = 1; k <= 500; ++k) {
        mav::HilSensor s;
        s.time_usec = k * 4000;
        s.zacc = -9.80665f;
        s.fields_updated = 0x3F;
        const auto r = link->exchange(s);
        ASSERT_TRUE(r.has_value()) << "step " << k << ": " << r.error().context;
        ASSERT_EQ(r->time_usec, s.time_usec);
        ASSERT_FLOAT_EQ(r->controls[0], 0.5f);
    }
    EXPECT_EQ(link->report().replies, 500u);
    EXPECT_EQ(link->report().max_replies_per_sensor, 1u);
    link = spade::Result<Px4Link>(std::unexpected(spade::Error{spade::Code::internal, "closed"}));  // drops the link: closes the socket
    peer.join();
    EXPECT_EQ(answered.load(), 500u);
}
