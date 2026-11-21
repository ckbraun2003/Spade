// tests/test_fc_link_px4.cpp -- the PX4 lockstep link (tools/fc_link/px4_link.hpp) against a SCRIPTED in-memory PX4.
//
// The fake plays what PX4's simulator module does (docs/design/interface/plans/2026-11-21-fc-links.md section 1.2): on connecting it sends one
// HEARTBEAT and one COMMAND_LONG (MAV_CMD_SET_MESSAGE_INTERVAL for HIL_STATE_QUATERNION); then for each HIL_SENSOR with id 0 it answers one
// HIL_ACTUATOR_CONTROLS stamped with that sensor's time_usec. Variants make it late, silent, stale, doubled, chunked or closed. The step is
// PX4's 4 ms (8 ticks of 500 us); the GPS every 50 steps (200 ticks), as the reconciled test plan has it.
//
// These run in the ordinary suite. The same lockstep against a real PX4 is the opt-in `fc` label (Test/Docs' L1, F1, R2).
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <string>
#include <thread>
#include <vector>

#include "fc_link/mavlink.hpp"
#include "fc_link/px4_link.hpp"

namespace {

using namespace spade::fc_link;
using namespace std::chrono_literals;
namespace mav = spade::fc_link::mavlink;

enum class Mode { normal, silent_after, stale, doubled, closed_after, garbage_first };

struct Script {
    Mode mode = Mode::normal;
    uint64_t after = 0;           // silent_after / closed_after: sensors answered before the change
    size_t chunk = 0;             // 0 = deliver everything queued in one read, else this many bytes per read
    bool send_heartbeat = true;   // PX4's first message on connecting
};

// What the fake saw us send, for the tests to check.
struct Seen {
    std::vector<uint8_t> raw;  // every byte we wrote
    std::vector<mav::HilSensor> sensors;
    std::vector<mav::HilGps> gps;
    std::vector<mav::Header> headers;
};

class FakePx4 final : public ByteChannel {
public:
    FakePx4(Script script, std::shared_ptr<Seen> seen) : script_(script), seen_(std::move(seen)) {
        if (script_.send_heartbeat) {
            queue(mav::Header{0, 1, 1}, mav::Heartbeat{0, 2, 12, 0, 3, 3});
            mav::CommandLong c;
            c.param[0] = 115.0f;
            c.param[1] = 5000.0f;
            c.command = 511;
            c.target_system = 1;
            c.target_component = 1;
            queue(mav::Header{1, 1, 1}, c);
        }
    }

    spade::Result<void> write(std::span<const uint8_t> bytes) override {
        if (closed_) return std::unexpected(spade::Error{spade::Code::io_error, "closed"});
        seen_->raw.insert(seen_->raw.end(), bytes.begin(), bytes.end());
        parser_.feed(bytes);
        while (auto f = parser_.next()) {
            const auto m = mav::decode(*f);
            if (!m) continue;
            seen_->headers.push_back(f->header);
            if (const auto* gps = std::get_if<mav::HilGps>(&*m)) seen_->gps.push_back(*gps);
            if (const auto* s = std::get_if<mav::HilSensor>(&*m)) {
                seen_->sensors.push_back(*s);
                answer(*s);
            }
        }
        return {};
    }

    spade::Result<std::vector<uint8_t>> read_some(std::chrono::milliseconds timeout) override {
        if (closed_ && inbox_.empty()) return std::unexpected(spade::Error{spade::Code::io_error, "peer closed"});
        if (inbox_.empty()) {
            std::this_thread::sleep_for(std::min<std::chrono::milliseconds>(timeout, 2ms));
            return std::vector<uint8_t>{};
        }
        const size_t n = script_.chunk == 0 ? inbox_.size() : std::min(script_.chunk, inbox_.size());
        std::vector<uint8_t> out(inbox_.begin(), inbox_.begin() + static_cast<std::ptrdiff_t>(n));
        inbox_.erase(inbox_.begin(), inbox_.begin() + static_cast<std::ptrdiff_t>(n));
        return out;
    }

private:
    void queue(const mav::Header& h, const mav::Message& m) {
        const std::vector<uint8_t> b = mav::encode(h, m);
        inbox_.insert(inbox_.end(), b.begin(), b.end());
    }

    static mav::HilActuatorControls controls_for(uint64_t time_usec, float bias) {
        mav::HilActuatorControls a;
        a.time_usec = time_usec;
        for (size_t i = 0; i < 4; ++i) a.controls[i] = 0.1f * static_cast<float>(i + 1) + bias;
        a.mode = 129;
        a.flags = 1;
        return a;
    }

    void answer(const mav::HilSensor& s) {
        if (s.id != 0) return;
        const uint64_t n = answered_++;
        if (script_.mode == Mode::silent_after && n >= script_.after) return;
        if (script_.mode == Mode::closed_after && n >= script_.after) {
            closed_ = true;
            return;
        }
        if (script_.mode == Mode::garbage_first) inbox_.insert(inbox_.end(), {0x00, 0xFF, 0x13, 0x37});
        const uint64_t t = script_.mode == Mode::stale ? s.time_usec - 4000 : s.time_usec;
        queue(mav::Header{static_cast<uint8_t>(seq_++), 1, 1}, controls_for(t, 0.0f));
        if (script_.mode == Mode::doubled) queue(mav::Header{static_cast<uint8_t>(seq_++), 1, 1}, controls_for(t, 0.05f));
    }

    Script script_;
    std::shared_ptr<Seen> seen_;
    mav::Parser parser_;
    std::deque<uint8_t> inbox_;
    uint64_t answered_ = 0;
    uint8_t seq_ = 2;
    bool closed_ = false;
};

constexpr uint64_t kStepUs = 4000;  // PX4's 4 ms sensor period: 8 ticks of 500 us

mav::HilSensor sensor_at(uint64_t k) {  // k = 1, 2, ...: the state at the END of step k
    mav::HilSensor s;
    s.time_usec = k * kStepUs;
    s.zacc = -9.80665f;
    s.xgyro = 0.001f * static_cast<float>(k % 7);
    s.fields_updated = 0x3F;  // accel + gyro
    return s;
}
mav::HilGps gps_at(uint64_t k) {
    mav::HilGps g;
    g.time_usec = k * kStepUs;
    g.fix_type = 3;
    g.lat = 473977418;
    g.lon = 85455939;
    g.alt = 488000;
    g.satellites_visible = 10;
    return g;
}

spade::Result<Px4Link> open_with(Script script, std::shared_ptr<Seen> seen, Px4LinkConfig cfg = {}) {
    return Px4Link::open(std::make_unique<FakePx4>(script, std::move(seen)), cfg);
}

}  // namespace

TEST(FcLinkPx4, OpeningWaitsForPx4sFirstHeartbeat) {
    auto seen = std::make_shared<Seen>();
    auto link = open_with({}, seen);
    ASSERT_TRUE(link.has_value()) << link.error().context;
    EXPECT_EQ(link->report().heartbeats_seen, 1u);
    EXPECT_EQ(link->report().sensor_messages, 0u);
    EXPECT_TRUE(seen->raw.empty()) << "nothing is sent before the first exchange";
}

TEST(FcLinkPx4, OpeningARefusedByASilentPx4NamesIt) {
    Script s;
    s.send_heartbeat = false;
    Px4LinkConfig cfg;
    cfg.heartbeat_timeout = 60ms;
    auto link = open_with(s, std::make_shared<Seen>(), cfg);
    ASSERT_FALSE(link.has_value());
    EXPECT_NE(link.error().context.find("px4_no_heartbeat"), std::string::npos) << link.error().context;
}

TEST(FcLinkPx4, TenThousandLockstepStepsHaveNoGap) {
    auto seen = std::make_shared<Seen>();
    auto link = open_with({}, seen);
    ASSERT_TRUE(link.has_value());
    for (uint64_t k = 1; k <= 10000; ++k) {
        const bool with_gps = (k % 50) == 0;  // the GPS every 200 ticks of 500 us = every 50 steps
        const auto r = link->exchange(sensor_at(k), with_gps ? std::optional<mav::HilGps>(gps_at(k)) : std::nullopt);
        ASSERT_TRUE(r.has_value()) << "step " << k << ": " << r.error().context;
        ASSERT_EQ(r->time_usec, k * kStepUs) << "step " << k << ": the reply answers THIS sensor message";
        ASSERT_TRUE(r->armed);
        ASSERT_TRUE(r->lockstep);
        ASSERT_FLOAT_EQ(r->controls[0], 0.1f);
        ASSERT_FLOAT_EQ(r->controls[3], 0.4f);
    }
    const Px4LinkReport& rep = link->report();
    EXPECT_EQ(rep.sensor_messages, 10000u);
    EXPECT_EQ(rep.replies, 10000u) << "one reply per sensor message: no gap";
    EXPECT_EQ(rep.max_replies_per_sensor, 1u);
    EXPECT_EQ(rep.gps_messages, 200u);
    EXPECT_EQ(rep.last_time_usec, 10000 * kStepUs);
    EXPECT_EQ(rep.heartbeats_seen, 1u);
    EXPECT_EQ(rep.command_longs_seen, 1u);
    EXPECT_EQ(rep.parser.bad_crc + rep.parser.unknown_msgid + rep.parser.bad_length, 0u);
    // what PX4 received: every sensor message in order, our sysid/compid, a sequence number that counts up and wraps
    ASSERT_EQ(seen->sensors.size(), 10000u);
    ASSERT_EQ(seen->gps.size(), 200u);
    for (size_t i = 0; i < seen->sensors.size(); ++i) ASSERT_EQ(seen->sensors[i].time_usec, (i + 1) * kStepUs);
    for (size_t i = 0; i < seen->headers.size(); ++i) {
        ASSERT_EQ(seen->headers[i].sysid, 1) << i;
        ASSERT_EQ(seen->headers[i].compid, 200) << i;
        ASSERT_EQ(seen->headers[i].seq, static_cast<uint8_t>(i & 0xFF)) << "message " << i << ": the sequence counts up and wraps at 256";
    }
}

TEST(FcLinkPx4, AReplyArrivingInPiecesStillAnswersTheStep) {
    Script s;
    s.chunk = 3;
    auto link = open_with(s, std::make_shared<Seen>());
    ASSERT_TRUE(link.has_value());
    for (uint64_t k = 1; k <= 20; ++k) {
        const auto r = link->exchange(sensor_at(k));
        ASSERT_TRUE(r.has_value()) << r.error().context;
        EXPECT_EQ(r->time_usec, k * kStepUs);
    }
}

TEST(FcLinkPx4, GarbageBeforeAReplyIsSkipped) {
    Script s;
    s.mode = Mode::garbage_first;
    auto link = open_with(s, std::make_shared<Seen>());
    ASSERT_TRUE(link.has_value());
    const auto r = link->exchange(sensor_at(1));
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_GE(link->report().parser.garbage_bytes, 4u);
}

TEST(FcLinkPx4, ASilentPx4GivesANamedRefusalAtTheTimeoutAndNeverHangs) {
    Script s;
    s.mode = Mode::silent_after;
    s.after = 3;
    Px4LinkConfig cfg;
    cfg.reply_timeout = 120ms;
    auto link = open_with(s, std::make_shared<Seen>(), cfg);
    ASSERT_TRUE(link.has_value());
    for (uint64_t k = 1; k <= 3; ++k) ASSERT_TRUE(link->exchange(sensor_at(k)).has_value());
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = link->exchange(sensor_at(4));
    const auto waited = std::chrono::steady_clock::now() - t0;
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("px4_no_reply"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find(std::to_string(4 * kStepUs)), std::string::npos) << "the text names the last time_usec: " << r.error().context;
    EXPECT_GE(waited, 120ms) << "it waited the whole timeout";
    EXPECT_LT(waited, 3s) << "and not much longer";
    EXPECT_GE(link->report().wall_wait_max, 120ms);
    EXPECT_EQ(link->report().replies, 3u);
    EXPECT_EQ(link->report().sensor_messages, 4u);
}

TEST(FcLinkPx4, ATimeThatDoesNotIncreaseIsRefusedBeforeAnythingIsSent) {
    auto seen = std::make_shared<Seen>();
    auto link = open_with({}, seen);
    ASSERT_TRUE(link.has_value());
    ASSERT_TRUE(link->exchange(sensor_at(2)).has_value());
    const size_t sent = seen->raw.size();
    const auto same = link->exchange(sensor_at(2));
    ASSERT_FALSE(same.has_value());
    EXPECT_EQ(same.error().code, spade::Code::invalid_argument);
    const auto back = link->exchange(sensor_at(1));
    ASSERT_FALSE(back.has_value());
    EXPECT_EQ(back.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(seen->raw.size(), sent) << "PX4's clock is set from time_usec: nothing that could set it backwards goes out";
}

TEST(FcLinkPx4, AReplyStampedBeforeItsSensorMessageIsRefused) {
    Script s;
    s.mode = Mode::stale;
    auto link = open_with(s, std::make_shared<Seen>());
    ASSERT_TRUE(link.has_value());
    const auto r = link->exchange(sensor_at(2));
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("px4_stale_reply"), std::string::npos) << r.error().context;
}

TEST(FcLinkPx4, AClosedChannelIsNamed) {
    Script s;
    s.mode = Mode::closed_after;
    s.after = 2;
    auto link = open_with(s, std::make_shared<Seen>());
    ASSERT_TRUE(link.has_value());
    ASSERT_TRUE(link->exchange(sensor_at(1)).has_value());
    ASSERT_TRUE(link->exchange(sensor_at(2)).has_value());
    const auto r = link->exchange(sensor_at(3));
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("px4_link_closed"), std::string::npos) << r.error().context;
}

TEST(FcLinkPx4, TwoRepliesToOneSensorAreCountedAndTheFresherIsUsed) {
    Script s;
    s.mode = Mode::doubled;
    auto link = open_with(s, std::make_shared<Seen>());
    ASSERT_TRUE(link.has_value());
    const auto r = link->exchange(sensor_at(1));
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(link->report().max_replies_per_sensor, 2u) << "the gap checker reads this";
    EXPECT_FLOAT_EQ(r->controls[0], 0.15f) << "the later output of the two";
    const auto next = link->exchange(sensor_at(2));
    ASSERT_TRUE(next.has_value()) << "the doubled output of the previous step did not poison the next one: " << next.error().context;
    EXPECT_EQ(next->time_usec, 2 * kStepUs);
}

TEST(FcLinkPx4, TheSameCallsGiveTheSameBytes) {
    auto run = [] {
        auto seen = std::make_shared<Seen>();
        auto link = open_with({}, seen);
        for (uint64_t k = 1; k <= 200; ++k) (void)link->exchange(sensor_at(k), (k % 50) == 0 ? std::optional<mav::HilGps>(gps_at(k)) : std::nullopt);
        return seen->raw;
    };
    const std::vector<uint8_t> a = run();
    const std::vector<uint8_t> b = run();
    EXPECT_FALSE(a.empty());
    EXPECT_EQ(a, b) << "no clock, address or counter beyond the sequence number enters the bytes";
}
