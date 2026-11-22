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
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "../tools/fc_link/mavlink.hpp"
#include "../tools/fc_link/px4_link.hpp"

namespace {

using namespace spade::fc_link;
using namespace std::chrono_literals;
namespace mav = spade::fc_link::mavlink;

enum class Mode { normal, silent_after, stale, doubled, closed_after, garbage_first, silent_first };

struct Script {
    Mode mode = Mode::normal;
    uint64_t after = 0;           // silent_after / closed_after: sensors answered before the change
    size_t chunk = 0;             // 0 = deliver everything queued in one read, else this many bytes per read
    bool send_heartbeat = true;   // PX4's first message on connecting
    bool backlog = false;         // silent_first: when it finally answers, it answers every sensor message it had held, in order
    std::chrono::milliseconds write_delay{0};  // write() sleeps this long AFTER queueing its answer: the reply has arrived, and the link's deadline has passed, before the link reads
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
        if (script_.write_delay.count() > 0) std::this_thread::sleep_for(script_.write_delay);
        return {};
    }

    spade::Result<std::vector<uint8_t>> read_some(std::chrono::milliseconds timeout) override {
        if (closed_ && inbox_.empty()) return std::unexpected(spade::Error{spade::Code::io_error, "peer closed"});
        if (inbox_.empty()) {
            // This fake answers inside write() (no thread), so nothing can arrive while the link waits: sleeping only added the OS timer quantum (a 2 ms
            // sleep took ~15.6 ms on Windows, so a 300-boot-step test took 5 s alone and timed out at 60 s under load). Yield and let the link's own
            // deadline end the wait.
            (void)timeout;
            std::this_thread::yield();
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
        if (script_.mode == Mode::silent_first) {  // a PX4 still booting: no output for the first `after` sensor messages
            if (n < script_.after) {
                if (script_.backlog) pending_.push_back(s.time_usec);
                return;
            }
            for (const uint64_t t : pending_) queue(mav::Header{static_cast<uint8_t>(seq_++), 1, 1}, controls_for(t, 0.0f));
            pending_.clear();
            queue(mav::Header{static_cast<uint8_t>(seq_++), 1, 1}, controls_for(s.time_usec, 0.0f));
            return;
        }
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
    std::vector<uint64_t> pending_;
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

// ---- the boot phase: PX4's boot runs on the lockstep clock, so it answers nothing until it is up (first real run, 2026-11-21) ----

TEST(FcLinkPx4, UntilPx4sFirstReplyTheLinkReportsBootingAndKeepsSending) {
    Script s;
    s.mode = Mode::silent_first;
    s.after = 3;
    auto seen = std::make_shared<Seen>();
    auto link = open_with(s, seen);
    ASSERT_TRUE(link.has_value());
    for (uint64_t k = 1; k <= 3; ++k) {
        const auto r = link->exchange(sensor_at(k));
        ASSERT_TRUE(r.has_value()) << "boot step " << k << ": " << r.error().context;
        EXPECT_TRUE(r->booting);
        EXPECT_FALSE(r->armed);
        for (const float c : r->controls) EXPECT_EQ(c, 0.0f) << "no controls while PX4 boots";
    }
    EXPECT_EQ(link->report().replies, 0u);
    EXPECT_EQ(link->report().boot_steps, 3u);
    EXPECT_EQ(link->report().boot_end_time_usec, 0u) << "not ended yet";
    const auto first = link->exchange(sensor_at(4));
    ASSERT_TRUE(first.has_value()) << first.error().context;
    EXPECT_FALSE(first->booting);
    EXPECT_EQ(first->time_usec, 4 * kStepUs);
    EXPECT_TRUE(first->armed);
    EXPECT_EQ(link->report().boot_steps, 3u);
    EXPECT_EQ(link->report().boot_end_time_usec, 4 * kStepUs) << "Spade's tick 0 is the step that answers PX4's first output";
    EXPECT_EQ(link->report().replies, 1u);
    ASSERT_EQ(seen->sensors.size(), 4u);
    for (size_t i = 0; i < 4; ++i) EXPECT_EQ(seen->sensors[i].time_usec, (i + 1) * kStepUs) << "boot sensors advance the time like any other";
}

TEST(FcLinkPx4, ABacklogAnsweredWithTheFirstReplyIsCaughtUpNotRefusedAsStale) {
    Script s;
    s.mode = Mode::silent_first;
    s.after = 3;
    s.backlog = true;
    auto link = open_with(s, std::make_shared<Seen>());
    ASSERT_TRUE(link.has_value());
    for (uint64_t k = 1; k <= 3; ++k) ASSERT_TRUE(link->exchange(sensor_at(k))->booting);
    const auto first = link->exchange(sensor_at(4));
    ASSERT_TRUE(first.has_value()) << first.error().context;
    EXPECT_FALSE(first->booting);
    EXPECT_EQ(first->time_usec, 4 * kStepUs) << "the reply that counts answers the CURRENT sensor message";
    EXPECT_EQ(link->report().catchup_replies, 3u);
    EXPECT_EQ(link->report().replies, 1u);
    EXPECT_EQ(link->report().max_replies_per_sensor, 1u) << "the gap checker does not see the boot backlog as a doubled reply";
    const auto next = link->exchange(sensor_at(5));
    ASSERT_TRUE(next.has_value()) << next.error().context;
    EXPECT_EQ(next->time_usec, 5 * kStepUs);
}

TEST(FcLinkPx4, AFirstReplyThatIsStaleWithNothingBeforeItIsStillRefused) {
    Script s;
    s.mode = Mode::stale;  // every reply is a step behind; with no boot backlog that is a fault, not a catch-up
    auto link = open_with(s, std::make_shared<Seen>());
    ASSERT_TRUE(link.has_value());
    const auto r = link->exchange(sensor_at(2));
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("px4_stale_reply"), std::string::npos) << r.error().context;
}

TEST(FcLinkPx4, APx4ThatNeverStartsIsRefusedByNameWithTheBootCount) {
    Script s;
    s.mode = Mode::silent_first;
    s.after = 1000000;
    Px4LinkConfig cfg;
    cfg.boot_timeout = 250ms;
    cfg.boot_poll = 5ms;
    auto link = open_with(s, std::make_shared<Seen>(), cfg);
    ASSERT_TRUE(link.has_value());
    const auto t0 = std::chrono::steady_clock::now();
    uint64_t k = 0;
    spade::Result<ActuatorReply> r = ActuatorReply{};
    do {
        r = link->exchange(sensor_at(++k));
        ASSERT_LT(k, 100000u) << "it must end";
    } while (r.has_value());
    const auto waited = std::chrono::steady_clock::now() - t0;
    EXPECT_NE(r.error().context.find("px4_no_reply"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("never started"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find(std::to_string(link->report().boot_steps)), std::string::npos) << "the text carries the boot step count: " << r.error().context;
    EXPECT_GE(link->report().boot_steps, 1u);
    EXPECT_GE(waited, 250ms);
    EXPECT_LT(waited, 5s);
    EXPECT_EQ(link->report().replies, 0u);
}

namespace {

// A stand-in for "Spade's world": its state is held while PX4 boots, and from the first reply on it is a function of PX4's replies only. The digest
// folds the world's state and the controls it was given, tick by tick, and NEVER the boot length or the link's absolute time.
uint64_t world_digest_after_boot(uint64_t boot_steps_px4_takes, uint64_t* boot_steps_seen, uint64_t* boot_end_us) {
    Script s;
    s.mode = Mode::silent_first;
    s.after = boot_steps_px4_takes;
    s.backlog = (boot_steps_px4_takes % 2) == 0;  // both kinds of PX4: one that answers the backlog and one that does not
    Px4LinkConfig cfg;
    cfg.boot_poll = std::chrono::milliseconds(1);  // the boot length is counted in steps, not wall time: 300 steps cost ~0.3 s, not 5 s
    auto link = open_with(s, std::make_shared<Seen>(), cfg);
    if (!link) return 0;
    double world = 0.0;  // the held spawn state
    uint64_t digest = 1469598103934665603ULL;
    const auto fold = [&](double v) {
        uint64_t bits;
        static_assert(sizeof(bits) == sizeof(v));
        std::memcpy(&bits, &v, sizeof(bits));
        digest = (digest ^ bits) * 1099511628211ULL;
    };
    uint64_t link_step = 0;
    bool started = false;
    uint64_t tick = 0;
    while (tick < 200) {
        mav::HilSensor sensor = sensor_at(++link_step);
        sensor.zacc = static_cast<float>(-9.80665 + world);  // built from the held (or running) state
        const auto r = link->exchange(sensor);
        if (!r) return 0;
        if (r->booting) continue;  // the world is NOT advanced while PX4 boots
        started = true;
        world += static_cast<double>(r->controls[0]) * 0.001;  // tick `tick` of the world: advanced by PX4's reply
        fold(world);
        fold(static_cast<double>(r->controls[3]));
        ++tick;
    }
    (void)started;
    *boot_steps_seen = link->report().boot_steps;
    *boot_end_us = link->report().boot_end_time_usec;
    return digest;
}

}  // namespace

TEST(FcLinkPx4, TheWorldAfterBootIsTheSameWhetherPx4TookThreeStepsOrThreeHundred) {
    uint64_t boot_a = 0, boot_b = 0, end_a = 0, end_b = 0;
    const uint64_t a = world_digest_after_boot(3, &boot_a, &end_a);
    const uint64_t b = world_digest_after_boot(300, &boot_b, &end_b);
    ASSERT_NE(a, 0u);
    ASSERT_NE(b, 0u);
    EXPECT_EQ(boot_a, 3u);
    EXPECT_EQ(boot_b, 300u);
    EXPECT_NE(end_a, end_b) << "the link time offset differs: it is recorded, not hidden";
    EXPECT_EQ(a, b) << "same function of PX4's replies from the world's tick 0, whatever PX4's boot length";
}

// The airframe's data-link default (found in the first real run: PX4 refused to arm with "Preflight Fail: No connection to the GCS"). In PX4 v1.17.0's
// rcAndDataLinkCheck.cpp a ground station is REQUIRED to arm only when NAV_DLL_ACT is above 0, and PX4's own default is 0 (Disabled). A headless
// lockstep run has no ground station, so airframe 10090 must not turn the requirement on. Text check on the file the image is built from.
TEST(FcLinkPx4, TheAirframeDoesNotRequireAGroundStation) {
    const std::filesystem::path file = std::filesystem::path(SPADE_REPO_DIR) / "tools" / "fc_link" / "px4" / "airframes" / "10090_spade_x500_px4";
    std::ifstream in(file);
    ASSERT_TRUE(in.good()) << "cannot read " << file.string();
    std::string line;
    int found = 0;
    long value = -1;
    while (std::getline(in, line)) {
        const std::size_t hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);  // a comment is not a setting
        const std::string key = "param set-default NAV_DLL_ACT";
        const std::size_t at = line.find(key);
        if (at == std::string::npos) continue;
        ++found;
        value = std::strtol(line.c_str() + at + key.size(), nullptr, 10);
    }
    EXPECT_EQ(found, 1) << "airframe 10090 sets NAV_DLL_ACT " << found << " times (expected once, explicitly)";
    EXPECT_EQ(value, 0) << "NAV_DLL_ACT is " << value << ": above 0 makes PX4 require a ground station to arm";
}

// The ordering bug gate 49b's debug run found (the 1 ms poll exposed it): Px4Link::exchange took the clock for its deadline BEFORE it sent the
// sensor message, ran take() (the parser's buffer only) and checked the deadlines, and only then read the channel. When the deadline had already
// passed on entry (a slow send, a slow debug build), it returned `booting` (or px4_no_reply) without ever reading a reply that was already
// readable. These fakes make that deterministic: write() sleeps 5 ms after it has queued its answer.
TEST(FcLinkPx4, AReplyThatArrivedDuringTheSendIsNotCountedAsBootSilence) {
    Script s;
    s.mode = Mode::silent_first;
    s.after = 3;  // the first three sensor messages get no answer; the fourth does
    s.write_delay = 5ms;
    Px4LinkConfig cfg;
    cfg.boot_poll = 1ms;  // the deadline is long past by the time the send returns
    auto link = open_with(s, std::make_shared<Seen>(), cfg);
    ASSERT_TRUE(link.has_value());
    uint64_t booting = 0;
    bool answered = false;
    for (uint64_t k = 1; k <= 4; ++k) {
        const auto r = link->exchange(sensor_at(k));
        ASSERT_TRUE(r.has_value()) << r.error().context;
        if (r->booting) ++booting;
        else answered = true;
    }
    EXPECT_EQ(booting, 3u) << "the fourth step's reply was in the inbox: it is a reply, not a boot step";
    EXPECT_TRUE(answered);
    EXPECT_EQ(link->report().boot_steps, 3u);
    EXPECT_EQ(link->report().boot_end_time_usec, 4 * kStepUs) << "boot ended at the fourth sensor message (16000 us)";
    EXPECT_EQ(link->report().catchup_replies, 0u);
}

// The same ordering on the strict side: after boot, a reply that has ARRIVED must never be reported as px4_no_reply.
TEST(FcLinkPx4, AReplyThatHasArrivedIsNeverReportedMissing) {
    Script s;
    s.write_delay = 5ms;
    Px4LinkConfig cfg;
    cfg.reply_timeout = 1ms;  // shorter than the send itself
    cfg.boot_poll = 500ms;    // the first exchange must end boot by reading its reply: a 5 ms sleep can take 16 ms on Windows' timer quantum
    auto link = open_with(s, std::make_shared<Seen>(), cfg);
    ASSERT_TRUE(link.has_value());
    const auto first = link->exchange(sensor_at(1));
    ASSERT_TRUE(first.has_value()) << first.error().context;
    ASSERT_FALSE(first->booting) << "boot ended with the first reply, so the steps below are on the strict side";
    for (uint64_t k = 2; k <= 5; ++k) {
        const auto r = link->exchange(sensor_at(k));
        ASSERT_TRUE(r.has_value()) << "step " << k << ": " << r.error().context;
        EXPECT_FALSE(r->booting);
    }
    EXPECT_EQ(link->report().replies, 5u);
}
