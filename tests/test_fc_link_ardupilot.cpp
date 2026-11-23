// The ArduPilot SITL JSON wire layer (tools/fc_link/ardupilot_json.hpp), tested without ArduPilot. The golden bytes are built from the DOCUMENTED layout (plan section
// 6), not from a real capture: nothing here proves ArduPilot accepts them; that is the phase that runs ArduPilot.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "fc_link/ardupilot_json.hpp"

namespace {

using namespace spade::fc_link::ardupilot_json;

void put16(std::vector<uint8_t>& b, uint16_t v) {
    b.push_back(static_cast<uint8_t>(v & 0xFF));
    b.push_back(static_cast<uint8_t>(v >> 8));
}
void put32(std::vector<uint8_t>& b, uint32_t v) {
    for (int i = 0; i < 4; ++i) b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}

std::vector<uint8_t> packet(uint16_t magic, uint16_t rate, uint32_t count, std::size_t channels) {
    std::vector<uint8_t> b;
    put16(b, magic);
    put16(b, rate);
    put32(b, count);
    for (std::size_t i = 0; i < channels; ++i) put16(b, static_cast<uint16_t>(1000 + i * 10));
    return b;
}

TEST(FcLinkArdupilotJson, ASixteenChannelServoPacketDecodesFieldByField) {
    const std::vector<uint8_t> bytes = packet(18458, 400, 0x01020304u, 16);
    ASSERT_EQ(bytes.size(), 40u);
    ASSERT_EQ(bytes[0], 0x1A) << "18458 = 0x481A, little-endian: 1A 48";
    ASSERT_EQ(bytes[1], 0x48);
    const auto p = decode_servo_packet(bytes);
    ASSERT_TRUE(p.has_value()) << p.error().context;
    EXPECT_EQ(p->frame_rate, 400);
    EXPECT_EQ(p->frame_count, 0x01020304u);
    ASSERT_EQ(p->pwm.size(), 16u);
    EXPECT_EQ(p->pwm[0], 1000);
    EXPECT_EQ(p->pwm[15], 1150);
}

TEST(FcLinkArdupilotJson, AThirtyTwoChannelServoPacketDecodes) {
    const std::vector<uint8_t> bytes = packet(29569, 1200, 7, 32);
    ASSERT_EQ(bytes.size(), 72u);
    const auto p = decode_servo_packet(bytes);
    ASSERT_TRUE(p.has_value()) << p.error().context;
    EXPECT_EQ(p->frame_rate, 1200);
    EXPECT_EQ(p->frame_count, 7u);
    ASSERT_EQ(p->pwm.size(), 32u);
    EXPECT_EQ(p->pwm[31], 1310);
}

TEST(FcLinkArdupilotJson, APacketThatIsNotTheDocumentedShapeIsRefusedNamingWhy) {
    const auto refused = [](const std::vector<uint8_t>& b, const char* needle) {
        const auto p = decode_servo_packet(b);
        ASSERT_FALSE(p.has_value()) << needle;
        EXPECT_NE(p.error().context.find(needle), std::string::npos) << p.error().context;
    };
    refused({}, "length");
    refused(std::vector<uint8_t>(39, 0), "length");
    auto short16 = packet(18458, 400, 1, 16);
    short16.pop_back();
    refused(short16, "length");
    refused(packet(18458, 400, 1, 17), "length");
    refused(packet(0x1234, 400, 1, 16), "magic");
    refused(packet(29569, 400, 1, 16), "magic");  // the 32-channel magic on 40 bytes
    refused(packet(18458, 400, 1, 32), "magic");  // the 16-channel magic on 72 bytes
}

StateReply sample_state() {
    StateReply s;
    s.timestamp_s = 1.5;
    s.gyro_frd = {0.01, -0.02, 0.0};
    s.accel_frd = {0.0, 0.0, -9.80665};
    s.position_ned = {1.25, -2.5, -3.0};
    s.velocity_ned = {0.5, 0.0, -0.125};
    s.quaternion_wxyz = {1.0, 0.0, 0.0, 0.0};
    return s;
}

TEST(FcLinkArdupilotJson, TheReplyIsOneNewlineTerminatedObjectWithTheExactBytes) {
    const auto text = encode_state_reply(sample_state());
    ASSERT_TRUE(text.has_value()) << text.error().context;
    EXPECT_EQ(*text,
              "{\"timestamp\":1.5,\"imu\":{\"gyro\":[0.01,-0.02,0],\"accel_body\":[0,0,-9.80665]},\"position\":[1.25,-2.5,-3],"
              "\"quaternion\":[1,0,0,0],\"velocity\":[0.5,0,-0.125]}\n");
}

TEST(FcLinkArdupilotJson, NumbersAreShortestRoundTripPlainDecimalWhateverTheLocale) {
    StateReply s = sample_state();
    s.position_ned = {0.1, 1e-7, 123456.789};
    s.velocity_ned = {1e10, -0.0, 1.0 / 3.0};
    const auto text = encode_state_reply(s);
    ASSERT_TRUE(text.has_value()) << text.error().context;
    EXPECT_NE(text->find("\"position\":[0.1,0.0000001,123456.789]"), std::string::npos) << *text;
    EXPECT_NE(text->find("\"velocity\":[10000000000,-0,0.3333333333333333]"), std::string::npos) << *text;
    EXPECT_EQ(text->find(','), text->find(",\"imu\"")) << "the first comma is the one between members (a decimal comma would come earlier)";
    EXPECT_EQ(text->find("e-"), std::string::npos) << "no exponent form (the key names hold an e, so the check looks for the exponent's own signs)";
    EXPECT_EQ(text->find("e+"), std::string::npos) << *text;
    EXPECT_EQ(text->find('E'), std::string::npos) << *text;
}

TEST(FcLinkArdupilotJson, ANonFiniteValueIsRefusedNamingTheField) {
    for (const double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        StateReply s = sample_state();
        s.timestamp_s = bad;
        auto r = encode_state_reply(s);
        ASSERT_FALSE(r.has_value());
        EXPECT_NE(r.error().context.find("timestamp"), std::string::npos) << r.error().context;
        s = sample_state();
        s.accel_frd[2] = bad;
        r = encode_state_reply(s);
        ASSERT_FALSE(r.has_value());
        EXPECT_NE(r.error().context.find("accel"), std::string::npos) << r.error().context;
        s = sample_state();
        s.quaternion_wxyz[1] = bad;
        r = encode_state_reply(s);
        ASSERT_FALSE(r.has_value());
        EXPECT_NE(r.error().context.find("quaternion"), std::string::npos) << r.error().context;
    }
}

TEST(FcLinkArdupilotJson, AFrameCountThatRepeatsIsAResendAndOneThatFallsIsARestart) {
    FrameTracker t;
    EXPECT_EQ(t.observe(100), FrameKind::fresh) << "the first frame";
    EXPECT_EQ(t.observe(101), FrameKind::fresh);
    EXPECT_EQ(t.observe(101), FrameKind::resend) << "the same count again: ArduPilot re-sent the last frame";
    EXPECT_EQ(t.observe(102), FrameKind::fresh);
    EXPECT_EQ(t.skipped(), 0u);
    EXPECT_EQ(t.observe(105), FrameKind::fresh);
    EXPECT_EQ(t.skipped(), 2u) << "103 and 104 never arrived";
    EXPECT_EQ(t.observe(3), FrameKind::restart) << "a smaller count: SITL restarted";
    EXPECT_EQ(t.observe(4), FrameKind::fresh) << "counting again from the restart";
}

TEST(FcLinkArdupilotJson, ReplyTimestampsMustAdvanceByLessThanATenthOfASecond) {
    ReplyClock c;
    EXPECT_TRUE(c.accept(1.0).has_value()) << "the first reply has no previous";
    EXPECT_TRUE(c.accept(1.0025).has_value());
    EXPECT_TRUE(c.accept(1.0025 + 0.0999).has_value());
    const auto same = c.accept(1.0025 + 0.0999);
    ASSERT_FALSE(same.has_value()) << "dt == 0 is not a new frame";
    EXPECT_NE(same.error().context.find("dt"), std::string::npos) << same.error().context;
    EXPECT_TRUE(c.accept(1.0025 + 0.0999, /*resend=*/true).has_value()) << "a resend answered again carries the same timestamp";
    EXPECT_FALSE(c.accept(1.0025 + 0.0999 + 0.1001).has_value()) << "dt of more than 0.1 s is ignored by ArduPilot";
    EXPECT_FALSE(c.accept(1.0).has_value()) << "going backwards is a physics reset in ArduPilot: only after reset()";
    EXPECT_TRUE(c.accept(1.0025 + 0.0999 + 0.05).has_value()) << "a refusal changed nothing: the previous is still 1.1024";
}

TEST(FcLinkArdupilotJson, AfterAResetTheNextReplyHasNoPrevious) {
    ReplyClock c;
    ASSERT_TRUE(c.accept(50.0).has_value());
    c.reset();
    EXPECT_TRUE(c.accept(0.0).has_value()) << "ArduPilot restarted: time starts over";
    EXPECT_TRUE(c.accept(0.0025).has_value());
}

}  // namespace
