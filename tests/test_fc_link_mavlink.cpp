// tests/test_fc_link_mavlink.cpp -- the autopilot links' MAVLink 2 codec (tools/fc_link/mavlink.hpp) against frames a REFERENCE made.
//
// The golden frames below were made by pymavlink 2.4.49 (the v20 common dialect) run INSIDE the pinned PX4 build image
// (px4io/px4-dev@sha256:82e3ddcf...), network off, by tools/fc_link/golden/golden_gen.py with sysid 1, compid 200 and the sequence
// number shown. Every input is spelt out here beside its bytes, so a failure names the message and the field that is wrong. What they pin
// that a hand-written encoder gets wrong: the field order on the wire (HIL_ACTUATOR_CONTROLS puts `flags` before `controls`), the
// extension fields (HIL_GPS's `yaw` lands unaligned at offset 37, `id` after the last base field), the trailing-zero truncation on send
// and its zero-fill on receive, the 24-bit message id, and each message's CRC_EXTRA.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "../tools/fc_link/mavlink.hpp"

namespace {

using namespace spade::fc_link::mavlink;

std::vector<uint8_t> from_hex(const std::string& hex) {
    std::vector<uint8_t> out;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) out.push_back(static_cast<uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    return out;
}

struct Golden {
    const char* name;
    uint8_t seq;
    Message message;
    const char* hex;
    size_t payload_len;  // as it is on the wire, after the zero truncation
};

std::vector<Golden> golden() {
    std::vector<Golden> g;
    g.push_back({"heartbeat", 0, Heartbeat{0x01020304u, 2, 12, 0x81, 4, 3}, "fd0900000001c800000004030201020c8104037a2a", 9});
    g.push_back({"heartbeat_all_zero_but_version", 1, Heartbeat{0, 0, 0, 0, 0, 3}, "fd0900000101c8000000000000000000000003997b", 9});
    {
        CommandLong c;
        c.param[0] = 115.0f;
        c.param[1] = 5000.0f;
        c.command = 511;
        c.target_system = 1;
        c.target_component = 1;
        c.confirmation = 0;
        g.push_back({"command_long_set_message_interval", 2, c,
                     "fd2000000201c84c00000000e64200409c450000000000000000000000000000000000000000ff0101019d9e", 32});
    }
    {
        HilSensor s;
        s.time_usec = 1234567890123ull;
        s.xacc = 0.25f; s.yacc = -0.5f; s.zacc = -9.80665f;
        s.xgyro = 0.01f; s.ygyro = -0.02f; s.zgyro = 0.03f;
        s.xmag = 0.2f; s.ymag = 0.01f; s.zmag = 0.4f;
        s.abs_pressure = 1013.25f; s.diff_pressure = 0.0f; s.pressure_alt = 0.0f; s.temperature = 15.0f;
        s.fields_updated = 0x1fff;
        s.id = 0;
        g.push_back({"hil_sensor_full", 3, s,
                     "fd3e00000301c86b0000cb04fb711f0100000000803e000000bf0ae81cc10ad7233c0ad7a3bc8fc2f53ccdcc4c3e0ad7233ccdcccc3e"
                     "00507d44000000000000000000007041ff1f1b88",
                     62});
    }
    {
        HilSensor s;
        s.time_usec = 4000;
        s.zacc = -9.80665f;
        s.fields_updated = 0x3f;
        s.id = 1;
        g.push_back({"hil_sensor_imu_only_id1", 4, s,
                     "fd4100000401c86b0000a00f00000000000000000000000000000ae81cc1000000000000000000000000000000000000000000000000"
                     "000000000000000000000000000000003f00000001a2be",
                     65});
    }
    {
        HilGps p;
        p.time_usec = 987654321; p.fix_type = 3; p.lat = 473977418; p.lon = 85455939; p.alt = 488000;
        p.eph = 100; p.epv = 150; p.vel = 523; p.vn = 300; p.ve = -400; p.vd = -50; p.cog = 31500;
        p.satellites_visible = 10; p.id = 0; p.yaw = 4500;
        g.push_back({"hil_gps_with_yaw", 5, p,
                     "fd2700000501c8710000b168de3a000000004a52401c43f4170540720700640096000b022c0170feceff0c7b030a0094113155", 39});
    }
    {
        HilGps p;
        p.time_usec = 1000; p.fix_type = 3; p.lat = -335000000; p.lon = -1511000000; p.alt = -2000;
        p.eph = 65535; p.epv = 65535; p.cog = 65535; p.satellites_visible = 255; p.id = 0; p.yaw = 0;
        g.push_back({"hil_gps_no_extension_bytes", 6, p,
                     "fd2400000601c8710000e803000000000000404e08ec40f8efa530f8ffffffffffff0000000000000000ffff03ff60d7", 36});
    }
    {
        HilActuatorControls a;
        a.time_usec = 8000;
        a.controls[0] = 0.5f; a.controls[1] = 0.6f; a.controls[2] = 0.7f; a.controls[3] = 0.8f;
        a.mode = 129;
        a.flags = 1;
        g.push_back({"hil_actuator_controls_armed", 7, a,
                     "fd5100000701c85d0000401f00000000000001000000000000000000003f9a99193f3333333fcdcc4c3f000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000814591",
                     81});
    }
    {
        HilActuatorControls a;
        a.time_usec = 12000;  // everything else zero: the payload is cut to the two bytes of the time
        g.push_back({"hil_actuator_controls_disarmed_truncated", 8, a, "fd0200000801c85d0000e02ed1e0", 2});
    }
    {
        HilActuatorControls a;
        a.time_usec = 16000;
        a.controls = {1.0f, 0.25f, 0.0f, 0.75f, 0, 0, 0, 0, -1.0f, 0, 0, 0, 0, 0, 0, 0.125f};
        a.mode = 129;
        a.flags = 1;
        g.push_back({"hil_actuator_controls_servo_channels", 9, a,
                     "fd5100000901c85d0000803e00000000000001000000000000000000803f0000803e000000000000403f000000000000000000000000"
                     "00000000000080bf0000000000000000000000000000000000000000000000000000003e81f1de",
                     81});
    }
    {
        HilStateQuaternion q;
        q.time_usec = 2000000;
        q.attitude_quaternion = {0.70710678f, 0.0f, 0.0f, -0.70710678f};
        q.rollspeed = 0.1f; q.pitchspeed = -0.2f; q.yawspeed = 0.3f;
        q.lat = 473977418; q.lon = 85455939; q.alt = 488000;
        q.vx = 100; q.vy = -200; q.vz = -300;
        q.zacc = -1000;
        g.push_back({"hil_state_quaternion", 10, q,
                     "fd4000000a01c873000080841e0000000000f304353f0000000000000000f30435bfcdcccc3dcdcc4cbe9a99993e4a52401c43f41705"
                     "40720700640038ffd4fe000000000000000018fc84c7",
                     64});
    }
    return g;
}

// Parse everything `bytes` holds, however it was chunked.
std::vector<Frame> parse_all(const std::vector<uint8_t>& bytes, size_t chunk, Parser::Stats* stats = nullptr) {
    Parser p;
    std::vector<Frame> out;
    for (size_t i = 0; i < bytes.size(); i += chunk) {
        const size_t n = std::min(chunk, bytes.size() - i);
        p.feed(std::span<const uint8_t>(bytes.data() + i, n));
        while (auto f = p.next()) out.push_back(std::move(*f));
    }
    if (stats) *stats = p.stats();
    return out;
}

}  // namespace

TEST(FcLinkMavlink, TheCrcMatchesItsPublishedCheckValue) {
    const std::string s = "123456789";
    EXPECT_EQ(crc16(std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(s.data()), s.size())), 0x6F91) << "CRC-16/MCRF4XX of \"123456789\"";
    EXPECT_EQ(crc16(std::span<const uint8_t>{}), 0xFFFF) << "the empty message leaves the initial value";
}

TEST(FcLinkMavlink, EachMessageCarriesTheCrcExtraAndLengthTheReferenceUses) {
    struct Row { uint32_t id; uint8_t extra; size_t max_len; };
    const Row rows[] = {{0, 50, 9}, {76, 152, 33}, {93, 47, 81}, {107, 108, 65}, {113, 124, 39}, {115, 4, 64}};
    for (const Row& r : rows) {
        ASSERT_TRUE(crc_extra(r.id).has_value()) << "message " << r.id;
        EXPECT_EQ(*crc_extra(r.id), r.extra) << "CRC_EXTRA of message " << r.id;
        ASSERT_TRUE(max_payload_len(r.id).has_value()) << "message " << r.id;
        EXPECT_EQ(*max_payload_len(r.id), r.max_len) << "longest payload of message " << r.id;
    }
    EXPECT_FALSE(crc_extra(999).has_value()) << "a message we do not carry has no CRC_EXTRA";
    EXPECT_FALSE(max_payload_len(999).has_value());
}

TEST(FcLinkMavlink, EncodingMakesTheReferencesBytes) {
    for (const Golden& g : golden()) {
        const std::vector<uint8_t> got = encode(Header{g.seq, 1, 200}, g.message);
        EXPECT_EQ(got, from_hex(g.hex)) << g.name;
        ASSERT_GE(got.size(), kHeaderLen + kChecksumLen) << g.name;
        EXPECT_EQ(got[1], g.payload_len) << g.name << ": payload length after the trailing zeros are dropped";
    }
}

TEST(FcLinkMavlink, ParsingTheReferencesBytesGivesBackTheMessages) {
    for (const Golden& g : golden()) {
        const std::vector<uint8_t> bytes = from_hex(g.hex);
        for (size_t chunk : {bytes.size(), size_t{1}, size_t{3}, size_t{7}}) {
            Parser::Stats stats;
            const std::vector<Frame> frames = parse_all(bytes, chunk, &stats);
            ASSERT_EQ(frames.size(), 1u) << g.name << " in chunks of " << chunk;
            EXPECT_EQ(frames[0].header, (Header{g.seq, 1, 200})) << g.name;
            EXPECT_EQ(frames[0].msgid, msgid_of(g.message)) << g.name;
            EXPECT_EQ(frames[0].payload.size(), g.payload_len) << g.name << ": delivered as on the wire, not zero-filled";
            const spade::Result<Message> m = decode(frames[0]);
            ASSERT_TRUE(m.has_value()) << g.name;
            EXPECT_TRUE(*m == g.message) << g.name << ": every field, after the zero-fill";
            EXPECT_EQ(stats.frames, 1u) << g.name;
            EXPECT_EQ(stats.bad_crc + stats.garbage_bytes + stats.unknown_msgid + stats.bad_length, 0u) << g.name;
        }
    }
}

TEST(FcLinkMavlink, ATruncatedPayloadIsZeroFilledOnReceive) {
    const std::vector<Frame> frames = parse_all(from_hex("fd0200000801c85d0000e02ed1e0"), 5);
    ASSERT_EQ(frames.size(), 1u);
    ASSERT_EQ(frames[0].payload.size(), 2u);
    const spade::Result<Message> m = decode(frames[0]);
    ASSERT_TRUE(m.has_value());
    const auto* a = std::get_if<HilActuatorControls>(&*m);
    ASSERT_NE(a, nullptr);
    EXPECT_EQ(a->time_usec, 12000u) << "0x2ee0, the two bytes that were sent";
    EXPECT_EQ(a->mode, 0);
    EXPECT_EQ(a->flags, 0u);
    for (float c : a->controls) EXPECT_EQ(c, 0.0f);
}

TEST(FcLinkMavlink, AFlippedBitIsRefusedAndCounted) {
    for (const Golden& g : golden()) {
        std::vector<uint8_t> bytes = from_hex(g.hex);
        bytes[kHeaderLen] ^= 0x10;  // first payload byte
        Parser::Stats stats;
        const std::vector<Frame> frames = parse_all(bytes, bytes.size(), &stats);
        EXPECT_TRUE(frames.empty()) << g.name << ": a corrupt payload is not delivered";
        EXPECT_EQ(stats.bad_crc, 1u) << g.name;
    }
}

TEST(FcLinkMavlink, TheParserFindsFramesInGarbageAndBackToBack) {
    const std::vector<Golden> g = golden();
    std::vector<uint8_t> stream = {0x00, 0xFF, 0x12};  // noise, no STX
    const std::vector<uint8_t> hb = from_hex(g[0].hex);
    stream.insert(stream.end(), hb.begin(), hb.end());
    stream.push_back(0xFD);  // a stray STX that is not a frame
    stream.push_back(0x01);
    const std::vector<uint8_t> act = from_hex(g[7].hex);
    stream.insert(stream.end(), act.begin(), act.end());
    const std::vector<uint8_t> act2 = from_hex(g[8].hex);
    stream.insert(stream.end(), act2.begin(), act2.end());
    Parser::Stats stats;
    const std::vector<Frame> frames = parse_all(stream, 4, &stats);
    ASSERT_EQ(frames.size(), 3u) << "the heartbeat and both actuator frames, in arrival order";
    EXPECT_EQ(frames[0].msgid, 0u);
    EXPECT_EQ(frames[1].msgid, 93u);
    EXPECT_EQ(frames[2].msgid, 93u);
    EXPECT_EQ(frames[2].header.seq, 8) << "the disarmed frame's own sequence number";
    EXPECT_GE(stats.garbage_bytes, 3u);
}

TEST(FcLinkMavlink, AnUnknownMessageIsDroppedNotGuessed) {
    // A well-formed frame of message 30 (ATTITUDE, not carried here): its CRC_EXTRA is unknown to us, so it cannot be checked.
    std::vector<uint8_t> f = {0xFD, 4, 0, 0, 1, 1, 1, 30, 0, 0, 1, 2, 3, 4, 0, 0};
    Parser::Stats stats;
    const std::vector<Frame> frames = parse_all(f, f.size(), &stats);
    EXPECT_TRUE(frames.empty());
    EXPECT_EQ(stats.unknown_msgid, 1u);
    EXPECT_FALSE(decode(Frame{Header{}, 30, {1, 2, 3, 4}}).has_value()) << "decode: not_found for a message we do not carry";
    EXPECT_EQ(decode(Frame{Header{}, 30, {1}}).error().code, spade::Code::not_found);
}

TEST(FcLinkMavlink, ALongerPayloadThanTheMessageHasIsRefused) {
    // A heartbeat claiming 10 payload bytes (its longest is 9), with a CRC that is right for what it says.
    std::vector<uint8_t> f = {0xFD, 10, 0, 0, 0, 1, 1, 0, 0, 0};
    for (int i = 0; i < 10; ++i) f.push_back(static_cast<uint8_t>(i + 1));
    uint16_t crc = crc16(std::span<const uint8_t>(f.data() + 1, f.size() - 1));
    const uint8_t extra = 50;
    crc = crc16(std::span<const uint8_t>(&extra, 1), crc);
    f.push_back(static_cast<uint8_t>(crc & 0xFF));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    Parser::Stats stats;
    const std::vector<Frame> frames = parse_all(f, f.size(), &stats);
    EXPECT_TRUE(frames.empty());
    EXPECT_EQ(stats.bad_length, 1u);
}

TEST(FcLinkMavlink, ASignedFrameIsAcceptedWithItsSignatureSkippedAndAnUnknownFlagIsNot) {
    const std::vector<uint8_t> plain = from_hex(golden()[7].hex);  // the armed actuator frame, 81 payload bytes
    auto resign = [&](uint8_t incompat, size_t sig_bytes) {
        std::vector<uint8_t> f(plain.begin(), plain.end() - 2);
        f[2] = incompat;
        uint16_t crc = crc16(std::span<const uint8_t>(f.data() + 1, f.size() - 1));
        const uint8_t extra = 47;
        crc = crc16(std::span<const uint8_t>(&extra, 1), crc);
        f.push_back(static_cast<uint8_t>(crc & 0xFF));
        f.push_back(static_cast<uint8_t>(crc >> 8));
        for (size_t i = 0; i < sig_bytes; ++i) f.push_back(static_cast<uint8_t>(0xA0 + i));
        return f;
    };
    {
        std::vector<uint8_t> stream = resign(0x01, kSignatureLen);
        const std::vector<uint8_t> next = from_hex(golden()[8].hex);
        stream.insert(stream.end(), next.begin(), next.end());
        Parser::Stats stats;
        const std::vector<Frame> frames = parse_all(stream, 6, &stats);
        ASSERT_EQ(frames.size(), 2u) << "the signed frame, then the plain one right behind its signature";
        EXPECT_EQ(frames[0].msgid, 93u);
        EXPECT_EQ(frames[1].header.seq, 8);
        EXPECT_EQ(stats.unsupported_flags, 0u);
    }
    {
        Parser::Stats stats;
        const std::vector<Frame> frames = parse_all(resign(0x02, 0), 81 + 12, &stats);
        EXPECT_TRUE(frames.empty()) << "an incompat flag we do not understand: the frame must be discarded";
        EXPECT_EQ(stats.unsupported_flags, 1u);
    }
}

TEST(FcLinkMavlink, TheMessageIdIsThreeBytesLowFirst) {
    EXPECT_EQ(msgid_of(Message{HilSensor{}}), 107u);
    const std::vector<uint8_t> f = encode(Header{3, 1, 200}, Message{HilGps{}});
    ASSERT_GE(f.size(), kHeaderLen);
    EXPECT_EQ(f[7], 113);
    EXPECT_EQ(f[8], 0);
    EXPECT_EQ(f[9], 0);
}
