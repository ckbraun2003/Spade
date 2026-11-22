// test_fc_host_trace.cpp -- the lockstep host's trace and replay (tools/fc_host/lockstep.hpp: trace.bin v1, commands.bin v1, the rolling hash).
// The layout tests read the bytes with their own little-endian reader and recompute the hash chain with world/sha256.hpp: nothing here shares code
// with the writer, so the format is pinned by the test and not by the implementation (TD-4).
#include <gtest/gtest.h>

#include "fc_host/lockstep.hpp"
#include "fc_host_test_rig.hpp"
#include "physics/blade_azimuth.hpp"
#include "world/sha256.hpp"

namespace {

using namespace fc_host_test;

[[nodiscard]] uint32_t rd_u32(const std::vector<std::byte>& b, std::size_t at) {
    uint32_t v = 0;
    for (int i = 3; i >= 0; --i) v = (v << 8) | static_cast<uint32_t>(std::to_integer<uint8_t>(b[at + static_cast<std::size_t>(i)]));
    return v;
}
[[nodiscard]] uint64_t rd_u64(const std::vector<std::byte>& b, std::size_t at) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | static_cast<uint64_t>(std::to_integer<uint8_t>(b[at + static_cast<std::size_t>(i)]));
    return v;
}
[[nodiscard]] float rd_f32(const std::vector<std::byte>& b, std::size_t at) { return std::bit_cast<float>(rd_u32(b, at)); }
[[nodiscard]] std::string rd_text(const std::vector<std::byte>& b, std::size_t at, std::size_t n) {
    std::string s;
    for (std::size_t i = 0; i < n; ++i) s.push_back(static_cast<char>(std::to_integer<uint8_t>(b[at + i])));
    return s;
}

constexpr float kCmd[4] = {500.0f, 520.0f, 500.0f, 480.0f};

// Steps `cycles` sensor periods of 8 ticks, commanding every cycle, taking samples after each.
[[nodiscard]] bool drive(LockstepHost& host, int cycles, bool take_each_cycle = true) {
    for (int cycle = 1; cycle <= cycles; ++cycle) {
        const float cmd[4] = {kCmd[0] + static_cast<float>(cycle), kCmd[1], kCmd[2] - static_cast<float>(cycle), kCmd[3]};
        if (!host.command(cmd)) return false;
        if (!host.advance_to(static_cast<uint64_t>(cycle) * 8 * kDt)) return false;
        if (take_each_cycle && !host.take_sensors()) return false;
    }
    return true;
}

// ===========================================================================
// trace.bin v1
// ===========================================================================

TEST(FcHostTrace, TheHeaderIsPinnedByteForByte) {
    std::optional<Rig> rig = make_rig(8);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    ASSERT_TRUE(drive(*host, 3));  // 24 ticks: 3 imu samples, no baro (20 ms) and 2 mag samples (10 ms) at ticks 19 and ... (mag every 20 ticks: tick 19)
    const spade::Result<std::vector<std::byte>> file = host->trace_bytes();
    ASSERT_OK(file);
    const std::vector<std::byte>& b = *file;
    ASSERT_GE(b.size(), 192u + 64u);
    EXPECT_EQ(rd_text(b, 0, 4), "SPTR");
    EXPECT_EQ(rd_u32(b, 4), 1u);
    EXPECT_EQ(rd_u32(b, 8), 192u);
    EXPECT_EQ(rd_u32(b, 12), 0u);
    EXPECT_EQ(rd_u64(b, 16), kDt);
    EXPECT_EQ(rd_u32(b, 24), 1u);
    EXPECT_EQ(rd_u32(b, 28), 4u);
    EXPECT_EQ(rd_u64(b, 32), 24u);
    EXPECT_EQ(rd_u64(b, 40), 0u);
    EXPECT_EQ(rd_u32(b, 48), 76u + 12u * 4u);
    EXPECT_EQ(rd_u32(b, 52), 16u);
    EXPECT_EQ(rd_u64(b, 56), 192u);
    EXPECT_EQ(rd_u64(b, 64), 192u + 24u * (76u + 12u * 4u));
    const uint64_t samples = rd_u64(b, 72);
    EXPECT_EQ(samples, 3u + 1u) << "3 imu (ticks 7, 15, 23) and 1 mag (tick 19)";
    EXPECT_EQ(rd_u32(b, 80), 48u);
    EXPECT_EQ(rd_u32(b, 84), 48u);
    EXPECT_EQ(rd_u32(b, 88), 32u);
    EXPECT_EQ(rd_u32(b, 92), 32u);
    EXPECT_EQ(rd_text(b, 96, 64), host->trace_hash());
    EXPECT_EQ(host->trace_hash().size(), 64u);
    for (std::size_t i = 160; i < 192; ++i) EXPECT_EQ(std::to_integer<uint8_t>(b[i]), 0u) << "header byte " << i << " is reserved zero";
    // the file is exactly its sections and the 64-byte trailer
    const std::size_t sample_bytes = 3u * (16u + 48u) + 1u * (16u + 32u);
    EXPECT_EQ(b.size(), 192u + 24u * 124u + sample_bytes + 64u);
    // the trailer is the SHA-256 of everything before it
    const std::string body(reinterpret_cast<const char*>(b.data()), b.size() - 64);
    EXPECT_EQ(rd_text(b, b.size() - 64, 64), spade::sha256_hex(body));
}

TEST(FcHostTrace, ATickRecordHoldsTheStateTheRotorsAndTheBladeAzimuthAtTheEndOfTheTick) {
    std::optional<Rig> rig = make_rig(8);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    ASSERT_TRUE(drive(*host, 3));
    const spade::Result<std::vector<std::byte>> file = host->trace_bytes();
    ASSERT_OK(file);
    const std::vector<std::byte>& b = *file;
    const std::size_t rec = 124;
    const spade::Result<spade::fc_host::Truth> truth = host->truth();
    ASSERT_OK(truth);
    const std::size_t last = 192 + 23 * rec;
    EXPECT_EQ(rd_u64(b, last), 23u);
    EXPECT_EQ(rd_u64(b, last + 8), 24u * kDt) << "time_ns = (tick + 1) * dt_ns";
    EXPECT_EQ(rd_f32(b, last + 16), truth->pos.x);
    EXPECT_EQ(rd_f32(b, last + 20), truth->pos.y);
    EXPECT_EQ(rd_f32(b, last + 24), truth->pos.z);
    EXPECT_EQ(rd_f32(b, last + 28), truth->vel.x);
    EXPECT_EQ(rd_f32(b, last + 32), truth->vel.y);
    EXPECT_EQ(rd_f32(b, last + 40), truth->orient.w) << "the quaternion is written (w, x, y, z)";
    EXPECT_EQ(rd_f32(b, last + 44), truth->orient.x);
    EXPECT_EQ(rd_f32(b, last + 56), truth->omega_body.x);
    EXPECT_GT(rd_f32(b, last + 32), 0.0f) << "the rotors pushed: vel.y is up";
    // the applied command at tick 23 is the cycle-3 command (released at the cycle's first tick, no latency)
    EXPECT_EQ(rd_f32(b, last + 68), kCmd[0] + 3.0f);
    EXPECT_EQ(rd_f32(b, last + 72), kCmd[1]);
    EXPECT_EQ(rd_f32(b, last + 76), kCmd[2] - 3.0f);
    // with tau = 0 the shaft speed is the command; the azimuth is blade_azimuth's rule, rebuilt here from the recorded speeds
    const std::size_t omega_at = 76 + 4 * 4;
    const std::size_t psi_at = 76 + 8 * 4;
    float psi[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float h = static_cast<float>(static_cast<double>(kDt) * 1e-9);
    for (std::size_t k = 0; k < 24; ++k) {
        const std::size_t at = 192 + k * rec;
        for (std::size_t r = 0; r < 4; ++r) {
            const float omega = rd_f32(b, at + omega_at + 4 * r);
            EXPECT_TRUE(spade::physics::advance_blade_azimuth(psi[r], omega, h));
            EXPECT_EQ(rd_f32(b, at + psi_at + 4 * r), psi[r]) << "tick " << k << " rotor " << r;
            EXPECT_GE(psi[r], 0.0f);
            EXPECT_LT(psi[r], spade::physics::kAzimuthTwoPi);
        }
    }
    EXPECT_EQ(rd_f32(b, last + omega_at), kCmd[0] + 3.0f);
}

TEST(FcHostTrace, EachSampleIsInTheSampleSectionOnceAndItsTickRecordPointsAtIt) {
    std::optional<Rig> rig = make_rig(8);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    ASSERT_TRUE(drive(*host, 3));
    const spade::Result<std::vector<std::byte>> file = host->trace_bytes();
    ASSERT_OK(file);
    const std::vector<std::byte>& b = *file;
    const std::size_t rec = 124;
    const std::size_t sample_at = 68 + 4 * 4;  // sample_first
    const auto count_at = [&](std::size_t tick) { return rd_u32(b, 192 + tick * rec + sample_at + 4); };
    const auto first_at = [&](std::size_t tick) { return rd_u32(b, 192 + tick * rec + sample_at); };
    EXPECT_EQ(count_at(0), 0u);
    EXPECT_EQ(count_at(7), 1u);
    EXPECT_EQ(first_at(7), 0u);
    EXPECT_EQ(count_at(15), 1u);
    EXPECT_EQ(first_at(15), 1u);
    EXPECT_EQ(count_at(19), 1u) << "the mag, every 20 ticks";
    EXPECT_EQ(first_at(19), 2u);
    EXPECT_EQ(first_at(23), 3u);
    // the first sample record: kind 1 (imu), binding 0, index 1, reserved 0, then 48 bytes
    const std::size_t samples_off = rd_u64(b, 64);
    EXPECT_EQ(rd_u32(b, samples_off), 1u);
    EXPECT_EQ(rd_u32(b, samples_off + 4), 0u);
    EXPECT_EQ(rd_u32(b, samples_off + 8), 1u);
    EXPECT_EQ(rd_u32(b, samples_off + 12), 0u);
    // the mag record: kind 5, binding 2, index 1
    const std::size_t mag_off = samples_off + 2 * (16 + 48);
    EXPECT_EQ(rd_u32(b, mag_off), 5u);
    EXPECT_EQ(rd_u32(b, mag_off + 4), 2u);
    EXPECT_EQ(rd_u32(b, mag_off + 8), 1u);
}

TEST(FcHostTrace, TheHashIsAChainOverEachTicksRecordAndItsSamples) {
    std::optional<Rig> rig = make_rig(8);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    ASSERT_TRUE(drive(*host, 3));
    const spade::Result<std::vector<std::byte>> file = host->trace_bytes();
    ASSERT_OK(file);
    const std::vector<std::byte>& b = *file;
    const std::size_t rec = 124;
    const std::size_t samples_off = rd_u64(b, 64);
    const auto sample_len = [&](std::size_t at) { return std::size_t{16} + (rd_u32(b, at) <= 2 ? 48u : 32u); };
    std::string h = spade::sha256_hex(std::string("spade.trace.v1"));
    for (std::size_t tick = 0; tick < 24; ++tick) {
        const std::size_t at = 192 + tick * rec;
        std::string bytes = h;
        bytes.append(reinterpret_cast<const char*>(b.data()) + at, rec);
        const uint32_t first = rd_u32(b, at + 68 + 16);
        const uint32_t count = rd_u32(b, at + 72 + 16);
        std::size_t sat = samples_off;
        for (uint32_t i = 0; i < first; ++i) sat += sample_len(sat);
        for (uint32_t i = 0; i < count; ++i) {
            const std::size_t n = sample_len(sat);
            bytes.append(reinterpret_cast<const char*>(b.data()) + sat, n);
            sat += n;
        }
        h = spade::sha256_hex(bytes);
    }
    EXPECT_EQ(h, host->trace_hash());
    EXPECT_EQ(host->trace_ticks(), 24u);
}

TEST(FcHostTrace, TheTraceDoesNotDependOnWhenTheCallerTakesSamples) {
    std::string hashes[2];
    std::vector<std::byte> files[2];
    for (int run = 0; run < 2; ++run) {
        std::optional<Rig> rig = make_rig(8);
        ASSERT_TRUE(rig.has_value());
        const std::vector<SensorBinding> sensors = all_bindings(*rig);
        spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
        ASSERT_OK(host);
        ASSERT_TRUE(drive(*host, 12, run == 0));  // run 1 never takes: the 64-deep rings wrap, the host's own polls did not
        const spade::Result<std::vector<std::byte>> file = host->trace_bytes();
        ASSERT_OK(file);
        files[run] = *file;
        hashes[run] = host->trace_hash();
    }
    EXPECT_EQ(hashes[0], hashes[1]);
    EXPECT_TRUE(files[0] == files[1]);
}

// ===========================================================================
// commands.bin v1 and replay
// ===========================================================================

TEST(FcHostTrace, CommandsBinHoldsTheCommandsAsTheRotorsWereGivenThemAfterLatency) {
    std::optional<Rig> rig = make_rig(8);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    HostConfig config;
    config.command_latency_ticks = 2;
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors, config);
    ASSERT_OK(host);
    ASSERT_TRUE(drive(*host, 2));  // commands at ticks 0 and 8: released at 2 and 10
    const std::vector<std::byte> c = host->commands_bytes();
    ASSERT_EQ(c.size(), 24u + 2u * (8u + 4u * 4u));
    EXPECT_EQ(rd_text(c, 0, 4), "SPCM");
    EXPECT_EQ(rd_u32(c, 4), 1u);
    EXPECT_EQ(rd_u32(c, 8), 4u);
    EXPECT_EQ(rd_u32(c, 12), 0u);
    EXPECT_EQ(rd_u64(c, 16), 2u);
    EXPECT_EQ(rd_u64(c, 24), 2u) << "the first command is applied before tick 2's step";
    EXPECT_EQ(rd_f32(c, 32), kCmd[0] + 1.0f);
    EXPECT_EQ(rd_u64(c, 24 + 24), 10u);
}

TEST(FcHostTrace, AReplayOfTheRecordedCommandsReproducesTheTraceByteForByte) {
    std::string live_hash;
    std::vector<std::byte> live_trace;
    std::vector<std::byte> commands;
    {
        std::optional<Rig> rig = make_rig(8);
        ASSERT_TRUE(rig.has_value());
        const std::vector<SensorBinding> sensors = all_bindings(*rig);
        HostConfig config;
        config.command_latency_ticks = 3;
        spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors, config);
        ASSERT_OK(host);
        ASSERT_TRUE(drive(*host, 10));
        live_hash = host->trace_hash();
        const spade::Result<std::vector<std::byte>> file = host->trace_bytes();
        ASSERT_OK(file);
        live_trace = *file;
        commands = host->commands_bytes();
    }
    std::optional<Rig> rig = make_rig(8);  // the same world, no autopilot, no queue
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> replay = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(replay);
    ASSERT_OK(replay->replay_commands(commands, 80));
    EXPECT_EQ(replay->trace_hash(), live_hash);
    const spade::Result<std::vector<std::byte>> file = replay->trace_bytes();
    ASSERT_OK(file);
    EXPECT_TRUE(*file == live_trace);
    EXPECT_TRUE(replay->commands_bytes() == commands) << "a replay records the commands it applied: the same file";
}

TEST(FcHostTrace, AReplayRefusesAFileItCannotFollowNamingWhy) {
    std::optional<Rig> rig = make_rig(8);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    ASSERT_TRUE(drive(*host, 2));
    const std::vector<std::byte> good = host->commands_bytes();
    ASSERT_GT(good.size(), 24u);

    const auto refused = [&](std::vector<std::byte> bytes, const char* needle) {
        std::optional<Rig> r = make_rig(8);
        ASSERT_TRUE(r.has_value());
        const std::vector<SensorBinding> s = all_bindings(*r);
        spade::Result<LockstepHost> h = LockstepHost::open(r->sim, r->vehicle, s);
        ASSERT_OK(h);
        const spade::Result<void> result = h->replay_commands(bytes, 16);
        ASSERT_FALSE(result.has_value()) << needle;
        EXPECT_EQ(result.error().code, spade::Code::invalid_argument);
        EXPECT_NE(result.error().context.find(needle), std::string::npos) << result.error().context;
        EXPECT_EQ(h->tick(), 0u) << "a refused replay steps nothing";
    };
    refused(std::vector<std::byte>(good.begin(), good.begin() + 10), "short");
    std::vector<std::byte> bad_magic = good;
    bad_magic[0] = std::byte{'X'};
    refused(bad_magic, "magic");
    std::vector<std::byte> bad_version = good;
    bad_version[4] = std::byte{9};
    refused(bad_version, "version");
    std::vector<std::byte> bad_rotors = good;
    bad_rotors[8] = std::byte{3};
    refused(bad_rotors, "rotor");
    std::vector<std::byte> truncated(good.begin(), good.end() - 3);
    refused(truncated, "truncated");
}

}  // namespace
