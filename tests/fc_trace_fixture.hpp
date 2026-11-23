#pragma once

// ---------------------------------------------------------------------------
// fc_trace_fixture.hpp -- a trace.bin v1 and a commands.bin v1 written from the SPEC, for the run-folder checker's tests.
//
// docs/design/core/plans/2026-11-21-fc-core.md section 4.1 and the comment in tools/fc_host/lockstep.hpp give the layout. This is a second,
// independent writer of those bytes (TD-4): it includes nothing from tools/fc_host, so the checker is shown to agree with the SPEC and not with
// Core's code. If the two ever disagree, the test against Core's sample run (SPADE_FC_SAMPLE_RUN) says so.
//
// Header-only test support, never installed. Little-endian, as the format says and as every machine Spade builds on is.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "world/sha256.hpp"

namespace spade::fc::fixture {

struct TraceSpec {
    uint32_t rotors = 4;
    uint64_t ticks = 3;
    uint64_t first_tick = 0;
    uint64_t dt_ns = 500000;
};

struct Built {
    std::string trace;     // trace.bin
    std::string commands;  // commands.bin
    std::string hash;      // the rolling trace_hash (64 lower-case hex)
    uint64_t sample_count = 0;
};

inline void put32(std::string& s, uint32_t v) {
    for (int i = 0; i < 4; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFFu));
}
inline void put64(std::string& s, uint64_t v) {
    for (int i = 0; i < 8; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFFu));
}
inline void putf(std::string& s, float f) {
    uint32_t v = 0;
    std::memcpy(&v, &f, 4);
    put32(s, v);
}
inline void set32(std::string& s, size_t at, uint32_t v) {
    for (int i = 0; i < 4; ++i) s[at + i] = static_cast<char>((v >> (8 * i)) & 0xFFu);
}
inline void set64(std::string& s, size_t at, uint64_t v) {
    for (int i = 0; i < 8; ++i) s[at + i] = static_cast<char>((v >> (8 * i)) & 0xFFu);
}

// What the fixture's commands do: a command at tick 0 and one at tick 2 (rad/s per rotor), held in between.
inline float command_value(uint32_t rotor, uint64_t tick) { return (tick < 2 ? 100.0f : 150.0f) + static_cast<float>(rotor); }

inline Built build_trace(const TraceSpec& sp = {}) {
    constexpr uint32_t kHeader = 192;
    constexpr uint32_t kSampleHeader = 16;
    constexpr uint32_t kPayload[4] = {48, 48, 32, 32};  // imu, gnss, baro, mag
    const uint32_t n = sp.rotors;
    const uint32_t rec = 76 + 12 * n;

    // The sensor samples of tick k: an IMU every tick, a barometer on even ticks, a magnetometer every third, a GNSS fix on the first.
    struct Sample { uint32_t kind; uint32_t binding; uint32_t index; std::string payload; };
    uint32_t next_index[4] = {1, 1, 1, 1};
    auto sample = [&](int slot, uint32_t kind, uint64_t tick) {
        Sample s{kind, static_cast<uint32_t>(slot), next_index[slot]++, std::string()};
        for (uint32_t i = 0; i < kPayload[slot]; ++i) s.payload.push_back(static_cast<char>((tick * 7 + slot * 13 + i * 3) & 0xFF));
        return s;
    };

    std::string ticks_section;
    std::string samples_section;
    uint64_t sample_total = 0;
    std::string h = spade::sha256_hex("spade.trace.v1");
    for (uint64_t k = 0; k < sp.ticks; ++k) {
        const uint64_t tick = sp.first_tick + k;
        std::vector<Sample> ss;
        ss.push_back(sample(0, 1, tick));
        if (tick % 2 == 0) ss.push_back(sample(2, 4, tick));
        if (tick % 3 == 0) ss.push_back(sample(3, 5, tick));
        if (tick == sp.first_tick) ss.push_back(sample(1, 2, tick));

        std::string r;
        put64(r, tick);
        put64(r, (tick + 1) * sp.dt_ns);
        for (int i = 0; i < 3; ++i) putf(r, 0.25f * static_cast<float>(tick) + static_cast<float>(i));       // position
        for (int i = 0; i < 3; ++i) putf(r, 0.5f * static_cast<float>(tick) - static_cast<float>(i));        // velocity
        for (float q : {1.0f, 0.0f, 0.0f, 0.0f}) putf(r, q);                                                  // orientation (w, x, y, z)
        for (int i = 0; i < 3; ++i) putf(r, 0.01f * static_cast<float>(tick + i));                            // body rate
        for (uint32_t i = 0; i < n; ++i) putf(r, command_value(i, k));                                        // applied rotor command
        put32(r, static_cast<uint32_t>(sample_total));
        put32(r, static_cast<uint32_t>(ss.size()));
        for (uint32_t i = 0; i < n; ++i) putf(r, 90.0f + static_cast<float>(i) + static_cast<float>(tick));   // shaft speed
        for (uint32_t i = 0; i < n; ++i) putf(r, 0.1f * static_cast<float>(i + 1));                           // blade azimuth, in [0, 2 pi)

        std::string these;
        for (const Sample& s : ss) {
            put32(these, s.kind);
            put32(these, s.binding);
            put32(these, s.index);
            put32(these, 0);
            these += s.payload;
        }
        sample_total += ss.size();
        h = spade::sha256_hex(h + r + these);
        ticks_section += r;
        samples_section += these;
    }

    std::string t;
    t += "SPTR";
    put32(t, 1);
    put32(t, kHeader);
    put32(t, 0);
    put64(t, sp.dt_ns);
    put32(t, 1);  // substeps
    put32(t, n);
    put64(t, sp.ticks);
    put64(t, sp.first_tick);
    put32(t, rec);
    put32(t, kSampleHeader);
    put64(t, kHeader);
    put64(t, kHeader + static_cast<uint64_t>(ticks_section.size()));
    put64(t, sample_total);
    for (uint32_t p : kPayload) put32(t, p);
    t += h;  // 96..160, 64 characters
    while (t.size() < kHeader) t.push_back('\0');
    t += ticks_section;
    t += samples_section;
    t += spade::sha256_hex(t);  // the trailer: lower-case hex SHA-256 of every byte before it

    std::string c;
    c += "SPCM";
    put32(c, 1);
    put32(c, n);
    put32(c, 0);
    put64(c, 2);  // the commands at tick 0 and tick 2
    for (uint64_t tick : {sp.first_tick, sp.first_tick + 2}) {
        put64(c, tick);
        for (uint32_t i = 0; i < n; ++i) putf(c, command_value(i, tick - sp.first_tick));
    }
    return Built{t, c, h, sample_total};
}

}  // namespace spade::fc::fixture
