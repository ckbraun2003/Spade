#pragma once

// ---------------------------------------------------------------------------
// fc_ulog_fixture.hpp -- a small ULog file written from the ULog specification, by this file's own code (nothing of PX4's), for the K4.log tests
// (tests/fc_ulog_check.hpp). A log is a list of messages {type, body}; `assemble` frames them behind the 16-byte header, so a test changes ONE
// message, drops one, or reorders, and asks which check names it. Spec: https://docs.px4.io/v1.17/en/dev_log/ulog_file_format
// ---------------------------------------------------------------------------

#include <cstdint>
#include <string>
#include <vector>

namespace spade::fc::ulogfx {

struct Msg {
    char type = 'D';
    std::string body;
};

inline void put16(std::string& s, uint16_t v) {
    s.push_back(static_cast<char>(v & 0xFF));
    s.push_back(static_cast<char>(v >> 8));
}
inline void put64(std::string& s, uint64_t v) {
    for (int i = 0; i < 8; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
}

// 16 bytes: the magic, the version, the start time (microseconds).
inline std::string header(uint8_t version = 1, uint64_t start_us = 316000) {
    std::string h("ULog\x01\x12\x35", 7);
    h.push_back(static_cast<char>(version));
    put64(h, start_us);
    return h;
}

inline std::string frame(const Msg& m) {
    std::string s;
    put16(s, static_cast<uint16_t>(m.body.size()));
    s.push_back(m.type);
    s += m.body;
    return s;
}

inline std::string assemble(const std::vector<Msg>& msgs, uint8_t version = 1) {
    std::string out = header(version);
    for (const Msg& m : msgs) out += frame(m);
    return out;
}

// key_len, key, value (the shape of I, P; M and Q put one byte more in front).
inline std::string keyed(const std::string& key, const std::string& value) {
    std::string s(1, static_cast<char>(key.size()));
    return s + key + value;
}

// A well-formed log: B first; two formats, info, a parameter with its default, a multi info; two subscriptions; logged data on both; a logged string,
// a tagged one, a sync message and a dropout inside the data section. The indices let a test reach one message.
enum Index : size_t { kB = 0, kFGyro, kFStatus, kInfoName, kInfoVer, kParam, kDefault, kMulti, kSubGyro, kSubStatus, kDataGyro0, kLogged, kTagged, kSync, kDropout, kDataStatus, kDataGyro1 };

inline std::vector<Msg> good_messages() {
    std::vector<Msg> v;
    v.push_back({'B', std::string(40, '\0')});
    v.push_back({'F', "sensor_gyro:uint64_t timestamp;float[3] xyz;uint8_t[4] _padding0;"});
    v.push_back({'F', "vehicle_status:uint64_t timestamp;uint8_t arming_state;"});
    v.push_back({'I', keyed("char[3] sys_name", "PX4")});
    std::string ver;
    ver += std::string("\xFF\x02\x04\x01", 4);  // 0x010402FF, little-endian: version 1.4.2, release
    v.push_back({'I', keyed("uint32_t ver_sw_release", ver)});
    v.push_back({'P', keyed("int32_t SYS_AUTOSTART", std::string("\x92\x27\x00\x00", 4))});
    std::string q(1, '\x03');
    q += keyed("float MC_ROLL_P", std::string("\x00\x00\xC0\x40", 4));
    v.push_back({'Q', q});
    std::string mi(1, '\0');
    mi += keyed("char[4] note", "abcd");
    v.push_back({'M', mi});
    std::string a0(1, '\0');
    put16(a0, 0);
    a0 += "sensor_gyro";
    v.push_back({'A', a0});
    std::string a1(1, '\0');
    put16(a1, 1);
    a1 += "vehicle_status";
    v.push_back({'A', a1});
    const auto gyro = [](uint16_t id, uint64_t t) {
        std::string d;
        put16(d, id);
        put64(d, t);
        d += std::string(12 + 4, '\0');
        return d;
    };
    v.push_back({'D', gyro(0, 1000)});
    std::string l(1, '6');  // level
    put64(l, 2000);
    l += "boot";
    v.push_back({'L', l});
    std::string c(1, '6');
    put16(c, 7);  // tag
    put64(c, 3000);
    c += "tagged";
    v.push_back({'C', c});
    v.push_back({'S', std::string("\x2F\x73\x13\x20\x25\x0C\xBB\x12", 8)});
    std::string o;
    put16(o, 5);
    v.push_back({'O', o});
    std::string st;
    put16(st, 1);
    put64(st, 4000);
    st.push_back('\x01');
    v.push_back({'D', st});
    v.push_back({'D', gyro(0, 5000)});
    return v;
}

}  // namespace spade::fc::ulogfx
