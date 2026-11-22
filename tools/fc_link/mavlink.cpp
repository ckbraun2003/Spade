#include "mavlink.hpp"

#include <algorithm>
#include <bit>
#include <string>
#include <type_traits>

// The MAVLink 2 codec (mavlink.hpp). Every message is written and read field by field in WIRE order (the definition's fields sorted by
// size, largest first, extension fields last), with the byte offsets spelt out beside each field; the golden frames of
// tests/test_fc_link_mavlink.cpp pin each offset against a reference.
namespace spade::fc_link::mavlink {

namespace {

// ---- little-endian byte access (no reliance on the host's byte order or on struct layout) ----

class Writer {
public:
    explicit Writer(std::array<uint8_t, kMaxPayload>& buf) : buf_(buf) { buf_.fill(0); }
    template <class T>
    void put(T v) {
        if constexpr (std::is_same_v<T, float>) {
            put(std::bit_cast<uint32_t>(v));
        } else {
            using U = std::make_unsigned_t<T>;
            const U u = static_cast<U>(v);
            for (size_t i = 0; i < sizeof(T); ++i) buf_[off_ + i] = static_cast<uint8_t>((u >> (8 * i)) & 0xFF);
            off_ += sizeof(T);
        }
    }
    size_t size() const { return off_; }

private:
    std::array<uint8_t, kMaxPayload>& buf_;
    size_t off_ = 0;
};

class Reader {
public:
    explicit Reader(const std::array<uint8_t, kMaxPayload>& buf) : buf_(buf) {}
    template <class T>
    T get() {
        if constexpr (std::is_same_v<T, float>) {
            return std::bit_cast<float>(get<uint32_t>());
        } else {
            using U = std::make_unsigned_t<T>;
            U u = 0;
            for (size_t i = 0; i < sizeof(T); ++i) u = static_cast<U>(u | (static_cast<U>(buf_[off_ + i]) << (8 * i)));
            off_ += sizeof(T);
            return static_cast<T>(u);
        }
    }

private:
    const std::array<uint8_t, kMaxPayload>& buf_;
    size_t off_ = 0;
};

struct Def {
    uint32_t id;
    uint8_t crc_extra;
    size_t max_len;
};
// CRC_EXTRA and the full payload length of each message, from the common dialect (and the reference frames' CRCs).
constexpr Def kDefs[] = {
    {0, 50, 9},     // HEARTBEAT
    {76, 152, 33},  // COMMAND_LONG
    {93, 47, 81},   // HIL_ACTUATOR_CONTROLS
    {107, 108, 65}, // HIL_SENSOR
    {113, 124, 39}, // HIL_GPS
    {115, 4, 64},   // HIL_STATE_QUATERNION
};

const Def* find_def(uint32_t id) {
    for (const Def& d : kDefs)
        if (d.id == id) return &d;
    return nullptr;
}

void write_payload(Writer& w, const Heartbeat& m) {
    w.put(m.custom_mode);   // u32 @0
    w.put(m.type);          // @4
    w.put(m.autopilot);     // @5
    w.put(m.base_mode);     // @6
    w.put(m.system_status); // @7
    w.put(m.mavlink_version);  // @8
}
void write_payload(Writer& w, const CommandLong& m) {
    for (float p : m.param) w.put(p);  // f32 x7 @0..27
    w.put(m.command);                  // u16 @28
    w.put(m.target_system);            // @30
    w.put(m.target_component);         // @31
    w.put(m.confirmation);             // @32
}
void write_payload(Writer& w, const HilSensor& m) {
    w.put(m.time_usec);  // u64 @0
    w.put(m.xacc); w.put(m.yacc); w.put(m.zacc);          // f32 @8..19
    w.put(m.xgyro); w.put(m.ygyro); w.put(m.zgyro);       // @20..31
    w.put(m.xmag); w.put(m.ymag); w.put(m.zmag);          // @32..43
    w.put(m.abs_pressure); w.put(m.diff_pressure); w.put(m.pressure_alt); w.put(m.temperature);  // @44..59
    w.put(m.fields_updated);  // u32 @60
    w.put(m.id);              // extension u8 @64
}
void write_payload(Writer& w, const HilGps& m) {
    w.put(m.time_usec);                                   // u64 @0
    w.put(m.lat); w.put(m.lon); w.put(m.alt);             // i32 @8, 12, 16
    w.put(m.eph); w.put(m.epv); w.put(m.vel);             // u16 @20, 22, 24
    w.put(m.vn); w.put(m.ve); w.put(m.vd);                // i16 @26, 28, 30
    w.put(m.cog);                                         // u16 @32
    w.put(m.fix_type); w.put(m.satellites_visible);       // u8 @34, 35
    w.put(m.id);                                          // extension u8 @36
    w.put(m.yaw);                                         // extension u16 @37 (unaligned: extensions are not reordered)
}
void write_payload(Writer& w, const HilActuatorControls& m) {
    w.put(m.time_usec);  // u64 @0
    w.put(m.flags);      // u64 @8 (the second u64 sorts ahead of the 4-byte floats)
    for (float c : m.controls) w.put(c);  // f32 x16 @16..79
    w.put(m.mode);       // u8 @80
}
void write_payload(Writer& w, const HilStateQuaternion& m) {
    w.put(m.time_usec);  // u64 @0
    for (float q : m.attitude_quaternion) w.put(q);        // f32 x4 @8..23
    w.put(m.rollspeed); w.put(m.pitchspeed); w.put(m.yawspeed);  // @24..35
    w.put(m.lat); w.put(m.lon); w.put(m.alt);              // i32 @36, 40, 44
    w.put(m.vx); w.put(m.vy); w.put(m.vz);                 // i16 @48, 50, 52
    w.put(m.ind_airspeed); w.put(m.true_airspeed);         // u16 @54, 56
    w.put(m.xacc); w.put(m.yacc); w.put(m.zacc);           // i16 @58, 60, 62
}

Heartbeat read_heartbeat(Reader& r) {
    Heartbeat m;
    m.custom_mode = r.get<uint32_t>();
    m.type = r.get<uint8_t>();
    m.autopilot = r.get<uint8_t>();
    m.base_mode = r.get<uint8_t>();
    m.system_status = r.get<uint8_t>();
    m.mavlink_version = r.get<uint8_t>();
    return m;
}
CommandLong read_command_long(Reader& r) {
    CommandLong m;
    for (float& p : m.param) p = r.get<float>();
    m.command = r.get<uint16_t>();
    m.target_system = r.get<uint8_t>();
    m.target_component = r.get<uint8_t>();
    m.confirmation = r.get<uint8_t>();
    return m;
}
HilSensor read_hil_sensor(Reader& r) {
    HilSensor m;
    m.time_usec = r.get<uint64_t>();
    m.xacc = r.get<float>(); m.yacc = r.get<float>(); m.zacc = r.get<float>();
    m.xgyro = r.get<float>(); m.ygyro = r.get<float>(); m.zgyro = r.get<float>();
    m.xmag = r.get<float>(); m.ymag = r.get<float>(); m.zmag = r.get<float>();
    m.abs_pressure = r.get<float>(); m.diff_pressure = r.get<float>(); m.pressure_alt = r.get<float>(); m.temperature = r.get<float>();
    m.fields_updated = r.get<uint32_t>();
    m.id = r.get<uint8_t>();
    return m;
}
HilGps read_hil_gps(Reader& r) {
    HilGps m;
    m.time_usec = r.get<uint64_t>();
    m.lat = r.get<int32_t>(); m.lon = r.get<int32_t>(); m.alt = r.get<int32_t>();
    m.eph = r.get<uint16_t>(); m.epv = r.get<uint16_t>(); m.vel = r.get<uint16_t>();
    m.vn = r.get<int16_t>(); m.ve = r.get<int16_t>(); m.vd = r.get<int16_t>();
    m.cog = r.get<uint16_t>();
    m.fix_type = r.get<uint8_t>(); m.satellites_visible = r.get<uint8_t>();
    m.id = r.get<uint8_t>();
    m.yaw = r.get<uint16_t>();
    return m;
}
HilActuatorControls read_hil_actuator_controls(Reader& r) {
    HilActuatorControls m;
    m.time_usec = r.get<uint64_t>();
    m.flags = r.get<uint64_t>();
    for (float& c : m.controls) c = r.get<float>();
    m.mode = r.get<uint8_t>();
    return m;
}
HilStateQuaternion read_hil_state_quaternion(Reader& r) {
    HilStateQuaternion m;
    m.time_usec = r.get<uint64_t>();
    for (float& q : m.attitude_quaternion) q = r.get<float>();
    m.rollspeed = r.get<float>(); m.pitchspeed = r.get<float>(); m.yawspeed = r.get<float>();
    m.lat = r.get<int32_t>(); m.lon = r.get<int32_t>(); m.alt = r.get<int32_t>();
    m.vx = r.get<int16_t>(); m.vy = r.get<int16_t>(); m.vz = r.get<int16_t>();
    m.ind_airspeed = r.get<uint16_t>(); m.true_airspeed = r.get<uint16_t>();
    m.xacc = r.get<int16_t>(); m.yacc = r.get<int16_t>(); m.zacc = r.get<int16_t>();
    return m;
}

uint16_t frame_crc(const uint8_t* after_stx, size_t n, uint8_t extra) {
    uint16_t crc = crc16(std::span<const uint8_t>(after_stx, n));
    return crc16(std::span<const uint8_t>(&extra, 1), crc);
}

}  // namespace

uint16_t crc16(std::span<const uint8_t> bytes, uint16_t init) {
    uint16_t crc = init;
    for (const uint8_t b : bytes) {
        uint8_t tmp = static_cast<uint8_t>(b ^ (crc & 0xFF));
        tmp = static_cast<uint8_t>(tmp ^ (tmp << 4));
        crc = static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^ (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
    }
    return crc;
}

std::optional<uint8_t> crc_extra(uint32_t msgid) {
    const Def* d = find_def(msgid);
    return d ? std::optional<uint8_t>(d->crc_extra) : std::nullopt;
}

std::optional<size_t> max_payload_len(uint32_t msgid) {
    const Def* d = find_def(msgid);
    return d ? std::optional<size_t>(d->max_len) : std::nullopt;
}

uint32_t msgid_of(const Message& m) {
    return std::visit(
        [](const auto& v) -> uint32_t {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, Heartbeat>) return static_cast<uint32_t>(MsgId::heartbeat);
            else if constexpr (std::is_same_v<T, CommandLong>) return static_cast<uint32_t>(MsgId::command_long);
            else if constexpr (std::is_same_v<T, HilSensor>) return static_cast<uint32_t>(MsgId::hil_sensor);
            else if constexpr (std::is_same_v<T, HilGps>) return static_cast<uint32_t>(MsgId::hil_gps);
            else if constexpr (std::is_same_v<T, HilActuatorControls>) return static_cast<uint32_t>(MsgId::hil_actuator_controls);
            else return static_cast<uint32_t>(MsgId::hil_state_quaternion);
        },
        m);
}

std::vector<uint8_t> encode(const Header& header, const Message& message) {
    std::array<uint8_t, kMaxPayload> payload;
    Writer w(payload);
    std::visit([&](const auto& m) { write_payload(w, m); }, message);
    size_t len = w.size();
    while (len > 1 && payload[len - 1] == 0) --len;  // trailing zeros are dropped, never the first byte

    const uint32_t id = msgid_of(message);
    std::vector<uint8_t> f;
    f.reserve(kHeaderLen + len + kChecksumLen);
    f.push_back(kStx);
    f.push_back(static_cast<uint8_t>(len));
    f.push_back(0);  // incompat flags: unsigned
    f.push_back(0);  // compat flags
    f.push_back(header.seq);
    f.push_back(header.sysid);
    f.push_back(header.compid);
    f.push_back(static_cast<uint8_t>(id & 0xFF));
    f.push_back(static_cast<uint8_t>((id >> 8) & 0xFF));
    f.push_back(static_cast<uint8_t>((id >> 16) & 0xFF));
    f.insert(f.end(), payload.begin(), payload.begin() + static_cast<std::ptrdiff_t>(len));
    const uint16_t crc = frame_crc(f.data() + 1, f.size() - 1, find_def(id)->crc_extra);
    f.push_back(static_cast<uint8_t>(crc & 0xFF));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    return f;
}

Result<Message> decode(const Frame& frame) {
    const Def* d = find_def(frame.msgid);
    if (!d) return std::unexpected(Error{Code::not_found, "MAVLink message " + std::to_string(frame.msgid) + " is not one this codec carries"});
    if (frame.payload.size() > d->max_len)
        return std::unexpected(Error{Code::invalid_argument, "MAVLink message " + std::to_string(frame.msgid) + ": payload of " +
                                                                 std::to_string(frame.payload.size()) + " bytes, its longest is " + std::to_string(d->max_len)});
    std::array<uint8_t, kMaxPayload> buf{};  // zero-filled: the sender dropped the trailing zeros
    std::copy(frame.payload.begin(), frame.payload.end(), buf.begin());
    Reader r(buf);
    switch (static_cast<MsgId>(frame.msgid)) {
        case MsgId::heartbeat: return Message{read_heartbeat(r)};
        case MsgId::command_long: return Message{read_command_long(r)};
        case MsgId::hil_actuator_controls: return Message{read_hil_actuator_controls(r)};
        case MsgId::hil_sensor: return Message{read_hil_sensor(r)};
        case MsgId::hil_gps: return Message{read_hil_gps(r)};
        case MsgId::hil_state_quaternion: return Message{read_hil_state_quaternion(r)};
    }
    return std::unexpected(Error{Code::internal, "unreachable"});
}

void Parser::feed(std::span<const uint8_t> bytes) {
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
    size_t pos = 0;  // everything before pos is consumed
    for (;;) {
        // find the next candidate STX
        while (pos < buffer_.size() && buffer_[pos] != kStx) {
            ++pos;
            ++stats_.garbage_bytes;
        }
        if (buffer_.size() - pos < kHeaderLen) break;  // not enough to read a header yet
        const uint8_t* p = buffer_.data() + pos;
        const size_t len = p[1];
        const uint8_t incompat = p[2];
        const uint32_t id = static_cast<uint32_t>(p[7]) | (static_cast<uint32_t>(p[8]) << 8) | (static_cast<uint32_t>(p[9]) << 16);
        if ((incompat & ~uint8_t{0x01}) != 0) {
            ++stats_.unsupported_flags;
            ++pos;
            continue;
        }
        const Def* d = find_def(id);
        if (!d) {
            ++stats_.unknown_msgid;
            ++pos;
            continue;
        }
        if (len > d->max_len) {
            ++stats_.bad_length;
            ++pos;
            continue;
        }
        const bool signed_frame = (incompat & 0x01) != 0;
        const size_t total = kHeaderLen + len + kChecksumLen + (signed_frame ? kSignatureLen : 0);
        if (buffer_.size() - pos < total) break;  // wait for the rest
        const uint16_t want = frame_crc(p + 1, kHeaderLen - 1 + len, d->crc_extra);
        const uint16_t got = static_cast<uint16_t>(p[kHeaderLen + len] | (p[kHeaderLen + len + 1] << 8));
        if (want != got) {
            ++stats_.bad_crc;
            ++pos;
            continue;
        }
        Frame f;
        f.header = Header{p[4], p[5], p[6]};
        f.msgid = id;
        f.payload.assign(p + kHeaderLen, p + kHeaderLen + len);
        ready_.push_back(std::move(f));
        ++stats_.frames;
        pos += total;
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(pos));
}

std::optional<Frame> Parser::next() {
    if (ready_head_ >= ready_.size()) {
        ready_.clear();
        ready_head_ = 0;
        return std::nullopt;
    }
    return std::move(ready_[ready_head_++]);
}

}  // namespace spade::fc_link::mavlink
