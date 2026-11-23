#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>
#include <vector>

#include "core/error.hpp"

// ---------------------------------------------------------------------------
// fc_link/mavlink.hpp -- a small MAVLink 2 codec for the few messages the autopilot links use (docs/design/interface/plans/
// 2026-11-21-fc-links.md section 3): HEARTBEAT, COMMAND_LONG, HIL_SENSOR, HIL_GPS, HIL_ACTUATOR_CONTROLS, HIL_STATE_QUATERNION.
// Our own, not the generated headers: a fixed handful of messages, no third-party tree, and every wire detail pinned by a test against
// frames a reference (pymavlink 2.4.49, run inside the pinned PX4 build image with the network off) made.
//
// THE WIRE (https://mavlink.io/en/guide/serialization.html): 0xFD, len, incompat flags, compat flags, seq, sysid, compid, msgid (3 bytes,
// low first), payload, CRC (2 bytes, low first), and a 13-byte signature after the CRC when incompat flag 0x01 is set. The payload holds
// the fields sorted by size, largest first (extension fields last, in definition order, not reordered), little-endian, and its trailing
// zero bytes are dropped on send and zero-filled on receive. The CRC is CRC-16/MCRF4XX over everything after the STX, then one CRC_EXTRA
// byte per message (a hash of the message definition; pinned below, checked by the golden frames).
//
// No I/O here and no engine state: bytes in, structs out. Errors are spade::Error, never exceptions.
// ---------------------------------------------------------------------------

namespace spade::fc_link::mavlink {

inline constexpr uint8_t kStx = 0xFD;
inline constexpr size_t kHeaderLen = 10;     // STX .. msgid
inline constexpr size_t kChecksumLen = 2;
inline constexpr size_t kSignatureLen = 13;
inline constexpr size_t kMaxPayload = 255;
inline constexpr size_t kMaxFrame = kHeaderLen + kMaxPayload + kChecksumLen + kSignatureLen;  // 280

// CRC-16/MCRF4XX (poly 0x1021 reflected, init 0xFFFF, no final xor); check("123456789") == 0x6F91.
[[nodiscard]] uint16_t crc16(std::span<const uint8_t> bytes, uint16_t init = 0xFFFF);

enum class MsgId : uint32_t {
    heartbeat = 0,
    command_long = 76,
    command_ack = 77,
    hil_actuator_controls = 93,
    hil_sensor = 107,
    hil_gps = 113,
    hil_state_quaternion = 115,
};

// The CRC_EXTRA of a message we know (nullopt for any other id): the parser can only check a frame whose message it knows.
[[nodiscard]] std::optional<uint8_t> crc_extra(uint32_t msgid);
// The payload length with every field present (the longest a frame of this message may be), nullopt for an unknown id.
[[nodiscard]] std::optional<size_t> max_payload_len(uint32_t msgid);

// Who sent the frame: the three header fields the sender chooses.
struct Header {
    uint8_t seq = 0;
    uint8_t sysid = 1;
    uint8_t compid = 1;
    friend bool operator==(const Header&, const Header&) = default;
};

// ---- the messages (fields in definition order; units as MAVLink's common dialect states them) ----

struct Heartbeat {
    uint32_t custom_mode = 0;
    uint8_t type = 0;
    uint8_t autopilot = 0;
    uint8_t base_mode = 0;
    uint8_t system_status = 0;
    uint8_t mavlink_version = 3;
    friend bool operator==(const Heartbeat&, const Heartbeat&) = default;
};

struct CommandLong {
    std::array<float, 7> param{};
    uint16_t command = 0;
    uint8_t target_system = 0;
    uint8_t target_component = 0;
    uint8_t confirmation = 0;
    friend bool operator==(const CommandLong&, const CommandLong&) = default;
};

// What the autopilot answers to a command (MAV_RESULT: 0 accepted, 1 temporarily rejected, 2 denied, 3 unsupported, 4 failed, 5 in progress);
// target_system/target_component are the command's sender, progress and result_param2 the extension fields (progress 255 = unknown).
struct CommandAck {
    uint16_t command = 0;
    uint8_t result = 0;
    uint8_t progress = 0;
    int32_t result_param2 = 0;
    uint8_t target_system = 0;
    uint8_t target_component = 0;
    friend bool operator==(const CommandAck&, const CommandAck&) = default;
};

// The flight commands the video #3 script sends (docs/design/interface/plans/2026-11-21-fc-links.md section 13), as COMMAND_LONG to the
// autopilot's own system and component (1, 1 by default). arm: MAV_CMD_COMPONENT_ARM_DISARM (400), param1 1 / 0. takeoff: MAV_CMD_NAV_TAKEOFF
// (22): yaw and position NaN (unchanged, here), the altitude in param7 ABOVE MEAN SEA LEVEL (PX4 reads it as an absolute altitude). land:
// MAV_CMD_NAV_LAND (21): yaw and position NaN (here). The bytes are pinned to pymavlink's (tests/test_fc_link_mavlink.cpp).
[[nodiscard]] CommandLong command_arm(bool arm, uint8_t target_system = 1, uint8_t target_component = 1);
[[nodiscard]] CommandLong command_takeoff(float altitude_amsl_m, uint8_t target_system = 1, uint8_t target_component = 1);
[[nodiscard]] CommandLong command_land(uint8_t target_system = 1, uint8_t target_component = 1);

// IMU in SI units, FRD body; mag in gauss; pressure in hPa; temperature in degC; fields_updated is the bitmap of which groups are valid
// (PX4: accel 0x7, gyro 0x38, mag 0x1C0, baro 0x1A00, diff pressure 0x400); id is the sensor instance (0 drives PX4's lockstep).
struct HilSensor {
    uint64_t time_usec = 0;
    float xacc = 0, yacc = 0, zacc = 0;
    float xgyro = 0, ygyro = 0, zgyro = 0;
    float xmag = 0, ymag = 0, zmag = 0;
    float abs_pressure = 0, diff_pressure = 0, pressure_alt = 0, temperature = 0;
    uint32_t fields_updated = 0;
    uint8_t id = 0;
    friend bool operator==(const HilSensor&, const HilSensor&) = default;
};

// lat/lon in degrees e7, alt in mm (MSL), eph/epv HDOP/VDOP x100 (65535 unknown), vel and vn/ve/vd in cm/s, cog in cdeg (65535 unknown),
// yaw in cdeg (0 = not available, 36000 = north).
struct HilGps {
    uint64_t time_usec = 0;
    int32_t lat = 0, lon = 0, alt = 0;
    uint16_t eph = 0, epv = 0, vel = 0;
    int16_t vn = 0, ve = 0, vd = 0;
    uint16_t cog = 0;
    uint8_t fix_type = 0, satellites_visible = 0, id = 0;
    uint16_t yaw = 0;
    friend bool operator==(const HilGps&, const HilGps&) = default;
};

// PX4 to the simulator: 16 outputs (motors 0..1, servos -1..1), mode 129 when armed, flags bit 0 = lockstep.
struct HilActuatorControls {
    uint64_t time_usec = 0;
    std::array<float, 16> controls{};
    uint8_t mode = 0;
    uint64_t flags = 0;
    friend bool operator==(const HilActuatorControls&, const HilActuatorControls&) = default;
};

struct HilStateQuaternion {
    uint64_t time_usec = 0;
    std::array<float, 4> attitude_quaternion{};  // w, x, y, z
    float rollspeed = 0, pitchspeed = 0, yawspeed = 0;
    int32_t lat = 0, lon = 0, alt = 0;
    int16_t vx = 0, vy = 0, vz = 0;
    uint16_t ind_airspeed = 0, true_airspeed = 0;
    int16_t xacc = 0, yacc = 0, zacc = 0;
    friend bool operator==(const HilStateQuaternion&, const HilStateQuaternion&) = default;
};

using Message = std::variant<Heartbeat, CommandLong, CommandAck, HilSensor, HilGps, HilActuatorControls, HilStateQuaternion>;

[[nodiscard]] uint32_t msgid_of(const Message& m);

// One complete, unsigned v2 frame (trailing zeros of the payload dropped, at least one payload byte kept).
[[nodiscard]] std::vector<uint8_t> encode(const Header& header, const Message& message);

// A frame as the parser delivers it: the header fields and the payload as it came on the wire (not zero-filled).
struct Frame {
    Header header;
    uint32_t msgid = 0;
    std::vector<uint8_t> payload;
};

// Zero-fills a short payload and reads the fields; not_found for a message id we do not carry.
[[nodiscard]] Result<Message> decode(const Frame& frame);

// A byte-at-a-time parser: feed it whatever the socket gave (any chunking, partial frames, garbage between frames); it yields each
// frame whose CRC (with the message's CRC_EXTRA) matches. A signed frame (incompat flag 0x01) is accepted with its 13 signature bytes
// skipped, NOT verified; a frame with any other incompat flag, an unknown message id or an over-long payload is dropped and counted.
class Parser {
public:
    struct Stats {
        uint64_t frames = 0;
        uint64_t bad_crc = 0;
        uint64_t unknown_msgid = 0;
        uint64_t bad_length = 0;
        uint64_t unsupported_flags = 0;
        uint64_t garbage_bytes = 0;  // bytes skipped while looking for a 0xFD
    };

    void feed(std::span<const uint8_t> bytes);
    // The next complete frame in arrival order, or nullopt.
    [[nodiscard]] std::optional<Frame> next();
    [[nodiscard]] const Stats& stats() const { return stats_; }

private:
    std::vector<uint8_t> buffer_;  // bytes not yet consumed, starting at a candidate STX
    std::vector<Frame> ready_;
    size_t ready_head_ = 0;
    Stats stats_;
};

}  // namespace spade::fc_link::mavlink
