#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "core/error.hpp"
#include "mavlink.hpp"

// ---------------------------------------------------------------------------
// fc_link/px4_link.hpp -- Spade as the simulator PX4 expects (docs/design/interface/plans/2026-11-21-fc-links.md section 1). PX4 SITL,
// started with no simulator of its own (`none`), CONNECTS to a TCP port Spade listens on; the link then runs PX4's lockstep:
//
//     Spade sends HIL_SENSOR (and HIL_GPS on its period)  ->  PX4 sets its clock to time_usec, runs one iteration, and
//     answers HIL_ACTUATOR_CONTROLS  ->  Spade steps its physics, and sends the next sensors.
//
// PX4 has NO lockstep timeout: a silent PX4 stalls both sides. So the timeout is the link's: exchange() waits at most `reply_timeout` of
// WALL time for the reply and refuses with a named error ("px4_no_reply: ...") instead of waiting or guessing. The wall-clock wait is I/O
// only; it never enters Spade's state, so a run's determinism is untouched.
//
// The link knows nothing of Spade's engine or of frames: it is handed finished MAVLink messages (the caller converted, tools/fc_link/
// frames.hpp) and returns the autopilot's outputs. The bytes move through a ByteChannel, so the lockstep logic is tested against a scripted
// in-memory peer (tests/test_fc_link_px4.cpp) and the real TCP channel is a separate piece.
// ---------------------------------------------------------------------------

namespace spade::fc_link {

// A byte stream to the autopilot.
class ByteChannel {
public:
    virtual ~ByteChannel() = default;
    // Sends all of `bytes` or fails.
    [[nodiscard]] virtual Result<void> write(std::span<const uint8_t> bytes) = 0;
    // Whatever has arrived, waiting at most `timeout` for the first byte; an empty vector means nothing arrived in time; an error means
    // the stream is closed or broken.
    [[nodiscard]] virtual Result<std::vector<uint8_t>> read_some(std::chrono::milliseconds timeout) = 0;
};

struct Px4LinkConfig {
    std::chrono::milliseconds reply_timeout{5000};      // wall time to wait for HIL_ACTUATOR_CONTROLS (a phase 1 requirement: 5 s)
    std::chrono::milliseconds heartbeat_timeout{30000};  // wall time for PX4's first HEARTBEAT after it connects
    mavlink::Header sender{0, 1, 200};                  // our sysid/compid (seq is ours to count)
    // BOOT PHASE: PX4's own boot runs on the lockstep clock and answers nothing until its modules are up, so until its first
    // HIL_ACTUATOR_CONTROLS the link keeps sending sensor messages and waits at most boot_poll of wall time per step for a reply; it reports
    // `booting` (zero controls) and counts boot_steps. A PX4 that has not answered after boot_timeout of wall time since the first sensor
    // message is refused as "px4_no_reply ... never started". From the first reply on the link is strict lockstep (reply_timeout).
    std::chrono::milliseconds boot_timeout{120000};
    std::chrono::milliseconds boot_poll{10};
};

// PX4's answer to one sensor message.
struct ActuatorReply {
    uint64_t time_usec = 0;
    std::array<float, 16> controls{};  // motors 0..1 (PWM-1000)/1000, servos -1..1, all 0 when disarmed
    bool armed = false;                // MAV_MODE_FLAG_SAFETY_ARMED (128) in `mode`
    bool lockstep = false;             // HIL_ACTUATOR_CONTROLS_FLAGS_LOCKSTEP (bit 0) in `flags`
    bool booting = false;              // PX4 has not answered yet: no controls (all 0); the caller holds its world and sends the next sensors
};

// What the lockstep gap checker (Test/Docs) and the run record read.
struct Px4LinkReport {
    uint64_t sensor_messages = 0;     // HIL_SENSOR sent (== exchanges attempted)
    uint64_t gps_messages = 0;        // HIL_GPS sent
    uint64_t replies = 0;             // HIL_ACTUATOR_CONTROLS accepted as the answer to a sensor message
    uint64_t max_replies_per_sensor = 0;  // most HIL_ACTUATOR_CONTROLS that arrived between two of our sensor messages (1 expected)
    uint64_t heartbeats_seen = 0;
    uint64_t command_longs_seen = 0;  // PX4's one MAV_CMD_SET_MESSAGE_INTERVAL for HIL_STATE_QUATERNION, normally
    uint64_t last_time_usec = 0;      // time_usec of the last HIL_SENSOR sent
    uint64_t boot_steps = 0;          // sensor messages sent before PX4's first reply (it depends on wall time: PX4's side)
    uint64_t boot_end_time_usec = 0;  // time_usec of the sensor message whose exchange returned the first reply (0 until then): Spade's tick 0
    uint64_t catchup_replies = 0;     // replies to boot-time sensor messages that arrived with the first one
    std::chrono::nanoseconds wall_wait_total{0};  // wall time spent waiting for replies
    std::chrono::nanoseconds wall_wait_max{0};
    mavlink::Parser::Stats parser{};
};

class Px4Link {
public:
    // Waits for PX4's first HEARTBEAT on `channel` (PX4 sends one when it connects): refuses with "px4_no_heartbeat" after
    // heartbeat_timeout.
    [[nodiscard]] static Result<Px4Link> open(std::unique_ptr<ByteChannel> channel, const Px4LinkConfig& config = {});

    // One lockstep exchange: send `sensor` (and `gps` when given), wait for the reply. time_usec must strictly increase between
    // calls (PX4's whole clock is set from it): refused as invalid_argument otherwise. Refusals, each named:
    //   px4_no_reply      no HIL_ACTUATOR_CONTROLS within reply_timeout (the last time_usec and the count of sensors sent are in the text)
    //   px4_stale_reply   a reply stamped before the sensor message it should answer
    //   px4_link_closed   the channel closed or broke
    [[nodiscard]] Result<ActuatorReply> exchange(const mavlink::HilSensor& sensor, const std::optional<mavlink::HilGps>& gps = std::nullopt);

    [[nodiscard]] const Px4LinkReport& report() const { return report_; }

    Px4Link(Px4Link&&) noexcept = default;
    Px4Link& operator=(Px4Link&&) noexcept = default;

private:
    Px4Link(std::unique_ptr<ByteChannel> channel, const Px4LinkConfig& config) : channel_(std::move(channel)), config_(config) {}
    // One bounded read from the channel into the parser (an empty read is fine: nothing arrived yet).
    [[nodiscard]] Result<void> fill(std::chrono::milliseconds wait);
    // Consumes every parsed frame: HEARTBEAT and COMMAND_LONG go to the report; of the HIL_ACTUATOR_CONTROLS frames the LAST is returned
    // (the freshest output) and all are counted.
    struct Taken {
        std::optional<mavlink::HilActuatorControls> actuator;
        uint64_t actuator_count = 0;
    };
    [[nodiscard]] Taken take();

    std::unique_ptr<ByteChannel> channel_;
    Px4LinkConfig config_;
    mavlink::Parser parser_;
    Px4LinkReport report_;
    uint8_t seq_ = 0;
    std::optional<uint64_t> last_sensor_time_usec_;
    bool booted_ = false;  // PX4's first reply has been seen: strict lockstep from here
    std::optional<std::chrono::steady_clock::time_point> boot_start_;  // wall time of the first sensor message (the boot limit counts from it)
    uint64_t first_sensor_time_usec_ = 0;
};

}  // namespace spade::fc_link
