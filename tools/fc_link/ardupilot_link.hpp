#pragma once
// ---------------------------------------------------------------------------
// fc_link/ardupilot_link.hpp -- the seam between Core's LockstepHost and ArduPilot's SITL JSON interface (docs/design/interface/plans/2026-11-21-fc-links.md
// sections 6 and 14), the second autopilot beside autopilot_link.hpp (PX4). The wire layer (ardupilot_json.hpp) is pure; this is the one conversion site.
//
// ArduPilot asks, Spade answers: each SERVO PACKET is one lockstep period. respond() takes the packet's four motor pulses through the propulsion chain to the
// host's rotor command, advances the host by one frame period, and answers with the state at its end: the IMU sample of that period (gyro, specific force) in
// FRD, the vehicle's position, velocity and attitude in NED/FRD from the host's truth. The transport (UDP 9002) is the caller's: this takes a decoded packet and
// returns the state to encode, so it is tested with no socket and no ArduPilot.
//
//   - The frame period is 1 / frame_rate and must be the IMU's own period in whole ticks (the IMU sample IS the frame's gyro and accelerometer); a packet that
//     says otherwise is refused naming both, and nothing advances.
//   - A repeated frame_count is a RESEND: the same state is answered again (the host does not move). A smaller one is a RESTART of ArduPilot, which the run
//     cannot follow (the world is not rewound): refused, naming it.
//   - Only motors 1 to 4 (pwm[0..3]) are read; the other channels are ignored.
// Written from the documented layout; no real ArduPilot has been run against it.
// ---------------------------------------------------------------------------
#include <cstdint>
#include <optional>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "fc_host/lockstep.hpp"
#include "fc_link/ardupilot_json.hpp"
#include "fc_link/frames.hpp"
#include "vehicles/propulsion_steady.hpp"

namespace spade::fc_link {

struct ArduPilotConfig {
    frames::FrameMap frame = frames::compass_x_north();
    const vehicles::PropulsionChain* chain = nullptr;  // required: a duty is turned into a rotor speed through it
    double density = 1.225;                             // kg/m^3 and the battery's state of charge, for rotor_omega_for_duty
    double soc = 1.0;
    uint32_t imu_binding = 0;                           // the index of the IMU in the sensors the host was opened with
    glm::dvec3 home_world{0.0};                         // the Spade world point that IS the autopilot's NED origin
};

struct ArduPilotStep {
    ardupilot_json::StateReply state;
    ardupilot_json::FrameKind kind = ardupilot_json::FrameKind::fresh;
    uint64_t tick = 0;  // the host's tick after the step
};

class ArduPilotLink {
public:
    [[nodiscard]] static Result<ArduPilotLink> create(const ArduPilotConfig& config);

    // One servo packet, one reply. Refusals name what: a frame period that is not the IMU's, a restart, a pulse or chain the map refuses, the host's own errors.
    [[nodiscard]] Result<ArduPilotStep> respond(fc_host::LockstepHost& host, const ardupilot_json::ServoPacket& packet);

private:
    explicit ArduPilotLink(const ArduPilotConfig& config) : config_(config) {}

    ArduPilotConfig config_;
    ardupilot_json::FrameTracker tracker_;
    ardupilot_json::ReplyClock clock_;
    std::optional<ardupilot_json::StateReply> last_;
};

}  // namespace spade::fc_link
