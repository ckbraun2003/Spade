#pragma once

#include <cstdint>
#include <functional>
#include <optional>

#include "core/error.hpp"
#include "fc_host/lockstep.hpp"
#include "fc_link/frames.hpp"
#include "fc_link/mavlink.hpp"
#include "fc_link/px4_link.hpp"
#include "vehicles/propulsion_steady.hpp"

// ---------------------------------------------------------------------------
// fc_link/autopilot_link.hpp -- the seam between Core's LockstepHost and an autopilot link (docs/design/interface/plans/2026-11-21-fc-links.md
// section 7.1, section 14). ONE conversion site: the host's samples become HIL_SENSOR (and HIL_GPS), the autopilot's outputs become the host's
// rotor command, and the boot phase holds the host's world.
//
// One exchange() is one IMU period of lockstep:
//   - AFTER boot: the host advances to the next IMU sample time, the sample goes out as HIL_SENSOR in the autopilot's frame (frames.hpp), the
//     reply's four motor outputs go through the propulsion chain (autopilot_map.hpp) and become host.command().
//   - DURING boot (the reply says `booting`): the host is NOT advanced; the message is built from host.boot_readings(boot_step); when the first
//     real reply arrives the host is told how long the boot was (host.note_boot) and that reply's outputs are the command before tick 0.
//   - time_usec is the host's time plus the boot offset, so the autopilot's clock never goes backwards and boot never reaches the world.
// The transport is a function, so the seam is tested against a scripted peer; Px4Link::exchange is the real one (px4_transport()).
//
// ---------------------------------------------------------------------------

namespace spade::fc_link {

using Px4Transport = std::function<Result<ActuatorReply>(const mavlink::HilSensor&, const std::optional<mavlink::HilGps>&)>;

struct ExchangeConfig {
    frames::FrameMap frame = frames::compass_x_north();
    const vehicles::PropulsionChain* chain = nullptr;  // required: a duty is turned into a rotor speed through it
    double density = 1.225;                             // kg/m^3 and the battery's state of charge, for rotor_omega_for_duty
    double soc = 1.0;
    uint32_t imu_binding = 0;                           // the index of the IMU in the sensors the host was opened with
    std::optional<uint32_t> baro_binding;               // the barometer's and the magnetometer's, when the host has them bound (slice 2)
    std::optional<uint32_t> mag_binding;
};

struct ExchangeResult {
    bool booting = false;       // the autopilot has not answered yet; the host was held
    uint64_t time_usec = 0;     // what the autopilot's clock read in the message
    uint64_t tick = 0;          // the host's tick after the exchange
};

class AutopilotLink {
  public:
    [[nodiscard]] static Result<AutopilotLink> create(Px4Transport transport, const ExchangeConfig& config) {
        if (!transport) return std::unexpected(Error{Code::invalid_argument, "AutopilotLink: no transport"});
        return AutopilotLink(std::move(transport), config);
    }

    [[nodiscard]] Result<ExchangeResult> exchange(fc_host::LockstepHost& host);

  private:
    AutopilotLink(Px4Transport transport, const ExchangeConfig& config) : transport_(std::move(transport)), config_(config) {}

    Px4Transport transport_;
    ExchangeConfig config_;
    uint64_t boot_steps_ = 0;         // booting replies so far
    uint64_t boot_end_usec_ = 0;      // the autopilot's clock at the end of boot: the offset of the host's time
    bool boot_over_ = false;
    mavlink::HilSensor last_{};       // the last message sent: a group with no new sample carries its last values (it is not marked updated)
};

}  // namespace spade::fc_link
