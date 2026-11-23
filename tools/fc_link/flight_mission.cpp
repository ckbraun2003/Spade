#include "flight_mission.hpp"

#include <algorithm>
#include <cmath>

namespace spade::fc_link {

namespace {
constexpr double kEps = 1e-9;  // ticks land on 0.1 s steps with float drift: a limit reached "exactly" must count
constexpr uint16_t kCmdArm = 400;
}  // namespace

std::vector<mavlink::CommandLong> FlightMission::step(const FlightObservation& o) {
    std::vector<mavlink::CommandLong> out;
    const auto fail = [&](const char* name, double measured, double limit) {
        journal_.push_back({name, false, measured, limit, o.time_s});
        phase_ = Phase::failed;
    };
    switch (phase_) {
        case Phase::boot:
            if (o.time_s >= config_.ready_s - kEps) {
                out.push_back(mavlink::command_arm(true));
                first_arm_s_ = last_arm_s_ = o.time_s;
                phase_ = Phase::arming;
            }
            break;
        case Phase::arming: {
            if (o.ack && o.ack->command == kCmdArm && o.ack->result != 0 && o.ack->result != 1 && o.ack->result != 5) {
                fail("arm_ack", static_cast<double>(o.ack->result), 0.0);  // denied, unsupported or failed: PX4 said no
                break;
            }
            const double since = o.time_s - first_arm_s_;
            if (o.armed) {
                journal_.push_back({"armed_within_s", true, since, config_.arm_timeout_s, o.time_s});
                out.push_back(mavlink::command_takeoff(static_cast<float>(config_.home_amsl_m + config_.takeoff_height_m)));
                takeoff_s_ = o.time_s;
                phase_ = Phase::takeoff;
            } else if (since >= config_.arm_timeout_s - kEps) {
                fail("armed_within_s", since, config_.arm_timeout_s);
            } else if (o.time_s - last_arm_s_ >= config_.arm_retry_s - kEps) {
                out.push_back(mavlink::command_arm(true));
                last_arm_s_ = o.time_s;
            }
            break;
        }
        case Phase::takeoff: {
            const double off = std::fabs(o.altitude_m - config_.takeoff_height_m);
            if (off <= config_.takeoff_band_m + kEps) {
                journal_.push_back({"takeoff_height", true, off, config_.takeoff_band_m, o.time_s});
                hover_start_s_ = o.time_s;
                hover_ref_m_ = o.altitude_m;
                max_dev_m_ = 0.0;
                phase_ = Phase::hover;
            } else if (o.time_s - takeoff_s_ >= config_.takeoff_timeout_s - kEps) {
                fail("takeoff_height", off, config_.takeoff_band_m);
            }
            break;
        }
        case Phase::hover: {
            const double dev = std::fabs(o.altitude_m - hover_ref_m_);
            max_dev_m_ = std::max(max_dev_m_, dev);
            if (dev > config_.hover_band_m + kEps) {
                fail("hover_within_m", dev, config_.hover_band_m);
            } else if (o.time_s - hover_start_s_ >= config_.hover_s - kEps) {
                journal_.push_back({"hover_within_m", true, max_dev_m_, config_.hover_band_m, o.time_s});
                out.push_back(mavlink::command_land());
                land_s_ = o.time_s;
                phase_ = Phase::land;
            }
            break;
        }
        case Phase::land: {
            const double since = o.time_s - land_s_;
            if (o.altitude_m < config_.landed_height_m && !o.armed) {
                journal_.push_back({"landed_within_s", true, since, config_.land_timeout_s, o.time_s});
                phase_ = Phase::done;
            } else if (since >= config_.land_timeout_s - kEps) {
                fail("landed_within_s", since, config_.land_timeout_s);
            }
            break;
        }
        case Phase::done:
        case Phase::failed:
            break;
    }
    return out;
}

}  // namespace spade::fc_link
