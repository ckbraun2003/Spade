#include "fc_link/autopilot_map.hpp"

#include <cmath>
#include <expected>
#include <span>
#include <string>

namespace spade::fc_link {

namespace {
constexpr float kPx4Slack = 1.0e-3f;  // float noise either side of [0, 1] that is clamped; anything further is refused
}

Result<double> px4_output_to_duty(float output) {
    if (!std::isfinite(output) || output < -kPx4Slack || output > 1.0f + kPx4Slack) {
        return std::unexpected(Error{Code::invalid_argument, "px4_output_to_duty: the output " + std::to_string(output) +
                                                                 " is not finite and in [0, 1] (1e-3 slack at each end is clamped)"});
    }
    if (output <= 0.0f) return 0.0;
    if (output >= 1.0f) return 1.0;
    return static_cast<double>(output);
}

Result<double> ardupilot_pwm_to_duty(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max) {
    if (pwm_max <= pwm_min) {
        return std::unexpected(Error{Code::invalid_argument, "ardupilot_pwm_to_duty: pwm_max " + std::to_string(pwm_max) +
                                                                 " must be above pwm_min " + std::to_string(pwm_min)});
    }
    if (pwm <= pwm_min) return 0.0;
    if (pwm >= pwm_max) return 1.0;
    return static_cast<double>(pwm - pwm_min) / static_cast<double>(pwm_max - pwm_min);
}

Result<std::array<float, kAutopilotQuadXMotors>> autopilot_duties_to_rotor_commands(const vehicles::PropulsionChain& chain, std::span<const double> duties,
                                                                                    double density, double soc) {
    if (duties.size() != kAutopilotQuadXMotors) {
        return std::unexpected(Error{Code::invalid_argument, "autopilot_duties_to_rotor_commands: " + std::to_string(duties.size()) +
                                                                 " duties for a quad of " + std::to_string(kAutopilotQuadXMotors) + " motors"});
    }
    std::array<float, kAutopilotQuadXMotors> out{};
    for (std::size_t i = 0; i < kAutopilotQuadXMotors; ++i) {
        const Result<vehicles::DutyCommand> cmd = vehicles::rotor_omega_for_duty(chain, duties[i], density, soc);
        if (!cmd) {
            return std::unexpected(Error{cmd.error().code, "autopilot_duties_to_rotor_commands: motor " + std::to_string(i) + ": " + cmd.error().context});
        }
        out[kAutopilotQuadXToSpade[i]] = cmd->omega_cmd;
    }
    return out;
}

}  // namespace spade::fc_link
