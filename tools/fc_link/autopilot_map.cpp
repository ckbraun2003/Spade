#include "fc_link/autopilot_map.hpp"

#include <expected>
#include <span>

namespace spade::fc_link {

// THE RED STUB: every function refuses, so the tests fail by name before the code exists.
Result<double> px4_output_to_duty(float) { return std::unexpected(Error{Code::invalid_argument, "px4_output_to_duty: not implemented"}); }

Result<double> ardupilot_pwm_to_duty(uint16_t, uint16_t, uint16_t) {
    return std::unexpected(Error{Code::invalid_argument, "ardupilot_pwm_to_duty: not implemented"});
}

Result<std::array<float, kAutopilotQuadXMotors>> autopilot_duties_to_rotor_commands(const vehicles::PropulsionChain&, std::span<const double>, double, double) {
    return std::unexpected(Error{Code::invalid_argument, "autopilot_duties_to_rotor_commands: not implemented"});
}

}  // namespace spade::fc_link
