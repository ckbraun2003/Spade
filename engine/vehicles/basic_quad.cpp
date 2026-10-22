#include "vehicles/basic_quad.hpp"

namespace spade::vehicles {

MotorBlock BasicQuadParams::default_motor() { return MotorBlock{}; }
PropBlock BasicQuadParams::default_prop() { return PropBlock{}; }
EscBlock BasicQuadParams::default_esc() { return EscBlock{}; }
BatteryBlock BasicQuadParams::default_battery() { return BatteryBlock{}; }

Result<AirframeSpec> basic_quad_airframe(const BasicQuadParams& params) {
    (void)params;
    return AirframeSpec{};
}

std::vector<BasicQuadAssumption> basic_quad_assumptions() { return {}; }

}  // namespace spade::vehicles
