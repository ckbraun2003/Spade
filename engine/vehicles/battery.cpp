#include "vehicles/battery.hpp"

// STUB: every function returns 0 so the tests run red for the right reason.
// The implementation replaces this file in the next commit.

namespace spade::vehicles {

float battery_pack_resistance(uint32_t, uint32_t, float) noexcept { return 0.0f; }
double battery_pack_resistance(uint32_t, uint32_t, double) noexcept { return 0.0; }
float battery_ocv(float, std::span<const float>) noexcept { return 0.0f; }
double battery_ocv(double, std::span<const float>) noexcept { return 0.0; }
Result<std::vector<float>> battery_resample_ocv(std::span<const std::array<double, 2>>, uint32_t count) {
    return std::vector<float>(count, 0.0f);
}
float battery_rc_beta(float, float, float) noexcept { return 0.5f; }
float battery_polarization_step(float, float, float, float) noexcept { return 0.0f; }
double battery_polarization_step(double, double, double, double) noexcept { return 0.0; }
float battery_soc_step(float, float, float, float, uint32_t&) noexcept { return 0.0f; }
double battery_soc_step(double, double, double, double, uint32_t&) noexcept { return 0.0; }
float battery_terminal_voltage(float, float, float, float) noexcept { return 0.0f; }
double battery_terminal_voltage(double, double, double, double) noexcept { return 0.0; }
BusResult bus_solve(float, float, float, float, std::span<const BusMotor>, std::span<float>) noexcept { return {}; }
BusResultD bus_solve(double, double, double, double, std::span<const BusMotorD>, std::span<double>) noexcept {
    return {};
}

}  // namespace spade::vehicles
