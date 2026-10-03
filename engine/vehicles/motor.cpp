#include "vehicles/motor.hpp"

// STUB: every function returns 0 so the tests run red for the right reason.
// The implementation replaces this file in the next commit.

namespace spade::vehicles {

float esc_clamp_duty(float, uint32_t&) noexcept { return 0.0f; }
double esc_clamp_duty(double, uint32_t&) noexcept { return 0.0; }
float motor_kv_si(float) noexcept { return 0.0f; }
double motor_kv_si(double) noexcept { return 0.0; }
float motor_effective_resistance(float, float, float, float) noexcept { return 0.0f; }
double motor_effective_resistance(double, double, double, double) noexcept { return 0.0; }
float motor_current(float, float, float, float, float) noexcept { return 0.0f; }
double motor_current(double, double, double, double, double) noexcept { return 0.0; }
float motor_clamp_current(float, float, float, uint32_t&) noexcept { return 0.0f; }
double motor_clamp_current(double, double, double, uint32_t&) noexcept { return 0.0; }
float motor_torque(float, float, float, float) noexcept { return 0.0f; }
double motor_torque(double, double, double, double) noexcept { return 0.0; }
float motor_time_constant(float, float, float) noexcept { return 0.0f; }
double motor_time_constant(double, double, double) noexcept { return 0.0; }
float motor_alpha(float, float) noexcept { return 0.0f; }
float motor_speed_target(float, float, float, float, float, float) noexcept { return 0.0f; }
double motor_speed_target(double, double, double, double, double, double) noexcept { return 0.0; }
float motor_speed_step(float, float, float, uint32_t&) noexcept { return 0.0f; }
double motor_speed_step(double, double, double, uint32_t&) noexcept { return 0.0; }
float motor_speed_step_limited(float, float, float, float, float, uint32_t&) noexcept { return 0.0f; }
double motor_speed_step_limited(double, double, double, double, double, uint32_t&) noexcept { return 0.0; }

}  // namespace spade::vehicles
