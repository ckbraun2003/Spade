#include "vehicles/motor.hpp"

#include <cmath>
#include <limits>

#include "vehicles/propulsion_flags.hpp"
#include "vehicles/rotor.hpp"  // rotor_lag_alpha

namespace spade::vehicles {
namespace {

// One template per function: the float and double overloads below are the
// same operations in two precisions (motor.hpp, "FLOAT AND DOUBLE").

template <typename T>
[[nodiscard]] bool is_finite(T x) noexcept {
    return std::isfinite(x);
}

template <typename T>
[[nodiscard]] T clamp_duty(T duty, uint32_t& flags) noexcept {
    if (!is_finite(duty)) {
        flags |= propulsion_flags::duty_clamped;
        return T(0);
    }
    if (duty < T(0)) {
        flags |= propulsion_flags::duty_clamped;
        return T(0);
    }
    if (duty > T(1)) {
        flags |= propulsion_flags::duty_clamped;
        return T(1);
    }
    return duty;
}

template <typename T>
[[nodiscard]] T kv_si(T kv_rpm_per_volt) noexcept {
    if (!is_finite(kv_rpm_per_volt) || !(kv_rpm_per_volt > T(0))) return T(0);
    // pi/30 rounded once to T; the product is the only operation.
    return kv_rpm_per_volt * T(0.10471975511965977461542144610932);
}

template <typename T>
[[nodiscard]] T effective_resistance(T resistance, T pole_pairs, T inductance, T omega) noexcept {
    if (!is_finite(resistance) || !(resistance > T(0))) return T(0);
    if (!is_finite(pole_pairs) || !is_finite(inductance) || !is_finite(omega)) return resistance;
    const T x = pole_pairs * omega * inductance;  // reactance, ohm
    const T r_eff = resistance + (x * x) / resistance;
    if (!is_finite(r_eff)) return std::numeric_limits<T>::max();
    return r_eff;
}

template <typename T>
[[nodiscard]] T current(T duty, T v_bus, T omega, T kv, T r_eff) noexcept {
    if (!is_finite(duty) || !is_finite(v_bus) || !is_finite(omega) || !is_finite(kv) || !is_finite(r_eff)) return T(0);
    if (!(kv > T(0)) || !(r_eff > T(0))) return T(0);
    const T i = (duty * v_bus - omega / kv) / r_eff;
    return is_finite(i) ? i : T(0);
}

template <typename T>
[[nodiscard]] T clamp_current(T i, T i_lo, T i_max, uint32_t& flags) noexcept {
    if (!is_finite(i)) i = T(0);
    if (is_finite(i_max) && i > i_max) {
        flags |= propulsion_flags::current_limited;
        return i_max;
    }
    if (is_finite(i_lo) && i < i_lo) {
        flags |= propulsion_flags::current_limited;
        return i_lo;
    }
    return i;
}

template <typename T>
[[nodiscard]] T torque(T i, T i0, T kv, T omega) noexcept {
    if (!is_finite(i) || !is_finite(i0) || !is_finite(kv) || !is_finite(omega) || !(kv > T(0))) return T(0);
    const T q = (i - i0) / kv;  // Kt (I - I0), Kt = 1/Kv
    if (!is_finite(q)) return T(0);
    // Friction holds a stopped shaft; it never drives it backwards.
    if (!(omega > T(0)) && q < T(0)) return T(0);
    return q;
}

template <typename T>
[[nodiscard]] T time_constant(T inertia, T r_eff, T kv) noexcept {
    if (!is_finite(inertia) || !is_finite(r_eff) || !is_finite(kv)) return T(0);
    if (!(inertia > T(0)) || !(r_eff > T(0)) || !(kv > T(0))) return T(0);
    const T tau = inertia * r_eff * kv * kv;
    return is_finite(tau) ? tau : std::numeric_limits<T>::max();
}

template <typename T>
[[nodiscard]] T speed_target(T duty, T v_bus, T kv, T r_eff, T i0, T load) noexcept {
    if (!is_finite(duty) || !is_finite(v_bus) || !is_finite(kv) || !is_finite(r_eff) || !is_finite(i0) || !is_finite(load)) {
        return T(0);
    }
    if (!(kv > T(0))) return T(0);
    // Steady state of J dw/dt = Kt((d V - w/Kv)/R_eff - I0) - Q_load.
    const T w = kv * (duty * v_bus - r_eff * (i0 + load * kv));
    return is_finite(w) ? w : T(0);
}

template <typename T>
[[nodiscard]] T clamp_speed(T omega, uint32_t& flags) noexcept {
    if (!is_finite(omega)) return T(0);
    if (omega < T(0)) {
        flags |= propulsion_flags::speed_clamped;
        return T(0);
    }
    return omega;
}

template <typename T>
[[nodiscard]] T speed_step(T omega, T omega_inf, T alpha, uint32_t& flags) noexcept {
    if (!is_finite(omega)) omega = T(0);
    if (!is_finite(omega_inf) || !is_finite(alpha)) return clamp_speed(omega, flags);
    return clamp_speed(omega + (omega_inf - omega) * alpha, flags);
}

template <typename T>
[[nodiscard]] T speed_step_limited(T omega, T drive, T load, T inertia, T h, uint32_t& flags) noexcept {
    if (!is_finite(omega)) omega = T(0);
    if (!is_finite(drive) || !is_finite(load) || !is_finite(inertia) || !is_finite(h)) return clamp_speed(omega, flags);
    if (!(inertia > T(0)) || !(h > T(0))) return clamp_speed(omega, flags);
    return clamp_speed(omega + h * (drive - load) / inertia, flags);
}

}  // namespace

float esc_clamp_duty(float duty, uint32_t& flags) noexcept { return clamp_duty(duty, flags); }
double esc_clamp_duty(double duty, uint32_t& flags) noexcept { return clamp_duty(duty, flags); }

float motor_kv_si(float kv_rpm_per_volt) noexcept { return kv_si(kv_rpm_per_volt); }
double motor_kv_si(double kv_rpm_per_volt) noexcept { return kv_si(kv_rpm_per_volt); }

float motor_effective_resistance(float resistance, float pole_pairs, float inductance, float omega) noexcept {
    return effective_resistance(resistance, pole_pairs, inductance, omega);
}
double motor_effective_resistance(double resistance, double pole_pairs, double inductance, double omega) noexcept {
    return effective_resistance(resistance, pole_pairs, inductance, omega);
}

float motor_current(float duty, float v_bus, float omega, float kv, float r_eff) noexcept {
    return current(duty, v_bus, omega, kv, r_eff);
}
double motor_current(double duty, double v_bus, double omega, double kv, double r_eff) noexcept {
    return current(duty, v_bus, omega, kv, r_eff);
}

float motor_clamp_current(float current_a, float i_lo, float i_max, uint32_t& flags) noexcept {
    return clamp_current(current_a, i_lo, i_max, flags);
}
double motor_clamp_current(double current_a, double i_lo, double i_max, uint32_t& flags) noexcept {
    return clamp_current(current_a, i_lo, i_max, flags);
}

float motor_torque(float current_a, float no_load_current, float kv, float omega) noexcept {
    return torque(current_a, no_load_current, kv, omega);
}
double motor_torque(double current_a, double no_load_current, double kv, double omega) noexcept {
    return torque(current_a, no_load_current, kv, omega);
}

float motor_time_constant(float inertia, float r_eff, float kv) noexcept { return time_constant(inertia, r_eff, kv); }
double motor_time_constant(double inertia, double r_eff, double kv) noexcept {
    return time_constant(inertia, r_eff, kv);
}

float motor_alpha(float h, float tau) noexcept { return rotor_lag_alpha(h, tau); }

float motor_speed_target(float duty, float v_bus, float kv, float r_eff, float no_load_current,
                         float load_torque) noexcept {
    return speed_target(duty, v_bus, kv, r_eff, no_load_current, load_torque);
}
double motor_speed_target(double duty, double v_bus, double kv, double r_eff, double no_load_current,
                          double load_torque) noexcept {
    return speed_target(duty, v_bus, kv, r_eff, no_load_current, load_torque);
}

float motor_speed_step(float omega, float omega_inf, float alpha, uint32_t& flags) noexcept {
    return speed_step(omega, omega_inf, alpha, flags);
}
double motor_speed_step(double omega, double omega_inf, double alpha, uint32_t& flags) noexcept {
    return speed_step(omega, omega_inf, alpha, flags);
}

float motor_speed_step_limited(float omega, float drive_torque, float load_torque, float inertia, float h,
                               uint32_t& flags) noexcept {
    return speed_step_limited(omega, drive_torque, load_torque, inertia, h, flags);
}
double motor_speed_step_limited(double omega, double drive_torque, double load_torque, double inertia, double h,
                                uint32_t& flags) noexcept {
    return speed_step_limited(omega, drive_torque, load_torque, inertia, h, flags);
}

}  // namespace spade::vehicles
