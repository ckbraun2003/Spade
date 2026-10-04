#include "vehicles/battery.hpp"

#include <cmath>
#include <string>

#include "core/fp32_math.hpp"
#include "vehicles/propulsion_flags.hpp"

namespace spade::vehicles {
namespace {

template <typename T>
[[nodiscard]] bool is_finite(T x) noexcept {
    return std::isfinite(x);
}

template <typename T>
[[nodiscard]] T pack_resistance(uint32_t s, uint32_t p, T cell_r) noexcept {
    if (s == 0 || p == 0 || !is_finite(cell_r) || !(cell_r > T(0))) return T(0);
    return T(s) * cell_r / T(p);
}

template <typename T>
[[nodiscard]] T ocv(T soc, std::span<const float> table) noexcept {
    if (table.empty()) return T(0);
    if (table.size() == 1) return T(table[0]);
    if (!is_finite(soc) || soc < T(0)) soc = T(0);
    if (soc > T(1)) soc = T(1);
    const std::size_t last = table.size() - 1;
    const T x = soc * T(last);
    std::size_t i = static_cast<std::size_t>(x);  // x >= 0: truncation is floor
    if (i >= last) i = last - 1;
    const T f = x - T(i);
    const T a = T(table[i]);
    const T b = T(table[i + 1]);
    return a + (b - a) * f;
}

template <typename T>
[[nodiscard]] T polarization_step(T v1, T current, T r1, T beta) noexcept {
    if (!is_finite(v1) || !is_finite(current) || !is_finite(r1) || !is_finite(beta)) return T(0);
    const T v = v1 * beta + r1 * current * (T(1) - beta);
    return is_finite(v) ? v : T(0);
}

template <typename T>
[[nodiscard]] T soc_step(T soc, T current, T h, T capacity_ah, uint32_t& flags) noexcept {
    if (!is_finite(soc)) soc = T(0);
    T next = soc;
    if (is_finite(current) && is_finite(h) && is_finite(capacity_ah) && h > T(0) && capacity_ah > T(0)) {
        next = soc - current * h / (T(3600) * capacity_ah);
        if (!is_finite(next)) next = soc;
    }
    if (next > T(1)) next = T(1);
    if (next <= T(0)) {
        if (current > T(0)) flags |= propulsion_flags::battery_empty;
        next = T(0);
    }
    return next;
}

template <typename T>
[[nodiscard]] T terminal_voltage(T v_oc, T v1, T r0, T current) noexcept {
    if (!is_finite(v_oc) || !is_finite(v1) || !is_finite(r0) || !is_finite(current)) return T(0);
    const T v = v_oc - v1 - r0 * current;
    return is_finite(v) ? v : T(0);
}

// --- the bus ---------------------------------------------------------------

template <typename T>
[[nodiscard]] bool motor_usable(const BusMotorT<T>& m) noexcept {
    return is_finite(m.duty) && is_finite(m.back_emf) && is_finite(m.r_eff) && m.r_eff > T(0);
}

// One solve at a given duty scale. Returns V and I_b; writes currents; ORs
// current_limited into `flags`. `fixed` tracks clamped motors (bit k).
template <typename T>
void solve_at(T scale, T v_source, T r0, std::span<const BusMotorT<T>> motors, std::span<T> currents, T& v_out,
              T& ib_out, uint32_t& flags) noexcept {
    uint64_t fixed = 0;
    const std::size_t n = motors.size();
    for (std::size_t k = 0; k < n; ++k) currents[k] = T(0);

    T v = v_source;
    for (std::size_t pass = 0; pass <= n; ++pass) {
        T a = T(0);  // sum d^2 / R   over free motors
        T b = T(0);  // sum d e / R   over free motors
        T c = T(0);  // sum d I       over fixed motors
        for (std::size_t k = 0; k < n; ++k) {
            const BusMotorT<T>& m = motors[k];
            if (!motor_usable(m)) continue;
            const T d = scale * m.duty;
            if ((fixed >> k) & 1u) {
                c += d * currents[k];
            } else {
                a += d * d / m.r_eff;
                b += d * m.back_emf / m.r_eff;
            }
        }
        v = (v_source + r0 * b - r0 * c) / (T(1) + r0 * a);

        bool changed = false;
        for (std::size_t k = 0; k < n; ++k) {
            const BusMotorT<T>& m = motors[k];
            if (!motor_usable(m) || ((fixed >> k) & 1u)) continue;
            const T d = scale * m.duty;
            T i = (d * v - m.back_emf) / m.r_eff;
            if (!is_finite(i)) i = T(0);
            if (is_finite(m.current_max) && i > m.current_max) {
                i = m.current_max;
                fixed |= (uint64_t{1} << k);
                changed = true;
            } else if (is_finite(m.current_min) && i < m.current_min) {
                i = m.current_min;
                fixed |= (uint64_t{1} << k);
                changed = true;
            }
            currents[k] = i;
        }
        if (!changed) break;
    }
    if (fixed != 0) flags |= propulsion_flags::current_limited;

    T ib = T(0);
    for (std::size_t k = 0; k < n; ++k) {
        if (!motor_usable(motors[k])) continue;
        ib += scale * motors[k].duty * currents[k];
    }
    v_out = v;
    ib_out = ib;
}

// The ESC side's total: sum |I_k| over the usable motors, in slot order.
template <typename T>
[[nodiscard]] T motor_side_total(std::span<const BusMotorT<T>> motors, std::span<const T> currents) noexcept {
    T total = T(0);
    for (std::size_t k = 0; k < motors.size(); ++k) {
        if (!motor_usable(motors[k])) continue;
        total += currents[k] < T(0) ? -currents[k] : currents[k];
    }
    return total;
}

template <typename T>
[[nodiscard]] BusResultT<T> bus(T v_source, T r_series, T ib_max, T v_cut, T esc_max,
                                std::span<const BusMotorT<T>> motors, std::span<T> currents) noexcept {
    BusResultT<T> out;
    if (motors.size() > kMaxBusMotors || currents.size() < motors.size() || !is_finite(v_source)) {
        for (T& i : currents) i = T(0);
        out.duty_scale = T(0);
        return out;
    }
    const T r0 = (is_finite(r_series) && r_series > T(0)) ? r_series : T(0);
    const bool has_ib_max = is_finite(ib_max) && ib_max > T(0);
    const bool has_cut = is_finite(v_cut) && v_cut > T(0);
    const bool has_esc = is_finite(esc_max) && esc_max > T(0);

    T v = T(0);
    T ib = T(0);
    uint32_t flags = 0;
    solve_at(T(1), v_source, r0, motors, currents, v, ib, flags);

    const bool over_current = has_ib_max && ib > ib_max;
    const bool under_voltage = has_cut && v < v_cut;
    const bool over_esc = has_esc && motor_side_total(motors, std::span<const T>(currents)) > esc_max;
    T scale = T(1);
    if (over_current || under_voltage || over_esc) {
        T lo = T(0);
        T hi = T(1);
        for (int it = 0; it < 24; ++it) {
            const T mid = (lo + hi) * T(0.5);
            T vm = T(0);
            T ibm = T(0);
            uint32_t fm = 0;
            solve_at(mid, v_source, r0, motors, currents, vm, ibm, fm);
            const bool ok = (!has_ib_max || ibm <= ib_max) && (!has_cut || vm >= v_cut) &&
                            (!has_esc || motor_side_total(motors, std::span<const T>(currents)) <= esc_max);
            if (ok) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        scale = lo;
        flags = 0;
        solve_at(scale, v_source, r0, motors, currents, v, ib, flags);
        if (over_current) flags |= propulsion_flags::battery_current_limited;
        if (under_voltage) flags |= propulsion_flags::battery_cutoff;
        if (over_esc) flags |= propulsion_flags::esc_total_limited;
    }
    out.bus_voltage = is_finite(v) ? v : T(0);
    out.battery_current = is_finite(ib) ? ib : T(0);
    out.duty_scale = scale;
    out.flags = flags;
    return out;
}

}  // namespace

float battery_pack_resistance(uint32_t cells_series, uint32_t cells_parallel, float cell_resistance) noexcept {
    return pack_resistance(cells_series, cells_parallel, cell_resistance);
}
double battery_pack_resistance(uint32_t cells_series, uint32_t cells_parallel, double cell_resistance) noexcept {
    return pack_resistance(cells_series, cells_parallel, cell_resistance);
}

float battery_ocv(float soc, std::span<const float> table) noexcept { return ocv(soc, table); }
double battery_ocv(double soc, std::span<const float> table) noexcept { return ocv(soc, table); }

Result<std::vector<float>> battery_resample_ocv(std::span<const std::array<double, 2>> points, uint32_t count) {
    if (count < 2) {
        return std::unexpected(Error{Code::invalid_argument, "battery_resample_ocv: count must be at least 2"});
    }
    if (points.size() < 2) {
        return std::unexpected(
            Error{Code::invalid_argument, "battery_resample_ocv: at least two [SoC, V] points are needed"});
    }
    for (std::size_t k = 0; k < points.size(); ++k) {
        const double s = points[k][0];
        const double v = points[k][1];
        if (!is_finite(s) || !is_finite(v)) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "battery_resample_ocv: point " + std::to_string(k) + " is not finite"});
        }
        if (s < 0.0 || s > 1.0) {
            return std::unexpected(Error{Code::invalid_argument, "battery_resample_ocv: point " + std::to_string(k) +
                                                                     "'s SoC is outside [0, 1]"});
        }
        if (!(v > 0.0)) {
            return std::unexpected(Error{Code::invalid_argument, "battery_resample_ocv: point " + std::to_string(k) +
                                                                     "'s voltage is not positive"});
        }
        if (k > 0 && !(s > points[k - 1][0])) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "battery_resample_ocv: SoC is not strictly increasing at point " +
                                             std::to_string(k)});
        }
    }
    std::vector<float> out(count);
    std::size_t seg = 0;
    for (uint32_t g = 0; g < count; ++g) {
        const double s = static_cast<double>(g) / static_cast<double>(count - 1);
        double v = 0.0;
        if (s <= points.front()[0]) {
            v = points.front()[1];
        } else if (s >= points.back()[0]) {
            v = points.back()[1];
        } else {
            while (seg + 1 < points.size() && points[seg + 1][0] < s) ++seg;
            const double s0 = points[seg][0];
            const double s1 = points[seg + 1][0];
            const double v0 = points[seg][1];
            const double v1 = points[seg + 1][1];
            v = v0 + (v1 - v0) * (s - s0) / (s1 - s0);
        }
        out[g] = static_cast<float>(v);
    }
    return out;
}

float battery_rc_beta(float h, float r1, float c1) noexcept {
    if (!is_finite(h) || !(h > 0.0f)) return 1.0f;
    if (!is_finite(r1) || !is_finite(c1)) return 0.0f;
    const float tau = r1 * c1;
    if (!is_finite(tau) || !(tau > 0.0f)) return 0.0f;
    return math::exp32(-(h / tau));
}

float battery_polarization_step(float v1, float current, float r1, float beta) noexcept {
    return polarization_step(v1, current, r1, beta);
}
double battery_polarization_step(double v1, double current, double r1, double beta) noexcept {
    return polarization_step(v1, current, r1, beta);
}

float battery_soc_step(float soc, float current, float h, float capacity_ah, uint32_t& flags) noexcept {
    return soc_step(soc, current, h, capacity_ah, flags);
}
double battery_soc_step(double soc, double current, double h, double capacity_ah, uint32_t& flags) noexcept {
    return soc_step(soc, current, h, capacity_ah, flags);
}

float battery_terminal_voltage(float v_oc, float v1, float r0, float current) noexcept {
    return terminal_voltage(v_oc, v1, r0, current);
}
double battery_terminal_voltage(double v_oc, double v1, double r0, double current) noexcept {
    return terminal_voltage(v_oc, v1, r0, current);
}

BusResult bus_solve(float v_source, float r_series, float battery_current_max, float cutoff_voltage,
                    std::span<const BusMotor> motors, std::span<float> currents) noexcept {
    return bus(v_source, r_series, battery_current_max, cutoff_voltage, 0.0f, motors, currents);
}
BusResultD bus_solve(double v_source, double r_series, double battery_current_max, double cutoff_voltage,
                     std::span<const BusMotorD> motors, std::span<double> currents) noexcept {
    return bus(v_source, r_series, battery_current_max, cutoff_voltage, 0.0, motors, currents);
}
BusResult bus_solve(float v_source, float r_series, float battery_current_max, float cutoff_voltage,
                    float esc_current_total_max, std::span<const BusMotor> motors,
                    std::span<float> currents) noexcept {
    return bus(v_source, r_series, battery_current_max, cutoff_voltage, esc_current_total_max, motors, currents);
}
BusResultD bus_solve(double v_source, double r_series, double battery_current_max, double cutoff_voltage,
                     double esc_current_total_max, std::span<const BusMotorD> motors,
                     std::span<double> currents) noexcept {
    return bus(v_source, r_series, battery_current_max, cutoff_voltage, esc_current_total_max, motors, currents);
}

}  // namespace spade::vehicles
