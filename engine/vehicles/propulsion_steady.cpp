#include "vehicles/propulsion_steady.hpp"

#include <array>
#include <cmath>
#include <span>

#include "vehicles/battery.hpp"
#include "vehicles/motor.hpp"
#include "vehicles/propulsion_flags.hpp"

namespace spade::vehicles {
namespace {

[[nodiscard]] bool is_finite(double x) noexcept { return std::isfinite(x); }

[[nodiscard]] bool chain_valid(const PropulsionChain& c) noexcept {
    return is_finite(c.kv) && c.kv > 0.0 && is_finite(c.resistance) && c.resistance > 0.0 &&
           is_finite(c.diameter) && c.diameter > 0.0 && c.table.ct.size() >= 2 &&
           c.table.cq.size() == c.table.ct.size() && c.cells_series > 0 && c.cells_parallel > 0 &&
           !c.ocv_table.empty() && c.motors_on_bus >= 1 && c.motors_on_bus <= kMaxBusMotors;
}

// Everything the chain does at one shaft speed, with its own flags.
struct Eval {
    double balance = 0.0;  // Q_m - Q_load
    double drive_torque = 0.0;  // the motor's
    double load_torque = 0.0;   // the propeller's
    double current = 0.0;
    double bus_voltage = 0.0;
    double pack_current = 0.0;
    double duty_scale = 1.0;
    double advance_ratio = 0.0;
    uint32_t flags = 0;
};

[[nodiscard]] Eval evaluate(const PropulsionChain& c, double duty, double density, double v_axial, double soc,
                            double omega) noexcept {
    Eval e;
    const double r_eff = motor_effective_resistance(c.resistance, c.pole_pairs, c.inductance, omega);
    const double back_emf = omega / c.kv;

    std::array<BusMotorD, kMaxBusMotors> motors{};
    std::array<double, kMaxBusMotors> currents{};
    for (uint32_t k = 0; k < c.motors_on_bus; ++k) {
        motors[k] = BusMotorD{duty, back_emf, r_eff, c.current_min, c.current_max};
    }
    const double s = static_cast<double>(c.cells_series);
    const double v_source = s * battery_ocv(soc, std::span<const float>(c.ocv_table));
    const double r_series = battery_pack_resistance(c.cells_series, c.cells_parallel, c.cell_resistance) +
                            (is_finite(c.pack_polarization_resistance) && c.pack_polarization_resistance > 0.0
                                 ? c.pack_polarization_resistance
                                 : 0.0);
    const double cutoff = s * c.cell_cutoff_voltage;
    const BusResultD bus = bus_solve(v_source, r_series, c.pack_current_max, cutoff, c.esc_current_total_max,
                                     std::span<const BusMotorD>(motors.data(), c.motors_on_bus),
                                     std::span<double>(currents.data(), c.motors_on_bus));
    e.flags |= bus.flags;
    e.bus_voltage = bus.bus_voltage;
    e.pack_current = bus.battery_current;
    e.duty_scale = bus.duty_scale;
    e.current = currents[0];

    e.drive_torque = motor_torque(e.current, c.no_load_current, c.kv, omega);
    e.advance_ratio = propeller_advance_ratio(v_axial, omega, c.diameter);
    const double cq = propeller_coefficient(std::span<const float>(c.table.cq), static_cast<double>(c.table.j_min),
                                            static_cast<double>(c.table.j_max), e.advance_ratio, e.flags);
    e.load_torque = propeller_torque(cq, density, omega, c.diameter);
    e.balance = e.drive_torque - e.load_torque;
    return e;
}

[[nodiscard]] SteadyPoint point_at(const PropulsionChain& c, double duty, double density, double v_axial,
                                   double soc, double omega, uint32_t extra_flags) noexcept {
    const Eval e = evaluate(c, duty, density, v_axial, soc, omega);
    SteadyPoint p;
    p.duty = duty;
    p.duty_scale = e.duty_scale;
    p.omega = omega;
    p.advance_ratio = e.advance_ratio;
    uint32_t flags = e.flags | extra_flags;
    const double ct = propeller_coefficient(std::span<const float>(c.table.ct), static_cast<double>(c.table.j_min),
                                            static_cast<double>(c.table.j_max), e.advance_ratio, flags);
    p.thrust = propeller_thrust(ct, density, omega, c.diameter);
    p.torque = e.load_torque;
    p.current = e.current;
    p.bus_voltage = e.bus_voltage;
    p.pack_current = e.pack_current;
    p.power_electrical = e.bus_voltage * e.duty_scale * duty * e.current;
    p.power_shaft = e.drive_torque * omega;
    p.efficiency = p.power_electrical > 0.0 ? p.power_shaft / p.power_electrical : 0.0;
    p.flags = flags;
    return p;
}

}  // namespace

SteadyPoint steady_state_at_duty(const PropulsionChain& chain, double duty, double density, double v_axial,
                                 double soc) noexcept {
    if (!chain_valid(chain) || !is_finite(density) || !is_finite(v_axial) || !is_finite(soc)) {
        SteadyPoint bad;
        bad.flags = propulsion_flags::no_bracket;
        return bad;
    }
    uint32_t flags = 0;
    const double d = esc_clamp_duty(duty, flags);

    const double v_oc = static_cast<double>(chain.cells_series) *
                        battery_ocv(soc, std::span<const float>(chain.ocv_table));
    double lo = 0.0;
    double hi = chain.kv * d * v_oc;
    if (!(hi > 0.0)) return point_at(chain, d, density, v_axial, soc, 0.0, flags);

    const double f_lo = evaluate(chain, d, density, v_axial, soc, lo).balance;
    const double f_hi = evaluate(chain, d, density, v_axial, soc, hi).balance;
    if (!(f_lo > 0.0)) return point_at(chain, d, density, v_axial, soc, 0.0, flags);
    if (f_hi > 0.0) return point_at(chain, d, density, v_axial, soc, hi, flags | propulsion_flags::no_bracket);

    for (int it = 0; it < 100; ++it) {
        const double mid = 0.5 * (lo + hi);
        if (evaluate(chain, d, density, v_axial, soc, mid).balance > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return point_at(chain, d, density, v_axial, soc, 0.5 * (lo + hi), flags);
}

SteadyPoint steady_state_for_thrust(const PropulsionChain& chain, double thrust, double density, double v_axial,
                                    double soc) noexcept {
    if (!chain_valid(chain) || !is_finite(thrust) || !(thrust > 0.0)) {
        return steady_state_at_duty(chain, 0.0, density, v_axial, soc);
    }
    const SteadyPoint full = steady_state_at_duty(chain, 1.0, density, v_axial, soc);
    if (full.thrust < thrust) {
        SteadyPoint p = full;
        p.flags |= propulsion_flags::unreachable;
        return p;
    }
    double lo = 0.0;
    double hi = 1.0;
    for (int it = 0; it < 60; ++it) {
        const double mid = 0.5 * (lo + hi);
        if (steady_state_at_duty(chain, mid, density, v_axial, soc).thrust < thrust) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return steady_state_at_duty(chain, 0.5 * (lo + hi), density, v_axial, soc);
}

double steady_state_time_constant(const PropulsionChain& chain, double duty, double density, double v_axial,
                                  double soc, double rotor_inertia) noexcept {
    if (!chain_valid(chain) || !is_finite(rotor_inertia) || !(rotor_inertia > 0.0)) return 0.0;
    const SteadyPoint p = steady_state_at_duty(chain, duty, density, v_axial, soc);
    if ((p.flags & propulsion_flags::no_bracket) != 0u || !(p.omega > 0.0)) return 0.0;
    const double step = 1e-4 * p.omega;
    const double ahead = evaluate(chain, p.duty, density, v_axial, soc, p.omega + step).balance;
    const double behind = evaluate(chain, p.duty, density, v_axial, soc, p.omega - step).balance;
    const double slope = (ahead - behind) / (2.0 * step);
    if (!is_finite(slope) || !(slope < 0.0)) return 0.0;
    return rotor_inertia / -slope;
}

}  // namespace spade::vehicles
