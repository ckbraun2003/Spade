// The analysis's coefficients (see the header for the definitions).

#include "analysis/coefficients.hpp"

#include <cmath>
#include <format>
#include <numbers>
#include <string>

namespace spade::analysis {

namespace {

constexpr double kPi = std::numbers::pi;

[[nodiscard]] std::unexpected<Error> refuse(std::string why) {
    return std::unexpected(Error{Code::invalid_argument, std::move(why)});
}

}  // namespace

Convention default_convention(FlightKind kind) noexcept {
    return kind == FlightKind::hover ? Convention::rotorcraft : Convention::propeller;
}

Result<std::vector<Coefficient>> rotor_coefficients(Convention convention, const RotorLoads& loads, float density,
                                                    float freestream) {
    if (!std::isfinite(loads.omega) || loads.omega <= 0.0f) {
        return refuse(std::format("a stopped rotor (Omega {} rad/s) has no coefficients", loads.omega));
    }
    if (!std::isfinite(loads.radius) || loads.radius <= 0.0f) {
        return refuse(std::format("a rotor radius of {} m has no coefficients", loads.radius));
    }
    if (!std::isfinite(density) || density <= 0.0f) {
        return refuse(std::format("a density of {} kg/m3 has no coefficients", density));
    }
    if (!std::isfinite(freestream) || freestream < 0.0f) {
        return refuse(std::format("a freestream of {} m/s must be at least 0", freestream));
    }
    if (!std::isfinite(loads.thrust) || !std::isfinite(loads.torque)) {
        return refuse("the rotor's thrust and torque must be finite");
    }
    const double t = loads.thrust;
    const double q = loads.torque;
    const double omega = loads.omega;
    const double r = loads.radius;
    const double rho = density;
    const double v = freestream;
    const double power = q * omega;
    std::vector<Coefficient> out;
    if (convention == Convention::rotorcraft) {
        const double area = kPi * r * r;
        const double tip = omega * r;
        const double ct = t / (rho * area * tip * tip);
        const double cp = power / (rho * area * tip * tip * tip);
        out.push_back({"C_T", "C_T = T / (rho A (Omega R)^2), A = pi R^2 (rotorcraft)", ct});
        out.push_back({"C_Q", "C_Q = Q / (rho A (Omega R)^2 R) (rotorcraft)", q / (rho * area * tip * tip * r)});
        out.push_back({"C_P", "C_P = P / (rho A (Omega R)^3), P = Q Omega (rotorcraft; equals C_Q)", cp});
        if (v == 0.0 && ct >= 0.0 && cp > 0.0) {
            out.push_back({"FM", "FM = C_T^(3/2) / (sqrt(2) C_P): ideal over actual power, in hover",
                           std::pow(ct, 1.5) / (std::sqrt(2.0) * cp)});
        }
    } else {
        const double n = omega / (2.0 * kPi);
        const double d = 2.0 * r;
        const double ct = t / (rho * n * n * std::pow(d, 4));
        const double cp = power / (rho * n * n * n * std::pow(d, 5));
        const double j = v / (n * d);
        out.push_back({"C_T", "C_T = T / (rho n^2 D^4), n = Omega / (2 pi), D = 2 R (propeller)", ct});
        out.push_back({"C_Q", "C_Q = Q / (rho n^2 D^5) (propeller)", q / (rho * n * n * std::pow(d, 5))});
        out.push_back({"C_P", "C_P = P / (rho n^3 D^5), P = Q Omega (propeller; 2 pi C_Q)", cp});
        out.push_back({"J", "J = V / (n D): the advance ratio", j});
        if (v > 0.0 && cp > 0.0) {
            out.push_back({"eta", "eta = J C_T / C_P = T V / P: the propulsive efficiency", j * ct / cp});
        }
    }
    return out;
}

Result<double> pressure_coefficient(float pressure, float reference, float density, float reference_speed) {
    if (!std::isfinite(reference_speed) || reference_speed <= 0.0f) {
        return refuse(std::format("C_p needs a reference speed above 0, not {} m/s", reference_speed));
    }
    if (!std::isfinite(density) || density <= 0.0f) {
        return refuse(std::format("C_p needs a density above 0, not {} kg/m3", density));
    }
    const double q = 0.5 * static_cast<double>(density) * static_cast<double>(reference_speed) *
                     static_cast<double>(reference_speed);
    return (static_cast<double>(pressure) - static_cast<double>(reference)) / q;
}

float reference_speed(Convention convention, float omega, float radius, float freestream) noexcept {
    return convention == Convention::rotorcraft ? omega * radius : freestream;
}

}  // namespace spade::analysis
