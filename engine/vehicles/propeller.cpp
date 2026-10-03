#include "vehicles/propeller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "vehicles/propulsion_flags.hpp"

namespace spade::vehicles {
namespace {

template <typename T>
[[nodiscard]] bool is_finite(T x) noexcept {
    return std::isfinite(x);
}

// 2 pi and 1/(4 pi^2), each rounded once to T.
template <typename T>
inline constexpr T kTwoPi = T(6.283185307179586476925286766559);
template <typename T>
inline constexpr T kInvFourPiSq = T(0.025330295910584444);

template <typename T>
[[nodiscard]] T advance_ratio(T v_axial, T omega, T diameter) noexcept {
    if (!is_finite(v_axial) || !is_finite(omega) || !is_finite(diameter)) return T(0);
    if (!(omega > T(0)) || !(diameter > T(0))) return T(0);
    const T j = kTwoPi<T> * v_axial / (omega * diameter);
    return is_finite(j) ? j : T(0);
}

template <typename T>
[[nodiscard]] T coefficient(std::span<const float> table, T j_min, T j_max, T j, uint32_t& flags) noexcept {
    if (table.empty()) return T(0);
    if (!is_finite(j)) {
        flags |= propulsion_flags::out_of_table;
        return T(0);
    }
    if (table.size() == 1) return T(table[0]);
    if (!is_finite(j_min) || !is_finite(j_max) || !(j_max > j_min)) return T(0);
    if (j < j_min) {
        flags |= propulsion_flags::out_of_table;
        return T(table.front());
    }
    if (j > j_max) {
        flags |= propulsion_flags::out_of_table;
        return T(table.back());
    }
    const std::size_t last = table.size() - 1;
    const T x = (j - j_min) / (j_max - j_min) * T(last);
    std::size_t i = static_cast<std::size_t>(x);  // x >= 0: truncation is floor
    if (i >= last) i = last - 1;
    const T f = x - T(i);
    const T a = T(table[i]);
    const T b = T(table[i + 1]);
    return a + (b - a) * f;
}

template <typename T>
[[nodiscard]] T thrust(T ct, T density, T omega, T diameter) noexcept {
    if (!is_finite(ct) || !is_finite(density) || !is_finite(omega) || !is_finite(diameter)) return T(0);
    const T d2 = diameter * diameter;
    const T t = ct * density * (omega * omega * kInvFourPiSq<T>) * (d2 * d2);
    return is_finite(t) ? t : T(0);
}

template <typename T>
[[nodiscard]] T torque(T cq, T density, T omega, T diameter) noexcept {
    if (!is_finite(cq) || !is_finite(density) || !is_finite(omega) || !is_finite(diameter)) return T(0);
    const T d2 = diameter * diameter;
    const T q = cq * density * (omega * omega * kInvFourPiSq<T>) * (d2 * d2 * diameter);
    return is_finite(q) ? q : T(0);
}

[[nodiscard]] Result<void> check_points(std::span<const std::array<double, 2>> points, const char* what) {
    if (points.size() < 2) {
        return std::unexpected(Error{Code::invalid_argument,
                                     std::string("propeller_resample_table: ") + what + " needs at least 2 points"});
    }
    for (std::size_t k = 0; k < points.size(); ++k) {
        if (!is_finite(points[k][0]) || !is_finite(points[k][1])) {
            return std::unexpected(Error{Code::invalid_argument, std::string("propeller_resample_table: ") + what +
                                                                     " point " + std::to_string(k) +
                                                                     " is not finite"});
        }
        if (k > 0 && !(points[k][0] > points[k - 1][0])) {
            return std::unexpected(Error{Code::invalid_argument, std::string("propeller_resample_table: ") + what +
                                                                     " J is not strictly increasing at point " +
                                                                     std::to_string(k)});
        }
    }
    return {};
}

[[nodiscard]] double interpolate(std::span<const std::array<double, 2>> points, double j) noexcept {
    if (j <= points.front()[0]) return points.front()[1];
    if (j >= points.back()[0]) return points.back()[1];
    std::size_t seg = 0;
    while (seg + 1 < points.size() && points[seg + 1][0] < j) ++seg;
    const double j0 = points[seg][0];
    const double j1 = points[seg + 1][0];
    return points[seg][1] + (points[seg + 1][1] - points[seg][1]) * (j - j0) / (j1 - j0);
}

}  // namespace

Result<PropellerTable> propeller_resample_table(std::span<const std::array<double, 2>> ct_points,
                                                std::span<const std::array<double, 2>> cq_points, uint32_t count) {
    if (count < 2) {
        return std::unexpected(Error{Code::invalid_argument, "propeller_resample_table: count must be at least 2"});
    }
    if (Result<void> r = check_points(ct_points, "C_T"); !r) return std::unexpected(r.error());
    if (Result<void> r = check_points(cq_points, "C_Q"); !r) return std::unexpected(r.error());

    const double j_lo = std::min(ct_points.front()[0], cq_points.front()[0]);
    const double j_hi = std::max(ct_points.back()[0], cq_points.back()[0]);
    PropellerTable out;
    out.j_min = static_cast<float>(j_lo);
    out.j_max = static_cast<float>(j_hi);
    out.ct.resize(count);
    out.cq.resize(count);
    for (uint32_t g = 0; g < count; ++g) {
        const double j = j_lo + (j_hi - j_lo) * static_cast<double>(g) / static_cast<double>(count - 1);
        out.ct[g] = static_cast<float>(interpolate(ct_points, j));
        out.cq[g] = static_cast<float>(interpolate(cq_points, j));
    }
    return out;
}

float propeller_advance_ratio(float v_axial, float omega, float diameter) noexcept {
    return advance_ratio(v_axial, omega, diameter);
}
double propeller_advance_ratio(double v_axial, double omega, double diameter) noexcept {
    return advance_ratio(v_axial, omega, diameter);
}

float propeller_coefficient(std::span<const float> table, float j_min, float j_max, float j,
                            uint32_t& flags) noexcept {
    return coefficient(table, j_min, j_max, j, flags);
}
double propeller_coefficient(std::span<const float> table, double j_min, double j_max, double j,
                             uint32_t& flags) noexcept {
    return coefficient(table, j_min, j_max, j, flags);
}

float propeller_thrust(float ct, float density, float omega, float diameter) noexcept {
    return thrust(ct, density, omega, diameter);
}
double propeller_thrust(double ct, double density, double omega, double diameter) noexcept {
    return thrust(ct, density, omega, diameter);
}

float propeller_torque(float cq, float density, float omega, float diameter) noexcept {
    return torque(cq, density, omega, diameter);
}
double propeller_torque(double cq, double density, double omega, double diameter) noexcept {
    return torque(cq, density, omega, diameter);
}

}  // namespace spade::vehicles
