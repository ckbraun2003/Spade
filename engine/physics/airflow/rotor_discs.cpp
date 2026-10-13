#include "physics/airflow/rotor_discs.hpp"

#include <cmath>
#include <string>

namespace spade::physics::airflow {

namespace {

[[nodiscard]] bool finite3(glm::vec3 v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Finite, and of unit length within check_disc()'s tolerance on an axis.
[[nodiscard]] bool unit(const glm::quat& q) noexcept {
    const float n = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return std::isfinite(n) && std::fabs(n - 1.0f) <= 1.0e-5f;
}

}  // namespace

Result<ActuatorDisc> disc_from_rotor(const DiscRotor& r, const DiscBody& body, glm::vec3 grid_origin_world,
                                     const GridShape& grid) {
    const auto refuse = [](const std::string& why) -> Result<ActuatorDisc> {
        return std::unexpected(Error{Code::invalid_argument, why});
    };
    if (!finite3(body.com_world) || !finite3(grid_origin_world)) return refuse("the body's or the grid's position is not finite");
    if (!unit(body.orient_world)) return refuse("the body's orientation is not a unit quaternion");
    if (!finite3(r.hub_body)) return refuse("the hub is not finite");
    if (!unit(r.orient_body)) return refuse("the rotor's orientation is not a unit quaternion");
    if (r.spin_dir != 1.0f && r.spin_dir != -1.0f && r.spin_dir != 0.0f) {
        return refuse("the spin is " + std::to_string(r.spin_dir) + ", not -1, 0 or +1 (a direction, not a scale)");
    }
    if (!std::isfinite(r.thrust) || !std::isfinite(r.torque)) return refuse("the thrust or the torque is not finite");
    if (r.torque < 0.0f) return refuse("the torque magnitude is negative; the spin carries the sense");

    const glm::vec3 thrust_axis = body.orient_world * (r.orient_body * glm::vec3(0.0f, 1.0f, 0.0f));
    ActuatorDisc d{.centre = body.com_world + body.orient_world * r.hub_body - grid_origin_world,
                   .axis = -thrust_axis,
                   .radius = r.radius,
                   .thrust = r.thrust,
                   .torque = -r.spin_dir * r.torque};
    if (Result<void> ok = check_disc(d, grid); !ok) return std::unexpected(ok.error());
    return d;
}

Result<void> discs_from_rotors(std::span<const DiscRotor> rotors, const DiscBody& body, glm::vec3 grid_origin_world,
                               const GridShape& grid, std::span<ActuatorDisc> out) {
    if (out.size() != rotors.size()) {
        return std::unexpected(Error{Code::invalid_argument, "airflow rotor discs: " + std::to_string(rotors.size()) +
                                                                 " rotors into " + std::to_string(out.size()) +
                                                                 " discs"});
    }
    for (std::size_t i = 0; i < rotors.size(); ++i) {
        Result<ActuatorDisc> d = disc_from_rotor(rotors[i], body, grid_origin_world, grid);
        if (!d) {
            return std::unexpected(
                Error{d.error().code, "airflow rotor discs: rotor " + std::to_string(i) + ": " + d.error().context});
        }
        out[i] = *d;
    }
    return {};
}

}  // namespace spade::physics::airflow
