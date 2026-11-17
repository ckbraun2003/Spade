#include "physics/airflow/rotor_lines.hpp"

#include <cmath>
#include <string>

namespace spade::physics::airflow {

namespace {

[[nodiscard]] bool finite3(glm::vec3 v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Finite, and of unit length within the same tolerance as the discs' (rotor_discs.cpp).
[[nodiscard]] bool unit(const glm::quat& q) noexcept {
    const float n = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return std::isfinite(n) && std::fabs(n - 1.0f) <= 1.0e-5f;
}

}  // namespace

Result<LineRotor> line_from_rotor(const LineRotorSpec& r, const DiscBody& body, glm::vec3 grid_origin_world,
                                  const GridShape&) {
    const auto refuse = [](const std::string& why) -> Result<LineRotor> {
        return std::unexpected(Error{Code::invalid_argument, "airflow rotor line: " + why});
    };
    if (!finite3(body.com_world) || !finite3(grid_origin_world)) return refuse("the body's or the grid's position (pose) is not finite");
    if (!unit(body.orient_world)) return refuse("the body's orientation is not a unit quaternion");
    if (!finite3(r.hub_body)) return refuse("the hub is not finite");
    if (!unit(r.orient_body)) return refuse("the rotor's orientation is not a unit quaternion");
    if (r.spin_dir != 1.0f && r.spin_dir != -1.0f) {
        return refuse("the spin is " + std::to_string(r.spin_dir) + ", not -1 or +1 (a line's blades always turn)");
    }
    if (!std::isfinite(r.omega) || r.omega < 0.0f) return refuse("omega is not finite and at least 0");
    if (!std::isfinite(r.psi)) return refuse("psi (the blade azimuth) is not finite");
    if (r.blade == nullptr) return refuse("no blade");
    if (r.polar == nullptr) return refuse("no polar");

    const glm::quat to_world = body.orient_world * r.orient_body;
    LineRotor line;
    line.centre = body.com_world + body.orient_world * r.hub_body - grid_origin_world;
    line.axis = to_world * glm::vec3(0.0f, 1.0f, 0.0f);
    line.ref = to_world * glm::vec3(1.0f, 0.0f, 0.0f);
    line.spin = r.spin_dir;
    line.omega = r.omega;
    line.psi = r.psi;
    line.blade = r.blade;
    line.polar = r.polar;
    return line;
}

}  // namespace spade::physics::airflow
