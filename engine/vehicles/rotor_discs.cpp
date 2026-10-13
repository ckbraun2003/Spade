#include "vehicles/rotor_discs.hpp"

#include <string>

#include "physics/airflow/rotor_discs.hpp"

namespace spade::vehicles {

Result<void> rotor_discs(std::span<const RotorRow> rows, const BodyState& body, glm::vec3 grid_origin_world,
                         const physics::airflow::GridShape& grid, std::span<physics::airflow::ActuatorDisc> out) {
    namespace airflow = physics::airflow;
    if (out.size() != rows.size()) {
        return std::unexpected(Error{Code::invalid_argument, "rotor discs: " + std::to_string(rows.size()) +
                                                                 " rotor rows into " + std::to_string(out.size()) +
                                                                 " discs"});
    }
    const airflow::DiscBody b{.com_world = body.pos, .orient_world = body.orient};
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const RotorRow& row = rows[i];
        // apply_rotors()'s curves (rotor.cpp, step 3): one w^2 for both.
        const float omega_sq = row.omega * row.omega;
        const bool on = row.enabled != 0u;
        const airflow::DiscRotor r{.hub_body = row.local_pos,
                                   .orient_body = row.local_orient,
                                   .radius = row.radius,
                                   .spin_dir = row.spin_dir,
                                   .thrust = on ? row.thrust_coeff * omega_sq : 0.0f,
                                   .torque = on ? row.torque_coeff * omega_sq : 0.0f};
        Result<airflow::ActuatorDisc> d = airflow::disc_from_rotor(r, b, grid_origin_world, grid);
        if (!d) {
            return std::unexpected(
                Error{d.error().code, "rotor discs: rotor " + std::to_string(i) + ": " + d.error().context});
        }
        out[i] = *d;
    }
    return {};
}

}  // namespace spade::vehicles
