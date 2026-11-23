#include "vehicles/rotor_lines.hpp"

#include <cmath>
#include <string>

#include "physics/airflow/rotor_discs.hpp"
#include "physics/airflow/rotor_lines.hpp"

namespace spade::vehicles {

Result<void> rotor_lines(std::span<const RotorRow> rows, const BodyState& body, std::span<const float> psi,
                         glm::vec3 grid_origin_world, const physics::airflow::GridShape& grid,
                         const physics::airflow::BladeShape& blade, const physics::airflow::PolarTable& polar,
                         std::span<physics::airflow::LineRotor> out, float eps_cells) {
    namespace airflow = physics::airflow;
    if (out.size() != rows.size() || psi.size() != rows.size()) {
        return std::unexpected(Error{Code::invalid_argument, "rotor lines: " + std::to_string(rows.size()) + " rotor rows, " +
                                                                 std::to_string(psi.size()) + " blade azimuths, " +
                                                                 std::to_string(out.size()) + " lines"});
    }
    const airflow::DiscBody b{.com_world = body.pos, .orient_world = body.orient};
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const RotorRow& row = rows[i];
        if (!(std::fabs(row.radius - blade.radius) <= 1.0e-4f)) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "rotor lines: rotor " + std::to_string(i) + ": the row's radius " +
                                             std::to_string(row.radius) + " m is not the blade's " +
                                             std::to_string(blade.radius) + " m"});
        }
        const airflow::LineRotorSpec spec{.hub_body = row.local_pos,
                                          .orient_body = row.local_orient,
                                          .spin_dir = row.spin_dir,
                                          .omega = row.enabled != 0u ? row.omega : 0.0f,
                                          .psi = psi[i],
                                          .blade = &blade,
                                          .polar = &polar,
                                          .eps_cells = eps_cells};
        Result<airflow::LineRotor> line = airflow::line_from_rotor(spec, b, grid_origin_world, grid);
        if (!line) {
            return std::unexpected(
                Error{line.error().code, "rotor lines: rotor " + std::to_string(i) + ": " + line.error().context});
        }
        out[i] = *line;
    }
    return {};
}

}  // namespace spade::vehicles
