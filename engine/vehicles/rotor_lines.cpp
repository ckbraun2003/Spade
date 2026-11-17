#include "vehicles/rotor_lines.hpp"

namespace spade::vehicles {

Result<void> rotor_lines(std::span<const RotorRow>, const BodyState&, std::span<const float>, glm::vec3,
                         const physics::airflow::GridShape&, const physics::airflow::BladeShape&,
                         const physics::airflow::PolarTable&, std::span<physics::airflow::LineRotor>) {
    return std::unexpected(Error{Code::invalid_argument, "rotor lines: not written yet"});
}

}  // namespace spade::vehicles
