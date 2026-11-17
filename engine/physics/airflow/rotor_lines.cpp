#include "physics/airflow/rotor_lines.hpp"

namespace spade::physics::airflow {

Result<LineRotor> line_from_rotor(const LineRotorSpec&, const DiscBody&, glm::vec3, const GridShape&) {
    return std::unexpected(Error{Code::invalid_argument, "airflow rotor line: not written yet"});
}

}  // namespace spade::physics::airflow
