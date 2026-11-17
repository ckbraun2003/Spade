#include "scene/prop_line.hpp"

namespace spade::scene {

Result<LinePropeller> line_propeller(const PropPart&) {
    return std::unexpected(Error{Code::invalid_argument, "line propeller: not written yet"});
}

}  // namespace spade::scene
