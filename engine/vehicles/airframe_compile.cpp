#include "vehicles/airframe_compile.hpp"

// STUB: every function refuses or returns nothing, so the tests run red for
// the right reason. The implementation replaces this file in the next commit.

namespace spade::vehicles {

Result<PropulsionChain> airframe_propulsion_chain(const AirframeSpec&) {
    return std::unexpected(Error{Code::internal, "stub"});
}
std::vector<AirframeIssue> check_airframe(const AirframeSpec&) { return {}; }
Result<CompiledAirframe> compile_airframe(const AirframeSpec&) { return std::unexpected(Error{Code::internal, "stub"}); }

}  // namespace spade::vehicles
