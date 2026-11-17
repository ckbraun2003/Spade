#include "core/geodesy.hpp"

// RED stub: every function answers zero until the GREEN commit (L85 SP-1).
namespace spade::geo {

Ecef geodetic_to_ecef(const Geodetic&) noexcept { return {}; }
Geodetic ecef_to_geodetic(const Ecef&) noexcept { return {}; }
LocalFrame make_local_frame(const Geodetic&) noexcept { return {}; }
Ecef enu_to_ecef(const LocalFrame&, const glm::dvec3&) noexcept { return {}; }
glm::dvec3 ecef_to_enu(const LocalFrame&, const Ecef&) noexcept { return glm::dvec3(0.0); }
Geodetic world_to_geodetic(const LocalFrame&, const glm::dvec3&) noexcept { return {}; }
glm::dvec3 geodetic_to_world(const LocalFrame&, const Geodetic&) noexcept { return glm::dvec3(0.0); }
Result<void> validate_anchor(const Geodetic&) { return {}; }
Geodetic canonical_anchor(const Geodetic& anchor) noexcept { return anchor; }
uint64_t anchor_identity(const Geodetic&) noexcept { return 0; }

}  // namespace spade::geo
