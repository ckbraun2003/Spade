#include "vehicles/composite_inertia.hpp"

// STUB: every function returns zeros so the tests run red for the right reason.
// The implementation replaces this file in the next commit.

namespace spade::vehicles {

glm::dmat3 part_inertia_local(const PartInertia&) noexcept { return glm::dmat3(0.0); }
Result<CompositeInertia> composite_inertia(std::span<const PartInertia>) { return CompositeInertia{}; }

}  // namespace spade::vehicles
