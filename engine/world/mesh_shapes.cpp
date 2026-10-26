#include "world/mesh_shapes.hpp"

#include <string>

namespace spade::shapes {

namespace {

[[nodiscard]] Error not_yet(const char* what) {
    return Error{Code::internal, std::string(what) + ": not built yet (L7a's RED)"};
}

}  // namespace

Result<GeneratedMesh> rounded_box(glm::dvec3, double, uint32_t) { return std::unexpected(not_yet("rounded_box")); }

Result<GeneratedMesh> capped_tube(double, double, uint32_t) { return std::unexpected(not_yet("capped_tube")); }

Result<GeneratedMesh> motor_bell(double, double, double, uint32_t) {
    return std::unexpected(not_yet("motor_bell"));
}

Result<GeneratedMesh> prop(const PropMeshSpec&, PropHand) { return std::unexpected(not_yet("prop")); }

}  // namespace spade::shapes
