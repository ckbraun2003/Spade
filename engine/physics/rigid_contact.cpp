#include "physics/rigid_contact.hpp"

#include <utility>

namespace spade::physics {

// L68 RED stubs: the declarations compile and the behaviour is missing. The GREEN commits replace them.

ShapeDesc sphere_shape(float radius) {
    ShapeDesc d;
    d.kind = ShapeKind::sphere;
    d.radius = radius;
    return d;
}

ShapeDesc capsule_shape(glm::vec3 a, glm::vec3 b, float radius) {
    ShapeDesc d;
    d.kind = ShapeKind::capsule;
    d.a = a;
    d.b = b;
    d.radius = radius;
    return d;
}

ShapeDesc box_shape(glm::vec3 half_extents, float skin) {
    ShapeDesc d;
    d.kind = ShapeKind::box;
    d.half_extents = half_extents;
    d.radius = skin;
    return d;
}

ShapeDesc hull_shape(std::vector<glm::vec3> points, float skin) {
    ShapeDesc d;
    d.kind = ShapeKind::hull;
    d.points = std::move(points);
    d.radius = skin;
    return d;
}

Result<ShapeRow> compile_shape(const ShapeDesc&) {
    return std::unexpected(Error{Code::unavailable, "set_body_shape: not implemented"});
}

glm::vec3 box_inv_inertia(float, glm::vec3) noexcept { return glm::vec3(0.0f); }

void resolve_rigid_contacts(std::span<BodyState>, std::span<const ShapeRow>, const SdfProgram&, const ContactParams&, glm::vec3, float,
                            std::span<ContactDvRow>) noexcept {}

}  // namespace spade::physics
