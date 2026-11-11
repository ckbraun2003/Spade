#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "physics/contact_dv.hpp"
#include "physics/contacts.hpp"
#include "state/layout.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// physics/rigid_contact.hpp -- contact tumbling (L68; docs/design/physics/plans/2026-11-11-l68-contact-tumbling.md). A body with a
// SHAPE (sphere, capsule, box or convex hull) meets the world SDF at several points, and each contact's impulse acts at its
// point, so `r x J` goes into omega_body through inv_inertia_diag: a corner hit spins the body up and it rolls and settles. Friction
// and restitution act per point. The module is OPT-IN (sim/module.hpp's rigid_contact_module()); a set without it, and a body
// without a shape in a set with it, are untouched, bit for bit.
//
// THE SHAPE ROW (ShapeRow, the per_body array rigid_contact.shape, 288 bytes): every shape is compiled to a point list (body frame,
// the centre of mass at the origin, the principal axes) plus a radius, so one contact test serves all four.
//   sphere   one point at the origin, radius r;
//   capsule  points on the segment a-b from a to b, spaced at most r apart, ends included, radius r;
//   box      its 8 corners, corner c taking +h.x if c & 1, +h.y if c & 2, +h.z if c & 4 (else minus), radius = the skin (default 0);
//   hull     the host's points in the host's order, at most kShapeMaxPoints, radius = the skin (default 0).
// A sphere ROLLS ON: there is no rolling resistance, so a sphere on a flat floor keeps its speed once omega r = v (a box or a capsule
// stops). Body-body contact is not part of this pass.
// ---------------------------------------------------------------------------

namespace spade::physics {

inline constexpr uint32_t kShapeMaxPoints = 16;
// Sweeps of the velocity solve per substep: the default (a row's iterations lane 0) and the most a row may ask for.
inline constexpr uint32_t kDefaultSweeps = 4;
inline constexpr uint32_t kMaxSweeps = 16;

// rigid_contact.resolve's index of the shape array in its access list (sim/standard_modules.cpp pins it); the contact scratch is at
// kContactPassContactDvAccess, as in the other two contact passes.
inline constexpr std::size_t kRigidContactShapeAccess = 2;

enum class ShapeKind : uint32_t { none = 0, sphere = 1, capsule = 2, box = 3, hull = 4 };

// What the host describes. Negative friction/restitution mean "this world's ContactParams" (the SDF carries no material, so a
// world has one surface material). rest_speed 0 is the default 0.1 m/s; the pass uses max(rest_speed, 2 |g| h). iterations is the
// number of sweeps of the contact solve per substep: 0 is the default (kDefaultSweeps), at most kMaxSweeps.
struct ShapeDesc {
    ShapeKind kind = ShapeKind::none;  // none clears the body's shape
    float radius = 0.0f;               // sphere and capsule: the radius (> 0); box and hull: the skin (>= 0)
    glm::vec3 a{0.0f};                 // capsule: one end, body frame
    glm::vec3 b{0.0f};                 // capsule: the other end
    glm::vec3 half_extents{0.0f};      // box: half sizes (> 0)
    std::vector<glm::vec3> points;     // hull: 1 to kShapeMaxPoints body-frame points
    float friction = -1.0f;            // Coulomb mu, >= 0; negative: the world's
    float restitution = -1.0f;         // e in [0, 1]; negative: the world's
    float rest_speed = 0.0f;           // m/s, >= 0
    uint32_t iterations = 0;           // sweeps of the contact solve per substep: 0 = kDefaultSweeps, else 1..kMaxSweeps
};

[[nodiscard]] ShapeDesc sphere_shape(float radius);
[[nodiscard]] ShapeDesc capsule_shape(glm::vec3 a, glm::vec3 b, float radius);
[[nodiscard]] ShapeDesc box_shape(glm::vec3 half_extents, float skin = 0.0f);
[[nodiscard]] ShapeDesc hull_shape(std::vector<glm::vec3> points, float skin = 0.0f);

// The row as the arena holds it: 18 float4 rows. Row 0 {kind, count, radius, rest_speed}, row 1 {friction, restitution, iterations,
// 0}, then kShapeMaxPoints points {x, y, z, 0}. All zero is "no shape".
struct alignas(kStd430StructAlignment) ShapeRow {
    uint32_t kind = 0;
    uint32_t count = 0;
    float radius = 0.0f;
    float rest_speed = 0.0f;
    float friction = 0.0f;
    float restitution = 0.0f;
    uint32_t iterations = 0;
    uint32_t _r0 = 0;
    float points[kShapeMaxPoints][4]{};
};
static_assert(std::is_standard_layout_v<ShapeRow> && std::is_trivially_copyable_v<ShapeRow>);
static_assert(sizeof(ShapeRow) == 288 && alignof(ShapeRow) == 16, "18 float4 rows");

// The row a description compiles to, or a refusal by name (invalid_argument; every text opens "set_body_shape:"): a negative or
// non-finite radius, a capsule or sphere without a radius, a capsule with equal ends, a box with a non-positive half extent, a hull
// with no point or more than kShapeMaxPoints, a non-finite point, friction < 0 only as the sentinel (NaN refused), a restitution
// above 1, a negative rest_speed.
[[nodiscard]] Result<ShapeRow> compile_shape(const ShapeDesc& desc);

// 1/I about each principal axis of a solid box of `mass` and `half_extents`: 3 / (mass (hy^2 + hz^2)) and so on.
[[nodiscard]] glm::vec3 box_inv_inertia(float mass, glm::vec3 half_extents) noexcept;

// static_contact's sibling for shaped bodies: every ACTIVE body whose row in `shapes` (indexed like `bodies`) has a count above 0.
// `gravity` and `h` give the restitution threshold's floor. `contact_dv`: the contact scratch, as resolve_static_contacts has it.
void resolve_rigid_contacts(std::span<BodyState> bodies, std::span<const ShapeRow> shapes, const SdfProgram& world_sdf,
                            const ContactParams& params, glm::vec3 gravity, float h, std::span<ContactDvRow> contact_dv = {}) noexcept;

}  // namespace spade::physics
