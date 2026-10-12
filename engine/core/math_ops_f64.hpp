#pragma once

// ---------------------------------------------------------------------------
// core/math_ops_f64.hpp -- Spade's own kernels for a triangle mesh (CORE-6;
// the parts and meshes plan, Task B2). INTERNAL: not installed.
// ---------------------------------------------------------------------------

#include <cstdint>

#include <glm/vec3.hpp>

namespace spade::math {

// atan2 in double, from + - * / and sqrt only.
[[nodiscard]] double atan2_arith(double y, double x) noexcept;

// The angle between a and b, in [0, pi]: atan2_arith(|a x b|, a . b).
[[nodiscard]] double corner_angle(const glm::dvec3& a, const glm::dvec3& b) noexcept;

// Which feature of a triangle (a, b, c) holds the closest point.
enum class TriangleFeature : uint32_t {
    face = 0,
    edge_ab,
    edge_bc,
    edge_ca,
    vertex_a,
    vertex_b,
    vertex_c,
};

struct TriangleClosest {
    glm::vec3 point{0.0f};  // the closest point
    float distance2 = 0.0f;  // |p - point|^2
    TriangleFeature feature = TriangleFeature::face;
};

// The closest point of the triangle (a, b, c) to p, fp32. ia, ib, ic are the
// vertices' indices in their mesh.
[[nodiscard]] TriangleClosest closest_on_triangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c,
                                                  uint32_t ia, uint32_t ib, uint32_t ic) noexcept;

}  // namespace spade::math
