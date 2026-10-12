#include "core/math_ops_f64.hpp"

namespace spade::math {

// RED stubs (Task B2 step 2): the tests fail on behaviour.
double atan2_arith(double, double) noexcept {
    return 0.0;
}

double corner_angle(const glm::dvec3&, const glm::dvec3&) noexcept {
    return 0.0;
}

TriangleClosest closest_on_triangle(glm::vec3, glm::vec3 a, glm::vec3, glm::vec3, uint32_t, uint32_t,
                                    uint32_t) noexcept {
    return {a, 0.0f, TriangleFeature::face};
}

}  // namespace spade::math
