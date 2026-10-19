#pragma once
// Analytic solids for the airflow solids tests: exact signed distances (or,
// for a wedge, a 1-Lipschitz bound), their gradients, rigid velocities, and
// their bounding boxes. They are not vehicle parts, so their bounds are their
// own, analytic; a vehicle's parts take vehicles::part_bounds() (TD-9).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "physics/airflow/solids.hpp"

namespace airflow_test {

using spade::physics::airflow::kNoSolidPart;
using spade::physics::airflow::SolidPart;
using spade::physics::airflow::SolidQuery;
using spade::physics::airflow::SolidSampler;

enum class Kind { sphere, box, cylinder_y, outside_cylinder_y, wedge_y, two_boxes, hemi_shell };

// One shape in the grid's frame. `to_local` turns a point's offset from the
// centre into the shape's own axes (a pure rotation; identity by default).
struct Shape {
    std::string name = "part";
    Kind kind = Kind::sphere;
    glm::vec3 centre{0.0f};
    glm::vec3 half{0.0f};          // box: half extents; wedge: (tip-to-back length, half height in y, 0)
    glm::vec3 centre2{0.0f};       // two_boxes: the second box, axis-aligned
    glm::vec3 half2{0.0f};
    float radius = 0.0f;           // sphere, cylinder: radius; wedge: half-angle, radians; hemi_shell: mid-surface radius
    glm::mat3 to_local{1.0f};      // local = to_local * (x - centre)
    glm::vec3 omega{0.0f};         // rigid rotation about `centre`, rad/s
    float min_feature = 0.0f;      // declared, m
    bool porous = false;
};

struct Field {
    float d;
    glm::vec3 g;  // grid frame, unit
};

// The box's exact distance and gradient in local coordinates.
[[nodiscard]] inline Field box_field(glm::vec3 p, glm::vec3 half) {
    const glm::vec3 q = glm::abs(p) - half;
    const glm::vec3 out = glm::max(q, glm::vec3(0.0f));
    const float out_len = std::sqrt(glm::dot(out, out));
    if (out_len > 0.0f) {
        glm::vec3 g = out / out_len;
        for (int a = 0; a < 3; ++a) g[a] = p[a] < 0.0f ? -g[a] : g[a];
        return {out_len, g};
    }
    int a = 0;
    if (q.y > q[a]) a = 1;
    if (q.z > q[a]) a = 2;
    glm::vec3 g(0.0f);
    g[a] = p[a] < 0.0f ? -1.0f : 1.0f;
    return {q[a], g};
}

[[nodiscard]] inline Field shape_field(const Shape& s, glm::vec3 x) {
    const glm::vec3 p = s.to_local * (x - s.centre);
    const glm::mat3 to_grid = glm::transpose(s.to_local);
    Field f{};
    switch (s.kind) {
        case Kind::sphere: {
            const float r = std::sqrt(glm::dot(p, p));
            f = {r - s.radius, r > 0.0f ? p / r : glm::vec3(1.0f, 0.0f, 0.0f)};
            break;
        }
        case Kind::box:
            f = box_field(p, s.half);
            break;
        case Kind::cylinder_y: {  // infinite along local y
            const float r = std::sqrt(p.x * p.x + p.z * p.z);
            f = {r - s.radius, r > 0.0f ? glm::vec3(p.x / r, 0.0f, p.z / r) : glm::vec3(1.0f, 0.0f, 0.0f)};
            break;
        }
        case Kind::outside_cylinder_y: {  // solid where r > radius (a fixed tube's wall)
            const float r = std::sqrt(p.x * p.x + p.z * p.z);
            f = {s.radius - r, r > 0.0f ? glm::vec3(-p.x / r, 0.0f, -p.z / r) : glm::vec3(1.0f, 0.0f, 0.0f)};
            break;
        }
        case Kind::hemi_shell: {  // a spherical shell, half.x thick each side of `radius`, cut to local y >= 0
            const float r = std::sqrt(glm::dot(p, p));
            const glm::vec3 radial = r > 0.0f ? p / r : glm::vec3(0.0f, 1.0f, 0.0f);
            const Field shell = r >= s.radius ? Field{r - s.radius - s.half.x, radial}
                                              : Field{s.radius - r - s.half.x, -radial};
            const Field cap{-p.y, glm::vec3(0.0f, -1.0f, 0.0f)};  // the max: a 1-Lipschitz bound, like the wedge's
            f = cap.d > shell.d ? cap : shell;
            break;
        }
        case Kind::two_boxes: {  // the union of two axis-aligned boxes: the smaller distance wins
            const Field a = box_field(p, s.half);
            const Field b = box_field(x - s.centre2, s.half2);
            f = b.d < a.d ? b : a;
            break;
        }
        case Kind::wedge_y: {
            // A wedge along local +x from its tip at the centre: |z| <= x tan(a),
            // 0 <= x <= L, |y| <= H, the intersection of half-spaces. The max
            // of their signed plane distances is a 1-Lipschitz bound (exact
            // inside and on the faces), which sphere tracing needs.
            const float ca = std::cos(s.radius);  // test-only libm: not engine code
            const float sa = std::sin(s.radius);
            const Field planes[5] = {
                {p.z * ca - p.x * sa, glm::vec3(-sa, 0.0f, ca)},   // the +z face
                {-p.z * ca - p.x * sa, glm::vec3(-sa, 0.0f, -ca)}, // the -z face
                {p.x - s.half.x, glm::vec3(1.0f, 0.0f, 0.0f)},     // the back
                {p.y - s.half.y, glm::vec3(0.0f, 1.0f, 0.0f)},
                {-p.y - s.half.y, glm::vec3(0.0f, -1.0f, 0.0f)},
            };
            f = planes[0];
            for (const Field& pl : planes) {
                if (pl.d > f.d) f = pl;
            }
            break;
        }
    }
    f.g = to_grid * f.g;
    return f;
}

[[nodiscard]] inline glm::vec3 shape_velocity(const Shape& s, glm::vec3 x) {
    return glm::cross(s.omega, x - s.centre);
}

// The shape's grid-frame bounding box and part-frame extent.
[[nodiscard]] inline SolidPart part_of(const Shape& s) {
    glm::vec3 local_half(0.0f);
    switch (s.kind) {
        case Kind::sphere: local_half = glm::vec3(s.radius); break;
        case Kind::box: local_half = s.half; break;
        case Kind::cylinder_y: local_half = glm::vec3(s.radius, 1.0e3f, s.radius); break;
        case Kind::outside_cylinder_y: local_half = glm::vec3(1.0e3f); break;
        case Kind::wedge_y: local_half = glm::vec3(s.half.x, s.half.y, s.half.x * std::tan(s.radius)); break;
        case Kind::two_boxes: break;  // its box is the union, below
        case Kind::hemi_shell: local_half = glm::vec3(s.radius + s.half.x); break;  // the cap's half below
    }
    SolidPart p;
    p.name = s.name;
    p.porous = s.porous;
    p.min_feature = s.min_feature;
    p.extent = 2.0f * local_half;
    if (s.kind == Kind::two_boxes) {
        p.box_min = glm::min(s.centre - s.half, s.centre2 - s.half2);
        p.box_max = glm::max(s.centre + s.half, s.centre2 + s.half2);
        p.extent = p.box_max - p.box_min;
        return p;
    }
    // The rotated box's eight corners (the wedge's box from its tip).
    const glm::mat3 to_grid = glm::transpose(s.to_local);
    const glm::vec3 lo_local = s.kind == Kind::wedge_y      ? glm::vec3(0.0f, -local_half.y, -local_half.z)
                               : s.kind == Kind::hemi_shell ? glm::vec3(-local_half.x, 0.0f, -local_half.z)
                                                            : -local_half;
    const glm::vec3 hi_local = s.kind == Kind::wedge_y ? glm::vec3(s.half.x, local_half.y, local_half.z) : local_half;
    if (s.kind == Kind::wedge_y || s.kind == Kind::hemi_shell) p.extent = hi_local - lo_local;
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(-std::numeric_limits<float>::max());
    for (int c = 0; c < 8; ++c) {
        const glm::vec3 corner((c & 1) ? hi_local.x : lo_local.x, (c & 2) ? hi_local.y : lo_local.y,
                               (c & 4) ? hi_local.z : lo_local.z);
        const glm::vec3 x = s.centre + to_grid * corner;
        lo = glm::min(lo, x);
        hi = glm::max(hi, x);
    }
    p.box_min = lo;
    p.box_max = hi;
    return p;
}

// A scene of shapes, sampled as sample_parts() does: the smallest distance
// wins, a tie to the lower index.
struct Scene {
    std::vector<Shape> shapes;

    [[nodiscard]] std::vector<SolidPart> parts() const {
        std::vector<SolidPart> out;
        for (const Shape& s : shapes) out.push_back(part_of(s));
        return out;
    }

    static SolidQuery nearest(const void* ctx, glm::vec3 x) noexcept {
        const auto& scene = *static_cast<const Scene*>(ctx);
        SolidQuery q;
        for (uint32_t i = 0; i < scene.shapes.size(); ++i) {
            const Field f = shape_field(scene.shapes[i], x);
            if (q.part == kNoSolidPart || f.d < q.distance) {
                q = SolidQuery{f.d, f.g, i, shape_velocity(scene.shapes[i], x)};
            }
        }
        return q;
    }

    static SolidQuery one(const void* ctx, uint32_t part, glm::vec3 x) noexcept {
        const auto& scene = *static_cast<const Scene*>(ctx);
        const Field f = shape_field(scene.shapes[part], x);
        return SolidQuery{f.d, f.g, part, shape_velocity(scene.shapes[part], x)};
    }

    [[nodiscard]] SolidSampler sampler() const { return SolidSampler{this, &Scene::nearest, &Scene::one}; }
};

}  // namespace airflow_test
