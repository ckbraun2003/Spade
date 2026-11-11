#include "physics/rigid_contact.hpp"

#include <cmath>
#include <string>
#include <utility>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/integrator.hpp"  // body_flags::active
#include "world/sdf.hpp"

namespace spade::physics {

namespace {

[[nodiscard]] Error refuse(const std::string& what) {
    return Error{Code::invalid_argument, "set_body_shape: " + what};
}

[[nodiscard]] bool finite3(glm::vec3 v) noexcept { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

void put_point(ShapeRow& row, uint32_t k, glm::vec3 p) noexcept {
    row.points[k][0] = p.x;
    row.points[k][1] = p.y;
    row.points[k][2] = p.z;
    row.points[k][3] = 0.0f;
}

}  // namespace

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

Result<ShapeRow> compile_shape(const ShapeDesc& desc) {
    ShapeRow row{};
    if (desc.kind == ShapeKind::none) return row;  // all zero: no shape

    // The material and the thresholds first: they are the same for every kind. A negative friction or restitution is the sentinel "this
    // world's", stored as -1; NaN is refused.
    if (std::isnan(desc.friction) || std::isinf(desc.friction)) {
        return std::unexpected(refuse("friction must be finite (negative means the world's)"));
    }
    if (std::isnan(desc.restitution) || desc.restitution > 1.0f) {
        return std::unexpected(refuse("restitution must be in [0, 1] (negative means the world's)"));
    }
    if (!std::isfinite(desc.rest_speed) || desc.rest_speed < 0.0f) return std::unexpected(refuse("rest_speed must be finite and >= 0"));
    if (desc.iterations > kMaxSweeps) {
        return std::unexpected(refuse("iterations must be 0 (the default) to " + std::to_string(kMaxSweeps) + ", not " + std::to_string(desc.iterations)));
    }
    row.friction = desc.friction < 0.0f ? -1.0f : desc.friction;
    row.restitution = desc.restitution < 0.0f ? -1.0f : desc.restitution;
    row.rest_speed = desc.rest_speed;
    row.iterations = desc.iterations;
    row.kind = static_cast<uint32_t>(desc.kind);

    switch (desc.kind) {
        case ShapeKind::sphere: {
            if (!(desc.radius > 0.0f) || !std::isfinite(desc.radius)) {
                return std::unexpected(refuse("a sphere's radius must be finite and > 0"));
            }
            row.radius = desc.radius;
            row.count = 1;
            put_point(row, 0, glm::vec3(0.0f));
            return row;
        }
        case ShapeKind::capsule: {
            if (!(desc.radius > 0.0f) || !std::isfinite(desc.radius)) {
                return std::unexpected(refuse("a capsule's radius must be finite and > 0"));
            }
            if (!finite3(desc.a) || !finite3(desc.b)) return std::unexpected(refuse("a capsule's ends must be finite"));
            const float length = glm::length(desc.b - desc.a);
            if (!(length > 0.0f)) return std::unexpected(refuse("a capsule's ends coincide: use a sphere"));
            // Points on the segment spaced at most `radius` apart, ends included: the fewest intervals n with n * radius >= length (to a
            // part in 1e5, so 0.6 m at 0.05 m is 12 intervals, not 13).
            const float need = length * (1.0f - 1e-5f);
            uint32_t intervals = 1;
            while (intervals < kShapeMaxPoints && static_cast<float>(intervals) * desc.radius < need) ++intervals;
            if (intervals + 1 > kShapeMaxPoints) {
                return std::unexpected(refuse("a capsule of length " + std::to_string(length) + " m and radius " +
                                              std::to_string(desc.radius) + " m needs more than " + std::to_string(kShapeMaxPoints) +
                                              " points (16 at most); shorten it or widen it"));
            }
            row.radius = desc.radius;
            row.count = intervals + 1;
            for (uint32_t k = 0; k <= intervals; ++k) {
                const glm::vec3 p = (k == 0)           ? desc.a
                                    : (k == intervals) ? desc.b
                                                       : desc.a + (desc.b - desc.a) * (static_cast<float>(k) / static_cast<float>(intervals));
                put_point(row, k, p);
            }
            return row;
        }
        case ShapeKind::box: {
            if (!(desc.half_extents.x > 0.0f) || !(desc.half_extents.y > 0.0f) || !(desc.half_extents.z > 0.0f) ||
                !finite3(desc.half_extents)) {
                return std::unexpected(refuse("a box's half_extents must be finite and > 0 on every axis"));
            }
            if (!std::isfinite(desc.radius) || desc.radius < 0.0f) return std::unexpected(refuse("a box's skin radius must be finite and >= 0"));
            row.radius = desc.radius;
            row.count = 8;
            for (uint32_t c = 0; c < 8; ++c) {  // corner c: +h.x if c & 1, +h.y if c & 2, +h.z if c & 4, else the minus
                put_point(row, c,
                          glm::vec3((c & 1u) ? desc.half_extents.x : -desc.half_extents.x, (c & 2u) ? desc.half_extents.y : -desc.half_extents.y,
                                    (c & 4u) ? desc.half_extents.z : -desc.half_extents.z));
            }
            return row;
        }
        case ShapeKind::hull: {
            if (desc.points.empty() || desc.points.size() > kShapeMaxPoints) {
                return std::unexpected(refuse("a hull needs 1 to " + std::to_string(kShapeMaxPoints) + " points, not " +
                                              std::to_string(desc.points.size())));
            }
            if (!std::isfinite(desc.radius) || desc.radius < 0.0f) return std::unexpected(refuse("a hull's skin radius must be finite and >= 0"));
            for (std::size_t k = 0; k < desc.points.size(); ++k) {
                if (!finite3(desc.points[k])) return std::unexpected(refuse("hull point " + std::to_string(k) + " is not finite"));
            }
            row.radius = desc.radius;
            row.count = static_cast<uint32_t>(desc.points.size());
            for (uint32_t k = 0; k < row.count; ++k) put_point(row, k, desc.points[k]);
            return row;
        }
        case ShapeKind::none: break;
    }
    return std::unexpected(refuse("unknown shape kind"));
}

glm::vec3 box_inv_inertia(float mass, glm::vec3 half_extents) noexcept {
    const float x2 = half_extents.x * half_extents.x;
    const float y2 = half_extents.y * half_extents.y;
    const float z2 = half_extents.z * half_extents.z;
    return glm::vec3(3.0f / (mass * (y2 + z2)), 3.0f / (mass * (x2 + z2)), 3.0f / (mass * (x2 + y2)));
}

// ---------------------------------------------------------------------------------------------------------------------------------------
// The pass: sequential impulses on the points' contacts. THE NUMBERED OP ORDER BELOW IS THE PARITY CONTRACT: collision_shape.slang performs the
// same operations, in the same order, with the same groupings, in fp32 (the divides and square roots through fp32_math's div32_ftz and
// sqrt32_ftz). Bodies in slot order, points in row order.
//
// A: the contacts, found ONCE per substep (one SDF walk per point; the points do not move inside the substep's velocity sweeps).
// B: `sweeps` passes over the found contacts, each point's normal impulse and then its friction impulse, with ACCUMULATED impulses: the
//    normal one clamped at >= 0 and the friction one inside the cone |j_t| <= mu j_n, so a later sweep can take back what an earlier one over-did.
//    Without that a body rests on jitter (a rocking box leaks into a lateral creep). One sweep with nothing accumulated is the plain
//    "approaching points only" response.
// C: the positional correction of the deepest point.
// ---------------------------------------------------------------------------------------------------------------------------------------
void resolve_rigid_contacts(std::span<BodyState> bodies, std::span<const ShapeRow> shapes, const SdfProgram& world_sdf,
                            const ContactParams& params, glm::vec3 gravity, float h, std::span<ContactDvRow> contact_dv) noexcept {
    const float e_world = params.restitution_e;
    const float mu_world = params.friction_mu;
    const float beta = params.baumgarte_beta;
    const float slop = params.slop;
    // The threshold's floor: a resting body's approach speed in one substep is |g| h, so a threshold below 2 |g| h would let it micro-bounce.
    const float settle_floor = 2.0f * glm::length(gravity) * h;

    for (std::size_t i = 0; i < bodies.size() && i < shapes.size(); ++i) {
        BodyState& body = bodies[i];
        const ShapeRow& shape = shapes[i];
        if ((body.flags & body_flags::active) == 0u || shape.count == 0u) continue;
        if (!(body.mass > 0.0f)) continue;  // no inverse mass to apply an impulse with

        const float e = shape.restitution >= 0.0f ? shape.restitution : e_world;
        const float mu = shape.friction >= 0.0f ? shape.friction : mu_world;
        const float rest_speed = glm::max(shape.rest_speed > 0.0f ? shape.rest_speed : 0.1f, settle_floor);
        const float radius = shape.radius;
        const float inv_m = 1.0f / body.mass;
        const glm::vec3 inv_i = body.inv_inertia_diag;
        const glm::vec3 vel_in = body.vel;
        const uint32_t sweeps = shape.iterations == 0u ? kDefaultSweeps : glm::min(shape.iterations, kMaxSweeps);

        // A. The contacts. STRICT `<`, NaN fails (as the sphere's); |grad| == 0 has no normal and is skipped.
        glm::vec3 c_n[kShapeMaxPoints];
        glm::vec3 c_r[kShapeMaxPoints];
        float c_kn[kShapeMaxPoints];
        float c_bias[kShapeMaxPoints];
        float c_acc_n[kShapeMaxPoints];
        glm::vec3 c_acc_t[kShapeMaxPoints];
        uint32_t found = 0;
        float deepest = 0.0f;
        glm::vec3 deepest_n(0.0f);
        const uint32_t count = glm::min(shape.count, kShapeMaxPoints);
        for (uint32_t k = 0; k < count; ++k) {
            const glm::vec3 local(shape.points[k][0], shape.points[k][1], shape.points[k][2]);
            const glm::vec3 p = contact_point_world(body, local);
            const SdfSample s = spade::sample(world_sdf, p);
            if (!(s.distance < radius)) continue;
            const float grad_len = glm::length(s.gradient);
            if (!(grad_len > 0.0f)) continue;
            const glm::vec3 n = s.gradient / grad_len;
            const float depth = radius - s.distance;
            const glm::vec3 lever = (p - n * radius) - body.pos;
            // The restitution bias: the separating speed the contact is solved to, set from the approach speed BEFORE any impulse of this
            // substep, and 0 (a plastic contact) under the rest speed.
            const glm::vec3 v_c = body.vel + glm::cross(body.orient * body.omega_body, lever);
            const float v_n0 = glm::dot(v_c, n);
            const glm::vec3 a = glm::conjugate(body.orient) * glm::cross(lever, n);
            c_n[found] = n;
            c_r[found] = lever;
            c_kn[found] = inv_m + glm::dot(a, a * inv_i);
            c_bias[found] = (-v_n0 > rest_speed) ? e * (-v_n0) : 0.0f;
            c_acc_n[found] = 0.0f;
            c_acc_t[found] = glm::vec3(0.0f);
            // The deepest point (strict `>`: the first in row order wins a tie) takes the positional correction.
            if (found == 0u || depth > deepest) {
                deepest = depth;
                deepest_n = n;
            }
            ++found;
        }
        if (found == 0u) continue;

        // B. The sweeps.
        for (uint32_t it = 0; it < sweeps; ++it) {
            for (uint32_t c = 0; c < found; ++c) {
                const glm::vec3 n = c_n[c];
                const glm::vec3 lever = c_r[c];
                // Normal: drive the point's normal speed to its bias; the accumulated impulse stays >= 0 (a contact pushes, never pulls).
                glm::vec3 v_c = body.vel + glm::cross(body.orient * body.omega_body, lever);
                const float v_n = glm::dot(v_c, n);
                const float wanted = c_acc_n[c] + (c_bias[c] - v_n) / c_kn[c];
                const float acc_n = glm::max(wanted, 0.0f);
                const float j_n = acc_n - c_acc_n[c];
                c_acc_n[c] = acc_n;
                body.vel += (j_n * inv_m) * n;
                body.omega_body += inv_i * (glm::conjugate(body.orient) * glm::cross(lever, j_n * n));

                // Friction on the velocity after that: one tangent (the sliding direction); the accumulated friction impulse stays in the
                // cone |j_t| <= mu acc_n, so it is an exact stop and never a reversal, and a later sweep can give back an over-shoot.
                v_c = body.vel + glm::cross(body.orient * body.omega_body, lever);
                const glm::vec3 v_t = v_c - glm::dot(v_c, n) * n;
                const float v_t_len = glm::length(v_t);
                if (v_t_len > 0.0f) {
                    const glm::vec3 t = v_t / v_t_len;
                    const glm::vec3 b = glm::conjugate(body.orient) * glm::cross(lever, t);
                    const float k_t = inv_m + glm::dot(b, b * inv_i);
                    glm::vec3 next = c_acc_t[c] - (v_t_len / k_t) * t;
                    const float cap = mu * acc_n;
                    const float next_len = glm::length(next);
                    if (next_len > cap) next = next * (cap / next_len);
                    const glm::vec3 dj = next - c_acc_t[c];
                    c_acc_t[c] = next;
                    body.vel += inv_m * dj;
                    body.omega_body += inv_i * (glm::conjugate(body.orient) * glm::cross(lever, dj));
                }
            }
        }

        // C. Positional correction of the deepest point, as the sphere's: beta of the excess over the slop, along its normal, on `pos`.
        body.pos += deepest_n * (beta * glm::max(deepest - slop, 0.0f));
        if (!contact_dv.empty() && i < contact_dv.size()) contact_dv[i].dv += body.vel - vel_in;
    }
}

}  // namespace spade::physics
