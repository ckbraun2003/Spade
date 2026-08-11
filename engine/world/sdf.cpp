#include "world/sdf.hpp"

#include <cassert>
#include <cmath>
#include <string>

#include <glm/glm.hpp>

namespace spade {
namespace {

// ---------------------------------------------------------------------------
// Primitive distances -- all in NODE-LOCAL space, all fp32, all exact
// Euclidean distances except the heightfield (documented bound in sdf.hpp).
//
// The forms below are the standard closed forms (Quilez's catalogue); they are
// written branch-free where a branch-free form exists so the same source
// shape ports to the GPU kernel at S6.
// ---------------------------------------------------------------------------

[[nodiscard]] float primitive_distance(uint32_t kind, const glm::vec4& prm, glm::vec3 p) noexcept {
    switch (static_cast<SdfPrim>(kind)) {
        case SdfPrim::plane:
            // Half-space { dot(p, n) <= offset }, n unit (enforced by validate).
            return glm::dot(p, glm::vec3(prm)) - prm.w;

        case SdfPrim::sphere:
            return glm::length(p) - prm.x;

        case SdfPrim::box: {
            // q = |p| - half_extents, folded into the positive octant.
            //   exterior: length(max(q,0))          -- distance to the nearest face/edge/corner
            //   interior: min(max(q.x,q.y,q.z), 0)  -- negative distance to the nearest face
            // Exactly one of the two terms is non-zero, so summing them is the
            // whole signed distance and corner cases (edges, corners) fall out
            // of length(max(q,0)) without any special casing.
            const glm::vec3 q = glm::abs(p) - glm::vec3(prm);
            return glm::length(glm::max(q, 0.0f)) +
                   glm::min(glm::max(q.x, glm::max(q.y, q.z)), 0.0f);
        }

        case SdfPrim::cylinder: {
            // Capped cylinder about local +Y, reduced to a 2D box in
            // (radial, axial) coordinates -- same interior/exterior split as box.
            const glm::vec2 d{glm::length(glm::vec2(p.x, p.z)) - prm.x, glm::abs(p.y) - prm.y};
            return glm::min(glm::max(d.x, d.y), 0.0f) + glm::length(glm::max(d, 0.0f));
        }

        case SdfPrim::capsule: {
            // Segment (0, -h, 0)..(0, +h, 0) inflated by radius: project onto
            // the segment (the clamp), then it is a sphere distance.
            glm::vec3 q = p;
            q.y -= glm::clamp(q.y, -prm.y, prm.y);
            return glm::length(q) - prm.x;
        }

        case SdfPrim::torus: {
            // Ring of radius R in local XZ (hole axis +Y), tube radius r:
            // distance to the ring's centre circle, minus r.
            const glm::vec2 q{glm::length(glm::vec2(p.x, p.z)) - prm.x, p.y};
            return glm::length(q) - prm.y;
        }

        case SdfPrim::heightfield: {
            // phi = (p.y - h(x,z)) / L, L = exact Lipschitz bound of the
            // numerator. Derivation (sdf.hpp carries the statement):
            //   h  = base + a*sin(fx*x)*sin(fz*z)
            //   hx = a*fx*cos(fx*x)*sin(fz*z), hz = a*fz*sin(fx*x)*cos(fz*z)
            //   max(hx^2 + hz^2) is bilinear in (sin^2(fx*x), sin^2(fz*z)), so
            //   its maximum sits at a corner: a^2 * max(fx^2, fz^2).
            const float amp = prm.x;
            const float fx = prm.y;
            const float fz = prm.z;
            const float base = prm.w;
            const float h = base + amp * std::sin(fx * p.x) * std::sin(fz * p.z);
            const float slope = glm::abs(amp) * glm::max(glm::abs(fx), glm::abs(fz));
            return (p.y - h) / std::sqrt(1.0f + slope * slope);
        }
    }
    // Unreachable for a validated program (validate() rejects unknown kinds).
    return kSdfEmptyDistance;
}

// Sign with a deterministic tie-break at zero (+1), matching what a central
// difference converges to on a box face through the origin. glm::sign() would
// return 0 there and hand back a short gradient.
[[nodiscard]] constexpr float sgn(float x) noexcept { return x < 0.0f ? -1.0f : 1.0f; }

// Central differences on a single primitive, in local space. Pinned stencil:
// step kSdfGradientStep, axes in x, y, z order, scaled by (0.5f / h).
[[nodiscard]] glm::vec3 central_difference(uint32_t kind, const glm::vec4& prm,
                                           glm::vec3 p) noexcept {
    constexpr float h = kSdfGradientStep;
    constexpr float inv_2h = 0.5f / h;
    const float dx = primitive_distance(kind, prm, glm::vec3(p.x + h, p.y, p.z)) -
                     primitive_distance(kind, prm, glm::vec3(p.x - h, p.y, p.z));
    const float dy = primitive_distance(kind, prm, glm::vec3(p.x, p.y + h, p.z)) -
                     primitive_distance(kind, prm, glm::vec3(p.x, p.y - h, p.z));
    const float dz = primitive_distance(kind, prm, glm::vec3(p.x, p.y, p.z + h)) -
                     primitive_distance(kind, prm, glm::vec3(p.x, p.y, p.z - h));
    return glm::vec3(dx * inv_2h, dy * inv_2h, dz * inv_2h);
}

// Local-space gradient: analytic for plane/sphere/box, pinned central
// differences for the rest (cylinder/capsule/torus have closed forms too, but
// the brief pins the stencil for them so CPU and GPU cannot drift apart on a
// hand-derived formula; the heightfield's exact gradient is cheap but its
// distance is already a bound, so nothing is gained by special-casing it).
[[nodiscard]] glm::vec3 primitive_gradient(uint32_t kind, const glm::vec4& prm,
                                           glm::vec3 p) noexcept {
    switch (static_cast<SdfPrim>(kind)) {
        case SdfPrim::plane:
            return glm::vec3(prm);

        case SdfPrim::sphere: {
            const float len = glm::length(p);
            // Exactly at the centre the gradient is undefined (every direction
            // is a subgradient). Report zero rather than an arbitrary axis --
            // a central difference degenerates to zero there too.
            return len > 0.0f ? p / len : glm::vec3(0.0f);
        }

        case SdfPrim::box: {
            const glm::vec3 q = glm::abs(p) - glm::vec3(prm);
            const float qmax = glm::max(q.x, glm::max(q.y, q.z));
            if (qmax > 0.0f) {
                // Exterior: direction to the nearest surface point, unfolded
                // out of the positive octant. Components with q_i <= 0 do not
                // contribute, so their sign never matters.
                const glm::vec3 e = glm::max(q, 0.0f);
                const float len = glm::length(e);
                if (len <= 0.0f) {
                    return glm::vec3(0.0f);  // unreachable: qmax > 0 implies len > 0
                }
                return glm::vec3(sgn(p.x), sgn(p.y), sgn(p.z)) * (e / len);
            }
            // Interior: the field is max(q.x, q.y, q.z), so the gradient is the
            // outward axis of the deepest component. Ties break x, then y, then
            // z -- deterministic, and the same order the GPU kernel must use.
            if (q.x >= q.y && q.x >= q.z) {
                return glm::vec3(sgn(p.x), 0.0f, 0.0f);
            }
            if (q.y >= q.z) {
                return glm::vec3(0.0f, sgn(p.y), 0.0f);
            }
            return glm::vec3(0.0f, 0.0f, sgn(p.z));
        }

        case SdfPrim::cylinder:
        case SdfPrim::capsule:
        case SdfPrim::torus:
        case SdfPrim::heightfield:
            return central_difference(kind, prm, p);
    }
    return glm::vec3(0.0f);
}

// ---------------------------------------------------------------------------
// CSG combination
//
// Postfix operand order is "a b op": `a` is the deeper stack entry (pushed
// first), `b` the shallower one. Subtraction is therefore a minus b.
//
// smooth_union (polynomial / quadratic smooth minimum, Quilez's form) -- the
// exact expressions, pinned for the GPU port:
//     h = clamp(0.5 + 0.5*(b - a)/k, 0, 1)
//     d = (b + (a - b)*h) - k*h*(1 - h)
// Its partials are exactly h and (1 - h): differentiating d with respect to a
// gives h + dh/da * [(a - b) - k*(1 - 2h)], and the unclamped h satisfies
// (a - b) = k*(1 - 2h), so the bracket vanishes. When h clamps, dh/da is zero
// and the same result holds. Hence grad = gb + (ga - gb)*h, a convex blend, and
// min(a,b) - k/4 <= d <= min(a,b).
// ---------------------------------------------------------------------------

[[nodiscard]] float smooth_union_weight(float k, float a, float b) noexcept {
    return glm::clamp(0.5f + 0.5f * (b - a) / k, 0.0f, 1.0f);
}

[[nodiscard]] float combine_distance(uint32_t op, const glm::vec4& prm, float a, float b) noexcept {
    switch (static_cast<SdfOp>(op)) {
        case SdfOp::none:
            return a;  // unreachable: primitives never reach here
        case SdfOp::union_:
            return a <= b ? a : b;
        case SdfOp::intersect:
            return a >= b ? a : b;
        case SdfOp::subtract:
            return a >= -b ? a : -b;
        case SdfOp::smooth_union: {
            const float k = prm.x;
            if (!(k > 0.0f)) {
                return a <= b ? a : b;  // degenerate blend radius == plain union
            }
            const float h = smooth_union_weight(k, a, b);
            return (b + (a - b) * h) - k * h * (1.0f - h);
        }
    }
    return a;
}

// Gradient of the combination. The distance itself is delegated to
// combine_distance() so sample() and eval() cannot drift apart by even an ulp.
[[nodiscard]] glm::vec3 combine_gradient(uint32_t op, const glm::vec4& prm, float a, float b,
                                         glm::vec3 ga, glm::vec3 gb) noexcept {
    switch (static_cast<SdfOp>(op)) {
        case SdfOp::none:
            return ga;  // unreachable
        case SdfOp::union_:
            return a <= b ? ga : gb;
        case SdfOp::intersect:
            return a >= b ? ga : gb;
        case SdfOp::subtract:
            // d = max(a, -b): the subtracted branch's field is negated, so its
            // gradient flips with it.
            return a >= -b ? ga : -gb;
        case SdfOp::smooth_union: {
            const float k = prm.x;
            if (!(k > 0.0f)) {
                return a <= b ? ga : gb;
            }
            const float h = smooth_union_weight(k, a, b);
            return gb + (ga - gb) * h;
        }
    }
    return ga;
}

// ---------------------------------------------------------------------------
// Validation helpers
// ---------------------------------------------------------------------------

[[nodiscard]] bool finite(float v) noexcept { return std::isfinite(v); }

[[nodiscard]] bool finite(const glm::vec4& v) noexcept {
    return finite(v.x) && finite(v.y) && finite(v.z) && finite(v.w);
}

[[nodiscard]] bool finite(const glm::mat4& m) noexcept {
    return finite(m[0]) && finite(m[1]) && finite(m[2]) && finite(m[3]);
}

// Dimension sanity per kind. Returns nullptr when the parameters are usable,
// otherwise the reason. Degenerate-but-meaningful values (a zero-radius sphere
// is a point, a zero blend radius is a plain union) stay legal; values with no
// meaning at all (a negative radius) do not.
[[nodiscard]] const char* check_primitive_params(SdfPrim kind, const glm::vec4& prm) noexcept {
    switch (kind) {
        case SdfPrim::plane: {
            const glm::vec3 n(prm);
            const float n2 = glm::dot(n, n);
            // Metricity depends on a unit normal; the builder normalizes, so
            // only hand-built programs can fail this.
            return glm::abs(n2 - 1.0f) <= 1e-3f ? nullptr : "plane normal must be unit length";
        }
        case SdfPrim::sphere:
            return prm.x >= 0.0f ? nullptr : "sphere radius must be >= 0";
        case SdfPrim::box:
            return (prm.x >= 0.0f && prm.y >= 0.0f && prm.z >= 0.0f)
                       ? nullptr
                       : "box half extents must be >= 0";
        case SdfPrim::cylinder:
            return (prm.x >= 0.0f && prm.y >= 0.0f)
                       ? nullptr
                       : "cylinder radius and half height must be >= 0";
        case SdfPrim::capsule:
            return (prm.x >= 0.0f && prm.y >= 0.0f)
                       ? nullptr
                       : "capsule radius and half height must be >= 0";
        case SdfPrim::torus:
            return (prm.x >= 0.0f && prm.y >= 0.0f) ? nullptr : "torus radii must be >= 0";
        case SdfPrim::heightfield:
            // amplitude, frequencies and base height may take any finite value.
            return nullptr;
    }
    return "unknown SDF primitive kind";
}

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

[[nodiscard]] std::string at_node(size_t index) {
    return " (node " + std::to_string(index) + ")";
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

float eval(const SdfProgram& program, glm::vec3 p) noexcept {
    float stack[kMaxSdfDepth];
    uint32_t sp = 0;

    for (const SdfNode& node : program.nodes) {
        if (node.op == static_cast<uint32_t>(SdfOp::none)) {
            assert(sp < kMaxSdfDepth && "SdfProgram exceeds kMaxSdfDepth -- unvalidated program");
            assert(node.transform < program.transforms.size());
            const SdfTransform& t = program.transforms[node.transform];
            const glm::vec3 local(t.world_to_local * glm::vec4(p, 1.0f));
            stack[sp++] = primitive_distance(node.kind, node.params, local) * t.scale;
        } else {
            assert(sp >= 2 && "SdfProgram operator without two operands -- unvalidated program");
            const float b = stack[--sp];
            const float a = stack[--sp];
            stack[sp++] = combine_distance(node.op, node.params, a, b);
        }
    }

    // sp == 0 only for an empty program; a validated program always ends at 1.
    return sp == 1 ? stack[0] : kSdfEmptyDistance;
}

SdfSample sample(const SdfProgram& program, glm::vec3 p) noexcept {
    float dist[kMaxSdfDepth];
    glm::vec3 grad[kMaxSdfDepth];
    uint32_t sp = 0;

    for (const SdfNode& node : program.nodes) {
        if (node.op == static_cast<uint32_t>(SdfOp::none)) {
            assert(sp < kMaxSdfDepth && "SdfProgram exceeds kMaxSdfDepth -- unvalidated program");
            assert(node.transform < program.transforms.size());
            const SdfTransform& t = program.transforms[node.transform];
            const glm::vec3 local(t.world_to_local * glm::vec4(p, 1.0f));
            dist[sp] = primitive_distance(node.kind, node.params, local) * t.scale;
            // Chain rule through the inverse transform. For rigid + uniform
            // scale this reduces to the rotation, so |grad| is preserved.
            grad[sp] = (glm::transpose(glm::mat3(t.world_to_local)) *
                        primitive_gradient(node.kind, node.params, local)) *
                       t.scale;
            ++sp;
        } else {
            assert(sp >= 2 && "SdfProgram operator without two operands -- unvalidated program");
            const float b = dist[--sp];
            const glm::vec3 gb = grad[sp];
            const float a = dist[--sp];
            const glm::vec3 ga = grad[sp];
            dist[sp] = combine_distance(node.op, node.params, a, b);
            grad[sp] = combine_gradient(node.op, node.params, a, b, ga, gb);
            ++sp;
        }
    }

    if (sp != 1) {
        return SdfSample{kSdfEmptyDistance, glm::vec3(0.0f)};
    }
    return SdfSample{dist[0], grad[0]};
}

glm::vec3 gradient(const SdfProgram& program, glm::vec3 p) noexcept {
    return sample(program, p).gradient;
}

Result<uint32_t> SdfProgram::validate() const {
    for (size_t i = 0; i < transforms.size(); ++i) {
        const SdfTransform& t = transforms[i];
        if (!finite(t.world_to_local) || !finite(t.scale)) {
            return std::unexpected(
                invalid("SDF transform has non-finite entries (transform " + std::to_string(i) + ")"));
        }
        if (!(t.scale > 0.0f)) {
            return std::unexpected(
                invalid("SDF transform scale must be > 0 (transform " + std::to_string(i) + ")"));
        }
    }

    uint32_t depth = 0;
    uint32_t peak = 0;

    for (size_t i = 0; i < nodes.size(); ++i) {
        const SdfNode& node = nodes[i];

        if (node.op >= kSdfOpCount) {
            return std::unexpected(invalid("unknown SDF op" + at_node(i)));
        }
        if (!finite(node.params)) {
            return std::unexpected(invalid("SDF node has non-finite parameters" + at_node(i)));
        }

        if (node.op == static_cast<uint32_t>(SdfOp::none)) {
            if (node.kind >= kSdfPrimCount) {
                return std::unexpected(invalid("unknown SDF primitive kind" + at_node(i)));
            }
            if (node.transform >= transforms.size()) {
                return std::unexpected(invalid("SDF transform index out of range" + at_node(i)));
            }
            if (const char* why = check_primitive_params(static_cast<SdfPrim>(node.kind), node.params)) {
                return std::unexpected(invalid(std::string(why) + at_node(i)));
            }
            ++depth;
            if (depth > kMaxSdfDepth) {
                // A fixed capacity (eval's stack) exceeded -- not a malformed
                // program, so this is capacity_exceeded, not invalid_argument.
                return std::unexpected(Error{
                    Code::capacity_exceeded,
                    "SDF program needs more than " + std::to_string(kMaxSdfDepth) +
                        " evaluation stack slots" + at_node(i)});
            }
            peak = depth > peak ? depth : peak;
        } else {
            if (node.op == static_cast<uint32_t>(SdfOp::smooth_union) && node.params.x < 0.0f) {
                return std::unexpected(invalid("smooth_union blend radius must be >= 0" + at_node(i)));
            }
            if (depth < 2) {
                return std::unexpected(
                    invalid("SDF operator needs two operands on the stack" + at_node(i)));
            }
            --depth;
        }
    }

    if (!nodes.empty() && depth != 1) {
        return std::unexpected(invalid("SDF program must reduce to exactly one value, left " +
                                       std::to_string(depth)));
    }
    return peak;
}

}  // namespace spade
