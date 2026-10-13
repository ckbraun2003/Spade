#include "world/sdf.hpp"

#include <bit>
#include <cassert>
#include <cmath>
#include <limits>
#include <string>

#include <glm/glm.hpp>

#include "core/fp32_math.hpp"
#include "core/validate.hpp"

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
            //
            // sin32, NOT std::sin (S5 Task 1). This is the engine's only
            // transcendental on a COLLISION path -- phi feeds contacts.cpp and
            // vehicles/rotor.cpp's ground effect, both of which land in
            // registered state and the golden digest -- so a libm here was the
            // same class of platform dependency fp32_math.hpp was created to
            // remove, and it survived the first pass only because nobody swept
            // for it. See that header's scope note.
            //
            // THE ARGUMENTS ARE UNBOUNDED, which is the one thing this call
            // site has to answer for: fx*p.x is a frequency times a world
            // coordinate, so a large enough world leaves sin32's documented
            // <= 1-ulp domain (|x| <= kMaxAccurateAngle == 100 rad). Precisely
            // what happens out there, in the two regimes it happens in, and
            // what each does to the Lipschitz claim above:
            //
            //   * 100 rad UP TO kMaxReducibleAngle (2^24 ~ 1.7e7): ACCURACY
            //     degrades, the BOUND does not, and the reason is that the loss
            //     is a PHASE SHIFT rather than a distortion. What stops being
            //     exact is the product n*(pi/2 part) in sin32's Cody-Waite
            //     reduction -- but n is CONSTANT across a quadrant, so within
            //     one quadrant the evaluated field is EXACTLY the analytic
            //     field with a constant phase offset, and a phase offset does
            //     not change |grad h| at all. a^2*max(fx^2, fz^2) still bounds
            //     it and phi is still 1-Lipschitz there. What is left is a STEP
            //     at quadrant boundaries, bounded by ulp(n*pi/2) ~ 6e-8*|fx*x|
            //     in phase, hence |a| * 6e-8 * |fx*x| in height: at
            //     |fx*x| = 1e3, a kilometre-scale world at unit frequency, that
            //     is 6e-5 of the amplitude -- three orders below
            //     kSdfGradientStep's own resolution of the surface.
            //   * AT OR BEYOND kMaxReducibleAngle the reduction is not
            //     attempted at all: sin32 returns 0, h collapses to `base`, and
            //     the terrain reads as FLAT. Bounded, defined, and trivially
            //     1-Lipschitz (zero gradient), at the price of a discontinuity
            //     at that surface. Not an accuracy claim and not offered as
            //     one -- fp32_math.cpp's guard note explains why no accurate
            //     answer is available at a magnitude where the argument's own
            //     ulp is 2 radians.
            //
            // DETERMINISM IS UNCONDITIONAL, and BECAUSE OF that guard rather
            // than in spite of it. Without it, sin32's reduction casts a float
            // to int32 out of range past |x| ~ 3.37e9 -- undefined behaviour
            // that resolves to INT_MIN on x86 and INT_MAX on ARM64, i.e.
            // exactly the platform-divergence class this engine's determinism
            // contract exists to exclude, sitting on exactly the path
            // (phi -> contacts, phi -> rotor ground effect) that reaches the
            // digest. Swapping std::sin for sin32 here is what put unbounded
            // arguments in front of that cast, so the guard landed with it.
            //
            // The bound therefore survives at every magnitude, and the honest
            // statement of the limit is "a heightfield sampled beyond
            // |f*coordinate| = 2^24 reads as flat" -- strictly weaker than the
            // "NOT an exact metric" caveat sdf.hpp already carries for this
            // primitive, and seven orders past any world Spade runs.
            const float amp = prm.x;
            const float fx = prm.y;
            const float fz = prm.z;
            const float base = prm.w;
            const float h = base + amp * math::sin32(fx * p.x) * math::sin32(fz * p.z);
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

// Mirrors combine_distance()/combine_gradient()'s own branch selection
// EXACTLY (S7a Task R8 fix round 1, world/sdf.hpp's own nearest_leaf_node()
// doc comment) -- rendering-only, never called by eval()/gradient()/sample()
// above. `leaf_a`/`leaf_b` are the postfix node indices already carried by
// the `a`/`b` operands; this picks the same one combine_distance's own
// comparison would keep.
[[nodiscard]] uint32_t combine_leaf(uint32_t op, const glm::vec4& prm, float a, float b, uint32_t leaf_a,
                                     uint32_t leaf_b) noexcept {
    switch (static_cast<SdfOp>(op)) {
        case SdfOp::none:
            return leaf_a;  // unreachable: primitives never reach here
        case SdfOp::union_:
            return a <= b ? leaf_a : leaf_b;
        case SdfOp::intersect:
            return a >= b ? leaf_a : leaf_b;
        case SdfOp::subtract:
            return a >= -b ? leaf_a : leaf_b;
        case SdfOp::smooth_union: {
            const float k = prm.x;
            if (!(k > 0.0f)) {
                return a <= b ? leaf_a : leaf_b;
            }
            const float h = smooth_union_weight(k, a, b);
            return h >= 0.5f ? leaf_a : leaf_b;
        }
    }
    return leaf_a;
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

// Rendering-only (sdf.hpp's own doc comment) -- a near-mirror of eval()'s own
// stack loop, but threading a NODE-INDEX tag through the stack instead of a
// gradient: a primitive leaf's tag is its own postfix index; an operator's
// tag is combine_leaf()'s selection, which mirrors combine_distance()'s
// comparison exactly so this can never disagree with which value eval()
// itself actually returned.
uint32_t nearest_leaf_node(const SdfProgram& program, glm::vec3 p) noexcept {
    float dist[kMaxSdfDepth];
    uint32_t leaf[kMaxSdfDepth];
    uint32_t sp = 0;

    for (size_t i = 0; i < program.nodes.size(); ++i) {
        const SdfNode& node = program.nodes[i];
        if (node.op == static_cast<uint32_t>(SdfOp::none)) {
            assert(sp < kMaxSdfDepth && "SdfProgram exceeds kMaxSdfDepth -- unvalidated program");
            assert(node.transform < program.transforms.size());
            const SdfTransform& t = program.transforms[node.transform];
            const glm::vec3 local(t.world_to_local * glm::vec4(p, 1.0f));
            dist[sp] = primitive_distance(node.kind, node.params, local) * t.scale;
            leaf[sp] = static_cast<uint32_t>(i);
            ++sp;
        } else {
            assert(sp >= 2 && "SdfProgram operator without two operands -- unvalidated program");
            const float b = dist[--sp];
            const uint32_t leaf_b = leaf[sp];
            const float a = dist[--sp];
            const uint32_t leaf_a = leaf[sp];
            dist[sp] = combine_distance(node.op, node.params, a, b);
            leaf[sp] = combine_leaf(node.op, node.params, a, b, leaf_a, leaf_b);
            ++sp;
        }
    }

    // sp == 0 only for an empty program (mirrors eval()'s own check).
    return sp == 1 ? leaf[0] : kNoSdfLeaf;
}

Result<uint32_t> SdfProgram::validate() const {
    // PA-2: node_materials is a parallel, host-only array (sdf.hpp) -- the
    // only thing THIS type can check about it is its length, since whether an
    // index is in range depends on WorldDesc::materials, which this type does
    // not have (validate_world_desc() checks that half).
    if (!node_materials.empty() && node_materials.size() != nodes.size()) {
        return std::unexpected(invalid(
            "SDF node_materials must be empty or exactly nodes.size() (" +
            std::to_string(nodes.size()) + "), found " + std::to_string(node_materials.size())));
    }

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
            // An operator node's `transform` is UNUSED: sample()/gradient()
            // above only ever index `transforms` inside the op==none branch.
            // Nothing enforced that until now, which is exactly how it can
            // carry silent garbage -- world_file.cpp's emit() writes it
            // verbatim (n.transform, no special-casing for an operator node),
            // so a hand-edited or corrupted file's out-of-range or nonzero
            // value here would round-trip and pass validation forever, and
            // config_hash (sim/world_set.cpp) folds every SdfNode's raw
            // bytes -- transform field included -- so two programs that are
            // otherwise IDENTICAL CSG trees could hash differently purely
            // from what garbage sits in a dead lane. WorldBuilder::push_op()
            // already always writes 0 here (builder.cpp), and every world in
            // the corpus already writes `transform: 0` on every op node (the
            // file format's own documented convention, world_file.cpp's
            // emission comment: "`transform` indexes `transforms` above; on
            // an `op` node it is unused") -- so this tightens validation
            // only, exactly mirroring the reserved-lane-must-be-zero checks
            // elsewhere in this codebase (ContactParams::_r0.._r2,
            // GridParams::_r0.._r2), and moves no corpus file.
            if (node.transform != 0) {
                return std::unexpected(
                    invalid("SDF operator node's transform index must be 0 (unused on an operator "
                            "node)" +
                            at_node(i)));
            }
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

// ---------------------------------------------------------------------------
// Bounds: sdf_bounded() and sdf_bounds() (sdf.hpp says what each promises).
// One postfix walk beside eval()'s stack machine serves both, in double.
// ---------------------------------------------------------------------------

namespace {

// One value on the bounds walk's stack: a box in program coordinates, which
// may be infinite (an unbounded operand) or inverted (lo > hi on an axis: an
// intersect whose operands do not overlap there), and the growth rate a field
// value maps to program coordinates by (sdf.hpp, GROWTH RATE).
//
// Inverted boxes stay in the arithmetic on purpose: what the walk keeps is
// "the points where the field is <= t lie in [lo - rate*t, hi + rate*t]" for
// every t >= 0, which union, intersect, subtract and the smooth union's growth
// all preserve, inverted or not. Only the final box, at t = 0, must not be.
struct WalkBox {
    glm::dvec3 lo{0.0};
    glm::dvec3 hi{0.0};
    double rate = 1.0;
};

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// The padding (sdf.hpp, PADDING): eval()'s fp32 rounding is a few ulps (2^-23
// each) of the magnitudes in play, so 2^-16 is a 2^7 margin.
constexpr double kPrimitivePad = 0x1p-16;
constexpr double kBlendPad = 0x1p-15;

constexpr size_t kNoNode = std::numeric_limits<size_t>::max();

[[nodiscard]] double largest_coordinate(const WalkBox& b) noexcept {
    const glm::dvec3 m = glm::max(glm::abs(b.lo), glm::abs(b.hi));
    return glm::max(m.x, glm::max(m.y, m.z));
}

// How far a bounded primitive's solid reaches along a direction, through its
// transform: the support h(row) = max over the local solid of dot(row, x), row
// being row i of A (below), which is the box's half extent on axis i. Each
// solid is a Minkowski sum of a few pieces whose supports add:
//   a ball of radius r:            r * |row|
//   a disc of radius r (local XZ): r * |(row.x, row.z)|
//   a segment +-h along local Y:   h * |row.y|
//   a box of half extents h:       sum_j h_j * |row_j|
// so a sphere is a ball, a box its corners, a cylinder a disc plus a segment,
// a capsule a segment plus a ball, and a torus its ring (a circle of radius R,
// whose support is the disc's) plus a ball. Exact for any invertible A: a
// turned sphere, or a cylinder turned about its own axis, keeps its box.
[[nodiscard]] double primitive_support(SdfPrim kind, const glm::vec4& prm, const glm::dvec3& row) noexcept {
    const glm::dvec4 q(prm);
    const double ball = std::sqrt(row.x * row.x + row.y * row.y + row.z * row.z);
    const double disc = std::sqrt(row.x * row.x + row.z * row.z);
    switch (kind) {
        case SdfPrim::sphere:
            return q.x * ball;
        case SdfPrim::box:
            return q.x * glm::abs(row.x) + q.y * glm::abs(row.y) + q.z * glm::abs(row.z);
        case SdfPrim::cylinder:
            return q.x * disc + q.y * glm::abs(row.y);
        case SdfPrim::capsule:
            return q.y * glm::abs(row.y) + q.x * ball;
        case SdfPrim::torus:
            return q.x * disc + q.y * ball;
        case SdfPrim::plane:
        case SdfPrim::heightfield:
            break;
    }
    return 0.0;
}

// A bounded primitive's box in program coordinates, or false for a singular
// transform. The node evaluates its primitive at local = W p + m (W the
// top-left 3x3 of world_to_local, m its translation), so its solid is the
// local one mapped by A = W^-1 and moved to c = -A m: the box about c whose
// half extent on axis i is the solid's support along row i of A. The growth
// rate is the largest |row i of A| / scale, and the padding scales with
// |A| |W| / 3, the condition number's stand-in: both are 1 for a rigid +
// uniform-scale transform, whose A is scale times a rotation.
[[nodiscard]] bool primitive_box(const SdfNode& node, const SdfTransform& t, WalkBox& out) noexcept {
    const glm::dmat4 m(t.world_to_local);
    const glm::dmat3 w(m);
    const double det = glm::determinant(w);
    if (!(det != 0.0) || !std::isfinite(det)) return false;
    const glm::dmat3 a = glm::inverse(w);
    const glm::dvec3 centre = -(a * glm::dvec3(m[3]));
    const auto kind = static_cast<SdfPrim>(node.kind);

    glm::dvec3 half(0.0);
    double rate = 0.0;
    double a_norm2 = 0.0;
    double w_norm2 = 0.0;
    for (int i = 0; i < 3; ++i) {  // glm is column-major: a[j][i] is row i, column j
        const glm::dvec3 row(a[0][i], a[1][i], a[2][i]);
        half[i] = primitive_support(kind, node.params, row);
        const double row2 = glm::dot(row, row);
        a_norm2 += row2;
        w_norm2 += w[0][i] * w[0][i] + w[1][i] * w[1][i] + w[2][i] * w[2][i];
        rate = glm::max(rate, std::sqrt(row2));
    }
    rate /= static_cast<double>(t.scale);
    const double condition = std::sqrt(a_norm2 * w_norm2) / 3.0;

    out.lo = centre - half;
    out.hi = centre + half;
    const double pad = kPrimitivePad * glm::max(condition, 1.0) * largest_coordinate(out);
    out.lo -= pad;
    out.hi += pad;
    out.rate = rate;
    return std::isfinite(out.lo.x + out.lo.y + out.lo.z + out.hi.x + out.hi.y + out.hi.z);
}

struct BoundsWalk {
    WalkBox box{};
    bool bounded = false;          // a finite box: the structural rule (sdf.hpp, sdf_bounded)
    size_t singular = kNoNode;     // the first primitive whose transform has no inverse
};

// The walk. `boxes` false is sdf_bounded(): a bounded primitive pushes a
// finite placeholder and no number is read, so `bounded` is the structural
// rule alone. A plane or a heightfield pushes all of space either way.
// PRECONDITION: program.validate() passed.
[[nodiscard]] BoundsWalk walk_bounds(const SdfProgram& program, bool boxes) noexcept {
    WalkBox stack[kMaxSdfDepth];
    uint32_t sp = 0;
    BoundsWalk result;

    for (size_t i = 0; i < program.nodes.size(); ++i) {
        const SdfNode& node = program.nodes[i];
        if (node.op == static_cast<uint32_t>(SdfOp::none)) {
            assert(sp < kMaxSdfDepth && "SdfProgram exceeds kMaxSdfDepth -- unvalidated program");
            const auto kind = static_cast<SdfPrim>(node.kind);
            WalkBox& b = stack[sp++];
            b = WalkBox{};
            if (kind == SdfPrim::plane || kind == SdfPrim::heightfield) {
                b.lo = glm::dvec3(-kInfinity);
                b.hi = glm::dvec3(kInfinity);
            } else if (boxes && !primitive_box(node, program.transforms[node.transform], b)) {
                b = WalkBox{};
                if (result.singular == kNoNode) result.singular = i;
            }
            continue;
        }

        assert(sp >= 2 && "SDF operator without two operands -- unvalidated program");
        const WalkBox b = stack[--sp];
        const WalkBox a = stack[--sp];
        WalkBox& out = stack[sp++];
        out.rate = glm::max(a.rate, b.rate);
        switch (static_cast<SdfOp>(node.op)) {
            case SdfOp::intersect:
                out.lo = glm::max(a.lo, b.lo);
                out.hi = glm::min(a.hi, b.hi);
                break;
            case SdfOp::subtract:  // "a minus b" lies inside a
                out = a;
                break;
            case SdfOp::union_:
            case SdfOp::smooth_union:
            case SdfOp::none:
                out.lo = glm::min(a.lo, b.lo);
                out.hi = glm::max(a.hi, b.hi);
                break;
        }
        const double k = node.params.x;
        if (boxes && node.op == static_cast<uint32_t>(SdfOp::smooth_union) && k > 0.0) {
            // min(a,b) - k/4 <= d: the blend reaches at most k/4 (in field
            // units) past the union, plus eval()'s rounding of its arithmetic.
            const double grow = 0.25 * k * out.rate + kBlendPad * largest_coordinate(out) + kPrimitivePad * k;
            out.lo -= grow;
            out.hi += grow;
        }
    }

    if (sp == 1) {
        result.box = stack[0];
        const glm::dvec3 lo = result.box.lo, hi = result.box.hi;
        result.bounded = std::isfinite(lo.x) && std::isfinite(lo.y) && std::isfinite(lo.z) &&
                         std::isfinite(hi.x) && std::isfinite(hi.y) && std::isfinite(hi.z);
    }
    return result;
}

// The fp32 at or below x (at or above, for round_up), for |x| <= FLT_MAX.
[[nodiscard]] float round_down(double x) noexcept {
    float f = static_cast<float>(x);
    if (static_cast<double>(f) > x) {
        const uint32_t bits = std::bit_cast<uint32_t>(f);
        f = f == 0.0f ? -std::numeric_limits<float>::denorm_min()
                      : std::bit_cast<float>(f > 0.0f ? bits - 1u : bits + 1u);
    }
    return f;
}

[[nodiscard]] float round_up(double x) noexcept {
    return -round_down(-x);
}

}  // namespace

bool sdf_bounded(const SdfProgram& program) noexcept {
    return walk_bounds(program, false).bounded;
}

Result<Aabb> sdf_bounds(const SdfProgram& program) {
    if (const Result<uint32_t> valid = program.validate(); !valid) {
        return std::unexpected(valid.error());
    }
    if (program.nodes.empty()) {
        return std::unexpected(invalid("the SDF program is empty, so its solid is empty"));
    }
    const BoundsWalk walk = walk_bounds(program, true);
    if (!walk.bounded) {
        return std::unexpected(invalid("the solid is unbounded: a plane or a heightfield fills half of space, so "
                                       "bound it inside an intersect or cut it from a bounded shape"));
    }
    if (walk.singular != kNoNode) {
        return std::unexpected(invalid("SDF transform is singular, so its primitive's solid has no finite bounds" +
                                       at_node(walk.singular)));
    }
    const glm::dvec3 lo = walk.box.lo, hi = walk.box.hi;
    if (lo.x > hi.x || lo.y > hi.y || lo.z > hi.z) {
        return std::unexpected(invalid("the solid is empty: an intersect's operands have bounds that do not overlap"));
    }
    constexpr double kFloatMax = std::numeric_limits<float>::max();
    if (glm::max(glm::abs(lo.x), glm::max(glm::abs(lo.y), glm::abs(lo.z))) > kFloatMax ||
        glm::max(glm::abs(hi.x), glm::max(glm::abs(hi.y), glm::abs(hi.z))) > kFloatMax) {
        return std::unexpected(invalid("the solid's bounds exceed fp32's range"));
    }
    return Aabb{glm::vec3(round_down(lo.x), round_down(lo.y), round_down(lo.z)),
                glm::vec3(round_up(hi.x), round_up(hi.y), round_up(hi.z))};
}

}  // namespace spade
