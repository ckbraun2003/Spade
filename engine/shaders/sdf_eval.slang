// ---------------------------------------------------------------------------
// sdf_eval.slang (S6 Task 6) -- the Slang PORT of engine/world/sdf.cpp's
// flat-postfix SDF evaluator: `sdf_sample(world, p)` returns the same distance
// and the same gradient the CPU twin's spade::sample() returns, walking the
// same program with the same op order.
//
// A MODULE, NOT A KERNEL. It declares no entry point; collision_static.slang
// imports it. It reaches SPIR-V only through that kernel, which is what puts
// its arithmetic under the SPIR-V float-controls gate on the real compiled
// artifact (the same arrangement fp32_math.slang has through
// fp32_math_probe.slang).
//
// ===========================================================================
// WHAT "SAME OP ORDER" MEANS HERE, CONCRETELY
//
// world/sdf.cpp's header states the parity contract: "fp32 everywhere ... the
// GPU port must match these exactly". The CPU twin is written in glm, and glm's
// vector helpers are not free-form -- each has ONE pinned expression, and fp32
// addition and multiplication are not associative, so the port has to reproduce
// the expression rather than the mathematical function. Every glm call this
// file replaces is therefore spelled out below, with the glm source it mirrors
// named (glm 1.0.1, vendored):
//
//   glm::dot(a, b)      vec3: `tmp.x + tmp.y + tmp.z`, i.e. ((ax*bx + ay*by) +
//                       az*bz)  -- detail/func_geometric.inl compute_dot<vec3>
//   glm::length(v)      sqrt(dot(v, v)) -- same file
//   glm::min(x, y)      (y < x) ? y : x   -- detail/func_common.inl
//   glm::max(x, y)      (x < y) ? y : x   -- same file
//   glm::clamp(x, a, b) min(max(x, a), b) -- same file
//   glm::max(v, s)      componentwise, the scalar broadcast -- _vectorize.hpp
//   mat4 * vec4         ((c0*x + c1*y) + (c2*z + c3*w)), a PAIRWISE TREE, not
//                       a left fold -- detail/type_mat4x4.inl
//   mat3 * vec3         componentwise `m[0][i]*x + m[1][i]*y + m[2][i]*z`,
//                       LEFT-associated -- detail/type_mat3x3.inl
//
// The min/max spellings are ternaries rather than Slang's own `min`/`max`
// because the two differ on SIGNED ZERO: glm::max(-0.0f, +0.0f) evaluates
// `(-0.0f < +0.0f)` -> false -> returns -0.0f, while GLSL.std.450 FMax is free
// to return either. A ternary is the same instruction count and removes the
// question.
//
// ===========================================================================
// DIVISION AND SQRT: WHERE, AND WHY THEY ARE ALLOWED HERE
//
// Global constraint: the fp32_math module must contain NO OpFDiv and NO sqrt
// (they are <= 2.5 ulp on Vulkan, not correctly rounded), but "downstream
// kernels MAY use div/sqrt -- that is why CPU<->GPU is banded rather than
// bit-identical". This module is downstream. Every site is listed here, and
// every one of them exists because the CPU twin has the identical operation at
// the identical place -- none is an optimization and none may be respelled
// without breaking the op-order mirror:
//
//   SQRT  (5 sites) -- sphere/capsule/torus distances (glm::length), the box's
//         exterior branch (glm::length), the cylinder's exterior branch
//         (two glm::lengths), and the heightfield's Lipschitz divisor
//         (std::sqrt). sdf.hpp says so outright: "std::sqrt stays: IEEE
//         mandates it correctly rounded, so it is already bit-identical
//         everywhere" -- true of the CPU, and the reason the GPU's <= 2.5-ulp
//         sqrt is the FIRST band contributor in this file.
//   DIV   (4 sites) -- the sphere's `p / len`, the box exterior's `e / len`,
//         the heightfield's `(p.y - h) / sqrt(...)`, and smooth_union's
//         `0.5f * (b - a) / k`.
//
// TRANSCENDENTALS: exactly one, the heightfield's sine, and it calls the T4
// port sin32() -- NEVER the driver's sin. world/sdf.cpp's own call site carries
// the full argument (phi feeds contacts and therefore the determinism corpus).
// CHECKED BEFORE PORTING, since the brief asked: smooth_union in world/sdf.cpp
// uses NO exponential at all -- it is Quilez's POLYNOMIAL (quadratic) smooth
// minimum, `h = clamp(0.5 + 0.5*(b-a)/k, 0, 1); d = (b + (a-b)*h) - k*h*(1-h)`
// -- so there is no exp32 call to port here, only the division named above.
//
// ===========================================================================
// NO WORKGROUP-SHARED ANYTHING. The whole walk is per-thread, over per-thread
// arrays, with no cross-lane communication of any kind -- so Task 9's
// workgroup_size {32, 64, 128} bit-invariance holds here by construction
// rather than by measurement (kernel design constraint, carried from the
// pre-flight scan).
// ---------------------------------------------------------------------------

// `import layouts` AS WELL AS `import bindings`: Slang's `import` is NOT
// transitive -- bindings.slang imports layouts.slang for its own declarations,
// but that does not re-export the STRUCT NAMES to a module importing bindings.
// A module that only SUBSCRIPTS a buffer never needs them; one that names an
// element type (`const SdfNodeRow node = ...`) does. (The example this note
// used to give was pipeline_smoke.slang, the stub kernel S6 Task 8 deleted;
// every module in the tree now names element types, so the distinction is
// stated rather than illustrated.)
import layouts;
import bindings;
import fp32_math;
// The glm expression mirrors this module used to declare for itself
// (glm_min/glm_max/glm_clamp/glm_length2/glm_length3). Extracted to one shared
// module by S6 Task 8 -- see mirrors.slang's header for the review finding (T7
// minor M3) that forced it: five hand-written copies of a left-associated dot
// product are five chances to associate one of them right.
import mirrors;

// ---------------------------------------------------------------------------
// Pinned constants, mirroring world/sdf.hpp's. Each is the SAME fp32 value the
// CPU sees, not merely the same decimal.
// ---------------------------------------------------------------------------

// world/sdf.hpp: kMaxSdfDepth. The evaluation stack's fixed capacity. Two
// per-thread arrays of this length (distance + gradient) are the whole of this
// module's private storage.
static const uint kMaxSdfDepth = 32;

// world/sdf.hpp: kSdfGradientStep. The pinned central-difference step. Written
// as the same literal the CPU uses, which rounds to 0x3A83126F.
static const float kSdfGradientStep = 1e-3f;

// world/sdf.cpp's central_difference(): `constexpr float inv_2h = 0.5f / h;`.
// Written here as the LITERAL RESULT of that fp32 division rather than as the
// division itself, so no assumption about Slang's constant folding can move it:
// 0.5f / 1e-3f == 499.9999694824219 == 0x43F9FFFF (computed on this box, this
// task's report has the transcript). Nine significant digits, per the house
// fp32 text rule -- 499.999969 round-trips to that exact bit pattern.
static const float kSdfInvTwoStep = 499.999969f;

// world/sdf.hpp: kSdfEmptyDistance == std::numeric_limits<float>::max().
static const float kSdfEmptyDistance = 3.40282347e+38f;

// world/sdf.hpp's SdfPrim / SdfOp, as their underlying uint32 values. Spelled
// here rather than imported because they are ENUM VALUES, not layouts --
// layouts.slang mirrors byte images, and a constant is not one. Every kernel
// that needs a sentinel spells it locally for the same reason.
static const uint kPrimPlane = 0;
static const uint kPrimSphere = 1;
static const uint kPrimBox = 2;
static const uint kPrimCylinder = 3;
static const uint kPrimCapsule = 4;
static const uint kPrimTorus = 5;
static const uint kPrimHeightfield = 6;

static const uint kOpNone = 0;
static const uint kOpUnion = 1;
static const uint kOpIntersect = 2;
static const uint kOpSubtract = 3;
static const uint kOpSmoothUnion = 4;

// ---------------------------------------------------------------------------
// glm_min / glm_max / glm_clamp / glm_length2 / glm_length3 come from
// mirrors.slang (imported above), which carries the glm source citation and the
// signed-zero argument for each. This module declared its own copies through S6
// Task 7.
//
// world/sdf.cpp's sgn() stays HERE, and the difference is worth a line: it is
// not a glm expression at all but a local of world/sdf.cpp with a deliberate
// deviation from glm::sign -- +1 at exactly zero, because a central difference
// on a box face through the origin converges to +1 and glm::sign() would answer
// 0 and hand back a short gradient. A shared module of glm mirrors is the wrong
// home for a function whose whole point is not being one.
// ---------------------------------------------------------------------------

// world/sdf.cpp's sgn(): +1 at exactly zero, deliberately (a central difference
// on a box face through the origin converges to +1; glm::sign() would answer 0
// and hand back a short gradient).
float sdf_sgn(float x) { return x < 0.0f ? -1.0f : 1.0f; }

// ---------------------------------------------------------------------------
// primitive_distance -- world/sdf.cpp lines 24-139, kind by kind, in local
// space. Line citations name the CPU expression each branch mirrors.
// ---------------------------------------------------------------------------
float primitive_distance(uint kind, float4 prm, float3 p) {
    if (kind == kPrimPlane) {
        // sdf.cpp:28 -- `glm::dot(p, glm::vec3(prm)) - prm.w`.
        // dot spelled out: OpDot is forbidden outright (spirv_scan.hpp P2).
        return ((p.x * prm.x + p.y * prm.y) + p.z * prm.z) - prm.w;
    }
    if (kind == kPrimSphere) {
        // sdf.cpp:31 -- `glm::length(p) - prm.x`.
        return glm_length3(p) - prm.x;
    }
    if (kind == kPrimBox) {
        // sdf.cpp:40-42 -- q = |p| - half_extents;
        //   length(max(q, 0)) + min(max(q.x, max(q.y, q.z)), 0)
        const float3 q = abs(p) - float3(prm.x, prm.y, prm.z);
        const float3 e = float3(glm_max(q.x, 0.0f), glm_max(q.y, 0.0f), glm_max(q.z, 0.0f));
        return glm_length3(e) + glm_min(glm_max(q.x, glm_max(q.y, q.z)), 0.0f);
    }
    if (kind == kPrimCylinder) {
        // sdf.cpp:48-49 -- d = (length(p.xz) - r, |p.y| - h);
        //   min(max(d.x, d.y), 0) + length(max(d, 0))
        const float2 d = float2(glm_length2(float2(p.x, p.z)) - prm.x, abs(p.y) - prm.y);
        const float2 e = float2(glm_max(d.x, 0.0f), glm_max(d.y, 0.0f));
        return glm_min(glm_max(d.x, d.y), 0.0f) + glm_length2(e);
    }
    if (kind == kPrimCapsule) {
        // sdf.cpp:55-57 -- q = p; q.y -= clamp(q.y, -h, h); length(q) - r
        float3 q = p;
        q.y = q.y - glm_clamp(q.y, -prm.y, prm.y);
        return glm_length3(q) - prm.x;
    }
    if (kind == kPrimTorus) {
        // sdf.cpp:63-64 -- q = (length(p.xz) - R, p.y); length(q) - r
        const float2 q = float2(glm_length2(float2(p.x, p.z)) - prm.x, p.y);
        return glm_length2(q) - prm.y;
    }
    if (kind == kPrimHeightfield) {
        // sdf.cpp:128-134, the ONE transcendental on a collision path:
        //   h     = base + amp * sin32(fx*x) * sin32(fz*z)
        //   slope = |amp| * max(|fx|, |fz|)
        //   phi   = (p.y - h) / sqrt(1 + slope*slope)
        // sin32, never the driver's sin -- the GPU corollary of the
        // bit-portability rule. The C++ multiply associates left, so
        // `amp * s1 * s2` is `(amp * s1) * s2`.
        const float amp = prm.x;
        const float fx = prm.y;
        const float fz = prm.z;
        const float base = prm.w;
        const float h = base + (amp * sin32(fx * p.x)) * sin32(fz * p.z);
        const float slope = abs(amp) * glm_max(abs(fx), abs(fz));
        return (p.y - h) / sqrt(1.0f + slope * slope);
    }
    // Unreachable for a validated program (sdf.cpp:138).
    return kSdfEmptyDistance;
}

// ---------------------------------------------------------------------------
// central_difference -- world/sdf.cpp lines 148-159. Pinned stencil: step
// kSdfGradientStep, axes evaluated in x, y, z order, scaled by (0.5f / h).
// ---------------------------------------------------------------------------
float3 central_difference(uint kind, float4 prm, float3 p) {
    const float h = kSdfGradientStep;
    const float dx = primitive_distance(kind, prm, float3(p.x + h, p.y, p.z)) -
                     primitive_distance(kind, prm, float3(p.x - h, p.y, p.z));
    const float dy = primitive_distance(kind, prm, float3(p.x, p.y + h, p.z)) -
                     primitive_distance(kind, prm, float3(p.x, p.y - h, p.z));
    const float dz = primitive_distance(kind, prm, float3(p.x, p.y, p.z + h)) -
                     primitive_distance(kind, prm, float3(p.x, p.y, p.z - h));
    return float3(dx * kSdfInvTwoStep, dy * kSdfInvTwoStep, dz * kSdfInvTwoStep);
}

// ---------------------------------------------------------------------------
// primitive_gradient -- world/sdf.cpp lines 166-213. Analytic for plane,
// sphere and box; the pinned central-difference stencil for the other four.
// ---------------------------------------------------------------------------
float3 primitive_gradient(uint kind, float4 prm, float3 p) {
    if (kind == kPrimPlane) {
        // sdf.cpp:170
        return float3(prm.x, prm.y, prm.z);
    }
    if (kind == kPrimSphere) {
        // sdf.cpp:173-177 -- p/len, or exactly zero at the centre (no defined
        // gradient there; an invented axis would be worse than none).
        const float len = glm_length3(p);
        return len > 0.0f ? float3(p.x / len, p.y / len, p.z / len) : float3(0.0f, 0.0f, 0.0f);
    }
    if (kind == kPrimBox) {
        // sdf.cpp:181-203
        const float3 q = abs(p) - float3(prm.x, prm.y, prm.z);
        const float qmax = glm_max(q.x, glm_max(q.y, q.z));
        if (qmax > 0.0f) {
            const float3 e = float3(glm_max(q.x, 0.0f), glm_max(q.y, 0.0f), glm_max(q.z, 0.0f));
            const float len = glm_length3(e);
            if (len <= 0.0f) {
                // sdf.cpp:189-191 -- unreachable: qmax > 0 implies len > 0.
                // Spelled `len <= 0.0f` verbatim to match sdf.cpp:189 (NOT
                // `!(len > 0.0f)`, which is NOT NaN-equivalent -- both
                // branches are provably unreachable on every input this
                // guard can see, but the port's own contract is to
                // reproduce the CPU's expression, not just its result).
                return float3(0.0f, 0.0f, 0.0f);
            }
            // `glm::vec3(sgn...) * (e / len)`: the division happens FIRST,
            // componentwise, then the sign multiply.
            return float3(sdf_sgn(p.x) * (e.x / len), sdf_sgn(p.y) * (e.y / len),
                          sdf_sgn(p.z) * (e.z / len));
        }
        // Interior: the outward axis of the deepest component. Ties break x,
        // then y, then z -- deterministic, and the order sdf.cpp:197-203 pins.
        if (q.x >= q.y && q.x >= q.z) {
            return float3(sdf_sgn(p.x), 0.0f, 0.0f);
        }
        if (q.y >= q.z) {
            return float3(0.0f, sdf_sgn(p.y), 0.0f);
        }
        return float3(0.0f, 0.0f, sdf_sgn(p.z));
    }
    // cylinder / capsule / torus / heightfield -- sdf.cpp:206-210.
    return central_difference(kind, prm, p);
}

// ---------------------------------------------------------------------------
// CSG combination -- world/sdf.cpp lines 232-283. Postfix operand order is
// "a b op": `a` is the deeper stack entry, `b` the shallower one.
// ---------------------------------------------------------------------------

// sdf.cpp:232-234 -- clamp(0.5 + 0.5*(b - a)/k, 0, 1). The one division in the
// CSG layer; banded, and it is the CPU's own expression.
float smooth_union_weight(float k, float a, float b) {
    return glm_clamp(0.5f + 0.5f * (b - a) / k, 0.0f, 1.0f);
}

float combine_distance(uint op, float4 prm, float a, float b) {
    if (op == kOpUnion) return a <= b ? a : b;              // sdf.cpp:240-241
    if (op == kOpIntersect) return a >= b ? a : b;          // sdf.cpp:242-243
    if (op == kOpSubtract) return a >= -b ? a : -b;         // sdf.cpp:244-245
    if (op == kOpSmoothUnion) {                              // sdf.cpp:246-253
        const float k = prm.x;
        if (!(k > 0.0f)) {
            return a <= b ? a : b;  // degenerate blend radius == plain union
        }
        const float h = smooth_union_weight(k, a, b);
        return (b + (a - b) * h) - k * h * (1.0f - h);
    }
    return a;  // kOpNone -- unreachable: primitives never reach here
}

float3 combine_gradient(uint op, float4 prm, float a, float b, float3 ga, float3 gb) {
    if (op == kOpUnion) return a <= b ? ga : gb;             // sdf.cpp:265-266
    if (op == kOpIntersect) return a >= b ? ga : gb;         // sdf.cpp:267-268
    if (op == kOpSubtract) return a >= -b ? ga : -gb;        // sdf.cpp:269-272
    if (op == kOpSmoothUnion) {                               // sdf.cpp:273-280
        const float k = prm.x;
        if (!(k > 0.0f)) {
            return a <= b ? ga : gb;
        }
        const float h = smooth_union_weight(k, a, b);
        return gb + (ga - gb) * h;
    }
    return ga;  // kOpNone -- unreachable
}

// ---------------------------------------------------------------------------
// The transform application, spelled column by column.
//
// glm::mat4 * glm::vec4 (detail/type_mat4x4.inl) is a PAIRWISE TREE:
//     Add0 = c0*x + c1*y;  Add1 = c2*z + c3*w;  result = Add0 + Add1
// and NOT the left fold the commented-out alternative in that same file shows.
// Getting this wrong would be invisible on an identity transform and a
// last-bits error on every posed one, so it is spelled explicitly.
// `w` is always 1 here (sdf.cpp:344 and :368 build glm::vec4(p, 1.0f)), and
// the c3*1.0f multiply is KEPT rather than folded away: fl(c3 * 1.0f) == c3
// exactly for every finite c3, but writing the multiply keeps the expression a
// literal transcription of the one being mirrored.
// ---------------------------------------------------------------------------
float3 sdf_transform_point(SdfTransformRow t, float3 p) {
    const float4 mul0 = t.col0 * p.x;
    const float4 mul1 = t.col1 * p.y;
    const float4 add0 = mul0 + mul1;
    const float4 mul2 = t.col2 * p.z;
    const float4 mul3 = t.col3 * 1.0f;
    const float4 add1 = mul2 + mul3;
    const float4 add2 = add0 + add1;
    // glm::vec3(vec4) truncates.
    return float3(add2.x, add2.y, add2.z);
}

// transpose(mat3(world_to_local)) * g, with glm's mat3*vec3 expression
// (detail/type_mat3x3.inl: `m[0][i]*v.x + m[1][i]*v.y + m[2][i]*v.z`,
// LEFT-associated). Transposing turns "column i of the transpose" into "row i
// of the original", so component i reads col_i's first three lanes -- which is
// why this comes out as three independent dot-shaped chains over ONE column
// each rather than across the columns.
float3 sdf_transform_gradient(SdfTransformRow t, float3 g) {
    return float3((t.col0.x * g.x + t.col0.y * g.y) + t.col0.z * g.z,
                  (t.col1.x * g.x + t.col1.y * g.y) + t.col1.z * g.z,
                  (t.col2.x * g.x + t.col2.y * g.y) + t.col2.z * g.z);
}

// ---------------------------------------------------------------------------
// SdfSample -- world/sdf.hpp's struct, ported. Not a mirrored LAYOUT (nothing
// uploads or downloads one; it never leaves a thread's registers), so it is
// declared here rather than in layouts.slang.
// ---------------------------------------------------------------------------
struct SdfSample {
    float distance;
    float3 gradient;
};

// ---------------------------------------------------------------------------
// sdf_sample -- world/sdf.cpp's sample(), lines 358-392: ONE walk of world
// `world`'s program yielding both the distance and the gradient.
//
// The stack discipline is the CPU's exactly: a primitive node PUSHES, an
// operator node POPS two (b = shallower, a = deeper) and PUSHES the combined
// pair. A program that does not reduce to exactly one value -- which for a
// VALIDATED program means an EMPTY one -- answers
// {kSdfEmptyDistance, (0,0,0)}, sdf.cpp:388-390.
//
// NO ASSERTS, matching the CPU pass: eval/sample are the physics inner loop and
// their documented precondition is a validated program. The one structural
// guard kept is the stack-depth clamp below, which cannot fire for a validated
// program (SdfProgram::validate() rejects anything needing more than
// kMaxSdfDepth slots) and which exists so a malformed upload cannot index a
// per-thread array out of bounds on a device where that is not a caught fault.
// ---------------------------------------------------------------------------
SdfSample sdf_sample(uint world, float3 p) {
    // ZERO-INITIALIZED, not merely declared. Slang's flow analysis cannot see
    // that the stack discipline below never reads a slot it did not write (a
    // VALIDATED program guarantees it, and validate() is the documented
    // precondition), and warns E41035 on the two pops. The initializer costs one
    // store of a null composite rather than a loop, and turns a would-be
    // undefined read on a MALFORMED upload into a defined zero -- which is the
    // same posture as the two `break` guards further down.
    float dist[kMaxSdfDepth] = {};
    float3 grad[kMaxSdfDepth] = {};
    uint sp = 0;

    const SdfWorldRange range = sdf_world_ranges[world];

    for (uint i = 0; i < range.node_count; ++i) {
        const SdfNodeRow node = sdf_nodes[range.node_begin + i];

        if (node.op == kOpNone) {
            if (sp >= kMaxSdfDepth) break;  // unreachable for a validated program
            const SdfTransformRow t = sdf_transforms[range.transform_begin + node.transform];
            const float3 local = sdf_transform_point(t, p);
            dist[sp] = primitive_distance(node.kind, node.params, local) * t.scale;
            // Chain rule through the inverse transform (sdf.cpp:370-374).
            grad[sp] = sdf_transform_gradient(t, primitive_gradient(node.kind, node.params, local)) * t.scale;
            ++sp;
        } else {
            if (sp < 2) break;  // unreachable for a validated program
            --sp;
            const float b = dist[sp];
            const float3 gb = grad[sp];
            --sp;
            const float a = dist[sp];
            const float3 ga = grad[sp];
            dist[sp] = combine_distance(node.op, node.params, a, b);
            grad[sp] = combine_gradient(node.op, node.params, a, b, ga, gb);
            ++sp;
        }
    }

    SdfSample out;
    if (sp != 1) {
        out.distance = kSdfEmptyDistance;
        out.gradient = float3(0.0f, 0.0f, 0.0f);
        return out;
    }
    out.distance = dist[0];
    out.gradient = grad[0];
    return out;
}
