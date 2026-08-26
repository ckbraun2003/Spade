#pragma once

// ---------------------------------------------------------------------------
// Analytic SDF scene programs (engine design D3, spec section 5 "Collision").
//
// The static world is a signed distance field built from primitives + rigid
// transforms + CSG ops, compiled by the WorldBuilder (builder.hpp) into a
// FLATTENED POSTFIX ARRAY of nodes. The flattening is the point: the CPU twin
// here and the Vulkan/Slang backend at S6 must evaluate the *identical*
// program, so the representation carries no pointers, no virtual dispatch and
// no recursion -- just an array of PODs plus an array of transforms, both of
// which upload to a GPU buffer verbatim.
//
// Evaluation is a stack machine over that array:
//   * a PRIMITIVE node computes its distance and PUSHES it;
//   * an OPERATOR node POPS two operands (a = deeper, b = shallower, i.e. the
//     postfix order "a b op") and PUSHES the combined distance.
// A well-formed program leaves exactly one value on the stack. The stack is a
// fixed-size automatic array of kMaxSdfDepth floats -- eval() never allocates.
//
// CONVENTIONS (parity-relevant -- the GPU port must match these exactly):
//   * fp32 everywhere; no doubles, and the only transcendental is the
//     heightfield's sine -- core/fp32_math.hpp's sin32, NEVER std::sin, since
//     this field feeds contacts and therefore the determinism corpus (see the
//     long note at that call site in sdf.cpp). std::sqrt stays: IEEE mandates
//     it correctly rounded, so it is already bit-identical everywhere.
//   * Local axes: cylinder and capsule are aligned to local +Y; the torus ring
//     lies in the local XZ plane with its hole axis along local +Y; the
//     heightfield's up axis is local +Y. Anything else is expressed with the
//     node's transform.
//   * Distances are negative inside the solid, positive outside, and (for every
//     primitive except heightfield) exact Euclidean distances.
//   * Subtraction is "a minus b": max(a, -b).
// ---------------------------------------------------------------------------

#include <cstdint>
#include <limits>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"

namespace spade {

// ---------------------------------------------------------------------------
// Node kinds and ops
// ---------------------------------------------------------------------------

// Primitive shapes. Underlying type fixed at uint32_t so the enum round-trips
// through the node's POD field (and, later, through a std430 buffer) and so
// casting an out-of-range value back is well defined rather than UB.
enum class SdfPrim : uint32_t {
    plane = 0,
    sphere,
    box,
    cylinder,
    capsule,
    torus,
    heightfield,
};
inline constexpr uint32_t kSdfPrimCount = 7;

// CSG operators. `none` marks a node as a primitive leaf -- it is not an op.
// `union_` carries the trailing underscore only because `union` is a keyword.
enum class SdfOp : uint32_t {
    none = 0,
    union_,
    intersect,
    subtract,
    smooth_union,
};
inline constexpr uint32_t kSdfOpCount = 5;

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

// One entry of the flattened program. A node is EITHER a primitive leaf
// (op == SdfOp::none; `kind` selects the shape and `transform` its pose) OR an
// operator (op != none; `kind` and `transform` are unused).
//
// `params` packing, by kind (unused lanes are zero):
//   plane        (n.x, n.y, n.z, offset)   -- unit normal n, plane is dot(p,n) = offset
//   sphere       (radius, 0, 0, 0)
//   box          (hx, hy, hz, 0)           -- half extents about the local origin
//   cylinder     (radius, half_height, 0, 0)          -- capped, local +Y axis
//   capsule      (radius, half_height, 0, 0)          -- segment (0,+-h,0), local +Y
//   torus        (major_radius, minor_radius, 0, 0)   -- ring in local XZ, hole axis +Y
//   heightfield  (amplitude, freq_x, freq_z, base_y)  -- see below
//   smooth_union (k, 0, 0, 0)              -- blend radius; k <= 0 degenerates to min
//
// One vec4 is deliberate, not an oversight: every kind above fits in four
// floats, and this array is uploaded per world to the GPU at S6. A second vec4
// is a size change, not a semantic one, whenever a kind needs it.
//
// The heightfield is a PROCEDURAL, analytic terrain (D3 says "analytic SDF
// static world"; a data-backed sample grid would need a third program-owned
// buffer and belongs with the world file at S5):
//     h(x, z) = base_y + amplitude * sin(freq_x * x) * sin(freq_z * z)
//     phi(p)  = (p.y - h(p.x, p.z)) / sqrt(1 + (amplitude * max(|fx|,|fz|))^2)
// Solid below the surface. The divisor is the exact Lipschitz bound of the
// numerator (max over x,z of |grad h| is amplitude * max(|fx|,|fz|)), so phi is
// a conservative distance bound with |grad phi| <= 1 everywhere -- safe for
// sphere-tracing and for penetration depth, but NOT an exact metric. It is the
// one primitive whose distance under-estimates.
struct SdfNode {
    uint32_t kind = 0;       // SdfPrim; meaningful only when op == SdfOp::none
    uint32_t op = 0;         // SdfOp
    uint32_t transform = 0;  // index into SdfProgram::transforms
    uint32_t _pad = 0;       // keeps params on a 16-byte boundary for std430
    glm::vec4 params{0.0f};
};
static_assert(sizeof(SdfNode) == 32, "SdfNode must stay a 32-byte POD (std430 upload at S6)");
static_assert(sizeof(SdfNode) % 16 == 0, "SdfNode must be a whole number of vec4s");

// A node's pose, stored PRE-INVERTED: evaluation transforms the query point
// into node-local space, so the inverse is what the hot loop needs and
// inverting once at build time keeps it out of eval().
//
// Rigid + UNIFORM scale only. Non-uniform scale does not preserve the metric
// (distances along different axes scale differently, so the result is no longer
// a distance field), which is why the builder takes a single float scale.
//   p_local  = world_to_local * p_world
//   d_world  = scale * d_local(p_local)
//   grad_world = scale * transpose(mat3(world_to_local)) * grad_local
// For a rigid+uniform transform L = T*R*S(s), scale*transpose(mat3(inv(L)))
// reduces to the rotation R, so gradient magnitude is preserved exactly.
struct SdfTransform {
    glm::mat4 world_to_local{1.0f};
    float scale = 1.0f;
    float _pad[3] = {0.0f, 0.0f, 0.0f};
};
static_assert(sizeof(SdfTransform) == 80, "SdfTransform must stay an 80-byte POD");
static_assert(sizeof(SdfTransform) % 16 == 0, "SdfTransform must be a whole number of vec4s");

// ---------------------------------------------------------------------------
// Pinned constants
// ---------------------------------------------------------------------------

// Maximum evaluation-stack depth. This is the peak number of operand results
// live at once (NOT the node count and NOT the tree height): a chain of
// unions never exceeds 2, while N primitives pushed back-to-back before any op
// costs N. eval()'s stack is exactly this many floats.
inline constexpr uint32_t kMaxSdfDepth = 32;

// Central-difference step for the kinds without an analytic gradient. PINNED:
// both backends must use this literal stencil or gradients (hence contact
// normals) diverge. See gradient() for the exact expression.
inline constexpr float kSdfGradientStep = 1e-3f;

// Distance reported for a program with no geometry. Finite rather than
// infinity so that central differences over an empty program yield 0 instead
// of NaN; a NaN normal would poison a whole world's contact solve.
inline constexpr float kSdfEmptyDistance = std::numeric_limits<float>::max();

// ---------------------------------------------------------------------------
// Program
// ---------------------------------------------------------------------------

// The flattened program. Aggregate on purpose: tests and (later) the world-file
// loader build one directly; the WorldBuilder is the sanctioned authoring path.
// transforms[0] is the identity by convention -- every node whose pose is the
// identity points at it.
struct SdfProgram {
    std::vector<SdfNode> nodes;
    std::vector<SdfTransform> transforms;

    // Per-node material index (schema v2, PA-2). A PARALLEL, HOST-ONLY array,
    // deliberately NOT a field of SdfNode: SdfNode is mirrored byte-for-byte
    // into compute::SdfNodeRow (compute/sdf_program.hpp) and into
    // shaders/shared/layouts.slang, both of which index it BY NAME, and
    // SdfNode's only spare lane (`_pad`) exists to keep `params` on a
    // 16-byte boundary for that std430 upload -- putting a material index
    // there would silently corrupt GPU-side layout. Rendering-only: physics
    // (eval/sample/gradient above) never reads this array.
    //
    // Either empty (every node uses WorldDesc::materials[0], the default) or
    // exactly nodes.size() long, one entry per node in the SAME order as
    // `nodes` -- including operator nodes, which simply carry whatever index
    // WorldBuilder::material_for_last_node() last set for that slot (a
    // consumer that shades by leaf primitive, not by CSG node, is free to
    // ignore an operator node's entry). validate() below checks the length;
    // WHETHER an index is in range depends on WorldDesc::materials, which
    // this type does not have, so that check lives in validate_world_desc()
    // (world/builder.hpp) instead.
    std::vector<uint32_t> node_materials;

    [[nodiscard]] bool empty() const noexcept { return nodes.empty(); }

    // Structural + parameter validation. On success returns the peak evaluation
    // stack depth (0 for an empty program).
    //
    // Codes: capacity_exceeded when the program would need more than
    // kMaxSdfDepth stack slots (a fixed capacity, exceeded); invalid_argument
    // for everything else -- malformed postfix (operator without two operands,
    // or more than one value left over), out-of-range kind/op/transform index,
    // non-finite or nonsensical parameters, non-unit plane normal,
    // non-positive transform scale, or a node_materials length that is
    // neither 0 nor nodes.size().
    [[nodiscard]] Result<uint32_t> validate() const;
};

// ---------------------------------------------------------------------------
// Evaluation
//
// PRECONDITION for eval/gradient/sample: `program` has passed validate().
// They are the physics inner loop -- no revalidation, no bounds checks beyond a
// debug assert. Anything the builder or the world loader produces is valid by
// construction; a hand-built program must be validated by its author.
// ---------------------------------------------------------------------------

// Signed distance at world-space point p. Negative inside the solid.
[[nodiscard]] float eval(const SdfProgram& program, glm::vec3 p) noexcept;

// Field gradient at p (the un-normalized surface normal; magnitude is 1
// wherever the field is an exact metric and differentiable).
//
// Analytic for plane, sphere and box, and analytically propagated through every
// op: union/intersect/subtract select a branch, smooth_union blends with weight
// h (see combine notes in the .cpp). For cylinder, capsule, torus and
// heightfield the LOCAL primitive gradient comes from central differences with
// the pinned step h = kSdfGradientStep, axes evaluated in x, y, z order:
//     g_i = (phi(p + h*e_i) - phi(p - h*e_i)) * (0.5f / h)
[[nodiscard]] glm::vec3 gradient(const SdfProgram& program, glm::vec3 p) noexcept;

// Distance and gradient in one walk. Contact generation wants both, and
// gradient() computes the distance anyway. sample(p).distance is bit-identical
// to eval(p): both funnel through the same primitive and combine helpers.
struct SdfSample {
    float distance = kSdfEmptyDistance;
    glm::vec3 gradient{0.0f};
};
[[nodiscard]] SdfSample sample(const SdfProgram& program, glm::vec3 p) noexcept;

}  // namespace spade
