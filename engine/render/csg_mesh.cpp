#include "render/csg_mesh.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace spade::render {
namespace {

// ---------------------------------------------------------------------------
// subtree_start -- given the index of a node whose pushed value is the root
// of some subtree (any node, not just the program's own overall root), finds
// the index of the FIRST node belonging to that subtree, by scanning
// backward and tracking how many more "productions" the scan still owes.
//
// A primitive fills exactly one owed production; an operator fills one
// (itself) but then owes two more (its own two operands, `a b op`, both of
// which appear somewhere to its left). Starting `need` at 1 (for `end`
// itself) and stopping the instant it returns to 0 is precisely the
// postfix-parsing rule that makes a flat array's subtrees contiguous ranges
// in the first place -- SdfProgram::validate() already guarantees the whole
// program parses this way (every operator has exactly two operands
// available when it runs), so the same rule applied to a sub-range starting
// at `end` and scanning left is guaranteed to terminate inside [0, end].
//
// PRECONDITION: `program` has passed validate() and `end < program.nodes.
// size()`. Matches world/sdf.hpp's own eval()/gradient()/sample() PRECONDITION
// convention -- this is an internal helper on the same trusted-input basis,
// not a second validator.
[[nodiscard]] uint32_t subtree_start(const SdfProgram& program, uint32_t end) {
    int64_t need = 1;
    for (int64_t i = static_cast<int64_t>(end); i >= 0; --i) {
        --need;
        if (program.nodes[static_cast<size_t>(i)].op != static_cast<uint32_t>(SdfOp::none)) {
            need += 2;
        }
        if (need == 0) {
            return static_cast<uint32_t>(i);
        }
    }
    // Unreachable for a validated program: node `end`'s own subtree is by
    // construction fully contained in [0, end].
    assert(false && "subtree_start: malformed postfix program (unvalidated input?)");
    return 0;
}

// Returns (a_root, b_root): the two operand subtree ROOTS of the operator
// node at `op_node_index` (postfix "a b op" -- a is the deeper/first-
// authored operand, b the shallower/second, i.e. the one immediately
// preceding the operator). Shared by split_program()'s union decomposition
// below and csg_subtree_world_bounds()'s recursive CSG-aware walk further
// down -- both need to locate an operator's two operands the same way, and
// the arithmetic (subtree_start() applied twice) does not depend on which
// operator it is.
[[nodiscard]] std::pair<uint32_t, uint32_t> operand_roots(const SdfProgram& program, uint32_t op_node_index) {
    const uint32_t b_root = op_node_index - 1;
    const uint32_t b_start = subtree_start(program, b_root);
    assert(b_start >= 1 && "an operator's first operand must occupy at least one node");
    const uint32_t a_root = b_start - 1;
    return {a_root, b_root};
}

// ---------------------------------------------------------------------------
// split_program -- see csg_mesh.hpp's file comment for the two populations.
//
// Recursion depth is bounded by kMaxSdfDepth (SdfProgram's own stack-depth
// cap): visit() only ever recurses into `union`'s two operands, and a chain
// of N union nodes over primitives never nests more than N frames deep --
// the same bound the evaluation stack itself is pinned to.
// ---------------------------------------------------------------------------
struct SplitCollector {
    const SdfProgram& program;
    SubtreeSplit result;

    void visit(uint32_t node_index) {
        assert(node_index < program.nodes.size());
        const SdfNode& node = program.nodes[node_index];

        if (node.op == static_cast<uint32_t>(SdfOp::none)) {
            result.union_primitive_nodes.push_back(node_index);
            return;
        }

        if (node.op == static_cast<uint32_t>(SdfOp::union_)) {
            // Visiting a before b keeps both output vectors in the
            // program's own left-to-right order (csg_mesh.hpp's ORDER
            // guarantee) -- exactly the order a human authoring
            // `.box(...).sphere(...).union_()` wrote them in.
            const auto [a_root, b_root] = operand_roots(program, node_index);
            visit(a_root);
            visit(b_root);
            return;
        }

        // subtract / intersect / smooth_union: genuine CSG (csg_mesh.hpp's
        // file comment). The WHOLE subtree meshes as one unit -- do not
        // recurse into its operands, even if one of them is itself a
        // `union` of several primitives: mesh_csg_subtree() evaluates the
        // subtree's SDF directly, which already accounts for every operator
        // nested inside it.
        result.csg_roots.push_back(node_index);
    }
};

}  // namespace

Result<SubtreeSplit> split_program(const SdfProgram& program) {
    if (program.nodes.empty()) {
        return SubtreeSplit{};
    }
    SplitCollector collector{program, SubtreeSplit{}};
    collector.visit(static_cast<uint32_t>(program.nodes.size()) - 1);
    return std::move(collector.result);
}

namespace {

// ---------------------------------------------------------------------------
// csg_subtree_world_bounds -- a CSG-AWARE recursive walk (S7a Task R5 fix
// wave, review IMPORTANT #1) that respects what each OPERATOR does to its
// operands' extents, not a flat union of every leaf's own shape regardless
// of what sits above it -- see subtree_extent()'s own doc comment further
// down for the per-operator rules.
// ---------------------------------------------------------------------------

// Mirrors render/scene.cpp's own local_to_world_of() exactly (same one-line
// body): SdfTransform::world_to_local is stored PRE-INVERTED, so recovering
// the forward matrix is one glm::inverse() call. Duplicated rather than
// shared because scene.cpp's copy has internal linkage (anonymous
// namespace) in its own translation unit -- there is nothing to link
// against, and a one-line function is not worth a shared header for.
[[nodiscard]] glm::mat4 local_to_world_of(const SdfTransform& xf) {
    return glm::inverse(xf.world_to_local);
}

// Local-space AABB of one primitive's own shape. nullopt for plane and
// heightfield, whose analytic field has no inherent extent over its own
// local axes (PA-5, tessellate.hpp's identical exemption for the same two
// kinds) -- csg_subtree_world_bounds() substitutes `fallback_bounds` for
// these instead of a shape extent that does not exist.
[[nodiscard]] std::optional<Aabb> primitive_local_aabb(SdfPrim kind, glm::vec4 params) {
    switch (kind) {
        case SdfPrim::plane:
        case SdfPrim::heightfield:
            return std::nullopt;
        case SdfPrim::sphere: {
            const float r = params.x;
            return Aabb{glm::vec3(-r), glm::vec3(r)};
        }
        case SdfPrim::box: {
            const glm::vec3 h(params.x, params.y, params.z);
            return Aabb{-h, h};
        }
        case SdfPrim::cylinder: {
            const float r = params.x, hh = params.y;
            return Aabb{glm::vec3(-r, -hh, -r), glm::vec3(r, hh, r)};
        }
        case SdfPrim::capsule: {
            // The hemispherical caps extend `radius` beyond half_height
            // along local Y (tessellate.hpp's own capsule comment: "a REAL
            // capsule ... never the flat-cap cylinder shortcut").
            const float r = params.x, hh = params.y;
            return Aabb{glm::vec3(-r, -(hh + r), -r), glm::vec3(r, hh + r, r)};
        }
        case SdfPrim::torus: {
            const float major = params.x, minor = params.y;
            const float outer = major + minor;
            return Aabb{glm::vec3(-outer, -minor, -outer), glm::vec3(outer, minor, outer)};
        }
    }
    return std::nullopt;
}

[[nodiscard]] Aabb transform_aabb(const Aabb& local, const glm::mat4& local_to_world) {
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(std::numeric_limits<float>::lowest());
    for (uint32_t corner = 0; corner < 8; ++corner) {
        const glm::vec3 c((corner & 1u) ? local.max.x : local.min.x, (corner & 2u) ? local.max.y : local.min.y,
                           (corner & 4u) ? local.max.z : local.min.z);
        const glm::vec3 w(local_to_world * glm::vec4(c, 1.0f));
        lo = glm::min(lo, w);
        hi = glm::max(hi, w);
    }
    return Aabb{lo, hi};
}

[[nodiscard]] Aabb union_aabb(const Aabb& a, const Aabb& b) {
    return Aabb{glm::min(a.min, b.min), glm::max(a.max, b.max)};
}

// Aabb-Aabb intersection (component-wise). If the two boxes do not actually
// overlap on some axis (not expected of any REAL CSG operand pair -- an
// author who authors intersect() of two disjoint shapes gets an empty
// result everywhere -- but not provably impossible of caller-supplied
// data), the naive min(max)/max(min) would invert that axis's extent;
// collapsing it to a single point instead keeps the returned Aabb always
// well-formed (min <= max on every axis) rather than propagating a negative
// extent into the grid-sizing arithmetic downstream.
[[nodiscard]] Aabb intersect_aabb(const Aabb& a, const Aabb& b) {
    Aabb result{glm::max(a.min, b.min), glm::min(a.max, b.max)};
    for (int axis = 0; axis < 3; ++axis) {
        if (result.min[axis] > result.max[axis]) {
            const float mid = 0.5f * (result.min[axis] + result.max[axis]);
            result.min[axis] = result.max[axis] = mid;
        }
    }
    return result;
}

// ---------------------------------------------------------------------------
// subtree_extent -- the CSG-AWARE recursive walk that replaced a flat
// "union of every leaf's own extent" (S7a Task R5 fix wave, review
// IMPORTANT #1). `nullopt` means UNBOUNDED: the subtree
// rooted at `node_index` provably has no finite extent (a plane/heightfield
// leaf, or any combination that stays infinite through it) -- the
// union/intersect cases below need that as a real value, not just "missing
// data", because union() and intersect() treat "one operand is unbounded"
// completely differently:
//
//   union         -- unbounded if EITHER operand is (a genuinely infinite
//                     piece makes the whole union infinite); otherwise the
//                     union of both operands' extents.
//   intersect     -- an unbounded operand is NEUTRAL: "a b intersect"'s
//                     result is a subset of BOTH a and b (sdf.cpp's
//                     combine_distance: max(a,b), solid only where both
//                     are), so intersecting with "everywhere" cannot make
//                     the OTHER operand's own bound any looser. Only
//                     unbounded when BOTH operands are.
//   subtract      -- "a minus b" is a subset of `a` ALONE (sdf.cpp: max(a,
//                     -b)) -- `b`'s extent, bounded or not, never enters the
//                     result's own bound.
//   smooth_union  -- the union of both operands' extents, DILATED by k/4:
//                     sdf.cpp's own bound on the blend (min(a,b) - k/4 <= d
//                     <= min(a,b)) means the solid can extend up to k/4
//                     beyond where the naive union's own boundary would put
//                     it. Unbounded under the same rule as plain union.
//
// This is what lets an intersect() clipped to a small box (maximal.world.
// yaml's own node 4: (plane u sphere) n box) get the BOX's own tight bound
// instead of either the union's unbounded result or a world-sized fallback
// -- the fallback is only ever reached (csg_subtree_world_bounds(), below)
// when the WHOLE subtree's own extent is still unbounded after every
// operator in it has had its say.
[[nodiscard]] Result<std::optional<Aabb>> subtree_extent(const SdfProgram& program, uint32_t node_index) {
    const SdfNode& node = program.nodes[node_index];

    if (node.op == static_cast<uint32_t>(SdfOp::none)) {
        if (node.transform >= program.transforms.size()) {
            return std::unexpected(
                Error{Code::invalid_argument, "csg_subtree_world_bounds: SDF transform index out of range"});
        }
        const std::optional<Aabb> local = primitive_local_aabb(static_cast<SdfPrim>(node.kind), node.params);
        if (!local.has_value()) {
            return std::optional<Aabb>(std::nullopt);  // plane/heightfield: unbounded.
        }
        const glm::mat4 local_to_world = local_to_world_of(program.transforms[node.transform]);
        return std::optional<Aabb>(transform_aabb(*local, local_to_world));
    }

    const auto [a_root, b_root] = operand_roots(program, node_index);
    const Result<std::optional<Aabb>> a_extent = subtree_extent(program, a_root);
    if (!a_extent) {
        return a_extent;
    }
    const Result<std::optional<Aabb>> b_extent = subtree_extent(program, b_root);
    if (!b_extent) {
        return b_extent;
    }
    const std::optional<Aabb>& a = *a_extent;
    const std::optional<Aabb>& b = *b_extent;

    switch (static_cast<SdfOp>(node.op)) {
        case SdfOp::union_:
            if (!a.has_value() || !b.has_value()) {
                return std::optional<Aabb>(std::nullopt);
            }
            return std::optional<Aabb>(union_aabb(*a, *b));
        case SdfOp::intersect:
            if (!a.has_value()) {
                return b_extent;  // b alone, bounded or not (a was neutral).
            }
            if (!b.has_value()) {
                return a_extent;  // a alone, bounded or not (b was neutral).
            }
            return std::optional<Aabb>(intersect_aabb(*a, *b));
        case SdfOp::subtract:
            return a_extent;  // "a minus b": b's extent never enters the bound.
        case SdfOp::smooth_union: {
            if (!a.has_value() || !b.has_value()) {
                return std::optional<Aabb>(std::nullopt);
            }
            const float k = node.params.x;
            const float dilation = k > 0.0f ? k * 0.25f : 0.0f;
            const Aabb u = union_aabb(*a, *b);
            return std::optional<Aabb>(Aabb{u.min - glm::vec3(dilation), u.max + glm::vec3(dilation)});
        }
        case SdfOp::none:
            break;  // unreachable -- handled above.
    }
    return std::optional<Aabb>(std::nullopt);  // unreachable for a validated op.
}

}  // namespace

Result<Aabb> csg_subtree_world_bounds(const SdfProgram& program, uint32_t root_node, const Aabb& fallback_bounds) {
    if (root_node >= program.nodes.size()) {
        return std::unexpected(Error{Code::invalid_argument, "csg_subtree_world_bounds: root_node out of range"});
    }
    const Result<std::optional<Aabb>> extent = subtree_extent(program, root_node);
    if (!extent) {
        return std::unexpected(extent.error());
    }
    // Only reached when the SUBTREE'S OWN extent is still unbounded after
    // every operator inside it has had its say (subtree_extent()'s own file
    // comment) -- e.g. a subtract whose minuend is itself a plane. Not the
    // fallback's ONLY job before this fix wave: previously every plane/
    // heightfield leaf routed here regardless of what operator sat above
    // it, which is what starved intersect()'s and subtract()'s OTHER,
    // genuinely-bounded operand of grid resolution (review IMPORTANT #1).
    return extent->has_value() ? **extent : fallback_bounds;
}

// ---------------------------------------------------------------------------
// mesh_csg_subtree -- surface nets.
// ---------------------------------------------------------------------------
namespace {

// A thin, ordered accumulator -- identical shape to tessellate.cpp's own
// MeshBuilder (duplicated for the same reason local_to_world_of() is above:
// a private helper with internal linkage in another translation unit is not
// something this file can reach).
struct MeshBuilder {
    MeshData mesh;

    uint32_t add_vertex(glm::vec3 position, glm::vec3 normal) {
        const uint32_t index = static_cast<uint32_t>(mesh.positions.size());
        mesh.positions.push_back(position);
        mesh.normals.push_back(normal);
        return index;
    }

    void add_triangle(uint32_t a, uint32_t b, uint32_t c) {
        mesh.indices.push_back(a);
        mesh.indices.push_back(b);
        mesh.indices.push_back(c);
    }
};

// The 12 edges of a unit cube, as pairs of corner indices (corner `c` has
// local coordinate ((c&1), (c>>1&1), (c>>2&1))). Generated rather than
// hand-listed: every corner connects to the 3 neighbours one bit away, and
// requiring `i < p` counts each of the resulting 12 undirected edges exactly
// once.
[[nodiscard]] std::array<std::array<uint32_t, 2>, 12> make_cube_edges() {
    std::array<std::array<uint32_t, 2>, 12> edges{};
    uint32_t k = 0;
    for (uint32_t i = 0; i < 8; ++i) {
        for (const uint32_t bit : {1u, 2u, 4u}) {
            const uint32_t p = i ^ bit;
            if (i < p) {
                edges[k++] = {i, p};
            }
        }
    }
    assert(k == 12);
    return edges;
}

// Emits the two triangles of a quad (v00, v10, v11, v01), a cyclic loop
// around one interior grid edge (see the three edge-scan loops below for
// what each of the four names means for that axis). `flip` reverses both
// triangles' winding -- exactly tessellate.cpp's append_grid()'s own
// flip_winding parameter, same shape, same reason: which cyclic direction
// faces outward depends on facts (which axis, which side of the sign
// transition) the caller already knows and this function does not need to
// re-derive.
void emit_quad(MeshBuilder& out, uint32_t v00, uint32_t v10, uint32_t v11, uint32_t v01, bool flip) {
    if (!flip) {
        out.add_triangle(v00, v10, v11);
        out.add_triangle(v00, v11, v01);
    } else {
        out.add_triangle(v00, v11, v10);
        out.add_triangle(v00, v01, v11);
    }
}

// `inside` is `SdfProgram`'s own sign convention (sdf.hpp: "negative inside
// the solid"), spelled once so every call site agrees on the boundary case
// (v == 0.0f reads as outside/boundary, not inside -- an arbitrary but
// consistent tie-break, exactly sdf.cpp's own sgn() for the identical
// reason: a deterministic answer at a measure-zero case beats an
// unspecified one).
[[nodiscard]] bool inside(float v) noexcept { return v < 0.0f; }

// AXIS HANDEDNESS CONSTANTS -- derived below, then independently confirmed
// empirically (S7a Task R5 fix wave, review): iterating a placeholder guess
// against CsgMesh.TriangleWindingIsConsistentlyOutward and reading off which
// axes needed a flip landed on exactly the values the derivation predicts,
// which is what makes it a derivation rather than three numbers that merely
// pass one test.
//
// Every edge-scan loop's UNFLIPPED order (v00, v10, v11, v01) walks the two
// non-scan axes in the cyclic order (lo,lo) -> (hi,lo) -> (hi,hi) -> (lo,hi)
// -- e.g. the X-scan (mesh_csg_subtree()'s first loop) walks (j,k), i.e.
// (Y,Z). The unflipped triangle (v00,v10,v11)'s own face normal is
// cross(v10-v00, v11-v00): for the X-scan that is cross(+Y, +Y+Z) = +X
// (cross(Y,Z) = +X in this right-handed frame); by the identical
// construction the Y-scan walks (X,Z) and gives cross(+X,+X+Z) = cross(X,Z)
// = -Y; the Z-scan walks (X,Y) and gives cross(+X,+X+Y) = cross(X,Y) = +Z.
//
// `va` (mesh_csg_subtree()'s own name for the field value at the LOWER
// corner along the scan axis) tells which way is actually outward: if `va`
// is INSIDE (solid sits at the lower coordinate), the solid-to-empty
// direction -- hence the wanted outward normal -- is the scan axis's OWN
// POSITIVE direction. So the unflipped order is already correct (no flip
// needed) exactly when its own default normal already points in that
// scan axis's positive direction:
//   X-scan: unflipped default is +X (positive)     -> no flip when va INSIDE
//                                                      -> flip = !inside(va) -> kFlipX = true
//   Y-scan: unflipped default is -Y (NEGATIVE)      -> flip WHEN va INSIDE
//                                                      -> flip =  inside(va) -> kFlipY = false
//   Z-scan: unflipped default is +Z (positive)      -> no flip when va INSIDE
//                                                      -> flip = !inside(va) -> kFlipZ = true
// (`flip = inside(va) != kFlipX` reduces to `!inside(va)` when kFlipX=true,
// and to `inside(va)` when kFlipY=false -- exactly the two rules above.)
constexpr bool kFlipX = true;
constexpr bool kFlipY = false;
constexpr bool kFlipZ = true;

// csg_mesh.hpp's grid rule: max(min, ceil(longest / cell_size)), capped at
// max. 0 for limits that cannot give a grid.
[[nodiscard]] uint32_t cells_for_extent(float longest, const CsgMeshLimits& limits) {
    if (!(limits.cell_size > 0.0f) || limits.min_cells_per_axis == 0u ||
        limits.max_cells_per_axis < limits.min_cells_per_axis) {
        return 0u;
    }
    const float wanted = std::ceil(longest / limits.cell_size);
    if (!(wanted < static_cast<float>(limits.max_cells_per_axis))) {
        return limits.max_cells_per_axis;
    }
    return std::max(limits.min_cells_per_axis, static_cast<uint32_t>(std::max(wanted, 0.0f)));
}

}  // namespace

SdfProgram csg_subtree_program(const SdfProgram& program, uint32_t root_node) {
    const uint32_t start = subtree_start(program, root_node);
    SdfProgram subtree;
    subtree.nodes.assign(program.nodes.begin() + start, program.nodes.begin() + root_node + 1);
    subtree.transforms = program.transforms;
    return subtree;
}

Aabb csg_subtree_sample_box(const SdfProgram& program, uint32_t root_node, const Aabb& subtree_bounds,
                            const CsgMeshLimits& limits) {
    // The padding margin is `limits.aabb_margin`, EXCEPT when `root_node`
    // is itself a smooth_union: sdf.cpp's own bound on the blend (min(a,b)
    // - k/4 <= d <= min(a,b)) means the true surface can sit up to k/4
    // beyond wherever `subtree_bounds` puts it, and a FIXED aabb_margin is
    // not a bound against an arbitrary k (review IMPORTANT #2 -- this
    // task's own committed smooth_union golden uses k=0.25, k/4=0.0625,
    // already past the 0.05 default). This is deliberately independent of
    // csg_subtree_world_bounds()'s OWN k/4 dilation for a smooth_union
    // ANYWHERE in the subtree (subtree_extent(), above): that dilation
    // tightens the STARTING bound for resolution's sake and only ever
    // widens `subtree_bounds` for a caller who actually uses it; this check
    // is the sample box's own defence against a caller who does not
    // (a hand-picked bound, a future call site), so the root's own blend
    // radius is never silently clipped no matter how `subtree_bounds` was
    // produced.
    float aabb_margin = limits.aabb_margin;
    if (root_node < program.nodes.size()) {
        const SdfNode& root = program.nodes[root_node];
        if (root.op == static_cast<uint32_t>(SdfOp::smooth_union)) {
            const float k = root.params.x;
            if (k > 0.0f) {
                aabb_margin = std::max(aabb_margin, k * 0.25f);
            }
        }
    }
    const glm::vec3 margin(aabb_margin);
    return Aabb{.min = subtree_bounds.min - margin, .max = subtree_bounds.max + margin};
}

uint32_t csg_cells_per_axis(const Aabb& subtree_bounds, const CsgMeshLimits& limits) {
    const glm::vec3 extent = subtree_bounds.max - subtree_bounds.min + glm::vec3(2.0f * limits.aabb_margin);
    return cells_for_extent(std::max({extent.x, extent.y, extent.z}), limits);
}

float csg_mesh_cell_size(const SdfProgram& program, uint32_t root_node, const Aabb& subtree_bounds,
                         const CsgMeshLimits& limits) {
    const Aabb box = csg_subtree_sample_box(program, root_node, subtree_bounds, limits);
    const glm::vec3 extent = box.max - box.min;
    const float longest = std::max({extent.x, extent.y, extent.z});
    const uint32_t cells = cells_for_extent(longest, limits);
    return cells == 0u ? 0.0f : longest / static_cast<float>(cells);
}

CsgFoldReport find_folded_triangles(const SdfProgram& program, uint32_t root_node, const MeshData& mesh,
                                    float cell) {
    (void)cell;  // RED: every fold still reads as a thin wall
    CsgFoldReport report;
    if (root_node >= program.nodes.size()) {
        return report;
    }
    const SdfProgram subtree = csg_subtree_program(program, root_node);
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(std::numeric_limits<float>::lowest());
    for (size_t t = 0; t + 2u < mesh.indices.size(); t += 3u) {
        const glm::vec3& a = mesh.positions[mesh.indices[t]];
        const glm::vec3& b = mesh.positions[mesh.indices[t + 1u]];
        const glm::vec3& c = mesh.positions[mesh.indices[t + 2u]];
        const glm::vec3 n = glm::cross(b - a, c - a);
        if (!(glm::dot(n, n) > 0.0f)) {
            continue;  // degenerate: it has no facing to judge
        }
        ++report.triangles;
        if (glm::dot(n, gradient(subtree, (a + b + c) / 3.0f)) < 0.0f) {
            ++report.folded;
            lo = glm::min(lo, glm::min(a, glm::min(b, c)));
            hi = glm::max(hi, glm::max(a, glm::max(b, c)));
        }
    }
    if (report.folded > 0u) {
        report.bounds = Aabb{.min = lo, .max = hi};
        report.thin = report.folded;
        report.thin_bounds = report.bounds;
    }
    return report;
}

Result<MeshData> mesh_csg_subtree(const SdfProgram& program, uint32_t root_node, const Aabb& subtree_bounds,
                                   const CsgMeshLimits& limits) {
    if (root_node >= program.nodes.size()) {
        return std::unexpected(Error{Code::invalid_argument, "mesh_csg_subtree: root_node out of range"});
    }
    // Evaluate the SUBTREE ALONE, not the whole program: this subtree sits
    // inside a larger union with the rest of the world's geometry (a gate
    // stands on a ground plane, say), and evaluating the full program near
    // the gate would let that OTHER geometry leak into this mesh's field
    // wherever it happens to be close by -- exactly the wrong answer, since
    // the whole point of splitting the program (csg_mesh.hpp's file
    // comment) is that a union's silhouette comes from drawing separate
    // meshes with depth testing, never from baking the union into one mesh.
    //
    // Extracting [start, root_node] into its own SdfProgram and calling the
    // PUBLIC eval()/gradient() on it (rather than re-implementing
    // primitive_distance()/combine_distance() here) is what guarantees this
    // mesh's surface agrees with the physics field bit for bit: same code,
    // same rounding, by construction rather than by two hand-written copies
    // staying in sync. `transforms` is copied WHOLE (not sliced) because the
    // extracted nodes' `transform` fields are indices into the ORIGINAL
    // program's transform table and must keep resolving to the same entries.
    const SdfProgram subtree = csg_subtree_program(program, root_node);
    const Result<uint32_t> peak = subtree.validate();
    if (!peak) {
        // Unreachable for a subtree of an already-validated `program` (see
        // csg_mesh.hpp's own doc comment) -- kept as a propagated Result
        // rather than an assert because this still names the SPECIFIC
        // rejection reason if the precondition is ever violated.
        return std::unexpected(peak.error());
    }

    // Sampled over the padded box (csg_subtree_sample_box()'s comment says
    // why a smooth_union root pads by k/4).
    const Aabb box = csg_subtree_sample_box(program, root_node, subtree_bounds, limits);
    const glm::vec3 lo = box.min;
    const glm::vec3 hi = box.max;
    const glm::vec3 extent = hi - lo;

    const uint32_t cells = cells_for_extent(std::max({extent.x, extent.y, extent.z}), limits);
    if (cells == 0u) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "mesh_csg_subtree: the limits give no cells (cell_size, min_cells_per_axis and "
                                     "max_cells_per_axis must be positive, with min <= max)"});
    }
    const uint32_t verts_per_axis = cells + 1;

    const auto corner_pos = [&](uint32_t i, uint32_t j, uint32_t k) -> glm::vec3 {
        return lo + extent * glm::vec3(static_cast<float>(i) / static_cast<float>(cells),
                                        static_cast<float>(j) / static_cast<float>(cells),
                                        static_cast<float>(k) / static_cast<float>(cells));
    };
    const auto field_index = [&](uint32_t i, uint32_t j, uint32_t k) -> size_t {
        return (static_cast<size_t>(k) * verts_per_axis + j) * verts_per_axis + i;
    };

    // Sample the subtree's own SDF at every grid corner, once. FIXED,
    // index-driven loop order (constraint 3, determinism) -- k outermost,
    // i innermost -- the same shape every generator in tessellate.cpp uses.
    std::vector<float> field(static_cast<size_t>(verts_per_axis) * verts_per_axis * verts_per_axis);
    for (uint32_t k = 0; k < verts_per_axis; ++k) {
        for (uint32_t j = 0; j < verts_per_axis; ++j) {
            for (uint32_t i = 0; i < verts_per_axis; ++i) {
                field[field_index(i, j, k)] = eval(subtree, corner_pos(i, j, k));
            }
        }
    }

    MeshBuilder out;
    constexpr uint32_t kNoVertex = std::numeric_limits<uint32_t>::max();
    std::vector<uint32_t> cell_vertex(static_cast<size_t>(cells) * cells * cells, kNoVertex);
    const auto cell_index = [&](uint32_t ci, uint32_t cj, uint32_t ck) -> size_t {
        return (static_cast<size_t>(ck) * cells + cj) * cells + ci;
    };

    static const std::array<std::array<uint32_t, 2>, 12> kCubeEdges = make_cube_edges();

    // --- Pass 1: one vertex per ACTIVE cell (a cell whose 8 corners are not
    // all the same sign) -- surface nets' own placement rule: average the
    // linearly-interpolated crossing point of every one of the cell's 12
    // edges where the sign changes. FIXED nested loop order (ck, cj, ci),
    // so vertex creation order -- hence every later index into `out` -- is a
    // pure function of the sampled field, never of anything unordered.
    for (uint32_t ck = 0; ck < cells; ++ck) {
        for (uint32_t cj = 0; cj < cells; ++cj) {
            for (uint32_t ci = 0; ci < cells; ++ci) {
                float v[8];
                bool any_inside = false, any_outside = false;
                for (uint32_t c = 0; c < 8; ++c) {
                    const uint32_t i = ci + (c & 1u);
                    const uint32_t j = cj + ((c >> 1u) & 1u);
                    const uint32_t k = ck + ((c >> 2u) & 1u);
                    v[c] = field[field_index(i, j, k)];
                    if (inside(v[c])) {
                        any_inside = true;
                    } else {
                        any_outside = true;
                    }
                }
                if (!(any_inside && any_outside)) {
                    continue;  // not active: the surface does not pass through this cell.
                }

                glm::vec3 sum(0.0f);
                uint32_t count = 0;
                for (const std::array<uint32_t, 2>& e : kCubeEdges) {
                    const float va = v[e[0]];
                    const float vb = v[e[1]];
                    if (inside(va) == inside(vb)) {
                        continue;
                    }
                    const uint32_t ia = ci + (e[0] & 1u), ja = cj + ((e[0] >> 1u) & 1u), ka = ck + ((e[0] >> 2u) & 1u);
                    const uint32_t ib = ci + (e[1] & 1u), jb = cj + ((e[1] >> 1u) & 1u), kb = ck + ((e[1] >> 2u) & 1u);
                    const float t = std::clamp(va / (va - vb), 0.0f, 1.0f);
                    sum += glm::mix(corner_pos(ia, ja, ka), corner_pos(ib, jb, kb), t);
                    ++count;
                }
                // Guaranteed count >= 1: any_inside && any_outside means two
                // of the cube's 8 corners disagree in sign, and the cube's
                // 12-edge graph is connected, so some edge along a path
                // between them must itself cross the sign boundary.
                assert(count >= 1);
                const glm::vec3 position = sum / static_cast<float>(count);

                const glm::vec3 grad = gradient(subtree, position);
                const float glen = glm::length(grad);
                // A zero gradient is only possible at a genuine field
                // singularity (sdf.cpp: exactly a sphere's own centre, or an
                // analogous degenerate point) -- vanishingly unlikely ON the
                // interpolated surface itself. +Y is an arbitrary but finite
                // fallback, matching sdf.cpp's own sphere-at-centre
                // fallback's spirit (a defined answer, not a NaN).
                const glm::vec3 normal = glen > 0.0f ? grad / glen : glm::vec3(0.0f, 1.0f, 0.0f);

                cell_vertex[cell_index(ci, cj, ck)] = out.add_vertex(position, normal);
            }
        }
    }

    // --- Pass 2: one quad per INTERIOR grid edge whose two endpoints
    // disagree in sign, connecting the (always exactly 4, always already
    // active) neighbouring cells' vertices. Three axis-aligned scans, each
    // in a FIXED nested loop order, together covering every interior edge of
    // the grid exactly once.

    // X-direction edges: corner (i,j,k) -- (i+1,j,k). The 4 cells sharing it
    // are the ones straddling it in (Y, Z); it has 4 neighbours only when
    // j, k are themselves interior grid-corner indices (not the first or
    // last), which is exactly j, k in [1, cells-1].
    for (uint32_t k = 1; k < cells; ++k) {
        for (uint32_t j = 1; j < cells; ++j) {
            for (uint32_t i = 0; i < cells; ++i) {
                const float va = field[field_index(i, j, k)];
                const float vb = field[field_index(i + 1, j, k)];
                if (inside(va) == inside(vb)) {
                    continue;
                }
                const uint32_t v00 = cell_vertex[cell_index(i, j - 1, k - 1)];
                const uint32_t v10 = cell_vertex[cell_index(i, j, k - 1)];
                const uint32_t v11 = cell_vertex[cell_index(i, j, k)];
                const uint32_t v01 = cell_vertex[cell_index(i, j - 1, k)];
                emit_quad(out, v00, v10, v11, v01, inside(va) != kFlipX);
            }
        }
    }

    // Y-direction edges: corner (i,j,k) -- (i,j+1,k). Straddling cells are
    // in (X, Z); interior when i, k in [1, cells-1].
    for (uint32_t k = 1; k < cells; ++k) {
        for (uint32_t i = 1; i < cells; ++i) {
            for (uint32_t j = 0; j < cells; ++j) {
                const float va = field[field_index(i, j, k)];
                const float vb = field[field_index(i, j + 1, k)];
                if (inside(va) == inside(vb)) {
                    continue;
                }
                const uint32_t v00 = cell_vertex[cell_index(i - 1, j, k - 1)];
                const uint32_t v10 = cell_vertex[cell_index(i, j, k - 1)];
                const uint32_t v11 = cell_vertex[cell_index(i, j, k)];
                const uint32_t v01 = cell_vertex[cell_index(i - 1, j, k)];
                emit_quad(out, v00, v10, v11, v01, inside(va) != kFlipY);
            }
        }
    }

    // Z-direction edges: corner (i,j,k) -- (i,j,k+1). Straddling cells are
    // in (X, Y); interior when i, j in [1, cells-1].
    for (uint32_t j = 1; j < cells; ++j) {
        for (uint32_t i = 1; i < cells; ++i) {
            for (uint32_t k = 0; k < cells; ++k) {
                const float va = field[field_index(i, j, k)];
                const float vb = field[field_index(i, j, k + 1)];
                if (inside(va) == inside(vb)) {
                    continue;
                }
                const uint32_t v00 = cell_vertex[cell_index(i - 1, j - 1, k)];
                const uint32_t v10 = cell_vertex[cell_index(i, j - 1, k)];
                const uint32_t v11 = cell_vertex[cell_index(i, j, k)];
                const uint32_t v01 = cell_vertex[cell_index(i - 1, j, k)];
                emit_quad(out, v00, v10, v11, v01, inside(va) != kFlipZ);
            }
        }
    }

    // Submesh arrays stay empty (SR-11): single implicit submesh at
    // material 0, exactly tessellate_primitive()'s own contract.
    return std::move(out.mesh);
}

}  // namespace spade::render
