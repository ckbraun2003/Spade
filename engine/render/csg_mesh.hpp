#pragma once

// ---------------------------------------------------------------------------
// CSG subtree meshing -- S7a Task R5, the last geometry task of Phase 1.
//
// Spade's static world is a flat postfix SDF program (world/sdf.hpp):
// primitives combined by `union`, `intersect`, `subtract` and `smooth_union`.
// Across the shipped world corpus the overwhelming majority of operators are
// `union`, and a union costs nothing to render: draw both operands with
// depth testing on and the depth buffer resolves the silhouette exactly,
// which is Task R2's tessellate_primitive() applied to each operand in turn
// (RS3a). The other three operators are GENUINE CSG: their silhouette is not
// the union of their operands' silhouettes, so no per-primitive draw-item
// split can render them correctly. A `box - box` gate is the canonical
// case -- the subtracted box IS the hole a body flies through, and rendering
// it as a union produces a solid block where the hole should be.
//
// split_program() (below) partitions a validated postfix program into the
// two populations: primitive leaves that sit under nothing but `union` all
// the way to the program's root (tessellate these individually, R2's job),
// and the roots of maximal subtrees rooted at a `subtract`/`intersect`/
// `smooth_union` node (mesh these as ONE unit, this file's job). A CSG
// root's subtree is meshed WHOLE, including any `union` nested inside
// it -- once a subtree needs real CSG at all, evaluating its SDF at a grid
// point already accounts for every operator inside it, nested unions
// included, so nothing is gained by splitting further within it.
//
// mesh_csg_subtree() meshes that subtree with SURFACE NETS: sample the
// subtree's OWN SDF (not the whole program's -- see csg_mesh.cpp for why)
// on a fixed, uniform, WORLD-SPACE grid over its AABB; one vertex per grid
// cell whose corners disagree in sign; one quad per interior grid edge whose
// endpoints disagree in sign. Fixed cell count, fixed margin, no adaptive
// refinement, no camera or resolution dependence (RS3 -- the identical
// discipline tessellate.hpp's kTessellationDefaults already documents), so a
// subtree's mesh is a pure function of `(subtree, limits)`: byte-identical
// across calls and across platforms.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <vector>

#include "core/error.hpp"
#include "render/scene.hpp"   // MeshData
#include "render/target.hpp"  // Aabb -- do not redeclare
#include "world/sdf.hpp"      // SdfProgram

namespace spade::render {

// Fixed constant table -- NEVER adaptive, NEVER camera- or resolution-
// dependent (see the file comment above and tessellate.hpp's identical rule
// for kTessellationDefaults).
struct CsgMeshLimits {
    uint32_t cells_per_axis = 48;
    float aabb_margin = 0.05f;
};
inline constexpr CsgMeshLimits kCsgMeshDefaults{};

// Splits a validated postfix program (PRECONDITION: program.validate() has
// succeeded -- matching world/sdf.hpp's own eval()/gradient()/sample()
// convention; this function does not re-validate) into the two draw-unit
// populations the file comment above describes.
//
// ORDER: both vectors are in the program's own left-to-right authoring
// order (a later-appearing node always has a strictly larger index than
// an earlier one in either vector) -- what lets a caller reconstruct the
// program's overall left-to-right draw order by merging the two lists on
// node index, rather than drawing every union-primitive before every CSG
// root regardless of how they were authored (render/scene.cpp does exactly
// this merge).
struct SubtreeSplit {
    std::vector<uint32_t> union_primitive_nodes;  // tessellate these individually
    std::vector<uint32_t> csg_roots;              // mesh these
};
[[nodiscard]] Result<SubtreeSplit> split_program(const SdfProgram& program);

// World-space AABB tightly enclosing the CSG subtree rooted at `root_node`
// (as identified by split_program()'s csg_roots / consumed by
// mesh_csg_subtree() below): the union, over every primitive leaf inside the
// subtree, of that primitive's own local shape extent transformed by its own
// SdfTransform.
//
// `fallback_bounds` stands in for a primitive whose own shape has no finite
// local extent (plane, heightfield -- PA-5, tessellate.hpp's identical
// exemption for the same two kinds): CALLERS pass the world's own overall
// bounds (render/scene.cpp's world_bounds_of()), which is finite by
// construction (world_bounds_of() always returns a real box, falling back to
// a fixed default one itself when the world has no other spatial data).
//
// Errors: invalid_argument for `root_node` out of range, an out-of-range
// transform index, or a subtree with no primitive leaves at all -- the last
// is unreachable for a genuine CSG root (subtract/intersect/smooth_union
// always has two operands, so at least one primitive sits under it), kept as
// a defensive Result rather than an assert because this function inspects
// caller-suppliable node indices.
[[nodiscard]] Result<Aabb> csg_subtree_world_bounds(const SdfProgram& program, uint32_t root_node,
                                                     const Aabb& fallback_bounds);

// Surface-nets meshing of ONE subtree of `program` rooted at `root_node`,
// evaluated over its AABB. Used only for subtract/intersect/smooth_union
// roots -- split_program() never places a `union` node in csg_roots, so a
// pure-union subtree never reaches this function; it is tessellated
// per-primitive instead (render/tessellate.hpp).
//
// Positions/normals are WORLD-SPACE, not subtree-local: a CSG root node is
// always an operator node, whose own `transform` field is forced to 0 (the
// identity transform) by SdfProgram::validate() -- so the DrawItem
// render/scene.cpp builds for this mesh always has an identity
// local_to_world, and the geometry has to already live in the space that
// matrix maps to. `subtree_bounds` is therefore expected in world space too
// (csg_subtree_world_bounds() above produces exactly that).
//
// Submesh arrays stay EMPTY (SR-11, the controller ruling in scene.hpp's
// MeshData comment): a CSG mesh is single-material, exactly
// tessellate_primitive()'s own producer contract.
//
// Errors: invalid_argument for `root_node` out of range, `limits.
// cells_per_axis == 0`, or anything the internal re-validation of the
// extracted subtree program rejects (unreachable for a subtree of an
// already-validated `program`, kept as a defensive Result for the same
// reason csg_subtree_world_bounds() above is one).
[[nodiscard]] Result<MeshData> mesh_csg_subtree(const SdfProgram& program, uint32_t root_node,
                                                 const Aabb& subtree_bounds,
                                                 const CsgMeshLimits& limits = kCsgMeshDefaults);

}  // namespace spade::render
