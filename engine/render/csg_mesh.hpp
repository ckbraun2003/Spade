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
//
// The grid is sized from a WORLD-SPACE cell, so a large subtree is not meshed
// more coarsely than a small one (a fixed 48 cells per subtree made an 8 m
// shell's cells 0.17 m). A subtree gets max(min_cells_per_axis,
// ceil(longest padded extent / cell_size)) cells per axis, capped at
// max_cells_per_axis. Small subtrees keep the old 48.
struct CsgMeshLimits {
    float cell_size = 0.05f;           // metres along the longest axis
    uint32_t min_cells_per_axis = 48;  // the old fixed count: small subtrees lose no detail
    uint32_t max_cells_per_axis = 160; // bounds load time and memory for a large subtree
    float aabb_margin = 0.05f;
};
inline constexpr CsgMeshLimits kCsgMeshDefaults{};

// Cells per axis for a subtree with these bounds (before the margin), by the
// rule above. 0 when the limits are invalid. It pads by aabb_margin alone; a
// smooth_union root pads by k/4 when that is larger (csg_subtree_sample_box()),
// which csg_mesh_cell_size() below takes into account.
[[nodiscard]] uint32_t csg_cells_per_axis(const Aabb& subtree_bounds, const CsgMeshLimits& limits);

// Folded triangles: their geometric normal points against the subtree's SDF
// gradient at the centroid. Surface nets folds a wall thinner than about two
// cells, because both faces share one cell's vertex, and a folded triangle is
// culled as a back face. It also folds a few triangles at a sharp CSG edge,
// where the gradient flips, however thick the solid. Each fold is judged by
// the solid's thickness inward from it: under two cells of `cell` (the mesh's
// own, csg_mesh_cell_size()) is a thin wall, anything else a sharp edge.
// scene_from_world() names each kind in a warning of its own (L6).
struct CsgFoldReport {
    uint32_t folded = 0;     // triangles that fold: thin + sharp
    uint32_t triangles = 0;  // all triangles judged (degenerate ones are skipped)
    Aabb bounds{};           // world-space box around the folded triangles; empty when none fold
    uint32_t thin = 0;       // folds on a wall thinner than two cells
    Aabb thin_bounds{};
    uint32_t sharp = 0;      // folds at a sharp edge of thicker solid
    Aabb sharp_bounds{};
};
[[nodiscard]] CsgFoldReport find_folded_triangles(const SdfProgram& program, uint32_t root_node,
                                                  const MeshData& mesh, float cell);

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

// The subtree ending at `root_node` as a program of its own.
// eval() and gradient() on it see that subtree alone, never the rest of the
// world's geometry. `transforms` is copied whole, so the nodes' transform
// indices still resolve. Precondition: a validated `program` and an
// in-range `root_node` (split_program()'s csg_roots qualify).
[[nodiscard]] SdfProgram csg_subtree_program(const SdfProgram& program, uint32_t root_node);

// The box mesh_csg_subtree() samples, and the box a ray-march clips to.
// It is `subtree_bounds` padded by limits.aabb_margin, or by k/4 when
// `root_node` is a smooth_union with a larger k/4. The subtree's surface
// lies inside it.
[[nodiscard]] Aabb csg_subtree_sample_box(const SdfProgram& program, uint32_t root_node,
                                          const Aabb& subtree_bounds,
                                          const CsgMeshLimits& limits = kCsgMeshDefaults);

// The longest edge of one cell of the grid mesh_csg_subtree() samples: the
// sample box's longest axis over its cells per axis, as the mesh computes
// them. 0 when the limits are invalid. The fold warning names this figure.
[[nodiscard]] float csg_mesh_cell_size(const SdfProgram& program, uint32_t root_node, const Aabb& subtree_bounds,
                                       const CsgMeshLimits& limits = kCsgMeshDefaults);

// World-space AABB enclosing the CSG subtree rooted at `root_node` (as
// identified by split_program()'s csg_roots / consumed by mesh_csg_subtree()
// below).
//
// The extent is OPERATOR-AWARE, not a flat union of the subtree's leaves.
// Each operator narrows or widens what its operands contribute -- intersect
// takes the tighter operand, subtract keeps only the minuend, union takes
// both, smooth_union takes both dilated by k/4 (see subtree_extent()'s own
// comment in the .cpp for the per-operator rules and why the dilation is
// exactly k/4). This is what lets `(plane u sphere) n box` come back with the
// BOX's tight bound instead of an unbounded union or a world-sized fallback.
//
// `fallback_bounds` is a SUBTREE-level fallback, not a per-primitive one: it
// is returned only when the whole subtree is still unbounded after every
// operator in it has had its say -- e.g. a subtract whose minuend is itself a
// plane. An unbounded leaf (plane, heightfield -- PA-5, tessellate.hpp's
// identical exemption for the same two kinds) sitting under an operator that
// bounds it does NOT reach here. CALLERS pass the world's own overall bounds
// (render/scene.cpp's world_bounds_of()), finite by construction.
//
// Errors: invalid_argument for `root_node` out of range, or an out-of-range
// SDF transform index (raised inside subtree_extent() and propagated). Kept
// as a Result rather than asserts because this function inspects
// caller-suppliable node indices.
//
// NOTE: there is deliberately NO "subtree with no primitive leaves" error.
// That case is not an error at all -- it yields no extent and therefore
// returns `fallback_bounds` by the rule above.
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
// Errors: invalid_argument for `root_node` out of range, limits for which
// csg_cells_per_axis() is 0, or anything the internal re-validation of the
// extracted subtree program rejects (unreachable for a subtree of an
// already-validated `program`, kept as a defensive Result for the same
// reason csg_subtree_world_bounds() above is one).
[[nodiscard]] Result<MeshData> mesh_csg_subtree(const SdfProgram& program, uint32_t root_node,
                                                 const Aabb& subtree_bounds,
                                                 const CsgMeshLimits& limits = kCsgMeshDefaults);

}  // namespace spade::render
