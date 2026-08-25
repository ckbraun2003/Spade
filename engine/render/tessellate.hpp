#pragma once

// ---------------------------------------------------------------------------
// Primitive tessellation -- S7a Task R2. Turns one analytic SDF primitive
// (world/sdf.hpp's SdfPrim) into real triangles, in the primitive's own LOCAL
// space (the caller applies the node's local_to_world -- see scene.hpp's
// DrawItem). Genuine CSG subtrees (subtract/intersect/smooth_union) are NOT
// this task's job: a union's silhouette is just both operands drawn with
// depth testing, which is correct and free, so tessellate_primitive() only
// ever sees a single primitive leaf. CSG subtree meshing is Task R5's.
//
// THE LIMITS TABLE IS A FIXED CONSTANT, NEVER ADAPTIVE (RS3). kTessellationDefaults
// is a compile-time constant, not a function of the camera, the render target's
// resolution, screen-space error, or anything else that could vary between two
// calls with the same primitive -- that is what makes the golden corpus
// (tests/golden/render/tessellation/) byte-exact and reproducible rather than a
// moving target. A caller MAY pass a different TessellationLimits (tests do,
// to exercise the closed-form vertex/index counts at other densities), but
// nothing in this module ever chooses one on its own.
//
// THE CAPSULE IS A REAL CAPSULE HERE: two hemispherical caps plus a
// cylindrical body, not a cylinder with flat ends. The existing kat-side
// wireframe rasterizer draws a capsule as a plain cylinder -- a deliberate
// shortcut for a wireframe overlay, where nobody is close enough to notice
// the caps are wrong. That shortcut is EXPLICITLY REVERSED here: a shaded
// triangle mesh with flat caps would show a visible seam and a wrong silhouette
// at the ends, so tessellate_primitive(capsule) always emits hemispheres.
// Nobody should reinstate the cylinder shortcut in this module.
//
// PLANE (AND HEIGHTFIELD) ARE ANALYTICALLY INFINITE AND MUST NOT BE DRAWN THAT
// WAY (PA-5). Both primitives' distance fields are defined over the whole
// (x, z) plane with no inherent extent, so tessellate_primitive() fits a
// bounded grid to `world_bounds` instead -- the same Aabb every other draw
// item's bounds already come from (render/scene.cpp's world_bounds_of()) --
// so the result is reproducible from the world's own data rather than from
// whatever the camera happens to be looking at.
//
// PRECONDITION (matches world/sdf.hpp's own eval()/gradient() convention):
// `params` are the already-validated params of an SdfNode of kind `kind`
// (SdfProgram::validate() has run) -- this function does not re-validate
// parameter sanity (non-negative radii, a unit plane normal, and so on).
// ---------------------------------------------------------------------------

#include <cstdint>

#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "render/scene.hpp"   // MeshData
#include "render/target.hpp"  // Aabb -- do not redeclare
#include "world/sdf.hpp"      // SdfPrim

namespace spade::render {

// Fixed constant table -- NEVER adaptive, NEVER camera- or resolution-dependent
// (RS3, constraint 6 above).
struct TessellationLimits {
    uint32_t circle_segments = 24;  // cylinder/torus/capsule rings
    uint32_t sphere_rings = 12, sphere_segments = 24;
    uint32_t torus_ring_segments = 16;
    uint32_t heightfield_cells = 48;  // per axis
    uint32_t plane_grid_cells = 40;   // bounded quad fitted to world bounds (PA-5)
};
inline constexpr TessellationLimits kTessellationDefaults{};

// Generates triangles for one SDF primitive leaf, in the primitive's own
// local space (params packing: world/sdf.hpp's SdfNode doc comment).
//
// `world_bounds` is used ONLY by `plane` and `heightfield` -- the two
// primitives whose analytic field has no inherent extent (PA-5, above) --
// and is ignored by the other five, whose geometry is already bounded by
// their own parameters.
//
// ONLY positions/normals/indices ARE FILLED IN. This task is geometry
// generation, not scene wiring: submesh_first_index/submesh_index_count/
// submesh_material stay empty, exactly as scene_from_world() leaves them on
// the placeholder MeshData it allocates (scene.cpp's SR-9 comment) -- how a
// tessellated primitive's triangles map to a submesh/material is scene
// wiring's decision (Task R5), not a fact this function is in a position to
// invent.
//
// Errors: invalid_argument for a `kind` outside SdfPrim's seven values --
// unreachable for a kind that came from a validated SdfNode, exactly like
// world/sdf.hpp's own primitive_distance() switch.
[[nodiscard]] Result<MeshData> tessellate_primitive(SdfPrim kind, glm::vec4 params,
                                                     const Aabb& world_bounds,
                                                     const TessellationLimits& limits = kTessellationDefaults);

}  // namespace spade::render
