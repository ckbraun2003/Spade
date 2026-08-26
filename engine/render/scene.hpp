#pragma once

// ---------------------------------------------------------------------------
// RenderScene -- the renderer's own view of a world (S7a Task R1). Built once
// per world load from a spade::WorldDesc plus a set of already-resolved
// visual meshes; refreshed once per frame from a tick-boundary copy of body
// poses. This is the seam between "what the SDF program/physics know" and
// "what a draw pass needs" -- everything downstream (the rasterizer, a
// headless renderer, a training harness's frame dump) reads only this
// struct, never the WorldDesc or the live simulation state directly.
//
// PURITY (binding global constraint): scene_from_world() and
// update_dynamics() are PURE functions of their explicit inputs -- no
// simulation state, no RNG, no wall clock, no static mutable state anywhere
// in this module. update_dynamics() takes `RenderScene&` because it writes
// into an existing container (no per-frame reallocation of the whole scene),
// not because its OUTPUT depends on the scene's prior contents: `dynamics`
// is rebuilt from scratch from `bodies` on every call, so the result never
// depends on anything the scene held before the call.
//
// SR-9 (controller ruling, task-R1-brief.md; REVISED at Task R5): one
// DrawItem per entry of render/csg_mesh.hpp's split_program() -- one per
// primitive leaf that sits under nothing but `union` (Task R2's
// tessellate_primitive() fills its mesh), and one per CSG root
// (subtract/intersect/smooth_union -- Task R5's mesh_csg_subtree() fills
// its mesh, the WHOLE subtree collapsed into a single draw item). The two
// output lists are merged back into the program's own left-to-right
// authoring order (ascending node index) before becoming `statics`, so a
// world's draw order is reproducible from the program alone and does not
// depend on which of the two populations a given piece of geometry happens
// to fall into.
//
// This SUPERSEDES Task R1's original one-non-op-node-one-EMPTY-item mapping:
// a CSG root's own node IS an operator node (op != SdfOp::none), yet it now
// gets exactly one draw item, while the primitive leaves consumed into its
// subtree get none of their own -- the node/item correspondence is
// split_program()'s, not "is this node a primitive leaf".
//
// MESH INDEX SPACE (this task's own design decision, driven by
// task-H1-brief.md's plan for how the eventual host wires a body to its
// visual mesh): `resolved_meshes` -- geometry the CALLER already produced by
// resolving each of a world's visual references to a file (see NamedMesh's
// own comment) -- is copied into `RenderScene::meshes` VERBATIM, IN SPAN
// ORDER, at the FRONT (indices [0, resolved_meshes.size())). The SDF-node-
// derived (currently empty) placeholder meshes follow immediately after, one
// per static DrawItem, in program order. This keeps a caller's own index
// into the `resolved_meshes` vector it built (e.g. "which loaded mesh does
// this body's model type reference") numerically identical to the matching
// index into `RenderScene::meshes` -- BodyPose::mesh_index is exactly that
// index, with no renumbering owed by either side of the seam.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/error.hpp"
#include "render/shadow.hpp"   // ShadowMap (S7a Task R7) -- forward-declares RenderScene itself, no cycle
#include "render/target.hpp"   // Aabb, kNoMaterial, kNoMesh (Task 0) -- do not redeclare
#include "world/builder.hpp"   // WorldDesc
#include "world/sdf.hpp"       // SdfProgram

namespace spade::render {

// Transforms a LOCAL unit surface normal to WORLD space and re-normalizes
// (S7a Task R6). For a rigid + UNIFORM-scale local_to_world (SdfTransform's
// own contract, world/sdf.hpp), mat3(local_to_world) applied to a local
// direction yields that direction correctly ROTATED and then scaled by the
// uniform factor (the same relationship world/sdf.hpp's own gradient_world
// derivation states for the SDF gradient) -- normalizing removes that scale,
// leaving exactly the rotated normal.
//
// Used IDENTICALLY by two call sites that must agree bit-for-bit (SR-17's
// load-bearing seam): the tessellated-mesh per-vertex normal transform
// (raster_cpu.cpp's draw_mesh_item, shading a real triangle) and the
// analytic background ground plane's precomputed world normal
// (scene_from_world()'s ground-plane extraction, scene.cpp) -- for a
// standalone ground-plane node, both call this SAME function with the SAME
// local_to_world and the SAME local normal, so the two paths' shading
// agrees exactly, not merely approximately.
[[nodiscard]] inline glm::vec3 transform_normal(const glm::mat4& local_to_world, const glm::vec3& local_normal) {
    return glm::normalize(glm::mat3(local_to_world) * local_normal);
}

struct Material {
    glm::vec4 base_color{0.72f, 0.72f, 0.74f, 1.0f};
    uint32_t shading = 0;   // 0 = lambert, 1 = unlit, 2 = emissive
};

struct Lighting {
    glm::vec3 sun_direction{-0.35f, -0.86f, -0.37f};   // normalised, world space
    glm::vec3 sun_color{1.0f, 0.98f, 0.94f};
    float sun_intensity = 1.0f;
    glm::vec3 ambient_color{0.30f, 0.34f, 0.42f};
    glm::vec3 sky_zenith{0.28f, 0.42f, 0.62f}, sky_horizon{0.68f, 0.74f, 0.80f};
};

// One loaded or generated mesh. Positions/normals/indices stay empty for a
// slot this task allocated but did not fill (SR-9) -- R2/R5 fill the
// SDF-derived slots; a resolved visual reference (NamedMesh, below) arrives
// already filled by the caller.
//
// SUBMESH CONTRACT (controller ruling SR-11, settled at Task R2's review --
// binding on every producer AND consumer of this struct, not just R2's own
// tessellate_primitive()): an EMPTY submesh triple (all three arrays size 0,
// including a mesh whose positions/normals/indices are otherwise fully
// populated -- Task R2's tessellate_primitive() is exactly such a producer)
// means EXACTLY ONE IMPLICIT SUBMESH spanning the whole index buffer, with
// material index 0. Index 0 is always a valid default material: every
// RenderScene's `materials` carries a defaulted entry there
// (scene_from_world()'s own guarantee, scene.cpp). When the three arrays are
// NON-empty, they are PARALLEL (equal length) and their
// (submesh_first_index[i], submesh_index_count[i]) ranges PARTITION
// `indices` -- every index belongs to exactly one submesh, in submesh order.
// A consumer therefore never special-cases "no submeshes were declared" as a
// draw failure: it is the one-submesh-at-material-0 case, spelled with empty
// arrays instead of a redundant single-entry ones.
struct MeshData {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;              // per-vertex; flat meshes duplicate vertices
    std::vector<uint32_t>  indices;              // triangle list
    std::vector<uint32_t>  submesh_first_index;  // parallel; size = submesh count
    std::vector<uint32_t>  submesh_index_count;
    std::vector<uint32_t>  submesh_material;
};

// A world visual reference already resolved to its geometry. `ref` names the
// same string one of WorldDesc::visual_refs carries; resolving ref -> file ->
// MeshData is entirely the CALLER's job (glTF loading, asset lookup -- no
// kat- or file-format-specific vocabulary belongs in spade/).
struct NamedMesh {
    std::string ref;
    MeshData mesh;
};

// Aabb, kNoMaterial and kNoMesh come from target.hpp (Task 0) -- do not redeclare.

// An infinite analytic ground candidate (S7a Task R6, ruling SR-17): one per
// STANDALONE plane primitive -- a node split_program() (render/csg_mesh.hpp)
// classifies as a union-primitive leaf, never one buried inside a
// subtract/intersect/smooth_union subtree (that plane is a cutting
// half-space, not a floor). `normal`/`offset` are already WORLD-SPACE
// (scene_from_world() bakes the node's own SdfTransform in once, at scene-
// build time, via transform_normal() above) so the background pass
// (raster_cpu.cpp) never re-derives them per pixel: solid is
// dot(p, normal) <= offset, the SAME convention world/sdf.hpp's plane node
// uses, so `normal` points away from the solid (the side a camera normally
// stands on) exactly like a plane's outward mesh normal does. `material` is
// always a valid index into RenderScene::materials (resolved the same way
// DrawItem::material_override resolves for that node -- node_materials, or
// submesh material 0 when node_materials is empty).
struct GroundPlane {
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    float offset = 0.0f;
    uint32_t material = 0;
};

struct DrawItem {
    uint32_t mesh_index = 0;   // index into RenderScene::meshes
    glm::mat4 local_to_world{1.0f};
    uint32_t material_override = kNoMaterial;
};

// A tick-boundary copy of one body's pose (position/orientation only -- no
// scale; bodies are rigid). `mesh_index` is an index into RenderScene::meshes
// -- normally one of the `resolved_meshes` slots the caller resolved at
// world-load time (see the MESH INDEX SPACE note above), or kNoMesh for a
// body with nothing to draw.
struct BodyPose {
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    uint32_t mesh_index = kNoMesh;
};

struct RenderScene {
    std::vector<MeshData> meshes;
    std::vector<DrawItem> statics;    // built once at world load
    std::vector<DrawItem> dynamics;   // refreshed per frame from BodyPose
    std::vector<Material> materials;  // index 0 is always the default material
    Lighting               lighting;
    Aabb                   bounds;
    std::vector<glm::vec3> spawn_positions;   // overlay layer (PA-4)
    std::vector<glm::quat> spawn_orientations;
    const SdfProgram*      sdf = nullptr;     // non-owning; raymarch + agreement only
    float                  ground_y = 0.0f;
    bool                   has_ground = false;
    // Analytic background ground candidates (S7a Task R6, SR-17) -- empty for
    // a hand-built RenderScene (only scene_from_world() populates it) and
    // for a world with no standalone ground plane. Never consulted by
    // has_ground/ground_y's OLDER, unrelated purpose (the overlay grid's own
    // best-effort +Y-identity-plane heuristic, scene.cpp's ground_plane_y());
    // this is the raster background pass's own general, transform-aware list.
    std::vector<GroundPlane> ground_planes;
    // The sun's own shadow map (S7a Task R7), built ONCE from `statics` alone
    // at scene-build time and reused every frame -- raster_cpu.cpp's render()
    // never rebuilds this; it only re-rasterises `dynamics` into a per-frame
    // COPY of it (Step 3). std::nullopt for a hand-built RenderScene that
    // never calls build_static_shadow_map() (every existing fixture in this
    // corpus, until a test opts in) and, per scene_from_world()'s own
    // contract below, for a world with no static geometry at all -- either
    // way, RenderOptions::shadows has nothing to sample against and render()
    // treats every pixel as unshadowed, identically to shadows being off.
    std::optional<ShadowMap> static_shadow;
};

// Builds a RenderScene from a validated WorldDesc plus its already-resolved
// visual meshes.
//
// PRECONDITION: `world` has already passed validate_world_desc() -- true of
// anything WorldBuilder::build() or world_from_yaml() produced. This is a
// presentation-layer builder, not a second validator, and never fails on a
// well-formed WorldDesc; it returns Result<RenderScene> for the same reason
// validate_target() does (a stable seam for a future real failure mode),
// not because today's implementation has one.
//
// `sdf` in the result points at `world.sdf`: the returned RenderScene must
// not outlive `world`.
//
// `static_shadow` (S7a Task R7): populated via build_static_shadow_map()
// whenever `statics` ends up non-empty (props count too -- any real geometry
// the sun could cast a shadow FROM), left std::nullopt otherwise (an empty
// world has nothing to build a meaningful shadow map from).
[[nodiscard]] Result<RenderScene> scene_from_world(const WorldDesc& world,
                                                    std::span<const NamedMesh> resolved_meshes);

// Rebuilds `scene.dynamics` from `bodies`, one DrawItem per pose, in order.
// Never touches `scene.statics`, `scene.meshes`, or anything else in `scene`.
void update_dynamics(RenderScene& scene, std::span<const BodyPose> bodies);

}  // namespace spade::render
