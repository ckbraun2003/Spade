#include "render/scene.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include <glm/gtc/matrix_inverse.hpp>  // glm::inverse(mat4) -- see local_to_world_of() below

#include "render/csg_mesh.hpp"    // split_program, csg_subtree_world_bounds, mesh_csg_subtree (Task R5)
#include "render/shadow.hpp"      // build_static_shadow_map (Task R7)
#include "render/tessellate.hpp"  // tessellate_primitive (Task R2)

namespace spade::render {
namespace {

// ---------------------------------------------------------------------------
// local_to_world_of -- SdfTransform::world_to_local is stored PRE-INVERTED
// (world/sdf.hpp), so recovering the forward, local-to-world matrix a draw
// item needs is one glm::inverse() call. This is dronesim/spade/raster.cpp's
// own localToWorldOf(), verbatim (S7a Task R1 brief, SR-9): float, not
// double -- world_to_local is already fp32, so inverting in double would
// manufacture precision the source data never had.
// ---------------------------------------------------------------------------
[[nodiscard]] glm::mat4 local_to_world_of(const SdfTransform& xf) {
    return glm::inverse(xf.world_to_local);
}

// ---------------------------------------------------------------------------
// ground_plane_y / world_bounds_of -- PORTED, not re-derived, from
// dronesim/spade/raster.cpp's groundPlaneY()/computeWorldBounds() (~lines
// 250-330; task-R1-brief.md Step 2): same heuristics, same constants, same
// default-box rule (PA-5). The only changes from the source are cosmetic --
// snake_case names to match this module's convention, and the local Aabb
// type renamed to WorldBoundsD to avoid colliding with render::Aabb
// (target.hpp, Task 0), which this file also uses. No heuristic was
// re-derived; see that file for the full rationale behind each constant.
// ---------------------------------------------------------------------------

// Best-effort ground-plane height -- true only for an IDENTITY-transform,
// +Y-normal plane node; not a general SDF evaluator. See raster.cpp's own
// groundPlaneY() doc comment for the full rationale.
[[nodiscard]] std::optional<float> ground_plane_y(const WorldDesc& world) {
    for (const SdfNode& node : world.sdf.nodes) {
        if (node.op != static_cast<uint32_t>(SdfOp::none)) {
            continue;
        }
        if (node.kind != static_cast<uint32_t>(SdfPrim::plane)) {
            continue;
        }
        if (node.transform != 0) {
            continue;  // only the identity transform (index 0) is handled.
        }
        const float nx = node.params.x;
        const float ny = node.params.y;
        const float nz = node.params.z;
        const float offset = node.params.w;
        constexpr float kUpEpsilon = 1e-3f;
        if (std::fabs(ny - 1.0f) < kUpEpsilon && std::fabs(nx) < kUpEpsilon &&
            std::fabs(nz) < kUpEpsilon) {
            // plane: dot(p, n) <= offset, n = (0,1,0) => p.y <= offset.
            return offset;
        }
    }
    return std::nullopt;
}

struct WorldBoundsD {
    double lo[3];
    double hi[3];
};

constexpr double kDefaultBoundsHalfExtent = 5.0;
constexpr double kBoundsMargin = 2.0;
constexpr double kMinBoundsSize = 2.0;

// Not a schema field -- derived here from whatever spatial data the world
// actually has, same algorithm as raster.cpp's computeWorldBounds: expand
// over every spawn position and every non-op SDF node's world-space
// transform ORIGIN (not its true geometric extent -- a coarse heuristic
// carried over unchanged), then pad with a fixed margin, then clamp each
// axis to a minimum size. A world with neither spawns nor SDF nodes falls
// back to a fixed default box.
[[nodiscard]] WorldBoundsD world_bounds_of(const WorldDesc& world) {
    bool any = false;
    WorldBoundsD box{};
    const auto expand = [&](const glm::vec3& p) {
        if (!any) {
            box.lo[0] = box.hi[0] = static_cast<double>(p.x);
            box.lo[1] = box.hi[1] = static_cast<double>(p.y);
            box.lo[2] = box.hi[2] = static_cast<double>(p.z);
            any = true;
            return;
        }
        box.lo[0] = std::min(box.lo[0], static_cast<double>(p.x));
        box.hi[0] = std::max(box.hi[0], static_cast<double>(p.x));
        box.lo[1] = std::min(box.lo[1], static_cast<double>(p.y));
        box.hi[1] = std::max(box.hi[1], static_cast<double>(p.y));
        box.lo[2] = std::min(box.lo[2], static_cast<double>(p.z));
        box.hi[2] = std::max(box.hi[2], static_cast<double>(p.z));
    };
    for (const SpawnPoint& sp : world.spawns) {
        expand(sp.position);
    }
    for (const SdfNode& node : world.sdf.nodes) {
        if (node.op != static_cast<uint32_t>(SdfOp::none)) {
            continue;
        }
        if (node.transform >= world.sdf.transforms.size()) {
            continue;  // defensive -- SdfProgram::validate() already guarantees this.
        }
        const glm::mat4 local_to_world = local_to_world_of(world.sdf.transforms[node.transform]);
        const glm::vec4 origin_world = local_to_world * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
        expand(glm::vec3(origin_world));
    }
    if (!any) {
        for (int i = 0; i < 3; ++i) {
            box.lo[i] = -kDefaultBoundsHalfExtent;
            box.hi[i] = kDefaultBoundsHalfExtent;
        }
        return box;
    }
    for (int i = 0; i < 3; ++i) {
        box.lo[i] -= kBoundsMargin;
        box.hi[i] += kBoundsMargin;
        if (box.hi[i] - box.lo[i] < kMinBoundsSize) {
            const double c = (box.hi[i] + box.lo[i]) * 0.5;
            box.lo[i] = c - kMinBoundsSize * 0.5;
            box.hi[i] = c + kMinBoundsSize * 0.5;
        }
    }
    return box;
}

[[nodiscard]] Aabb to_render_aabb(const WorldBoundsD& box) {
    return Aabb{
        .min = glm::vec3(static_cast<float>(box.lo[0]), static_cast<float>(box.lo[1]),
                          static_cast<float>(box.lo[2])),
        .max = glm::vec3(static_cast<float>(box.hi[0]), static_cast<float>(box.hi[1]),
                          static_cast<float>(box.hi[2])),
    };
}

}  // namespace

Result<RenderScene> scene_from_world(const WorldDesc& world,
                                      std::span<const NamedMesh> resolved_meshes) {
    RenderScene scene;

    // Mesh index space (scene.hpp's own note): resolved_meshes copies in FIRST,
    // verbatim and in span order, so a caller's own index into that vector is
    // numerically identical to the matching RenderScene::meshes index. The
    // SDF-derived meshes (below) follow, one per static DrawItem --
    // world.sdf.nodes.size() is a safe upper bound on that count (never
    // exact after R5: a CSG root's whole subtree collapses into one slot),
    // so this reserve() may over-allocate slightly but never under-allocates.
    scene.meshes.reserve(resolved_meshes.size() + world.sdf.nodes.size());
    for (const NamedMesh& named : resolved_meshes) {
        scene.meshes.push_back(named.mesh);
    }

    // Materials/lighting (schema v2, S7a Task R6): scene.materials/
    // scene.lighting now CONSUME world.materials/world.lighting instead of
    // this module's own hardcoded struct-literal defaults -- render::
    // Material/Lighting mirror world::MaterialDesc/LightingDesc field for
    // field, so the conversion is a straight per-entry copy.
    // validate_world_desc() guarantees world.materials is never empty for a
    // validated WorldDesc (this function's own precondition, builder.hpp),
    // so scene.materials is never empty either.
    scene.materials.reserve(world.materials.size());
    for (const MaterialDesc& m : world.materials) {
        scene.materials.push_back(Material{
            .base_color = m.base_color,
            .shading = static_cast<uint32_t>(m.shading),
        });
    }

    // sun_direction need not be pre-normalized (LightingDesc's own doc
    // comment, builder.hpp) -- validate_world_desc() guarantees it is
    // non-zero, so normalizing it is always well-defined (never a 0/0 NaN).
    // Normalized HERE, once, rather than at every shading evaluation,
    // matching Lighting's own doc comment above ("normalised, world space").
    scene.lighting = Lighting{
        .sun_direction = glm::normalize(world.lighting.sun_direction),
        .sun_color = world.lighting.sun_color,
        .sun_intensity = world.lighting.sun_intensity,
        .ambient_color = world.lighting.ambient_color,
        .sky_zenith = world.lighting.sky_zenith,
        .sky_horizon = world.lighting.sky_horizon,
    };

    scene.bounds = to_render_aabb(world_bounds_of(world));

    // SR-9 (REVISED at Task R5 -- see this file's own header comment): one
    // DrawItem per split_program() output entry, each carrying REAL geometry
    // -- tessellate_primitive() (R2) for a union-primitive leaf,
    // mesh_csg_subtree() (R5) for a whole CSG root's subtree. The two output
    // lists are merged back into ascending node-index order (a simple
    // 2-pointer merge -- both lists are already ascending, split_program()'s
    // own ORDER guarantee) so `statics`' own order matches the program's
    // left-to-right authoring order regardless of which population a given
    // piece of geometry landed in.
    const Result<SubtreeSplit> split = split_program(world.sdf);
    if (!split) {
        return std::unexpected(split.error());
    }
    const std::vector<uint32_t>& union_primitives = split->union_primitive_nodes;
    const std::vector<uint32_t>& csg_roots = split->csg_roots;

    scene.statics.reserve(union_primitives.size() + csg_roots.size() + world.props.size());

    const auto local_to_world_for = [&](const SdfNode& node) {
        // Defensive fallback to identity for an out-of-range transform index --
        // SdfProgram::validate() already guarantees this cannot happen for a
        // validated WorldDesc (this function's own precondition), matching
        // world_bounds_of()'s identical defensive skip above.
        return node.transform < world.sdf.transforms.size() ? local_to_world_of(world.sdf.transforms[node.transform])
                                                              : glm::mat4(1.0f);
    };

    // node_materials (PA-2, world/sdf.hpp): empty means every node uses
    // materials[0] (kNoMaterial -- draw_mesh_item's own submesh-material
    // fallback resolves that to 0, per SR-11); otherwise one entry per node,
    // already range-checked against world.materials by validate_world_desc().
    const auto node_material_override = [&](uint32_t node_index) {
        return world.sdf.node_materials.empty() ? kNoMaterial : world.sdf.node_materials[node_index];
    };

    size_t next_primitive = 0, next_csg_root = 0;
    while (next_primitive < union_primitives.size() || next_csg_root < csg_roots.size()) {
        const bool take_primitive =
            next_csg_root >= csg_roots.size() ||
            (next_primitive < union_primitives.size() && union_primitives[next_primitive] < csg_roots[next_csg_root]);

        if (take_primitive) {
            const uint32_t node_index = union_primitives[next_primitive++];
            const SdfNode& node = world.sdf.nodes[node_index];
            Result<MeshData> mesh = tessellate_primitive(static_cast<SdfPrim>(node.kind), node.params, scene.bounds);
            if (!mesh) {
                return std::unexpected(mesh.error());
            }
            const uint32_t mesh_index = static_cast<uint32_t>(scene.meshes.size());
            scene.meshes.push_back(std::move(*mesh));
            const glm::mat4 local_to_world = local_to_world_for(node);
            const uint32_t material_override = node_material_override(node_index);
            scene.statics.push_back(DrawItem{
                .mesh_index = mesh_index,
                .local_to_world = local_to_world,
                .material_override = material_override,
            });

            // SR-17 (S7a Task R6): a standalone plane primitive -- this
            // branch only ever sees union-primitive LEAVES (split_program()'s
            // own contract, csg_mesh.hpp), never one buried inside a
            // subtract/intersect/smooth_union subtree -- becomes an infinite
            // analytic ground candidate for the background pass, ALONGSIDE
            // its own bounded tessellated grid (tessellate_plane, above),
            // never instead of it: the grid still draws real, depth-tested
            // geometry unchanged; the analytic entry only ever shows through
            // on a background pixel no drawn triangle already covered.
            if (static_cast<SdfPrim>(node.kind) == SdfPrim::plane) {
                // n_local: the SAME expression tessellate_plane (tessellate.cpp)
                // computes from this identical node.params, so every one of
                // that mesh's per-vertex normals is bit-identical to this
                // local normal -- required for the seam's byte-identity
                // (transform_normal() then applies the SAME local_to_world
                // computed above, so the two paths' WORLD normal matches too).
                const glm::vec3 n_local = glm::normalize(glm::vec3(node.params));
                const glm::vec3 n_world = transform_normal(local_to_world, n_local);
                // A point known to lie ON the local plane (dot(p,n_local) ==
                // offset_local, since n_local is unit) carried through the
                // SAME local_to_world to WORLD space, from which the plane's
                // world-space offset (dot(p_world, n_world)) is re-derived --
                // general for any transform (SR-17's "no Y-up special case"),
                // not merely the identity-transform heuristic
                // ground_plane_y() (below) uses for the unrelated overlay grid.
                const glm::vec3 p_local = n_local * node.params.w;
                const glm::vec3 p_world = glm::vec3(local_to_world * glm::vec4(p_local, 1.0f));
                scene.ground_planes.push_back(GroundPlane{
                    .normal = n_world,
                    .offset = glm::dot(p_world, n_world),
                    // Same resolution rule as draw_mesh_item's own submesh
                    // fallback: tessellate_plane's mesh has empty submesh
                    // arrays (SR-11), so its one implicit submesh is always
                    // material 0 unless overridden.
                    .material = material_override != kNoMaterial ? material_override : 0u,
                });
            }
        } else {
            const uint32_t root_node = csg_roots[next_csg_root++];
            const Result<Aabb> subtree_bounds = csg_subtree_world_bounds(world.sdf, root_node, scene.bounds);
            if (!subtree_bounds) {
                return std::unexpected(subtree_bounds.error());
            }
            Result<MeshData> mesh = mesh_csg_subtree(world.sdf, root_node, *subtree_bounds);
            if (!mesh) {
                return std::unexpected(mesh.error());
            }
            const uint32_t mesh_index = static_cast<uint32_t>(scene.meshes.size());
            scene.meshes.push_back(std::move(*mesh));
            scene.statics.push_back(DrawItem{
                .mesh_index = mesh_index,
                // Identity, WRITTEN explicitly rather than read back via
                // local_to_world_for(world.sdf.nodes[root_node]) (review
                // IMPORTANT #5): mesh_csg_subtree() always emits WORLD-space
                // geometry (csg_mesh.hpp's own doc comment), so this item's
                // transform MUST be the identity regardless of what
                // transforms[0] happens to hold. Reading it back through
                // node.transform (always 0 on an operator node) only works
                // by leaning on "transforms[0] is the identity" being TRUE
                // -- a convention sdf.hpp documents but SdfProgram::
                // validate() never enforces, so a hand-built or
                // hand-edited-file program with a non-identity slot 0 would
                // silently double-transform every CSG mesh while every
                // primitive item stayed correct. Writing the identity here
                // has no such dependency.
                .local_to_world = glm::mat4(1.0f),
                // An operator node's own node_materials entry (WorldBuilder::
                // material_for_last_node()'s target when called right after
                // an operator adder) -- this consumer shades by CSG root, not
                // by leaf primitive, so it is exactly the entry this mesh's
                // single implicit submesh (SR-11: a CSG mesh is always
                // single-material) should use.
                .material_override = node_material_override(root_node),
            });
        }
    }

    // Props (schema v2, S7a Task R6 Step 3): one static DrawItem per
    // world.props entry, in authoring order, appended after every SDF-
    // derived static item -- a prop is never part of the SDF program, so it
    // has no node index to merge into that ordering, and ordering among
    // props themselves has no other consumer that cares.
    //
    // `mesh_ref` resolution mirrors NamedMesh's own doc comment (scene.hpp):
    // resolving a world reference to a file is the CALLER's job, so this
    // function only ever LOOKS UP a match already present in
    // `resolved_meshes` (linear scan -- `resolved_meshes` is typically a
    // handful of entries, one per distinct visual reference a world uses,
    // not per prop instance) and falls back to kNoMesh -- draw_mesh_item's
    // own existing "nothing to draw" skip -- when the caller never resolved
    // that reference, rather than treating an unresolved prop as an error
    // here (resolving every prop is the caller's responsibility, not this
    // builder's, the same posture visual_refs' own "opaque to the engine"
    // framing takes).
    for (const PropDesc& prop : world.props) {
        uint32_t mesh_index = kNoMesh;
        for (size_t i = 0; i < resolved_meshes.size(); ++i) {
            if (resolved_meshes[i].ref == prop.mesh_ref) {
                mesh_index = static_cast<uint32_t>(i);
                break;
            }
        }
        // Rigid + uniform scale, same T*R*S(s) construction WorldBuilder::
        // add_transform() uses for the (pre-inverted) SDF-node direction:
        // rotation matrix scaled by `scale` in its three column vectors,
        // translation written into the last column.
        glm::mat4 local_to_world = glm::mat4_cast(prop.pose.rotation);
        local_to_world[0] *= prop.pose.scale;
        local_to_world[1] *= prop.pose.scale;
        local_to_world[2] *= prop.pose.scale;
        local_to_world[3] = glm::vec4(prop.pose.position, 1.0f);
        scene.statics.push_back(DrawItem{
            .mesh_index = mesh_index,
            .local_to_world = local_to_world,
            // Always a concrete index, never kNoMaterial: validate_world_desc()
            // guarantees prop.material < world.materials.size(), and a prop
            // (unlike an SDF node) has no "use the mesh's own submesh
            // material" fallback to defer to -- it always names one.
            .material_override = prop.material,
        });
    }

    scene.spawn_positions.reserve(world.spawns.size());
    scene.spawn_orientations.reserve(world.spawns.size());
    for (const SpawnPoint& sp : world.spawns) {
        scene.spawn_positions.push_back(sp.position);
        scene.spawn_orientations.push_back(sp.orientation);
    }

    scene.sdf = &world.sdf;

    const std::optional<float> ground = ground_plane_y(world);
    scene.has_ground = ground.has_value();
    scene.ground_y = ground.value_or(0.0f);

    // S7a Task R7 (this function's own interface note, scene.hpp): a shadow
    // map is only meaningful when there is real static geometry the sun
    // could cast a shadow FROM -- an empty world (`statics` empty, since
    // props are appended into it above) leaves `static_shadow` at its
    // std::nullopt default rather than building a map with nothing ever
    // rasterised into it.
    if (!scene.statics.empty()) {
        const Result<ShadowMap> shadow_map = build_static_shadow_map(scene);
        if (!shadow_map) {
            return std::unexpected(shadow_map.error());
        }
        scene.static_shadow = *shadow_map;
    }

    return scene;
}

void update_dynamics(RenderScene& scene, std::span<const BodyPose> bodies) {
    // Rebuilt from scratch every call -- the new content depends only on
    // `bodies`, never on whatever `dynamics` held before (purity, scene.hpp's
    // own note). `statics`/`meshes`/everything else in `scene` is untouched.
    scene.dynamics.clear();
    scene.dynamics.reserve(bodies.size());
    for (const BodyPose& body : bodies) {
        // Rigid pose, no scale: local_to_world's rotation part is R (the
        // orientation quaternion's matrix), and its translation column is the
        // body position -- same T*R construction WorldBuilder::add_transform()
        // uses for the inverse direction (world/builder.cpp).
        glm::mat4 local_to_world = glm::mat4_cast(body.orientation);
        local_to_world[3] = glm::vec4(body.position, 1.0f);

        scene.dynamics.push_back(DrawItem{
            .mesh_index = body.mesh_index,
            .local_to_world = local_to_world,
            .material_override = kNoMaterial,
        });
    }
}

}  // namespace spade::render
