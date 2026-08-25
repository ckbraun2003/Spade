#include "render/scene.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include <glm/gtc/matrix_inverse.hpp>  // glm::inverse(mat4) -- see local_to_world_of() below

#include "render/csg_mesh.hpp"    // split_program, csg_subtree_world_bounds, mesh_csg_subtree (Task R5)
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

    scene.materials.push_back(Material{});  // index 0 is always the default material
    scene.lighting = Lighting{};

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

    scene.statics.reserve(union_primitives.size() + csg_roots.size());

    const auto local_to_world_for = [&](const SdfNode& node) {
        // Defensive fallback to identity for an out-of-range transform index --
        // SdfProgram::validate() already guarantees this cannot happen for a
        // validated WorldDesc (this function's own precondition), matching
        // world_bounds_of()'s identical defensive skip above.
        return node.transform < world.sdf.transforms.size() ? local_to_world_of(world.sdf.transforms[node.transform])
                                                              : glm::mat4(1.0f);
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
            scene.statics.push_back(DrawItem{
                .mesh_index = mesh_index,
                .local_to_world = local_to_world_for(node),
                .material_override = kNoMaterial,
            });
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
                .material_override = kNoMaterial,
            });
        }
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
