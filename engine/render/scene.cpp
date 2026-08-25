#include "render/scene.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include <glm/gtc/matrix_inverse.hpp>  // glm::inverse(mat4) -- see local_to_world_of() below

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
    // SDF-node placeholders (below) follow, one per static DrawItem.
    scene.meshes.reserve(resolved_meshes.size() + world.sdf.nodes.size());
    for (const NamedMesh& named : resolved_meshes) {
        scene.meshes.push_back(named.mesh);
    }

    // SR-9: one DrawItem per non-op SDF node, in program order. The mesh slot
    // it points at is an empty placeholder here -- R2 (tessellate_primitive)
    // and R5 (CSG subtree splitting) fill real geometry into scene_from_world
    // later; this task builds the container only.
    scene.statics.reserve(world.sdf.nodes.size());
    for (const SdfNode& node : world.sdf.nodes) {
        if (node.op != static_cast<uint32_t>(SdfOp::none)) {
            continue;
        }
        const uint32_t mesh_index = static_cast<uint32_t>(scene.meshes.size());
        scene.meshes.emplace_back();  // empty MeshData placeholder (R2/R5 fill it in)

        // Defensive fallback to identity for an out-of-range transform index --
        // SdfProgram::validate() already guarantees this cannot happen for a
        // validated WorldDesc (this function's own precondition), matching
        // world_bounds_of()'s identical defensive skip above.
        const glm::mat4 local_to_world = node.transform < world.sdf.transforms.size()
                                              ? local_to_world_of(world.sdf.transforms[node.transform])
                                              : glm::mat4(1.0f);

        scene.statics.push_back(DrawItem{
            .mesh_index = mesh_index,
            .local_to_world = local_to_world,
            .material_override = kNoMaterial,
        });
    }

    scene.materials.push_back(Material{});  // index 0 is always the default material
    scene.lighting = Lighting{};

    scene.bounds = to_render_aabb(world_bounds_of(world));

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
