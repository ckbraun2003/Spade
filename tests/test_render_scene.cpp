#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// RenderScene / scene_from_world / update_dynamics tests (S7a Task R1).
//
// SR-9 (controller ruling, task-R1-brief.md; REVISED at Task R5, see
// render/scene.hpp's own SR-9 comment): scene_from_world() emits one
// DrawItem per split_program() output entry -- one per union-primitive leaf
// (tessellate_primitive() fills its mesh) and one per CSG root
// (mesh_csg_subtree() fills its mesh, the whole subtree collapsed into a
// single item). These tests assert draw-item COUNT, TRANSFORM correctness,
// and (post-R5) that the mesh slot actually carries REAL geometry -- never
// a specific triangle count or shape, which is R2's/R5's own tests' job, not
// this seam's.
//
// Transform checks compare against an INDEPENDENTLY constructed forward
// matrix (translate(position) * scale * rotation) rather than re-deriving
// scene.cpp's own inversion -- SdfTransform::world_to_local is stored
// pre-inverted (world/sdf.hpp), so scene_from_world's contract IS "invert it
// back"; building the forward matrix straight from the authored pose and
// checking scene_from_world recovered it is an independent check, not a
// restatement of the code under test.
//
// The two ported helpers (world_bounds_of/ground_plane_y, scene.cpp) get
// their own dedicated cases below, each locking one heuristic from
// dronesim/spade/raster.cpp's computeWorldBounds()/groundPlaneY() exactly:
// the default box for a world with no spatial data, the margin+min-size rule
// otherwise, and the ground plane's identity-transform/+Y-normal-only match.
// ---------------------------------------------------------------------------

namespace {

using spade::Capacities;
using spade::SdfPose;
using spade::SpawnPoint;
using spade::WorldBuilder;
using spade::WorldDesc;
using spade::render::BodyPose;
using spade::render::DrawItem;
using spade::render::Material;
using spade::render::MeshData;
using spade::render::NamedMesh;
using spade::render::RenderScene;
using spade::render::scene_from_world;
using spade::render::update_dynamics;

constexpr float kTol = 1e-4f;

// A builder with the world-level requirements already satisfied, so each
// test only says what it is actually testing (mirrors test_sdf.cpp's own
// base_builder()).
WorldBuilder base_builder() {
    WorldBuilder b;
    b.name("render-scene-test")
        .capacities(Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    return b;
}

WorldDesc build_or_fail(const WorldBuilder& b) {
    const spade::Result<WorldDesc> world = b.build();
    if (!world) {
        ADD_FAILURE() << "builder failed: " << world.error().context;
        return WorldDesc{};
    }
    return *world;
}

RenderScene scene_or_fail(const WorldDesc& world,
                           std::span<const NamedMesh> resolved_meshes = {}) {
    spade::Result<RenderScene> scene = scene_from_world(world, resolved_meshes);
    if (!scene) {
        ADD_FAILURE() << "scene_from_world failed: " << scene.error().context;
        return RenderScene{};
    }
    return std::move(*scene);
}

// Post-R5, an SDF-derived mesh slot is never left empty (R2's
// tessellate_primitive() or R5's mesh_csg_subtree() always fills it) --
// these tests check for REAL geometry, the mirror image of R1-era's
// mesh_is_empty() check.
bool mesh_has_real_geometry(const MeshData& mesh) {
    return !mesh.positions.empty() && !mesh.indices.empty() && mesh.positions.size() == mesh.normals.size();
}

bool mat4_near(const glm::mat4& a, const glm::mat4& b, float tol) {
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            if (std::fabs(a[col][row] - b[col][row]) > tol) {
                return false;
            }
        }
    }
    return true;
}

// Independent forward transform for a pose (position, rotation, scale):
// p_world = position + scale * R * p_local -- see world/sdf.hpp's own
// SdfTransform doc comment for this exact relationship, which
// scene_from_world's local_to_world is required (SR-9) to reproduce by
// inverting the stored, pre-inverted world_to_local back.
glm::mat4 forward_transform(const SdfPose& pose) {
    glm::mat4 m = glm::mat4_cast(pose.rotation);
    m[0] *= pose.scale;
    m[1] *= pose.scale;
    m[2] *= pose.scale;
    m[3] = glm::vec4(pose.position, 1.0f);
    return m;
}

}  // namespace

// ---------------------------------------------------------------------------
// Static draw items: one per split_program() output entry (SR-9, revised at
// Task R5).
// ---------------------------------------------------------------------------

TEST(SceneFromWorld, StaticDrawItemCountMatchesNonOpNodes) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f, SdfPose{.position = {2.0f, 0.0f, 0.0f}})
        .box(glm::vec3(0.5f), SdfPose{.position = {-3.0f, 1.0f, 0.0f}})
        .union_();
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    // sphere + box + union -- both are union-primitive leaves (no CSG root),
    // so exactly 2 static items, one per tessellated primitive.
    EXPECT_EQ(scene.statics.size(), 2u);
    for (const DrawItem& item : scene.statics) {
        EXPECT_EQ(item.material_override, spade::render::kNoMaterial);
        ASSERT_LT(item.mesh_index, scene.meshes.size());
        EXPECT_TRUE(mesh_has_real_geometry(scene.meshes[item.mesh_index]))
            << "R2's tessellate_primitive() must have filled real geometry into this slot";
    }
}

TEST(SceneFromWorld, CsgRootSubtreeCollapsesToOneStaticItemNotOnePerPrimitive) {
    // box, box, subtract -- the gate-square prefab's own shape (S7a Task R5
    // brief). Two primitive nodes but ONE genuine CSG root: this must
    // produce exactly ONE static draw item, not two -- the defect this
    // whole task exists to prevent is exactly "rendered as 2 separate boxes"
    // (a solid block) instead of "rendered as 1 mesh with a hole".
    WorldBuilder b = base_builder();
    b.box(glm::vec3(2.0f, 2.0f, 0.3f)).box(glm::vec3(1.2f, 1.2f, 0.5f)).subtract();
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.statics.size(), 1u);
    ASSERT_LT(scene.statics[0].mesh_index, scene.meshes.size());
    const MeshData& mesh = scene.meshes[scene.statics[0].mesh_index];
    EXPECT_TRUE(mesh_has_real_geometry(mesh))
        << "R5's mesh_csg_subtree() must have filled real geometry into the CSG root's slot";
    // A subtract's surface-nets mesh is never axis-aligned-quad-flat the way
    // a single tessellated box's own 24 vertices are -- more than a bare box
    // is a cheap, independent signal that this really is the merged CSG
    // mesh and not, say, an accidental single-operand fallback.
    EXPECT_GT(mesh.positions.size(), 24u);

    // The CSG root node is an operator node, whose own `transform` field is
    // always 0 (the identity) -- SdfProgram::validate() enforces this, and
    // render/csg_mesh.hpp's mesh_csg_subtree() bakes its output in WORLD
    // space on that assumption (see its own doc comment).
    EXPECT_TRUE(mat4_near(scene.statics[0].local_to_world, glm::mat4(1.0f), kTol));
}

TEST(SceneFromWorld, MixedUnionAndCsgRootProgramPreservesAuthoringOrder) {
    // sphere, box, union, box, box, subtract, union -- a union of one plain
    // primitive (sphere) with a gate-square-shaped CSG root (box, box,
    // subtract). The merged static-item order must follow the PROGRAM's own
    // left-to-right authoring order (render/scene.cpp's own 2-pointer-merge
    // comment) -- sphere first, then the CSG root -- not "every
    // union-primitive before every CSG root" regardless of how they were
    // authored.
    WorldBuilder b = base_builder();
    b.sphere(1.0f)
        .box(glm::vec3(2.0f, 2.0f, 0.3f))
        .box(glm::vec3(1.2f, 1.2f, 0.5f))
        .subtract()
        .union_();
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.statics.size(), 2u);
    ASSERT_LT(scene.statics[0].mesh_index, scene.meshes.size());
    ASSERT_LT(scene.statics[1].mesh_index, scene.meshes.size());
    // The sphere (tessellate_primitive(), 156-vertex closed form at
    // kTessellationDefaults) precedes the CSG root (surface nets, no fixed
    // vertex count) -- checked structurally (which one looks like a UV
    // sphere grid) rather than pinning a specific count, which is R2's own
    // test's job.
    EXPECT_TRUE(mesh_has_real_geometry(scene.meshes[scene.statics[0].mesh_index]));
    EXPECT_TRUE(mesh_has_real_geometry(scene.meshes[scene.statics[1].mesh_index]));
    EXPECT_LT(scene.meshes[scene.statics[0].mesh_index].positions.size(),
              scene.meshes[scene.statics[1].mesh_index].positions.size())
        << "expected the sphere (item 0, authored first) before the CSG root (item 1, authored second)";
}

TEST(SceneFromWorld, StaticTransformIsInverseOfStoredNodeTransform) {
    const SdfPose pose{.position = {5.0f, -1.0f, 2.0f},
                        .rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)),
                        .scale = 2.0f};
    WorldBuilder b = base_builder();
    b.sphere(1.0f, pose);
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.statics.size(), 1u);
    const glm::mat4 expected = forward_transform(pose);
    EXPECT_TRUE(mat4_near(scene.statics[0].local_to_world, expected, kTol))
        << "expected a matrix recovering position/rotation/scale from the pose";
}

TEST(SceneFromWorld, StaticTransformIsIdentityForDefaultPose) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f);  // default SdfPose: identity transform (index 0)
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.statics.size(), 1u);
    EXPECT_TRUE(mat4_near(scene.statics[0].local_to_world, glm::mat4(1.0f), kTol));
}

// ---------------------------------------------------------------------------
// Materials: index 0 always the default material.
// ---------------------------------------------------------------------------

TEST(SceneFromWorld, MaterialsHasDefaultAtIndexZero) {
    const WorldDesc world = build_or_fail(base_builder());

    const RenderScene scene = scene_or_fail(world);

    ASSERT_GE(scene.materials.size(), 1u);
    const Material& def = scene.materials[0];
    EXPECT_FLOAT_EQ(def.base_color.x, 0.72f);
    EXPECT_FLOAT_EQ(def.base_color.y, 0.72f);
    EXPECT_FLOAT_EQ(def.base_color.z, 0.74f);
    EXPECT_FLOAT_EQ(def.base_color.w, 1.0f);
    EXPECT_EQ(def.shading, 0u);
}

// ---------------------------------------------------------------------------
// Spawn overlay (PA-4) and the sdf back-pointer.
// ---------------------------------------------------------------------------

TEST(SceneFromWorld, SpawnPositionsAndOrientationsMatchWorldSpawns) {
    const glm::quat rot = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    WorldBuilder b = base_builder();
    b.spawn("start", glm::vec3(1.0f, 2.0f, 3.0f))
        .spawn("finish", glm::vec3(-4.0f, 0.0f, 6.0f), rot);
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.spawn_positions.size(), 2u);
    ASSERT_EQ(scene.spawn_orientations.size(), 2u);
    EXPECT_EQ(scene.spawn_positions[0], world.spawns[0].position);
    EXPECT_EQ(scene.spawn_positions[1], world.spawns[1].position);
    EXPECT_EQ(scene.spawn_orientations[0], world.spawns[0].orientation);
    EXPECT_EQ(scene.spawn_orientations[1], world.spawns[1].orientation);
}

TEST(SceneFromWorld, SdfPointsAtTheWorldsOwnProgram) {
    const WorldDesc world = build_or_fail(base_builder());

    const RenderScene scene = scene_or_fail(world);

    EXPECT_EQ(scene.sdf, &world.sdf);
}

// ---------------------------------------------------------------------------
// Mesh index space: resolved_meshes copy in verbatim, at the front, in order
// (this task's own design note in scene.hpp -- see the MESH INDEX SPACE
// comment there for why).
// ---------------------------------------------------------------------------

TEST(SceneFromWorld, ResolvedMeshesOccupyTheFrontOfTheMeshArrayInOrder) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f);
    const WorldDesc world = build_or_fail(b);

    std::vector<NamedMesh> resolved(2);
    resolved[0].ref = "mesh:drone";
    resolved[0].mesh.positions = {glm::vec3(1.0f, 2.0f, 3.0f)};
    resolved[1].ref = "mesh:gate";
    resolved[1].mesh.positions = {glm::vec3(4.0f, 5.0f, 6.0f)};

    const RenderScene scene = scene_or_fail(world, resolved);

    ASSERT_GE(scene.meshes.size(), 3u);  // 2 resolved + 1 SDF-node placeholder
    EXPECT_EQ(scene.meshes[0].positions, resolved[0].mesh.positions);
    EXPECT_EQ(scene.meshes[1].positions, resolved[1].mesh.positions);

    ASSERT_EQ(scene.statics.size(), 1u);
    EXPECT_EQ(scene.statics[0].mesh_index, 2u)  // offset past the 2 resolved slots
        << "static draw items must not alias a resolved-mesh slot";
    EXPECT_TRUE(mesh_has_real_geometry(scene.meshes[scene.statics[0].mesh_index]))
        << "R2's tessellate_primitive() must have filled real geometry into the SDF-node slot";
}

// ---------------------------------------------------------------------------
// update_dynamics: rebuilds `dynamics` from `bodies`; never touches `statics`.
// ---------------------------------------------------------------------------

TEST(UpdateDynamics, ThreePosesYieldThreeDynamicItemsAndLeaveStaticsAlone) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).box(glm::vec3(0.5f)).union_();
    const WorldDesc world = build_or_fail(b);
    RenderScene scene = scene_or_fail(world);
    const std::vector<DrawItem> statics_before = scene.statics;

    const std::vector<BodyPose> bodies = {
        BodyPose{.position = {1.0f, 0.0f, 0.0f}, .mesh_index = 0},
        BodyPose{.position = {0.0f, 2.0f, 0.0f}, .mesh_index = 1},
        BodyPose{.position = {0.0f, 0.0f, 3.0f},
                 .orientation = glm::angleAxis(glm::radians(30.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
                 .mesh_index = spade::render::kNoMesh},
    };

    update_dynamics(scene, bodies);

    ASSERT_EQ(scene.dynamics.size(), 3u);
    for (size_t i = 0; i < bodies.size(); ++i) {
        EXPECT_EQ(scene.dynamics[i].mesh_index, bodies[i].mesh_index);
        EXPECT_EQ(scene.dynamics[i].material_override, spade::render::kNoMaterial);
        const glm::vec3 translation = glm::vec3(scene.dynamics[i].local_to_world[3]);
        EXPECT_EQ(translation, bodies[i].position);
    }

    // statics must be byte-for-byte unchanged.
    ASSERT_EQ(scene.statics.size(), statics_before.size());
    for (size_t i = 0; i < statics_before.size(); ++i) {
        EXPECT_EQ(scene.statics[i].mesh_index, statics_before[i].mesh_index);
        EXPECT_EQ(scene.statics[i].material_override, statics_before[i].material_override);
        EXPECT_TRUE(
            mat4_near(scene.statics[i].local_to_world, statics_before[i].local_to_world, kTol));
    }
}

TEST(UpdateDynamics, RebuildsFromScratchRatherThanAccumulating) {
    const WorldDesc world = build_or_fail(base_builder());
    RenderScene scene = scene_or_fail(world);

    const std::vector<BodyPose> three(3);
    update_dynamics(scene, three);
    ASSERT_EQ(scene.dynamics.size(), 3u);

    const std::vector<BodyPose> two(2);
    update_dynamics(scene, two);
    EXPECT_EQ(scene.dynamics.size(), 2u)
        << "update_dynamics must rebuild dynamics from `bodies`, not append to it";
}

// ---------------------------------------------------------------------------
// world_bounds_of (ported from raster.cpp's computeWorldBounds -- see
// scene.cpp): the default-box rule for an empty world, the origin-only
// heuristic for SDF nodes, and the fixed margin.
// ---------------------------------------------------------------------------

TEST(SceneFromWorldBounds, EmptyWorldGetsTheDefaultBox) {
    const WorldDesc world = build_or_fail(base_builder());  // no spawns, no SDF nodes

    const RenderScene scene = scene_or_fail(world);

    // kDefaultBoundsHalfExtent = 5.0 (raster.cpp's own constant, ported verbatim).
    EXPECT_EQ(scene.bounds.min, glm::vec3(-5.0f, -5.0f, -5.0f));
    EXPECT_EQ(scene.bounds.max, glm::vec3(5.0f, 5.0f, 5.0f));
}

TEST(SceneFromWorldBounds, SingleSpawnExpandsWithFixedMargin) {
    WorldBuilder b = base_builder();
    b.spawn("only", glm::vec3(10.0f, 0.0f, 0.0f));
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    // computeWorldBounds's ported constants: kBoundsMargin = 2.0 each side.
    EXPECT_EQ(scene.bounds.min, glm::vec3(8.0f, -2.0f, -2.0f));
    EXPECT_EQ(scene.bounds.max, glm::vec3(12.0f, 2.0f, 2.0f));
}

TEST(SceneFromWorldBounds, SdfNodeContributesItsTransformOriginNotItsShapeExtent) {
    // A box with half-extents (10,10,10) at the origin: the ported heuristic
    // expands bounds by the node's TRANSFORM ORIGIN only, not its true
    // geometric footprint -- so the result must NOT reflect the box's own
    // 10-unit half-extent, only the origin + fixed margin.
    WorldBuilder b = base_builder();
    b.box(glm::vec3(10.0f, 10.0f, 10.0f));
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    EXPECT_EQ(scene.bounds.min, glm::vec3(-2.0f, -2.0f, -2.0f));
    EXPECT_EQ(scene.bounds.max, glm::vec3(2.0f, 2.0f, 2.0f));
}

// ---------------------------------------------------------------------------
// ground_plane_y (ported from raster.cpp's groundPlaneY): identity-transform,
// +Y-normal plane only.
// ---------------------------------------------------------------------------

TEST(SceneFromWorldGround, IdentityUpPlaneIsDetected) {
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 3.5f);  // default (identity) pose
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    EXPECT_TRUE(scene.has_ground);
    EXPECT_FLOAT_EQ(scene.ground_y, 3.5f);
}

TEST(SceneFromWorldGround, NonUpNormalIsNotAGroundPlane) {
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(1.0f, 0.0f, 0.0f), 0.0f);  // a wall, not a floor
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    EXPECT_FALSE(scene.has_ground);
    EXPECT_FLOAT_EQ(scene.ground_y, 0.0f);
}

TEST(SceneFromWorldGround, PosedPlaneIsNotAGroundPlane) {
    // Only the identity transform (index 0) is handled -- a posed plane is
    // skipped even though its normal is up, matching raster.cpp exactly.
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f, SdfPose{.position = {0.0f, 1.0f, 0.0f}});
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    EXPECT_FALSE(scene.has_ground);
}

TEST(SceneFromWorldGround, NoPlaneMeansNoGround) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f);
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    EXPECT_FALSE(scene.has_ground);
    EXPECT_FLOAT_EQ(scene.ground_y, 0.0f);
}
