#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"
#include "world/world_file.hpp"

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

// ---------------------------------------------------------------------------
// node_materials wiring into DrawItem::material_override (S7a Task R6, W1's
// PA-2 field, world/sdf.hpp) -- for BOTH populations split_program() emits:
// a union-primitive leaf reads its OWN node's entry; a CSG root reads the
// OPERATOR node's own entry (WorldBuilder::material_for_last_node()'s target
// when called right after the operator adder), never a leaf's.
// ---------------------------------------------------------------------------

TEST(SceneFromWorld, UnionPrimitiveMaterialOverrideComesFromItsOwnNodeMaterialsEntry) {
    WorldBuilder b = base_builder();
    b.material(spade::MaterialDesc{.name = "default"})
        .material(spade::MaterialDesc{.name = "accent"})
        .sphere(1.0f)
        .material_for_last_node(1);
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.statics.size(), 1u);
    EXPECT_EQ(scene.statics[0].material_override, 1u);
}

TEST(SceneFromWorld, NodeMaterialsEmptyMeansEveryStaticItemUsesNoOverride) {
    // No .material_for_last_node() call anywhere -- node_materials stays
    // empty (sdf.hpp's own "empty means every node uses materials[0]"), so
    // every static item -- both a union primitive and a CSG root -- must
    // carry kNoMaterial, not some other stale or defaulted index.
    WorldBuilder b = base_builder();
    b.sphere(1.0f).box(glm::vec3(2.0f)).box(glm::vec3(1.2f)).subtract().union_();
    const WorldDesc world = build_or_fail(b);
    ASSERT_TRUE(world.sdf.node_materials.empty());

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.statics.size(), 2u);
    EXPECT_EQ(scene.statics[0].material_override, spade::render::kNoMaterial);
    EXPECT_EQ(scene.statics[1].material_override, spade::render::kNoMaterial);
}

TEST(SceneFromWorld, CsgRootMaterialOverrideComesFromTheOperatorNodesOwnEntryNotALeafs) {
    WorldBuilder b = base_builder();
    b.material(spade::MaterialDesc{.name = "default"})
        .material(spade::MaterialDesc{.name = "leaf_only"})
        .material(spade::MaterialDesc{.name = "root_only"})
        .box(glm::vec3(2.0f))
        .material_for_last_node(1)  // one of the two leaves under the subtract
        .box(glm::vec3(1.2f))
        .subtract()
        .material_for_last_node(2);  // the subtract node itself -- the CSG root
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.statics.size(), 1u) << "box, box, subtract is one CSG root, not two static items";
    EXPECT_EQ(scene.statics[0].material_override, 2u)
        << "the CSG root's own item must use the SUBTRACT node's node_materials entry, not either leaf's";
}

// S7a Task R5 fix wave (review IMPORTANT #3): every OTHER test in this file
// authors its own WorldBuilder fixture with default (identity) or trivial
// poses -- the one real world file this test suite loads anywhere
// (SplitProgram's own maximal.world.yaml test, test_render_csg.cpp) is used
// for split_program() alone and never put through scene_from_world(). That
// leaves transform_aabb()/local_to_world_of() (csg_mesh.cpp) completely
// unexercised against a genuine rotation or scale -- exactly the kind of
// pose a shipped, placed gate actually has -- while every existing golden
// hash stays self-consistent with whatever the code does, wrong or not.
//
// This test closes that gap end to end: load the real, committed
// tests/golden/worlds/maximal.world.yaml (three of its six primitives sit
// behind a non-identity, non-unit-scale transform -- transforms[1], [2] and
// [4]) through the real scene_from_world(), and assert every mesh slot is
// real geometry at a PLAUSIBLE resolution. A bounds bug that silently
// starves a subtree's grid (review IMPORTANT #1's own defect) would show up
// here as a near-empty or degenerate CSG mesh; every earlier, hand-authored
// fixture in this file is too well-resolved by construction to ever notice.
TEST(SceneFromWorld, RealWorldFileWithNonIdentityPosesWiresGeometryThroughEndToEnd) {
    const spade::Result<WorldDesc> loaded =
        spade::load_world_file(std::filesystem::path(SPADE_GOLDEN_DIR) / "worlds" / "maximal.world.yaml");
    ASSERT_TRUE(loaded.has_value()) << "failed to load maximal.world.yaml";

    const RenderScene scene = scene_or_fail(*loaded);

    // split_program()'s own dedicated test (test_render_csg.cpp) proves
    // this program decomposes into csg_roots={8}, union_primitive_nodes=
    // {9, 10} -- 3 SDF-derived static items, in that ascending-node-index
    // order: the CSG root (node 8, a smooth_union of an intersect-clipped
    // box with a subtract), then the torus (node 9), then the heightfield
    // (node 10). Node 0's plane sits INSIDE that CSG subtree (it feeds
    // node 4's intersect, not a plain union all the way to the program's
    // root) and so is correctly absorbed into item 0's mesh rather than
    // becoming its own item or a standalone ground-plane candidate (SR-17:
    // "never one that is a member of a CSG subtree").
    //
    // scene_from_world() then appends TWO more static items (S7a Task R6,
    // Step 3) -- one per this file's own `props:` list (tower, then flag,
    // in authoring order) -- for 5 total.
    ASSERT_EQ(scene.statics.size(), 5u);
    for (size_t i = 0; i < 3; ++i) {
        const DrawItem& item = scene.statics[i];
        ASSERT_LT(item.mesh_index, scene.meshes.size());
        EXPECT_TRUE(mesh_has_real_geometry(scene.meshes[item.mesh_index]));
    }

    // Item 0 is the CSG root. Its own subtree includes node 4's box, placed
    // behind transforms[2] -- a real rotation + a 0.4x uniform scale, not
    // the identity every other fixture in this file uses.
    const MeshData& csg_mesh = scene.meshes[scene.statics[0].mesh_index];
    const size_t triangle_count = csg_mesh.indices.size() / 3;
    EXPECT_GT(triangle_count, 200u)
        << "the CSG root's mesh looks collapsed -- got only " << triangle_count
        << " triangles; a resolution-starved AABB (review IMPORTANT #1) would look exactly like this";

    // Items 3/4 are the two props (tower, material 1; flag, material 2) --
    // this call passes no resolved_meshes, so neither mesh_ref ("mesh:props/
    // tower"/"mesh:props/flag") resolves to anything, and both items must
    // carry kNoMesh (draw_mesh_item's own existing "nothing to draw" skip)
    // rather than an out-of-range or aliased mesh_index.
    EXPECT_EQ(scene.statics[3].mesh_index, spade::render::kNoMesh);
    EXPECT_EQ(scene.statics[3].material_override, 1u);
    EXPECT_EQ(scene.statics[4].mesh_index, spade::render::kNoMesh);
    EXPECT_EQ(scene.statics[4].material_override, 2u);
}

TEST(SceneFromWorld, StaticTransformIsInverseOfStoredNodeTransform) {
    // 90 deg about Y, half-angle 45 deg -- spelled as literal float32
    // quaternion components rather than glm::angleAxis. Not itself a
    // committed-golden value (this pose is checked against an
    // independently-computed forward_transform() at runtime, below), but
    // this FILE now also loads tests/golden/worlds/maximal.world.yaml
    // (RealWorldFileWithNonIdentityPosesWiresGeometryThroughEndToEnd,
    // above), which pulls the whole translation unit into SR-14's widened
    // libm-transcendental scan (test_m1b_bar.cpp) -- a scan that flags any
    // glm::angleAxis call in a golden-feeding file regardless of which
    // specific test uses it. 0x1.6a09e6p-1f is the nearest float32 to
    // cos(45deg) = sin(45deg) = sqrt(2)/2 -- the identical literal
    // test_render_raster.cpp's camera_top_down() already uses for the same
    // 90-degree-about-an-axis shape.
    const SdfPose pose{.position = {5.0f, -1.0f, 2.0f},
                        .rotation = glm::quat(0x1.6a09e6p-1f, 0.0f, 0x1.6a09e6p-1f, 0.0f),
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

    // world.materials[0] is WorldBuilder::build()'s own default MaterialDesc{}
    // (builder.hpp) when a test never calls .material() -- base_color
    // (1,1,1,1), lambert shading. NOT render::Material{}'s own struct-literal
    // default (scene.hpp's (0.72,0.72,0.74,1.0) grey): scene_from_world() now
    // CONSUMES world.materials (S7a Task R6) instead of hardcoding its own
    // default, so this test's expectation moved from this module's default to
    // the WORLD's.
    ASSERT_GE(scene.materials.size(), 1u);
    const Material& def = scene.materials[0];
    EXPECT_FLOAT_EQ(def.base_color.x, 1.0f);
    EXPECT_FLOAT_EQ(def.base_color.y, 1.0f);
    EXPECT_FLOAT_EQ(def.base_color.z, 1.0f);
    EXPECT_FLOAT_EQ(def.base_color.w, 1.0f);
    EXPECT_EQ(def.shading, 0u);
}

TEST(SceneFromWorld, MaterialsMirrorWorldMaterialsFieldForField) {
    WorldBuilder b = base_builder();
    b.material(spade::MaterialDesc{.name = "red", .base_color = {1.0f, 0.0f, 0.0f, 1.0f},
                                    .shading = spade::MaterialShading::lambert})
        .material(spade::MaterialDesc{.name = "glow", .base_color = {0.1f, 0.9f, 0.2f, 0.5f},
                                       .shading = spade::MaterialShading::emissive})
        .material(spade::MaterialDesc{.name = "decal", .base_color = {0.5f, 0.5f, 0.5f, 1.0f},
                                       .shading = spade::MaterialShading::unlit});
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.materials.size(), 3u);
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(scene.materials[i].base_color, world.materials[i].base_color) << "material " << i;
        EXPECT_EQ(scene.materials[i].shading, static_cast<uint32_t>(world.materials[i].shading)) << "material " << i;
    }
}

TEST(SceneFromWorld, LightingMirrorsWorldLightingAndNormalizesSunDirection) {
    WorldBuilder b = base_builder();
    // Not pre-normalized (length 5) -- LightingDesc's own doc comment says
    // this is allowed, and scene_from_world() (not the caller) normalizes it.
    b.lighting(spade::LightingDesc{
        .sun_direction = {0.0f, 5.0f, 0.0f},
        .sun_color = {0.5f, 0.6f, 0.7f},
        .sun_intensity = 2.5f,
        .ambient_color = {0.11f, 0.12f, 0.13f},
        .sky_zenith = {0.2f, 0.3f, 0.4f},
        .sky_horizon = {0.6f, 0.7f, 0.8f},
    });
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    EXPECT_EQ(scene.lighting.sun_direction, glm::vec3(0.0f, 1.0f, 0.0f))
        << "sun_direction must come out unit-length even though the world file's copy is not";
    EXPECT_EQ(scene.lighting.sun_color, world.lighting.sun_color);
    EXPECT_FLOAT_EQ(scene.lighting.sun_intensity, world.lighting.sun_intensity);
    EXPECT_EQ(scene.lighting.ambient_color, world.lighting.ambient_color);
    EXPECT_EQ(scene.lighting.sky_zenith, world.lighting.sky_zenith);
    EXPECT_EQ(scene.lighting.sky_horizon, world.lighting.sky_horizon);
}

// ---------------------------------------------------------------------------
// Spawn overlay (PA-4) and the sdf back-pointer.
// ---------------------------------------------------------------------------

TEST(SceneFromWorld, SpawnPositionsAndOrientationsMatchWorldSpawns) {
    // 45 deg about Y, half-angle 22.5 deg -- same literal-quaternion
    // rationale as StaticTransformIsInverseOfStoredNodeTransform above.
    // 0x1.d906bcp-1f/0x1.87de2ap-2f are the nearest float32 values to
    // cos(22.5deg)/sin(22.5deg) -- the identical literals
    // test_render_raster.cpp's own spawn_orientations fixture already uses.
    const glm::quat rot(0x1.d906bcp-1f, 0.0f, 0x1.87de2ap-2f, 0.0f);
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
// Props (schema v2, S7a Task R6 Step 3): one static DrawItem per world.props
// entry, using the caller-resolved mesh list the same way a NamedMesh's own
// doc comment describes.
// ---------------------------------------------------------------------------

TEST(SceneFromWorldProps, ResolvedPropGetsItsMeshIndexPoseAndMaterial) {
    WorldBuilder b = base_builder();
    b.material(spade::MaterialDesc{.name = "default"}).material(spade::MaterialDesc{.name = "banner"});
    const SdfPose pose{.position = {3.0f, 0.0f, -2.0f}, .scale = 2.0f};
    b.prop("mesh:props/banner", pose, 1);
    const WorldDesc world = build_or_fail(b);

    std::vector<NamedMesh> resolved(1);
    resolved[0].ref = "mesh:props/banner";
    resolved[0].mesh.positions = {glm::vec3(0.0f)};

    const RenderScene scene = scene_or_fail(world, resolved);

    ASSERT_EQ(scene.statics.size(), 1u);
    EXPECT_EQ(scene.statics[0].mesh_index, 0u) << "must resolve to the matching NamedMesh's slot";
    EXPECT_EQ(scene.statics[0].material_override, 1u);
    EXPECT_TRUE(mat4_near(scene.statics[0].local_to_world, forward_transform(pose), kTol));
}

TEST(SceneFromWorldProps, UnresolvedPropMeshRefGetsNoMeshRatherThanAnErrorOrWrongSlot) {
    WorldBuilder b = base_builder();
    b.prop("mesh:props/never_resolved", SdfPose{}, 0);
    const WorldDesc world = build_or_fail(b);

    // Deliberately empty -- the caller never resolved this prop's reference.
    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.statics.size(), 1u);
    EXPECT_EQ(scene.statics[0].mesh_index, spade::render::kNoMesh);
}

TEST(SceneFromWorldProps, MultiplePropsAppendAfterSdfStaticsInAuthoringOrder) {
    WorldBuilder b = base_builder();
    b.material(spade::MaterialDesc{.name = "default"})
        .material(spade::MaterialDesc{.name = "a"})
        .material(spade::MaterialDesc{.name = "b"});
    b.sphere(1.0f);
    b.prop("mesh:first", SdfPose{}, 1).prop("mesh:second", SdfPose{}, 2);
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.statics.size(), 3u);
    // Item 0 is the sphere (the only SDF-derived static); items 1/2 are the
    // props, in authoring order, both after every SDF-derived item.
    EXPECT_EQ(scene.statics[1].material_override, 1u);
    EXPECT_EQ(scene.statics[2].material_override, 2u);
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
        // 30 deg about X, half-angle 15 deg -- same literal-quaternion
        // rationale as StaticTransformIsInverseOfStoredNodeTransform above.
        // 0x1.ee8dd4p-1f/0x1.0907dcp-2f are the nearest float32 values to
        // cos(15deg)/sin(15deg) -- the identical literals
        // test_render_raster.cpp's own box_spin fixture already uses.
        BodyPose{.position = {0.0f, 0.0f, 3.0f},
                 .orientation = glm::quat(0x1.ee8dd4p-1f, 0x1.0907dcp-2f, 0.0f, 0.0f),
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

// ---------------------------------------------------------------------------
// scene.ground_planes (S7a Task R6, ruling SR-17) -- a DIFFERENT, newer, more
// general mechanism than has_ground/ground_y above (that pair is the OLDER
// overlay grid's own identity-transform/+Y-only heuristic and is untouched
// by this task): one entry per STANDALONE plane primitive (split_program()'s
// own union-primitive-leaf classification), in WORLD space, general for any
// transform -- "no Y-up special case" (SR-17's own ruling text).
// ---------------------------------------------------------------------------

TEST(SceneFromWorldGroundPlanes, IdentityPlaneBecomesOneCandidateWithMaterialZero) {
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 3.5f);  // identity pose
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.ground_planes.size(), 1u);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(scene.ground_planes[0].normal, glm::vec3(0.0f, 1.0f, 0.0f), kTol)));
    EXPECT_NEAR(scene.ground_planes[0].offset, 3.5f, kTol);
    EXPECT_EQ(scene.ground_planes[0].material, 0u);
}

TEST(SceneFromWorldGroundPlanes, NodeMaterialsEntrySetsTheGroundPlanesMaterial) {
    WorldBuilder b = base_builder();
    b.material(spade::MaterialDesc{.name = "default"}).material(spade::MaterialDesc{.name = "asphalt"});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).material_for_last_node(1);
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.ground_planes.size(), 1u);
    EXPECT_EQ(scene.ground_planes[0].material, 1u);
    // The plane's own static DrawItem must resolve identically -- the two
    // paths (tessellated grid, analytic ground) shade with the SAME material
    // (task-R6-brief.md's own "the seam is load-bearing" requirement).
    ASSERT_EQ(scene.statics.size(), 1u);
    EXPECT_EQ(scene.statics[0].material_override, 1u);
}

TEST(SceneFromWorldGroundPlanes, PlaneInsideACsgSubtreeIsNeverAGroundPlaneCandidate) {
    // box, plane, subtract -- the plane is a CUTTING half-space, not a floor
    // (SR-17's own "a plane inside a subtract is a cutting half-space, not a
    // floor" text): it must never appear in ground_planes, even though it is
    // still a `prim: plane` node in the program.
    WorldBuilder b = base_builder();
    b.box(glm::vec3(2.0f)).plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).subtract();
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.statics.size(), 1u) << "sanity: this must be exactly one CSG root, not two static items";
    EXPECT_TRUE(scene.ground_planes.empty());
}

TEST(SceneFromWorldGroundPlanes, PosedPlaneUsesTheGeneralTransformNotAYUpSpecialCase) {
    // 90 degrees about Z (half-angle 45 deg), spelled as a literal float32
    // quaternion rather than glm::angleAxis -- this file loads
    // maximal.world.yaml elsewhere (RealWorldFileWithNonIdentityPosesWires-
    // GeometryThroughEndToEnd's own comment), which pulls the whole
    // translation unit into SR-14's libm-transcendental scan; 0x1.6a09e6p-1f
    // is the nearest float32 to cos(45deg) = sin(45deg) = sqrt(2)/2, the same
    // literal this file's own StaticTransformIsInverseOfStoredNodeTransform
    // test already uses for a different axis.
    //
    // Rotating a LOCAL +Y-normal, offset-2 plane by +90 deg about +Z sends
    // the normal to WORLD -X (x' = x*cos - y*sin = -1, y' = x*sin + y*cos = 0
    // at exactly 90 deg) and the local point (0,2,0) that sits on the plane
    // to WORLD (-2,0,0) before translation, so the expected world-space
    // offset -- dot(p_world, n_world) -- is 0 once the +2 X-translation below
    // is folded in: p_world = (-2,0,0) + (2,3,-1) = (0,3,-1),
    // dot((0,3,-1), (-1,0,0)) = 0.
    const glm::quat rot(0x1.6a09e6p-1f, 0.0f, 0.0f, 0x1.6a09e6p-1f);
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 2.0f,
            SdfPose{.position = {2.0f, 3.0f, -1.0f}, .rotation = rot});
    const WorldDesc world = build_or_fail(b);

    const RenderScene scene = scene_or_fail(world);

    ASSERT_EQ(scene.ground_planes.size(), 1u);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(scene.ground_planes[0].normal, glm::vec3(-1.0f, 0.0f, 0.0f), kTol)))
        << "got (" << scene.ground_planes[0].normal.x << ", " << scene.ground_planes[0].normal.y << ", "
        << scene.ground_planes[0].normal.z << ")";
    EXPECT_NEAR(scene.ground_planes[0].offset, 0.0f, kTol);
}
