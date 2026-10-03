// spade::scene's composer (scene/compose.hpp): compose_transform, compose(),
// compose_file() and instantiate(). Plan:
// docs/design/interface/plans/2026-10-03-scene-composer-plan.md, "Tests, when
// it is built", with Core's review. The scene file itself (reader, writer,
// world hash) is Core's and is tested in test_scene_file.cpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "scene/compose.hpp"
#include "scene/scene_file.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "testing/replay.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"
#include "world/sdf.hpp"
#include "world/world_file.hpp"

namespace {

using spade::SdfTransform;
using spade::scene::ComposedScene;
using spade::scene::SceneAsset;
using spade::scene::SceneDesc;
using spade::scene::SceneVehicle;

constexpr uint64_t kDtNs = 2'000'000;
constexpr uint32_t kSubsteps = 2;

// The bytes config_hash folds: two transforms are "the same" only if these are.
bool same_bytes(const SdfTransform& a, const SdfTransform& b) {
    return std::memcmp(&a, &b, sizeof(SdfTransform)) == 0;
}

// A pose as transform_of() builds it off the identity: world_to_local =
// transpose(R) / s with the translation -(m * position). At a zero position
// that translation is -(m * 0) = -0 in every lane: an identity with negative
// zeros, exactly what the short-cuts must still recognise.
SdfTransform builder_style(const glm::vec3& position, const glm::quat& rotation, float scale) {
    const glm::mat3 m = glm::transpose(glm::mat3_cast(rotation / glm::length(rotation))) / scale;
    const glm::vec3 c = -(m * position);
    SdfTransform t{};
    t.world_to_local = glm::mat4(m);
    t.world_to_local[3] = glm::vec4(c, 1.0f);
    t.scale = scale;
    return t;
}

const glm::quat kTilt = glm::angleAxis(0.7f, glm::normalize(glm::vec3(0.3f, 1.0f, -0.4f)));

spade::SdfPose pose_at(glm::vec3 position, glm::quat rotation, float scale) {
    spade::SdfPose p;
    p.position = position;
    p.rotation = rotation;
    p.scale = scale;
    return p;
}

// The world every test composes over, as a builder so a test can add to it.
spade::WorldBuilder ground_builder() {
    spade::WorldBuilder b;
    b.name("ground")
        .environment(spade::Environment{})
        .capacities(spade::Capacities{1, 2, 3, 4})
        .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    return b;
}

spade::WorldDesc ground_world() {
    return ground_builder().build().value();
}

spade::SdfNode primitive(spade::SdfPrim kind, glm::vec4 params, uint32_t transform = 0) {
    spade::SdfNode n{};
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.transform = transform;
    n.params = params;
    return n;
}

// A one-sphere collider whose only transform is `t`.
spade::SdfProgram sphere_collider(float radius, const SdfTransform& t = SdfTransform{}) {
    spade::SdfProgram p;
    p.transforms.push_back(t);
    p.nodes.push_back(primitive(spade::SdfPrim::sphere, glm::vec4(radius, 0.0f, 0.0f, 0.0f)));
    return p;
}

SceneDesc scene_over(const spade::WorldDesc& world) {
    SceneDesc s;
    s.name = "test_scene";
    s.world.file = "ground.world.yaml";
    s.world.hash = spade::scene::world_hash(world).value();
    return s;
}

SceneAsset collider_asset(std::string name, const spade::SdfPose& pose, spade::SdfProgram collider) {
    SceneAsset a;
    a.name = std::move(name);
    a.pose = pose;
    a.collider = std::move(collider);
    return a;
}

SceneAsset visual_asset(std::string name, const spade::SdfPose& pose, std::string mesh, std::string material) {
    SceneAsset a;
    a.name = std::move(name);
    a.pose = pose;
    a.visual.mesh_ref = std::move(mesh);
    a.visual.material = std::move(material);
    return a;
}

spade::MaterialDesc named_material(std::string name) {
    spade::MaterialDesc m;
    m.name = std::move(name);
    m.base_color = glm::vec4(0.95f, 0.45f, 0.1f, 1.0f);
    return m;
}

// bench_sim.cpp's 1 kg airframe (as test_design_frame.cpp has it): valid, with
// rotors, drag and an IMU, so a vehicle needs force elements and a sensor.
spade::vehicles::ModelType quad_model(std::string name, glm::quat q_bd = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                      glm::vec3 com = glm::vec3(0.0f)) {
    spade::vehicles::QuadrotorParams p;
    p.name = std::move(name);
    p.mass = 1.0f;
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f);
    p.arm_length = 0.18f;
    p.rotor_height = 0.02f;
    p.proxy_radius = 0.2f;
    for (spade::vehicles::RotorParams& rotor : p.rotors) {
        rotor.tau = 0.02f;
        rotor.radius = 0.13f;
        rotor.thrust_coeff = 1.2e-5f;
        rotor.torque_coeff = 1.9e-7f;
    }
    p.drag.mode = spade::physics::drag_mode::quadratic;
    p.drag.area = 0.05f;
    p.drag.coeffs = glm::vec3(1.6f, 0.0f, 0.0f);
    p.imu.rate_divider = 1;
    spade::vehicles::ModelType m = spade::vehicles::make_quadrotor(p).value();
    m.design_to_principal = q_bd;
    m.com_offset = com;
    return m;
}

SceneVehicle vehicle(std::string name, std::string model, glm::vec3 position) {
    SceneVehicle v;
    v.name = std::move(name);
    v.model = std::move(model);
    v.start.pos = position;
    return v;
}

spade::WorldInstanceDesc instance_of(const spade::WorldDesc& world) {
    spade::WorldInstanceDesc inst;
    inst.world = world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return inst;
}

uint64_t config_hash_of(const spade::WorldDesc& world) {
    return spade::config_hash(spade::WorldSetDesc{{instance_of(world)}});
}

std::string yaml_of(const spade::WorldDesc& world) {
    return spade::world_to_yaml(world).value();
}

// The state digest after one step of a composed scene.
uint64_t first_digest(const ComposedScene& composed) {
    spade::Result<spade::scene::SceneRun> run =
        spade::scene::instantiate(composed, instance_of(composed.world), kDtNs, kSubsteps);
    EXPECT_TRUE(run.has_value()) << (run ? "" : run.error().context);
    if (!run) {
        return 0;
    }
    EXPECT_TRUE(run->sim.step(1).has_value());
    return spade::testing::state_digest(run->sim);
}

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

// A scratch directory of the test's own, emptied first.
std::filesystem::path scratch_dir(const std::string& name) {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / ("spade_scene_compose_" + name);
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir);
    return dir;
}

void write_text(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    out << text;
}

}  // namespace

// ===========================================================================
// compose_transform
// ===========================================================================

// The reason for the short-cuts: a 4x4 product with the identity is not
// bitwise identity-preserving, because (-0)*1 + (+0) is +0. Rotation matrices
// carry -0 entries, and config_hash folds bytes. If glm ever stopped doing
// this, the short-cuts would still be right, but this test says why they exist.
TEST(SceneComposeTransform, AProductWithTheIdentityCanFlipNegativeZero) {
    const SdfTransform posed = builder_style(glm::vec3(0.0f), kTilt, 1.0f);  // -0 translation
    const glm::mat4 product = posed.world_to_local * glm::mat4(1.0f);
    const glm::mat4 product2 = glm::mat4(1.0f) * posed.world_to_local;
    const bool flipped = std::memcmp(&product, &posed.world_to_local, sizeof(glm::mat4)) != 0 ||
                         std::memcmp(&product2, &posed.world_to_local, sizeof(glm::mat4)) != 0;
    EXPECT_TRUE(flipped) << "the plain product preserved every byte; the short-cuts' stated reason no longer holds";
}

// Short-cut (a): an asset at the identity pose leaves its collider's
// transform bitwise unchanged, negative zeros included.
TEST(SceneComposeTransform, AnIdentityAssetPoseCopiesTheColliderBitwise) {
    const SdfTransform collider = builder_style(glm::vec3(0.0f, 1.8f, 0.0f), kTilt, 1.5f);
    const SdfTransform identity_pose = builder_style(glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1.0f);
    EXPECT_TRUE(same_bytes(spade::scene::compose_transform(collider, identity_pose), collider));
}

// Short-cut (b): a collider at the identity takes the asset's pose verbatim,
// so a one-primitive asset gets exactly the bytes WorldBuilder would give that
// primitive at the asset's pose.
TEST(SceneComposeTransform, AnIdentityColliderTakesTheAssetPoseVerbatim) {
    const SdfTransform identity_collider = builder_style(glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1.0f);
    const SdfTransform asset = builder_style(glm::vec3(2.0f, 0.5f, -3.0f), kTilt, 0.8f);
    EXPECT_TRUE(same_bytes(spade::scene::compose_transform(identity_collider, asset), asset));
}

// The general case: a world point goes into the asset's frame, then into the
// collider's. The composed transform does both in one step, and the distance
// scales multiply.
TEST(SceneComposeTransform, TheProductTakesAWorldPointThroughBoth) {
    const SdfTransform collider = builder_style(glm::vec3(0.4f, 0.0f, 0.1f), glm::angleAxis(0.3f, glm::vec3(1, 0, 0)), 1.5f);
    const SdfTransform asset = builder_style(glm::vec3(2.0f, 0.5f, -3.0f), kTilt, 0.8f);
    const SdfTransform composed = spade::scene::compose_transform(collider, asset);
    EXPECT_EQ(composed.scale, collider.scale * asset.scale);
    for (const glm::vec3 p : {glm::vec3(0.0f), glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(-4.0f, 0.5f, 2.5f)}) {
        const glm::vec4 two_steps = collider.world_to_local * (asset.world_to_local * glm::vec4(p, 1.0f));
        const glm::vec4 one_step = composed.world_to_local * glm::vec4(p, 1.0f);
        EXPECT_NEAR(glm::length(glm::vec3(two_steps - one_step)), 0.0f, 1e-5f) << "point " << p.x << "," << p.y << "," << p.z;
    }
}

// Determinism: the same inputs give the same bytes, every time.
TEST(SceneComposeTransform, ComposingTwiceGivesTheSameBytes) {
    const SdfTransform collider = builder_style(glm::vec3(0.4f, 0.0f, 0.1f), glm::angleAxis(0.3f, glm::vec3(1, 0, 0)), 1.5f);
    const SdfTransform asset = builder_style(glm::vec3(2.0f, 0.5f, -3.0f), kTilt, 0.8f);
    EXPECT_TRUE(same_bytes(spade::scene::compose_transform(collider, asset),
                           spade::scene::compose_transform(collider, asset)));
}

// ===========================================================================
// compose(): geometry
// ===========================================================================

// Pinned to the builder: a one-primitive collider at the origin, posed by its
// asset, gives the world WorldBuilder gives for that primitive at that pose,
// byte for byte, so the config hash agrees too (short-cut (b) and
// transform_of()).
TEST(SceneCompose, AOnePrimitiveAssetIsTheBuildersPrimitiveAtItsPose) {
    const spade::SdfPose pose = pose_at(glm::vec3(1.5f, 2.0f, -0.5f), kTilt, 0.8f);
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(collider_asset("ball", pose, sphere_collider(0.5f)));

    const ComposedScene composed = spade::scene::compose(scene, world).value();
    const spade::WorldDesc built = ground_builder().sphere(0.5f, pose).union_().build().value();
    EXPECT_EQ(yaml_of(composed.world), yaml_of(built));
    EXPECT_EQ(config_hash_of(composed.world), config_hash_of(built));
}

// Union semantics: at sample points the composed SDF is the smaller of the
// world's distance and the asset's (its collider's, in the asset's frame,
// scaled by the asset).
TEST(SceneCompose, TheComposedSdfIsTheUnionOfTheWorldAndThePosedAsset) {
    spade::SdfProgram collider;
    collider.transforms.push_back(builder_style(glm::vec3(0.3f, 0.0f, 0.1f), glm::angleAxis(0.4f, glm::vec3(1, 0, 0)), 1.25f));
    collider.nodes.push_back(primitive(spade::SdfPrim::box, glm::vec4(0.6f, 0.3f, 0.2f, 0.0f)));
    const spade::SdfPose pose = pose_at(glm::vec3(2.0f, 1.0f, -1.0f), kTilt, 0.8f);

    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(collider_asset("block", pose, collider));
    const ComposedScene composed = spade::scene::compose(scene, world).value();

    const SdfTransform asset_t = spade::transform_of(pose).value();
    for (const glm::vec3 p : {glm::vec3(2.0f, 1.0f, -1.0f), glm::vec3(2.4f, 1.2f, -0.8f), glm::vec3(0.0f, 0.5f, 0.0f),
                              glm::vec3(3.0f, 0.2f, -2.0f), glm::vec3(-1.0f, 4.0f, 2.0f)}) {
        const glm::vec3 local(asset_t.world_to_local * glm::vec4(p, 1.0f));
        const float asset_d = spade::eval(collider, local) * pose.scale;
        const float expected = std::min(spade::eval(world.sdf, p), asset_d);
        EXPECT_NEAR(spade::eval(composed.world.sdf, p), expected, 1e-4f * (1.0f + std::abs(expected)))
            << "point " << p.x << "," << p.y << "," << p.z;
    }
}

// Short-cut (a) through compose(): an asset at the identity pose appends its
// collider's transforms bitwise.
TEST(SceneCompose, AnAssetAtTheIdentityPoseKeepsItsColliderTransformsBitwise) {
    const SdfTransform own = builder_style(glm::vec3(0.0f, 1.8f, 0.0f), kTilt, 1.5f);
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(collider_asset("ring", spade::SdfPose{}, sphere_collider(0.5f, own)));

    const ComposedScene composed = spade::scene::compose(scene, world).value();
    ASSERT_EQ(composed.world.sdf.transforms.size(), world.sdf.transforms.size() + 1);
    EXPECT_TRUE(same_bytes(composed.world.sdf.transforms.back(), own));
}

// Only a primitive's transform index moves; an operator node's stays 0, and
// one union_ joins each asset to the program before it.
TEST(SceneCompose, PrimitivesAreReindexedAndEachAssetIsJoinedByOneUnion) {
    spade::SdfProgram pair;
    pair.transforms.push_back(SdfTransform{});
    pair.transforms.push_back(builder_style(glm::vec3(0.5f, 0.0f, 0.0f), kTilt, 1.0f));
    pair.nodes.push_back(primitive(spade::SdfPrim::sphere, glm::vec4(0.2f, 0.0f, 0.0f, 0.0f), 0));
    pair.nodes.push_back(primitive(spade::SdfPrim::sphere, glm::vec4(0.3f, 0.0f, 0.0f, 0.0f), 1));
    spade::SdfNode op{};
    op.op = static_cast<uint32_t>(spade::SdfOp::union_);
    pair.nodes.push_back(op);

    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(collider_asset("pair", pose_at(glm::vec3(1.0f, 1.0f, 0.0f), kTilt, 1.0f), pair));
    const ComposedScene composed = spade::scene::compose(scene, world).value();

    const auto base = static_cast<uint32_t>(world.sdf.transforms.size());
    const std::vector<spade::SdfNode>& nodes = composed.world.sdf.nodes;
    ASSERT_EQ(nodes.size(), world.sdf.nodes.size() + 4);  // the pair's three, then the join
    const std::size_t first = world.sdf.nodes.size();
    EXPECT_EQ(nodes[first].transform, base + 0);
    EXPECT_EQ(nodes[first + 1].transform, base + 1);
    EXPECT_EQ(nodes[first + 2].op, static_cast<uint32_t>(spade::SdfOp::union_));
    EXPECT_EQ(nodes[first + 2].transform, 0u);
    EXPECT_EQ(nodes[first + 3].op, static_cast<uint32_t>(spade::SdfOp::union_));
    EXPECT_EQ(nodes[first + 3].transform, 0u);
}

// The first geometry of an empty program needs no union.
TEST(SceneCompose, TheFirstAssetInAnEmptyWorldIsNotJoined) {
    const spade::WorldDesc world = spade::WorldBuilder()
                                       .name("empty")
                                       .environment(spade::Environment{})
                                       .capacities(spade::Capacities{1, 1, 1, 1})
                                       .build()
                                       .value();
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(collider_asset("ball", pose_at(glm::vec3(0.0f, 1.0f, 0.0f), kTilt, 1.0f), sphere_collider(0.5f)));
    const ComposedScene composed = spade::scene::compose(scene, world).value();
    ASSERT_EQ(composed.world.sdf.nodes.size(), 1u);
    EXPECT_EQ(composed.world.sdf.nodes[0].op, static_cast<uint32_t>(spade::SdfOp::none));
}

// ===========================================================================
// compose(): materials and visuals
// ===========================================================================

// Assets that name no node material keep the world's empty array, so the
// world's bytes for it do not change.
TEST(SceneCompose, PlainAssetsKeepTheWorldsEmptyNodeMaterials) {
    const spade::WorldDesc world = ground_world();
    ASSERT_TRUE(world.sdf.node_materials.empty());
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(collider_asset("a", pose_at(glm::vec3(1.0f, 1.0f, 0.0f), kTilt, 1.0f), sphere_collider(0.5f)));
    scene.assets.push_back(collider_asset("b", pose_at(glm::vec3(-1.0f, 1.0f, 0.0f), kTilt, 1.0f), sphere_collider(0.5f)));
    const ComposedScene composed = spade::scene::compose(scene, world).value();
    EXPECT_TRUE(composed.world.sdf.node_materials.empty());
}

// One named node fills the array to full length: 0 for the world's nodes, for
// unnamed assets and for every union_, and the resolved index for the name.
TEST(SceneCompose, OneNamedNodeFillsNodeMaterialsWithTheDefaultElsewhere) {
    const spade::WorldDesc world = ground_world();  // palette: "default"
    SceneDesc scene = scene_over(world);
    scene.materials.push_back(named_material("gate_orange"));  // index 1 once appended
    scene.assets.push_back(collider_asset("plain", pose_at(glm::vec3(1.0f, 1.0f, 0.0f), kTilt, 1.0f), sphere_collider(0.5f)));
    SceneAsset named = collider_asset("named", pose_at(glm::vec3(-1.0f, 1.0f, 0.0f), kTilt, 1.0f), sphere_collider(0.5f));
    named.collider_materials = {"gate_orange"};
    scene.assets.push_back(named);

    const ComposedScene composed = spade::scene::compose(scene, world).value();
    // plane, plain sphere, union, named sphere, union
    EXPECT_EQ(composed.world.sdf.node_materials, (std::vector<uint32_t>{0, 0, 0, 1, 0}));
    ASSERT_EQ(composed.world.materials.size(), 2u);
    EXPECT_EQ(composed.world.materials[1].name, "gate_orange");
}

// A visual becomes a prop at the asset's pose, its material resolved by name;
// an asset with no collider leaves the SDF program as it was.
TEST(SceneCompose, AVisualBecomesAPropAtTheAssetsPose) {
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.materials.push_back(named_material("gate_orange"));
    const spade::SdfPose pose = pose_at(glm::vec3(0.0f, 1.8f, 0.0f), kTilt, 1.0f);
    scene.assets.push_back(visual_asset("gate_1", pose, "gates/ring_1500", "gate_orange"));

    const ComposedScene composed = spade::scene::compose(scene, world).value();
    ASSERT_EQ(composed.world.props.size(), world.props.size() + 1);
    const spade::PropDesc& prop = composed.world.props.back();
    EXPECT_EQ(prop.mesh_ref, "gates/ring_1500");
    EXPECT_EQ(prop.material, 1u);
    EXPECT_EQ(prop.pose.position, pose.position);
    EXPECT_EQ(prop.pose.rotation, pose.rotation);
    EXPECT_EQ(prop.pose.scale, pose.scale);
    EXPECT_EQ(config_hash_of(composed.world), config_hash_of(world));  // render data only
}

// ===========================================================================
// compose(): models, vehicles and capacities
// ===========================================================================

// Each field is the world's count, plus what the vehicles need, plus spare;
// spare defaults to 0.
TEST(SceneCompose, CapacitiesAreTheWorldsPlusTheVehiclesNeedPlusSpare) {
    const spade::WorldDesc world = ground_world();  // {1, 2, 3, 4}
    const spade::vehicles::ModelType quad = quad_model("quad");
    const auto force = static_cast<uint32_t>(quad.rotors.size() + quad.drag_bodies.size());
    const auto sensors = static_cast<uint32_t>(quad.imu_mounts.size());
    ASSERT_GT(force, 0u);
    ASSERT_GT(sensors, 0u);

    SceneDesc scene = scene_over(world);
    scene.models.push_back(quad);
    scene.vehicles.push_back(vehicle("q1", "quad", glm::vec3(0.0f, 2.0f, 0.0f)));
    scene.vehicles.push_back(vehicle("q2", "quad", glm::vec3(3.0f, 2.0f, 0.0f)));

    const spade::Capacities no_spare = spade::scene::compose(scene, world).value().world.capacities;
    EXPECT_EQ(no_spare.bodies, 1u + 2u);
    EXPECT_EQ(no_spare.force_elements, 2u + 2u * force);
    EXPECT_EQ(no_spare.sensors, 3u + 2u * sensors);
    EXPECT_EQ(no_spare.contacts, 4u);

    scene.spare = spade::scene::SceneSpare{5, 6, 7, 8};
    const spade::Capacities spared = spade::scene::compose(scene, world).value().world.capacities;
    EXPECT_EQ(spared.bodies, 1u + 2u + 5u);
    EXPECT_EQ(spared.force_elements, 2u + 2u * force + 6u);
    EXPECT_EQ(spared.sensors, 3u + 2u * sensors + 7u);
    EXPECT_EQ(spared.contacts, 4u + 8u);
}

TEST(SceneCompose, ACapacityBeyondTwoToThe32IsRefused) {
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(visual_asset("marker", spade::SdfPose{}, "marker", "default"));
    scene.spare.bodies = std::numeric_limits<uint32_t>::max();
    const auto composed = spade::scene::compose(scene, world);
    ASSERT_FALSE(composed.has_value());
    EXPECT_EQ(composed.error().code, spade::Code::capacity_exceeded);
    EXPECT_TRUE(contains(composed.error().context, "bodies")) << composed.error().context;
}

// Models keep the models: section's order, so reordering vehicles changes
// only the placements' indices.
TEST(SceneCompose, ReorderingVehiclesDoesNotRenumberModels) {
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.models.push_back(quad_model("alpha"));
    scene.models.push_back(quad_model("beta"));
    scene.vehicles.push_back(vehicle("b", "beta", glm::vec3(0.0f, 2.0f, 0.0f)));
    scene.vehicles.push_back(vehicle("a", "alpha", glm::vec3(3.0f, 2.0f, 0.0f)));
    const ComposedScene first = spade::scene::compose(scene, world).value();
    std::swap(scene.vehicles[0], scene.vehicles[1]);
    const ComposedScene second = spade::scene::compose(scene, world).value();

    ASSERT_EQ(first.models.size(), 2u);
    ASSERT_EQ(second.models.size(), 2u);
    EXPECT_EQ(first.models[0].name, "alpha");
    EXPECT_EQ(second.models[0].name, "alpha");
    EXPECT_EQ(first.vehicles[0].model, 1u);
    EXPECT_EQ(first.vehicles[1].model, 0u);
    EXPECT_EQ(second.vehicles[0].model, 0u);
    EXPECT_EQ(second.vehicles[1].model, 1u);
}

// ===========================================================================
// compose(): order, determinism and identity
// ===========================================================================

TEST(SceneCompose, SwappingTwoAssetsChangesTheConfigHash) {
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(collider_asset("a", pose_at(glm::vec3(1.0f, 1.0f, 0.0f), kTilt, 1.0f), sphere_collider(0.5f)));
    scene.assets.push_back(collider_asset("b", pose_at(glm::vec3(-1.0f, 1.0f, 0.0f), kTilt, 1.0f), sphere_collider(0.3f)));
    const uint64_t before = config_hash_of(spade::scene::compose(scene, world).value().world);
    std::swap(scene.assets[0], scene.assets[1]);
    const uint64_t after = config_hash_of(spade::scene::compose(scene, world).value().world);
    EXPECT_NE(before, after);
}

TEST(SceneCompose, ComposingTwiceGivesTheSameWorldBytes) {
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.materials.push_back(named_material("gate_orange"));
    SceneAsset named = collider_asset("named", pose_at(glm::vec3(-1.0f, 1.0f, 0.0f), kTilt, 0.7f), sphere_collider(0.5f));
    named.collider_materials = {"gate_orange"};
    named.visual = {"gates/ring", "gate_orange"};
    scene.assets.push_back(named);
    scene.models.push_back(quad_model("quad"));
    scene.vehicles.push_back(vehicle("q1", "quad", glm::vec3(0.0f, 2.0f, 0.0f)));
    EXPECT_EQ(yaml_of(spade::scene::compose(scene, world).value().world),
              yaml_of(spade::scene::compose(scene, world).value().world));
}

// ===========================================================================
// compose(): refusals, each naming its cause
// ===========================================================================

TEST(SceneCompose, AWorldHashMismatchIsRefusedNamingBothHashes) {
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(visual_asset("marker", spade::SdfPose{}, "marker", "default"));
    scene.world.hash ^= 1u;
    const auto composed = spade::scene::compose(scene, world);
    ASSERT_FALSE(composed.has_value());
    EXPECT_EQ(composed.error().code, spade::Code::invalid_argument);
    const std::string& why = composed.error().context;
    EXPECT_TRUE(contains(why, "hash")) << why;
    EXPECT_TRUE(contains(why, "re-pin")) << why;
}

// world_hash() validates the world it hashes, so a world no WorldBuilder or
// loader would produce is refused before anything is composed.
TEST(SceneCompose, AWorldThatDoesNotValidateIsRefused) {
    spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(visual_asset("marker", spade::SdfPose{}, "marker", "default"));
    world.capacities.bodies = 0;  // validate_world_desc refuses a zero capacity
    const auto composed = spade::scene::compose(scene, world);
    ASSERT_FALSE(composed.has_value());
    EXPECT_EQ(composed.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(composed.error().context, "does not validate")) << composed.error().context;
}

TEST(SceneCompose, ASceneMaterialMayNotReuseAWorldMaterialsName) {
    const spade::WorldDesc world = ground_world();  // palette: "default"
    SceneDesc scene = scene_over(world);
    scene.materials.push_back(named_material("default"));
    scene.assets.push_back(visual_asset("marker", spade::SdfPose{}, "marker", "default"));
    const auto composed = spade::scene::compose(scene, world);
    ASSERT_FALSE(composed.has_value());
    EXPECT_EQ(composed.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(composed.error().context, "reuses the name")) << composed.error().context;
}

TEST(SceneCompose, AnUnknownMaterialNameIsRefusedForAVisualAndForANode) {
    const spade::WorldDesc world = ground_world();
    SceneDesc visual = scene_over(world);
    visual.assets.push_back(visual_asset("marker", spade::SdfPose{}, "marker", "no_such_material"));
    const auto by_visual = spade::scene::compose(visual, world);
    ASSERT_FALSE(by_visual.has_value());
    EXPECT_EQ(by_visual.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(by_visual.error().context, "no_such_material")) << by_visual.error().context;

    SceneDesc node = scene_over(world);
    SceneAsset named = collider_asset("ball", pose_at(glm::vec3(0.0f, 1.0f, 0.0f), kTilt, 1.0f), sphere_collider(0.5f));
    named.collider_materials = {"no_such_material"};
    node.assets.push_back(named);
    const auto by_node = spade::scene::compose(node, world);
    ASSERT_FALSE(by_node.has_value());
    EXPECT_EQ(by_node.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(by_node.error().context, "no_such_material")) << by_node.error().context;
}

// A world palette may repeat a name (it is addressed by index), but a scene
// refers by name, so a repeated name is refused when, and only when, used.
TEST(SceneCompose, AMaterialNameTheWorldHoldsTwiceIsRefusedOnlyWhenUsed) {
    const spade::WorldDesc world =
        ground_builder().material(named_material("steel")).material(named_material("steel")).build().value();
    SceneDesc unused = scene_over(world);
    unused.assets.push_back(collider_asset("ball", pose_at(glm::vec3(0.0f, 1.0f, 0.0f), kTilt, 1.0f), sphere_collider(0.5f)));
    EXPECT_TRUE(spade::scene::compose(unused, world).has_value());

    SceneDesc used = scene_over(world);
    used.assets.push_back(visual_asset("beam", spade::SdfPose{}, "beam", "steel"));
    const auto composed = spade::scene::compose(used, world);
    ASSERT_FALSE(composed.has_value());
    EXPECT_EQ(composed.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(composed.error().context, "twice")) << composed.error().context;
}

// compose() runs validate_scene() itself, because a SceneDesc can be built in
// memory without the reader.
TEST(SceneCompose, ASceneTheValidatorRefusesIsRefused) {
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.vehicles.push_back(vehicle("q1", "no_such_model", glm::vec3(0.0f, 2.0f, 0.0f)));
    const auto composed = spade::scene::compose(scene, world);
    ASSERT_FALSE(composed.has_value());
    EXPECT_EQ(composed.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(composed.error().context, "no_such_model")) << composed.error().context;
}

// An asset at the depth limit on its own is valid, but joined to a world with
// geometry it needs one more stack slot: the composed world is refused.
TEST(SceneCompose, AnAssetThatDeepensTheProgramPastTheLimitIsRefused) {
    spade::SdfProgram deep;
    deep.transforms.push_back(SdfTransform{});
    for (uint32_t i = 0; i < spade::kMaxSdfDepth; ++i) {
        deep.nodes.push_back(primitive(spade::SdfPrim::sphere, glm::vec4(0.1f, 0.0f, 0.0f, 0.0f)));
    }
    for (uint32_t i = 1; i < spade::kMaxSdfDepth; ++i) {
        spade::SdfNode join{};
        join.op = static_cast<uint32_t>(spade::SdfOp::union_);
        deep.nodes.push_back(join);
    }
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.assets.push_back(collider_asset("deep", pose_at(glm::vec3(0.0f, 1.0f, 0.0f), kTilt, 1.0f), deep));
    ASSERT_TRUE(spade::scene::validate_scene(scene).has_value());

    const auto composed = spade::scene::compose(scene, world);
    ASSERT_FALSE(composed.has_value());
    EXPECT_EQ(composed.error().code, spade::Code::capacity_exceeded);
}

// ===========================================================================
// compose_file()
// ===========================================================================

// A scene finds its world relative to its own directory, and composing from
// files gives what composing the loaded pair gives.
TEST(SceneComposeFile, ASceneFindsItsWorldRelativeToItself) {
    const std::filesystem::path dir = scratch_dir("relative");
    std::filesystem::create_directories(dir / "worlds");
    std::filesystem::create_directories(dir / "scenes");
    ASSERT_TRUE(spade::save_world_file(ground_world(), dir / "worlds" / "ground.world.yaml").has_value());
    const spade::WorldDesc loaded = spade::load_world_file(dir / "worlds" / "ground.world.yaml").value();

    SceneDesc scene = scene_over(loaded);
    scene.world.file = "../worlds/ground.world.yaml";
    scene.assets.push_back(collider_asset("ball", pose_at(glm::vec3(0.0f, 1.0f, 0.0f), kTilt, 1.0f), sphere_collider(0.5f)));
    const std::filesystem::path scene_file = dir / "scenes" / "ball.scene.yaml";
    write_text(scene_file, spade::scene::scene_to_yaml(scene).value());

    const auto from_files = spade::scene::compose_file(scene_file);
    ASSERT_TRUE(from_files.has_value()) << from_files.error().context;
    EXPECT_EQ(yaml_of(from_files->world), yaml_of(spade::scene::compose(scene, loaded).value().world));

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// The hash is checked on this path too: a world edited after the scene was
// written is refused, and the error names the scene file.
TEST(SceneComposeFile, AWorldChangedAfterTheSceneIsRefused) {
    const std::filesystem::path dir = scratch_dir("changed");
    const std::filesystem::path world_file = dir / "ground.world.yaml";
    ASSERT_TRUE(spade::save_world_file(ground_world(), world_file).has_value());
    SceneDesc scene = scene_over(spade::load_world_file(world_file).value());
    scene.assets.push_back(visual_asset("marker", spade::SdfPose{}, "marker", "default"));
    const std::filesystem::path scene_file = dir / "marker.scene.yaml";
    write_text(scene_file, spade::scene::scene_to_yaml(scene).value());

    const spade::WorldDesc edited = ground_builder().sphere(0.5f, pose_at(glm::vec3(0.0f, 3.0f, 0.0f), kTilt, 1.0f)).union_().build().value();
    ASSERT_TRUE(spade::save_world_file(edited, world_file).has_value());

    const auto composed = spade::scene::compose_file(scene_file);
    ASSERT_FALSE(composed.has_value());
    EXPECT_EQ(composed.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(composed.error().context, "marker.scene.yaml")) << composed.error().context;
    EXPECT_TRUE(contains(composed.error().context, "hash")) << composed.error().context;

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

TEST(SceneComposeFile, AMissingWorldIsNamedWithTheScene) {
    const std::filesystem::path dir = scratch_dir("missing");
    SceneDesc scene = scene_over(ground_world());
    scene.world.file = "nowhere.world.yaml";
    scene.assets.push_back(visual_asset("marker", spade::SdfPose{}, "marker", "default"));
    const std::filesystem::path scene_file = dir / "orphan.scene.yaml";
    write_text(scene_file, spade::scene::scene_to_yaml(scene).value());

    const auto composed = spade::scene::compose_file(scene_file);
    ASSERT_FALSE(composed.has_value());
    EXPECT_EQ(composed.error().code, spade::Code::io_error);
    EXPECT_TRUE(contains(composed.error().context, "orphan.scene.yaml")) << composed.error().context;
    EXPECT_TRUE(contains(composed.error().context, "nowhere.world.yaml")) << composed.error().context;

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// ===========================================================================
// instantiate()
// ===========================================================================

// A start reaches spawn() exactly as the scene wrote it, in the design frame:
// instantiate() equals registering and spawning by hand. The model's design
// frame is not the identity, so a second conversion would show.
TEST(SceneInstantiate, StartsReachSpawnAsTheSceneWroteThem) {
    const spade::vehicles::ModelType quad =
        quad_model("tilted", glm::angleAxis(0.5f, glm::vec3(0.0f, 0.0f, 1.0f)), glm::vec3(0.01f, -0.02f, 0.005f));
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.models.push_back(quad);
    SceneVehicle v = vehicle("q1", "tilted", glm::vec3(0.5f, 2.0f, -0.25f));
    v.start.orient = glm::angleAxis(0.3f, glm::vec3(0.0f, 1.0f, 0.0f));
    v.start.vel = glm::vec3(0.1f, 0.0f, -0.2f);
    v.start.omega_body = glm::vec3(0.0f, 0.4f, 0.0f);
    scene.vehicles.push_back(v);
    const ComposedScene composed = spade::scene::compose(scene, world).value();

    spade::Result<spade::scene::SceneRun> run =
        spade::scene::instantiate(composed, instance_of(composed.world), kDtNs, kSubsteps);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    ASSERT_EQ(run->vehicles.size(), 1u);
    EXPECT_EQ(run->sim.model_count(), 1u);

    spade::Simulation by_hand =
        spade::Simulation::create(spade::WorldSetDesc{{instance_of(composed.world)}}, kDtNs, kSubsteps).value();
    const spade::ModelTypeId id = by_hand.register_model(quad).value();
    ASSERT_TRUE(by_hand.spawn(0, id, v.start).has_value());
    ASSERT_TRUE(by_hand.flush_structural().has_value());

    EXPECT_EQ(spade::testing::state_digest(run->sim), spade::testing::state_digest(by_hand));
}

// Vehicle order is configuration: two different vehicles swapped change the
// first state digest.
TEST(SceneInstantiate, SwappingTwoDifferentVehiclesChangesTheFirstDigest) {
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.models.push_back(quad_model("quad"));
    scene.vehicles.push_back(vehicle("q1", "quad", glm::vec3(0.0f, 2.0f, 0.0f)));
    scene.vehicles.push_back(vehicle("q2", "quad", glm::vec3(3.0f, 2.5f, 0.0f)));
    const uint64_t before = first_digest(spade::scene::compose(scene, world).value());
    std::swap(scene.vehicles[0], scene.vehicles[1]);
    const uint64_t after = first_digest(spade::scene::compose(scene, world).value());
    EXPECT_NE(before, after);
}

// Names are diagnostics only (DBE-010 puts order, not names, into the
// configuration): renaming a vehicle moves no world byte, no config hash and
// no digest.
TEST(SceneInstantiate, RenamingAVehicleMovesNoHashOrDigest) {
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.models.push_back(quad_model("quad"));
    scene.vehicles.push_back(vehicle("q1", "quad", glm::vec3(0.0f, 2.0f, 0.0f)));
    const ComposedScene first = spade::scene::compose(scene, world).value();
    scene.vehicles[0].name = "renamed";
    const ComposedScene second = spade::scene::compose(scene, world).value();

    EXPECT_EQ(yaml_of(first.world), yaml_of(second.world));
    EXPECT_EQ(config_hash_of(first.world), config_hash_of(second.world));
    EXPECT_EQ(first_digest(first), first_digest(second));
}

// A vehicle that cannot spawn is named, with spawn()'s own code.
TEST(SceneInstantiate, AVehicleThatDoesNotSpawnIsNamed) {
    const spade::WorldDesc world = ground_world();
    SceneDesc scene = scene_over(world);
    scene.models.push_back(quad_model("quad"));
    scene.vehicles.push_back(vehicle("buried", "quad", glm::vec3(0.0f, -1.0f, 0.0f)));  // under the ground plane
    const ComposedScene composed = spade::scene::compose(scene, world).value();

    const auto run = spade::scene::instantiate(composed, instance_of(composed.world), kDtNs, kSubsteps);
    ASSERT_FALSE(run.has_value());
    EXPECT_EQ(run.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(run.error().context, "vehicle 'buried'")) << run.error().context;
}
