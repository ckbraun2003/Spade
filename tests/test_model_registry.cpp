// test_model_registry.cpp -- the model registry's identity in a snapshot
// (snapshot format v3; Core's drone-builder plan, Task C). A snapshot carries
// the module set's identity (v2) and now the model registry's: every model
// registered, in order, with its version and every field that reaches state or
// the design frame. restore() refuses a blob taken under another registry (L2).

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "core/rng.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"
#include "world/mesh.hpp"
#include "world/sdf.hpp"

namespace {

// bench_sim.cpp's 1 kg airframe, under a given name.
[[nodiscard]] spade::vehicles::ModelType quad(std::string name) {
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
    return spade::vehicles::make_quadrotor(p).value();
}

[[nodiscard]] spade::Simulation one_world() {
    const spade::WorldDesc world = spade::WorldBuilder()
                                       .name("registry")
                                       .environment(spade::Environment{})
                                       .capacities(spade::Capacities{2, 16, 4, 1})
                                       .build()
                                       .value();
    spade::WorldInstanceDesc inst;
    inst.world = world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::Simulation::create(spade::WorldSetDesc{{inst}}, 2'000'000, 2).value();
}

[[nodiscard]] spade::Simulation with_models(std::initializer_list<spade::vehicles::ModelType> models) {
    spade::Simulation sim = one_world();
    for (const spade::vehicles::ModelType& m : models) EXPECT_TRUE(sim.register_model(m).has_value());
    return sim;
}

}  // namespace

TEST(ModelSnapshot, ABlobCarriesTheModelRegistrysIdentity) {
    auto sim = with_models({quad("a"), quad("b")});
    const auto blob = sim.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    const spade::vehicles::ModelType models[] = {quad("a"), quad("b")};
    EXPECT_EQ(blob->model_registry_identity(), spade::vehicles::model_registry_identity(models));
}

TEST(ModelSnapshot, RestoreIntoTheSameRegistrySucceeds) {
    auto source = with_models({quad("a"), quad("b")});
    auto target = with_models({quad("a"), quad("b")});
    ASSERT_TRUE(source.step(3).has_value());
    const auto blob = source.snapshot();
    ASSERT_TRUE(blob.has_value());
    EXPECT_TRUE(target.restore(*blob).has_value());
}

TEST(ModelSnapshot, RestoreIntoAnotherModelRegistryIsRefused) {
    auto source = with_models({quad("a"), quad("b")});
    const auto blob = source.snapshot();
    ASSERT_TRUE(blob.has_value());

    auto reordered = with_models({quad("b"), quad("a")});
    const auto r1 = reordered.restore(*blob);
    ASSERT_FALSE(r1.has_value()) << "registration order is part of the registry";
    EXPECT_EQ(r1.error().code, spade::Code::invalid_argument);
    EXPECT_NE(r1.error().context.find("model registry"), std::string::npos) << r1.error().context;

    spade::vehicles::ModelType b2 = quad("b");
    b2.version = 2;
    auto reversioned = with_models({quad("a"), b2});
    EXPECT_FALSE(reversioned.restore(*blob).has_value()) << "a model's version is part of the registry";

    auto fewer = with_models({quad("a")});
    EXPECT_FALSE(fewer.restore(*blob).has_value()) << "a missing model";

    // A fresh Simulation that registered nothing: the case consumers feel.
    // Register the same models before restore() (SCN-006).
    auto none = with_models({});
    EXPECT_FALSE(none.restore(*blob).has_value()) << "an empty registry is another registry";
}

TEST(ModelSnapshot, AVisualRefDoesNotEnterTheIdentity) {
    spade::vehicles::ModelType plain = quad("a");
    spade::vehicles::ModelType dressed = quad("a");
    dressed.visual_ref = "mesh:kat/5in_frame";
    const spade::vehicles::ModelType one[] = {plain};
    const spade::vehicles::ModelType other[] = {dressed};
    EXPECT_EQ(spade::vehicles::model_registry_identity(one), spade::vehicles::model_registry_identity(other))
        << "visual_ref is render data (L5); it never reaches the step";
}

// CORE-7: parts never enter model_identity, and the registry identity does
// not fold them until stage 7 (through parts_identity), so a v3 blob taken
// before a model gained parts restores into a registry where it has them.
// Green at red: a pin of "every stored v3 blob restores".
TEST(ModelSnapshot, ABlobRestoresAfterTheModelsGainParts) {
    auto source = with_models({quad("a")});
    ASSERT_TRUE(source.step(3).has_value());
    const auto blob = source.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;

    spade::SdfProgram box;
    box.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    node.params = glm::vec4(0.05f, 0.02f, 0.05f, 0.0f);
    box.nodes.push_back(node);
    using spade::vehicles::PartAttach;
    using spade::vehicles::PartDesc;
    using spade::vehicles::PartSurface;
    spade::vehicles::ModelType dressed = quad("a");
    dressed.parts.push_back(PartDesc{
        .name = "frame",
        .role = "body",
        .attach = PartAttach::body,
        .rotor = 0,
        .mount_pos = glm::vec3(0.0f),
        .mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        .shape = box,
        .surface = PartSurface{.kind = PartSurface::Kind::wall, .inv_permeability = 0.0f, .inertial_coeff = 0.0f},
        .min_feature = 0.004f});
    dressed.parts.push_back(PartDesc{
        .name = "canopy",
        .role = "body",
        .attach = PartAttach::body,
        .rotor = 0,
        .mount_pos = glm::vec3(0.0f, 0.04f, 0.0f),
        .mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        .shape = box,
        .surface = PartSurface{.kind = PartSurface::Kind::porous, .inv_permeability = 1.0e6f, .inertial_coeff = 20.0f},
        .min_feature = 0.004f});
    auto target = with_models({dressed});
    const auto restored = target.restore(*blob);
    EXPECT_TRUE(restored.has_value()) << (restored ? "" : restored.error().context);
}

// CORE-6, CORE-7: a mesh part is no different (parts_identity folds its
// content_hash(), and stage 7 folds that into the registry identity), so a
// v3 blob taken before a model gained one restores into a registry where it
// has it.
TEST(ModelSnapshot, ABlobRestoresAfterAModelGainsAMeshPart) {
    auto source = with_models({quad("a")});
    ASSERT_TRUE(source.step(3).has_value());
    const auto blob = source.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;

    // A 10 cm right-corner tetrahedron, counter-clockwise seen from outside.
    const spade::MeshInput input{
        .positions = {{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, {0.0, 0.1, 0.0}, {0.0, 0.0, 0.1}},
        .indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3},
        .unit = spade::MeshUnit::metre};
    const auto mesh = spade::make_mesh(input, "wedge");
    ASSERT_TRUE(mesh.has_value()) << mesh.error().context;

    using spade::vehicles::PartAttach;
    using spade::vehicles::PartDesc;
    using spade::vehicles::PartSurface;
    spade::vehicles::ModelType dressed = quad("a");
    dressed.parts.push_back(PartDesc{
        .name = "wedge",
        .role = "body",
        .attach = PartAttach::body,
        .rotor = 0,
        .mount_pos = glm::vec3(0.0f, 0.03f, 0.0f),
        .mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        .shape = spade::SdfProgram{},
        .mesh = *mesh,
        .surface = PartSurface{.kind = PartSurface::Kind::wall, .inv_permeability = 0.0f, .inertial_coeff = 0.0f},
        .min_feature = 0.004f});
    auto target = with_models({dressed});
    const auto restored = target.restore(*blob);
    EXPECT_TRUE(restored.has_value()) << (restored ? "" : restored.error().context);
}

TEST(ModelSnapshot, TheDesignFrameEntersTheIdentity) {
    spade::vehicles::ModelType plain = quad("a");
    spade::vehicles::ModelType offset = quad("a");
    offset.com_offset = glm::vec3(0.0f, 0.01f, 0.0f);
    const spade::vehicles::ModelType one[] = {plain};
    const spade::vehicles::ModelType other[] = {offset};
    EXPECT_NE(spade::vehicles::model_registry_identity(one), spade::vehicles::model_registry_identity(other));
}

// A model's identity is the per-model fold the airframe compiler and Kat hash a
// compiled airframe by; the registry's is the count, then each model's
// identity, in registration order (vehicles/model_identity.hpp).
TEST(ModelIdentity, EachModelHasOneAndTheRegistryFoldsThemInOrder) {
    using spade::vehicles::model_identity;
    using spade::vehicles::model_registry_identity;
    const spade::vehicles::ModelType a = quad("a");
    spade::vehicles::ModelType dressed = quad("a");
    dressed.visual_ref = "mesh:kat/5in_frame";
    spade::vehicles::ModelType heavier = quad("a");
    heavier.body.mass *= 2.0f;
    EXPECT_EQ(model_identity(a), model_identity(dressed)) << "visual_ref is not folded (L5)";
    EXPECT_NE(model_identity(a), model_identity(heavier));
    EXPECT_NE(model_identity(a), model_identity(quad("b"))) << "the name is folded";

    // FNV-1a 64 over the count (u32 LE), then each model_identity (u64 LE).
    const spade::vehicles::ModelType two[] = {a, heavier};
    uint64_t h = spade::rng::kFnv1aOffsetBasis;
    const auto fold = [&h](uint64_t value, int bytes) {
        for (int i = 0; i < bytes; ++i) {
            h ^= static_cast<uint8_t>(value >> (8 * i));
            h *= spade::rng::kFnv1aPrime;
        }
    };
    fold(2, 4);
    fold(model_identity(a), 8);
    fold(model_identity(heavier), 8);
    EXPECT_EQ(model_registry_identity(two), h);
}
