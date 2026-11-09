// test_parts_fold.cpp -- the gated parts_identity fold (module API stage 7, Task C1; L35 question 1) and the blob pin.
//
// parts_identity() folds into the snapshot's registry identity ONLY for a set that declares a part table (a configuration table
// named kPartTableName, "parts"): CORE-8's "once a step reads it". Every set today declares none, so nothing existing moves.
// The fold is a tail after the configuration tables' tail: when the schedule declares a part table and at least one registered
// model has parts, FNV-1a 64 over the byte 0xFD, the five ASCII bytes "parts", the identity so far as 8 bytes little-endian, the
// count of models with parts (u32), then per such model, in registration order, its registration index (u32) and its
// parts_identity() as 8 bytes little-endian.
//
// Green at red (pins, named in their comments): a set without a part table keeps its registry identity; the standard blob.
#include <gtest/gtest.h>

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "core/rng.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/sdf.hpp"

namespace {

using spade::modules::ModuleDesc;
using spade::modules::ModuleSet;
using spade::vehicles::ModelType;
using spade::vehicles::PartAttach;
using spade::vehicles::PartDesc;
using spade::vehicles::PartSurface;

[[nodiscard]] PartDesc part(std::string name, float radius) {
    spade::SdfNode n{};
    n.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.params = glm::vec4(radius, 0.0f, 0.0f, 0.0f);
    spade::SdfProgram program;
    program.transforms.push_back(spade::SdfTransform{});
    program.nodes = {n};
    return PartDesc{.name = std::move(name),
                    .role = "arm",
                    .attach = PartAttach::body,
                    .rotor = 0,
                    .mount_pos = glm::vec3(0.0f),
                    .mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                    .shape = std::move(program),
                    .mesh = nullptr,
                    .surface = PartSurface{.kind = PartSurface::Kind::wall, .inv_permeability = 0.0f, .inertial_coeff = 0.0f},
                    .min_feature = 0.005f};
}

[[nodiscard]] ModelType quad_with(std::vector<PartDesc> parts) {
    spade::vehicles::QuadrotorParams p;
    p.name = "fold_quad";
    p.mass = 1.0f;
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f);
    p.arm_length = 0.19f;
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
    ModelType m = spade::vehicles::make_quadrotor(p).value();
    m.parts = std::move(parts);
    return m;
}

// A table of this name declares a part table; its build appends nothing, so the configuration-table fold stays out of it.
void build_nothing(std::span<const ModelType>, std::vector<float>&) {}
constexpr spade::modules::ConfigTableDecl kPartTable[] = {{.name = spade::modules::kPartTableName, .build = &build_nothing}};

[[nodiscard]] ModuleSet standard_plus_part_table() {
    ModuleSet set = spade::modules::standard_modules();
    set.push_back(ModuleDesc{.name = "partful", .tables = kPartTable});
    return set;
}

[[nodiscard]] spade::Simulation make(const ModuleSet& set) {
    const spade::Result<spade::WorldSetDesc> world = spade::test::floor_world_set(2);
    EXPECT_TRUE(world.has_value()) << world.error().context;
    spade::Result<spade::Simulation> sim = spade::Simulation::create(*world, 1'000'000, 2, spade::compute::BackendDesc{}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

// The registry identity as a blob carries it (Simulation::registry_identity() is private).
[[nodiscard]] uint64_t registry_of(const spade::Simulation& sim) {
    const auto blob = sim.snapshot();
    EXPECT_TRUE(blob.has_value()) << blob.error().context;
    return blob->model_registry_identity();
}

[[nodiscard]] uint64_t fold(uint64_t h, uint8_t b) { return (h ^ b) * spade::rng::kFnv1aPrime; }

}  // namespace

// PIN, green at red: a set without a part table keeps the registry identity it has, parts or not (CORE-7 and L35).
TEST(PartsFold, ASetWithoutAPartTableKeepsItsRegistryIdentity) {
    spade::Simulation bare = make(spade::modules::standard_modules());
    spade::Simulation dressed = make(spade::modules::standard_modules());
    ASSERT_TRUE(bare.register_model(quad_with({})).has_value());
    ASSERT_TRUE(dressed.register_model(quad_with({part("arm0", 0.01f), part("canopy", 0.02f)})).has_value());
    EXPECT_EQ(registry_of(bare), registry_of(dressed));
    EXPECT_EQ(registry_of(dressed),
              spade::vehicles::model_registry_identity(std::vector<ModelType>{quad_with({part("arm0", 0.01f), part("canopy", 0.02f)})}));
}

TEST(PartsFold, APartsModelChangesTheRegistryIdentityOfAPartTableSetOnly) {
    spade::Simulation bare = make(standard_plus_part_table());
    spade::Simulation dressed = make(standard_plus_part_table());
    ASSERT_TRUE(bare.register_model(quad_with({})).has_value());
    ASSERT_TRUE(dressed.register_model(quad_with({part("arm0", 0.01f)})).has_value());
    EXPECT_NE(registry_of(bare), registry_of(dressed)) << "parts reach the identity of a part-table set";

    // A model without parts folds nothing: the identity is the models' own.
    EXPECT_EQ(registry_of(bare), spade::vehicles::model_registry_identity(std::vector<ModelType>{quad_with({})}));
}

TEST(PartsFold, TheFoldIsSpeltByteByByteInRegistrationOrder) {
    spade::Simulation sim = make(standard_plus_part_table());
    const ModelType first = quad_with({part("arm0", 0.01f)});
    ASSERT_TRUE(sim.register_model(quad_with({})).has_value());  // index 0: no parts, not folded
    ASSERT_TRUE(sim.register_model(first).has_value());          // index 1
    const uint64_t models = spade::vehicles::model_registry_identity(std::vector<ModelType>{quad_with({}), first});

    uint64_t h = spade::rng::kFnv1aOffsetBasis;
    h = fold(h, 0xFD);
    for (const char c : std::string_view("parts")) h = fold(h, static_cast<uint8_t>(c));
    for (int i = 0; i < 8; ++i) h = fold(h, static_cast<uint8_t>(models >> (8 * i)));
    for (int i = 0; i < 4; ++i) h = fold(h, static_cast<uint8_t>(1u >> (8 * i)));  // one model with parts
    for (int i = 0; i < 4; ++i) h = fold(h, static_cast<uint8_t>(1u >> (8 * i)));  // its registration index
    const uint64_t parts = spade::vehicles::parts_identity(first);
    for (int i = 0; i < 8; ++i) h = fold(h, static_cast<uint8_t>(parts >> (8 * i)));
    EXPECT_EQ(registry_of(sim), h);
}

TEST(PartsFold, ARestoreUnderOtherPartsIsRefusedNamingBothIdentities) {
    spade::Simulation a = make(standard_plus_part_table());
    ASSERT_TRUE(a.register_model(quad_with({part("arm0", 0.01f)})).has_value());
    const auto blob = a.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;

    spade::Simulation same = make(standard_plus_part_table());
    ASSERT_TRUE(same.register_model(quad_with({part("arm0", 0.01f)})).has_value());
    EXPECT_TRUE(same.restore(*blob).has_value());

    spade::Simulation other = make(standard_plus_part_table());
    ASSERT_TRUE(other.register_model(quad_with({part("arm0", 0.02f)})).has_value());  // another part shape
    const auto refused = other.restore(*blob);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find(std::to_string(registry_of(a))), std::string::npos) << refused.error().context;
    EXPECT_NE(refused.error().context.find(std::to_string(registry_of(other))), std::string::npos) << refused.error().context;
}

// PIN, green at red: a blob taken from a standard set is byte for byte what master wrote. The literal is the FNV-1a 64 of the
// whole blob (the blob's identity header included) measured at the batch C red head, which is master's behaviour.
TEST(Snapshot, AStandardBlobIsByteIdenticalAfterStage7) {
    spade::Simulation sim = make(spade::modules::standard_modules());
    ASSERT_TRUE(sim.step(3).has_value());
    const auto blob = sim.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    uint64_t h = spade::rng::kFnv1aOffsetBasis;
    for (const std::byte b : blob->bytes()) h = fold(h, static_cast<uint8_t>(b));
    constexpr uint64_t kMasterBlobFnv = 0xd308a402feb67728ull;  // measured at the RED head (no batch C engine code), 2026-11-08
    EXPECT_EQ(h, kMasterBlobFnv) << "measured: 0x" << std::hex << h;
}
