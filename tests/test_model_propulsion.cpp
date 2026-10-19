// test_model_propulsion.cpp -- a model's propulsion chain (N5a; the editor
// end-goal design, section 8). ModelType::propulsion is the model's host-side
// motor, ESC, battery and propeller chain, the input of
// rotor_omega_for_duty(), so every copy of a model carries it.
//
// What is pinned here:
//   - THE ONE RULE (TD-9). validate() and issues() check a set chain by
//     vehicles::validate_propulsion_chain(), last, as it is declared: one
//     issue at the model's field "propulsion", the chain's message prefixed by
//     the model's name. register_model() refuses the same chain.
//   - THREE INDEPENDENT IDENTITIES (the lead's condition, 2026-10-19).
//     model_identity() and parts_identity() exclude the chain explicitly:
//     chain A, chain B and none leave both exactly where they are, while
//     propulsion_identity() tells A from B. Their pinned values are
//     AirframeCompile.TheReferenceQuadHasAPinnedIdentity,
//     AirframeCompile.PartsMoveOnlyThePartsIdentity and
//     PropulsionIdentity.AFixedChainHasAPinnedValue, unedited.
//   - THE REGISTRY IGNORES IT. A model with a chain and the same model without
//     one step to the same snapshot bytes, and a blob restores across them.
//   - DESIGNATED INITIALIZERS. The member is last and defaulted, so a
//     ModelType built by designated initializer need not name it.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "reference_airframe.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/propulsion_steady.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace {

using spade::vehicles::ModelIssue;
using spade::vehicles::ModelType;
using spade::vehicles::PropulsionChain;

// The refusal's context, or a marker when the call unexpectedly succeeded.
template <class T>
[[nodiscard]] std::string why(const spade::Result<T>& r) {
    return r ? std::string("<succeeded>") : r.error().context;
}

// The reference quad (tests/reference_airframe.hpp), compiled: a real model
// with parts, so its parts_identity is not the no-parts constant.
[[nodiscard]] ModelType reference_model() {
    const auto compiled = spade::vehicles::compile_airframe(spade::vehicles::fixtures::reference_quad());
    EXPECT_TRUE(compiled.has_value()) << why(compiled);
    return compiled ? compiled->model : ModelType{};
}

// The chain the reference quad's blocks describe.
[[nodiscard]] PropulsionChain reference_chain() {
    const auto chain = spade::vehicles::airframe_propulsion_chain(spade::vehicles::fixtures::reference_quad());
    EXPECT_TRUE(chain.has_value()) << why(chain);
    return chain.value_or(PropulsionChain{});
}

// A valid chain, every value a literal (TD-3: no libm): the model file's
// pinned chain (tests/test_model_file.cpp).
[[nodiscard]] PropulsionChain literal_chain() {
    PropulsionChain c;
    c.kv = 256.56340004316644;  // 2450 rpm/V
    c.resistance = 0.075;
    c.no_load_current = 1.2;
    c.current_max = 50.0;
    c.esc_current_total_max = 160.0;
    c.pole_pairs = 7.0;
    c.inductance = 1.5e-5;
    c.diameter = 0.1524;
    c.table.j_min = 0.0f;
    c.table.j_max = 0.8f;
    c.table.ct = {0.12f, 0.04f};
    c.table.cq = {0.008f, 0.004f};
    c.cells_series = 4;
    c.cells_parallel = 1;
    c.cell_resistance = 0.005;
    c.cell_cutoff_voltage = 3.0;
    c.pack_polarization_resistance = 0.002;
    c.pack_current_max = 120.0;
    c.ocv_table = {3.3f, 4.2f};
    c.motors_on_bus = 4;
    return c;
}

// One world with room for the reference quad.
[[nodiscard]] spade::Simulation one_world() {
    const spade::WorldDesc world = spade::WorldBuilder()
                                       .name("propulsion")
                                       .environment(spade::Environment{})
                                       .capacities(spade::Capacities{2, 16, 4, 1})
                                       .build()
                                       .value();
    spade::WorldInstanceDesc inst;
    inst.world = world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::Simulation::create(spade::WorldSetDesc{{inst}}, 2'000'000, 2).value();
}

}  // namespace

// ===========================================================================
// The member
// ===========================================================================

TEST(ModelPropulsion, AModelHasNoChainUntilOneIsSet) {
    EXPECT_FALSE(ModelType{}.propulsion.has_value());
    ModelType m = reference_model();
    m.propulsion = reference_chain();
    ASSERT_TRUE(m.propulsion.has_value());
    const ModelType copy = m;
    ASSERT_TRUE(copy.propulsion.has_value()) << "a copy of the model carries its chain";
    EXPECT_EQ(spade::vehicles::propulsion_identity(*copy.propulsion),
              spade::vehicles::propulsion_identity(*m.propulsion));
}

TEST(ModelPropulsion, ADesignatedInitializerNeedNotNameIt) {
    // The member is last and defaulted: an initializer written before it
    // existed still compiles, and leaves it unset. gcc's
    // -Wmissing-field-initializers (an error under -Werror) names each member
    // WITHOUT a default member initializer that a designated initializer
    // omits; visual_ref, rotors, drag_bodies, imu_mounts and parts have none,
    // so they are named here. `propulsion`, like `version` and `body`, is
    // defaulted, so it need not be.
    const ModelType plain{.name = "plain",
                          .visual_ref = "",
                          .version = 2,
                          .body = {.mass = 1.5f, .inertia_diag = glm::vec3(0.02f, 0.03f, 0.02f)},
                          .proxy_radius = 0.1f,
                          .rotors = {},
                          .drag_bodies = {},
                          .imu_mounts = {},
                          .parts = {}};
    EXPECT_FALSE(plain.propulsion.has_value());
    EXPECT_TRUE(plain.validate().has_value()) << why(plain.validate());

    // And one that names it, last.
    const ModelType powered{.name = "powered",
                            .visual_ref = "",
                            .body = {.mass = 1.5f, .inertia_diag = glm::vec3(0.02f, 0.03f, 0.02f)},
                            .rotors = {},
                            .drag_bodies = {},
                            .imu_mounts = {},
                            .parts = {},
                            .propulsion = literal_chain()};
    ASSERT_TRUE(powered.propulsion.has_value());
    EXPECT_EQ(powered.propulsion->motors_on_bus, 4u);
    EXPECT_TRUE(powered.validate().has_value()) << why(powered.validate());
}

// ===========================================================================
// The one rule (TD-9)
// ===========================================================================

TEST(ModelPropulsion, AValidChainPasses) {
    ModelType m = reference_model();
    m.propulsion = reference_chain();
    EXPECT_TRUE(m.validate().has_value()) << why(m.validate());
    EXPECT_TRUE(m.issues().empty());
    m.propulsion = literal_chain();
    EXPECT_TRUE(m.validate().has_value()) << why(m.validate());
}

TEST(ModelPropulsion, ABadChainIsRefusedNamingTheModelAndTheField) {
    struct Case {
        const char* what;
        std::function<void(PropulsionChain&)> edit;
        const char* field;
    };
    const std::vector<Case> cases = {
        {"kv 0", [](PropulsionChain& c) { c.kv = 0.0; }, "kv must be finite and > 0"},
        {"a NaN resistance", [](PropulsionChain& c) { c.resistance = std::numeric_limits<double>::quiet_NaN(); },
         "resistance must be finite and > 0"},
        {"a ct and cq size mismatch", [](PropulsionChain& c) { c.table.cq.push_back(0.002f); },
         "table.cq must hold as many values as table.ct"},
        {"motors_on_bus 0", [](PropulsionChain& c) { c.motors_on_bus = 0; }, "motors_on_bus must be in 1 to 64"},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        ModelType m = reference_model();
        PropulsionChain bad = reference_chain();
        c.edit(bad);
        const spade::Result<void> chain = spade::vehicles::validate_propulsion_chain(bad);
        ASSERT_FALSE(chain) << "the chain's own rule refuses it";
        m.propulsion = bad;

        // The chain's message, prefixed by the model's name as every other
        // message of validate() is.
        const std::string expected = "model type '" + m.name + "': " + chain.error().context;
        EXPECT_NE(expected.find(c.field), std::string::npos) << expected;
        const spade::Result<void> r = m.validate();
        ASSERT_FALSE(r);
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
        EXPECT_EQ(r.error().context, expected);

        // One issue, the model's own, at "propulsion".
        const std::vector<ModelIssue> issues = m.issues();
        ASSERT_EQ(issues.size(), 1u);
        EXPECT_EQ(issues[0].element, ModelIssue::Element::model);
        EXPECT_EQ(issues[0].index, 0u);
        EXPECT_EQ(issues[0].field, "propulsion");
        EXPECT_EQ(issues[0].message, expected);
        EXPECT_EQ(issues[0].code, spade::Code::invalid_argument);
    }
}

TEST(ModelPropulsion, TheChainIsCheckedLastAsItIsDeclared) {
    ModelType m = reference_model();
    m.body.mass = 0.0f;
    PropulsionChain bad = reference_chain();
    bad.kv = -1.0;
    m.propulsion = bad;
    const std::vector<ModelIssue> issues = m.issues();
    ASSERT_EQ(issues.size(), 2u);
    EXPECT_EQ(issues[0].field, "body.mass");
    EXPECT_EQ(issues[1].field, "propulsion");
    EXPECT_EQ(issues[1].message, "model type '" + m.name + "': propulsion chain: kv must be finite and > 0");
    const spade::Result<void> r = m.validate();
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().context, issues[0].message) << "validate() is the first issue";
}

TEST(ModelPropulsion, RegisterModelRefusesABadChain) {
    ModelType m = reference_model();
    PropulsionChain bad = reference_chain();
    bad.cells_series = 0;
    m.propulsion = bad;
    spade::Simulation sim = one_world();
    const auto refused = sim.register_model(m);
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(refused.error().context, "model type '" + m.name + "': propulsion chain: cells_series must be > 0");

    m.propulsion = reference_chain();
    EXPECT_TRUE(sim.register_model(m).has_value());
}

// ===========================================================================
// Three independent identities (the lead's condition)
// ===========================================================================

TEST(ModelPropulsion, TheChainEntersNeitherTheModelNorThePartsIdentity) {
    const ModelType none = reference_model();
    ASSERT_FALSE(none.propulsion.has_value());
    ASSERT_NE(spade::vehicles::parts_identity(none), spade::vehicles::kNoPartsIdentity)
        << "the reference quad has parts, so the parts identity is not vacuous";

    const PropulsionChain a = reference_chain();
    PropulsionChain b = a;
    b.kv = a.kv * 0.5;                    // another motor
    b.cells_series = a.cells_series + 2;  // and another pack
    const spade::Result<void> b_valid = spade::vehicles::validate_propulsion_chain(b);
    ASSERT_TRUE(b_valid.has_value()) << why(b_valid);
    EXPECT_NE(spade::vehicles::propulsion_identity(a), spade::vehicles::propulsion_identity(b))
        << "the chain's own identity tells the two apart";

    // A model with chain A, then the same model with B swapped in, then with
    // none: the model and parts identities stay exactly where they are.
    ModelType swapped = none;
    swapped.propulsion = a;
    const uint64_t model_with_a = spade::vehicles::model_identity(swapped);
    const uint64_t parts_with_a = spade::vehicles::parts_identity(swapped);
    swapped.propulsion = b;
    const uint64_t model_with_b = spade::vehicles::model_identity(swapped);
    const uint64_t parts_with_b = spade::vehicles::parts_identity(swapped);
    swapped.propulsion.reset();
    const uint64_t model_with_none = spade::vehicles::model_identity(swapped);
    const uint64_t parts_with_none = spade::vehicles::parts_identity(swapped);

    EXPECT_EQ(model_with_a, spade::vehicles::model_identity(none)) << std::hex << model_with_a;
    EXPECT_EQ(model_with_b, model_with_a) << std::hex << model_with_b;
    EXPECT_EQ(model_with_none, model_with_a) << std::hex << model_with_none;
    EXPECT_EQ(parts_with_a, spade::vehicles::parts_identity(none)) << std::hex << parts_with_a;
    EXPECT_EQ(parts_with_b, parts_with_a) << std::hex << parts_with_b;
    EXPECT_EQ(parts_with_none, parts_with_a) << std::hex << parts_with_none;

    // The registry's identity folds model_identity, so it does not move either.
    ModelType with_a = none;
    with_a.propulsion = a;
    const ModelType bare_set[] = {none};
    const ModelType powered_set[] = {with_a};
    EXPECT_EQ(spade::vehicles::model_registry_identity(std::span<const ModelType>(powered_set)),
              spade::vehicles::model_registry_identity(std::span<const ModelType>(bare_set)));
}

// ===========================================================================
// The Simulation's registry ignores it
// ===========================================================================

TEST(ModelPropulsion, TheSimulationRegistryIgnoresIt) {
    const ModelType bare = reference_model();
    ModelType powered = bare;
    powered.propulsion = reference_chain();

    // The same model with and without its chain, spawned at the same start and
    // commanded alike, steps to the same snapshot, byte for byte.
    const auto run = [](const ModelType& model) -> std::vector<std::byte> {
        spade::Simulation sim = one_world();
        const auto id = sim.register_model(model);
        EXPECT_TRUE(id.has_value()) << why(id);
        if (!id) return {};
        spade::VehicleSpawn start;
        start.pos = glm::vec3(0.0f, 2.0f, 0.0f);
        start.rotor_omega = 900.0f;
        const auto vehicle = sim.spawn(0, *id, start);
        EXPECT_TRUE(vehicle.has_value()) << why(vehicle);
        if (!vehicle) return {};
        EXPECT_TRUE(sim.flush_structural().has_value());
        const std::vector<float> commands(model.rotors.size(), 1100.0f);
        EXPECT_TRUE(sim.set_rotor_commands(*vehicle, commands).has_value());
        EXPECT_TRUE(sim.step(40).has_value());
        const auto blob = sim.snapshot();
        EXPECT_TRUE(blob.has_value()) << why(blob);
        if (!blob) return {};
        return std::vector<std::byte>(blob->bytes().begin(), blob->bytes().end());
    };
    const std::vector<std::byte> without = run(bare);
    const std::vector<std::byte> with = run(powered);
    ASSERT_FALSE(without.empty());
    EXPECT_TRUE(with == without) << "no step reads the chain, and no identity folds it";

    // A blob taken with the chain restores where the model has none, and back.
    for (const auto& [from, to] : {std::pair<const ModelType*, const ModelType*>{&powered, &bare}, {&bare, &powered}}) {
        spade::Simulation source = one_world();
        ASSERT_TRUE(source.register_model(*from).has_value());
        ASSERT_TRUE(source.step(3).has_value());
        const auto blob = source.snapshot();
        ASSERT_TRUE(blob.has_value()) << why(blob);
        spade::Simulation target = one_world();
        ASSERT_TRUE(target.register_model(*to).has_value());
        const auto restored = target.restore(*blob);
        EXPECT_TRUE(restored.has_value()) << why(restored);
    }
}
