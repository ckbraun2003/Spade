// test_contact_sets.cpp -- L10, the contact sets' plumbing (Core; the lead's
// rulings L10 and L11): BodyState::contact_set, spawn's write, the
// configuration table's capacity and shape checks in register_model(), and
// the registry identity that folds every non-empty table, with old blobs
// restoring to the sphere. Physics' derivation and pass are not here: these
// tests swap the table's build for contact_set_fakes.hpp's.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

#include "contact_set_fakes.hpp"
#include "core/error.hpp"
#include "physics/contact_sets.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "state/layout.hpp"
#include "state/snapshot.hpp"
#include "testing/replay.hpp"
#include "vehicles/model_identity.hpp"

namespace {

using namespace contact_set_fakes;
using spade::BodyState;
using spade::ModelTypeId;
using spade::Simulation;
using spade::SnapshotBlob;

constexpr uint64_t kDtNs = 2'000'000;
constexpr uint32_t kSubsteps = 2;

[[nodiscard]] Simulation make(const spade::modules::ModuleSet& set, bool ground = false) {
    auto sim = Simulation::create(world(ground), kDtNs, kSubsteps, {}, set);
    EXPECT_TRUE(sim.has_value()) << sim.error().context;
    return std::move(*sim);
}

[[nodiscard]] ModelTypeId add(Simulation& sim, float mass) {
    const auto id = sim.register_model(quad_of_mass(mass));
    EXPECT_TRUE(id.has_value()) << id.error().context;
    return id.value_or(ModelTypeId{});
}

[[nodiscard]] spade::VehicleRef put(Simulation& sim, ModelTypeId model, float x, float y = 10.0f) {
    const auto v = sim.spawn(0, model, at(x, y));
    EXPECT_TRUE(v.has_value()) << v.error().context;
    return v.value_or(spade::VehicleRef{});
}

[[nodiscard]] uint32_t lane(const Simulation& sim, const spade::VehicleRef& v) {
    const auto bodies = sim.world_bodies(0);
    EXPECT_TRUE(bodies.has_value());
    return (*bodies)[v.body.slot].contact_set;
}

[[nodiscard]] std::vector<float> table_of(const Simulation& sim) {
    const auto t = sim.config_table(kContactSetsTable);
    EXPECT_TRUE(t.has_value()) << t.error().context;
    if (!t) return {};
    return {t->begin(), t->end()};
}

// What the models alone fold to: master's registry identity.
[[nodiscard]] uint64_t models_identity(const Simulation& sim) {
    std::vector<spade::vehicles::ModelType> models;
    for (uint32_t i = 1; i <= sim.model_count(); ++i) models.push_back(**sim.model(ModelTypeId{i}));
    return spade::vehicles::model_registry_identity(models);
}

[[nodiscard]] SnapshotBlob blob_of(const Simulation& sim) {
    auto blob = sim.snapshot();
    EXPECT_TRUE(blob.has_value()) << blob.error().context;
    return std::move(*blob);
}

// `blob` with its header's registry identity replaced and, when `zero_lanes`,
// every body's contact_set lane cleared: a build from before L10 wrote
// exactly that (the pad was 0, and the identity folded the models alone).
[[nodiscard]] SnapshotBlob rewritten(const SnapshotBlob& blob, uint64_t registry_identity, bool zero_lanes) {
    std::vector<std::byte> bytes(blob.bytes().begin(), blob.bytes().end());
    std::memcpy(bytes.data() + offsetof(spade::SnapshotHeader, model_registry_identity), &registry_identity,
                sizeof registry_identity);
    if (zero_lanes) {
        const auto bodies = spade::find_section(blob, "bodies");
        EXPECT_TRUE(bodies.has_value());
        if (bodies) {
            const auto at = static_cast<std::size_t>(bodies->payload.data() - blob.bytes().data());
            for (std::size_t row = 0; row < bodies->payload.size() / sizeof(BodyState); ++row) {
                std::memset(bytes.data() + at + row * sizeof(BodyState) + offsetof(BodyState, contact_set), 0,
                            sizeof(uint32_t));
            }
        }
    }
    auto out = SnapshotBlob::from_bytes(std::move(bytes));
    EXPECT_TRUE(out.has_value());
    return std::move(*out);
}

// A light quad (no set) and a heavy one (a set), spawned and flushed: the
// shape most of these tests restore and compare.
struct Pair {
    spade::VehicleRef light;
    spade::VehicleRef heavy;
};
[[nodiscard]] Pair light_and_heavy(Simulation& sim, float y = 10.0f) {
    const ModelTypeId light = add(sim, 1.0f);
    const ModelTypeId heavy = add(sim, 2.0f);
    Pair p{put(sim, light, 0.0f, y), put(sim, heavy, 1.0f, y)};
    EXPECT_TRUE(sim.flush_structural().has_value());
    return p;
}

}  // namespace

// ===========================================================================
// The lane: BodyState's former pad, and who writes it.
// ===========================================================================

TEST(ContactSets, TheLaneIsBodyStatesFormerPadAndNothingElseMoves) {
    static_assert(std::is_same_v<decltype(BodyState::contact_set), uint32_t>);
    EXPECT_EQ(offsetof(BodyState, contact_set), 60u);
    EXPECT_EQ(sizeof(BodyState), 128u);
    EXPECT_EQ(offsetof(BodyState, inv_inertia_diag), 64u);
}

TEST(ContactSets, TheStandardSetDeclaresTheTableWithItsCapacity) {
    const ConfigTableDecl t = standard_contact_table();
    EXPECT_EQ(t.name, kContactSetsTable);
    EXPECT_EQ(t.build, &spade::physics::build_contact_sets);
    EXPECT_EQ(t.capacity, kContactSetsCapacityRows * kContactSetRowFloats);
    EXPECT_EQ(kContactSetsCapacityRows, 16384u) << "L11's capacity";
}

TEST(ContactSets, APlainBodyAndAModelWithoutASetKeepTheSphere) {
    Simulation sim = make(standard_set_with<&build_a>());
    const ModelTypeId light = add(sim, 1.0f);
    const spade::VehicleRef quad = put(sim, light, 0.0f);
    const auto plain = sim.spawn(0, spade::BodySpawn{.pos = glm::vec3(2.0f, 10.0f, 0.0f)});
    ASSERT_TRUE(plain.has_value()) << plain.error().context;
    ASSERT_TRUE(sim.flush_structural().has_value());
    EXPECT_EQ(lane(sim, quad), 0u);
    EXPECT_EQ((*sim.world_bodies(0))[plain->slot].contact_set, 0u);
    EXPECT_TRUE(table_of(sim).empty()) << "no model has a set, so the table is empty";
}

TEST(ContactSets, AModelWithASetSpawnsItsIdIntoTheLane) {
    Simulation sim = make(standard_set_with<&build_a>());
    const Pair p = light_and_heavy(sim);
    EXPECT_EQ(lane(sim, p.light), 0u) << "model 1 has no set: the sphere";
    EXPECT_EQ(lane(sim, p.heavy), 2u) << "model 2 has a set: its ModelTypeId";
}

TEST(ContactSets, ALaterRegistrationLeavesEveryLaneAndTheSetItNames) {
    Simulation sim = make(standard_set_with<&build_a>());
    const Pair p = light_and_heavy(sim);
    const ModelTypeId third = add(sim, 4.0f);
    EXPECT_EQ(third.value, 3u);
    ASSERT_EQ(lane(sim, p.heavy), 2u);

    // Model 2's range row, read the way a pass reads it, still names its two
    // points, wherever the rows moved to.
    const std::vector<float> t = table_of(sim);
    ASSERT_EQ(t.size(), (3u + 4u) * kContactSetRowFloats) << "three range rows, then 2 + 2 points";
    const std::size_t range = (lane(sim, p.heavy) - 1u) * kContactSetRowFloats;
    const auto first = static_cast<std::size_t>(t[range]);
    ASSERT_EQ(t[range + 1], 2.0f);
    EXPECT_EQ(t[first * kContactSetRowFloats], 2.0f);
    EXPECT_EQ(t[(first + 1) * kContactSetRowFloats + 1], 2.0f);
}

// ===========================================================================
// register_model(): the capacity and the layout (L11).
// ===========================================================================

TEST(ContactSets, ARegistrationPastTheCapacityIsRefusedByNameAndChangesNothing) {
    Simulation sim = make(standard_set_with<&build_huge>());
    (void)add(sim, 2.0f);
    const std::vector<float> before = table_of(sim);
    ASSERT_FALSE(before.empty());

    const auto huge = sim.register_model(quad_of_mass(3.0f));
    ASSERT_FALSE(huge.has_value()) << "the table would pass its capacity";
    EXPECT_EQ(huge.error().code, spade::Code::capacity_exceeded);
    EXPECT_NE(huge.error().context.find("contact_sets"), std::string::npos) << huge.error().context;
    EXPECT_EQ(sim.model_count(), 1u) << "the registry is as it was";
    EXPECT_EQ(table_of(sim), before) << "and so is the table";
    EXPECT_EQ(add(sim, 1.0f).value, 2u) << "the next model takes the id the refused one did not";
}

TEST(ContactSets, ATableThatBreaksTheLayoutIsRefusedByNameAndChangesNothing) {
    Simulation sim = make(standard_set_with<&build_malformed>());
    const auto bad = sim.register_model(quad_of_mass(2.0f));
    ASSERT_FALSE(bad.has_value()) << "model 1's range runs past the table's last row";
    EXPECT_NE(bad.error().context.find("contact_sets"), std::string::npos) << bad.error().context;
    EXPECT_EQ(sim.model_count(), 0u);
    EXPECT_TRUE(table_of(sim).empty());
}

TEST(ContactSets, ABoundTableWithNoCapacityIsRefusedAtCompile) {
    static constexpr ConfigTableDecl kBound[] = {{.name = "bound_points", .build = &build_none, .binding = 40}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "bounder", .tables = kBound});
    const auto compiled = spade::modules::compile_schedule(set);
    ASSERT_FALSE(compiled.has_value()) << "its device buffer could not be sized";
    EXPECT_NE(compiled.error().context.find("bound_points"), std::string::npos) << compiled.error().context;
}

// ===========================================================================
// The registry identity (L10/L11): every non-empty table, folded.
// ===========================================================================

TEST(ContactSets, AnAllEmptyRegistryKeepsMastersIdentity) {
    for (const spade::modules::ModuleSet& set : {spade::modules::standard_modules(), standard_set_with<&build_a>()}) {
        Simulation sim = make(set);
        (void)add(sim, 1.0f);
        (void)add(sim, 1.5f);
        ASSERT_TRUE(table_of(sim).empty());
        EXPECT_EQ(blob_of(sim).model_registry_identity(), models_identity(sim));
    }
}

TEST(ContactSets, ANonEmptyTableMovesTheIdentityAndOneMovedPointMovesItAgain) {
    Simulation a = make(standard_set_with<&build_a>());
    Simulation b = make(standard_set_with<&build_b>());
    (void)light_and_heavy(a);
    (void)light_and_heavy(b);
    ASSERT_EQ(models_identity(a), models_identity(b)) << "the same models";
    const uint64_t ia = blob_of(a).model_registry_identity();
    const uint64_t ib = blob_of(b).model_registry_identity();
    EXPECT_NE(ia, models_identity(a));
    EXPECT_NE(ib, models_identity(b));
    EXPECT_NE(ia, ib);
}

// ===========================================================================
// Restore: a new blob beside the same sets only; an old blob to the sphere.
// ===========================================================================

TEST(ContactSets, ABlobRestoresBesideTheSameSetsAndIsRefusedBesideOthers) {
    Simulation source = make(standard_set_with<&build_a>());
    const Pair p = light_and_heavy(source);
    const SnapshotBlob blob = blob_of(source);

    Simulation same = make(standard_set_with<&build_a>());
    (void)light_and_heavy(same);
    ASSERT_TRUE(same.restore(blob).has_value());
    EXPECT_EQ(lane(same, p.heavy), 2u);
    EXPECT_EQ(spade::testing::state_digest(same), spade::testing::state_digest(source));

    Simulation other = make(standard_set_with<&build_b>());
    (void)light_and_heavy(other);
    const auto refused = other.restore(blob);
    ASSERT_FALSE(refused.has_value()) << "the same models, a different table";
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_NE(refused.error().context.find("table"), std::string::npos) << refused.error().context;
}

TEST(ContactSets, AnOldBlobRestoresToTheSphere) {
    Simulation source = make(standard_set_with<&build_a>());
    const Pair p = light_and_heavy(source);
    const SnapshotBlob old = rewritten(blob_of(source), models_identity(source), true);

    Simulation target = make(standard_set_with<&build_a>());
    (void)light_and_heavy(target);
    ASSERT_TRUE(target.restore(old).has_value());
    EXPECT_EQ(lane(target, p.light), 0u);
    EXPECT_EQ(lane(target, p.heavy), 0u) << "an old blob's body keeps the sphere it had";
}

TEST(ContactSets, AnOldIdentityBesideANonZeroLaneIsRefused) {
    Simulation source = make(standard_set_with<&build_a>());
    (void)light_and_heavy(source);
    const SnapshotBlob forged = rewritten(blob_of(source), models_identity(source), false);

    Simulation target = make(standard_set_with<&build_a>());
    (void)light_and_heavy(target);
    const auto refused = target.restore(forged);
    ASSERT_FALSE(refused.has_value()) << "no build before L10 wrote a non-zero lane";
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
}

// THE LEAD'S TEST (L10): the crafted old blob, restored and stepped, is
// byte-identical to master's continuation of the same blob. "Master" is the
// standard set with a table that builds nothing -- what static_contact held
// before L10 -- and its blob is shown equal to the crafted one byte for byte
// before both are stepped. With Physics' pass in, the heavy quad's lane-0 body
// must still take the sphere, or the digests part; L11's no-version-bump
// exception rests on this test.
TEST(ContactSets, AnOldBlobReplaysAsMastersContinuationOfIt) {
    Simulation l10 = make(standard_set_with<&build_a>(), true);
    Simulation master = make(standard_set_with<&build_none>(), true);
    for (Simulation* sim : {&l10, &master}) {
        (void)light_and_heavy(*sim, 0.25f);  // 5 cm above contact: they land within the run
        ASSERT_TRUE(sim->step(20).has_value());
    }
    const SnapshotBlob old = rewritten(blob_of(l10), models_identity(l10), true);
    const SnapshotBlob masters = blob_of(master);
    ASSERT_EQ(old.bytes().size(), masters.bytes().size());
    EXPECT_EQ(std::memcmp(old.bytes().data(), masters.bytes().data(), old.bytes().size()), 0)
        << "the crafted blob is the one master writes";

    ASSERT_TRUE(l10.restore(old).has_value());
    ASSERT_TRUE(master.restore(masters).has_value());
    for (int i = 0; i < 4; ++i) {
        ASSERT_TRUE(l10.step(50).has_value());
        ASSERT_TRUE(master.step(50).has_value());
        EXPECT_EQ(spade::testing::state_digest(l10), spade::testing::state_digest(master)) << "after " << (i + 1) * 50;
    }
    const auto bodies = l10.world_bodies(0);
    ASSERT_TRUE(bodies.has_value());
    for (const BodyState& b : *bodies) EXPECT_EQ(b.contact_set, 0u);
}
