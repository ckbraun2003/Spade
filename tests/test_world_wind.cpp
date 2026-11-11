// test_world_wind.cpp -- Simulation::set_world_wind (L46): a running world's mean wind, set from the host.
//
// The wind is the world's WorldParams.wind row, world frame, m/s, signed. A set writes the registered row, takes effect at the next
// step(), and every substep of that step reads it (Dryden still adds its gust). The creation identity (config_hash folds env.wind
// once, into the replay_config row) is NOT changed by a set: the live wind is state, the creation wind is identity.
// The device half is test_gpu_world_wind.cpp.
#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>

#include "physics/field_row.hpp"
#include "physics/schedule.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "testing/replay.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace {

using spade::modules::Access;
using spade::modules::ModuleDesc;
using spade::modules::PassDecl;
using spade::modules::Phase;
using spade::modules::QuantityAccess;

constexpr glm::vec3 kCreated{3.0f, 0.0f, 1.0f};
constexpr glm::vec3 kSet{-4.0f, 5.5f, 0.25f};

// Two worlds with no turbulence, both created with the wind `created`.
[[nodiscard]] spade::WorldSetDesc two_calm_worlds(glm::vec3 created = kCreated) {
    spade::Environment env;
    env.wind = created;
    const auto world = spade::WorldBuilder().name("w").environment(env).capacities(spade::Capacities{1, 1, 1, 1}).build();
    EXPECT_TRUE(world.has_value());
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    spade::WorldSetDesc set{{inst, inst}};
    set.worlds[1].seed += 1;
    return set;
}

[[nodiscard]] spade::Simulation make(const spade::modules::ModuleSet& modules, uint32_t substeps, glm::vec3 created = kCreated) {
    spade::Result<spade::Simulation> sim =
        spade::Simulation::create(two_calm_worlds(created), 1'000'000, substeps, spade::compute::BackendDesc{}, modules);
    EXPECT_TRUE(sim.has_value()) << (sim ? "" : sim.error().context);
    return std::move(*sim);
}

[[nodiscard]] spade::Simulation make(uint32_t substeps = 2, glm::vec3 created = kCreated) {
    return make(spade::modules::standard_modules(), substeps, created);
}

[[nodiscard]] glm::vec3 row_wind(const spade::Simulation& sim, uint32_t world) {
    const auto row = sim.world_params(world);
    EXPECT_TRUE(row.has_value());
    return row ? glm::vec3((*row)->wind) : glm::vec3(0.0f);
}

[[nodiscard]] glm::vec3 field_wind(const spade::Simulation& sim, uint32_t world) {
    const auto f = sim.field_samples(world);
    EXPECT_TRUE(f.has_value());
    if (!f) return glm::vec3(0.0f);
    const uint32_t o = spade::modules::kFieldWindOffset;
    return glm::vec3((*f)[o], (*f)[o + 1], (*f)[o + 2]);
}

[[nodiscard]] bool bits_equal(glm::vec3 a, glm::vec3 b) {
    return std::bit_cast<uint32_t>(a.x) == std::bit_cast<uint32_t>(b.x) && std::bit_cast<uint32_t>(a.y) == std::bit_cast<uint32_t>(b.y) &&
           std::bit_cast<uint32_t>(a.z) == std::bit_cast<uint32_t>(b.z);
}

// A reader in the forces phase: records every world's field wind, once per substep.
std::array<std::vector<glm::vec3>, 2> g_seen;
void record(const spade::physics::SubstepContext& ctx) noexcept {
    for (std::size_t w = 0; w < ctx.worlds.size() && w < g_seen.size(); ++w) {
        const std::span<float> f = ctx.worlds[w].fields;
        g_seen[w].emplace_back(f[spade::modules::kFieldWindOffset], f[spade::modules::kFieldWindOffset + 1],
                               f[spade::modules::kFieldWindOffset + 2]);
    }
}
constexpr QuantityAccess kRecordAccess[] = {{"field.wind", Access::read}};
constexpr PassDecl kRecordPasses[] = {{.name = "record", .phase = Phase::forces, .access = kRecordAccess, .cpu = &record}};

[[nodiscard]] spade::modules::ModuleSet with_recorder() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(ModuleDesc{.name = "windwatch", .passes = kRecordPasses});
    return set;
}

}  // namespace

// (a) A set, then a step: the next sample of that world's air is the set value, bit for bit, and the other world is untouched.
TEST(WorldWind, ASetWindIsTheNextStepsWindOfThatWorldOnly) {
    spade::Simulation sim = make();
    ASSERT_TRUE(sim.step(1).has_value());
    ASSERT_TRUE(bits_equal(sim.sample_medium(1, glm::vec3(0.0f))->wind, kCreated));
    const spade::Result<void> set = sim.set_world_wind(1, kSet);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    EXPECT_TRUE(bits_equal(row_wind(sim, 1), kSet)) << "the registered row holds the set value at once";
    ASSERT_TRUE(sim.step(1).has_value());
    const auto medium = sim.sample_medium(1, glm::vec3(0.0f));
    ASSERT_TRUE(medium.has_value());
    EXPECT_TRUE(bits_equal(medium->wind, kSet));
    EXPECT_TRUE(bits_equal(field_wind(sim, 1), kSet));
    EXPECT_TRUE(bits_equal(sim.sample_medium(0, glm::vec3(0.0f))->wind, kCreated)) << "world 0 was not set";
    EXPECT_TRUE(bits_equal(field_wind(sim, 0), kCreated));
}

// (b) Every substep of the next step reads the set value, at one substep and at four; the step before it read the old wind.
class WorldWindSubsteps : public ::testing::TestWithParam<uint32_t> {};

TEST_P(WorldWindSubsteps, EverySubstepOfTheNextStepReadsTheSetWind) {
    const uint32_t substeps = GetParam();
    for (auto& v : g_seen) v.clear();
    spade::Simulation sim = make(with_recorder(), substeps);
    ASSERT_TRUE(sim.step(1).has_value());
    ASSERT_EQ(g_seen[0].size(), std::size_t{substeps});
    for (const glm::vec3 w : g_seen[0]) EXPECT_TRUE(bits_equal(w, kCreated)) << "before the set";
    ASSERT_TRUE(sim.set_world_wind(0, kSet).has_value());
    for (auto& v : g_seen) v.clear();
    ASSERT_TRUE(sim.step(2).has_value());
    ASSERT_EQ(g_seen[0].size(), std::size_t{2} * substeps);
    for (const glm::vec3 w : g_seen[0]) EXPECT_TRUE(bits_equal(w, kSet)) << "every substep of both later steps";
    for (const glm::vec3 w : g_seen[1]) EXPECT_TRUE(bits_equal(w, kCreated)) << "world 1 was not set";
}

INSTANTIATE_TEST_SUITE_P(Substeps, WorldWindSubsteps, ::testing::Values(1u, 4u));

// (c) The set wind is state: a snapshot taken after a set restores into a set created with the ORIGINAL wind and runs as the
// unrestored one does.
TEST(WorldWind, ASnapshotAfterASetRestoresAndStepsLikeTheUnrestoredRun) {
    spade::Simulation a = make();
    ASSERT_TRUE(a.step(3).has_value());
    ASSERT_TRUE(a.set_world_wind(0, kSet).has_value());
    const spade::Result<spade::SnapshotBlob> blob = a.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    spade::Simulation b = make();
    const spade::Result<void> restored = b.restore(*blob);
    ASSERT_TRUE(restored.has_value()) << "a set does not change the creation identity: " << restored.error().context;
    EXPECT_TRUE(bits_equal(row_wind(b, 0), kSet));
    EXPECT_EQ(spade::testing::state_digest(a), spade::testing::state_digest(b));
    ASSERT_TRUE(a.step(5).has_value());
    ASSERT_TRUE(b.step(5).has_value());
    EXPECT_EQ(spade::testing::state_digest(a), spade::testing::state_digest(b));
    EXPECT_TRUE(bits_equal(field_wind(b, 0), kSet));
}

// The creation identity is the wind AT CREATION, once, in the replay_config row; a set leaves it alone.
TEST(WorldWind, ASetLeavesTheCreationIdentityAlone) {
    spade::Simulation sim = make();
    const auto before = sim.arenas().array(sim.replay_config_array());
    ASSERT_TRUE(before.has_value());
    const std::vector<spade::ReplayConfig> rows(before->begin(), before->end());
    ASSERT_TRUE(sim.set_world_wind(0, kSet).has_value());
    ASSERT_TRUE(sim.step(1).has_value());
    const auto after = sim.arenas().array(sim.replay_config_array());
    ASSERT_TRUE(after.has_value());
    ASSERT_EQ(after->size(), rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i) EXPECT_EQ((*after)[i].config_hash, rows[i].config_hash) << "row " << i;
}

// (d) Setting the wind a world already has changes nothing: no digest moves, with or without a step in between.
TEST(WorldWind, SettingTheCurrentWindChangesNoDigest) {
    spade::Simulation sim = make();
    ASSERT_TRUE(sim.step(2).has_value());
    const uint64_t before = spade::testing::state_digest(sim);
    ASSERT_TRUE(sim.set_world_wind(0, kCreated).has_value());
    ASSERT_TRUE(sim.set_world_wind(1, row_wind(sim, 1)).has_value());
    EXPECT_EQ(spade::testing::state_digest(sim), before);
    spade::Simulation same = make();
    ASSERT_TRUE(same.step(2).has_value());
    ASSERT_TRUE(same.step(1).has_value());
    ASSERT_TRUE(sim.step(1).has_value());
    EXPECT_EQ(spade::testing::state_digest(sim), spade::testing::state_digest(same)) << "a no-op set leaves the run as it was";
}

// A real change does move the digest (the set is state), and setting it back restores the digest.
TEST(WorldWind, ASetToADifferentWindMovesTheDigestAndSettingItBackRestoresIt) {
    spade::Simulation sim = make();
    ASSERT_TRUE(sim.step(1).has_value());
    const uint64_t before = spade::testing::state_digest(sim);
    ASSERT_TRUE(sim.set_world_wind(0, kSet).has_value());
    EXPECT_NE(spade::testing::state_digest(sim), before);
    ASSERT_TRUE(sim.set_world_wind(0, kCreated).has_value());
    EXPECT_EQ(spade::testing::state_digest(sim), before);
}

// (e) Each refusal names its cause and leaves every row and every digest as it was.
TEST(WorldWind, ARefusalNamesItsCauseAndChangesNothing) {
    spade::Simulation sim = make();
    ASSERT_TRUE(sim.step(1).has_value());
    const uint64_t before = spade::testing::state_digest(sim);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    struct Case {
        uint32_t world;
        glm::vec3 wind;
        const char* names;
    };
    const Case cases[] = {
        {2, kSet, "world_index"},
        {0xFFFFFFFFu, kSet, "world_index"},
        {0, glm::vec3(nan, 0.0f, 0.0f), "wind.x"},
        {1, glm::vec3(0.0f, inf, 0.0f), "wind.y"},
        {1, glm::vec3(0.0f, 0.0f, -inf), "wind.z"},
    };
    for (const Case& c : cases) {
        const spade::Result<void> r = sim.set_world_wind(c.world, c.wind);
        ASSERT_FALSE(r.has_value()) << c.names;
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << c.names;
        EXPECT_NE(r.error().context.find("set_world_wind"), std::string::npos) << r.error().context;
        EXPECT_NE(r.error().context.find(c.names), std::string::npos) << r.error().context;
        EXPECT_TRUE(bits_equal(row_wind(sim, 0), kCreated));
        EXPECT_TRUE(bits_equal(row_wind(sim, 1), kCreated));
        EXPECT_EQ(spade::testing::state_digest(sim), before) << c.names;
    }
}

// A refused component leaves the world's OTHER components as they were: the write is whole or not at all.
TEST(WorldWind, ARefusalWritesNoComponent) {
    spade::Simulation sim = make();
    const glm::vec3 bad(1.0f, std::numeric_limits<float>::quiet_NaN(), 2.0f);
    ASSERT_FALSE(sim.set_world_wind(0, bad).has_value());
    EXPECT_TRUE(bits_equal(row_wind(sim, 0), kCreated));
}
