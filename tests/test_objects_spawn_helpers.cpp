// test_objects_spawn_helpers.cpp -- Plan A Task 11 (24th spec SL9e): bulk
// body placement.
//
// The two properties worth pinning are the ones v1 did not have: placement is
// SEEDED rather than ambient (v1 seeded mt19937 from std::random_device on
// every call, which is why its captures can never be a numerical reference),
// and a spawn is ALL OR NOTHING (a partial spawn looks like it worked).

#include <gtest/gtest.h>

#include <vector>

#include "objects/spawn_helpers.hpp"
#include "world/medium.hpp"

using spade::BodyRef;
using spade::Simulation;
using spade::objects::CubeSpawn;
using spade::objects::SphereSpawn;
using spade::objects::spawn_in_cube;
using spade::objects::spawn_in_sphere;

namespace {

[[nodiscard]] spade::Result<Simulation> empty_world(uint32_t capacity) {
    spade::Environment env;
    env.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;

    spade::Capacities caps;
    caps.bodies = capacity;
    caps.force_elements = 1;
    caps.sensors = 1;
    caps.contacts = 1;

    spade::Result<spade::WorldDesc> world =
        spade::WorldBuilder().name("spawn").environment(env).capacities(caps).build();
    if (!world) return std::unexpected(world.error());

    spade::WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 5;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.0f;
    instance.grid.cell_size = 1.0f;

    return Simulation::create(spade::WorldSetDesc{{instance}}, 4'000'000, 4);
}

// Positions only become readable after the structural queue is flushed --
// spawn() queues, it does not place.
[[nodiscard]] std::vector<glm::vec3> positions_of(Simulation& sim,
                                                  const std::vector<BodyRef>& refs) {
    std::vector<glm::vec3> out;
    if (!sim.flush_structural()) {
        ADD_FAILURE() << "flush_structural failed";
        return out;
    }
    for (const BodyRef& ref : refs) {
        const spade::Result<const spade::BodyState*> state = sim.body(ref);
        if (!state) {
            ADD_FAILURE() << "body ref did not resolve";
            return out;
        }
        out.push_back((*state)->pos);
    }
    return out;
}

}  // namespace

TEST(SpawnHelpers, SphereSpawnPlacesEveryBodyInsideTheRadius) {
    spade::Result<Simulation> sim = empty_world(128);
    ASSERT_TRUE(sim.has_value());
    const glm::vec3 center(1.0f, 2.0f, 3.0f);
    const auto refs =
        spawn_in_sphere(*sim, 0, SphereSpawn{.center = center, .radius = 4.0f, .count = 100, .seed = 12345});
    ASSERT_TRUE(refs.has_value()) << (refs ? "" : refs.error().context);
    EXPECT_EQ(refs->size(), 100u);

    const std::vector<glm::vec3> placed = positions_of(*sim, *refs);
    ASSERT_EQ(placed.size(), 100u);
    for (const glm::vec3& p : placed) {
        EXPECT_LE(glm::length(p - center), 4.0f + 1e-5f);
    }
}

// Rejection sampling must fill the volume, not hug the centre or a corner --
// a bug that placed everything at the centre would satisfy the radius test
// above completely.
TEST(SpawnHelpers, SphereSpawnActuallySpreadsBodiesOut) {
    spade::Result<Simulation> sim = empty_world(256);
    ASSERT_TRUE(sim.has_value());
    const auto refs = spawn_in_sphere(
        *sim, 0, SphereSpawn{.center = glm::vec3(0.0f), .radius = 4.0f, .count = 200, .seed = 7});
    ASSERT_TRUE(refs.has_value());

    const std::vector<glm::vec3> placed = positions_of(*sim, *refs);
    ASSERT_EQ(placed.size(), 200u);
    float nearest = 1e9f;
    float farthest = 0.0f;
    for (const glm::vec3& p : placed) {
        const float r = glm::length(p);
        nearest = std::min(nearest, r);
        farthest = std::max(farthest, r);
    }
    EXPECT_LT(nearest, 2.0f) << "nothing landed in the inner half";
    EXPECT_GT(farthest, 3.0f) << "nothing landed in the outer quarter";
}

TEST(SpawnHelpers, CubeSpawnPlacesEveryBodyInsideTheExtent) {
    spade::Result<Simulation> sim = empty_world(128);
    ASSERT_TRUE(sim.has_value());
    const glm::vec3 center(0.0f, 5.0f, 0.0f);
    const auto refs = spawn_in_cube(
        *sim, 0, CubeSpawn{.center = center, .half_extent = 2.0f, .count = 64, .seed = 4242});
    ASSERT_TRUE(refs.has_value());

    for (const glm::vec3& p : positions_of(*sim, *refs)) {
        EXPECT_LE(std::abs(p.x - center.x), 2.0f + 1e-5f);
        EXPECT_LE(std::abs(p.y - center.y), 2.0f + 1e-5f);
        EXPECT_LE(std::abs(p.z - center.z), 2.0f + 1e-5f);
    }
}

// The engine has no wall clock and no ambient RNG: the same seed must give
// identical placement, or a sandbox scene would not reopen the same way twice.
TEST(SpawnHelpers, IsSeedDeterministicAcrossRuns) {
    const SphereSpawn s{.center = glm::vec3(0.0f), .radius = 2.0f, .count = 64, .seed = 999};
    std::vector<glm::vec3> runs[2];
    for (int run = 0; run < 2; ++run) {
        spade::Result<Simulation> sim = empty_world(128);
        ASSERT_TRUE(sim.has_value());
        const auto refs = spawn_in_sphere(*sim, 0, s);
        ASSERT_TRUE(refs.has_value());
        runs[run] = positions_of(*sim, *refs);
        ASSERT_EQ(runs[run].size(), 64u);
    }
    EXPECT_EQ(runs[0], runs[1]);
}

TEST(SpawnHelpers, DifferentSeedsGiveDifferentPlacement) {
    std::vector<glm::vec3> runs[2];
    for (int run = 0; run < 2; ++run) {
        spade::Result<Simulation> sim = empty_world(128);
        ASSERT_TRUE(sim.has_value());
        const auto refs = spawn_in_sphere(
            *sim, 0,
            SphereSpawn{.center = glm::vec3(0.0f), .radius = 2.0f, .count = 64,
                        .seed = run == 0 ? 1u : 2u});
        ASSERT_TRUE(refs.has_value());
        runs[run] = positions_of(*sim, *refs);
    }
    EXPECT_NE(runs[0], runs[1]);
}

// A partial spawn would be worse than none: it looks like it worked.
TEST(SpawnHelpers, RefusesToOverrunCapacityWithoutSpawningAnything) {
    spade::Result<Simulation> sim = empty_world(16);
    ASSERT_TRUE(sim.has_value());
    const auto refs = spawn_in_sphere(
        *sim, 0, SphereSpawn{.center = glm::vec3(0.0f), .radius = 1.0f, .count = 100, .seed = 1});
    ASSERT_FALSE(refs.has_value());
    EXPECT_EQ(refs.error().code, spade::Code::capacity_exceeded);

    // And nothing was placed -- the refusal is total, not a stop-when-full.
    ASSERT_TRUE(sim->flush_structural().has_value());
    const spade::Result<uint32_t> live = sim->live_body_count(0);
    ASSERT_TRUE(live.has_value());
    EXPECT_EQ(*live, 0u);
}

// v1's SetVelocity/RandomizeVelocity half of the SL9e row. Zero must mean AT
// REST and must draw nothing, so requesting velocities cannot shift the
// position sequence -- a scene's placement has to be the same either way.
TEST(SpawnHelpers, VelocityIsOptionalAndDoesNotDisturbPlacement) {
    const glm::vec3 center(0.0f);
    std::vector<glm::vec3> at_rest;
    std::vector<glm::vec3> moving;
    for (int run = 0; run < 2; ++run) {
        spade::Result<Simulation> sim = empty_world(64);
        ASSERT_TRUE(sim.has_value());
        SphereSpawn s{.center = center, .radius = 3.0f, .count = 32, .seed = 77};
        if (run == 1) s.velocity_radius_mps = 5.0f;
        const auto refs = spawn_in_sphere(*sim, 0, s);
        ASSERT_TRUE(refs.has_value());
        (run == 0 ? at_rest : moving) = positions_of(*sim, *refs);

        // Velocities: zero on the first run, inside the ball on the second.
        bool any_moving = false;
        for (const BodyRef& ref : *refs) {
            const spade::Result<const spade::BodyState*> state = sim->body(ref);
            ASSERT_TRUE(state.has_value());
            const float speed = glm::length((*state)->vel);
            EXPECT_LE(speed, 5.0f + 1e-5f);
            // BRACED, and not stylistically: EXPECT_FLOAT_EQ expands to an
            // if/else, so a bare `if` makes that else ambiguous and gcc-13
            // rejects it under -Werror=dangling-else. MSVC does not warn.
            if (run == 0) {
                EXPECT_FLOAT_EQ(speed, 0.0f);
            }
            if (speed > 0.0f) any_moving = true;
        }
        EXPECT_EQ(any_moving, run == 1);
    }
    EXPECT_EQ(at_rest, moving) << "asking for velocities moved the bodies";
}

TEST(SpawnHelpers, RejectsADegenerateShape) {
    spade::Result<Simulation> sim = empty_world(16);
    ASSERT_TRUE(sim.has_value());
    EXPECT_FALSE(
        spawn_in_sphere(*sim, 0, SphereSpawn{.radius = 0.0f, .count = 1, .seed = 1}).has_value());
    EXPECT_FALSE(
        spawn_in_cube(*sim, 0, CubeSpawn{.half_extent = -1.0f, .count = 1, .seed = 1}).has_value());
}
