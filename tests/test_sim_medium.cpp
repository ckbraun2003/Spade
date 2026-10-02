// tests/test_sim_medium.cpp -- Simulation::sample_medium, the host-side read
// of a world's air: density from its WorldParams row, wind = the row's mean
// wind plus the world's current Dryden gust. That is the same sample the
// ForceElements pass takes, so a caller drawing or controlling against the air
// sees what the physics saw on the last substep.

#include <gtest/gtest.h>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

TEST(SimMedium, SampleMediumReturnsDensityAndWindWithNoTurbulence) {
    spade::Environment env;
    env.wind = glm::vec3(3.0f, 0.0f, 1.0f);
    env.air_density = 1.1f;
    auto world = spade::WorldBuilder().name("m").environment(env)
                     .capacities(spade::Capacities{1, 1, 1, 1}).build();
    ASSERT_TRUE(world.has_value());
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    auto sim = spade::Simulation::create(spade::WorldSetDesc{{inst}}, 2'000'000, 2);
    ASSERT_TRUE(sim.has_value());
    ASSERT_TRUE(sim->step(10).has_value());
    auto s = sim->sample_medium(0, glm::vec3(0.0f));
    ASSERT_TRUE(s.has_value());
    EXPECT_EQ(s->density, 1.1f);
    EXPECT_EQ(s->wind, glm::vec3(3.0f, 0.0f, 1.0f));
}

TEST(SimMedium, SampleMediumIncludesTheGustAndIsPositionIndependent) {
    spade::Environment env;
    auto world = spade::WorldBuilder().name("m").environment(env)
                     .capacities(spade::Capacities{1, 1, 1, 1}).build();
    ASSERT_TRUE(world.has_value());
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.seed = 42u;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::severe);
    auto sim = spade::Simulation::create(spade::WorldSetDesc{{inst}}, 2'000'000, 2);
    ASSERT_TRUE(sim.has_value());
    ASSERT_TRUE(sim->step(500).has_value());
    auto a = sim->sample_medium(0, glm::vec3(0.0f));
    auto b = sim->sample_medium(0, glm::vec3(10.0f, -3.0f, 7.0f));
    ASSERT_TRUE(a.has_value() && b.has_value());
    EXPECT_EQ(a->wind, b->wind);                       // Dryden is position-independent
    EXPECT_GT(glm::length(a->wind), 0.0f);             // a severe gust after 1 s is not zero

    auto missing = sim->sample_medium(1, glm::vec3(0.0f));  // there is no world 1
    ASSERT_FALSE(missing.has_value());
    EXPECT_EQ(missing.error().code, spade::Code::invalid_argument);
}
