// tests/test_sim_medium.cpp -- Simulation::sample_medium, the host-side read
// of a world's air: density from its WorldParams row, wind = the row's mean
// wind plus the world's current Dryden gust. That is the same sample the
// ForceElements pass takes, so a caller drawing or controlling against the air
// sees what the physics saw on the last substep.

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstdint>
#include <utility>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "testing/replay.hpp"
#include "vehicles/quadrotor.hpp"
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

// ---------------------------------------------------------------------------
// Module-API stage 3: the medium as fields. Providers write gravity, density
// and wind once per world per substep into a scratch sample row, and rotors,
// drag and Integrate read the row. The stored values are the same operations
// on the same inputs as the inline values they replace.
// ---------------------------------------------------------------------------
namespace {

[[nodiscard]] glm::vec3 sim_mean_wind() { return glm::vec3(3.0f, 0.0f, 1.0f); }

// A vec3's bits. An alias, so EXPECT_EQ's arguments carry no bare comma.
using Bits3 = std::array<uint32_t, 3>;

// One world, a mean wind, the given Dryden level, seed 0x5EED, 2 ms in 2
// substeps; room for one quadrotor. value() throws on failure, which gtest
// reports as a failure of the calling test.
[[nodiscard]] spade::Simulation make_turbulent_one_world(spade::TurbulenceLevel level) {
    spade::Environment env;
    env.wind = sim_mean_wind();
    const spade::WorldDesc world = spade::WorldBuilder()
                                       .name("m")
                                       .environment(env)
                                       .capacities(spade::Capacities{1, 8, 2, 1})
                                       .build()
                                       .value();
    spade::WorldInstanceDesc inst;
    inst.world = world;
    inst.seed = 0x5EEDu;
    inst.turbulence = spade::dryden_params(level);
    return spade::Simulation::create(spade::WorldSetDesc{{inst}}, 2'000'000, 2).value();
}

// A 1 kg quadrotor (bench_sim.cpp's airframe) at 10 m, spun up to hover, so
// its rotors and its quadratic drag read the sampled medium -- density and
// wind -- every substep.
[[nodiscard]] spade::vehicles::QuadrotorParams test_quadrotor_params() {
    spade::vehicles::QuadrotorParams p;
    p.name = "fields_quad";
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
    return p;
}

void spawn_one_quad(spade::Simulation& sim) {
    const spade::vehicles::QuadrotorParams params = test_quadrotor_params();
    const auto model = spade::vehicles::make_quadrotor(params);
    ASSERT_TRUE(model.has_value()) << model.error().context;
    const auto id = sim.register_model(*model);
    ASSERT_TRUE(id.has_value()) << id.error().context;
    spade::VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    where.rotor_omega = spade::vehicles::hover_command(params);
    const auto quad = sim.spawn(0, *id, where);
    ASSERT_TRUE(quad.has_value()) << quad.error().context;
}

}  // namespace

TEST(SimFields, StoredWindIsTheInlineDrydenSampleBitwise) {
    auto sim = make_turbulent_one_world(spade::TurbulenceLevel::moderate);
    ASSERT_TRUE(sim.step(3).has_value());
    const auto row = sim.field_samples(0);
    ASSERT_TRUE(row.has_value()) << row.error().context;
    const auto inline_sample = sim.sample_medium(0, glm::vec3(0.0f));  // the provider's CPU function, current state
    ASSERT_TRUE(inline_sample.has_value());
    const glm::vec3 stored_wind{(*row)[spade::modules::kFieldWindOffset + 0], (*row)[spade::modules::kFieldWindOffset + 1],
                                (*row)[spade::modules::kFieldWindOffset + 2]};
    EXPECT_EQ(std::bit_cast<Bits3>(stored_wind), std::bit_cast<Bits3>(inline_sample->wind));
    EXPECT_EQ(std::bit_cast<uint32_t>((*row)[spade::modules::kFieldDensityOffset]),
              std::bit_cast<uint32_t>(inline_sample->density));
}

TEST(SimFields, SampleMediumBeforeTheFirstStepIsMeanWindPlusTheInitialGust) {
    auto sim = make_turbulent_one_world(spade::TurbulenceLevel::moderate);
    const auto s = sim.sample_medium(0, glm::vec3(0.0f));
    ASSERT_TRUE(s.has_value());
    EXPECT_NE(s->wind, sim_mean_wind()) << "dryden_init places the filter on its stationary distribution at create()";
}

// The sample row is scratch, not state: the first substep after a restore
// must recompute it before any reader, or the restored run would read the
// target's stale row.
TEST(SimFields, RestoreThenStepMatchesAnUninterruptedRun) {
    auto a = make_turbulent_one_world(spade::TurbulenceLevel::severe);
    auto b = make_turbulent_one_world(spade::TurbulenceLevel::severe);
    spawn_one_quad(a);
    spawn_one_quad(b);
    ASSERT_TRUE(a.step(5).has_value());
    const auto blob = a.snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    ASSERT_TRUE(b.restore(*blob).has_value());
    ASSERT_TRUE(a.step(5).has_value() && b.step(5).has_value());
    EXPECT_EQ(spade::testing::state_digest(a), spade::testing::state_digest(b));
}
