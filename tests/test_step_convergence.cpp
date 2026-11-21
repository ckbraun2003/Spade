// ---------------------------------------------------------------------------
// test_step_convergence.cpp -- O140 phase 1, item 1 (docs/design/physics/plans/2026-11-21-fc-physics.md section 5.1): the rigid-body step the autopilot
// link will run at. The proposal is dt = 500 us with one substep, because 500 us divides PX4's 4 ms, ArduPilot's 2.5 ms and 2 ms exchanges (1 ms does not
// divide 2.5 ms). This test is the evidence the tick needs: the library's X500 kit model (tools/flow_movie/data/x500/x500.model.yaml, the rotors' lag, the
// inflow factor, the drag and the gyroscopic coupling all in the loop) is flown the same open-loop manoeuvre at dt = 1 ms, 500 us, 250 us and 125 us, and
// the state at the end is compared with the 125 us run. The manoeuvre is a hover, a +2% / -2% roll doublet of the rotors' commands (50 ms each), and a
// hover again, 0.5 s in all, in a void world (no ground, no wind).
//
//   StepConvergence.TheFiveHundredMicrosecondStepIsWithinTheBoundOfTheFinerRuns   attitude, position and body-rate error against the 125 us run; the
//                                                                                 error falls when the step is halved (a step that did not
//                                                                                 converge would not)
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "scene/model_file.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_type.hpp"
#include "world/builder.hpp"

namespace {

using spade::Simulation;

constexpr float kG = 9.80665f;
constexpr float kAirDensity = 1.225f;

struct EndState {
    glm::vec3 pos{0.0f};
    glm::vec3 omega_body{0.0f};
    glm::quat orient{1.0f, 0.0f, 0.0f, 0.0f};
};

[[nodiscard]] spade::Result<Simulation> void_sim(uint64_t dt_ns) {
    spade::Environment env;
    env.gravity = glm::vec3(0.0f, -kG, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = kAirDensity;
    spade::Capacities caps;
    caps.bodies = 4;
    caps.force_elements = 16;
    caps.sensors = 4;
    caps.contacts = 1;
    const spade::Result<spade::WorldDesc> world = spade::WorldBuilder().name("void").environment(env).capacities(caps).build();
    if (!world) return std::unexpected(world.error());
    spade::WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0x5EED0500ULL;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.1f;
    instance.grid.cell_size = 1.0f;
    return Simulation::create(spade::WorldSetDesc{{instance}}, dt_ns, 1);
}

// Flies the manoeuvre at `dt_ns` (a divisor of 50 ms) and returns the state at 0.5 s.
[[nodiscard]] spade::Result<EndState> fly(const spade::vehicles::ModelType& model, uint64_t dt_ns) {
    spade::Result<Simulation> sim = void_sim(dt_ns);
    if (!sim) return std::unexpected(sim.error());
    const auto id = sim->register_model(model);
    if (!id) return std::unexpected(id.error());

    // The hover speed from the rotors' static thrust constants: a quarter of the weight each. (The rotors' lag and inflow factor mean it is not an exact
    // trim, so the drone sinks a little; the comparison is between steps, not against a hover.)
    float k_t = 0.0f;
    for (const auto& r : model.rotors) k_t += r.thrust_coeff / static_cast<float>(model.rotors.size());
    const float w_hover = std::sqrt(model.body.mass * kG / (static_cast<float>(model.rotors.size()) * k_t));

    spade::VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 50.0f, 0.0f);
    where.rotor_omega = w_hover;
    const auto vehicle = sim->spawn(0, *id, where);
    if (!vehicle) return std::unexpected(vehicle.error());
    if (auto r = sim->flush_structural(); !r) return std::unexpected(r.error());

    const auto steps_for = [&](double seconds) { return static_cast<uint32_t>(std::llround(seconds * 1.0e9 / static_cast<double>(dt_ns))); };

    // Rotors 0 and 3 are on the +z side of the X500 model file, 1 and 2 on the -z side (rotor 0 front-right, 1 front-left, 2 rear-left, 3 rear-right).
    const auto command = [&](float side) -> spade::Result<void> {
        float c[4] = {w_hover * (1.0f + side), w_hover * (1.0f - side), w_hover * (1.0f - side), w_hover * (1.0f + side)};
        return sim->set_rotor_commands(*vehicle, std::span<const float>(c, 4));
    };
    struct Leg { float side; double seconds; };
    const Leg legs[] = {{0.0f, 0.10}, {+0.02f, 0.05}, {-0.02f, 0.05}, {0.0f, 0.30}};
    for (const Leg& leg : legs) {
        if (auto r = command(leg.side); !r) return std::unexpected(r.error());
        if (auto r = sim->step(steps_for(leg.seconds)); !r) return std::unexpected(r.error());
    }
    const auto body = sim->body(vehicle->body);
    if (!body) return std::unexpected(body.error());
    return EndState{(*body)->pos, (*body)->omega_body, (*body)->orient};
}

[[nodiscard]] float angle_between(const glm::quat& a, const glm::quat& b) {
    const glm::quat d = glm::conjugate(a) * b;
    return 2.0f * std::acos(std::min(1.0f, std::fabs(d.w)));
}

TEST(StepConvergence, TheFiveHundredMicrosecondStepIsWithinTheBoundOfTheFinerRuns) {
    const auto loaded = spade::scene::load_model_file(std::filesystem::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500/x500.model.yaml");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    ASSERT_EQ(loaded->model.rotors.size(), 4u);

    const uint64_t steps_ns[] = {1'000'000, 500'000, 250'000, 125'000};
    EndState end[4];
    for (int i = 0; i < 4; ++i) {
        const auto s = fly(loaded->model, steps_ns[i]);
        ASSERT_TRUE(s.has_value()) << "dt " << steps_ns[i] << " ns: " << s.error().context;
        end[i] = *s;
    }
    const EndState& ref = end[3];
    float att[3];
    float pos[3];
    float rate[3];
    for (int i = 0; i < 3; ++i) {
        att[i] = angle_between(end[i].orient, ref.orient);
        pos[i] = glm::length(end[i].pos - ref.pos);
        rate[i] = glm::length(end[i].omega_body - ref.omega_body);
        std::printf("[ step    ] dt %4llu us against 125 us: attitude %.3e rad (%.4f deg), position %.3e m, body rate %.3e rad/s\n",
                    static_cast<unsigned long long>(steps_ns[i] / 1000), static_cast<double>(att[i]), static_cast<double>(att[i]) * 57.29577951308232,
                    static_cast<double>(pos[i]), static_cast<double>(rate[i]));
    }
    std::printf("[ step    ] the 125 us run ends at pos (%.4f, %.4f, %.4f) m, rate (%.4f, %.4f, %.4f) rad/s, attitude %.4f rad from level\n",
                static_cast<double>(ref.pos.x), static_cast<double>(ref.pos.y), static_cast<double>(ref.pos.z), static_cast<double>(ref.omega_body.x),
                static_cast<double>(ref.omega_body.y), static_cast<double>(ref.omega_body.z),
                static_cast<double>(angle_between(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), ref.orient)));
    // The bound at 500 us, and the trend: halving the step must not make the error larger (a step that did not converge would not fall).
    EXPECT_LT(att[1], 1.0e-3f) << "attitude error at 500 us";
    EXPECT_LT(pos[1], 5.0e-3f) << "position error at 500 us";
    EXPECT_LT(rate[1], 1.0e-2f) << "body-rate error at 500 us";
    EXPECT_LE(att[2], att[1]);
    EXPECT_LE(att[1], att[0]);
    EXPECT_LE(pos[2], pos[1]);
    EXPECT_LE(pos[1], pos[0]);
}

}  // namespace
