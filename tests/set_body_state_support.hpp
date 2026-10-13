// set_body_state_support.hpp -- the Markov replay Simulation::set_body_state
// owes on both backends (docs/design/core/2026-10-13-set-body-state-design.md,
// section 6, tests 2 and 3: Kat's 2a and 2b). test_set_body_state.cpp runs it on
// the CPU, test_gpu_parity.cpp on Vulkan; one helper, so the two legs run one
// experiment. Test support, header-only.
#pragma once

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/layout.hpp"
#include "testing/replay.hpp"
#include "world/builder.hpp"

namespace spade::test {

// One world: a solid floor (the half-space y <= 0), contacts on, calm air,
// room for a few bodies, elements and sensors.
[[nodiscard]] inline Result<WorldSetDesc> floor_world_set(uint32_t bodies = 4) {
    Environment env;
    env.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    Capacities caps;
    caps.bodies = bodies;
    caps.force_elements = 4;
    caps.sensors = 2;
    caps.contacts = 1;
    const Result<WorldDesc> world = WorldBuilder()
                                        .name("floor")
                                        .environment(env)
                                        .capacities(caps)
                                        .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                        .build();
    if (!world) return std::unexpected(world.error());
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0x5E7B0D1E5ULL;
    instance.contacts.restitution_e = 0.2f;
    instance.contacts.friction_mu = 0.4f;
    instance.contacts.proxy_radius = 0.1f;
    instance.grid.cell_size = 0.5f;
    return WorldSetDesc{{instance}};
}

// A bare body of the replay's mass and (anisotropic) inertia at `pos`.
[[nodiscard]] inline BodySpawn bare_body(glm::vec3 pos) {
    BodySpawn body;
    body.pos = pos;
    body.mass = 0.8f;
    body.inv_inertia_diag = glm::vec3(40.0f, 55.0f, 30.0f);
    return body;
}

// The body row behind `ref`, by value; a failure (and a zero row) if body()
// refuses the ref.
[[nodiscard]] inline BodyState row_of(const Simulation& sim, BodyRef ref) {
    const Result<const BodyState*> row = sim.body(ref);
    if (!row.has_value()) {
        ADD_FAILURE() << "body(): " << row.error().context;
        return BodyState{};
    }
    return **row;
}

// The row's kinematics in hex floats, for a failure message.
[[nodiscard]] inline std::string row_text(const BodyState& b) {
    char text[512];
    std::snprintf(text, sizeof(text),
                  "pos (%a %a %a) orient (%a %a %a %a) vel (%a %a %a) omega (%a %a %a) sf (%a %a %a)",
                  b.pos.x, b.pos.y, b.pos.z, b.orient.w, b.orient.x, b.orient.y, b.orient.z, b.vel.x, b.vel.y,
                  b.vel.z, b.omega_body.x, b.omega_body.y, b.omega_body.z, b.specific_force.x, b.specific_force.y,
                  b.specific_force.z);
    return text;
}

struct MarkovCase {
    glm::vec3 omega0{0.0f};  // the original's spawn spin, body frame
    glm::vec3 torque{0.0f};  // the body-frame torque of the wrench applied every step
    uint64_t k = 30;         // the tick S is recorded at
    uint64_t n = 25;         // ticks compared after it
};

// The world force applied before step t, in both runs: a pure function of t.
[[nodiscard]] inline glm::vec3 markov_force(uint64_t t) {
    const float a = static_cast<float>(t % 5u) - 2.0f;
    return glm::vec3(0.2f + 0.05f * a, 6.0f, -0.1f);
}

// THE KEY TEST (Kat 2a; 2b with omega0 != 0). The ORIGINAL run is a bare body
// under a wrench applied every step; S = body() at tick k, and the body row is
// recorded at ticks k+1..k+n. The REPLAY is the same desc with the body spawned
// elsewhere, resting on the floor (so it has contact history), stepped to tick k
// with no wrench; then S is set and the same wrench applied each step. The row
// must equal the original's BYTE FOR BYTE at every tick after the set, and the
// whole state's digest must equal at the end. On Vulkan both runs are on the
// device (same-backend determinism) and the set forces one upload.
inline void expect_markov_replay_exact(const compute::BackendDesc& backend, const MarkovCase& c, const char* what) {
    const Result<WorldSetDesc> set = floor_world_set();
    ASSERT_TRUE(set.has_value()) << set.error().context;

    // --- the original ---------------------------------------------------------
    Result<Simulation> original = Simulation::create(*set, 1'000'000, 2, backend);
    ASSERT_TRUE(original.has_value()) << original.error().context;
    BodySpawn flying = bare_body(glm::vec3(0.0f, 3.0f, 0.0f));
    flying.orient = glm::quat(0.95f, 0.1f, -0.2f, 0.2f);
    flying.vel = glm::vec3(0.4f, 0.0f, -0.2f);
    flying.omega_body = c.omega0;
    const Result<BodyRef> body = original->spawn(0, flying);
    ASSERT_TRUE(body.has_value()) << body.error().context;
    ASSERT_TRUE(original->flush_structural().has_value());
    for (uint64_t t = 0; t < c.k; ++t) {
        ASSERT_TRUE(original->apply_wrench(*body, markov_force(t), c.torque).has_value());
        ASSERT_TRUE(original->step(1).has_value());
    }
    const BodyState s = row_of(*original, *body);
    std::vector<BodyState> rows;
    for (uint64_t t = c.k; t < c.k + c.n; ++t) {
        ASSERT_TRUE(original->apply_wrench(*body, markov_force(t), c.torque).has_value());
        ASSERT_TRUE(original->step(1).has_value());
        rows.push_back(row_of(*original, *body));
    }
    const uint64_t original_digest = spade::testing::state_digest(*original);

    // --- the replay -----------------------------------------------------------
    Result<Simulation> replay = Simulation::create(*set, 1'000'000, 2, backend);
    ASSERT_TRUE(replay.has_value()) << replay.error().context;
    const Result<BodyRef> resting = replay->spawn(0, bare_body(glm::vec3(1.5f, 0.1f, 1.0f)));
    ASSERT_TRUE(resting.has_value()) << resting.error().context;
    ASSERT_EQ(resting->slot, body->slot);
    ASSERT_EQ(resting->generation, body->generation);
    ASSERT_TRUE(replay->step(c.k).has_value());
    ASSERT_EQ(replay->tick().value, c.k);
    const BodyState before = row_of(*replay, *resting);
    EXPECT_GT(before.specific_force.y, 1.0f)
        << what << ": the replay body should rest on the floor at tick k, so it has contact history: "
        << row_text(before);

    const uint64_t uploads = replay->vulkan_upload_count();
    const Result<void> placed = replay->set_body_state(*resting, s.pos, s.orient, s.vel, s.omega_body);
    ASSERT_TRUE(placed.has_value()) << what << ": " << placed.error().context;
    for (uint64_t i = 0; i < c.n; ++i) {
        ASSERT_TRUE(replay->apply_wrench(*resting, markov_force(c.k + i), c.torque).has_value());
        ASSERT_TRUE(replay->step(1).has_value());
        const BodyState row = row_of(*replay, *resting);
        ASSERT_EQ(std::memcmp(&row, &rows[i], sizeof(BodyState)), 0)
            << what << ": the replay's row differs from the original's at tick " << (c.k + i + 1)
            << "\n  original " << row_text(rows[i]) << "\n  replay   " << row_text(row);
        if (i == 0 && backend.kind == compute::BackendKind::vulkan) {
            EXPECT_EQ(replay->vulkan_upload_count(), uploads + 1)
                << what << ": the set and the wrench before the step force exactly one re-upload";
        }
    }
    EXPECT_EQ(spade::testing::state_digest(*replay), original_digest)
        << what << ": the whole state, not only the row, must equal the original's";
}

}  // namespace spade::test
