// test_design_frame.cpp -- a vehicle's design frame (the drone-builder physics
// requirements DBP-44..47; Core's drone-builder plan, Task B). A model type
// carries the rotation from its design frame to its principal body frame and
// its centre of mass measured from the design origin; spawn() takes a start in
// the design frame and vehicle_state() returns one, and nothing else sees
// either quantity (DBP-45).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "sim/design_frame.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace {

// bench_sim.cpp's 1 kg airframe: valid, with rotors, drag and an IMU.
[[nodiscard]] spade::vehicles::QuadrotorParams quad_params() {
    spade::vehicles::QuadrotorParams p;
    p.name = "design_frame_quad";
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

[[nodiscard]] spade::vehicles::ModelType quad_model(glm::quat q_bd, glm::vec3 c) {
    spade::vehicles::ModelType m = spade::vehicles::make_quadrotor(quad_params()).value();
    m.design_to_principal = q_bd;
    m.com_offset = c;
    return m;
}

[[nodiscard]] spade::Simulation one_world() {
    const spade::WorldDesc world = spade::WorldBuilder()
                                       .name("frame")
                                       .environment(spade::Environment{})
                                       .capacities(spade::Capacities{2, 16, 4, 1})
                                       .build()
                                       .value();
    spade::WorldInstanceDesc inst;
    inst.world = world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::Simulation::create(spade::WorldSetDesc{{inst}}, 2'000'000, 2).value();
}

template <typename T>
[[nodiscard]] bool same_bytes(const T& a, const T& b) {
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

void expect_near(glm::vec3 got, glm::vec3 want, const char* what) {
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(got[i], want[i], 1e-6f * std::max(1.0f, std::fabs(want[i]))) << what << "[" << i << "]";
    }
}

const glm::quat kTilt = glm::normalize(glm::quat(0.96f, 0.10f, 0.20f, -0.15f));
const glm::vec3 kOffset(0.01f, -0.02f, 0.005f);

[[nodiscard]] spade::VehicleSpawn design_start() {
    spade::VehicleSpawn start;
    start.pos = glm::vec3(1.0f, 10.0f, -2.0f);
    start.orient = glm::normalize(glm::quat(0.9f, 0.05f, -0.3f, 0.2f));
    start.vel = glm::vec3(0.5f, -0.25f, 1.0f);
    start.omega_body = glm::vec3(0.3f, -0.2f, 0.1f);
    return start;
}

}  // namespace

TEST(ModelTypeValidation, TheDesignFrameMustBeFiniteAndOrientable) {
    EXPECT_TRUE(quad_model(kTilt, kOffset).validate().has_value());
    EXPECT_FALSE(quad_model(glm::quat(0.0f, 0.0f, 0.0f, 0.0f), kOffset).validate().has_value())
        << "a zero-length rotation";
    EXPECT_FALSE(quad_model(kTilt, glm::vec3(std::numeric_limits<float>::infinity(), 0.0f, 0.0f)).validate().has_value())
        << "a non-finite offset";
    spade::vehicles::ModelType unversioned = quad_model(kTilt, kOffset);
    unversioned.version = 0;
    EXPECT_FALSE(unversioned.validate().has_value()) << "version 0";
}

TEST(DesignFrame, TheIdentityFrameLeavesTheStateBitwise) {
    spade::FrameState s;
    s.pos = glm::vec3(1.5f, -2.0f, 3.25f);
    s.orient = glm::quat(0.7f, 0.1f, 0.2f, 0.3f);  // deliberately not unit: untouched means untouched
    s.vel = glm::vec3(-0.5f, 0.125f, 2.0f);
    s.omega = glm::vec3(0.25f, -1.0f, 0.5f);
    const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(same_bytes(spade::to_body_state(s, identity, glm::vec3(0.0f)), s));
    EXPECT_TRUE(same_bytes(spade::to_design_state(s, identity, glm::vec3(0.0f)), s));
}

TEST(DesignFrame, SpawnThenReadReturnsTheDesignStart) {
    auto sim = one_world();
    const auto id = sim.register_model(quad_model(kTilt, kOffset));
    ASSERT_TRUE(id.has_value()) << id.error().context;
    const spade::VehicleSpawn start = design_start();
    const auto vehicle = sim.spawn(0, *id, start);
    ASSERT_TRUE(vehicle.has_value()) << vehicle.error().context;
    ASSERT_TRUE(sim.flush_structural().has_value());

    const auto design = sim.vehicle_state(*vehicle);
    ASSERT_TRUE(design.has_value()) << design.error().context;
    expect_near(design->pos, start.pos, "pos");
    expect_near(design->vel, start.vel, "vel");
    expect_near(design->omega, start.omega_body, "omega");
    EXPECT_NEAR(std::fabs(glm::dot(design->orient, start.orient)), 1.0f, 1e-6f) << "orientation";

    // And the body is really elsewhere: its centre of mass is off the design origin.
    const auto body = sim.body(vehicle->body);
    ASSERT_TRUE(body.has_value());
    EXPECT_GT(glm::length((*body)->pos - start.pos), 0.01f);
}

TEST(DesignFrame, QAndMinusQRegisterTheSameModel) {
    auto a = one_world();
    auto b = one_world();
    const auto ia = a.register_model(quad_model(kTilt, kOffset));
    const auto ib = b.register_model(quad_model(-kTilt, kOffset));
    ASSERT_TRUE(ia.has_value() && ib.has_value());
    const auto va = a.spawn(0, *ia, design_start());
    const auto vb = b.spawn(0, *ib, design_start());
    ASSERT_TRUE(va.has_value() && vb.has_value());
    ASSERT_TRUE(a.flush_structural().has_value() && b.flush_structural().has_value());
    const auto ba = a.body(va->body);
    const auto bb = b.body(vb->body);
    ASSERT_TRUE(ba.has_value() && bb.has_value());
    EXPECT_TRUE(same_bytes(**ba, **bb)) << "q and -q are one rotation; register_model canonicalizes";
}

TEST(DesignFrame, TheCanonicalDesignRotationIsOneValuePerRotation) {
    using spade::vehicles::canonical_design_rotation;
    const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::quat same = canonical_design_rotation(identity);
    EXPECT_EQ(std::memcmp(&same, &identity, sizeof same), 0) << "the identity comes back bit for bit";

    // q and -q are one rotation: one stored value, with w > 0.
    const glm::quat plus = canonical_design_rotation(kTilt);
    const glm::quat minus = canonical_design_rotation(-kTilt);
    EXPECT_EQ(std::memcmp(&plus, &minus, sizeof plus), 0);
    EXPECT_GT(plus.w, 0.0f);

    // A non-unit input is normalized.
    EXPECT_NEAR(glm::length(canonical_design_rotation(kTilt * 2.0f)), 1.0f, 1e-6f);

    // With w == 0 the next non-zero component sets the sign.
    const glm::quat half_turn = canonical_design_rotation(glm::quat(0.0f, 0.0f, -1.0f, 0.0f));
    EXPECT_EQ(half_turn.y, 1.0f);
    EXPECT_EQ(half_turn.w, 0.0f);
    EXPECT_EQ(half_turn.x, 0.0f);
}

// Every model built today has the identity frame and a zero offset; spawn must
// leave its start untouched, bit for bit, or every vehicle golden moves.
TEST(DesignFrame, ADefaultFrameSpawnsTheStartBitwise) {
    auto sim = one_world();
    const auto id = sim.register_model(quad_model(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(0.0f)));
    ASSERT_TRUE(id.has_value()) << id.error().context;
    spade::VehicleSpawn start = design_start();
    start.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // unit exactly, so spawn's normalization is a no-op
    const auto vehicle = sim.spawn(0, *id, start);
    ASSERT_TRUE(vehicle.has_value()) << vehicle.error().context;
    ASSERT_TRUE(sim.flush_structural().has_value());
    const auto body = sim.body(vehicle->body);
    ASSERT_TRUE(body.has_value());
    EXPECT_TRUE(same_bytes((*body)->pos, start.pos));
    EXPECT_TRUE(same_bytes((*body)->orient, start.orient));
    EXPECT_TRUE(same_bytes((*body)->vel, start.vel));
    EXPECT_TRUE(same_bytes((*body)->omega_body, start.omega_body));
}
