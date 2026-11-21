// ---------------------------------------------------------------------------
// test_x500_ground_contact.cpp -- O140 phase 1, item 4 (docs/design/physics/plans/2026-11-21-ground-contact.md): the X500 kit model standing on its own feet, on
// the engine's existing multi-point contact (L68's rigid_contact module, a hull shape of body-frame points and a skin radius; L82's `collision:` block).
//
// THE FEET. Four rubber pads (the build table's JIAO-EVA objects, measured in the design frame: centres (+-0.0675, -0.2058, +-0.12) m, 93 x 19 x 19 mm), three
// points a pad (both ends and the middle, so a pad cannot rest on a point pair), a skin radius of 9.5 mm: twelve points, the lowest of the drone at y = -0.2153 m.
// The centre of mass is at y = -0.02543, so it should stand 0.18987 m above the ground. Friction 0.6 and restitution 0.1 are ASSUMED (EVA foam on a hard floor).
//
// THE MODEL IS BUILT IN MEMORY from tools/flow_movie/data/x500/x500.model.yaml (the file the posted film pins by its hash is not changed): load, set `shape`.
//
//   X500GroundContact.RestsOnItsFeetAtTheRightHeightAndTheImuReadsGravity    rest height 0.18987 m +-1 mm; the IMU reads +g up with little noise; the rest velocity
//   X500GroundContact.ADropSettlesWithoutBouncingMoreThanTheRestitutionAllows from 0.5 m: the rebound is about e^2 of the drop at most, then it rests
//   X500GroundContact.ThePitchedDroneRocksBackBelowTheTipOverAndTipsAbove    25 degrees of pitch rocks back, 40 degrees tips (the footprint's derived 31 degrees)
//   X500GroundContact.ARolledDroneRocksBackBelowTheTipOverAndTipsAbove       the same in roll (derived 34 degrees: 25 holds, 45 tips)
//   X500GroundContact.TheSlopeHoldsAboveTheFrictionAngleAndSlidesBelowIt     a tilted gravity is a slope: 20 degrees holds at mu 0.6, slides at mu 0.3
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "physics/rigid_contact.hpp"
#include "scene/model_file.hpp"
#include "sensors/imu.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "rigid_contact_support.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_type.hpp"
#include "world/builder.hpp"

namespace {

namespace ph = spade::physics;

constexpr float kG = 9.80665f;
constexpr float kSkin = 0.0095f;
constexpr double kComY = -0.0254301;  // x500.model.yaml com_offset.y: the design frame's centre of mass height
constexpr float kLowest = -0.2153f;   // the pads' bottom faces in the design frame
constexpr float kRestHeight = static_cast<float>(kComY - kLowest);  // 0.18987 m: the centre of mass over the plane at rest
constexpr float kDesignOriginAtRest = -kLowest;                      // 0.2153 m: the design origin over the plane at rest. A VEHICLE's spawn pose is the DESIGN
                                                                     // frame's (VehicleSpawn: the design origin and design axes, converted by spawn()), not the body's

// The twelve points in the DESIGN frame: three along each pad's length (both ends of the 19 mm-section bar, and the middle).
[[nodiscard]] std::vector<glm::vec3> pad_points_design() {
    std::vector<glm::vec3> out;
    for (const float sx : {1.0f, -1.0f}) {
        for (const float sz : {1.0f, -1.0f}) {
            for (const float dx : {0.0305f, 0.0675f, 0.1045f}) out.emplace_back(sx * dx, -0.2058f, sz * 0.12f);
        }
    }
    return out;
}

[[nodiscard]] spade::vehicles::ModelType load_model(float friction, float restitution) {
    const auto loaded = spade::scene::load_model_file(std::filesystem::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500/x500.model.yaml");
    EXPECT_TRUE(loaded.has_value()) << (loaded ? "" : loaded.error().context);
    spade::vehicles::ModelType model = loaded->model;
    // design frame -> body frame: p_body = q_d2p * (p_design - com_offset)
    std::vector<glm::vec3> body_points;
    for (const glm::vec3& p : pad_points_design()) body_points.push_back(model.design_to_principal * (p - model.com_offset));
    ph::ShapeDesc shape = ph::hull_shape(body_points, kSkin);
    shape.friction = friction;
    shape.restitution = restitution;
    // 16 sweeps (kMaxSweeps): the default 4 leaves a level 3.1 m/s drop spinning at 0.7 to 0.8 rad/s from an unconverged sequential-impulse solve over twelve points
    // (DiagnosticSpinAfterALevelDrop below: 0.017 rad/s at 16 sweeps), and a rest creep; the converged solution zeroes every point's normal speed, so no spin.
    shape.iterations = 16;
    model.shape = shape;
    return model;
}

[[nodiscard]] spade::WorldSetDesc make_world(glm::vec3 gravity) {
    spade::Environment env;
    env.gravity = gravity;
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    spade::WorldBuilder b;
    b.name("ground").environment(env).capacities(spade::Capacities{4, 16, 4, 1});
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    const auto built = b.build();
    EXPECT_TRUE(built.has_value());
    spade::WorldInstanceDesc inst;
    inst.world = *built;
    inst.seed = 11;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    inst.contacts.restitution_e = 0.0f;
    inst.contacts.friction_mu = 0.0f;
    inst.contacts.proxy_radius = 0.0f;
    return spade::WorldSetDesc{{inst}};
}

struct Rig {
    spade::Simulation sim;
    spade::VehicleRef vehicle{};
};

// A rig with the vehicle spawned at `pos` (the centre of mass) in `orient` (body to world; the design frame's own rotation is accounted for by the caller).
[[nodiscard]] Rig make_rig(const spade::vehicles::ModelType& model, glm::vec3 gravity, glm::vec3 pos, glm::quat orient) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::rigid_contact_module());
    auto sim = spade::Simulation::create(make_world(gravity), 1'000'000, 1, spade::compute::BackendDesc{.kind = spade::compute::BackendKind::cpu}, set);
    EXPECT_TRUE(sim.has_value()) << (sim ? "" : sim.error().context);
    Rig rig{std::move(*sim), {}};
    const auto id = rig.sim.register_model(model);
    EXPECT_TRUE(id.has_value()) << (id ? "" : id.error().context);
    spade::VehicleSpawn where;
    where.pos = pos;
    where.orient = orient;
    where.rotor_omega = 0.0f;
    const auto v = rig.sim.spawn(0, *id, where);
    EXPECT_TRUE(v.has_value()) << (v ? "" : v.error().context);
    rig.vehicle = *v;
    EXPECT_TRUE(rig.sim.flush_structural().has_value());
    return rig;
}

[[nodiscard]] spade::BodyState state(const Rig& rig) {
    const auto b = rig.sim.body(rig.vehicle.body);
    EXPECT_TRUE(b.has_value());
    return b ? **b : spade::BodyState{};
}

void steps(Rig& rig, uint32_t n) {
    const auto r = rig.sim.step(n);
    ASSERT_TRUE(r.has_value()) << r.error().context;
}

// Level: the design frame's axes along the world's. (The first runs of these tests spawned with the conjugate of design_to_principal, taking the vehicle's spawn
// pose for the body's: the drone landed 5 degrees yawed and tilted, which is what spun it. VehicleSpawn is the design frame's pose.)
[[nodiscard]] glm::quat level(const spade::vehicles::ModelType&) { return glm::quat(1.0f, 0.0f, 0.0f, 0.0f); }

// The IMU's last `count` samples' accelerometer, in the mount frame.
struct ImuStats {
    glm::vec3 mean{0.0f};
    glm::vec3 spread{0.0f};  // max - min per axis
    uint32_t n = 0;
};
[[nodiscard]] ImuStats imu_stats(const Rig& rig, uint32_t count) {
    std::vector<spade::sensors::ImuSample> buf(spade::sensors::kRingDepth);
    const auto poll = rig.sim.poll_imu(rig.vehicle.imu_sensors[0], 0, buf);
    EXPECT_TRUE(poll.has_value());
    ImuStats s;
    if (!poll) return s;
    const std::size_t have = poll->samples.size();
    const std::size_t from = have > count ? have - count : 0;
    glm::vec3 lo(1e9f), hi(-1e9f);
    for (std::size_t i = from; i < have; ++i) {
        s.mean += poll->samples[i].accel;
        lo = glm::min(lo, poll->samples[i].accel);
        hi = glm::max(hi, poll->samples[i].accel);
        ++s.n;
    }
    if (s.n > 0) s.mean /= static_cast<float>(s.n);
    s.spread = hi - lo;
    return s;
}

TEST(X500GroundContact, RestsOnItsFeetAtTheRightHeightAndTheImuReadsGravity) {
    const spade::vehicles::ModelType model = load_model(0.6f, 0.1f);
    Rig rig = make_rig(model, glm::vec3(0.0f, -kG, 0.0f), glm::vec3(0.0f, kDesignOriginAtRest + 0.01f, 0.0f), level(model));
    for (int k = 0; k < 8; ++k) {
        steps(rig, 250);
        const spade::BodyState t = state(rig);
        std::printf("[ ground  ]   rest t %.2f s: pos (%+.5f, %+.5f, %+.5f) vel (%+.2e, %+.2e, %+.2e)\n", 0.25 * (k + 1), static_cast<double>(t.pos.x), static_cast<double>(t.pos.y),
                    static_cast<double>(t.pos.z), static_cast<double>(t.vel.x), static_cast<double>(t.vel.y), static_cast<double>(t.vel.z));
    }
    const spade::BodyState b = state(rig);
    const ImuStats imu = imu_stats(rig, spade::sensors::kRingDepth);
    std::printf("[ ground  ] at rest after 2 s: centre of mass y %.5f m (expected %.5f), velocity (%+.2e, %+.2e, %+.2e) m/s; IMU (last %u samples) mean (%+.4f, %+.4f, %+.4f) m/s^2, "
                "spread (%.2e, %.2e, %.2e)\n",
                static_cast<double>(b.pos.y), static_cast<double>(kRestHeight), static_cast<double>(b.vel.x), static_cast<double>(b.vel.y), static_cast<double>(b.vel.z),
                imu.n, static_cast<double>(imu.mean.x), static_cast<double>(imu.mean.y), static_cast<double>(imu.mean.z), static_cast<double>(imu.spread.x),
                static_cast<double>(imu.spread.y), static_cast<double>(imu.spread.z));
    EXPECT_NEAR(b.pos.y, kRestHeight, 1.0e-3f) << "rest height";
    EXPECT_NEAR(glm::length(glm::vec2(b.pos.x, b.pos.z)), 0.0f, 5.0e-3f) << "it has not walked";
    EXPECT_LT(std::fabs(b.vel.y), 5.0e-3f) << "rest velocity (PHY-8: about -g dt = -1e-2 m/s at 1 ms)";
    EXPECT_NEAR(imu.mean.y, kG, 0.1f) << "the IMU reads +g up";
    EXPECT_LT(std::fabs(imu.mean.x), 0.1f);
    EXPECT_LT(std::fabs(imu.mean.z), 0.1f);
    EXPECT_LT(imu.spread.y, 0.5f) << "no jitter at rest";
}

TEST(X500GroundContact, ADropSettlesWithoutBouncingMoreThanTheRestitutionAllows) {
    const spade::vehicles::ModelType model = load_model(0.6f, 0.1f);
    Rig rig = make_rig(model, glm::vec3(0.0f, -kG, 0.0f), glm::vec3(0.0f, kDesignOriginAtRest + 0.5f, 0.0f), level(model));
    float highest_after_impact = 0.0f;
    bool landed = false;
    float min_y = 1e9f;
    for (int i = 0; i < 3000; ++i) {
        steps(rig, 1);
        const spade::BodyState b = state(rig);
        min_y = std::min(min_y, b.pos.y);
        if (!landed && b.vel.y > 0.0f && b.pos.y < kRestHeight + 0.05f) landed = true;
        if (landed) highest_after_impact = std::max(highest_after_impact, b.pos.y - kRestHeight);
    }
    const spade::BodyState b = state(rig);
    std::printf("[ ground  ] drop from 0.5 m: lowest centre of mass %.5f m, highest rebound %.4f m above rest, rests at %.5f m with speed %.2e m/s\n", static_cast<double>(min_y),
                static_cast<double>(highest_after_impact), static_cast<double>(b.pos.y), static_cast<double>(glm::length(b.vel)));
    // restitution 0.1: the rebound height is about e^2 x 0.5 m = 5 mm; allow 3x for the contact's own softness
    EXPECT_LT(highest_after_impact, 0.015f);
    EXPECT_NEAR(b.pos.y, kRestHeight, 1.0e-3f);
    EXPECT_LT(glm::length(b.vel), 1.0e-2f);
}

// DIAGNOSTIC (the first run of the drop above spun 5 rad/s after a level 3.1 m/s impact): a bare body of the X500's mass and inertia with the same twelve points,
// the centre of mass at the foot centroid or offset as the X500's, friction and restitution varied, dropped from 0.5 m; the angular speed 20 ms after the impact.
[[nodiscard]] float spin_after_drop(glm::vec3 com_in_design_xz, float friction, float restitution, uint32_t sweeps) {
    namespace rg = spade::test::rigid;
    rg::WorldOpts o;
    spade::Simulation sim = rg::make(rg::tumbling_set(), o);
    std::vector<glm::vec3> pts;
    for (const glm::vec3& p : pad_points_design()) pts.push_back(p - glm::vec3(com_in_design_xz.x, static_cast<float>(kComY), com_in_design_xz.z));
    const spade::BodyRef ref = rg::put(sim, glm::vec3(0.0f, kRestHeight + 0.5f, 0.0f), glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                       glm::vec3(1.0f / 0.0237f, 1.0f / 0.0290f, 1.0f / 0.0234f), 1.6326f);
    ph::ShapeDesc shape = ph::hull_shape(pts, kSkin);
    shape.friction = friction;
    shape.restitution = restitution;
    shape.iterations = sweeps;
    EXPECT_TRUE(rg::shaped(sim, ref, shape));
    float after = 0.0f;
    for (int i = 0; i < 400; ++i) {
        EXPECT_TRUE(sim.step(1).has_value());
        if (i == 330) after = glm::length(rg::body(sim, ref).omega_body);  // the impact is near step 307; this is 20 ms on
    }
    return after;
}

TEST(X500GroundContact, DiagnosticSpinAfterALevelDrop) {
    const float centred = spin_after_drop(glm::vec3(0.0f), 0.6f, 0.1f, 0);
    const float offset = spin_after_drop(glm::vec3(-0.0045f, 0.0f, 0.0009f), 0.6f, 0.1f, 0);
    const float no_friction = spin_after_drop(glm::vec3(-0.0045f, 0.0f, 0.0009f), 0.0f, 0.1f, 0);
    const float no_bounce = spin_after_drop(glm::vec3(-0.0045f, 0.0f, 0.0009f), 0.6f, 0.0f, 0);
    const float more_sweeps = spin_after_drop(glm::vec3(-0.0045f, 0.0f, 0.0009f), 0.6f, 0.1f, 16);
    std::printf("[ ground  ] spin 20 ms after a level drop (rad/s): centre of mass at the foot centroid %.3f; offset (-4.5, 0.9 mm) %.3f; offset with no friction %.3f; "
                "offset with no restitution %.3f; offset with 16 sweeps %.3f (a converged solve zeroes every point's normal speed: no spin)\n",
                static_cast<double>(centred), static_cast<double>(offset), static_cast<double>(no_friction), static_cast<double>(no_bounce), static_cast<double>(more_sweeps));
    EXPECT_LT(more_sweeps, 0.1f) << "16 sweeps converge: the spin after a level drop is under 0.1 rad/s";
}

// Pitch (about the design frame's +Z, nose up positive) or roll (about +X): the body placed with its lowest foot point 1 cm above the plane.
[[nodiscard]] float pitch_after(const spade::vehicles::ModelType& model, float degrees, glm::vec3 axis, float seconds) {
    const glm::quat tilt = glm::angleAxis(glm::radians(degrees), axis);
    const glm::quat orient = tilt * level(model);
    // the centre height that puts the lowest of the points (and the skin) 1 cm over the plane
    float low = 1e9f;
    for (const glm::vec3& p : pad_points_design()) low = std::min(low, (tilt * p).y);  // about the DESIGN origin: the spawn pose is the design frame's
    const float y = 0.01f + kSkin - low;
    Rig rig = make_rig(model, glm::vec3(0.0f, -kG, 0.0f), glm::vec3(0.0f, y, 0.0f), orient);
    steps(rig, static_cast<uint32_t>(seconds * 1000.0f));
    // the tilt from level: the angle of the design frame's up axis from world up
    const glm::quat design_orient = state(rig).orient * model.design_to_principal;  // body->world then design->body: design->world
    const glm::vec3 up = design_orient * glm::vec3(0.0f, 1.0f, 0.0f);
    return glm::degrees(std::acos(std::clamp(up.y, -1.0f, 1.0f)));
}

TEST(X500GroundContact, ThePitchedDroneRocksBackBelowTheTipOverAndTipsAbove) {
    const spade::vehicles::ModelType model = load_model(0.6f, 0.1f);
    const float held = pitch_after(model, 25.0f, glm::vec3(0.0f, 0.0f, 1.0f), 3.0f);
    const float tipped = pitch_after(model, 40.0f, glm::vec3(0.0f, 0.0f, 1.0f), 1.0f);
    std::printf("[ ground  ] pitch 25 degrees: tilt after 3 s %.2f degrees; pitch 40 degrees: tilt after 1 s %.1f degrees\n", static_cast<double>(held), static_cast<double>(tipped));
    EXPECT_LT(held, 3.0f) << "25 degrees of pitch rocks back to level";
    EXPECT_GT(tipped, 45.0f) << "40 degrees is past the tip-over (about 31 degrees) and keeps going";
}

TEST(X500GroundContact, ARolledDroneRocksBackBelowTheTipOverAndTipsAbove) {
    const spade::vehicles::ModelType model = load_model(0.6f, 0.1f);
    const float held = pitch_after(model, 25.0f, glm::vec3(1.0f, 0.0f, 0.0f), 3.0f);
    const float tipped = pitch_after(model, 45.0f, glm::vec3(1.0f, 0.0f, 0.0f), 1.0f);
    std::printf("[ ground  ] roll 25 degrees: tilt after 3 s %.2f degrees; roll 45 degrees: tilt after 1 s %.1f degrees\n", static_cast<double>(held), static_cast<double>(tipped));
    EXPECT_LT(held, 3.0f);
    EXPECT_GT(tipped, 50.0f) << "45 degrees is past the roll tip-over (about 34 degrees)";
}

// An inclined plane is a level plane under a tilted gravity: gravity 20 degrees off vertical along the design frame's +Z (a roll slope, below its tip-over).
[[nodiscard]] float slid_after(float friction, float seconds) {
    const spade::vehicles::ModelType model = load_model(friction, 0.0f);
    const float a = glm::radians(20.0f);
    Rig rig = make_rig(model, glm::vec3(0.0f, -kG * std::cos(a), kG * std::sin(a)), glm::vec3(0.0f, kDesignOriginAtRest + 0.005f, 0.0f), level(model));
    steps(rig, 500);  // settle
    const glm::vec3 from = state(rig).pos;
    steps(rig, static_cast<uint32_t>(seconds * 1000.0f));
    const glm::vec3 to = state(rig).pos;
    return glm::length(glm::vec2(to.x - from.x, to.z - from.z));
}

TEST(X500GroundContact, TheSlopeHoldsAboveTheFrictionAngleAndSlidesBelowIt) {
    const float holds = slid_after(0.6f, 2.0f);
    const float slides = slid_after(0.3f, 1.0f);
    std::printf("[ ground  ] 20 degree slope: mu 0.6 moved %.4f m in 2 s; mu 0.3 moved %.3f m in 1 s (tan 20 = 0.364)\n", static_cast<double>(holds), static_cast<double>(slides));
    EXPECT_LT(holds, 5.0e-3f) << "tan 20 = 0.364 < 0.6: it holds";
    EXPECT_GT(slides, 0.1f) << "tan 20 = 0.364 > 0.3: it slides (about 0.3 m in 1 s)";
}

}  // namespace
