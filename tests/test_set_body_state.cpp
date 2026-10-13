// test_set_body_state.cpp -- Simulation::set_body_state, re-posing a live body
// (docs/design/core/2026-10-13-set-body-state-design.md, section 6: Runtime's
// conformance cases, Spade's half). Numbered as the note numbers them. Test 2
// on Vulkan and test 10 (CPU and GPU after a set) are in test_gpu_parity.cpp,
// beside the other device tests; the Markov replay both backends run is
// set_body_state_support.hpp's.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "physics/forces.hpp"
#include "physics/integrator.hpp"
#include "physics/schedule.hpp"
#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "set_body_state_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/arenas.hpp"
#include "state/layout.hpp"
#include "state/registry.hpp"
#include "state/snapshot.hpp"
#include "state/state_view.hpp"
#include "testing/replay.hpp"
#include "vehicles/model_type.hpp"

namespace {

using spade::BodyRef;
using spade::BodySpawn;
using spade::BodyState;
using spade::Result;
using spade::Simulation;
using spade::WorldSetDesc;
using spade::test::bare_body;
using spade::test::floor_world_set;
using spade::test::row_of;
using spade::test::row_text;

constexpr uint64_t kDt = 1'000'000;
constexpr uint32_t kSubsteps = 2;

// A refusal's code as an int, -1 for a success (no .error() on a value).
template <class T>
[[nodiscard]] int code_of(const Result<T>& r) {
    return r ? -1 : static_cast<int>(r.error().code);
}
[[nodiscard]] constexpr int code(spade::Code c) { return static_cast<int>(c); }
// A refusal's text, "" for a success.
template <class T>
[[nodiscard]] std::string text_of(const Result<T>& r) {
    return r ? std::string() : r.error().context;
}

template <class T>
[[nodiscard]] bool same_bytes(const T& a, const T& b) {
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

// The texts the note's refusal table fixes (section 2).
constexpr std::string_view kFiniteRefusal = "set_body_state: pos/vel/omega_body must all be finite";
constexpr std::string_view kOrientRefusal =
    "set_body_state: orientation must be finite with |q| within 1 \xC2\xB1 2e-6 (got |q| = ";
constexpr std::string_view kUnflushedRefusal =
    "set_body_state: body is spawned but not yet flushed (call step() or flush_structural() first)";
constexpr std::string_view kBuriedRefusal =
    "set_body_state: position is deeper than one proxy radius inside the world SDF (uncapped positional "
    "correction would eject it)";

[[nodiscard]] Simulation make_sim(uint32_t bodies = 4) {
    const Result<WorldSetDesc> set = floor_world_set(bodies);
    EXPECT_TRUE(set.has_value()) << set.error().context;
    return Simulation::create(*set, kDt, kSubsteps).value();
}

// Every registered array's bytes, maps included, by name.
using ArrayBytes = std::map<std::string, std::vector<std::byte>>;
[[nodiscard]] ArrayBytes array_bytes(const Simulation& sim) {
    ArrayBytes out;
    sim.arenas().registry().for_each_array([&](const spade::RegisteredArray& array) {
        out[array.name].assign(array.data, array.data + array.byte_size());
    });
    return out;
}

// A pose S that is nowhere near anything the fixtures spawn: not buried, in
// tolerance, every field non-trivial.
struct Pose {
    glm::vec3 pos;
    glm::quat orient;
    glm::vec3 vel;
    glm::vec3 omega;
};
[[nodiscard]] Pose far_pose() {
    return {glm::vec3(-1.0f, 2.5f, 0.75f), glm::normalize(glm::quat(0.9f, 0.1f, 0.3f, -0.2f)),
            glm::vec3(0.5f, 1.0f, -0.25f), glm::vec3(0.2f, -0.3f, 0.4f)};
}

[[nodiscard]] Result<void> set_to(Simulation& sim, BodyRef ref, const Pose& p) {
    return sim.set_body_state(ref, p.pos, p.orient, p.vel, p.omega);
}

// ---------------------------------------------------------------------------
// THE RIG: one body carrying every kind of attached row the standard set has
// -- a vehicle with a rotor and an IMU (from its model), plus a drag element
// and a GNSS receiver -- and a bystander body, stepped until the rows and rings
// hold real content. The model overrides the proxy radius, so `proxy_radius`
// is a value the set can be seen to keep.
// ---------------------------------------------------------------------------
struct Rig {
    Simulation sim;
    spade::VehicleRef vehicle;
    spade::GnssSensorRef gnss;
    BodyRef bystander;
};

[[nodiscard]] spade::vehicles::ModelType rig_model() {
    spade::vehicles::ModelType model{};
    model.name = "set_body_state_rig";
    model.body.mass = 0.9f;
    model.body.inertia_diag = {0.02f, 0.03f, 0.025f};
    model.proxy_radius = 0.12f;
    spade::vehicles::RotorDesc rotor{};
    rotor.local_pos = {0.0f, 0.02f, 0.0f};
    rotor.spin_dir = 1.0f;
    rotor.tau = 0.02f;
    rotor.radius = 0.12f;
    rotor.thrust_coeff = 1.0e-5f;
    rotor.torque_coeff = 1.6e-7f;
    model.rotors.push_back(rotor);
    spade::vehicles::ImuMountDesc imu{};
    imu.mount_pos = {0.01f, 0.0f, -0.02f};
    imu.sigma_a = 0.05f;
    imu.sigma_g = 0.01f;
    imu.sigma_ba = 0.002f;
    imu.sigma_bg = 0.001f;
    model.imu_mounts.push_back(imu);
    return model;
}

[[nodiscard]] Rig make_rig() {
    Simulation sim = make_sim();
    const Result<spade::ModelTypeId> id = sim.register_model(rig_model());
    EXPECT_TRUE(id.has_value()) << id.error().context;
    spade::VehicleSpawn where{};
    where.pos = {0.0f, 2.0f, 0.0f};
    where.rotor_omega = 300.0f;
    const Result<spade::VehicleRef> vehicle = sim.spawn(0, *id, where);
    EXPECT_TRUE(vehicle.has_value()) << vehicle.error().context;
    spade::DragElementSpawn drag{};
    drag.mode = spade::physics::drag_mode::quadratic;
    drag.area = 0.02f;
    drag.coeffs = {1.0f, 0.0f, 0.0f};
    EXPECT_TRUE(sim.add_drag_element(vehicle->body, drag).has_value());
    spade::GnssSensorSpawn gnss{};
    gnss.rate_divider = 4;
    gnss.sigma_h = 0.5f;
    gnss.sigma_v = 0.8f;
    gnss.sigma_vel = 0.1f;
    gnss.bias_tau_s = 30.0f;
    gnss.sigma_bias = 1.5f;
    const Result<spade::GnssSensorRef> receiver = sim.add_gnss_sensor(vehicle->body, gnss);
    EXPECT_TRUE(receiver.has_value()) << receiver.error().context;
    const Result<BodyRef> bystander = sim.spawn(0, bare_body(glm::vec3(2.0f, 1.0f, 0.0f)));
    EXPECT_TRUE(bystander.has_value()) << bystander.error().context;
    EXPECT_TRUE(sim.flush_structural().has_value());
    const float command = 400.0f;
    EXPECT_TRUE(sim.set_rotor_commands(*vehicle, std::span<const float>(&command, 1)).has_value());
    EXPECT_TRUE(sim.step(10).has_value());
    return Rig{std::move(sim), *vehicle, *receiver, *bystander};
}

// A bare body with an IMU and a GNSS receiver, both noisy, flushed at tick 0.
struct SensedBody {
    Simulation sim;
    BodyRef body;
    spade::ImuSensorRef imu;
    spade::GnssSensorRef gnss;
};

[[nodiscard]] SensedBody make_sensed_body() {
    Simulation sim = make_sim();
    BodySpawn spawn = bare_body(glm::vec3(0.0f, 2.0f, 0.0f));
    spawn.vel = {0.3f, 0.5f, -0.1f};
    spawn.omega_body = {0.1f, 0.2f, -0.3f};
    const Result<BodyRef> body = sim.spawn(0, spawn);
    EXPECT_TRUE(body.has_value()) << body.error().context;
    spade::ImuSensorSpawn imu{};
    imu.sigma_a = 0.05f;
    imu.sigma_g = 0.01f;
    imu.sigma_ba = 0.002f;
    imu.sigma_bg = 0.001f;
    const Result<spade::ImuSensorRef> imu_ref = sim.add_imu_sensor(*body, imu);
    EXPECT_TRUE(imu_ref.has_value()) << imu_ref.error().context;
    spade::GnssSensorSpawn gnss{};
    gnss.rate_divider = 4;
    gnss.sigma_h = 0.5f;
    gnss.sigma_v = 0.8f;
    gnss.sigma_vel = 0.1f;
    gnss.bias_tau_s = 30.0f;
    gnss.sigma_bias = 1.5f;
    const Result<spade::GnssSensorRef> gnss_ref = sim.add_gnss_sensor(*body, gnss);
    EXPECT_TRUE(gnss_ref.has_value()) << gnss_ref.error().context;
    EXPECT_TRUE(sim.flush_structural().has_value());
    return SensedBody{std::move(sim), *body, *imu_ref, *gnss_ref};
}

}  // namespace

// ===========================================================================
// 1. ROUND TRIP (Kat 1). body() returns exactly what was set -- a quaternion in
// tolerance but not normalized included -- specific_force reads 0, a wrench
// applied before the set is still pending, and nothing else in the row, the
// tick or the queue moves.
// ===========================================================================
TEST(SetBodyState, RoundTripsTheBodyRowsOwnStateBitForBit) {
    Rig rig = make_rig();
    const BodyRef body = rig.vehicle.body;
    ASSERT_NE(row_of(rig.sim, body).specific_force, glm::vec3(0.0f))
        << "the rotor's thrust gives the body a non-zero specific force before the set";
    const glm::vec3 force(0.25f, -0.5f, 0.125f);
    const glm::vec3 torque(0.01f, -0.02f, 0.005f);
    ASSERT_TRUE(rig.sim.apply_wrench(body, force, torque).has_value());
    const BodyState pending = row_of(rig.sim, body);

    // In tolerance (|q| about 1 + 1.5e-6) and NOT normalized: the setter must
    // store it as given, where spawn would normalize it.
    const double d[4] = {0.7, 0.3, -0.5, 0.4};
    const double norm = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2] + d[3] * d[3]);
    const double s = (1.0 + 1.5e-6) / norm;
    const glm::quat q(static_cast<float>(d[0] * s), static_cast<float>(d[1] * s), static_cast<float>(d[2] * s),
                      static_cast<float>(d[3] * s));
    ASSERT_FALSE(same_bytes(glm::normalize(q), q)) << "the case needs a quaternion normalize would change";
    const glm::vec3 pos(0.5f, 1.5f, -0.25f);
    const glm::vec3 vel(1.0f, -0.5f, 0.25f);
    const glm::vec3 omega(0.3f, -0.7f, 1.1f);
    const uint64_t tick = rig.sim.tick().value;
    const std::size_t queued = rig.sim.pending_structural_ops();

    const Result<void> placed = rig.sim.set_body_state(body, pos, q, vel, omega);
    ASSERT_TRUE(placed.has_value()) << placed.error().context;

    const BodyState after = row_of(rig.sim, body);
    EXPECT_TRUE(same_bytes(after.pos, pos));
    EXPECT_TRUE(same_bytes(after.orient, q)) << "the quaternion is stored as given, never normalized";
    EXPECT_TRUE(same_bytes(after.vel, vel));
    EXPECT_TRUE(same_bytes(after.omega_body, omega));
    EXPECT_TRUE(same_bytes(after.specific_force, glm::vec3(0.0f))) << "specific_force reads +0 until the next step";
    EXPECT_TRUE(same_bytes(after.force_acc, pending.force_acc)) << "the pending wrench's force is kept";
    EXPECT_TRUE(same_bytes(after.torque_acc, pending.torque_acc)) << "the pending wrench's torque is kept";
    EXPECT_EQ(after.force_acc, force);
    EXPECT_EQ(after.torque_acc, torque);
    EXPECT_EQ(after.proxy_radius, 0.12f) << "the model's override";
    BodyState expected = pending;
    expected.pos = pos;
    expected.orient = q;
    expected.vel = vel;
    expected.omega_body = omega;
    expected.specific_force = glm::vec3(0.0f);
    EXPECT_TRUE(same_bytes(after, expected))
        << "mass, inertia, proxy radius, flags and the pads are the row's as it was\n  expected "
        << row_text(expected) << "\n  got      " << row_text(after);
    EXPECT_EQ(rig.sim.tick().value, tick);
    EXPECT_EQ(rig.sim.pending_structural_ops(), queued);
}

// ===========================================================================
// 2 and 3. THE MARKOV REPLAY (Kat 2a and 2b), on the CPU; test_gpu_parity.cpp
// runs test 2 on Vulkan. Exact both ways, because omega_body is native.
// ===========================================================================
TEST(SetBodyState, MarkovReplayIsExact) {
    spade::test::expect_markov_replay_exact(spade::compute::BackendDesc{}, spade::test::MarkovCase{},
                                            "cpu, omega = 0");
}

TEST(SetBodyState, MarkovReplayWithRotationIsExact) {
    spade::test::MarkovCase c;
    c.omega0 = {0.8f, -0.5f, 1.2f};
    c.torque = {0.01f, -0.02f, 0.015f};
    spade::test::expect_markov_replay_exact(spade::compute::BackendDesc{}, c, "cpu, omega != 0");
}

// ===========================================================================
// 4. THE TWIN. A body set to S and a body SPAWNED at S evolve bit-identically,
// with no wrench pending. The set body rested on the floor first (contact
// history); the twin's world stepped empty to the same tick. S lands on the
// floor within the run, so contact after the set is exercised too.
// ===========================================================================
TEST(SetBodyState, ASetBodyAndASpawnedTwinEvolveBitIdentically) {
    constexpr uint64_t kRest = 20;
    constexpr uint64_t kSteps = 200;

    // PRECONDITION, asserted: spawn normalizes and the setter does not, so S's
    // quaternion must be a fixed point of glm::normalize, bit for bit.
    glm::quat q = glm::normalize(glm::quat(0.8f, 0.2f, -0.4f, 0.3f));
    for (int i = 0; i < 16 && !same_bytes(glm::normalize(q), q); ++i) {
        q = glm::normalize(q);
    }
    ASSERT_TRUE(same_bytes(glm::normalize(q), q)) << "no fixed point of normalize near the chosen quaternion";
    const Pose s{glm::vec3(0.3f, 0.4f, -0.2f), q, glm::vec3(0.5f, -2.0f, 0.1f), glm::vec3(0.6f, -0.4f, 0.9f)};

    Simulation set = make_sim();
    const Result<BodyRef> resting = set.spawn(0, bare_body(glm::vec3(-1.0f, 0.1f, 0.5f)));
    ASSERT_TRUE(resting.has_value()) << resting.error().context;
    ASSERT_TRUE(set.step(kRest).has_value());
    ASSERT_GT(row_of(set, *resting).specific_force.y, 1.0f) << "the body rests on the floor before the set";
    const Result<void> placed = set_to(set, *resting, s);
    ASSERT_TRUE(placed.has_value()) << placed.error().context;

    Simulation twin = make_sim();
    ASSERT_TRUE(twin.step(kRest).has_value());
    BodySpawn spawn = bare_body(s.pos);
    spawn.orient = s.orient;
    spawn.vel = s.vel;
    spawn.omega_body = s.omega;
    const Result<BodyRef> spawned = twin.spawn(0, spawn);
    ASSERT_TRUE(spawned.has_value()) << spawned.error().context;
    ASSERT_TRUE(twin.flush_structural().has_value());
    ASSERT_EQ(*spawned, *resting);

    EXPECT_EQ(spade::testing::state_digest(set), spade::testing::state_digest(twin))
        << "at the set, before any step";
    bool landed = false;
    float vy = s.vel.y;
    for (uint64_t i = 0; i < kSteps; ++i) {
        ASSERT_TRUE(set.step(1).has_value());
        ASSERT_TRUE(twin.step(1).has_value());
        const BodyState a = row_of(set, *resting);
        const BodyState b = row_of(twin, *spawned);
        ASSERT_TRUE(same_bytes(a, b)) << "tick " << set.tick().value << "\n  set   " << row_text(a)
                                      << "\n  twin  " << row_text(b);
        // Gravity alone only lowers vel.y; the floor's impulse raises it.
        landed = landed || a.vel.y > vy;
        vy = a.vel.y;
    }
    EXPECT_TRUE(landed) << "S should reach the floor within the run, so contact after the set is compared; at the "
                        << "end: " << row_text(row_of(set, *resting));
    EXPECT_EQ(spade::testing::state_digest(set), spade::testing::state_digest(twin));
}

// ===========================================================================
// 5. A NO-OP SET AT TICK 0 (Kat 3). A flushed body with an IMU and a GNSS
// receiver, set to its own body() state: the whole digest, sensors included,
// equals an untouched twin's, after the call and after N steps.
// ===========================================================================
TEST(SetBodyState, ANoOpSetAtTickZeroMovesNoDigest) {
    SensedBody set = make_sensed_body();
    SensedBody twin = make_sensed_body();
    ASSERT_EQ(set.sim.tick().value, 0u);
    const BodyState own = row_of(set.sim, set.body);
    const Result<void> placed = set.sim.set_body_state(set.body, own.pos, own.orient, own.vel, own.omega_body);
    ASSERT_TRUE(placed.has_value()) << placed.error().context;
    EXPECT_EQ(spade::testing::state_digest(set.sim), spade::testing::state_digest(twin.sim)) << "after the call";

    ASSERT_TRUE(set.sim.step(30).has_value());
    ASSERT_TRUE(twin.sim.step(30).has_value());
    EXPECT_EQ(spade::testing::state_digest(set.sim), spade::testing::state_digest(twin.sim)) << "after 30 steps";
    const Result<const spade::sensors::ImuSensorRow*> imu = set.sim.imu_sensor(set.imu);
    ASSERT_TRUE(imu.has_value()) << imu.error().context;
    EXPECT_GT((*imu)->last_index, 0u) << "the IMU sampled, so the digest covers live sensor state";
    const Result<const spade::sensors::GnssSensorRow*> gnss = set.sim.gnss_sensor(set.gnss);
    ASSERT_TRUE(gnss.has_value()) << gnss.error().context;
    EXPECT_GT((*gnss)->last_index, 0u) << "the receiver fixed, so the digest covers live receiver state";
}

// ===========================================================================
// 6. BINDINGS SURVIVE (Kat 4). Every array but `bodies` -- the IMU and GNSS
// rows, their rings, the rotor and drag rows, the maps, the generations -- is
// byte-equal across the call, and last_index does not move. After N steps the
// sensors' noise streams and bias states equal an un-set twin's: their draws do
// not depend on kinematics. The samples themselves differ.
// ===========================================================================
TEST(SetBodyState, SensorAndElementBindingsSurvive) {
    Rig set = make_rig();
    Rig twin = make_rig();
    const spade::ImuSensorRef imu = set.vehicle.imu_sensors[0];
    const spade::sensors::SampleIndex imu_index = (*set.sim.imu_sensor(imu))->last_index;
    const spade::sensors::SampleIndex gnss_index = (*set.sim.gnss_sensor(set.gnss))->last_index;
    ASSERT_GT(imu_index, 0u);
    ASSERT_GT(gnss_index, 0u);
    const ArrayBytes before = array_bytes(set.sim);

    const Result<void> placed = set_to(set.sim, set.vehicle.body, far_pose());
    ASSERT_TRUE(placed.has_value()) << placed.error().context;

    const ArrayBytes after = array_bytes(set.sim);
    ASSERT_EQ(before.size(), after.size());
    for (const auto& [name, bytes] : before) {
        if (name == "bodies") continue;
        EXPECT_TRUE(after.at(name) == bytes) << "array '" << name << "' changed under set_body_state";
    }
    EXPECT_EQ((*set.sim.imu_sensor(imu))->last_index, imu_index);
    EXPECT_EQ((*set.sim.gnss_sensor(set.gnss))->last_index, gnss_index);

    ASSERT_TRUE(set.sim.step(40).has_value());
    ASSERT_TRUE(twin.sim.step(40).has_value());
    const spade::sensors::ImuSensorRow& a = **set.sim.imu_sensor(imu);
    const spade::sensors::ImuSensorRow& b = **twin.sim.imu_sensor(twin.vehicle.imu_sensors[0]);
    EXPECT_TRUE(same_bytes(a.noise, b.noise)) << "IMU noise stream";
    EXPECT_TRUE(same_bytes(a.bias_a, b.bias_a)) << "IMU accel bias";
    EXPECT_TRUE(same_bytes(a.bias_g, b.bias_g)) << "IMU gyro bias";
    EXPECT_EQ(a.last_index, b.last_index);
    const spade::sensors::GnssSensorRow& g = **set.sim.gnss_sensor(set.gnss);
    const spade::sensors::GnssSensorRow& h = **twin.sim.gnss_sensor(twin.gnss);
    EXPECT_TRUE(same_bytes(g.noise, h.noise)) << "GNSS noise stream";
    EXPECT_TRUE(same_bytes(g.bias, h.bias)) << "GNSS bias";
    EXPECT_EQ(g.last_index, h.last_index);
    EXPECT_FALSE(array_bytes(set.sim).at("imu_ring") == array_bytes(twin.sim).at("imu_ring"))
        << "the samples should differ: the set moved the body";
}

// ===========================================================================
// 7. TIME (Kat 5; the time-zero rule is Kat's). Our half: a set is legal at a
// mid-run boundary, and an unflushed body is refused with not_found.
// ===========================================================================
TEST(SetBodyState, IsLegalAtAMidRunBoundaryAndRefusesAnUnflushedBody) {
    Simulation sim = make_sim();
    const Result<BodyRef> body = sim.spawn(0, bare_body(glm::vec3(0.0f, 1.0f, 0.0f)));
    ASSERT_TRUE(body.has_value()) << body.error().context;
    ASSERT_TRUE(sim.step(7).has_value());
    const Pose p = far_pose();
    const Result<void> placed = set_to(sim, *body, p);
    ASSERT_TRUE(placed.has_value()) << placed.error().context;
    EXPECT_TRUE(same_bytes(row_of(sim, *body).pos, p.pos));
    EXPECT_TRUE(sim.step(1).has_value());

    const Result<BodyRef> queued = sim.spawn(0, bare_body(glm::vec3(1.0f, 1.0f, 0.0f)));
    ASSERT_TRUE(queued.has_value()) << queued.error().context;
    const Result<void> refused = set_to(sim, *queued, p);
    EXPECT_EQ(code_of(refused), code(spade::Code::not_found));
    EXPECT_EQ(text_of(refused), kUnflushedRefusal);
    ASSERT_TRUE(sim.flush_structural().has_value());
    EXPECT_TRUE(set_to(sim, *queued, p).has_value()) << "flushed, the same call is legal";
}

// ===========================================================================
// 8. BAD ARGUMENTS (Kat 6). Every refusal of the note's table, with its code
// and text, and the state digest unchanged after each.
// ===========================================================================
TEST(SetBodyState, RefusesEveryBadArgumentWithoutMovingState) {
    Simulation sim = make_sim();
    const Result<BodyRef> body = sim.spawn(0, bare_body(glm::vec3(0.0f, 1.0f, 0.0f)));
    ASSERT_TRUE(body.has_value()) << body.error().context;
    ASSERT_TRUE(sim.step(3).has_value());
    const Pose good = far_pose();

    const auto expect_refused = [&](BodyRef ref, const Pose& p, spade::Code expected_code, std::string_view text,
                                    bool prefix_only, const std::string& what) {
        const uint64_t digest = spade::testing::state_digest(sim);
        const Result<void> r = set_to(sim, ref, p);
        EXPECT_EQ(code_of(r), code(expected_code)) << what << ": " << text_of(r);
        if (prefix_only) {
            EXPECT_TRUE(text_of(r).starts_with(text)) << what << ": " << text_of(r);
            EXPECT_TRUE(text_of(r).ends_with(")")) << what << ": " << text_of(r);
        } else {
            EXPECT_EQ(text_of(r), text) << what;
        }
        EXPECT_EQ(spade::testing::state_digest(sim), digest) << what << ": a refusal moved state";
    };

    // Non-finite pos, vel, omega_body: NaN, +Inf and -Inf, on every component.
    const float bad[3] = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity()};
    for (int field = 0; field < 3; ++field) {
        for (int v = 0; v < 3; ++v) {
            for (int c = 0; c < 3; ++c) {
                Pose p = good;
                glm::vec3& target = field == 0 ? p.pos : field == 1 ? p.vel : p.omega;
                target[c] = bad[v];
                expect_refused(*body, p, spade::Code::invalid_argument, kFiniteRefusal, false,
                               "field " + std::to_string(field) + " value " + std::to_string(v) + " component " +
                                   std::to_string(c));
            }
        }
    }

    // The orientation: non-finite components, |q| just outside each bound, 0.
    for (int v = 0; v < 3; ++v) {
        for (int c = 0; c < 4; ++c) {
            Pose p = good;
            p.orient[c] = bad[v];
            expect_refused(*body, p, spade::Code::invalid_argument, kOrientRefusal, true,
                           "orient value " + std::to_string(v) + " component " + std::to_string(c));
        }
    }
    for (const double scale : {1.0 + 2.5e-6, 1.0 - 2.5e-6, 0.0}) {
        Pose p = good;
        const double norm = std::sqrt(0.36 + 0.09 + 0.25 + 0.04);
        p.orient = glm::quat(static_cast<float>(0.6 / norm * scale), static_cast<float>(-0.3 / norm * scale),
                             static_cast<float>(0.5 / norm * scale), static_cast<float>(0.2 / norm * scale));
        expect_refused(*body, p, spade::Code::invalid_argument, kOrientRefusal, true,
                       "|q| scaled by " + std::to_string(scale));
    }
    {
        Pose p = good;
        p.orient = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
        const Result<void> r = set_to(sim, *body, p);
        EXPECT_EQ(text_of(r), std::string(kOrientRefusal) + "0)") << "the zero quaternion reports |q| = 0";
    }

    // Refs: null, foreign, outside the partition, unallocated, stale, despawn-queued.
    expect_refused(BodyRef{}, good, spade::Code::not_found, "body ref is null", false, "null ref");
    {
        Result<WorldSetDesc> two = floor_world_set();
        ASSERT_TRUE(two.has_value());
        const spade::WorldInstanceDesc first = two->worlds[0];
        two->worlds.push_back(first);
        Result<Simulation> other = Simulation::create(*two, kDt, kSubsteps);
        ASSERT_TRUE(other.has_value()) << other.error().context;
        const Result<BodyRef> foreign = other->spawn(1, bare_body(glm::vec3(0.0f, 1.0f, 0.0f)));
        ASSERT_TRUE(foreign.has_value()) << foreign.error().context;
        expect_refused(*foreign, good, spade::Code::not_found, "body ref names a world outside this set", false,
                       "a ref from another Simulation's world 1");
    }
    const uint32_t capacity = sim.layout().body_capacity;
    expect_refused(BodyRef{0, capacity, 1}, good, spade::Code::not_found,
                   "body ref's slot does not lie in its world's partition", false, "a slot outside world 0");
    expect_refused(BodyRef{0, capacity - 1, 1}, good, spade::Code::not_found,
                   "body ref names a slot that is not allocated", false, "an unallocated slot");
    {
        const Result<BodyRef> doomed = sim.spawn(0, bare_body(glm::vec3(1.0f, 1.0f, 0.0f)));
        ASSERT_TRUE(doomed.has_value());
        ASSERT_TRUE(sim.step(1).has_value());
        ASSERT_TRUE(sim.despawn(*doomed).has_value());
        expect_refused(*doomed, good, spade::Code::not_found, "body ref is stale: its slot has been recycled", false,
                       "the held ref of a body whose despawn is queued");
        const BodyRef even{doomed->world_index, doomed->slot, doomed->generation + 1};
        expect_refused(even, good, spade::Code::not_found, "body ref names a body whose despawn is already queued",
                       false, "a ref minted between the despawn and the boundary");
        ASSERT_TRUE(sim.step(1).has_value());
        const Result<BodyRef> reborn = sim.spawn(0, bare_body(glm::vec3(1.0f, 1.0f, 0.0f)));
        ASSERT_TRUE(reborn.has_value());
        ASSERT_EQ(reborn->slot, doomed->slot);
        ASSERT_TRUE(sim.step(1).has_value());
        expect_refused(*doomed, good, spade::Code::not_found, "body ref is stale: its slot has been recycled", false,
                       "a ref to a slot recycled since");
    }

    // Unflushed, then buried.
    {
        const Result<BodyRef> queued = sim.spawn(0, bare_body(glm::vec3(-1.0f, 1.0f, 0.0f)));
        ASSERT_TRUE(queued.has_value());
        expect_refused(*queued, good, spade::Code::not_found, kUnflushedRefusal, false, "unflushed");
        ASSERT_TRUE(sim.flush_structural().has_value());
    }
    Pose buried = good;
    buried.pos = glm::vec3(0.0f, -0.3f, 0.0f);
    expect_refused(*body, buried, spade::Code::invalid_argument, kBuriedRefusal, false, "buried");
}

// The table's ORDER: of two faults, the earlier row reports.
TEST(SetBodyState, RefusesInTheNotesOrder) {
    Simulation sim = make_sim();
    const Result<BodyRef> body = sim.spawn(0, bare_body(glm::vec3(0.0f, 1.0f, 0.0f)));
    ASSERT_TRUE(body.has_value());
    ASSERT_TRUE(sim.step(1).has_value());
    const Result<BodyRef> queued = sim.spawn(0, bare_body(glm::vec3(1.0f, 1.0f, 0.0f)));
    ASSERT_TRUE(queued.has_value());
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const Pose good = far_pose();

    Pose nan_pos = good;
    nan_pos.pos.x = nan;
    EXPECT_EQ(text_of(set_to(sim, BodyRef{}, nan_pos)), "body ref is null") << "the ref before the arguments";

    Pose nan_pos_zero_q = nan_pos;
    nan_pos_zero_q.orient = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_EQ(text_of(set_to(sim, *body, nan_pos_zero_q)), kFiniteRefusal) << "pos/vel/omega before the quaternion";

    Pose zero_q = good;
    zero_q.orient = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(text_of(set_to(sim, *queued, zero_q)).starts_with(kOrientRefusal))
        << "the quaternion before the flush";

    Pose buried = good;
    buried.pos = glm::vec3(1.0f, -0.3f, 0.0f);
    EXPECT_EQ(text_of(set_to(sim, *queued, buried)), kUnflushedRefusal) << "the flush before the burial";
}

// THE BURIAL IS SPAWN'S, EXACTLY (Q1, confirmed by Kat's Runtime): at every
// rung of a depth ladder, the setter accepts iff spawn accepts -- for a bare
// body at the world's radius and for a vehicle at its model's own -- and its
// refusal is spawn's message under its own name.
TEST(SetBodyState, RefusesExactlyThePosesSpawnRefuses) {
    spade::vehicles::ModelType wide{};
    wide.name = "wide";
    wide.body.mass = 1.0f;
    wide.body.inertia_diag = {0.02f, 0.02f, 0.03f};
    wide.proxy_radius = 0.2f;  // the floor's grid cell (0.5) admits up to 0.25

    Simulation sim = make_sim(32);
    Simulation probe = make_sim(32);
    const Result<spade::ModelTypeId> id = sim.register_model(wide);
    ASSERT_TRUE(id.has_value()) << id.error().context;
    ASSERT_TRUE(probe.register_model(wide).has_value());
    const Result<BodyRef> bare = sim.spawn(0, bare_body(glm::vec3(0.0f, 1.0f, 0.0f)));
    ASSERT_TRUE(bare.has_value());
    spade::VehicleSpawn high{};
    high.pos = {1.0f, 1.0f, 0.0f};
    const Result<spade::VehicleRef> vehicle = sim.spawn(0, *id, high);
    ASSERT_TRUE(vehicle.has_value()) << vehicle.error().context;
    ASSERT_TRUE(sim.flush_structural().has_value());

    const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::vec3 still(0.0f);
    int bare_refused = 0, bare_accepted = 0, vehicle_refused = 0, vehicle_accepted = 0;
    for (const float y : {1.0f, 0.0f, -0.05f, -0.0999f, -0.1f, -0.1001f, -0.15f, -0.1999f, -0.2f, -0.2001f, -0.3f}) {
        const glm::vec3 pos(0.3f, y, -0.2f);

        const Result<BodyRef> spawned = probe.spawn(0, bare_body(pos));
        const Result<void> set = sim.set_body_state(*bare, pos, identity, still, still);
        EXPECT_EQ(set.has_value(), spawned.has_value()) << "bare body at y = " << y << ": " << text_of(set);
        if (!spawned.has_value()) {
            ++bare_refused;
            ASSERT_TRUE(text_of(spawned).starts_with("spawn: ")) << text_of(spawned);
            EXPECT_EQ(text_of(set), "set_body_state" + text_of(spawned).substr(std::string_view("spawn").size()))
                << "y = " << y;
        } else {
            ++bare_accepted;
        }

        spade::VehicleSpawn where{};
        where.pos = pos;
        const Result<spade::VehicleRef> manufactured = probe.spawn(0, *id, where);
        const Result<void> moved = sim.set_body_state(vehicle->body, pos, identity, still, still);
        EXPECT_EQ(moved.has_value(), manufactured.has_value()) << "vehicle at y = " << y << ": " << text_of(moved);
        if (!manufactured.has_value()) {
            ++vehicle_refused;
            ASSERT_TRUE(text_of(manufactured).starts_with("spawn(vehicle): ")) << text_of(manufactured);
            EXPECT_EQ(text_of(moved),
                      "set_body_state" + text_of(manufactured).substr(std::string_view("spawn(vehicle)").size()))
                << "y = " << y;
        } else {
            ++vehicle_accepted;
        }
    }
    EXPECT_GT(bare_refused, 0);
    EXPECT_GT(bare_accepted, 0);
    EXPECT_GT(vehicle_refused, 0);
    EXPECT_GT(vehicle_accepted, bare_accepted) << "the model's wider radius reaches the setter";
}

namespace {

// "poisoner": a per_world array and a pass that, while g_poison is set, reads
// it with a row type of the wrong size -- a misused view, which poisons the
// Simulation (step()'s note).
bool g_poison = false;
struct PoisonRow {
    uint32_t runs;
    uint32_t _pad[3];
};
void poison_pass(const spade::physics::SubstepContext& ctx) noexcept {
    if (ctx.state.empty()) return;
    if (g_poison) {
        (void)spade::world_rows<uint64_t>(ctx.state[0], 0);
    }
}
constexpr spade::modules::ArrayDecl kPoisonArrays[] = {
    {.name = "poison_marks", .elem_size = spade::modules::row_size<PoisonRow>()}};
constexpr spade::modules::QuantityAccess kPoisonAccess[] = {
    {.quantity = "poisoner.poison_marks", .access = spade::modules::Access::write}};
constexpr spade::modules::PassDecl kPoisonPasses[] = {
    {.name = "misuse", .phase = spade::modules::Phase::forces, .access = kPoisonAccess, .cpu = &poison_pass}};

}  // namespace

// Poisoned: the first row, before even the ref, with step()'s text verbatim.
TEST(SetBodyState, RefusesWhilePoisonedWithStepsText) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "poisoner", .passes = kPoisonPasses, .state = kPoisonArrays});
    const Result<WorldSetDesc> desc = floor_world_set();
    ASSERT_TRUE(desc.has_value());
    Result<Simulation> sim = Simulation::create(*desc, kDt, kSubsteps, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const Result<BodyRef> body = sim->spawn(0, bare_body(glm::vec3(0.0f, 1.0f, 0.0f)));
    ASSERT_TRUE(body.has_value());
    ASSERT_TRUE(sim->step(2).has_value());
    const Result<spade::SnapshotBlob> clean = sim->snapshot();
    ASSERT_TRUE(clean.has_value()) << clean.error().context;

    g_poison = true;
    const Result<void> fault = sim->step(1);
    g_poison = false;
    ASSERT_FALSE(fault.has_value()) << "the misuse must refuse the step";
    const Result<void> poisoned_step = sim->step(1);
    ASSERT_FALSE(poisoned_step.has_value());
    const std::string poisoned = poisoned_step.error().context;
    ASSERT_TRUE(poisoned.starts_with("step: state invalid since tick 2: ")) << poisoned;

    const uint64_t digest = spade::testing::state_digest(*sim);
    const Result<void> refused = set_to(*sim, *body, far_pose());
    EXPECT_EQ(code_of(refused), code(spade::Code::invalid_argument));
    EXPECT_EQ(text_of(refused), poisoned) << "step()'s text, verbatim";
    EXPECT_EQ(text_of(set_to(*sim, BodyRef{}, far_pose())), poisoned) << "poisoned before the ref is checked";
    EXPECT_EQ(spade::testing::state_digest(*sim), digest) << "a refusal moved state";

    ASSERT_TRUE(sim->restore(*clean).has_value());
    EXPECT_TRUE(set_to(*sim, *body, far_pose()).has_value()) << "a restore clears the poison";
}

// ===========================================================================
// 9. RESTORE AFTER A SET (Kat 7; the return to spawn is Kat's). Snapshot, set,
// step, restore: the snapshot's state. And a snapshot taken after a set
// restores the set state.
// ===========================================================================
TEST(SetBodyState, RestoreAfterASetReturnsTheBlobsState) {
    SensedBody rig = make_sensed_body();
    ASSERT_TRUE(rig.sim.step(10).has_value());
    const Result<spade::SnapshotBlob> before = rig.sim.snapshot();
    ASSERT_TRUE(before.has_value()) << before.error().context;
    const uint64_t before_digest = spade::testing::state_digest(rig.sim);

    const Pose p = far_pose();
    ASSERT_TRUE(set_to(rig.sim, rig.body, p).has_value());
    ASSERT_TRUE(rig.sim.step(5).has_value());
    ASSERT_TRUE(rig.sim.restore(*before).has_value());
    EXPECT_EQ(spade::testing::state_digest(rig.sim), before_digest) << "the blob's state, not the set's";
    const Result<spade::SnapshotBlob> again = rig.sim.snapshot();
    ASSERT_TRUE(again.has_value());
    EXPECT_TRUE(std::ranges::equal(again->bytes(), before->bytes()));

    ASSERT_TRUE(set_to(rig.sim, rig.body, p).has_value());
    const uint64_t set_digest = spade::testing::state_digest(rig.sim);
    const Result<spade::SnapshotBlob> after_set = rig.sim.snapshot();
    ASSERT_TRUE(after_set.has_value()) << after_set.error().context;
    ASSERT_TRUE(rig.sim.step(5).has_value());
    ASSERT_TRUE(rig.sim.restore(*after_set).has_value());
    EXPECT_EQ(spade::testing::state_digest(rig.sim), set_digest) << "a snapshot after a set carries the set";
    const BodyState row = row_of(rig.sim, rig.body);
    EXPECT_TRUE(same_bytes(row.pos, p.pos));
    EXPECT_TRUE(same_bytes(row.orient, p.orient));
    EXPECT_TRUE(same_bytes(row.vel, p.vel));
    EXPECT_TRUE(same_bytes(row.omega_body, p.omega));
}

// ===========================================================================
// 11. THE CENSUS (the lead's Q4 guard). Every per-body array of the standard
// set -- enumerated from the engine's own registry walk and schedule, not from
// a list -- must be classified below, kept or cleared, as simulation.hpp's
// census beside mark_vulkan_dirty()'s classifies it; a new one fails by name
// until someone classifies it. The set is then checked against the census,
// byte for byte, on a body carrying every kind of attached row.
// ===========================================================================
namespace {

enum class Ruling { written, cleared, kept, pad };

struct ArrayRuling {
    std::string_view array;
    Ruling ruling;
};

// simulation.hpp's census, row for row. A row's `.slot_to_world` map rides
// with it: the setter allocates and frees nothing.
constexpr ArrayRuling kPerBodyCensus[] = {
    {"bodies", Ruling::written},          // the body row; field by field in kBodyFields
    {"body_generation", Ruling::kept},    // refs survive
    {"drag_bodies", Ruling::kept},        // an attached element
    {"imu_sensors", Ruling::kept},        // stream, bias, phase, last_index
    {"imu_ring", Ruling::kept},           // samples already written
    {"rotors", Ruling::kept},             // omega and omega_cmd are not body state
    {"gnss_sensors", Ruling::kept},       // stream, bias, phase, last_index
    {"gnss_ring", Ruling::kept},          // fixes already written
};

// Per-body SCRATCH: not state, and zero at every boundary by its module's own
// invariant (integrate zeroes contact_dv every substep), so nothing to clear.
constexpr std::string_view kPerBodyScratchCensus[] = {"contact_dv"};

// The core arrays' extents, which the schedule does not carry.
constexpr std::pair<std::string_view, bool> kCoreArrayIsPerBody[] = {
    {"world_params", false}, {"bodies", true}, {"body_generation", true}, {"replay_config", false}};

struct FieldRuling {
    std::string_view field;
    std::size_t offset;
    std::size_t size;
    Ruling ruling;
};

#define SBS_FIELD(name, ruling) FieldRuling{#name, offsetof(BodyState, name), sizeof(BodyState::name), Ruling::ruling}
const FieldRuling kBodyFields[] = {
    SBS_FIELD(pos, written),        SBS_FIELD(proxy_radius, kept), SBS_FIELD(orient, written),
    SBS_FIELD(vel, written),        SBS_FIELD(mass, kept),         SBS_FIELD(omega_body, written),
    SBS_FIELD(_p1, pad),            SBS_FIELD(inv_inertia_diag, kept), SBS_FIELD(flags, kept),
    SBS_FIELD(force_acc, kept),     SBS_FIELD(_p2, pad),           SBS_FIELD(torque_acc, kept),
    SBS_FIELD(_p3, pad),            SBS_FIELD(specific_force, cleared), SBS_FIELD(_p4, pad),
};
#undef SBS_FIELD

struct CensusGaps {
    std::vector<std::string> unknown;           // a registered array whose extent nothing here knows
    std::vector<std::string> unlisted;          // a per-body array the census does not classify
    std::vector<std::string> stale;             // a census row naming no per-body array
    std::vector<std::string> unlisted_scratch;  // a per-body scratch the census does not list
    std::vector<std::string> stale_scratch;
};

// Every per-body array in `sim`'s registry walk: the core's two, and every
// module array whose rows belong to a body -- per_body, per_element and
// per_sensor, and per_row through its owner.
[[nodiscard]] std::vector<std::string> per_body_arrays(const Simulation& sim, std::vector<std::string>& unknown) {
    const std::vector<spade::modules::CompiledArray>& arrays = sim.schedule().arrays;
    const auto owned_by_a_body = [&](uint32_t i) {
        for (std::size_t hops = 0; hops <= arrays.size() && i < arrays.size(); ++hops) {
            switch (arrays[i].extent) {
                case spade::modules::Extent::per_world: return false;
                case spade::modules::Extent::per_body:
                case spade::modules::Extent::per_element:
                case spade::modules::Extent::per_sensor: return true;
                case spade::modules::Extent::per_row: i = arrays[i].owner; break;
            }
        }
        return false;
    };
    std::vector<std::string> out;
    sim.arenas().registry().for_each_array([&](const spade::RegisteredArray& array) {
        if (std::string_view(array.name).ends_with(spade::kSlotToWorldSuffix)) return;
        const auto core = std::ranges::find(spade::modules::kCoreArrays, std::string_view(array.name));
        if (core != std::ranges::end(spade::modules::kCoreArrays)) {
            const auto known = std::ranges::find(kCoreArrayIsPerBody, std::string_view(array.name),
                                                 &std::pair<std::string_view, bool>::first);
            if (known == std::ranges::end(kCoreArrayIsPerBody)) {
                unknown.push_back(array.name);
            } else if (known->second) {
                out.push_back(array.name);
            }
            return;
        }
        const auto declared = std::ranges::find(arrays, array.name, &spade::modules::CompiledArray::name);
        if (declared == arrays.end()) {
            unknown.push_back(array.name);
            return;
        }
        if (owned_by_a_body(static_cast<uint32_t>(declared - arrays.begin()))) out.push_back(array.name);
    });
    return out;
}

[[nodiscard]] CensusGaps census_gaps(const Simulation& sim) {
    CensusGaps gaps;
    const std::vector<std::string> per_body = per_body_arrays(sim, gaps.unknown);
    for (const std::string& name : per_body) {
        if (std::ranges::find(kPerBodyCensus, std::string_view(name), &ArrayRuling::array) ==
            std::ranges::end(kPerBodyCensus)) {
            gaps.unlisted.push_back(name);
        }
    }
    for (const ArrayRuling& row : kPerBodyCensus) {
        if (std::ranges::find(per_body, row.array) == per_body.end()) gaps.stale.emplace_back(row.array);
    }
    std::vector<std::string> scratch;
    for (const spade::modules::CompiledScratch& s : sim.schedule().scratch) {
        if (s.extent == spade::modules::Extent::per_body) scratch.push_back(s.name);
    }
    for (const std::string& name : scratch) {
        if (std::ranges::find(kPerBodyScratchCensus, std::string_view(name)) ==
            std::ranges::end(kPerBodyScratchCensus)) {
            gaps.unlisted_scratch.push_back(name);
        }
    }
    for (const std::string_view name : kPerBodyScratchCensus) {
        if (std::ranges::find(scratch, name) == scratch.end()) gaps.stale_scratch.emplace_back(name);
    }
    return gaps;
}

// A developer module holding per-body history -- the case the census exists
// for -- with a per_world array it must NOT flag.
struct ProbeRow {
    uint32_t value[4];
};
void probe_init(const spade::modules::RowInit&) noexcept {}
constexpr spade::modules::ArrayDecl kProbeArrays[] = {
    {.name = "probe_history",
     .elem_size = spade::modules::row_size<ProbeRow>(),
     .extent = spade::modules::Extent::per_body,
     .init = &probe_init},
    {.name = "probe_cache",
     .elem_size = spade::modules::row_size<ProbeRow>(),
     .extent = spade::modules::Extent::per_row,
     .owner = "probe_history",
     .depth = 2},
    {.name = "probe_world", .elem_size = spade::modules::row_size<ProbeRow>()}};
constexpr spade::modules::ScratchDecl kProbeScratch[] = {
    {.name = "probe_scratch",
     .elem_size = spade::modules::row_size<ProbeRow>(),
     .extent = spade::modules::Extent::per_body}};

}  // namespace

TEST(SetBodyState, CensusClassifiesEveryPerBodyArrayAndTheSetHonoursIt) {
    // The field census tiles BodyState: a new field fails here, a renamed pad
    // fails to compile.
    std::size_t at = 0;
    for (const FieldRuling& f : kBodyFields) {
        EXPECT_EQ(f.offset, at) << "BodyState::" << f.field << " is not where the census expects it";
        at = f.offset + f.size;
    }
    EXPECT_EQ(at, sizeof(BodyState)) << "a BodyState field the census does not classify";

    Rig rig = make_rig();
    const CensusGaps gaps = census_gaps(rig.sim);
    for (const std::string& name : gaps.unknown) {
        ADD_FAILURE() << "registered array '" << name << "' has no extent this census knows";
    }
    for (const std::string& name : gaps.unlisted) {
        ADD_FAILURE() << "per-body array '" << name << "' is not in set_body_state's census: classify it kept or "
                      << "cleared, in simulation.hpp's census beside mark_vulkan_dirty()'s and here";
    }
    for (const std::string& name : gaps.stale) {
        ADD_FAILURE() << "census row '" << name << "' names no per-body array of the standard set";
    }
    for (const std::string& name : gaps.unlisted_scratch) {
        ADD_FAILURE() << "per-body scratch '" << name << "' is not in set_body_state's census";
    }
    for (const std::string& name : gaps.stale_scratch) {
        ADD_FAILURE() << "scratch census row '" << name << "' names no per-body scratch of the standard set";
    }

    // The set honours the census, byte for byte.
    const BodyRef body = rig.vehicle.body;
    ASSERT_TRUE(rig.sim.apply_wrench(body, glm::vec3(0.5f, 0.0f, 0.25f), glm::vec3(0.0f, 0.01f, 0.0f)).has_value());
    const ArrayBytes before = array_bytes(rig.sim);
    const Pose p = far_pose();
    const Result<void> placed = set_to(rig.sim, body, p);
    ASSERT_TRUE(placed.has_value()) << placed.error().context;
    const ArrayBytes after = array_bytes(rig.sim);

    BodyState given{};
    given.pos = p.pos;
    given.orient = p.orient;
    given.vel = p.vel;
    given.omega_body = p.omega;
    for (const auto& [name, bytes] : before) {
        const std::vector<std::byte>& now = after.at(name);
        ASSERT_EQ(now.size(), bytes.size()) << name;
        if (name != "bodies") {
            const auto row = std::ranges::find(kPerBodyCensus, std::string_view(name), &ArrayRuling::array);
            if (row != std::ranges::end(kPerBodyCensus) && row->ruling == Ruling::cleared) {
                ADD_FAILURE() << "'" << name << "' is ruled cleared: give it a check here alongside its clear";
            }
            EXPECT_TRUE(now == bytes) << "array '" << name << "' changed, but the census keeps it";
            continue;
        }
        const std::size_t target = static_cast<std::size_t>(body.slot) * sizeof(BodyState);
        for (std::size_t k = 0; k < bytes.size(); k += sizeof(BodyState)) {
            if (k == target) continue;
            EXPECT_EQ(std::memcmp(now.data() + k, bytes.data() + k, sizeof(BodyState)), 0)
                << "body slot " << k / sizeof(BodyState) << " is not the one being set, and changed";
        }
        for (const FieldRuling& f : kBodyFields) {
            const std::byte* got = now.data() + target + f.offset;
            const std::byte* old = bytes.data() + target + f.offset;
            const std::byte* set = reinterpret_cast<const std::byte*>(&given) + f.offset;
            const std::array<std::byte, 16> zero{};
            switch (f.ruling) {
                case Ruling::written:
                    EXPECT_EQ(std::memcmp(got, set, f.size), 0) << "BodyState::" << f.field << " is written as given";
                    break;
                case Ruling::cleared:
                    EXPECT_EQ(std::memcmp(got, zero.data(), f.size), 0) << "BodyState::" << f.field << " is cleared";
                    EXPECT_NE(std::memcmp(old, zero.data(), f.size), 0)
                        << "BodyState::" << f.field << " should hold history before the set, or the check is vacuous";
                    break;
                case Ruling::kept:
                case Ruling::pad:
                    EXPECT_EQ(std::memcmp(got, old, f.size), 0) << "BodyState::" << f.field << " is kept";
                    break;
            }
        }
    }
}

// The census's teeth: a module that keeps per-body history (a per_body array,
// a per_row child of it, a per-body scratch) is named; its per_world array is
// not.
TEST(SetBodyState, CensusNamesAnUnlistedPerBodyArray) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "probe", .state = kProbeArrays, .scratch = kProbeScratch});
    const Result<WorldSetDesc> desc = floor_world_set();
    ASSERT_TRUE(desc.has_value());
    const Result<Simulation> sim = Simulation::create(*desc, kDt, kSubsteps, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const CensusGaps gaps = census_gaps(*sim);
    EXPECT_TRUE(gaps.unknown.empty());
    EXPECT_EQ(gaps.unlisted, (std::vector<std::string>{"probe_history", "probe_cache"}));
    EXPECT_TRUE(gaps.stale.empty());
    EXPECT_EQ(gaps.unlisted_scratch, (std::vector<std::string>{"probe_scratch"}));
    EXPECT_TRUE(gaps.stale_scratch.empty());
}

// ===========================================================================
// 12. THE |q| EDGE (the lead; 1 +/- 2e-6 confirmed by Runtime). Kat's binding
// check is DOUBLE, on the values as passed: accept iff | |q| - 1 | <= 1e-6.
// kat_host_spade converts each component to fp32 (a static_cast: round to
// nearest). Every quaternion Kat accepts is accepted here, as given; one at
// 1 +/- 2.5e-6 is refused.
// ===========================================================================
TEST(SetBodyState, AcceptsEveryQuaternionKatAcceptsAndRefusesPastTwoMicro) {
    Simulation sim = make_sim();
    const Result<BodyRef> body = sim.spawn(0, bare_body(glm::vec3(0.0f, 1.0f, 0.0f)));
    ASSERT_TRUE(body.has_value());
    ASSERT_TRUE(sim.flush_structural().has_value());
    const glm::vec3 pos(0.0f, 1.0f, 0.0f);
    const glm::vec3 still(0.0f);

    const auto kat_accepts = [](const std::array<double, 4>& q) {
        const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        return std::fabs(n - 1.0) <= 1.0e-6;
    };
    const std::array<std::array<double, 4>, 4> directions = {{
        {1.0, 0.0, 0.0, 0.0},
        {0.6, -0.3, 0.5, 0.2},
        {0.1, 0.9, -0.3, 0.2},
        {0.5, 0.5, 0.5, 0.5},
    }};
    for (const std::array<double, 4>& d : directions) {
        const double norm = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2] + d[3] * d[3]);
        const auto at = [&](double length) {
            std::array<double, 4> q{};
            for (std::size_t i = 0; i < 4; ++i) {
                q[i] = d[i] / norm * length;
            }
            return q;
        };
        const auto fp32 = [](const std::array<double, 4>& q) {
            return glm::quat(static_cast<float>(q[0]), static_cast<float>(q[1]), static_cast<float>(q[2]),
                             static_cast<float>(q[3]));
        };
        for (const double length : {1.0 + 0.99e-6, 1.0 - 0.99e-6, 1.0 + 0.5e-6, 1.0}) {
            const std::array<double, 4> q = at(length);
            ASSERT_TRUE(kat_accepts(q)) << length;
            const glm::quat f = fp32(q);
            const Result<void> r = sim.set_body_state(*body, pos, f, still, still);
            EXPECT_TRUE(r.has_value()) << "Kat accepts |q| = 1 + " << (length - 1.0) << ": " << text_of(r);
            EXPECT_TRUE(same_bytes(row_of(sim, *body).orient, f)) << "stored as given";
        }
        for (const double length : {1.0 + 2.5e-6, 1.0 - 2.5e-6}) {
            const std::array<double, 4> q = at(length);
            ASSERT_FALSE(kat_accepts(q)) << length;
            const Result<void> r = sim.set_body_state(*body, pos, fp32(q), still, still);
            EXPECT_EQ(code_of(r), code(spade::Code::invalid_argument)) << "|q| = 1 + " << (length - 1.0);
            EXPECT_TRUE(text_of(r).starts_with(kOrientRefusal)) << text_of(r);
        }
    }
}
