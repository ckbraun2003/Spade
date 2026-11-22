// test_fc_host_lockstep.cpp -- the lockstep host (tools/fc_host/lockstep.hpp; fc-core section 2). The clock, the whole-ticks rule, the samples
// with their times, the command path and its latency, and that the host adds nothing of its own to the world (same world, same commands, same bytes).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "fc_host/lockstep.hpp"
#include "sensors/baro.hpp"
#include "sensors/imu.hpp"
#include "sensors/mag.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"

namespace {

using spade::BodyRef;
using spade::Capacities;
using spade::Environment;
using spade::RowRef;
using spade::Simulation;
using spade::TurbulenceLevel;
using spade::VehicleRef;
using spade::VehicleSpawn;
using spade::WorldBuilder;
using spade::WorldInstanceDesc;
using spade::WorldSetDesc;
using spade::fc_host::HostConfig;
using spade::fc_host::LockstepHost;
using spade::fc_host::Sample;
using spade::fc_host::SensorBinding;
using spade::fc_host::SensorKind;
using spade::vehicles::QuadrotorParams;
using spade::vehicles::RotorParams;

#define ASSERT_OK(expr)                                                                         \
    do {                                                                                        \
        const auto& r_ = (expr);                                                                \
        ASSERT_TRUE(r_.has_value()) << #expr << " failed: " << (r_ ? "" : r_.error().context); \
    } while (false)

constexpr uint64_t kDt = 500'000;  // 500 us, one substep: the phase 1 default (4 ms is 8 ticks, 2.5 ms is 5)

[[nodiscard]] spade::modules::ModuleSet modules_with_baro_mag() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::baro_module());
    set.push_back(spade::modules::mag_module());
    return set;
}

// No gravity: a commanded rotor shows as acceleration and nothing else.
[[nodiscard]] spade::Result<WorldSetDesc> still_world() {
    Environment env;
    env.gravity = glm::vec3(0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    Capacities caps;
    caps.bodies = 4;
    caps.force_elements = 8;
    caps.sensors = 4;
    caps.contacts = 1;
    const spade::Result<spade::WorldDesc> world = WorldBuilder().name("still").environment(env).capacities(caps).build();
    if (!world) return std::unexpected(world.error());
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = 0x10C57E9ULL;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.1f;
    instance.grid.cell_size = 0.5f;
    return WorldSetDesc{{instance}};
}

struct Rig {
    Simulation sim;
    VehicleRef vehicle;
    RowRef baro;
    RowRef mag;
};

// A quadrotor with an IMU at `imu_divider`, a barometer at `baro_divider`, a magnetometer at `mag_divider`; dt 500 us, one substep.
[[nodiscard]] std::optional<Rig> make_rig(uint32_t imu_divider, uint32_t baro_divider = 40, uint32_t mag_divider = 20, uint64_t dt_ns = kDt,
                                          uint32_t substeps = 1) {
    const spade::Result<WorldSetDesc> set = still_world();
    if (!set) return std::nullopt;
    spade::Result<Simulation> sim = Simulation::create(*set, dt_ns, substeps, {}, modules_with_baro_mag());
    if (!sim) return std::nullopt;

    QuadrotorParams params;
    params.name = "host_quad";
    params.param_schema_id = 7;
    params.visual_ref = "meshes/host_quad";
    params.mass = 1.0f;
    params.inertia_diag = glm::vec3(0.012f, 0.021f, 0.016f);
    params.arm_length = 0.18f;
    params.rotor_height = 0.02f;
    params.proxy_radius = 0.18f;
    for (std::size_t i = 0; i < spade::vehicles::kQuadrotorRotorCount; ++i) {
        RotorParams& rotor = params.rotors[i];
        rotor.tau = 0.0f;
        rotor.radius = 0.12f;
        rotor.thrust_coeff = 1.0e-5f;
        rotor.torque_coeff = 1.6e-7f;
    }
    params.drag.mode = spade::physics::drag_mode::quadratic;
    params.drag.area = 0.02f;
    params.drag.coeffs = glm::vec3(0.0f);
    params.imu.rate_divider = imu_divider;
    const spade::Result<spade::vehicles::ModelType> model = spade::vehicles::make_quadrotor(params);
    if (!model) return std::nullopt;
    const spade::Result<spade::ModelTypeId> id = sim->register_model(*model);
    if (!id) return std::nullopt;
    VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 10.0f, 0.0f);
    where.rotor_omega = 0.0f;
    const spade::Result<VehicleRef> vehicle = sim->spawn(0, *id, where);
    if (!vehicle) return std::nullopt;

    spade::sensors::BaroSensorSpawn baro;
    baro.rate_divider = baro_divider;
    const spade::Result<RowRef> baro_row = sim->attach_row(vehicle->body, "baro_sensors", baro);
    spade::sensors::MagSensorSpawn mag;
    mag.rate_divider = mag_divider;
    mag.field_world_ut = glm::vec3(20.0f, -40.0f, 5.0f);
    const spade::Result<RowRef> mag_row = sim->attach_row(vehicle->body, "mag_sensors", mag);
    if (!baro_row || !mag_row) return std::nullopt;
    if (!sim->flush_structural()) return std::nullopt;
    return Rig{std::move(*sim), *vehicle, *baro_row, *mag_row};
}

[[nodiscard]] std::vector<SensorBinding> all_bindings(const Rig& rig) {
    return {SensorBinding::of(rig.vehicle.imu_sensors[0]), SensorBinding::baro(rig.baro), SensorBinding::mag(rig.mag)};
}

[[nodiscard]] std::size_t count_of(const std::vector<Sample>& samples, SensorKind kind) {
    std::size_t n = 0;
    for (const Sample& s : samples) n += s.kind == kind ? 1u : 0u;
    return n;
}

// ===========================================================================
// THE CLOCK
// ===========================================================================

TEST(FcHostLockstep, TheClockIsTheTickTimesDtAndNothingElse) {
    std::optional<Rig> rig = make_rig(8);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    EXPECT_EQ(host->tick(), 0u);
    EXPECT_EQ(host->dt_ns(), kDt);
    EXPECT_EQ(host->sim_time_ns(), 0u);
    ASSERT_OK(host->advance_to(8 * kDt));
    EXPECT_EQ(host->tick(), 8u);
    EXPECT_EQ(host->sim_time_ns(), 8 * kDt);
    EXPECT_EQ(rig->sim.tick().value, 8u) << "the host steps the Simulation and counts nothing of its own";
}

TEST(FcHostLockstep, AdvanceRefusesThePastAndAnOffBoundaryTime) {
    std::optional<Rig> rig = make_rig(8);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    ASSERT_OK(host->advance_to(4 * kDt));
    const spade::Result<void> past = host->advance_to(3 * kDt);
    ASSERT_FALSE(past.has_value());
    EXPECT_EQ(past.error().code, spade::Code::invalid_argument);
    const spade::Result<void> off = host->advance_to(4 * kDt + 1);
    ASSERT_FALSE(off.has_value());
    EXPECT_EQ(off.error().code, spade::Code::invalid_argument);
    EXPECT_EQ(host->tick(), 4u) << "a refusal moves nothing";
    ASSERT_OK(host->advance_to(4 * kDt));  // now is allowed: zero ticks
    EXPECT_EQ(host->tick(), 4u);
}

// ===========================================================================
// THE WHOLE-TICKS RULE
// ===========================================================================

TEST(FcHostLockstep, ASensorPeriodThatIsNotWholeTicksIsRefusedAtOpen) {
    // dt 1 ms, 2 substeps (h = 0.5 ms): an IMU every 3 substeps is 1.5 ticks.
    std::optional<Rig> rig = make_rig(3, 40, 20, 1'000'000, 2);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    const spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_FALSE(host.has_value()) << "1.5 ticks cannot be stamped";
    EXPECT_EQ(host.error().code, spade::Code::invalid_argument);
    EXPECT_NE(host.error().context.find("rate_divider 3"), std::string::npos) << host.error().context;
    EXPECT_NE(host.error().context.find("whole number of ticks"), std::string::npos) << host.error().context;
}

TEST(FcHostLockstep, APeriodOfWholeTicksIsAcceptedWithMoreThanOneSubstepPerTick) {
    std::optional<Rig> rig = make_rig(4, 40, 20, 1'000'000, 2);  // 4 substeps = 2 ticks
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    ASSERT_OK(LockstepHost::open(rig->sim, rig->vehicle, sensors));
}

// ===========================================================================
// SAMPLES AND THEIR TIMES
// ===========================================================================

TEST(FcHostLockstep, AnImuAtDivider8GivesOneSampleEveryFourMillisecondsStampedByTheTickItEndsOn) {
    std::optional<Rig> rig = make_rig(8, 1000, 1000);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    for (uint64_t k = 1; k <= 3; ++k) {
        ASSERT_OK(host->advance_to(k * 8 * kDt));
        const spade::Result<std::vector<Sample>> got = host->take_sensors();
        ASSERT_OK(got);
        ASSERT_EQ(got->size(), 1u) << "cycle " << k;
        const Sample& s = (*got)[0];
        EXPECT_EQ(s.kind, SensorKind::imu);
        EXPECT_EQ(s.binding, 0u);
        EXPECT_EQ(s.index, k);
        EXPECT_EQ(s.tick, k * 8 - 1) << "the sample is the state at the END of the tick it is stamped with";
        EXPECT_EQ(s.time_ns, (s.tick + 1) * kDt);
        EXPECT_EQ(s.size, sizeof(spade::sensors::ImuSample));
    }
}

TEST(FcHostLockstep, BaroAndMagComeOnTheirOwnPeriodsInTickOrder) {
    std::optional<Rig> rig = make_rig(8, 40, 20);  // baro 20 ms, mag 10 ms, imu 4 ms
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    ASSERT_OK(host->advance_to(40 * kDt));  // 20 ms
    const spade::Result<std::vector<Sample>> got = host->take_sensors();
    ASSERT_OK(got);
    EXPECT_EQ(count_of(*got, SensorKind::imu), 5u);
    EXPECT_EQ(count_of(*got, SensorKind::mag), 2u);
    EXPECT_EQ(count_of(*got, SensorKind::baro), 1u);
    for (std::size_t i = 1; i < got->size(); ++i) {
        const Sample& a = (*got)[i - 1];
        const Sample& b = (*got)[i];
        EXPECT_TRUE(a.tick < b.tick || (a.tick == b.tick && a.binding < b.binding)) << "ordered by (tick, binding) at " << i;
    }
    EXPECT_EQ(host->take_sensors()->size(), 0u) << "a second call returns only what is new";
}

TEST(FcHostLockstep, ALostPollIsRefusedNeverHidden) {
    std::optional<Rig> rig = make_rig(1, 1000, 1000);  // an IMU every tick: the 64-deep ring wraps in 64 ticks
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    ASSERT_OK(host->advance_to(100 * kDt));
    const spade::Result<std::vector<Sample>> got = host->take_sensors();
    ASSERT_FALSE(got.has_value()) << "36 samples fell out of the ring";
    EXPECT_NE(got.error().context.find("dropped"), std::string::npos) << got.error().context;
}

// ===========================================================================
// COMMANDS
// ===========================================================================

TEST(FcHostLockstep, ACommandIsRefusedForAWrongCountANonFiniteOrANegativeValue) {
    std::optional<Rig> rig = make_rig(8);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    const float three[3] = {1.0f, 1.0f, 1.0f};
    EXPECT_FALSE(host->command(three).has_value());
    const float nan4[4] = {1.0f, std::numeric_limits<float>::quiet_NaN(), 1.0f, 1.0f};
    EXPECT_FALSE(host->command(nan4).has_value());
    const float inf4[4] = {1.0f, std::numeric_limits<float>::infinity(), 1.0f, 1.0f};
    EXPECT_FALSE(host->command(inf4).has_value());
    const float neg4[4] = {1.0f, -1.0f, 1.0f, 1.0f};
    EXPECT_FALSE(host->command(neg4).has_value());
    const float ok4[4] = {100.0f, 100.0f, 100.0f, 100.0f};
    ASSERT_OK(host->command(ok4));
}

TEST(FcHostLockstep, ACommandReachesTheRotorsOnTheNextTickAndLatencyDelaysItByWholeTicks) {
    const float full[4] = {900.0f, 900.0f, 900.0f, 900.0f};
    for (const uint64_t latency : {uint64_t{0}, uint64_t{3}}) {
        std::optional<Rig> rig = make_rig(8);
        ASSERT_TRUE(rig.has_value());
        const std::vector<SensorBinding> sensors = all_bindings(*rig);
        HostConfig config;
        config.command_latency_ticks = latency;
        spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors, config);
        ASSERT_OK(host);
        ASSERT_OK(host->command(full));
        ASSERT_OK(host->advance_to(latency * kDt));  // the ticks the command has not reached yet
        const spade::Result<spade::fc_host::Truth> before = host->truth();
        ASSERT_OK(before);
        EXPECT_EQ(before->vel.y, 0.0f) << "latency " << latency << ": nothing has pushed yet";
        ASSERT_OK(host->advance_to((latency + 4) * kDt));
        const spade::Result<spade::fc_host::Truth> after = host->truth();
        ASSERT_OK(after);
        EXPECT_GT(after->vel.y, 0.0f) << "latency " << latency << ": four ticks after it is released, the rotors have pushed";
    }
}

// ===========================================================================
// THE HOST ADDS NOTHING
// ===========================================================================

TEST(FcHostLockstep, TwoHostsOnTheSameWorldAndCommandsGiveTheSameSamplesByte) {
    const float cmd[4] = {500.0f, 520.0f, 500.0f, 480.0f};
    std::vector<std::vector<Sample>> runs;
    for (int run = 0; run < 2; ++run) {
        std::optional<Rig> rig = make_rig(8, 40, 20);
        ASSERT_TRUE(rig.has_value());
        const std::vector<SensorBinding> sensors = all_bindings(*rig);
        spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
        ASSERT_OK(host);
        std::vector<Sample> all;
        for (int cycle = 1; cycle <= 10; ++cycle) {
            ASSERT_OK(host->command(cmd));
            ASSERT_OK(host->advance_to(static_cast<uint64_t>(cycle) * 8 * kDt));
            const spade::Result<std::vector<Sample>> got = host->take_sensors();
            ASSERT_OK(got);
            all.insert(all.end(), got->begin(), got->end());
        }
        runs.push_back(std::move(all));
    }
    ASSERT_EQ(runs[0].size(), runs[1].size());
    ASSERT_GT(runs[0].size(), 10u);
    for (std::size_t i = 0; i < runs[0].size(); ++i) {
        EXPECT_EQ(std::memcmp(&runs[0][i], &runs[1][i], sizeof(Sample)), 0) << "sample " << i;
    }
}

TEST(FcHostLockstep, TruthIsTheBodyAtTheCurrentTickAndModelIsTheVehiclesModel) {
    std::optional<Rig> rig = make_rig(8);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    const spade::Result<spade::fc_host::Truth> t = host->truth();
    ASSERT_OK(t);
    EXPECT_EQ(t->pos, glm::vec3(0.0f, 10.0f, 0.0f));
    EXPECT_EQ(host->model().name, "host_quad");
    EXPECT_EQ(host->model().rotors.size(), 4u);
}

}  // namespace
