// test_fc_host_lockstep.cpp -- the lockstep host (tools/fc_host/lockstep.hpp; fc-core section 2). The clock, the whole-ticks rule, the samples
// with their times, the command path and its latency, and that the host adds nothing of its own to the world (same world, same commands, same bytes).




#include <gtest/gtest.h>

#include "fc_host/lockstep.hpp"
#include "fc_host_test_rig.hpp"

namespace {

using namespace fc_host_test;

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

TEST(FcHostLockstep, ALateTakeLosesNothingBecauseTheHostPollsEveryTick) {
    std::optional<Rig> rig = make_rig(1, 1000, 1000);  // an IMU every tick: the engine's 64-deep ring wraps in 64 ticks
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    ASSERT_OK(host->advance_to(100 * kDt));
    const spade::Result<std::vector<Sample>> got = host->take_sensors();
    ASSERT_OK(got);
    ASSERT_EQ(got->size(), 100u) << "the ring held 64; the host's own poll after each tick held the rest";
    for (std::size_t i = 0; i < got->size(); ++i) EXPECT_EQ((*got)[i].index, i + 1) << i;
    EXPECT_EQ(host->take_sensors()->size(), 0u);
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
