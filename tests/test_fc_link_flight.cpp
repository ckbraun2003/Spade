// The flight film's script (tools/fc_link/flight_mission.hpp) driven with made-up observations: no PX4, no link, no clock.
#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <optional>
#include <vector>

#include "fc_link/flight_mission.hpp"

namespace {

using namespace spade::fc_link;
namespace mav = spade::fc_link::mavlink;

constexpr uint16_t kArm = 400, kTakeoff = 22, kLand = 21;

FlightConfig config() {
    FlightConfig c;
    c.home_amsl_m = 488.0;  // takeoff 2.0 m above it: param7 490.0
    return c;
}

std::vector<mav::CommandLong> at(FlightMission& m, double t, bool armed, double alt, std::optional<AckObservation> ack = std::nullopt) {
    return m.step(FlightObservation{t, armed, alt, ack});
}

// Drives the mission from `t0` to `t1` in 0.1 s ticks; `state(t)` gives {armed, altitude}. Returns every command sent, with the tick time.
struct Sent {
    double t;
    mav::CommandLong command;
};
std::vector<Sent> drive(FlightMission& m, double t0, double t1, const std::function<std::pair<bool, double>(double)>& state) {
    std::vector<Sent> out;
    for (int i = 0; t0 + i * 0.1 <= t1 + 1e-9; ++i) {
        const double t = t0 + i * 0.1;
        const auto [armed, alt] = state(t);
        for (const auto& c : at(m, t, armed, alt)) out.push_back({t, c});
        if (m.finished()) break;
    }
    return out;
}

const FlightCheck* check(const FlightMission& m, const std::string& name) {
    for (const auto& c : m.journal())
        if (c.name == name) return &c;
    return nullptr;
}

}  // namespace

TEST(FcLinkFlight, NothingIsSentBeforeTheReadyTimeAndThenOneArm) {
    FlightMission m(config());
    EXPECT_TRUE(drive(m, 0.0, 19.9, [](double) { return std::pair{false, 0.0}; }).empty()) << "PX4 is still getting ready";
    EXPECT_EQ(m.phase(), FlightMission::Phase::boot);
    const auto sent = at(m, 20.0, false, 0.0);
    ASSERT_EQ(sent.size(), 1u);
    EXPECT_EQ(sent[0].command, kArm);
    EXPECT_EQ(sent[0].param[0], 1.0f);
    EXPECT_EQ(m.phase(), FlightMission::Phase::arming);
    EXPECT_TRUE(at(m, 20.1, false, 0.0).empty()) << "one arm, not one per tick";
}

TEST(FcLinkFlight, ATemporaryRejectionIsRetriedAfterTheRetryTimeAndArmedBringsTheTakeoff) {
    FlightMission m(config());
    ASSERT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
    EXPECT_TRUE(at(m, 20.1, false, 0.0, AckObservation{kArm, 1}).empty()) << "temporarily rejected: wait";
    EXPECT_TRUE(at(m, 20.9, false, 0.0).empty());
    const auto again = at(m, 21.0, false, 0.0);
    ASSERT_EQ(again.size(), 1u) << "after arm_retry_s the arm is sent again";
    EXPECT_EQ(again[0].command, kArm);
    const auto take = at(m, 21.5, true, 0.0, AckObservation{kArm, 0});
    ASSERT_EQ(take.size(), 1u) << "armed: the takeoff goes out on the same observation";
    EXPECT_EQ(take[0].command, kTakeoff);
    EXPECT_FLOAT_EQ(take[0].param[6], 490.0f) << "2.0 m above 488 m AMSL";
    EXPECT_TRUE(std::isnan(take[0].param[3]) && std::isnan(take[0].param[4]) && std::isnan(take[0].param[5]));
    EXPECT_EQ(m.phase(), FlightMission::Phase::takeoff);
    const FlightCheck* armed = check(m, "armed_within_s");
    ASSERT_NE(armed, nullptr);
    EXPECT_TRUE(armed->pass);
    EXPECT_NEAR(armed->measured, 1.5, 1e-9) << "from the first arm command (20.0) to the armed reply (21.5)";
    EXPECT_EQ(armed->limit, 10.0);
}

TEST(FcLinkFlight, ADeniedArmEndsTheMissionAtOnceAndAFailedMissionSendsNothingMore) {
    FlightMission m(config());
    ASSERT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
    EXPECT_TRUE(at(m, 20.2, false, 0.0, AckObservation{kArm, 2}).empty());
    EXPECT_TRUE(m.finished());
    EXPECT_FALSE(m.passed());
    const FlightCheck* ack = check(m, "arm_ack");
    ASSERT_NE(ack, nullptr);
    EXPECT_FALSE(ack->pass);
    EXPECT_EQ(ack->measured, 2.0) << "MAV_RESULT denied";
    EXPECT_TRUE(at(m, 21.5, true, 0.0).empty()) << "a failed take sends nothing more";
    EXPECT_EQ(m.phase(), FlightMission::Phase::failed);
}

TEST(FcLinkFlight, NotArmedWithinTheTimeoutFailsTheMissionAndNoTakeoffIsSent) {
    FlightMission m(config());
    const auto sent = drive(m, 20.0, 31.0, [](double) { return std::pair{false, 0.0}; });
    for (const auto& s : sent) EXPECT_EQ(s.command.command, kArm) << "only arm commands, retried";
    EXPECT_GE(sent.size(), 2u);
    EXPECT_TRUE(m.finished());
    EXPECT_FALSE(m.passed());
    const FlightCheck* armed = check(m, "armed_within_s");
    ASSERT_NE(armed, nullptr);
    EXPECT_FALSE(armed->pass);
    EXPECT_NEAR(armed->time_s, 30.0, 0.05) << "judged when the 10 s ran out";
}

TEST(FcLinkFlight, ReachingTheTakeoffBandStartsTheHoverAndMissingItInTimeFails) {
    {
        FlightMission m(config());
        ASSERT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
        ASSERT_EQ(at(m, 20.5, true, 0.0).size(), 1u);  // takeoff
        EXPECT_TRUE(at(m, 24.0, true, 1.5).empty());
        EXPECT_EQ(m.phase(), FlightMission::Phase::takeoff) << "1.5 m is outside 2.0 +- 0.2";
        EXPECT_TRUE(at(m, 25.0, true, 1.85).empty());
        EXPECT_EQ(m.phase(), FlightMission::Phase::settle) << "inside 2.0 +- 0.2 but not yet inside the hover band of 0.1: the climb is still settling";
        const FlightCheck* h = check(m, "takeoff_height");
        ASSERT_NE(h, nullptr);
        EXPECT_TRUE(h->pass);
        EXPECT_NEAR(h->measured, 0.15, 1e-9) << "distance from 2.0 m";
        EXPECT_EQ(h->limit, 0.2);
        EXPECT_NEAR(h->time_s, 25.0, 1e-9);
        EXPECT_TRUE(at(m, 26.0, true, 1.95).empty());
        EXPECT_EQ(m.phase(), FlightMission::Phase::hover);
        const FlightCheck* s = check(m, "settled_within_m");
        ASSERT_NE(s, nullptr);
        EXPECT_TRUE(s->pass);
        EXPECT_NEAR(s->measured, 0.05, 1e-9) << "distance from the takeoff height when the hover began";
        EXPECT_EQ(s->limit, 0.1);
        EXPECT_NEAR(s->time_s, 26.0, 1e-9);
    }
    {
        FlightMission m(config());
        ASSERT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
        ASSERT_EQ(at(m, 20.5, true, 0.0).size(), 1u);
        const auto sent = drive(m, 21.0, 41.0, [](double) { return std::pair{true, 1.0}; });
        ASSERT_EQ(sent.size(), 1u) << "a failed check in the air sends LAND";
        EXPECT_EQ(sent[0].command.command, kLand);
        EXPECT_NEAR(sent[0].t, 40.5, 0.05);
        EXPECT_EQ(m.phase(), FlightMission::Phase::aborting);
        EXPECT_FALSE(m.finished()) << "not over until the vehicle is down and disarmed";
        EXPECT_FALSE(m.passed());
        const FlightCheck* h = check(m, "takeoff_height");
        ASSERT_NE(h, nullptr);
        EXPECT_FALSE(h->pass);
        EXPECT_NEAR(h->measured, 1.0, 1e-9) << "how far from 2.0 m it was when the 20 s ran out";
        EXPECT_NEAR(h->time_s, 40.5, 0.05);
    }
}

// The case of dry run 2 (PX4 climbing slowly up to its setpoint): the hover is judged against the TAKEOFF HEIGHT, not against the altitude where it began.
TEST(FcLinkFlight, AClimbStillCreepingUpToTheSetpointIsAHoverJudgedAgainstTheSetpoint) {
    FlightMission m(config());
    ASSERT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
    ASSERT_EQ(at(m, 20.5, true, 0.0).size(), 1u);  // the takeoff
    EXPECT_TRUE(at(m, 26.8, true, 1.80).empty());
    EXPECT_EQ(m.phase(), FlightMission::Phase::settle);
    EXPECT_TRUE(at(m, 28.0, true, 1.93).empty());
    EXPECT_EQ(m.phase(), FlightMission::Phase::hover) << "within 0.1 m of 2.0 m";
    // 10 s of creeping from 1.93 to 2.00: 0.07 m, inside the band against the setpoint (against where it began it would be 0.07 too, but a climb that began at
    // 1.90 and ended at 2.0 is 0.10: the failure of dry run 2 was this shape).
    const auto sent = drive(m, 28.1, 38.5, [](double t) { return std::pair{true, 1.93 + 0.007 * (t - 28.0)}; });
    ASSERT_EQ(sent.size(), 1u);
    EXPECT_EQ(sent[0].command.command, kLand);
    EXPECT_NEAR(sent[0].t, 38.0, 0.05) << "10 s after the hover began";
    const FlightCheck* hover = check(m, "hover_within_m");
    ASSERT_NE(hover, nullptr);
    EXPECT_TRUE(hover->pass);
    EXPECT_LE(hover->measured, 0.1) << "the largest distance from 2.0 m during the hover";
}

TEST(FcLinkFlight, NotSettledWithinTheTakeoffTimeoutFailsNamingTheDistanceFromTheSetpoint) {
    FlightMission m(config());
    ASSERT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
    ASSERT_EQ(at(m, 20.5, true, 0.0).size(), 1u);
    const auto sent = drive(m, 21.0, 41.0, [](double) { return std::pair{true, 1.85}; });
    ASSERT_EQ(sent.size(), 1u) << "inside the takeoff band, never inside the hover band: it fails and aborts to land";
    EXPECT_EQ(sent[0].command.command, kLand);
    EXPECT_FALSE(m.passed());
    const FlightCheck* s = check(m, "settled_within_m");
    ASSERT_NE(s, nullptr);
    EXPECT_FALSE(s->pass);
    EXPECT_NEAR(s->measured, 0.15, 1e-9);
    EXPECT_EQ(s->limit, 0.1);
    EXPECT_NEAR(s->time_s, 40.5, 0.05) << "judged when takeoff_timeout_s ran out, counted from the takeoff command";
}

TEST(FcLinkFlight, TheHoverIsHeldForTheHoverTimeAndThenTheLandCommandGoesOut) {
    FlightMission m(config());
    ASSERT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
    ASSERT_EQ(at(m, 20.5, true, 0.0).size(), 1u);
    ASSERT_TRUE(at(m, 25.0, true, 2.0).empty());  // hover begins at 25.0, altitude 2.0
    const auto sent = drive(m, 25.1, 36.0, [](double t) { return std::pair{true, 2.0 + 0.05 * std::sin(t)}; });  // wanders 0.05 m: inside 0.1
    ASSERT_EQ(sent.size(), 1u);
    EXPECT_EQ(sent[0].command.command, kLand);
    EXPECT_NEAR(sent[0].t, 35.0, 0.05) << "10 s after the hover began";
    EXPECT_EQ(m.phase(), FlightMission::Phase::land);
    const FlightCheck* s = check(m, "hover_within_m");
    ASSERT_NE(s, nullptr);
    EXPECT_TRUE(s->pass);
    EXPECT_LE(s->measured, 0.1);
    EXPECT_EQ(s->limit, 0.1);
}

TEST(FcLinkFlight, DriftingOutOfTheHoverBandFailsWhereItHappens) {
    FlightMission m(config());
    ASSERT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
    ASSERT_EQ(at(m, 20.5, true, 0.0).size(), 1u);
    ASSERT_TRUE(at(m, 25.0, true, 2.0).empty());
    EXPECT_TRUE(at(m, 28.0, true, 2.09).empty());
    EXPECT_FALSE(m.finished());
    const auto abort_cmds = at(m, 29.0, true, 2.13);
    ASSERT_EQ(abort_cmds.size(), 1u) << "0.13 m from the setpoint: the hover fails and LAND goes out at once";
    EXPECT_EQ(abort_cmds[0].command, kLand);
    EXPECT_EQ(m.phase(), FlightMission::Phase::aborting);
    EXPECT_FALSE(m.passed());
    const FlightCheck* s = check(m, "hover_within_m");
    ASSERT_NE(s, nullptr);
    EXPECT_FALSE(s->pass);
    EXPECT_NEAR(s->measured, 0.13, 1e-9);
    EXPECT_NEAR(s->time_s, 29.0, 1e-9);
}

TEST(FcLinkFlight, LandedMeansLowAndDisarmedWithinTheLandTimeAndPassesTheMission) {
    const auto to_land = [](FlightMission& m) {
        ASSERT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
        ASSERT_EQ(at(m, 20.5, true, 0.0).size(), 1u);
        ASSERT_TRUE(at(m, 25.0, true, 2.0).empty());
        ASSERT_EQ(drive(m, 25.1, 36.0, [](double) { return std::pair{true, 2.0}; }).size(), 1u);  // the land command at 35.0
    };
    {
        FlightMission m(config());
        to_land(m);
        EXPECT_TRUE(at(m, 40.0, true, 1.0).empty());
        EXPECT_TRUE(at(m, 45.0, true, 0.03).empty()) << "low but still armed: not landed";
        EXPECT_FALSE(m.finished());
        EXPECT_TRUE(at(m, 47.0, false, 0.03).empty());
        EXPECT_TRUE(m.finished());
        EXPECT_TRUE(m.passed());
        const FlightCheck* l = check(m, "landed_within_s");
        ASSERT_NE(l, nullptr);
        EXPECT_TRUE(l->pass);
        EXPECT_NEAR(l->measured, 12.0, 0.05) << "from the land command (35.0) to landed and disarmed (47.0)";
        EXPECT_EQ(l->limit, 30.0);
    }
    {
        FlightMission m(config());
        to_land(m);
        EXPECT_TRUE(drive(m, 36.0, 66.0, [](double) { return std::pair{true, 0.03}; }).empty());
        EXPECT_TRUE(m.finished());
        EXPECT_FALSE(m.passed()) << "still armed after the land time";
        const FlightCheck* l = check(m, "landed_within_s");
        ASSERT_NE(l, nullptr);
        EXPECT_FALSE(l->pass);
    }
}

// The abort: a failed check once armed sends LAND and the mission waits (up to land_timeout_s) for landed AND disarmed.
namespace {
// A hover that fails at 29.0 s (the abort begins there), the mission in the aborting phase.
FlightMission aborting_mission() {
    FlightMission m(config());
    EXPECT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
    EXPECT_EQ(at(m, 20.5, true, 0.0).size(), 1u);
    EXPECT_TRUE(at(m, 25.0, true, 2.0).empty());
    EXPECT_EQ(at(m, 29.0, true, 2.13).size(), 1u);
    return m;
}
}  // namespace

TEST(FcLinkFlight, AnAbortThatLandsAndDisarmsEndsCleanlyAndIsRecordedButTheMissionNeverPasses) {
    FlightMission m = aborting_mission();
    ASSERT_EQ(m.phase(), FlightMission::Phase::aborting);
    EXPECT_TRUE(at(m, 33.0, true, 1.0).empty()) << "no second LAND";
    EXPECT_TRUE(at(m, 37.0, true, 0.03).empty()) << "down but still armed: not over";
    EXPECT_FALSE(m.finished());
    EXPECT_TRUE(at(m, 40.0, false, 0.03).empty());
    EXPECT_TRUE(m.finished());
    EXPECT_FALSE(m.passed());
    EXPECT_EQ(m.abort_outcome(), FlightMission::AbortOutcome::landed_and_disarmed);
    const FlightCheck* a = check(m, "abort_landed_disarmed_s");
    ASSERT_NE(a, nullptr);
    EXPECT_TRUE(a->pass);
    EXPECT_NEAR(a->measured, 11.0, 1e-9) << "from the abort (29.0) to landed and disarmed (40.0)";
    EXPECT_EQ(a->limit, 30.0);
    EXPECT_TRUE(m.landed());
    EXPECT_FALSE(m.last_armed());
    EXPECT_NEAR(m.last_altitude_m(), 0.03, 1e-9);
}

TEST(FcLinkFlight, AnAbortWhoseBoundExpiresIsRecordedWithTheVehiclesStateAndStillEnds) {
    FlightMission m = aborting_mission();
    EXPECT_TRUE(drive(m, 29.1, 60.0, [](double) { return std::pair{true, 1.0}; }).empty());
    EXPECT_TRUE(m.finished());
    EXPECT_FALSE(m.passed());
    EXPECT_EQ(m.abort_outcome(), FlightMission::AbortOutcome::expired);
    const FlightCheck* a = check(m, "abort_landed_disarmed_s");
    ASSERT_NE(a, nullptr);
    EXPECT_FALSE(a->pass);
    EXPECT_NEAR(a->measured, 30.0, 0.05);
    EXPECT_NEAR(a->time_s, 59.0, 0.05);
    EXPECT_TRUE(m.last_armed());
    EXPECT_FALSE(m.landed());
    EXPECT_NEAR(m.last_altitude_m(), 1.0, 1e-9);
}

TEST(FcLinkFlight, AFailureBeforeLiftOffJustEndsAndSendsNoLand) {
    FlightMission m(config());
    ASSERT_EQ(at(m, 20.0, false, 0.0).size(), 1u);
    EXPECT_TRUE(at(m, 20.2, false, 0.0, AckObservation{kArm, 2}).empty());
    EXPECT_TRUE(m.finished());
    EXPECT_EQ(m.abort_outcome(), FlightMission::AbortOutcome::none) << "nothing was in the air";
    EXPECT_EQ(m.last_altitude_m(), 0.0);
    EXPECT_FALSE(m.last_armed());
}
