// The companion link's heartbeat (tools/fc_link/companion.hpp): when one is due in simulated time, and what it says.

#include <gtest/gtest.h>

#include <cstdint>

#include "fc_link/companion.hpp"

namespace {

namespace fcl = spade::fc_link;

TEST(FcLinkCompanion, TheFirstHeartbeatIsDueAtOnceAndThenOneEachSimulatedSecond) {
    fcl::HeartbeatSchedule schedule;
    EXPECT_TRUE(schedule.due(0));
    EXPECT_FALSE(schedule.due(0)) << "asked twice at the same time: once";
    EXPECT_FALSE(schedule.due(500000));
    EXPECT_FALSE(schedule.due(999999));
    EXPECT_TRUE(schedule.due(1000000));
    EXPECT_FALSE(schedule.due(1000000));
    EXPECT_FALSE(schedule.due(1999999));
    EXPECT_TRUE(schedule.due(2000000));
}

TEST(FcLinkCompanion, AScheduleThatStartsAtTheBootOffsetIsDueAtOnceAndKeepsTheSecondBoundaries) {
    fcl::HeartbeatSchedule schedule;
    EXPECT_TRUE(schedule.due(316000)) << "the first call, whatever the time";
    EXPECT_FALSE(schedule.due(999999));
    EXPECT_TRUE(schedule.due(1000000)) << "the next boundary, not 1,316,000";
    EXPECT_FALSE(schedule.due(1316000));
    EXPECT_TRUE(schedule.due(2004000));
}

TEST(FcLinkCompanion, AJumpAheadIsOneHeartbeatNotACatchUpAndTimeNeverGoesBack) {
    fcl::HeartbeatSchedule schedule;
    EXPECT_TRUE(schedule.due(0));
    EXPECT_TRUE(schedule.due(5300000)) << "five seconds passed: one heartbeat";
    EXPECT_FALSE(schedule.due(5900000));
    EXPECT_TRUE(schedule.due(6000000));
    EXPECT_FALSE(schedule.due(3000000)) << "a time before the last due one is never due";
    EXPECT_FALSE(schedule.due(6500000));
    EXPECT_TRUE(schedule.due(7000000));
}

TEST(FcLinkCompanion, ADifferentPeriodIsHonoured) {
    fcl::HeartbeatSchedule schedule(250000);
    EXPECT_TRUE(schedule.due(0));
    EXPECT_FALSE(schedule.due(249999));
    EXPECT_TRUE(schedule.due(250000));
    EXPECT_TRUE(schedule.due(500000));
}

TEST(FcLinkCompanion, TheHeartbeatIsAGroundStationOfNoAutopilotAndActive) {
    const spade::fc_link::mavlink::Heartbeat h = fcl::companion_heartbeat();
    EXPECT_EQ(h.type, 6) << "MAV_TYPE_GCS";
    EXPECT_EQ(h.autopilot, 8) << "MAV_AUTOPILOT_INVALID";
    EXPECT_EQ(h.base_mode, 0);
    EXPECT_EQ(h.custom_mode, 0u);
    EXPECT_EQ(h.system_status, 4) << "MAV_STATE_ACTIVE";
    EXPECT_EQ(h.mavlink_version, 3);
}

TEST(FcLinkCompanion, TheCompanionsSourceIdsAreOneNamedPairPx4AddressesItsAcksTo) {
    EXPECT_EQ(fcl::kCompanionSystemId, 1) << "PX4's own system id";
    EXPECT_EQ(fcl::kCompanionComponentId, 191) << "MAV_COMP_ID_ONBOARD_COMPUTER";
}

}  // namespace
