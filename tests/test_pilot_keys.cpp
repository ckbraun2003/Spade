// N17's pilot keys (sandbox/pilot_keys.hpp; the lead's Q1): what each key does
// to the pilot's analysis::Controls, display-free. Space arms and disarms
// (disarming drops the throttle); PageUp/PageDown move the throttle at 50% a
// second within [0, 1]; the arrows tilt the setpoint toward 30 degrees at 60
// degrees a second and back to level when released (Up is nose down, Right is
// the right side down, as the drone box's keys); Z and X turn the heading at
// 90 degrees a second (Z left, toward +yaw), wrapping; R levels pitch and roll
// and keeps the heading.

#include <gtest/gtest.h>

#include <string>

#include "../sandbox/pilot_keys.hpp"
#include "analysis/case.hpp"

namespace {

namespace fl = spade::sandbox::flight;
using spade::analysis::Controls;

// `keys` held for `seconds`, in 4 ms frames.
void hold(const fl::PilotKeys& keys, float seconds, Controls& c) {
    const int frames = static_cast<int>(seconds / 0.004f + 0.5f);
    for (int i = 0; i < frames; ++i) fl::apply_pilot_keys(keys, 0.004f, c);
}

}  // namespace

TEST(PilotKeys, SpaceTogglesArmingAndDisarmingDropsTheThrottle) {
    Controls c;
    fl::PilotKeys space;
    space.arm_pressed = true;
    fl::apply_pilot_keys(space, 0.004f, c);
    EXPECT_TRUE(c.armed);
    c.throttle = 0.6f;
    fl::apply_pilot_keys(space, 0.004f, c);
    EXPECT_FALSE(c.armed);
    EXPECT_EQ(c.throttle, 0.0f) << "a disarmed drone re-arms at idle, never at its old throttle";
}

TEST(PilotKeys, PageUpAndPageDownMoveTheThrottleAtHalfASecondWithinZeroToOne) {
    Controls c;
    c.armed = true;
    fl::PilotKeys up;
    up.throttle = 1.0f;
    hold(up, 1.0f, c);
    EXPECT_NEAR(c.throttle, 0.5f, 0.01f);
    hold(up, 2.0f, c);
    EXPECT_EQ(c.throttle, 1.0f);
    fl::PilotKeys down;
    down.throttle = -1.0f;
    hold(down, 3.0f, c);
    EXPECT_EQ(c.throttle, 0.0f);
}

TEST(PilotKeys, AnArrowTiltsTowardThirtyDegreesAndLevelsWhenReleased) {
    Controls c;
    fl::PilotKeys nose_down;
    nose_down.pitch = -1.0f;  // Up
    hold(nose_down, 0.25f, c);
    EXPECT_NEAR(c.pitch_deg, -15.0f, 0.5f);
    hold(nose_down, 1.0f, c);
    EXPECT_EQ(c.pitch_deg, -30.0f);
    hold(fl::PilotKeys{}, 0.25f, c);
    EXPECT_NEAR(c.pitch_deg, -15.0f, 0.5f);
    hold(fl::PilotKeys{}, 0.5f, c);
    EXPECT_EQ(c.pitch_deg, 0.0f);
}

TEST(PilotKeys, RightRollsRightAndZTurnsLeftWrappingTheHeading) {
    Controls c;
    fl::PilotKeys right;
    right.roll = 1.0f;
    hold(right, 1.0f, c);
    EXPECT_EQ(c.roll_deg, 30.0f);
    fl::PilotKeys left_turn;
    left_turn.yaw = 1.0f;  // Z
    hold(left_turn, 1.0f, c);
    EXPECT_NEAR(c.yaw_deg, 90.0f, 1.0f);
    hold(left_turn, 1.5f, c);
    EXPECT_NEAR(c.yaw_deg, -135.0f, 1.0f) << "225 degrees wraps to -135";
}

TEST(PilotKeys, RLevelsPitchAndRollAndKeepsTheHeading) {
    Controls c;
    c.pitch_deg = -20.0f;
    c.roll_deg = 12.0f;
    c.yaw_deg = 40.0f;
    fl::PilotKeys r;
    r.level_pressed = true;
    fl::apply_pilot_keys(r, 0.004f, c);
    EXPECT_EQ(c.pitch_deg, 0.0f);
    EXPECT_EQ(c.roll_deg, 0.0f);
    EXPECT_EQ(c.yaw_deg, 40.0f);
}

// The Pilot section lists the keys, so nobody has to remember them (the lead).
TEST(PilotKeys, TheirHelpLineNamesEveryKey) {
    const std::string help = fl::kPilotKeysHelp;
    for (const char* key : {"Space", "PgUp", "PgDn", "arrows", "Z", "X", "R"}) {
        EXPECT_NE(help.find(key), std::string::npos) << key << " in: " << help;
    }
}
