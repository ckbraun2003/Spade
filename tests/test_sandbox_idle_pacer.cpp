// L74b, render on demand: when the editor waits for an event and when it draws, asserted with no window.
//
// The loop in main.cpp asks the pacer how long to wait before each frame; the answer is "not at all" while anything
// is moving and "the heartbeat" otherwise, so an idle editor draws a few frames a second and a busy one draws as
// fast as before. Everything that DECIDES this lives in sandbox/idle_pacer.hpp, a header with no GLFW in it.
// Expectations are hand-written: the counts of frames and the seconds are the numbers the header promises.

#include <gtest/gtest.h>

#include "../sandbox/idle_pacer.hpp"

namespace {

using spade::sandbox::FrameInput;
using spade::sandbox::IdlePacer;
using spade::sandbox::input_held;
using spade::sandbox::kIdleHeartbeatS;
using spade::sandbox::kIdleSettleFrames;

// Draws frames as the loop does until the pacer first asks to wait; returns how many it drew without waiting.
int frames_until_idle(IdlePacer& pacer) {
    int frames = 0;
    while (pacer.wait_s() == 0.0 && frames < 1000) {
        pacer.frame_done(FrameInput{}, /*animating=*/false);
        ++frames;
    }
    return frames;
}

TEST(IdlePacer, ConstantsAreTheDocumentedOnes) {
    EXPECT_EQ(kIdleSettleFrames, 3);
    EXPECT_EQ(kIdleHeartbeatS, 0.25);
}

TEST(IdlePacer, FreshPacerDrawsItsSettleFramesThenWaitsTheHeartbeat) {
    IdlePacer pacer;
    EXPECT_EQ(frames_until_idle(pacer), 3);
    EXPECT_EQ(pacer.wait_s(), 0.25);
    EXPECT_EQ(pacer.wait_s(), 0.25);  // and keeps asking for it: an idle editor draws on the heartbeat alone
}

TEST(IdlePacer, AnEventDrawsTheSettleFramesAgain) {
    IdlePacer pacer;
    ASSERT_EQ(frames_until_idle(pacer), 3);
    pacer.woken();
    EXPECT_EQ(frames_until_idle(pacer), 3);
    EXPECT_EQ(pacer.wait_s(), 0.25);
}

TEST(IdlePacer, AnimationNeverWaits) {
    IdlePacer pacer;
    ASSERT_EQ(frames_until_idle(pacer), 3);
    // The click that started the run woke the loop out of the idle wait; that frame finds the run animating.
    ASSERT_EQ(pacer.wait_s(), 0.25);
    pacer.frame_done(FrameInput{}, /*animating=*/true);
    for (int i = 0; i < 500; ++i) {
        ASSERT_EQ(pacer.wait_s(), 0.0) << "frame " << i;
        pacer.frame_done(FrameInput{}, /*animating=*/true);
    }
    // The run stops: three more frames, then the heartbeat.
    EXPECT_EQ(frames_until_idle(pacer), 3);
}

TEST(IdlePacer, AHeldInputNeverWaits) {
    IdlePacer pacer;
    ASSERT_EQ(frames_until_idle(pacer), 3);
    FrameInput drag;
    drag.orbit_dx = 4.0f;
    ASSERT_EQ(pacer.wait_s(), 0.25);  // the first pixel of the drag woke the loop out of the idle wait
    pacer.frame_done(drag, /*animating=*/false);
    for (int i = 0; i < 100; ++i) {
        ASSERT_EQ(pacer.wait_s(), 0.0) << "frame " << i;
        pacer.frame_done(drag, /*animating=*/false);
    }
    EXPECT_EQ(frames_until_idle(pacer), 3);
}

TEST(IdlePacer, AQuietFrameDoesNotHeatThePacer) {
    IdlePacer pacer;
    ASSERT_EQ(frames_until_idle(pacer), 3);
    pacer.frame_done(FrameInput{}, false);  // the heartbeat's own frame
    EXPECT_EQ(pacer.wait_s(), 0.25);
}

TEST(IdlePacer, InputHeldNamesEveryAskThatNeedsAFollowingFrame) {
    EXPECT_FALSE(input_held(FrameInput{}));

    const auto only = [](auto set) {
        FrameInput in;
        set(in);
        return input_held(in);
    };
    EXPECT_TRUE(only([](FrameInput& in) { in.orbit_dx = 1.0f; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.orbit_dy = -1.0f; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.dolly = 1.0f; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.move_forward = -1.0f; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.move_right = 1.0f; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.move_up = 1.0f; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.want_close = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.left_click = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.left_down = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.left_release = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.delete_pressed = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.duplicate_pressed = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.frame_pressed = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.tool_move_pressed = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.tool_rotate_pressed = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.attitude_pitch = 1.0f; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.attitude_roll = -1.0f; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.attitude_yaw = 1.0f; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.level_pressed = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.arm_pressed = true; }));
    EXPECT_TRUE(only([](FrameInput& in) { in.throttle = 1.0f; }));

    // Not asks: where the cursor rests, a panel owning the pointer or the keys, and Ctrl being down. A cursor that
    // MOVES wakes the loop as an event; one that rests over a panel is exactly the idle editor.
    EXPECT_FALSE(only([](FrameInput& in) { in.mouse_x = 300.0f; in.mouse_y = 200.0f; }));
    EXPECT_FALSE(only([](FrameInput& in) { in.ui_captured_mouse = true; }));
    EXPECT_FALSE(only([](FrameInput& in) { in.ui_captured_keyboard = true; }));
    EXPECT_FALSE(only([](FrameInput& in) { in.snap_held = true; }));
}

}  // namespace
