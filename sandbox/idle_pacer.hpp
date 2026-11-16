// RENDER ON DEMAND (L74b): when the editor's loop waits for an event and when it draws.
//
// An idle editor used to draw every pass of its loop, flat out under --no-vsync (about 885 fps and more than a core
// in a 30 s hold, Test/Docs' measurement) and at the display's rate with vsync, for a window nothing was moving in.
// Now the loop asks this pacer how long to wait before each frame. The answer is NOT AT ALL while anything is moving
// (a run or Run air stepping, a key or the mouse held, a gizmo drag) and for a few frames after, and otherwise the
// HEARTBEAT: the loop sleeps in the window system until an event arrives or a quarter second passes, then draws once.
//
// WHY THE FEW FRAMES AFTER, AND WHY A HEARTBEAT AT ALL. ImGui needs more than one frame to settle after an input:
// a hover highlight shows a frame late, a popup opens on the second frame. And a few things change with no event:
// a text field's cursor blinks, the HUD's frame time moves, a worker thread hands over a result. The heartbeat is the
// floor under those, four frames a second of a frame that costs about a millisecond.
//
// Display-free and GLFW-free, so tests/test_sandbox_idle_pacer.cpp checks every decision; the wait itself is
// GlTargetSink::wait_events().
//
// THE TOUR DOES NOT USE IT. The live tour pumps a counted number of frames at a fixed step, so its length is the
// same on any machine; a pacer there would only change how many frames pass.

#pragma once

#include "orbit_camera.hpp"

namespace spade::sandbox {

// The longest an idle editor sleeps before it draws once anyway, seconds.
inline constexpr double kIdleHeartbeatS = 0.25;
// Frames drawn without waiting after an event or a held input, so ImGui's hover and popups settle.
inline constexpr int kIdleSettleFrames = 3;

// True when this frame's input asks for a following frame (a drag, a held key, an edge to act on). Where the cursor
// rests and whether a panel owns the pointer or the keys are not asks: a moving cursor is an EVENT and wakes the
// loop on its own, and one resting over a panel is the idle editor.
[[nodiscard]] constexpr bool input_held(const FrameInput& in) noexcept {
    return in.orbit_dx != 0.0f || in.orbit_dy != 0.0f || in.dolly != 0.0f || in.move_forward != 0.0f ||
           in.move_right != 0.0f || in.move_up != 0.0f || in.want_close || in.left_click || in.left_down ||
           in.left_release || in.delete_pressed || in.duplicate_pressed || in.frame_pressed ||
           in.tool_move_pressed || in.tool_rotate_pressed || in.attitude_pitch != 0.0f ||
           in.attitude_roll != 0.0f || in.attitude_yaw != 0.0f || in.level_pressed || in.arm_pressed ||
           in.throttle != 0.0f;
}

class IdlePacer {
public:
    // Seconds the loop may wait for an event before the next frame: 0 to draw now (a fresh pacer, an event, an
    // animation or a held input just happened), the heartbeat once the settle frames are drawn.
    [[nodiscard]] double wait_s() noexcept {
        if (hot_ > 0) {
            --hot_;
            return 0.0;
        }
        return kIdleHeartbeatS;
    }

    // The wait ended because an event arrived (not because the heartbeat ran out).
    void woken() noexcept { hot_ = kIdleSettleFrames; }

    // After a frame: `animating` is the session's (a run or Run air steps every frame).
    void frame_done(const FrameInput& in, bool animating) noexcept {
        if (animating || input_held(in)) hot_ = kIdleSettleFrames;
    }

private:
    int hot_ = kIdleSettleFrames;  // the first frames are drawn without waiting
};

}  // namespace spade::sandbox
