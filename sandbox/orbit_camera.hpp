// OrbitCamera -- Plan C task C2's input model, and it deliberately knows
// nothing about GLFW, GL or ImGui.
//
// ⭐⭐ WHY THIS IS ITS OWN HEADER RATHER THAN STATE INSIDE THE WINDOW.
// SL15b: "the window is a deliberately dumb shell -- any logic that migrates
// into it is a defect, because it becomes untestable by construction." Camera
// control IS logic: clamping, the spherical-to-cartesian conversion, and the
// basis the WASD keys move along are all things that can be wrong. Put them in
// the windowed sink and they can only be checked by a human looking at a
// screen, on a platform that has a display -- which the CI gate does not.
//
// Here, every one of them is a pure function of a struct, compiles on every
// platform including the no-backend Linux gate, and can be asserted without a
// window ever existing.
//
// ⚠ THE SIGN CONVENTION IS THE ENGINE'S AND IS NOT A CHOICE MADE HERE.
// raster_cpu.cpp states it: "camera space is right-handed with forward = -Z".
// glm::quatLookAt orients local -Z along its direction argument, which is the
// same convention, so the two agree BY CITATION rather than by coincidence.
// If that ever stops being true the fix belongs in to_render_camera() alone.

#pragma once

#include <algorithm>
#include <cmath>

#include <glm/glm.hpp>
// gtc, NOT gtx: quatLookAt is declared in gtc/quaternion.hpp in the pinned glm
// (verified in the fetched source), and every gtx/ header is a hard error
// unless GLM_ENABLE_EXPERIMENTAL is defined. Reaching for the experimental
// tree for a symbol the stable one already exports would put a compile-time
// dependency on a macro nobody in this target sets.
#include <glm/gtc/quaternion.hpp>

#include "render/target.hpp"

namespace spade::sandbox {

// What one frame of user input asked for, in units the camera understands.
// The windowed sink fills this and knows nothing else; the application applies
// it. That split is what keeps the shell dumb: the sink reports EVENTS, never
// camera state.
struct FrameInput {
    float orbit_dx = 0.0f;      // mouse drag, pixels, +x = drag right
    float orbit_dy = 0.0f;      // mouse drag, pixels, +y = drag down
    float dolly = 0.0f;         // scroll wheel, notches, + = toward the target
    float move_forward = 0.0f;  // -1..1, W/S, along the camera's ground facing
    float move_right = 0.0f;    // -1..1, D/A, across it
    float move_up = 0.0f;       // -1..1, E/Q, world +Y
    bool want_close = false;

    // ----- Builder input (the builder task) -------------------------------
    //
    // THE ORBIT MOVED TO THE RIGHT BUTTON AND THAT IS A REQUIREMENT, NOT A
    // PREFERENCE. Left-drag cannot both orbit the camera and move an object:
    // whichever one it does, the other becomes unreachable. Every builder in
    // this category resolves it the same way -- left selects and drags, right
    // orbits -- so `orbit_dx/dy` are now fed by the RIGHT button and the left
    // button reports as the three fields below.
    //
    // THREE FIELDS FOR ONE BUTTON, because a click and a drag are different
    // gestures and collapsing them loses one. `left_click` is the EDGE (select
    // or place), `left_down` is the LEVEL (continue a drag), `left_release`
    // ends it. A builder written against the level alone re-picks every frame
    // of a drag and the object you are moving swaps under the cursor.
    float mouse_x = 0.0f;   // framebuffer pixels, ORIGIN TOP-LEFT, +y DOWN
    float mouse_y = 0.0f;
    bool left_click = false;
    bool left_down = false;
    bool left_release = false;

    // Set when the UI owns the pointer/keys this frame. The scene must ignore
    // input while a panel has it, or every click that lands on the inspector
    // ALSO places an object behind it.
    bool ui_captured_mouse = false;
    bool ui_captured_keyboard = false;

    bool delete_pressed = false;     // Delete/Backspace -- remove the selection
    bool duplicate_pressed = false;  // Ctrl+D

    // ----- Drone sim box input ---------------------------------------------
    // Held axes, -1..1, each nudging an attitude target that holds when
    // released. Signs are drone_sim.hpp's AttitudeTarget's: + pitch = nose up,
    // + roll = right side down, + yaw = nose turns left (about +Y). The keys
    // map like a stick: Up arrow pushes the nose DOWN (-1), Right arrow rolls
    // right (+1), Z yaws left (+1).
    float attitude_pitch = 0.0f;  // Down = +1, Up = -1
    float attitude_roll = 0.0f;   // Right = +1, Left = -1
    float attitude_yaw = 0.0f;    // Z = +1, X = -1
    bool level_pressed = false;        // R, edge -- pitch and roll back to 0
    bool toggle_view_pressed = false;  // V, edge -- standard <-> heatmap
};

// An orbit rig: a target point, a direction to it, and a distance.
//
// Chosen over a free-fly camera for a reason worth stating -- the user asked
// for a surface to run INTERACTION TESTING on, and an orbit rig cannot get
// lost. A free-fly camera pointed at empty sky looks exactly like a renderer
// that drew nothing, which is the single worst failure mode for a tool whose
// job is to tell you whether the renderer works.
struct OrbitCamera {
    glm::vec3 target{0.0f, 0.5f, 0.0f};
    float yaw = 0.0f;        // radians about world +Y
    float pitch = 0.30f;     // radians; POSITIVE puts the camera ABOVE the target
    float distance = 8.0f;   // metres from target to eye

    // Tuning. Public so a future --sensitivity flag needs no surgery, and
    // named rather than inlined so the numbers are reviewable.
    float orbit_radians_per_pixel = 0.006f;
    float dolly_factor_per_notch = 0.9f;  // <1 so a notch toward the target shrinks distance
    float move_metres_per_second = 4.0f;

    static constexpr float kMinDistance = 0.35f;
    static constexpr float kMaxDistance = 400.0f;
    // Not pi/2: at exactly pole the view direction is parallel to `up` and
    // quatLookAt is degenerate -- it does not merely look odd, it produces a
    // NaN quaternion and every subsequent frame is garbage. The clamp is a
    // CORRECTNESS bound, which is why it is a named constant and not a nudge.
    static constexpr float kPitchLimit = 1.5533430342749532f;  // pi/2 - 0.0175 (~1 degree)

    void apply(const FrameInput& in, float dt_seconds) noexcept {
        yaw -= in.orbit_dx * orbit_radians_per_pixel;
        pitch += in.orbit_dy * orbit_radians_per_pixel;
        pitch = std::clamp(pitch, -kPitchLimit, kPitchLimit);

        if (in.dolly != 0.0f) {
            // Multiplicative, not additive: a fixed step feels glacial when far
            // out and unusable when close in, and the ratio is what the eye
            // reads as constant speed.
            distance *= std::pow(dolly_factor_per_notch, in.dolly);
            distance = std::clamp(distance, kMinDistance, kMaxDistance);
        }

        if (in.move_forward != 0.0f || in.move_right != 0.0f || in.move_up != 0.0f) {
            // Movement is scaled by distance so the rig pans at a speed
            // proportional to what is on screen -- the same argument as the
            // multiplicative dolly.
            const float speed = move_metres_per_second * dt_seconds * std::max(1.0f, distance * 0.25f);
            const glm::vec3 fwd = ground_forward();
            const glm::vec3 right = ground_right();
            target += fwd * (in.move_forward * speed);
            target += right * (in.move_right * speed);
            target.y += in.move_up * speed;
        }
    }

    // The eye's offset FROM the target. Spherical, +Y up.
    [[nodiscard]] glm::vec3 eye_offset() const noexcept {
        const float cp = std::cos(pitch);
        return glm::vec3(cp * std::sin(yaw), std::sin(pitch), cp * std::cos(yaw));
    }

    [[nodiscard]] glm::vec3 eye_position() const noexcept { return target + eye_offset() * distance; }

    // The facing direction flattened into the ground plane -- what W should
    // move along. Derived from `yaw` alone rather than from the 3D forward,
    // because normalising a nearly-vertical forward is exactly where a pan
    // starts drifting.
    [[nodiscard]] glm::vec3 ground_forward() const noexcept {
        return glm::vec3(-std::sin(yaw), 0.0f, -std::cos(yaw));
    }
    [[nodiscard]] glm::vec3 ground_right() const noexcept {
        return glm::vec3(std::cos(yaw), 0.0f, -std::sin(yaw));
    }

    [[nodiscard]] spade::render::Camera to_render_camera() const noexcept {
        spade::render::Camera cam;
        cam.position = eye_position();
        // Forward is FROM the eye TO the target, i.e. the negated offset. The
        // engine's forward is local -Z and quatLookAt orients local -Z along
        // its argument, so this hands it the forward directly.
        cam.orientation = glm::quatLookAt(-eye_offset(), glm::vec3(0.0f, 1.0f, 0.0f));
        return cam;
    }
};

// The drone sim box's camera: an orbit that never lets go of the drone.
//
// The target stays where the caller puts it (the drone), so the camera always
// faces it. The keys that pan the target in the builder orbit instead: A/D
// around it, E/Q over and under it, W/S nearer and further. Mouse drag and the
// scroll wheel work as in OrbitCamera::apply. Distance is clamped to a view
// sphere sized for a 36 cm airframe.
inline constexpr float kDroneMinDistance = 0.5f;
inline constexpr float kDroneMaxDistance = 6.0f;
inline constexpr float kDroneOrbitRadiansPerSecond = 1.5f;
inline constexpr float kDroneDollyPerSecond = 1.5f;  // e-folds of distance per second of W/S

inline void apply_drone_orbit(OrbitCamera& cam, const FrameInput& in, float dt_seconds) noexcept {
    // d(eye_offset)/d(yaw) is cos(pitch) * ground_right(), so +yaw carries the
    // eye to the camera's right: D, which reports move_right = +1.
    cam.yaw += (in.move_right * kDroneOrbitRadiansPerSecond * dt_seconds) - in.orbit_dx * cam.orbit_radians_per_pixel;
    cam.pitch += (in.move_up * kDroneOrbitRadiansPerSecond * dt_seconds) + in.orbit_dy * cam.orbit_radians_per_pixel;
    cam.pitch = std::clamp(cam.pitch, -OrbitCamera::kPitchLimit, OrbitCamera::kPitchLimit);

    // Multiplicative for the same reason the builder's dolly is: the ratio is
    // what reads as constant speed.
    cam.distance *= std::exp(-in.move_forward * kDroneDollyPerSecond * dt_seconds);
    if (in.dolly != 0.0f) cam.distance *= std::pow(cam.dolly_factor_per_notch, in.dolly);
    cam.distance = std::clamp(cam.distance, kDroneMinDistance, kDroneMaxDistance);
}

}  // namespace spade::sandbox
