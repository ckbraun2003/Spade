// N17's pilot keys (the lead's Q1): what each key does to the pilot's
// analysis::Controls, display-free. The window reads the keys into FrameInput
// (orbit_camera.hpp) only when ImGui doesn't have the keyboard, and the
// workspace applies them only while Run flies a drone.
//
//   Space        arm / disarm (an edge); disarming drops the throttle, so the
//                drone re-arms at idle, never at its old throttle
//   PgUp / PgDn  throttle up / down, 50% a second, within [0, 1]
//   arrows       tilt: Up nose down, Down nose up, Right the right side down,
//                Left the left (the drone box's keys), toward 30 degrees at
//                60 degrees a second, back to level when released
//   Z / X        heading left / right, 90 degrees a second, wrapping
//   R            level pitch and roll (an edge); the heading stays
#pragma once

#include <algorithm>
#include <cmath>

#include "analysis/case.hpp"  // Controls

namespace spade::sandbox::flight {

// The keys as one frame holds them: an axis is -1, 0 or +1.
struct PilotKeys {
    bool arm_pressed = false;    // Space, edge
    float throttle = 0.0f;       // PgUp +1, PgDn -1
    float pitch = 0.0f;          // Down +1 (nose up), Up -1 (nose down)
    float roll = 0.0f;           // Right +1, Left -1
    float yaw = 0.0f;            // Z +1 (left), X -1
    bool level_pressed = false;  // R, edge
};

inline constexpr float kThrottlePerSecond = 0.5f;
inline constexpr float kTiltDegPerSecond = 60.0f;
inline constexpr float kTiltMaxDeg = 30.0f;
inline constexpr float kYawDegPerSecond = 90.0f;

// The Pilot section shows it, so nobody has to remember the keys (the lead).
inline constexpr const char* kPilotKeysHelp =
    "Space arm/disarm, PgUp/PgDn throttle, arrows pitch and roll, Z/X turn, R level";

namespace detail {

// `deg` moved toward `held` * kTiltMaxDeg (0 when released) by at most the
// tilt rate times dt.
[[nodiscard]] inline float tilt_toward(float deg, float held, float dt) {
    const float target = std::clamp(held, -1.0f, 1.0f) * kTiltMaxDeg;
    const float step = kTiltDegPerSecond * dt;
    if (deg < target) return std::min(deg + step, target);
    if (deg > target) return std::max(deg - step, target);
    return deg;
}

}  // namespace detail

inline void apply_pilot_keys(const PilotKeys& keys, float dt, analysis::Controls& c) {
    if (!(dt >= 0.0f) || !std::isfinite(dt)) dt = 0.0f;
    if (keys.arm_pressed) {
        c.armed = !c.armed;
        if (!c.armed) c.throttle = 0.0f;
    }
    c.throttle = std::clamp(c.throttle + std::clamp(keys.throttle, -1.0f, 1.0f) * kThrottlePerSecond * dt, 0.0f, 1.0f);
    c.pitch_deg = detail::tilt_toward(c.pitch_deg, keys.pitch, dt);
    c.roll_deg = detail::tilt_toward(c.roll_deg, keys.roll, dt);
    float yaw = c.yaw_deg + std::clamp(keys.yaw, -1.0f, 1.0f) * kYawDegPerSecond * dt;
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw <= -180.0f) yaw += 360.0f;
    c.yaw_deg = yaw;
    if (keys.level_pressed) {
        c.pitch_deg = 0.0f;
        c.roll_deg = 0.0f;
    }
}

}  // namespace spade::sandbox::flight
