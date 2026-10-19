// The end-goal smoke's checks, display-free (N20,
// docs/design/test-docs/2026-10-19-end-goal-checks.md).
//
// Each check takes the state an editor step produced and returns a Verdict
// that names what it measured, or why the step can't be judged. The tour's
// end-goal steps call these; tests/test_sandbox_end_goal_checks.cpp shows each
// one failing on a stub of the fault it exists to catch, and passing on a stub
// of the right state. Header-only, like scene_checks.hpp, so spade_tests
// reaches it by include path.
//
// EVERY NUMBER BELOW IS THE DOC'S, FIXED A PRIORI. Change one only by a
// recorded lead decision, in the doc first.
//
// A verdict that can't be reached says why, by its first word:
//   "absent:"         the frames don't cover what the check needs (TD-5),
//                     or hold a time that is not finite and increasing, or a
//                     state that is not finite;
//   "mis-staged:"     the step didn't set up what the check presumes;
//   "under-resolved:" the region is too coarse for the check to mean anything.
// None of these passes.
//
// ONE FRAME: the rotors, the body and the published volume must be in the
// same world frame. LiveRun publishes in its own, centred on its box with the
// held body at the origin, so a step passes the body LiveRun holds, not the
// body's pose in the scene. A probe outside the volume is absent, never read.
//
// Owned by Test/Docs; reviewed by Interface.

#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/flow_volume.hpp"
#include "render/flow_sample.hpp"
#include "render/scene.hpp"
#include "render/view_mode.hpp"
#include "state/layout.hpp"
#include "vehicles/rotor.hpp"
#include "world/sdf.hpp"

namespace spade::sandbox::end_goal {

struct Verdict {
    bool ok = false;
    std::string detail;
};

namespace detail {

inline constexpr double kTimeEps = 1e-9;  // frame times are whole steps; this only absorbs their rounding

[[nodiscard]] inline std::string num(double v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.4g", v);
    return buf;
}

[[nodiscard]] inline bool finite(const glm::vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

[[nodiscard]] inline bool finite(const glm::quat& q) {
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

[[nodiscard]] inline bool positive(float x) { return std::isfinite(x) && x > 0.0f; }

// Empty when every frame's t_s is finite and later than the one before;
// otherwise which isn't.
template <class Frame>
[[nodiscard]] std::string time_fault(std::span<const Frame> frames) {
    for (std::size_t i = 0; i < frames.size(); ++i) {
        if (!std::isfinite(frames[i].t_s)) return "frame " + std::to_string(i) + "'s time is not finite";
        if (i > 0 && !(frames[i].t_s > frames[i - 1].t_s)) {
            return "frame " + std::to_string(i) + "'s time, " + num(frames[i].t_s) + " s, does not follow frame " +
                   std::to_string(i - 1) + "'s, " + num(frames[i - 1].t_s) + " s";
        }
    }
    return {};
}

}  // namespace detail

// ---------------------------------------------------------------------------
// ROTORS, as the checks see them: posed from the model's rotor rows by the
// body (rotor_discs()'s rule), never from a coupling's disc rows, so a disc
// the coupling misplaces is judged where the rotor really is.
// ---------------------------------------------------------------------------

struct RotorProbe {
    glm::vec3 hub{0.0f};                // world, m
    glm::vec3 axis{0.0f, 1.0f, 0.0f};   // the thrust axis, unit; the disc pushes the air along -axis
    float radius = 0.0f;                // m
    float thrust = 0.0f;                // k_T w^2, N: the static thrust, the rigid-body path's
};

[[nodiscard]] inline RotorProbe probe_rotor(const vehicles::RotorRow& row, const BodyState& body) {
    RotorProbe p;
    p.hub = body.pos + body.orient * row.local_pos;
    p.axis = glm::normalize(body.orient * (row.local_orient * vehicles::kRotorLocalThrustAxis));
    p.radius = row.radius;
    p.thrust = row.thrust_coeff * row.omega * row.omega;
    return p;
}

// F_y: the rotors' static thrust along world +Y, N.
[[nodiscard]] inline float vertical_thrust(std::span<const RotorProbe> rotors) {
    float f = 0.0f;
    for (const RotorProbe& r : rotors) f += r.thrust * r.axis.y;
    return f;
}

// The body's tilt from level: the angle between its +Y and the world's, rad.
[[nodiscard]] inline float tilt_of(const glm::quat& body_to_world) {
    const glm::vec3 up = body_to_world * glm::vec3(0.0f, 1.0f, 0.0f);
    return std::atan2(glm::length(glm::cross(up, glm::vec3(0.0f, 1.0f, 0.0f))), up.y);
}

// ---------------------------------------------------------------------------
// DOWNWASH (§5). Momentum theory's axial velocity 2R below a uniformly loaded
// disc, on the vortex-cylinder model (C8's reference, tests/data/airflow/
// README.md): v_h (1 + 2/sqrt 5), v_h = sqrt(T / (2 rho pi R^2)).
// ---------------------------------------------------------------------------

inline constexpr float kFarWake2R = 1.8944271909999159f;  // 1 + 2/sqrt(5)

[[nodiscard]] inline float momentum_theory_2r(float thrust, float radius, float density) {
    if (!(thrust > 0.0f && radius > 0.0f && density > 0.0f)) return 0.0f;
    const float area = std::numbers::pi_v<float> * radius * radius;
    return kFarWake2R * std::sqrt(thrust / (2.0f * density * area));
}

// The probe: 2R below the hub on its axis, a disc of 0.5R normal to the axis,
// read on a square lattice of pitch 0.05R (Physics: the axis point sits in the
// hub vortex; the disc averages it out).
inline constexpr float kProbeDepthR = 2.0f;
inline constexpr int kProbeLatticeHalf = 10;  // lattice index j in [-10, 10]: j * 0.05R
inline constexpr float kProbePitchR = 0.05f;  // kProbeLatticeHalf * kProbePitchR = the disc's 0.5R

enum class ProbeStatus : uint8_t {
    ok = 0,
    no_data = 1,  // a lattice point read no-data
    outside = 2,  // a lattice point is outside the volume's box, where the sampler holds the edge values
};

struct ProbeReading {
    float u = 0.0f;  // the downward axial velocity averaged over the probe disc, m/s; 0 unless ok
    ProbeStatus status = ProbeStatus::ok;
};

// Reads one rotor's probe. Precondition: `volume` validated, and `sampler`
// made from it.
[[nodiscard]] inline ProbeReading probe_downwash(const FlowVolume& volume, const render::FlowSampler& sampler,
                                                 const RotorProbe& r) {
    const FlowVolumeHeader& h = volume.header;
    const glm::vec3 box_lo = h.origin;
    const glm::vec3 box_hi = h.origin + h.cell * glm::vec3(static_cast<float>(h.nx), static_cast<float>(h.ny),
                                                           static_cast<float>(h.nz));
    const glm::vec3 n = r.axis;
    const glm::vec3 helper = std::abs(n.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 e1 = glm::normalize(glm::cross(n, helper));
    const glm::vec3 e2 = glm::cross(n, e1);
    const glm::vec3 centre = r.hub - kProbeDepthR * r.radius * n;
    const float pitch = kProbePitchR * r.radius;
    double sum = 0.0;
    int count = 0;
    for (int j = -kProbeLatticeHalf; j <= kProbeLatticeHalf; ++j) {
        for (int k = -kProbeLatticeHalf; k <= kProbeLatticeHalf; ++k) {
            if (j * j + k * k > kProbeLatticeHalf * kProbeLatticeHalf) continue;
            const glm::vec3 p = centre + (static_cast<float>(j) * pitch) * e1 + (static_cast<float>(k) * pitch) * e2;
            if (!(glm::all(glm::greaterThanEqual(p, box_lo)) && glm::all(glm::lessThanEqual(p, box_hi)))) {
                return {0.0f, ProbeStatus::outside};
            }
            const render::FlowVelocity v = render::sample_velocity(sampler, p);
            if (v.no_data) return {0.0f, ProbeStatus::no_data};
            sum += -static_cast<double>(glm::dot(n, v.u));
            ++count;
        }
    }
    return {static_cast<float>(sum / count), ProbeStatus::ok};
}

// One published frame, as the downwash checks read it.
struct DownwashFrame {
    double t_s = 0.0;                // simulated time since arming
    float cell = 0.0f;               // the published volume's dx, m
    float floor_y = 0.0f;            // the published volume's bottom face, world y, m
    std::vector<RotorProbe> rotors;  // posed this frame, in the volume's frame
    std::vector<float> u_down;       // per rotor: the probe's reading, NaN unless ok
    std::vector<ProbeStatus> probe;  // per rotor
    float tilt_rad = 0.0f;           // tilt_of() the body
    bool controls_constant = true;   // the controls are still the ones set at arming
};

// Reads one published frame. An invalid volume reads as no data everywhere.
[[nodiscard]] inline DownwashFrame sample_downwash(const FlowVolume& volume, std::vector<RotorProbe> rotors,
                                                   double t_s, float tilt_rad, bool controls_constant) {
    DownwashFrame f;
    f.t_s = t_s;
    f.cell = volume.header.cell;
    f.floor_y = volume.header.origin.y;
    f.tilt_rad = tilt_rad;
    f.controls_constant = controls_constant;
    f.u_down.assign(rotors.size(), std::numeric_limits<float>::quiet_NaN());
    f.probe.assign(rotors.size(), ProbeStatus::no_data);
    if (validate_flow_volume(volume).has_value()) {
        const render::FlowSampler sampler = render::make_flow_sampler(volume);
        for (std::size_t i = 0; i < rotors.size(); ++i) {
            const ProbeReading reading = probe_downwash(volume, sampler, rotors[i]);
            f.probe[i] = reading.status;
            if (reading.status == ProbeStatus::ok) f.u_down[i] = reading.u;
        }
    }
    f.rotors = std::move(rotors);
    return f;
}

// The case. L, the hub plane's height above the box's bottom face, is not
// declared here: the check takes it from the frames (the hubs and the
// published volume's floor).
struct DownwashCase {
    float density = 1.225f;    // kg/m^3, the case's
    float mass = 0.0f;         // kg, the model's
    float gravity = 9.80665f;  // m/s^2
};

// What a downwash check demands: lo <= mean u / mean v_ref <= hi, at R/dx >= min.
struct DownwashRule {
    const char* name;
    float lo;
    float hi;
    float min_r_over_dx;
};

// The headless band, 64^3: a coupling check, not a validation claim.
inline constexpr DownwashRule kDownwash{"downwash", 0.75f, 1.25f, 4.0f};
// The live floor, 32^3: direction and presence, not magnitude.
inline constexpr DownwashRule kDownwashPresent{"downwash-present", 0.25f, std::numeric_limits<float>::infinity(),
                                               2.0f};

inline constexpr double kDevelopFloorS = 0.30;   // developed no sooner than this after arming
inline constexpr double kDevelopTransits = 4.0;  // nor before four wake transits of L at v_ref
inline constexpr double kDownwashWindowS = 0.20;
inline constexpr std::size_t kDownwashMinFrames = 6;
inline constexpr float kLevelRad = 0.017453292519943295f;  // 1 degree
inline constexpr float kLoadedFraction = 0.5f;             // sum T >= this x m g

// Each rotor's mean probe_downwash() over the window, against its mean
// momentum-theory reference, judged by `rule`. The frames are in time order,
// from arming.
[[nodiscard]] inline Verdict downwash(std::span<const DownwashFrame> frames, const DownwashCase& c,
                                      const DownwashRule& rule) {
    using detail::num;
    const std::string name = std::string(rule.name) + ": ";
    if (!(detail::positive(c.density) && detail::positive(c.mass) && detail::positive(c.gravity))) {
        return {false, name + "mis-staged: the case's density " + num(c.density) + ", mass " + num(c.mass) +
                           " and gravity " + num(c.gravity) + " must all be positive"};
    }
    if (frames.empty()) return {false, name + "absent: no frames were recorded"};
    if (const std::string fault = detail::time_fault(frames); !fault.empty()) return {false, name + "absent: " + fault};
    const std::size_t n = frames.back().rotors.size();
    if (n == 0) return {false, name + "absent: the model has no rotors"};
    for (const DownwashFrame& f : frames) {
        if (f.rotors.size() != n || f.u_down.size() != n || f.probe.size() != n) {
            return {false, name + "absent: the frame at t = " + num(f.t_s) + " s lacks a rotor"};
        }
        bool finite = std::isfinite(f.cell) && std::isfinite(f.floor_y) && std::isfinite(f.tilt_rad);
        for (const RotorProbe& r : f.rotors) {
            finite = finite && detail::finite(r.hub) && detail::finite(r.axis) && std::isfinite(r.radius) &&
                     std::isfinite(r.thrust);
        }
        if (!finite) return {false, name + "absent: the frame at t = " + num(f.t_s) + " s holds a non-finite state"};
        for (std::size_t i = 0; i < n; ++i) {
            if (!(f.rotors[i].radius > 0.0f)) {
                return {false, name + "mis-staged: rotor " + std::to_string(i) + "'s radius is " +
                                   num(f.rotors[i].radius)};
            }
        }
    }

    // The resolution, from the published volume's own cell.
    float r_min = std::numeric_limits<float>::infinity();
    for (const RotorProbe& r : frames.back().rotors) r_min = std::min(r_min, r.radius);
    const float cell = frames.back().cell;
    const float r_over_dx = cell > 0.0f ? r_min / cell : 0.0f;
    if (!(r_over_dx >= rule.min_r_over_dx)) {
        return {false, name + "under-resolved: R/dx " + num(r_over_dx) + " is below " + num(rule.min_r_over_dx)};
    }

    // Developed, from the last frame's thrust: the controls are constant, so
    // the rotors are at their command by then, and one that makes no thrust
    // is a fault, not a staging. L from the same frame: the highest hub above
    // the published volume's floor.
    float v_min = std::numeric_limits<float>::infinity();
    float hub_top = -std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
        const RotorProbe& r = frames.back().rotors[i];
        const float v = momentum_theory_2r(r.thrust, r.radius, c.density);
        if (!(v > 0.0f)) {
            return {false, name + "rotor " + std::to_string(i) + " makes no thrust at the last frame, t = " +
                               num(frames.back().t_s) + " s"};
        }
        v_min = std::min(v_min, v);
        hub_top = std::max(hub_top, r.hub.y);
    }
    const float hub_above_floor = hub_top - frames.back().floor_y;
    if (!(hub_above_floor > 0.0f)) {
        return {false, name + "mis-staged: the hubs are " + num(hub_above_floor) +
                           " m above the published volume's floor"};
    }
    const double t_dev = std::max(kDevelopFloorS, kDevelopTransits * hub_above_floor / v_min);
    const double t_end = t_dev + kDownwashWindowS;
    const std::string window = "[" + num(t_dev) + ", " + num(t_end) + "] s";
    if (frames.back().t_s + detail::kTimeEps < t_end) {
        return {false, name + "absent: the frames end at t = " + num(frames.back().t_s) + " s, before the window " +
                           window + " closes"};
    }

    const float weight = c.mass * c.gravity;
    std::vector<double> u_sum(n, 0.0);
    std::vector<double> v_sum(n, 0.0);
    std::size_t count = 0;
    for (const DownwashFrame& f : frames) {
        if (f.t_s > t_end + detail::kTimeEps) break;
        if (!f.controls_constant) {
            return {false, name + "mis-staged: the controls changed at t = " + num(f.t_s) + " s"};
        }
        if (!(f.tilt_rad <= kLevelRad)) {
            return {false, name + "mis-staged: tilted " + num(f.tilt_rad / kLevelRad) + " degrees at t = " +
                               num(f.t_s) + " s, past 1"};
        }
        if (f.t_s + detail::kTimeEps < t_dev) continue;
        float total = 0.0f;
        for (const RotorProbe& r : f.rotors) total += r.thrust;
        if (!(total >= kLoadedFraction * weight)) {
            return {false, name + "mis-staged: the rotors carry " + num(total) + " N at t = " + num(f.t_s) +
                               " s, under half the weight, " + num(weight) + " N"};
        }
        for (std::size_t i = 0; i < n; ++i) {
            if (f.probe[i] == ProbeStatus::outside) {
                return {false, name + "absent: rotor " + std::to_string(i) + "'s probe is outside the published " +
                                   "volume at t = " + num(f.t_s) + " s"};
            }
            if (f.probe[i] != ProbeStatus::ok || !std::isfinite(f.u_down[i])) {
                return {false, name + "absent: no data at rotor " + std::to_string(i) + "'s probe at t = " +
                                   num(f.t_s) + " s"};
            }
            u_sum[i] += f.u_down[i];
            v_sum[i] += momentum_theory_2r(f.rotors[i].thrust, f.rotors[i].radius, c.density);
        }
        ++count;
    }
    if (count < kDownwashMinFrames) {
        return {false, name + "absent: " + std::to_string(count) + " frames in the window " + window + ", fewer than " +
                           std::to_string(kDownwashMinFrames)};
    }

    bool ok = true;
    std::string measured;
    for (std::size_t i = 0; i < n; ++i) {
        const double u = u_sum[i] / static_cast<double>(count);
        const double v = v_sum[i] / static_cast<double>(count);
        const double ratio = v > 0.0 ? u / v : std::numeric_limits<double>::quiet_NaN();
        const bool in = ratio >= rule.lo && ratio <= rule.hi;
        ok = ok && in;
        measured += "rotor " + std::to_string(i) + " " + num(u) + " / " + num(v) + " m/s = " + num(ratio) +
                    (in ? "" : " (OUT)") + "; ";
    }
    return {ok, name + measured + "band [" + num(rule.lo) + ", " + num(rule.hi) + "], " + std::to_string(count) +
                    " frames in " + window + ", R/dx " + num(r_over_dx)};
}

// ---------------------------------------------------------------------------
// FLIGHT (§4): what each frame of editor.fly records.
// ---------------------------------------------------------------------------

struct FlightFrame {
    double t_s = 0.0;
    float height = 0.0f;                            // the centre of mass's y, m
    glm::vec3 velocity{0.0f};                       // the centre of mass's, world, m/s
    glm::quat attitude{1.0f, 0.0f, 0.0f, 0.0f};     // body -> world; body +X the nose, +Y up, +Z the right
    float thrust_up = 0.0f;                         // vertical_thrust() of the posed rotors, N
    std::vector<float> rotor_omega;                 // rad/s, in the model's rotor order
};

namespace detail {

// Empty when the frames are in time and every state is finite; otherwise why not.
[[nodiscard]] inline std::string flight_fault(std::span<const FlightFrame> frames) {
    if (std::string fault = time_fault(frames); !fault.empty()) return fault;
    for (const FlightFrame& f : frames) {
        bool ok = std::isfinite(f.height) && finite(f.velocity) && finite(f.attitude) && std::isfinite(f.thrust_up);
        for (const float w : f.rotor_omega) ok = ok && std::isfinite(w);
        if (!ok) return "the frame at t = " + num(f.t_s) + " s holds a non-finite state";
    }
    return {};
}

}  // namespace detail

// + nose up.
[[nodiscard]] inline float pitch_of(const glm::quat& body_to_world) {
    const glm::vec3 nose = body_to_world * glm::vec3(1.0f, 0.0f, 0.0f);
    return std::asin(std::clamp(nose.y, -1.0f, 1.0f));
}

// + the right side down.
[[nodiscard]] inline float roll_of(const glm::quat& body_to_world) {
    const glm::vec3 right = body_to_world * glm::vec3(0.0f, 0.0f, 1.0f);
    return std::asin(std::clamp(-right.y, -1.0f, 1.0f));
}

inline constexpr double kArmSpinS = 0.5;      // every rotor at its idle by then
inline constexpr float kArmSpinFraction = 0.9f;
inline constexpr double kArmStillS = 1.0;     // and the drone still down over this long
inline constexpr float kArmStillM = 0.005f;

// Before arming every rotor is stopped; by 0.5 s after it (the last frame at
// or before then) every rotor turns at 90% of its idle command or more; over
// 1.0 s from arming the height changes by 5 mm or less. `idle_command` is
// each rotor's commanded idle speed.
[[nodiscard]] inline Verdict armed(std::span<const FlightFrame> frames, double arm_s,
                                   std::span<const float> idle_command) {
    using detail::num;
    constexpr double eps = detail::kTimeEps;
    const std::string name = "armed: ";
    if (!std::isfinite(arm_s)) return {false, name + "mis-staged: the arming time is not finite"};
    const std::size_t n = idle_command.size();
    if (n == 0) return {false, name + "mis-staged: no idle command was given"};
    for (const float w : idle_command) {
        if (!detail::positive(w)) return {false, name + "mis-staged: an idle command of " + num(w) + " rad/s"};
    }
    if (const std::string fault = detail::flight_fault(frames); !fault.empty()) {
        return {false, name + "absent: " + fault};
    }
    for (const FlightFrame& f : frames) {
        if (f.rotor_omega.size() != n) {
            return {false, name + "absent: the frame at t = " + num(f.t_s) + " s lacks a rotor"};
        }
    }

    std::size_t before = 0;
    for (const FlightFrame& f : frames) {
        if (!(f.t_s + eps < arm_s)) break;
        ++before;
        for (std::size_t i = 0; i < n; ++i) {
            if (f.rotor_omega[i] != 0.0f) {
                return {false, name + "rotor " + std::to_string(i) + " turned at " + num(f.rotor_omega[i]) +
                                   " rad/s before arming, at t = " + num(f.t_s) + " s"};
            }
        }
    }
    if (before == 0) return {false, name + "absent: no frame before arming at t = " + num(arm_s) + " s"};

    const auto first_at = [&](double t) {
        std::size_t i = 0;
        while (i < frames.size() && frames[i].t_s + eps < t) ++i;
        return i;
    };
    // The last frame at or before 0.5 s after arming, and not before arming.
    std::size_t is = frames.size();
    for (std::size_t i = 0; i < frames.size() && frames[i].t_s <= arm_s + kArmSpinS + eps; ++i) {
        if (frames[i].t_s + eps >= arm_s) is = i;
    }
    if (is == frames.size()) return {false, name + "absent: no frame in the 0.5 s after arming"};
    for (std::size_t i = 0; i < n; ++i) {
        const float w = frames[is].rotor_omega[i];
        if (!(w > 0.0f && w >= kArmSpinFraction * idle_command[i])) {
            return {false, name + "rotor " + std::to_string(i) + " turns at " + num(w) +
                               " rad/s at t = " + num(frames[is].t_s) + " s, under 90% of its idle " +
                               num(idle_command[i])};
        }
    }

    const std::size_t ia = first_at(arm_s);
    const double t_still = arm_s + kArmStillS;
    if (frames.back().t_s + eps < t_still) {
        return {false, name + "absent: the frames end at t = " + num(frames.back().t_s) + " s, before " + num(t_still)};
    }
    float moved = 0.0f;
    for (std::size_t k = ia; k < frames.size() && frames[k].t_s <= t_still + eps; ++k) {
        moved = std::max(moved, std::abs(frames[k].height - frames[ia].height));
    }
    const bool ok = moved <= kArmStillM;
    return {ok, name + "stopped in " + std::to_string(before) + " frames before arming; every rotor at 90% of " +
                    "idle by " + num(kArmSpinS) + " s; height moved " + num(moved * 1000.0f) +
                    " mm at idle (5 at most)"};
}

inline constexpr double kClimbWindowS = 1.0;
inline constexpr float kClimbPremise = 1.10f;  // F_y >= this x m g through the window
inline constexpr float kClimbClearR = 2.0f;    // the window starts this many R above the throttle-up's height
inline constexpr double kClimbLo = 0.5;
inline constexpr double kClimbHi = 1.10;

// The rise over a 1.0 s window against the drag-free double integral of the
// reported thrust. `radius` is the rotors' R (the ground-effect clearance).
[[nodiscard]] inline Verdict climbs(std::span<const FlightFrame> frames, double throttle_up_s, float mass,
                                    float gravity, float radius) {
    using detail::num;
    constexpr double eps = detail::kTimeEps;
    const std::string name = "climbs: ";
    if (!std::isfinite(throttle_up_s)) return {false, name + "mis-staged: the throttle-up time is not finite"};
    if (!(detail::positive(mass) && detail::positive(gravity) && detail::positive(radius))) {
        return {false, name + "mis-staged: the mass " + num(mass) + ", gravity " + num(gravity) + " and radius " +
                           num(radius) + " must all be positive"};
    }
    if (const std::string fault = detail::flight_fault(frames); !fault.empty()) {
        return {false, name + "absent: " + fault};
    }
    const double need = static_cast<double>(kClimbPremise) * mass * gravity;
    const double clear = static_cast<double>(kClimbClearR) * radius;

    std::size_t iu = 0;
    while (iu < frames.size() && frames[iu].t_s + eps < throttle_up_s) ++iu;
    if (iu == frames.size()) {
        return {false, name + "absent: no frame at or after the throttle-up at t = " + num(throttle_up_s) + " s"};
    }

    // The window opens at the first frame with the thrust premise met and the
    // drone 2R up: out of the ground effect the static thrust leaves out.
    bool premise_seen = false;
    double highest = 0.0;
    std::size_t i0 = frames.size();
    for (std::size_t i = iu; i < frames.size(); ++i) {
        const double rise = frames[i].height - frames[iu].height;
        highest = std::max(highest, rise);
        if (frames[i].thrust_up >= need) {
            premise_seen = true;
            if (rise >= clear) {
                i0 = i;
                break;
            }
        }
    }
    if (!premise_seen) {
        return {false, name + "mis-staged: the vertical thrust never reached 1.10 m g (" + num(need) +
                           " N) after the throttle-up"};
    }
    if (i0 == frames.size()) {
        return {false, name + "did not climb: never rose 2R (" + num(clear) +
                           " m) above its height at the throttle-up; its highest was +" + num(highest) + " m"};
    }
    const double t0 = frames[i0].t_s;
    std::size_t i1 = i0;
    while (i1 < frames.size() && frames[i1].t_s + eps < t0 + kClimbWindowS) ++i1;
    if (i1 == frames.size()) {
        return {false, name + "absent: the frames end at t = " + num(frames.back().t_s) +
                           " s, inside the window from " + num(t0) + " s"};
    }
    for (std::size_t k = i0; k <= i1; ++k) {
        if (frames[k].thrust_up < need) {
            return {false, name + "mis-staged: the vertical thrust fell to " + num(frames[k].thrust_up) +
                               " N at t = " + num(frames[k].t_s) + " s, under 1.10 m g"};
        }
    }

    // The drag-free double integral of the reported thrust, by the midpoint rule.
    const double t_end = frames[i1].t_s;
    double predicted = static_cast<double>(frames[i0].velocity.y) * (t_end - t0);
    for (std::size_t k = i0; k < i1; ++k) {
        const double dt = frames[k + 1].t_s - frames[k].t_s;
        const double f = 0.5 * (static_cast<double>(frames[k].thrust_up) + frames[k + 1].thrust_up);
        const double a = f / mass - gravity;
        const double t_mid = 0.5 * (frames[k].t_s + frames[k + 1].t_s);
        predicted += a * dt * (t_end - t_mid);
    }
    if (!(predicted > 0.0)) {
        return {false, name + "mis-staged: the reported thrust predicts " + num(predicted) + " m, not a rise"};
    }
    const double rise = static_cast<double>(frames[i1].height) - frames[i0].height;
    const double ratio = rise / predicted;
    const float v_end = frames[i1].velocity.y;
    const bool ok = !(ratio < kClimbLo) && !(ratio > kClimbHi) && std::isfinite(ratio) && v_end > 0.0f;
    return {ok, name + "rose " + num(rise) + " m in " + num(t_end - t0) + " s from t = " + num(t0) + " s, against " +
                    num(predicted) + " m predicted: " + num(ratio) + " (band [" + num(kClimbLo) + ", " +
                    num(kClimbHi) + "]); v_y " + num(v_end) + " m/s at the end"};
}

inline constexpr double kAttitudeReachS = 1.0;
inline constexpr double kAttitudeHoldS = 0.5;
inline constexpr double kAttitudeSpeedAtS = 1.5;
inline constexpr float kAttitudeTolRad = 0.05235987755982988f;  // 3 degrees
inline constexpr float kAttitudeSpeedGain = 0.5f;               // m/s along the heading, in the tilt's direction

// After a pitch command at command_s, the pitch reaches the command to 3
// degrees within 1.0 s and holds it for 0.5 s and through 1.5 s after the
// command; roll stays within 3 degrees; and by 1.5 s the horizontal velocity
// along the heading has gained more than 0.5 m/s since the command, the way
// the tilt points (nose down: forward).
[[nodiscard]] inline Verdict follows_attitude(std::span<const FlightFrame> frames, double command_s,
                                              float pitch_command_rad) {
    using detail::num;
    constexpr double eps = detail::kTimeEps;
    constexpr float kDeg = 57.29577951308232f;
    const std::string name = "follows-attitude: ";
    if (!(std::isfinite(command_s) && std::isfinite(pitch_command_rad))) {
        return {false, name + "mis-staged: the command's time or pitch is not finite"};
    }
    if (const std::string fault = detail::flight_fault(frames); !fault.empty()) {
        return {false, name + "absent: " + fault};
    }
    if (!(std::abs(pitch_command_rad) > kAttitudeTolRad)) {
        return {false, name + "mis-staged: a pitch command of " + num(pitch_command_rad * kDeg) +
                           " degrees is within 3 of level"};
    }
    std::size_t ic = 0;
    while (ic < frames.size() && frames[ic].t_s + eps < command_s) ++ic;
    if (ic == frames.size()) {
        return {false, name + "absent: no frame at or after the command at t = " + num(command_s) + " s"};
    }

    const glm::vec3 nose = frames[ic].attitude * glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 heading(nose.x, 0.0f, nose.z);
    if (!(glm::length(heading) > 1e-3f)) {
        return {false, name + "mis-staged: the nose points straight up or down at the command"};
    }
    heading = glm::normalize(heading);

    std::size_t ir = frames.size();
    float closest = std::numeric_limits<float>::infinity();
    for (std::size_t i = ic; i < frames.size() && frames[i].t_s <= command_s + kAttitudeReachS + eps; ++i) {
        const float off = std::abs(pitch_of(frames[i].attitude) - pitch_command_rad);
        closest = std::min(closest, off);
        if (off <= kAttitudeTolRad) {  // NaN never reaches
            ir = i;
            break;
        }
    }
    if (ir == frames.size()) {
        return {false, name + "the pitch came no nearer than " + num(closest * kDeg) + " degrees to the command of " +
                           num(pitch_command_rad * kDeg) + " within 1.0 s"};
    }
    // One end time. The reach is at most command + 1.0 s, so its 0.5 s hold
    // (kAttitudeHoldS) ends by command + 1.5 s: holding through command +
    // 1.5 s, where the speed is judged, covers both.
    const double t_speed = command_s + kAttitudeSpeedAtS;
    if (frames.back().t_s + eps < t_speed) {
        return {false, name + "absent: the frames end at t = " + num(frames.back().t_s) + " s, before " +
                           num(t_speed) + " s"};
    }
    float worst_roll = 0.0f;
    for (std::size_t k = ic; k < frames.size() && frames[k].t_s <= t_speed + eps; ++k) {
        const float roll = std::abs(roll_of(frames[k].attitude));
        worst_roll = std::max(worst_roll, roll);
        if (!(roll <= kAttitudeTolRad)) {
            return {false, name + "rolled " + num(roll * kDeg) + " degrees at t = " + num(frames[k].t_s) +
                               " s, past 3"};
        }
        const float off = std::abs(pitch_of(frames[k].attitude) - pitch_command_rad);
        if (k >= ir && !(off <= kAttitudeTolRad)) {
            return {false, name + "the pitch left the command by " + num(off * kDeg) + " degrees at t = " +
                               num(frames[k].t_s) + " s"};
        }
    }
    std::size_t is = ic;
    while (is < frames.size() && frames[is].t_s + eps < t_speed) ++is;
    const glm::vec3 dv = frames[is].velocity - frames[ic].velocity;
    const float along = glm::dot(glm::vec3(dv.x, 0.0f, dv.z), heading);
    const float gained = pitch_command_rad < 0.0f ? along : -along;  // nose down flies forward
    const bool ok = gained > kAttitudeSpeedGain;
    return {ok, name + "within 3 degrees of " + num(pitch_command_rad * kDeg) + " by t = " + num(frames[ir].t_s) +
                    " s and held; roll at most " + num(worst_roll * kDeg) + " degrees; gained " + num(gained) +
                    " m/s the way the tilt points by t = " + num(frames[is].t_s) + " s (more than " +
                    num(kAttitudeSpeedGain) + " needed)"};
}

// ---------------------------------------------------------------------------
// PLACING (§2, §3), measured by the step: the predicate judges the numbers.
// ---------------------------------------------------------------------------

struct Placement {
    std::size_t assets_before = 0;
    std::size_t assets_after = 0;
    bool name_unique = false;
    glm::vec3 pick{0.0f};               // where the click's ray met the surface
    glm::vec3 position{0.0f};           // the placed object's origin
    float rest_gap = 0.0f;              // m: its lowest point to the surface beneath, by the SDF without it
    std::size_t drawables_declared = 0;  // from its definition: collider primitives + CSG roots, parts, or meshes
    std::size_t drawables_added = 0;    // what the render scene gained
    float drawable_offset = 0.0f;       // m: the largest gap between a drawable and its pose
    float drawable_turn = 0.0f;         // rad: the largest turn between a drawable and its pose
    bool materials_match = false;
    float moved_after_run = 0.0f;       // m, over 1.0 s of Run
    float tilted_after_run = 0.0f;      // rad, over 1.0 s of Run
};

inline constexpr float kPlaceXZ = 1e-3f;
inline constexpr float kRestLo = -2e-3f;
inline constexpr float kRestHi = 5e-3f;
inline constexpr float kDrawOffset = 1e-5f;
inline constexpr float kDrawTurn = 1e-5f;
inline constexpr float kStillMove = 0.010f;
inline constexpr float kStillTilt = 0.03490658503988659f;  // 2 degrees

// Placed where asked, resting, and still after 1.0 s of Run.
[[nodiscard]] inline Verdict placed_at(const Placement& p) {
    using detail::num;
    const std::string name = "placed-at: ";
    if (!(detail::finite(p.pick) && detail::finite(p.position) && std::isfinite(p.rest_gap) &&
          std::isfinite(p.moved_after_run) && std::isfinite(p.tilted_after_run))) {
        return {false, name + "absent: a measurement is not finite"};
    }
    const float dx = std::abs(p.position.x - p.pick.x);
    const float dz = std::abs(p.position.z - p.pick.z);
    if (!(dx <= kPlaceXZ && dz <= kPlaceXZ)) {
        return {false, name + num(dx * 1000.0f) + " mm in x and " + num(dz * 1000.0f) +
                           " mm in z from the pick (1 at most)"};
    }
    const float off = std::max(dx, dz);
    if (!(p.rest_gap >= kRestLo && p.rest_gap <= kRestHi)) {
        return {false, name + (p.rest_gap < 0.0f ? "buried " : "floating ") + num(std::abs(p.rest_gap) * 1000.0f) +
                           " mm (a rest within [-2, +5] mm)"};
    }
    if (!(p.moved_after_run <= kStillMove && p.tilted_after_run <= kStillTilt)) {
        return {false, name + "moved " + num(p.moved_after_run * 1000.0f) + " mm and tilted " +
                           num(p.tilted_after_run * 57.29577951308232f) +
                           " degrees in 1.0 s of Run (10 mm and 2 at most)"};
    }
    return {true, name + num(off * 1000.0f) + " mm from the pick; resting " + num(p.rest_gap * 1000.0f) +
                      " mm above the surface; still in Run"};
}

// Exactly the drawables its definition declares, each at its pose.
[[nodiscard]] inline Verdict drawn_as_declared(const Placement& p) {
    using detail::num;
    const std::string name = "drawn: ";
    if (!(std::isfinite(p.drawable_offset) && std::isfinite(p.drawable_turn))) {
        return {false, name + "absent: a measurement is not finite"};
    }
    if (p.drawables_declared == 0) return {false, name + "mis-staged: its definition declares no drawables"};
    if (p.drawables_added != p.drawables_declared) {
        return {false, name + std::to_string(p.drawables_added) + " drawables added for " +
                           std::to_string(p.drawables_declared) + " declared"};
    }
    if (!(p.drawable_offset <= kDrawOffset && p.drawable_turn <= kDrawTurn)) {
        return {false, name + "a drawable is " + num(p.drawable_offset) + " m and " + num(p.drawable_turn) +
                           " rad off its pose (1e-5 at most)"};
    }
    return {true, name + std::to_string(p.drawables_added) + " drawables, each at its pose"};
}

// One new asset under a unique name, with its material, placed_at() and
// drawn_as_declared().
[[nodiscard]] inline Verdict object_added(const Placement& p) {
    const std::string name = "object-added: ";
    if (p.assets_after != p.assets_before + 1) {
        const long long added = static_cast<long long>(p.assets_after) - static_cast<long long>(p.assets_before);
        return {false, name + std::to_string(added) + " assets added for one place"};
    }
    if (!p.name_unique) return {false, name + "its name is another asset's"};
    if (!p.materials_match) return {false, name + "a drawable lacks the asset's material"};
    const Verdict at = placed_at(p);
    const Verdict drawn = drawn_as_declared(p);
    return {at.ok && drawn.ok, name + at.detail + "; " + drawn.detail};
}

// ---------------------------------------------------------------------------
// MEASURING A PLACED OBJECT (§2, §3, §7a): the numbers a Placement holds,
// measured here from raw inputs, not by the step (Interface's review: a step
// that measures leans on the code it checks).
// ---------------------------------------------------------------------------

// The drawables scene_from_world() makes from a collider, counted from the
// postfix program itself, not through the renderer's split_program() (TD-4):
// a primitive under nothing but union is one; a subtree rooted at subtract,
// intersect or smooth_union is one, whatever it holds. 0 for a program that
// is not well formed (an operator short of operands, or not one root).
[[nodiscard]] inline std::size_t declared_drawables(const SdfProgram& collider) {
    std::vector<std::size_t> stack;
    for (const SdfNode& node : collider.nodes) {
        const auto op = static_cast<SdfOp>(node.op);
        if (op == SdfOp::none) {
            stack.push_back(1);
            continue;
        }
        if (stack.size() < 2) return 0;
        const std::size_t b = stack.back();
        stack.pop_back();
        const std::size_t a = stack.back();
        stack.pop_back();
        stack.push_back(op == SdfOp::union_ ? a + b : 1);
    }
    return stack.size() == 1 ? stack.back() : 0;
}

// Every vertex of the given draw items, in world space: the points a placed
// object's rest is judged from. Empty when any item names a mesh the scene
// lacks: a missing part of the object must not shrink what is measured.
[[nodiscard]] inline std::vector<glm::vec3> posed_vertices(const render::RenderScene& scene,
                                                           std::span<const render::DrawItem> items) {
    std::vector<glm::vec3> points;
    for (const render::DrawItem& item : items) {
        if (item.mesh_index >= scene.meshes.size()) return {};
        for (const glm::vec3& v : scene.meshes[item.mesh_index].positions) {
            points.push_back(glm::vec3(item.local_to_world * glm::vec4(v, 1.0f)));
        }
    }
    return points;
}

// The gap from a placed object to what it rests on: the least distance, by
// the world's SDF WITHOUT the object, over the object's posed points.
// Negative: buried that deep. NaN when there are no points, or any point or
// distance is not finite. Precondition: the program validated.
//
// ITS BIAS, AND SO ITS PRECONDITIONS (Interface's review). The points are
// the drawables' vertices, which lie on the object's surface, so the gap
// reads HIGH, never low, by however far the true lowest point lies between
// vertices. It is exact for the shipped primitives posed upright (yawed
// only): their lowest points -- a sphere's pole, a box's or cylinder's
// bottom face and rim, a torus's underside -- are vertices. So the end-goal
// step places objects upright, with nothing else within 5 mm sideways (a
// least distance is not "beneath": a wall that close would read as the
// rest). It does not measure a tilted primitive (a 0.5 m sphere tilted 15
// degrees reads up to about 8 mm high, past the +5 mm band) or a CSG object,
// whose surface-nets mesh sits up to half a mesh cell inside the surface.
[[nodiscard]] inline float rest_gap(const SdfProgram& world_without_object, std::span<const glm::vec3> points) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    if (points.empty()) return nan;
    float least = std::numeric_limits<float>::infinity();
    for (const glm::vec3& p : points) {
        if (!detail::finite(p)) return nan;
        const float d = eval(world_without_object, p);
        if (!std::isfinite(d)) return nan;
        least = std::min(least, d);
    }
    return least;
}

// ---------------------------------------------------------------------------
// IMAGES (§1 thumbnails, §6 views). "Differs" is the doc's: at least 1% of
// the pixels differ by more than 8/255 in red, green or blue.
// ---------------------------------------------------------------------------

// An RGBA8 image, row-major, `stride` bytes from one row to the next (a view
// may be a region of a larger frame).
struct PixelsView {
    uint32_t width = 0;
    uint32_t height = 0;
    std::size_t stride = 0;
    std::span<const uint8_t> rgba;
};

using Rgb = std::array<uint8_t, 3>;

[[nodiscard]] inline PixelsView pixels(uint32_t width, uint32_t height, std::span<const uint8_t> rgba) {
    return {width, height, std::size_t{width} * 4u, rgba};
}

// The region [x, x + w) x [y, y + h) of `frame`, clipped to it.
[[nodiscard]] inline PixelsView region_of(const PixelsView& frame, uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    if (x >= frame.width || y >= frame.height) return {};
    w = std::min(w, frame.width - x);
    h = std::min(h, frame.height - y);
    const std::size_t offset = std::size_t{y} * frame.stride + std::size_t{x} * 4u;
    if (offset >= frame.rgba.size()) return {};
    return {w, h, frame.stride, frame.rgba.subspan(offset)};
}

[[nodiscard]] inline bool well_formed(const PixelsView& p) {
    if (p.width == 0 || p.height == 0 || p.stride < std::size_t{p.width} * 4u) return false;
    return p.rgba.size() >= (std::size_t{p.height} - 1u) * p.stride + std::size_t{p.width} * 4u;
}

[[nodiscard]] inline bool same_size(const PixelsView& a, const PixelsView& b) {
    return a.width == b.width && a.height == b.height;
}

inline constexpr int kDiffersLevel = 8;
inline constexpr double kDiffersFraction = 0.01;

namespace detail {

template <class Pixel>
[[nodiscard]] double fraction_off(const PixelsView& a, uint32_t w, uint32_t h, Pixel other) {
    if (w == 0 || h == 0) return 0.0;
    std::size_t off = 0;
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const uint8_t* pa = a.rgba.data() + std::size_t{y} * a.stride + std::size_t{x} * 4u;
            const Rgb pb = other(x, y);
            for (int c = 0; c < 3; ++c) {
                if (std::abs(int{pa[c]} - int{pb[static_cast<std::size_t>(c)]}) > kDiffersLevel) {
                    ++off;
                    break;
                }
            }
        }
    }
    return static_cast<double>(off) / (static_cast<double>(w) * h);
}

[[nodiscard]] inline std::string percent(double fraction) { return num(100.0 * fraction) + "%"; }

}  // namespace detail

// The fraction of pixels of `a` that differ from `b`'s. The verdicts below
// compare only images of one size (a size mismatch is mis-staged); this
// helper reads the size both share.
[[nodiscard]] inline double differ_fraction(const PixelsView& a, const PixelsView& b) {
    if (!well_formed(a) || !well_formed(b)) return 0.0;
    const auto at_b = [&](uint32_t x, uint32_t y) {
        const uint8_t* pb = b.rgba.data() + std::size_t{y} * b.stride + std::size_t{x} * 4u;
        return Rgb{pb[0], pb[1], pb[2]};
    };
    return detail::fraction_off(a, std::min(a.width, b.width), std::min(a.height, b.height), at_b);
}

// The fraction of pixels of `a` that differ from one colour.
[[nodiscard]] inline double differ_fraction(const PixelsView& a, Rgb colour) {
    if (!well_formed(a)) return 0.0;
    return detail::fraction_off(a, a.width, a.height, [&](uint32_t, uint32_t) { return colour; });
}

[[nodiscard]] inline bool differs(double fraction) { return fraction >= kDiffersFraction; }

// The image's most frequent colour.
[[nodiscard]] inline Rgb dominant_colour(const PixelsView& a) {
    if (!well_formed(a)) return {};
    std::vector<uint32_t> keys;
    keys.reserve(std::size_t{a.width} * a.height);
    for (uint32_t y = 0; y < a.height; ++y) {
        for (uint32_t x = 0; x < a.width; ++x) {
            const uint8_t* p = a.rgba.data() + std::size_t{y} * a.stride + std::size_t{x} * 4u;
            keys.push_back((uint32_t{p[0]} << 16) | (uint32_t{p[1]} << 8) | uint32_t{p[2]});
        }
    }
    std::sort(keys.begin(), keys.end());
    uint32_t best = keys.front();
    std::size_t best_run = 0;
    for (std::size_t i = 0; i < keys.size();) {
        std::size_t j = i;
        while (j < keys.size() && keys[j] == keys[i]) ++j;
        if (j - i > best_run) {
            best_run = j - i;
            best = keys[i];
        }
        i = j;
    }
    return Rgb{static_cast<uint8_t>(best >> 16), static_cast<uint8_t>(best >> 8), static_cast<uint8_t>(best)};
}

// ---------------------------------------------------------------------------
// THE LIBRARY (§1).
// ---------------------------------------------------------------------------

struct LibraryEntry {
    std::string name;
    std::string category;
    std::string file;
};

// §3's first entries: the quad, four primitives, a gate, a floor tile, a pole and a wall.
inline constexpr std::size_t kLibraryMinEntries = 9;

// The panel lists exactly the index's entries, in the index's order.
[[nodiscard]] inline Verdict lists_index(std::span<const LibraryEntry> index, std::span<const LibraryEntry> panel) {
    const std::string name = "lists-index: ";
    if (index.size() < kLibraryMinEntries) {
        return {false, name + "the index holds " + std::to_string(index.size()) + " entries, fewer than " +
                           std::to_string(kLibraryMinEntries)};
    }
    if (panel.size() != index.size()) {
        return {false, name + "the panel lists " + std::to_string(panel.size()) + " entries for the index's " +
                           std::to_string(index.size())};
    }
    for (std::size_t i = 0; i < index.size(); ++i) {
        const LibraryEntry& p = panel[i];
        const LibraryEntry& x = index[i];
        if (p.name != x.name || p.category != x.category || p.file != x.file) {
            return {false, name + "entry " + std::to_string(i) + " is '" + p.name + "' (" + p.category + ", " + p.file +
                               "); the index's is '" + x.name + "' (" + x.category + ", " + x.file + ")"};
        }
    }
    return {true, name + std::to_string(index.size()) + " entries, in the index's order"};
}

// Each entry shows the image decoded from ITS OWN thumbnail file: `shown[i]`
// is what the panel shows, `files[i]` the image the step decoded from entry
// i's file. Each pair must match (the same size, nothing differs), and the
// file's image must not be one colour (1% or more off its dominant colour).
// A placeholder, a border-framed placeholder or another entry's image fails.
[[nodiscard]] inline Verdict thumbnails(std::span<const PixelsView> shown, std::span<const PixelsView> files) {
    const std::string name = "thumbnails: ";
    if (files.empty()) return {false, name + "absent: no entries"};
    if (shown.size() != files.size()) {
        return {false, name + std::to_string(shown.size()) + " thumbnails shown for " + std::to_string(files.size()) +
                           " entries"};
    }
    for (std::size_t i = 0; i < files.size(); ++i) {
        const std::string at = "thumbnail " + std::to_string(i);
        if (!well_formed(files[i])) {
            return {false, name + "absent: entry " + std::to_string(i) + "'s file decoded to nothing"};
        }
        if (!well_formed(shown[i])) return {false, name + at + " shows nothing"};
        if (!same_size(shown[i], files[i])) {
            return {false, name + at + " is " + std::to_string(shown[i].width) + "x" + std::to_string(shown[i].height) +
                               "; its file's image is " + std::to_string(files[i].width) + "x" +
                               std::to_string(files[i].height)};
        }
        const double apart = differ_fraction(shown[i], files[i]);
        if (differs(apart)) {
            return {false, name + at + " is not its own file's image: " + detail::percent(apart) + " of it differs"};
        }
        const double varied = differ_fraction(files[i], dominant_colour(files[i]));
        if (!differs(varied)) {
            return {false, name + at + "'s file is one colour (" + detail::percent(varied) +
                               " off its dominant colour): a blank or a placeholder"};
        }
    }
    return {true, name + std::to_string(files.size()) + " thumbnails, each its own file's image"};
}

// ---------------------------------------------------------------------------
// IDENTITIES (§2 propulsion-kept, §7 round-trip).
// ---------------------------------------------------------------------------

struct Identity {
    std::string what;       // "model", "parts", "propulsion", "scene", "region", "drone 0 model"...
    uint64_t expected = 0;  // the source's: the library file's, or the scene's before saving
    uint64_t got = 0;       // the copy's: the placed model's, or the reopened scene's
};

// What propulsion-kept requires, for a placed drone.
inline constexpr std::array<std::string_view, 3> kDroneIdentities{"model", "parts", "propulsion"};

namespace detail {

[[nodiscard]] inline std::string hex(uint64_t v) {
    char buf[24];
    std::snprintf(buf, sizeof buf, "0x%016llx", static_cast<unsigned long long>(v));
    return buf;
}

}  // namespace detail

// Every REQUIRED identity is among `ids` and the copy's equals the source's.
// The step hands over what it measured; the required names are the check's,
// so a step that leaves one out is absent, not a pass. An expected 0 is
// unset: absent.
[[nodiscard]] inline Verdict identities_kept(std::string_view check, std::span<const Identity> ids,
                                             std::span<const std::string_view> required) {
    const std::string name = std::string(check) + ": ";
    if (required.empty()) return {false, name + "mis-staged: no identity is required"};
    std::string kept;
    for (const std::string_view want : required) {
        const auto it = std::find_if(ids.begin(), ids.end(), [&](const Identity& id) { return id.what == want; });
        if (it == ids.end()) return {false, name + "absent: no " + std::string(want) + " identity was given"};
        if (it->expected == 0) {
            return {false, name + "absent: the source's " + std::string(want) + " identity is unset (0)"};
        }
        if (it->got != it->expected) {
            return {false, name + "the " + std::string(want) + " identity is " + detail::hex(it->got) +
                               ", not the source's " + detail::hex(it->expected)};
        }
        kept += (kept.empty() ? "" : ", ") + std::string(want);
    }
    return {true, name + kept + " kept"};
}

// ---------------------------------------------------------------------------
// IMPORT (§3).
// ---------------------------------------------------------------------------

struct ImportShown {
    std::string shown_name;    // what the editor names the imported object
    std::string mesh_name;     // the glTF mesh's own name; empty when it has none
    std::string file;          // the path imported
    std::string shown_source;  // what the editor shows as its source
    bool sidecar_loaded = false;  // the .spmesh it wrote loads back
    std::size_t gltf_vertices = 0;
    std::size_t gltf_triangles = 0;
    std::size_t sidecar_vertices = 0;
    std::size_t sidecar_triangles = 0;
};

[[nodiscard]] inline Verdict import_named(const ImportShown& m) {
    const std::string name = "import-named: ";
    const std::string expected = m.mesh_name.empty() ? std::filesystem::path(m.file).stem().string() : m.mesh_name;
    if (expected.empty()) return {false, name + "absent: neither the mesh nor the file gives a name"};
    if (m.gltf_triangles == 0) return {false, name + "absent: read_gltf read no triangles"};
    if (m.shown_name != expected) return {false, name + "named '" + m.shown_name + "', not '" + expected + "'"};
    if (m.shown_source != m.file) {
        return {false, name + "its source shows '" + m.shown_source + "', not '" + m.file + "'"};
    }
    if (!m.sidecar_loaded) return {false, name + "its .spmesh sidecar does not load back"};
    if (m.sidecar_vertices != m.gltf_vertices || m.sidecar_triangles != m.gltf_triangles) {
        return {false, name + "the sidecar holds " + std::to_string(m.sidecar_vertices) + " vertices and " +
                           std::to_string(m.sidecar_triangles) + " triangles; read_gltf read " +
                           std::to_string(m.gltf_vertices) + " and " + std::to_string(m.gltf_triangles)};
    }
    return {true, name + "'" + expected + "' from " + m.file + "; the sidecar holds its " +
                      std::to_string(m.gltf_triangles) + " triangles"};
}

// ---------------------------------------------------------------------------
// THE REGION (§5): resolved, published, captioned.
// ---------------------------------------------------------------------------

struct GridNumbers {
    uint32_t nx = 0, ny = 0, nz = 0;
    float dx = 0.0f;
    float r_over_dx = 0.0f;
};

// What the inspector shows is LiveRun's resolve, number for number.
[[nodiscard]] inline Verdict region_resolved(const GridNumbers& shown, const GridNumbers& resolved) {
    const std::string name = "region-resolved: ";
    const auto text = [](const GridNumbers& g) {
        return std::to_string(g.nx) + "x" + std::to_string(g.ny) + "x" + std::to_string(g.nz) + ", dx " +
               detail::num(g.dx) + " m, R/dx " + detail::num(g.r_over_dx);
    };
    if (resolved.nx == 0 || resolved.ny == 0 || resolved.nz == 0 || !(resolved.dx > 0.0f)) {
        return {false, name + "absent: the resolve gave no grid"};
    }
    const bool same = shown.nx == resolved.nx && shown.ny == resolved.ny && shown.nz == resolved.nz &&
                      shown.dx == resolved.dx && shown.r_over_dx == resolved.r_over_dx;
    if (!same) return {false, name + "shows " + text(shown) + "; the resolve is " + text(resolved)};
    return {true, name + text(shown)};
}

struct Published {
    uint64_t generation = 0;
    bool valid = false;
    uint32_t no_data = 0;
};

[[nodiscard]] inline Published published(const FlowVolume& v) {
    return {v.generation, validate_flow_volume(v).has_value(), v.header.no_data};
}

// Every frame a new, valid volume with no no-data sample. New means a later
// generation than the frame before: next_flow_generation() only increases,
// so a repeat or a return to an older volume fails.
[[nodiscard]] inline Verdict publishes(std::span<const Published> frames) {
    const std::string name = "publishes: ";
    if (frames.empty()) return {false, name + "absent: no frames were published"};
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const Published& f = frames[i];
        const std::string at = "frame " + std::to_string(i);
        if (!f.valid) return {false, name + at + "'s volume fails validate_flow_volume()"};
        if (f.no_data != 0) return {false, name + at + " holds " + std::to_string(f.no_data) + " no-data samples"};
        if (f.generation == 0) return {false, name + at + "'s generation is 0 (unknown)"};
        if (i > 0 && !(f.generation > frames[i - 1].generation)) {
            return {false, name + at + "'s generation does not follow the previous frame's: not a new publish"};
        }
    }
    return {true, name + std::to_string(frames.size()) + " frames, each a new, valid volume"};
}

[[nodiscard]] inline Verdict run_line_shown(std::string_view shown, std::string_view expected) {
    const std::string name = "run-line-shown: ";
    if (expected.empty()) return {false, name + "absent: LiveRun gave no run line"};
    if (shown != expected) {
        return {false, name + "shows '" + std::string(shown) + "', not '" + std::string(expected) + "'"};
    }
    return {true, name + std::string(shown)};
}

// ---------------------------------------------------------------------------
// VIEWS (§6).
// ---------------------------------------------------------------------------

// "shaded", or "wireframe+vortex_iso+streamlines": the base and each layer's kind.
[[nodiscard]] inline std::string spec_name(const render::ViewSpec& spec) {
    std::string s(render::view_base_name(spec.base));
    for (const render::FlowViewSpec& layer : spec.layers) s += "+" + std::string(render::flow_view_name(layer.kind));
    return s;
}

struct ModeShown {
    std::string kind;           // the menu item
    bool supported = false;     // supports()'s answer for this view
    std::string why_not;        // supports()'s reason, when it refuses
    std::string shown_text;     // what the view shows instead, when refused
    bool applied = false;       // apply_view() was Ok
    bool drawn = false;         // draw() was Ok
    std::string unhonoured;     // unhonoured()'s text; empty when all was honoured
    PixelsView frame;           // this mode's view
    Rgb clear{0, 0, 0};         // the clear colour
};

// The step's modes, in the order switched. Each mode supports() accepts is
// drawn, honoured, not blank, and changed from the last mode drawn before it
// (the first mode drawn has none to change from: the check, not the step,
// decides which that is). A refused mode shows its why_not verbatim. Frames
// compared are one size, or the check is mis-staged.
[[nodiscard]] inline Verdict modes_drawn(std::span<const ModeShown> modes) {
    const std::string name = "each-mode-drawn: ";
    if (modes.empty()) return {false, name + "absent: no modes were switched"};
    const PixelsView* last = nullptr;
    std::size_t drawn_count = 0;
    for (const ModeShown& m : modes) {
        const std::string at = "(" + m.kind + ") ";
        if (!m.supported) {
            if (m.why_not.empty()) return {false, name + at + "mis-staged: refused with no why_not"};
            if (m.shown_text != m.why_not) {
                return {false, name + at + "shows '" + m.shown_text + "' for supports()'s why_not '" + m.why_not + "'"};
            }
            continue;
        }
        if (!m.applied) return {false, name + at + "apply_view() refused"};
        if (!m.drawn) return {false, name + at + "draw() failed"};
        if (!m.unhonoured.empty()) return {false, name + at + "unhonoured: " + m.unhonoured};
        if (!well_formed(m.frame)) return {false, name + at + "absent: no frame"};
        const double lit = differ_fraction(m.frame, m.clear);
        if (!differs(lit)) {
            return {false, name + at + "blank: " + detail::percent(lit) + " of the view differs from the clear colour"};
        }
        if (last != nullptr) {
            if (!same_size(m.frame, *last)) {
                return {false, name + at + "mis-staged: its frame is not the last one's size"};
            }
            const double changed = differ_fraction(m.frame, *last);
            if (!differs(changed)) {
                return {false, name + at + "unchanged from the last mode drawn: " + detail::percent(changed) +
                                   " of the view differs"};
            }
        }
        last = &m.frame;
        ++drawn_count;
    }
    return {true, name + std::to_string(drawn_count) + " drawn, each changed from the last; " +
                      std::to_string(modes.size() - drawn_count) + " refused with their why_not"};
}

struct LegendCheck {
    std::size_t layers = 0;                    // the view spec's layer count: apply_view returns one legend each
    std::vector<render::ViewLegend> returned;  // apply_view()'s
    std::vector<render::ViewLegend> shown;     // the editor's
    bool region_running = false;
};

inline constexpr float kSpeedLegendMin = 1.0f;  // m/s: a speed legend's max while the region runs

[[nodiscard]] inline Verdict legend_shown(const LegendCheck& c) {
    const std::string name = "legend-shown: ";
    if (c.returned.size() != c.layers) {
        return {false, name + "absent: apply_view() returned " + std::to_string(c.returned.size()) +
                           " legends for the spec's " + std::to_string(c.layers) + " layers"};
    }
    if (c.shown.size() != c.returned.size()) {
        return {false, name + std::to_string(c.shown.size()) + " legends shown for " +
                           std::to_string(c.returned.size()) + " layers"};
    }
    for (std::size_t i = 0; i < c.shown.size(); ++i) {
        const render::ViewLegend& s = c.shown[i];
        const render::ViewLegend& r = c.returned[i];
        const std::string at = "legend " + std::to_string(i) + " (" + r.quantity + ")";
        if (s.layer != r.layer || s.quantity != r.quantity || s.unit != r.unit || s.colours != r.colours ||
            s.range.min != r.range.min || s.range.max != r.range.max || s.range.from_header != r.range.from_header) {
            return {false, name + at + " is not the one apply_view() returned"};
        }
        if (c.region_running) {
            // Only the frame's own range can show still air: a fixed range
            // reads the same over a moving field and a still one.
            if (!r.range.from_header) {
                return {false, name + "mis-staged: " + at + "'s range is fixed, so it cannot show whether air moves"};
            }
            if (!(r.range.min < r.range.max)) {
                return {false, name + at + "'s range [" + detail::num(r.range.min) + ", " + detail::num(r.range.max) +
                                   "] is flat: a still field"};
            }
            if (r.quantity.rfind("speed", 0) == 0 && !(r.range.max >= kSpeedLegendMin)) {
                return {false, name + at + " tops out at " + detail::num(r.range.max) + " m/s, under " +
                                   detail::num(kSpeedLegendMin) + ": still air"};
            }
        }
    }
    return {true, name + std::to_string(c.shown.size()) + " legends, each apply_view()'s"};
}

// After Split: two views with different specs, each drawn, and different.
[[nodiscard]] inline Verdict two_views(std::string_view left_spec, std::string_view right_spec,
                                       const PixelsView& left, const PixelsView& right, Rgb clear) {
    const std::string name = "two-views: ";
    if (left_spec == right_spec) return {false, name + "both views are " + std::string(left_spec)};
    if (!well_formed(left) || !well_formed(right)) return {false, name + "absent: a view has no pixels"};
    if (!same_size(left, right)) return {false, name + "mis-staged: the two views are not one size"};
    const double l = differ_fraction(left, clear);
    const double r = differ_fraction(right, clear);
    if (!differs(l) || !differs(r)) {
        return {false, name + "a view is blank: " + detail::percent(l) + " and " + detail::percent(r) +
                           " off the clear colour"};
    }
    const double apart = differ_fraction(left, right);
    if (!differs(apart)) return {false, name + "the two views match: " + detail::percent(apart) + " differs"};
    return {true, name + std::string(left_spec) + " | " + std::string(right_spec) + "; " + detail::percent(apart) +
                      " of them differs"};
}

// ---------------------------------------------------------------------------
// SAVE AND REOPEN (§7).
// ---------------------------------------------------------------------------

inline constexpr uint32_t kRoundTripSteps = 60;

struct RoundTrip {
    std::string saved;       // the file's bytes
    std::string reemitted;   // the reopened scene, emitted again
    std::size_t drones = 0;  // the scene's drones: each one's model, parts and propulsion must be kept
    std::vector<Identity> identities;  // "scene", "region", and "drone <i> model|parts|propulsion"
    uint32_t steps_before = 0;         // Run steps from the scene before saving
    uint32_t steps_after = 0;          // and from the reopened one
    std::vector<BodyState> bodies_before;  // every body after those steps
    std::vector<BodyState> bodies_after;
};

// The identity names a round trip requires for `drones` drones.
[[nodiscard]] inline std::vector<std::string> round_trip_identities(std::size_t drones) {
    std::vector<std::string> names = {"scene", "region"};
    for (std::size_t i = 0; i < drones; ++i) {
        for (const std::string_view what : kDroneIdentities) {
            names.push_back("drone " + std::to_string(i) + " " + std::string(what));
        }
    }
    return names;
}

namespace detail {

template <class V>
[[nodiscard]] bool same_bits(const V& a, const V& b, int n) {
    for (int i = 0; i < n; ++i) {
        if (std::bit_cast<uint32_t>(a[i]) != std::bit_cast<uint32_t>(b[i])) return false;
    }
    return true;
}

[[nodiscard]] inline bool same_bits(float a, float b) {
    return std::bit_cast<uint32_t>(a) == std::bit_cast<uint32_t>(b);
}

// Every named field of the body, bit for bit; only the std430 pads are left out.
[[nodiscard]] inline bool same_state(const BodyState& a, const BodyState& b) {
    return same_bits(a.pos, b.pos, 3) && same_bits(a.proxy_radius, b.proxy_radius) &&
           same_bits(a.orient, b.orient, 4) && same_bits(a.vel, b.vel, 3) && same_bits(a.mass, b.mass) &&
           same_bits(a.omega_body, b.omega_body, 3) && same_bits(a.inv_inertia_diag, b.inv_inertia_diag, 3) &&
           a.flags == b.flags && same_bits(a.force_acc, b.force_acc, 3) && same_bits(a.torque_acc, b.torque_acc, 3) &&
           same_bits(a.specific_force, b.specific_force, 3);
}

}  // namespace detail

[[nodiscard]] inline Verdict round_trip(const RoundTrip& t) {
    const std::string name = "round-trip: ";
    if (t.saved.empty()) return {false, name + "absent: nothing was saved"};
    if (t.drones == 0) return {false, name + "mis-staged: the scene saved holds no drone"};
    if (t.reemitted != t.saved) {
        std::size_t at = 0;
        while (at < t.saved.size() && at < t.reemitted.size() && t.saved[at] == t.reemitted[at]) ++at;
        return {false, name + "the reopened scene emits differently from byte " + std::to_string(at) + " (" +
                           std::to_string(t.saved.size()) + " bytes saved, " + std::to_string(t.reemitted.size()) +
                           " emitted)"};
    }
    const std::vector<std::string> names = round_trip_identities(t.drones);
    const std::vector<std::string_view> required(names.begin(), names.end());
    if (const Verdict ids = identities_kept("round-trip", t.identities, required); !ids.ok) return ids;
    if (t.steps_before != kRoundTripSteps || t.steps_after != kRoundTripSteps) {
        return {false, name + "mis-staged: ran " + std::to_string(t.steps_before) + " and " +
                           std::to_string(t.steps_after) + " steps, not " + std::to_string(kRoundTripSteps)};
    }
    if (t.bodies_before.empty()) return {false, name + "absent: no bodies after the steps"};
    if (t.bodies_after.size() != t.bodies_before.size()) {
        return {false, name + "the reopened scene has " + std::to_string(t.bodies_after.size()) + " bodies, not " +
                           std::to_string(t.bodies_before.size())};
    }
    for (std::size_t i = 0; i < t.bodies_before.size(); ++i) {
        if (!detail::same_state(t.bodies_before[i], t.bodies_after[i])) {
            return {false, name + "body " + std::to_string(i) + " differs after " + std::to_string(kRoundTripSteps) +
                               " steps from the reopened scene"};
        }
    }
    return {true, name + std::to_string(t.saved.size()) + " bytes re-emitted exactly; identities kept; " +
                      std::to_string(t.bodies_before.size()) + " bodies bit-identical after " +
                      std::to_string(kRoundTripSteps) + " steps"};
}

}  // namespace spade::sandbox::end_goal
