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
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/flow_volume.hpp"
#include "render/flow_sample.hpp"
#include "state/layout.hpp"
#include "vehicles/rotor.hpp"

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
            if (glm::any(glm::lessThan(p, box_lo)) || glm::any(glm::greaterThan(p, box_hi))) {
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
    const bool ok = !(moved > kArmStillM);
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
    const double t_speed = command_s + kAttitudeSpeedAtS;
    const double t_hold_end = std::max(frames[ir].t_s + kAttitudeHoldS, t_speed);
    const double t_need = t_hold_end;
    if (frames.back().t_s + eps < t_need) {
        return {false, name + "absent: the frames end at t = " + num(frames.back().t_s) + " s, before " +
                           num(t_need) + " s"};
    }
    float worst_roll = 0.0f;
    for (std::size_t k = ic; k < frames.size() && frames[k].t_s <= t_need + eps; ++k) {
        const float roll = std::abs(roll_of(frames[k].attitude));
        worst_roll = std::max(worst_roll, roll);
        if (!(roll <= kAttitudeTolRad)) {
            return {false, name + "rolled " + num(roll * kDeg) + " degrees at t = " + num(frames[k].t_s) +
                               " s, past 3"};
        }
        const float off = std::abs(pitch_of(frames[k].attitude) - pitch_command_rad);
        if (k >= ir && frames[k].t_s <= t_hold_end + eps && !(off <= kAttitudeTolRad)) {
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

}  // namespace spade::sandbox::end_goal
