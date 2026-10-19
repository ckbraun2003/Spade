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
//   "absent:"         the frames don't cover what the check needs (TD-5);
//   "mis-staged:"     the step didn't set up what the check presumes;
//   "under-resolved:" the region is too coarse for the check to mean anything.
// None of these passes.
//
// Owned by Test/Docs; reviewed by Interface.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
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

// The downward axial velocity averaged over the probe disc, m/s; NaN when any
// lattice point has no data. Precondition: the sampler's volume validated.
[[nodiscard]] inline float probe_downwash(const render::FlowSampler& sampler, const RotorProbe& r) {
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
            const render::FlowVelocity v = render::sample_velocity(sampler, p);
            if (v.no_data) return std::numeric_limits<float>::quiet_NaN();
            sum += -static_cast<double>(glm::dot(n, v.u));
            ++count;
        }
    }
    return static_cast<float>(sum / count);
}

// One published frame, as the downwash checks read it.
struct DownwashFrame {
    double t_s = 0.0;                // simulated time since arming
    float cell = 0.0f;               // the published volume's dx, m
    std::vector<RotorProbe> rotors;  // posed this frame
    std::vector<float> u_down;       // per rotor: probe_downwash(), NaN for no data
    float tilt_rad = 0.0f;           // tilt_of() the body
    bool controls_constant = true;   // the controls are still the ones set at arming
};

// Reads one published frame. An invalid volume reads as no data everywhere.
[[nodiscard]] inline DownwashFrame sample_downwash(const FlowVolume& volume, std::vector<RotorProbe> rotors,
                                                   double t_s, float tilt_rad, bool controls_constant) {
    DownwashFrame f;
    f.t_s = t_s;
    f.cell = volume.header.cell;
    f.tilt_rad = tilt_rad;
    f.controls_constant = controls_constant;
    f.u_down.assign(rotors.size(), std::numeric_limits<float>::quiet_NaN());
    if (validate_flow_volume(volume).has_value()) {
        const render::FlowSampler sampler = render::make_flow_sampler(volume);
        for (std::size_t i = 0; i < rotors.size(); ++i) f.u_down[i] = probe_downwash(sampler, rotors[i]);
    }
    f.rotors = std::move(rotors);
    return f;
}

struct DownwashCase {
    float density = 1.225f;      // kg/m^3, the case's
    float mass = 0.0f;           // kg, the model's
    float gravity = 9.80665f;    // m/s^2
    float hub_above_floor = 0.0f;  // L: the hub plane above the box's bottom face, m
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

[[nodiscard]] inline Verdict downwash(std::span<const DownwashFrame> frames, const DownwashCase& c,
                                      const DownwashRule& rule) {
    (void)frames;
    (void)c;
    return {true, std::string(rule.name) + ": not yet judged"};
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

// Before arming every rotor is stopped; 0.5 s after it every rotor turns at
// 90% of its idle command or more; over 1.0 s from arming the height changes
// by 5 mm or less. `idle_command` is each rotor's commanded idle speed.
[[nodiscard]] inline Verdict armed(std::span<const FlightFrame> frames, double arm_s,
                                   std::span<const float> idle_command) {
    (void)frames;
    (void)arm_s;
    (void)idle_command;
    return {true, "armed: not yet judged"};
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
    (void)frames;
    (void)throttle_up_s;
    (void)mass;
    (void)gravity;
    (void)radius;
    return {true, "climbs: not yet judged"};
}

inline constexpr double kAttitudeReachS = 1.0;
inline constexpr double kAttitudeHoldS = 0.5;
inline constexpr double kAttitudeSpeedAtS = 1.5;
inline constexpr float kAttitudeTolRad = 0.05235987755982988f;  // 3 degrees
inline constexpr float kAttitudeSpeedGain = 0.5f;               // m/s along the heading, in the tilt's direction

// After a pitch command at command_s, the pitch reaches the command to 3
// degrees within 1.0 s and holds 0.5 s; roll stays within 3 degrees; and by
// 1.5 s the horizontal speed along the heading has gained 0.5 m/s the way the
// tilt points (nose down: forward).
[[nodiscard]] inline Verdict follows_attitude(std::span<const FlightFrame> frames, double command_s,
                                              float pitch_command_rad) {
    (void)frames;
    (void)command_s;
    (void)pitch_command_rad;
    return {true, "follows-attitude: not yet judged"};
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
    std::size_t drawables_declared = 0; // from its definition (collider primitives + CSG roots, parts, or meshes)
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
    (void)p;
    return {true, "placed-at: not yet judged"};
}

// Exactly the drawables its definition declares, each at its pose.
[[nodiscard]] inline Verdict drawn_as_declared(const Placement& p) {
    (void)p;
    return {true, "drawn: not yet judged"};
}

// One new asset under a unique name, placed_at() and drawn_as_declared().
[[nodiscard]] inline Verdict object_added(const Placement& p) {
    (void)p;
    return {true, "object-added: not yet judged"};
}

}  // namespace spade::sandbox::end_goal
