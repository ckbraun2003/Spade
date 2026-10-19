// The end-goal smoke's checks (N20, docs/design/test-docs/2026-10-19-end-goal-
// checks.md §8): each one fails on a stub of the fault it exists to catch, and
// passes on a stub of the right state. The stubs are the reference quad's
// numbers (tests/reference_airframe.hpp: 0.526 kg, arms 0.11 m, R = 76.2 mm)
// written out here, so a stub can't move with the model.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../sandbox/end_goal_checks.hpp"
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"

namespace eg = spade::sandbox::end_goal;

namespace {

constexpr float kMass = 0.526f;
constexpr float kG = 9.80665f;
constexpr float kRho = 1.225f;
constexpr float kR = 0.0762f;
constexpr float kArm = 0.11f;
constexpr float kHubY = 0.024f;     // the hub plane above the centre of mass
constexpr float kHalfBox = 0.25f;   // the stub volumes span [-0.25, 0.25]^3
constexpr float kHover = kMass * kG / 4.0f;
constexpr float kDeg = 0.017453292519943295f;

[[nodiscard]] bool starts_with(const std::string& s, const std::string& prefix) { return s.rfind(prefix, 0) == 0; }
[[nodiscard]] bool has(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }

// ---------------------------------------------------------------------------
// Downwash stubs
// ---------------------------------------------------------------------------

[[nodiscard]] std::vector<eg::RotorProbe> quad(float thrust_each = kHover) {
    const glm::vec3 hubs[4] = {{kArm, kHubY, 0.0f}, {0.0f, kHubY, kArm}, {-kArm, kHubY, 0.0f}, {0.0f, kHubY, -kArm}};
    std::vector<eg::RotorProbe> rotors;
    for (const glm::vec3& h : hubs) rotors.push_back({h, {0.0f, 1.0f, 0.0f}, kR, thrust_each});
    return rotors;
}

[[nodiscard]] std::vector<float> speeds(const std::vector<eg::RotorProbe>& rotors, float ratio) {
    std::vector<float> s;
    for (const eg::RotorProbe& r : rotors) s.push_back(ratio * eg::momentum_theory_2r(r.thrust, r.radius, kRho));
    return s;
}

// A volume over the stub box at `cell`. Inside a rotor's jet -- a top-hat of
// radius jet_r R below its hub, about its axis moved by `shift` -- the air
// goes down the axis at speed[i]; elsewhere it is still. Where jets would
// meet, a cell takes the nearest axis's.
[[nodiscard]] spade::FlowVolume jets(float cell, const std::vector<eg::RotorProbe>& rotors,
                                     const std::vector<float>& speed, float jet_r, glm::vec3 shift = glm::vec3(0.0f)) {
    spade::FlowVolume vol;
    const auto n = static_cast<uint32_t>(std::lround(2.0f * kHalfBox / cell));
    vol.header.origin = glm::vec3(-kHalfBox);
    vol.header.cell = cell;
    vol.header.nx = vol.header.ny = vol.header.nz = n;
    const std::size_t halves = std::size_t{n} * n * n * spade::kFlowHalvesPerCell;
    vol.v.assign(halves, 0u);
    vol.s.assign(halves, 0u);
    glm::vec3 lo(0.0f), hi(0.0f);
    for (uint32_t k = 0; k < n; ++k) {
        for (uint32_t j = 0; j < n; ++j) {
            for (uint32_t i = 0; i < n; ++i) {
                const glm::vec3 p = vol.header.origin + (glm::vec3(i, j, k) + 0.5f) * cell;
                glm::vec3 vel(0.0f);
                float nearest = jet_r * kR;
                for (std::size_t r = 0; r < rotors.size(); ++r) {
                    const glm::vec3 rel = p - (rotors[r].hub + shift);
                    const float along = glm::dot(rel, rotors[r].axis);
                    if (along > 0.0f) continue;
                    const float radial = glm::length(rel - along * rotors[r].axis);
                    if (radial <= nearest) {
                        nearest = radial;
                        vel = -rotors[r].axis * speed[r];
                    }
                }
                const std::size_t cell_index = spade::flow_cell_index(vol.header, i, j, k);
                for (int c = 0; c < 3; ++c) {
                    vol.v[cell_index * spade::kFlowHalvesPerCell + c] = spade::fp16::from_f32(vel[c]).bits;
                }
                lo = glm::min(lo, vel);
                hi = glm::max(hi, vel);
            }
        }
    }
    for (int c = 0; c < 3; ++c) {
        vol.header.channel_min[c] = lo[c];
        vol.header.channel_max[c] = hi[c];
    }
    vol.generation = spade::next_flow_generation();
    return vol;
}

// Every frame at 30 fps from arming to `until_s`, all reading `vol`.
[[nodiscard]] std::vector<eg::DownwashFrame> frames_of(const spade::FlowVolume& vol,
                                                       const std::vector<eg::RotorProbe>& rotors,
                                                       double until_s = 0.67, float tilt = 0.0f) {
    std::vector<eg::DownwashFrame> frames;
    for (int k = 0; k / 30.0 <= until_s + 1e-12; ++k) {
        frames.push_back(eg::sample_downwash(vol, rotors, k / 30.0, tilt, true));
    }
    return frames;
}

[[nodiscard]] eg::DownwashCase stub_case() { return {kRho, kMass, kG, kHalfBox - kHubY}; }

constexpr float kCell64 = 0.0125f;  // R/dx 6.1, the headless region's
constexpr float kCell32 = 0.025f;   // R/dx 3.05, the live region's

// ---------------------------------------------------------------------------
// Flight stubs
// ---------------------------------------------------------------------------

// A climb from rest on the ground (height 0): the vertical thrust is half the
// weight until throttle_up_s, then `thrust`; after that the centre of mass
// accelerates at accel(thrust, v). Integrated at 1/240 s, recorded at 30 fps.
[[nodiscard]] std::vector<eg::FlightFrame> climb(float thrust, const std::function<float(float, float)>& accel,
                                                 double throttle_up_s = 0.1, double until_s = 2.5) {
    std::vector<eg::FlightFrame> frames;
    double h = 0.0, v = 0.0;
    const int sub = 8;
    const double dt = 1.0 / (30.0 * sub);
    for (int k = 0; k / 30.0 <= until_s + 1e-12; ++k) {
        const double t = k / 30.0;
        const bool up = t + 1e-12 >= throttle_up_s;
        eg::FlightFrame f;
        f.t_s = t;
        f.height = static_cast<float>(h);
        f.velocity = glm::vec3(0.0f, static_cast<float>(v), 0.0f);
        f.thrust_up = up ? thrust : 0.5f * kMass * kG;
        frames.push_back(f);
        for (int s = 0; s < sub; ++s) {
            const double a = up ? accel(thrust, static_cast<float>(v)) : 0.0;
            v += a * dt;
            h += v * dt;
            if (h < 0.0) {
                h = 0.0;
                v = 0.0;
            }
        }
    }
    return frames;
}

[[nodiscard]] glm::quat attitude(float pitch, float roll) {
    return glm::angleAxis(pitch, glm::vec3(0.0f, 0.0f, 1.0f)) * glm::angleAxis(roll, glm::vec3(1.0f, 0.0f, 0.0f));
}

// Level until command_s, then the pitch eases to `command` on time constant
// tau (roll eases to `roll` alike); the forward acceleration is
// gain * g tan(-pitch) along +X (the heading), so nose down flies forward.
[[nodiscard]] std::vector<eg::FlightFrame> pitch_history(float command, float tau, float gain, float roll = 0.0f,
                                                         double command_s = 0.5, double until_s = 3.0) {
    std::vector<eg::FlightFrame> frames;
    double vx = 0.0;
    for (int k = 0; k / 30.0 <= until_s + 1e-12; ++k) {
        const double t = k / 30.0;
        const double ease = t < command_s ? 0.0 : 1.0 - std::exp(-(t - command_s) / tau);
        const auto pitch = static_cast<float>(command * ease);
        eg::FlightFrame f;
        f.t_s = t;
        f.height = 2.0f;
        f.attitude = attitude(pitch, static_cast<float>(roll * ease));
        f.velocity = glm::vec3(static_cast<float>(vx), 0.0f, 0.0f);
        f.thrust_up = kMass * kG;
        frames.push_back(f);
        vx += gain * kG * std::tan(-pitch) / 30.0;
    }
    return frames;
}

// Stopped until arm_s (or spinning, for the fault), then every rotor eases to
// `idle` on time constant tau; the height gains lift_m linearly over 1 s.
[[nodiscard]] std::vector<eg::FlightFrame> arming(double arm_s, float idle, float tau, float lift_m = 0.0f,
                                                  bool spinning_before = false, double until_s = 2.0) {
    std::vector<eg::FlightFrame> frames;
    for (int k = 0; k / 30.0 <= until_s + 1e-12; ++k) {
        const double t = k / 30.0;
        eg::FlightFrame f;
        f.t_s = t;
        const float w = t < arm_s ? (spinning_before ? 50.0f : 0.0f)
                                  : static_cast<float>(idle * (1.0 - std::exp(-(t - arm_s) / tau)));
        f.rotor_omega.assign(4, w);
        f.height = static_cast<float>(lift_m * std::clamp((t - arm_s) / 1.0, 0.0, 1.0));
        frames.push_back(f);
    }
    return frames;
}

[[nodiscard]] eg::Placement good_box() {
    eg::Placement p;
    p.assets_before = 3;
    p.assets_after = 4;
    p.name_unique = true;
    p.pick = glm::vec3(1.0f, 0.0f, 2.0f);
    p.position = glm::vec3(1.0f, 0.5f, 2.0f);
    p.rest_gap = 0.0f;
    p.drawables_declared = 1;
    p.drawables_added = 1;
    p.materials_match = true;
    return p;
}

}  // namespace

// ===========================================================================
// downwash: the headless band, 64^3
// ===========================================================================

// Known answers, by hand: C8's own record (T = 2 rho A v_h^2 with v_h 4 m/s, R
// 0.1 m: 7.5777 m/s), and the reference quad at hover thrust (1.2896 N, R
// 76.2 mm: v_h 5.3717 m/s, so 10.176 m/s).
TEST(SandboxEndGoalDownwash, TheReferenceIsMomentumTheoryTwoRadiiBelow) {
    const float c8_thrust = 2.0f * kRho * 3.14159265f * 0.01f * 16.0f;
    EXPECT_NEAR(eg::momentum_theory_2r(c8_thrust, 0.1f, kRho), 7.5777f, 1e-3f);
    EXPECT_NEAR(eg::momentum_theory_2r(kHover, kR, kRho), 10.176f, 2e-3f);
    EXPECT_EQ(eg::momentum_theory_2r(0.0f, kR, kRho), 0.0f);
}

TEST(SandboxEndGoalDownwash, PassesMomentumTheorysJets) {
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, 1.0f), 0.73f);
    const auto frames = frames_of(vol, rotors);
    const eg::Verdict v = eg::downwash(frames, stub_case(), eg::kDownwash);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalDownwash, FailsStillAir) {
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, 0.0f), 0.73f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalDownwash, FailsReversedFlow) {
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, -1.0f), 0.73f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalDownwash, FailsJetsMisplacedByOneRadius) {
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, 1.0f), 0.73f, glm::vec3(kR, 0.0f, 0.0f));
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalDownwash, FailsWithOneRotorsJetMissingAndNamesIt) {
    const auto rotors = quad();
    std::vector<float> s = speeds(rotors, 1.0f);
    s[2] = 0.0f;
    const spade::FlowVolume vol = jets(kCell64, rotors, s, 0.73f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "rotor 2")) << v.detail;
}

TEST(SandboxEndGoalDownwash, FailsAThrustDoubledInTheAir) {
    // The air carries twice the thrust the rotors report: sqrt 2 = 1.41 x v_ref.
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, std::sqrt(2.0f)), 0.73f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalDownwash, IsMisStagedWhenTheRotorsCarryUnderHalfTheWeight) {
    // Jets that match their (tiny) thrust: still air would pass a near-zero reference.
    const auto rotors = quad(0.1f * kHover);
    const spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, 1.0f), 0.73f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "mis-staged")) << v.detail;
}

TEST(SandboxEndGoalDownwash, IsUnderResolvedBelowFourCellsPerRadius) {
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell32, rotors, speeds(rotors, 1.0f), 0.73f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "under-resolved")) << v.detail;
}

TEST(SandboxEndGoalDownwash, IsAbsentWithFewerThanSixFramesInTheWindow) {
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, 1.0f), 0.73f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors, 0.40), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "absent")) << v.detail;
}

TEST(SandboxEndGoalDownwash, IsMisStagedWhenTheControlsChange) {
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, 1.0f), 0.73f);
    auto frames = frames_of(vol, rotors);
    frames[5].controls_constant = false;
    const eg::Verdict v = eg::downwash(frames, stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "mis-staged")) << v.detail;
}

TEST(SandboxEndGoalDownwash, IsMisStagedWhenTiltedPastOneDegree) {
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, 1.0f), 0.73f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors, 0.67, 2.0f * kDeg), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "mis-staged")) << v.detail;
}

TEST(SandboxEndGoalDownwash, FailsWhereAProbeReadsNoData) {
    const auto rotors = quad();
    spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, 1.0f), 0.73f);
    // Rotor 0's probe centre, (0.11, 0.024 - 2R, 0): its cell's u goes no-data.
    const glm::vec3 c = rotors[0].hub - 2.0f * kR * rotors[0].axis;
    const glm::uvec3 ijk = glm::uvec3((c - vol.header.origin) / vol.header.cell);
    vol.v[spade::flow_cell_index(vol.header, ijk.x, ijk.y, ijk.z) * spade::kFlowHalvesPerCell] =
        spade::fp16::kNoDataBits;
    vol.header.no_data = 1;
    vol.generation = spade::next_flow_generation();
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalDownwash, TheFloorIsNotTheBand) {
    // The live floor's smeared jets, at 0.43 v_ref, fail the headless band.
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell64, rotors, speeds(rotors, 0.43f), 1.7f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwash);
    EXPECT_FALSE(v.ok) << v.detail;
}

// ===========================================================================
// downwash-present: the live floor, 32^3
// ===========================================================================

TEST(SandboxEndGoalDownwashPresent, PassesSmearedJetsAtThreeCellsPerRadius) {
    // The R/dx 2 estimate: the slipstream widened to 1.7R, so 0.43 v_ref.
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell32, rotors, speeds(rotors, 0.43f), 1.7f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwashPresent);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalDownwashPresent, FailsStillAir) {
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(kCell32, rotors, speeds(rotors, 0.0f), 1.7f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwashPresent);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalDownwashPresent, FailsOneRotorReversed) {
    const auto rotors = quad();
    std::vector<float> s = speeds(rotors, 0.43f);
    s[1] = -s[1];
    const spade::FlowVolume vol = jets(kCell32, rotors, s, 1.7f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwashPresent);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "rotor 1")) << v.detail;
}

TEST(SandboxEndGoalDownwashPresent, FailsOneRotorStopped) {
    const auto rotors = quad();
    std::vector<float> s = speeds(rotors, 0.43f);
    s[3] = 0.0f;
    const spade::FlowVolume vol = jets(kCell32, rotors, s, 1.7f);
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwashPresent);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalDownwashPresent, IsUnderResolvedBelowTwoCellsPerRadius) {
    const auto rotors = quad();
    const spade::FlowVolume vol = jets(0.05f, rotors, speeds(rotors, 0.43f), 1.7f);  // R/dx 1.52
    const eg::Verdict v = eg::downwash(frames_of(vol, rotors), stub_case(), eg::kDownwashPresent);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "under-resolved")) << v.detail;
}

// ===========================================================================
// climbs
// ===========================================================================

TEST(SandboxEndGoalClimbs, PassesDragFreeKinematicsOfTheReportedThrust) {
    const float f = 1.2f * kMass * kG;
    const auto frames = climb(f, [](float thrust, float) { return thrust / kMass - kG; });
    const eg::Verdict v = eg::climbs(frames, 0.1, kMass, kG, kR);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalClimbs, PassesTheInflowClosuresLoss) {
    // 0.7 of the static prediction: the rotor model's lambda in climb (doc §4).
    const float f = 1.2f * kMass * kG;
    const auto frames = climb(f, [](float thrust, float) { return 0.7f * (thrust / kMass - kG); });
    const eg::Verdict v = eg::climbs(frames, 0.1, kMass, kG, kR);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalClimbs, FailsAPinnedDrone) {
    const float f = 1.2f * kMass * kG;
    const auto frames = climb(f, [](float, float) { return 0.0f; });
    const eg::Verdict v = eg::climbs(frames, 0.1, kMass, kG, kR);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_FALSE(starts_with(v.detail, "climbs: mis-staged")) << "the premise was met: this is a real fail";
}

TEST(SandboxEndGoalClimbs, FailsWithGravityMissing) {
    const float f = 1.2f * kMass * kG;
    const auto frames = climb(f, [](float thrust, float) { return thrust / kMass; });
    const eg::Verdict v = eg::climbs(frames, 0.1, kMass, kG, kR);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalClimbs, FailsThrustAppliedTwice) {
    const float f = 1.2f * kMass * kG;
    const auto frames = climb(f, [](float thrust, float) { return 2.0f * thrust / kMass - kG; });
    const eg::Verdict v = eg::climbs(frames, 0.1, kMass, kG, kR);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalClimbs, IsMisStagedBelowTheThrustPremise) {
    const float f = 1.05f * kMass * kG;
    const auto frames = climb(f, [](float thrust, float) { return thrust / kMass - kG; });
    const eg::Verdict v = eg::climbs(frames, 0.1, kMass, kG, kR);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "mis-staged")) << v.detail;
}

TEST(SandboxEndGoalClimbs, IsAbsentWhenTheFramesEndInsideTheWindow) {
    const float f = 1.2f * kMass * kG;
    const auto frames = climb(f, [](float thrust, float) { return thrust / kMass - kG; }, 0.1, 0.9);
    const eg::Verdict v = eg::climbs(frames, 0.1, kMass, kG, kR);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "absent")) << v.detail;
}

// ===========================================================================
// follows-attitude
// ===========================================================================

TEST(SandboxEndGoalFollowsAttitude, PassesATiltReachedAndFlownForward) {
    const auto frames = pitch_history(-15.0f * kDeg, 0.13f, 1.0f);
    const eg::Verdict v = eg::follows_attitude(frames, 0.5, -15.0f * kDeg);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalFollowsAttitude, FailsATiltThatDoesNotAccelerate) {
    const auto frames = pitch_history(-15.0f * kDeg, 0.13f, 0.0f);
    const eg::Verdict v = eg::follows_attitude(frames, 0.5, -15.0f * kDeg);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalFollowsAttitude, FailsNoseDownFlyingBackward) {
    const auto frames = pitch_history(-15.0f * kDeg, 0.13f, -1.0f);
    const eg::Verdict v = eg::follows_attitude(frames, 0.5, -15.0f * kDeg);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalFollowsAttitude, FailsRollCrossCoupledByFiveDegrees) {
    const auto frames = pitch_history(-15.0f * kDeg, 0.13f, 1.0f, 5.0f * kDeg);
    const eg::Verdict v = eg::follows_attitude(frames, 0.5, -15.0f * kDeg);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalFollowsAttitude, FailsATiltTooSlowToReachInOneSecond) {
    const auto frames = pitch_history(-15.0f * kDeg, 2.0f, 1.0f);
    const eg::Verdict v = eg::follows_attitude(frames, 0.5, -15.0f * kDeg);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalFollowsAttitude, IsAbsentWhenTheFramesEndEarly) {
    const auto frames = pitch_history(-15.0f * kDeg, 0.13f, 1.0f, 0.0f, 0.5, 1.5);
    const eg::Verdict v = eg::follows_attitude(frames, 0.5, -15.0f * kDeg);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "absent")) << v.detail;
}

// ===========================================================================
// armed
// ===========================================================================

TEST(SandboxEndGoalArmed, PassesStoppedThenIdlingOnTheGround) {
    const float idle[4] = {400.0f, 400.0f, 400.0f, 400.0f};
    const auto frames = arming(0.5, 400.0f, 0.05f);
    const eg::Verdict v = eg::armed(frames, 0.5, idle);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalArmed, FailsAnArmThatIsIgnored) {
    const float idle[4] = {400.0f, 400.0f, 400.0f, 400.0f};
    const auto frames = arming(0.5, 0.0f, 0.05f);
    const eg::Verdict v = eg::armed(frames, 0.5, idle);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalArmed, FailsAnIdleThatLifts) {
    const float idle[4] = {400.0f, 400.0f, 400.0f, 400.0f};
    const auto frames = arming(0.5, 400.0f, 0.05f, 0.02f);
    const eg::Verdict v = eg::armed(frames, 0.5, idle);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalArmed, FailsRotorsSpinningBeforeArming) {
    const float idle[4] = {400.0f, 400.0f, 400.0f, 400.0f};
    const auto frames = arming(0.5, 400.0f, 0.05f, 0.0f, true);
    const eg::Verdict v = eg::armed(frames, 0.5, idle);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalArmed, FailsASpinUpTooSlowForHalfASecond) {
    const float idle[4] = {400.0f, 400.0f, 400.0f, 400.0f};
    const auto frames = arming(0.5, 400.0f, 0.5f);  // 63% at 0.5 s
    const eg::Verdict v = eg::armed(frames, 0.5, idle);
    EXPECT_FALSE(v.ok) << v.detail;
}

TEST(SandboxEndGoalArmed, IsAbsentWithNoFrameBeforeArming) {
    const float idle[4] = {400.0f, 400.0f, 400.0f, 400.0f};
    const auto frames = arming(0.0, 400.0f, 0.05f);
    const eg::Verdict v = eg::armed(frames, 0.0, idle);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "absent")) << v.detail;
}

// ===========================================================================
// object-added (and its parts, placed-at and drawn)
// ===========================================================================

TEST(SandboxEndGoalObjectAdded, PassesABoxRestingWhereAskedAndDrawnOnce) {
    const eg::Verdict v = eg::object_added(good_box());
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalObjectAdded, PassesAGateDrawnAsItsThreePrimitives) {
    eg::Placement p = good_box();
    p.drawables_declared = 3;
    p.drawables_added = 3;
    const eg::Verdict v = eg::object_added(p);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalObjectAdded, FailsAnObjectBuriedOneCentimetre) {
    eg::Placement p = good_box();
    p.rest_gap = -0.01f;
    EXPECT_FALSE(eg::object_added(p).ok);
    EXPECT_FALSE(eg::placed_at(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsAnObjectFloatingOneCentimetre) {
    eg::Placement p = good_box();
    p.rest_gap = 0.01f;
    EXPECT_FALSE(eg::object_added(p).ok);
    EXPECT_FALSE(eg::placed_at(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsAnObjectTwoMillimetresFromThePick) {
    eg::Placement p = good_box();
    p.position.x += 0.002f;
    EXPECT_FALSE(eg::object_added(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsTwoDrawablesForOneDeclared) {
    eg::Placement p = good_box();
    p.drawables_added = 2;
    EXPECT_FALSE(eg::object_added(p).ok);
    EXPECT_FALSE(eg::drawn_as_declared(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsAnObjectPlacedButNotDrawn) {
    eg::Placement p = good_box();
    p.drawables_added = 0;
    EXPECT_FALSE(eg::object_added(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsADrawableAMillimetreOffItsPose) {
    eg::Placement p = good_box();
    p.drawable_offset = 1e-3f;
    EXPECT_FALSE(eg::drawn_as_declared(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsADrawableTurnedOffItsPose) {
    eg::Placement p = good_box();
    p.drawable_turn = 1e-3f;
    EXPECT_FALSE(eg::drawn_as_declared(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsTheWrongMaterial) {
    eg::Placement p = good_box();
    p.materials_match = false;
    EXPECT_FALSE(eg::object_added(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsTwoAssetsForOnePlace) {
    eg::Placement p = good_box();
    p.assets_after = 5;
    EXPECT_FALSE(eg::object_added(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsANameAnotherAssetHas) {
    eg::Placement p = good_box();
    p.name_unique = false;
    EXPECT_FALSE(eg::object_added(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsAnObjectThatMovesInRun) {
    eg::Placement p = good_box();
    p.moved_after_run = 0.02f;
    EXPECT_FALSE(eg::placed_at(p).ok);
}

TEST(SandboxEndGoalObjectAdded, FailsAnObjectThatTipsInRun) {
    eg::Placement p = good_box();
    p.tilted_after_run = 3.0f * kDeg;
    EXPECT_FALSE(eg::placed_at(p).ok);
}
