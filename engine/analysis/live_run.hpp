// LiveRun: the drone-box airflow run (I1; the harness design,
// docs/design/interface/plans/2026-10-13-i1-harness-design.md, approved by
// the lead 2026-10-13). The one stepping core behind D1's recording
// (flow_movie --case), D2's panel, the queued runner's real path and Kat's
// host.
//
// THE DRONE IS HELD at the box's centre (Cameron's drone-box premise): a
// Simulation of the case's model, set each step by set_body_state() to the
// commanded attitude, with zero velocity and the attitude's rate as its
// omega_body. Its motors spin up and down through the model's motor model.
// It is one-way: the air never pushes back. A held drone's IMU readings are
// meaningless (the hold, not the air, sets its motion), so none are exposed.
//
// TIME is whole nanoseconds. The drone and the fluid both step at the case's
// h, one drone step per fluid step, so the clocks never drift. Frame k ends
// at the first step at or past round(k / kFramesPerSecond) s. No wall clock
// (M1B): a host measures the real-time ratio around advance_frame().
//
// THIS SLICE: the grid, the hold, the time, the frames and the publish. The
// rotors' commands join with Core's throttle command, the discs with
// Physics' P7a, and the solids with P2. Until the rotors are coupled, a
// model with rotors starts but doesn't step (unavailable, "rotors not
// coupled yet"), so nothing records still air under a thrusting drone.
//
// Long calls: start() and advance_frame(). A host runs them off its GUI
// thread. A LiveRun is used from one thread at a time.
//
// Display-free. Owned by Interface (INT-6).

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "state/layout.hpp"  // BodyState
#include "vehicles/model_type.hpp"

namespace spade::analysis {

// How fast the hold turns the drone toward the controls' attitude, per axis.
inline constexpr float kHoldRateDegPerS = 180.0f;
inline constexpr uint64_t kFramesPerSecond = 30;

struct LiveState {
    double t_s = 0.0;  // simulated, whole steps of h
    uint64_t steps = 0;
    uint64_t frames = 0;
    glm::quat attitude{1.0f, 0.0f, 0.0f, 0.0f};  // the design frame's axes -> the world's, at the box's centre
    glm::vec3 omega_body{0.0f};                  // the hold's rate, design axes, rad/s
    bool armed = false;
    std::vector<float> rotor_omega;  // rad/s, in the model's rotor order
};

struct LiveFrame {
    uint64_t frame = 0;  // 1 for the first
    double t_s = 0.0;
    uint64_t steps = 0;
};

class LiveRun {
  public:
    // Builds the grid (the resolved base and drone-box faces), the fluid and
    // the held drone. Refused by name: a case without time.step (until
    // Physics' step rule exists), a grid make_grid() refuses, and a model or
    // world the Simulation refuses.
    [[nodiscard]] static Result<LiveRun> start(const ResolvedCase& resolved, const vehicles::ModelType& model);

    LiveRun(LiveRun&&) noexcept;
    LiveRun& operator=(LiveRun&&) noexcept;
    ~LiveRun();

    // One step of h: the hold, the drone, the fluid. A fluid fault refuses
    // the step, naming it, and every step after it.
    [[nodiscard]] Result<void> step(const Controls& controls);

    // Steps to the end of the next frame, then publishes the fluid into
    // `out` (its storage reused), with the world origin at the box's centre.
    [[nodiscard]] Result<LiveFrame> advance_frame(const Controls& controls, FlowVolume& out);

    [[nodiscard]] const LiveState& state() const noexcept;
    [[nodiscard]] double step_s() const noexcept;  // h

    // The held body row (its centre of mass and principal axes), as Core's
    // posed_parts() takes it: what a host draws the solid drone from. The
    // blade azimuth psi is Physics' rule (P11); until it lands, pass zeros.
    [[nodiscard]] const BodyState& body() const noexcept;

    // The caption: the rotor model, the grid, dx, R/dx and t, for example
    // "actuator disc · 128³, dx 12.5 mm, R/dx 6 · t = 0.250 s" (UTF-8).
    [[nodiscard]] std::string run_line() const;

    // The controls flown so far, a point wherever they changed: the live
    // session as a program, for "queue at higher fidelity".
    [[nodiscard]] const ControlProgram& recorded_controls() const noexcept;

  private:
    struct Impl;
    explicit LiveRun(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

}  // namespace spade::analysis
