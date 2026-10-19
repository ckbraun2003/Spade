// LiveRun: the drone-box airflow run (I1; the harness design,
// docs/design/interface/plans/2026-10-13-i1-harness-design.md, approved by
// the lead 2026-10-13). The one stepping core behind D1's recording
// (flow_movie --case), D2's panel, the queued runner's real path and Kat's
// host.
//
// THE DRONE IS HELD at the box's centre (Cameron's drone-box premise): a
// Simulation of the case's model, set each step by set_body_state() to the
// commanded attitude, with zero velocity and the attitude's rate as its
// omega_body. It is one-way: the air never pushes back. A held drone's IMU readings are
// meaningless (the hold, not the air, sets its motion), so none are exposed.
//
// TIME is whole nanoseconds. The drone and the fluid both step at the case's
// h, one drone step per fluid step, so the clocks never drift. Frame k ends
// at the first step at or past round(k / kFramesPerSecond) s. No wall clock
// (M1B): a host measures the real-time ratio around advance_frame().
//
// THE ROTORS turn through the drone's propulsion chain (Core's throttle
// map): the controls' throttle becomes a duty, armed_duty(armed, throttle,
// idle), and the duty every rotor's commanded speed, rotor_omega_for_duty()
// at the case's density and the battery's state of charge. Each rotor lags
// toward it on its fitted tau, so disarming spins them down. After each
// drone step, the rotors' rows at the held body become actuator discs
// (Physics' P7a, vehicles::rotor_discs()) spread into the fluid's step. A
// limit the chain settles at (a motor or the pack at its current rating) is
// announced in the state and on the run's label, never refused. A model with
// rotors needs its propulsion; it comes beside the model until Core's model
// file carries the chain and the case its idle and state of charge (the
// chain pin). The solids join with P2.
//
// Long calls: start() and advance_frame(). A host runs them off its GUI
// thread. A LiveRun is used from one thread at a time.
//
// Display-free. Owned by Interface (INT-6).

#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
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
#include "vehicles/propulsion_steady.hpp"

namespace spade::analysis {

// The save file's version (LiveRun::save()). 2: the propulsion it flew.
inline constexpr uint32_t kLiveRunSaveVersion = 2;

// How fast the hold turns the drone toward the controls' attitude, per axis.
inline constexpr float kHoldRateDegPerS = 180.0f;
inline constexpr uint64_t kFramesPerSecond = 30;

// The drone's propulsion, beside its model until the chain pin. One chain
// turns every rotor (one per airframe); `idle` and `soc` are the caller's to
// set, as the case will hold them.
struct LivePropulsion {
    vehicles::PropulsionChain chain;
    double idle = 0.0;  // the duty an armed drone spins at with zero throttle, in [0, 1)
    double soc = 1.0;   // the battery's state of charge, in [0, 1]
};

struct LiveState {
    double t_s = 0.0;  // simulated, whole steps of h
    uint64_t steps = 0;
    uint64_t frames = 0;
    glm::quat attitude{1.0f, 0.0f, 0.0f, 0.0f};  // the design frame's axes -> the world's, at the box's centre
    glm::vec3 omega_body{0.0f};                  // the hold's rate, design axes, rad/s
    bool armed = false;
    std::vector<float> rotor_omega;  // rad/s, in the model's rotor order
    uint32_t propulsion_flags = 0;   // the last command's propulsion_flags: limits, announced
};

struct LiveFrame {
    uint64_t frame = 0;  // 1 for the first
    double t_s = 0.0;
    uint64_t steps = 0;
};

// The showcase program's trim (controls.hpp) for `model` in the case's air:
// - a model without rotors has no hover, and both throttles are 0;
// - rotors without propulsion are refused, as LiveRun::start() refuses them;
// - else each throttle is the one whose duty, through armed_duty()'s idle,
//   gives the target thrust per rotor: the weight over the rotor count
//   (Physics' rule, steady_state_for_thrust() in still air at the case's
//   density and the propulsion's state of charge; the airframe fit's
//   hover_duty when those match the compile's), then kShowcaseClimbThrust
//   times it. The weight is in LiveRun's world, Environment{}'s gravity.
// Refused by name: the throttle map's own refusals; a chain that cannot hover
// the model; a climb out of reach; an idle at or above the hover duty (armed
// at zero throttle the drone would climb).
[[nodiscard]] Result<ShowcaseTrim> showcase_trim(const AnalysisCase& c, const vehicles::ModelType& model,
                                                 const std::optional<LivePropulsion>& propulsion);

class LiveRun {
  public:
    // Builds the grid (the resolved base and drone-box faces), the fluid and
    // the held drone. Refused by name: a case without time.step (until
    // Physics' step rule exists), a model with rotors and no propulsion, an
    // idle, state of charge or chain the throttle map refuses, a grid
    // make_grid() refuses, and a model or world the Simulation refuses.
    [[nodiscard]] static Result<LiveRun> start(const ResolvedCase& resolved, const vehicles::ModelType& model,
                                               const std::optional<LivePropulsion>& propulsion = std::nullopt);

    LiveRun(LiveRun&&) noexcept;
    LiveRun& operator=(LiveRun&&) noexcept;
    ~LiveRun();

    // One step of h: the hold, the rotors' command, the drone, the discs, the
    // fluid. A throttle the throttle map refuses is refused before anything
    // steps. A fluid fault refuses the step, naming it, and every step after.
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

    // The caption without the time: the rotor model, the grid, dx and R/dx,
    // then any limit the propulsion is at, for example "actuator disc ·
    // 128³, dx 12.5 mm, R/dx 6 · motors at their current limit" (UTF-8).
    [[nodiscard]] std::string run_label() const;

    // run_label(), then " · t = 0.250 s".
    [[nodiscard]] std::string run_line() const;

    // The controls flown so far, a point wherever they changed: the live
    // session as a program, for "queue at higher fidelity".
    [[nodiscard]] const ControlProgram& recorded_controls() const noexcept;

    // SAVE AND RESTORE, so a long run (a D1 film, a queued run) stops and
    // resumes in foreground chunks: a restored run gives exactly the frames
    // an unbroken one does. One file, replaced whole (a kill leaves the old
    // save or the new): the case's and model's identities, the counters,
    // the hold, the recorded controls, Core's Simulation snapshot and
    // Physics' grid (u, v, w, p), under one FNV-1a checksum. Solids and the
    // fluid's scratch are rebuilt, not saved (Physics). A faulted run isn't
    // saved: its last good save is the one before the fault.
    [[nodiscard]] Result<void> save(const std::filesystem::path& file) const;

    // Rebuilds the run from the case, model and propulsion, then its saved
    // state. Refused by name: a missing file (io_error), a foreign or short
    // one, a version this build doesn't read (schema_mismatch), a checksum
    // that doesn't match, and another case, model, parts, propulsion chain,
    // idle or state of charge.
    [[nodiscard]] static Result<LiveRun> restore(const std::filesystem::path& file, const ResolvedCase& resolved,
                                                 const vehicles::ModelType& model,
                                                 const std::optional<LivePropulsion>& propulsion = std::nullopt);

  private:
    struct Impl;
    explicit LiveRun(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

}  // namespace spade::analysis
