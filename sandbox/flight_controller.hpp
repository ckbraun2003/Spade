// N17's flight controller: the pilot's analysis::Controls {armed, throttle,
// pitch, roll, yaw} to each rotor's commanded speed, THROUGH THE CHAIN, once
// per simulation step. Interface's, in the sandbox, until a second consumer
// needs it (the end-goal design's ruling 2); its mixer is flight_mixer.hpp.
//
// - DISARMED: every rotor 0 (the arm rule, vehicles/propulsion_steady.hpp).
// - COLLECTIVE: the arm rule's duty for the throttle, idle + (1 - idle)
//   throttle, as each rotor's steady thrust (steady_state_at_duty), over the
//   cosine of the body's tilt, floored at 0.5, so a tilt doesn't sink it.
// - ATTITUDE: the target is yaw about +Y, then pitch about +Z, then roll about
//   +X, in the model's design frame (+X the nose), taken to its principal
//   frame. The error, in the body, splits into a tilt and a yaw about body +Y
//   (q_err = q_tilt * q_yaw), and the moment is the principal inertia times
//   kp e - kd w, each part with its own omega_n (Physics' review, 2026-10-22):
//   the tilt's min(12, 0.6 / tau_max), tau_max the model's slowest rotor lag,
//   since faster rings against it; yaw's a third of that, since a quad's yaw
//   authority is about a fifth of its pitch's; zeta 0.9.
// - Each rotor's thrust (flight_mixer.hpp's allocation within its idle and
//   full-duty thrust) to its duty (steady_state_for_thrust) and its speed
//   (rotor_omega_for_duty), so the chain's idle and limits apply, and an
//   armed rotor never drops below idle.
//
// Refused by name: a model without a chain (as LiveRun refuses one), a
// layout the mixer refuses, and what the chain's own functions refuse.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "analysis/case.hpp"  // Controls, kDefaultIdleDuty, kDefaultStateOfCharge
#include "core/error.hpp"
#include "flight_mixer.hpp"
#include "state/layout.hpp"  // BodyState
#include "vehicles/model_type.hpp"
#include "vehicles/propulsion_steady.hpp"

namespace spade::sandbox::flight {

struct Gains {
    float tau_max = 0.0f;     // the model's slowest rotor lag, s
    float omega_tilt = 0.0f;  // rad/s
    float omega_yaw = 0.0f;   // rad/s
    float zeta = 0.9f;
};

namespace detail {

// A unit quaternion's rotation vector, the short way round.
[[nodiscard]] inline glm::vec3 rotation_vector(glm::quat q) {
    if (q.w < 0.0f) q = -q;
    const glm::vec3 v(q.x, q.y, q.z);
    const float s = glm::length(v);
    if (s < 1e-9f) return 2.0f * v;
    return (2.0f * std::atan2(s, q.w) / s) * v;
}

}  // namespace detail

class FlightController {
  public:
    [[nodiscard]] static Result<FlightController> make(const vehicles::ModelType& model, float density,
                                                       float soc = analysis::kDefaultStateOfCharge,
                                                       float idle = analysis::kDefaultIdleDuty) {
        const auto refuse = [&](const std::string& why) -> Result<FlightController> {
            return std::unexpected(
                Error{Code::invalid_argument, "flight controller: model '" + model.name + "': " + why});
        };
        if (!model.propulsion) return refuse("it has rotors and no propulsion chain to command them through");
        Result<Mixer> mixer = make_mixer(model);
        if (!mixer) return std::unexpected(mixer.error());
        // The chain's own checks (density, state of charge, a chain that can
        // spin), through the speed it gives at full duty.
        if (Result<vehicles::DutyCommand> full = vehicles::rotor_omega_for_duty(*model.propulsion, 1.0, density, soc);
            !full) {
            return refuse(full.error().context);
        }
        if (Result<double> d = vehicles::armed_duty(true, 0.0, idle); !d) return refuse(d.error().context);

        FlightController c;
        c.chain_ = *model.propulsion;
        c.design_to_principal_ = model.design_to_principal;
        c.inertia_ = model.body.inertia_diag;
        c.mixer_ = std::move(*mixer);
        c.density_ = density;
        c.soc_ = soc;
        c.idle_ = idle;
        for (const vehicles::RotorDesc& r : model.rotors) c.gains_.tau_max = std::max(c.gains_.tau_max, r.tau);
        c.gains_.omega_tilt = c.gains_.tau_max > 0.0f ? std::min(12.0f, 0.6f / c.gains_.tau_max) : 12.0f;
        c.gains_.omega_yaw = c.gains_.omega_tilt / 3.0f;
        c.t_idle_ = static_cast<float>(vehicles::steady_state_at_duty(c.chain_, idle, density, 0.0, soc).thrust);
        c.t_full_ = static_cast<float>(vehicles::steady_state_at_duty(c.chain_, 1.0, density, 0.0, soc).thrust);
        for (const std::array<float, 4>& col : c.mixer_.b) c.up_share_ += col[0];
        return c;
    }

    [[nodiscard]] const Gains& gains() const noexcept { return gains_; }

    // Each rotor's commanded speed, rad/s, into `omega_cmd` (one per rotor,
    // in the model's order), for `controls` with the body as the simulation
    // holds it (principal frame).
    [[nodiscard]] Result<void> command(const analysis::Controls& controls, const BodyState& body,
                                       std::span<float> omega_cmd) const {
        if (omega_cmd.size() != mixer_.b.size()) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "flight controller: " + std::to_string(omega_cmd.size()) +
                                             " speeds asked of a model with " + std::to_string(mixer_.b.size()) +
                                             " rotors"});
        }
        const Result<double> duty = vehicles::armed_duty(controls.armed, controls.throttle, idle_);
        if (!duty) return std::unexpected(duty.error());
        if (!controls.armed) {
            std::fill(omega_cmd.begin(), omega_cmd.end(), 0.0f);
            return {};
        }

        // The collective, over the tilt.
        const float per_rotor =
            static_cast<float>(vehicles::steady_state_at_duty(chain_, *duty, density_, 0.0, soc_).thrust);
        const float up = (body.orient * glm::vec3(0.0f, 1.0f, 0.0f)).y;
        const float collective = per_rotor * up_share_ / std::max(up, 0.5f);

        // The attitude: the design frame's target in the principal frame, the
        // error in the body split into tilt and yaw (about body +Y).
        const glm::quat target_design = glm::angleAxis(glm::radians(controls.yaw_deg), glm::vec3(0.0f, 1.0f, 0.0f)) *
                                        glm::angleAxis(glm::radians(controls.pitch_deg), glm::vec3(0.0f, 0.0f, 1.0f)) *
                                        glm::angleAxis(glm::radians(controls.roll_deg), glm::vec3(1.0f, 0.0f, 0.0f));
        const glm::quat target = target_design * glm::conjugate(design_to_principal_);
        glm::quat error = glm::conjugate(body.orient) * target;
        if (error.w < 0.0f) error = -error;
        // The twist about +Y is undefined for a tilt error near 180 degrees (w and
        // y both near 0): there it is none, and the tilt carries the whole error
        // (Physics' guard).
        glm::quat yaw(1.0f, 0.0f, 0.0f, 0.0f);
        if (const float w2y2 = error.w * error.w + error.y * error.y; w2y2 > 1e-6f) {
            const float norm = std::sqrt(w2y2);
            yaw = glm::quat(error.w / norm, 0.0f, error.y / norm, 0.0f);
        }
        const glm::quat tilt = error * glm::conjugate(yaw);
        const glm::vec3 e_tilt = detail::rotation_vector(tilt);
        const glm::vec3 e_yaw = detail::rotation_vector(yaw);
        const float kp_t = gains_.omega_tilt * gains_.omega_tilt;
        const float kp_y = gains_.omega_yaw * gains_.omega_yaw;
        const glm::vec3 kd(2.0f * gains_.zeta * gains_.omega_tilt, 2.0f * gains_.zeta * gains_.omega_yaw,
                           2.0f * gains_.zeta * gains_.omega_tilt);
        const glm::vec3 accel = kp_t * e_tilt + kp_y * e_yaw - kd * body.omega_body;
        const Allocation a = allocate(mixer_, Wrench{collective, inertia_ * accel}, t_idle_, t_full_);

        // Each rotor's thrust to its duty and speed, through the chain.
        for (std::size_t i = 0; i < a.thrust.size(); ++i) {
            const vehicles::SteadyPoint p = vehicles::steady_state_for_thrust(chain_, a.thrust[i], density_, 0.0, soc_);
            const double d = std::clamp(p.duty, idle_, 1.0);
            const Result<vehicles::DutyCommand> w = vehicles::rotor_omega_for_duty(chain_, d, density_, soc_);
            if (!w) return std::unexpected(w.error());
            omega_cmd[i] = w->omega_cmd;
        }
        return {};
    }

  private:
    FlightController() = default;

    vehicles::PropulsionChain chain_{};
    glm::quat design_to_principal_{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 inertia_{1.0f};
    Mixer mixer_{};
    Gains gains_{};
    double density_ = 0.0;
    double soc_ = 1.0;
    double idle_ = 0.0;
    float t_idle_ = 0.0f;    // each rotor's steady thrust at idle, N
    float t_full_ = 0.0f;    // and at full duty
    float up_share_ = 0.0f;  // the sum of the rotors' axes along body +Y
};

}  // namespace spade::sandbox::flight
