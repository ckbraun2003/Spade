// DroneSim -- the drone sim box's physics: a quadrotor held on a test stand,
// flown in attitude only, through the engine's own rotor model.
//
// WHAT THE STAND IS. The drone never translates; the air moves around it, like
// a model in a wind tunnel. Two CPU behaviors hold it there without any engine
// change:
//   * kinematic slot: pos = 0 and vel = 0;
//   * force slot (after ForceElements): force_acc = -mass * gravity, leaving
//     torque_acc alone. Integrate then adds exactly +gravity back, so with a
//     power-of-two mass the net linear acceleration is bitwise 0 and so is vel.
// The rotor thrust, drag and every moment are computed by the engine as for a
// free vehicle; only the resulting translation is thrown away. Side effect:
// the IMU reads +g, as a stand-mounted sensor does.
//
// BEHAVIORS DO NOT RUN ON VULKAN TODAY, so selecting Vulkan here is refused
// with a reason rather than stepped unpinned. That lifts when the engine has a
// translation lock on both backends.
//
// WHY A HEADER WITH NO DISPLAY IN IT: SL15b -- the window is a dumb shell, and
// everything here is asserted by tests/test_sandbox_drone.cpp with no window.
//
// The controller and mixer are scene code -- template material for anyone
// flying a Spade quadrotor -- not engine API.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <utility>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "objects/behavior.hpp"
#include "physics/schedule.hpp"
#include "sensors/rings.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/quadrotor.hpp"
#include "vehicles/rotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace spade::sandbox {

// What the physics panel edits. Every field lands in config_hash (or in the
// world description it hashes), so a change means a new Simulation -- see
// DroneSim::apply_options.
struct DronePhysicsOptions {
    float wind_speed_mps = 0.0f;
    float wind_heading_deg = 0.0f;  // direction the air moves toward: 0 = +X, 90 = +Z
    TurbulenceLevel turbulence = TurbulenceLevel::none;
    float air_density = 1.225f;
    float gravity = 9.80665f;  // magnitude, along -Y
    float throttle = 1.0f;     // fraction of hover thrust
    bool vulkan = false;
    friend bool operator==(const DronePhysicsOptions&, const DronePhysicsOptions&) = default;
};

// Radians. Yaw about +Y, pitch about +Z (nose +X up), roll about +X.
struct AttitudeTarget {
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
};

inline constexpr float kMaxTiltRad = 1.0471975512f;  // 60 degrees

// State carried across a rebuild. rotor_omega < 0 means "spawn at trim".
struct DroneCarry {
    glm::quat orient{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 omega_body{0.0f};
    float rotor_omega = -1.0f;
};

struct DroneReadouts {
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 omega_body{0.0f};
    std::array<float, 4> rotor_omega{};
    std::array<float, 4> rotor_thrust{};  // static thrust k_T w^2, N
    glm::vec3 moment_cmd{0.0f};           // last commanded body moment, N m
    float hover_omega = 0.0f;
    float hover_induced_velocity = 0.0f;
    glm::vec3 imu_accel{0.0f};
    glm::vec3 imu_gyro{0.0f};
    uint64_t tick = 0;
};

// The stand's airframe. mass is 1 kg on purpose: a power of two keeps the
// pin's (-m g) / m == -g exact, which is what makes vel bitwise 0.
[[nodiscard]] inline vehicles::QuadrotorParams drone_stand_params() {
    vehicles::QuadrotorParams p;
    p.name = "sandbox_quad";
    p.mass = 1.0f;
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f);
    p.arm_length = 0.18f;
    for (auto& r : p.rotors) {
        r.thrust_coeff = 1.2e-5f;
        r.torque_coeff = 1.9e-7f;
    }
    return p;
}

[[nodiscard]] inline glm::quat attitude_quat(const AttitudeTarget& t) {
    return glm::angleAxis(t.yaw, glm::vec3(0.0f, 1.0f, 0.0f)) *
           glm::angleAxis(t.pitch, glm::vec3(0.0f, 0.0f, 1.0f)) *
           glm::angleAxis(t.roll, glm::vec3(1.0f, 0.0f, 0.0f));
}

// The inverse of attitude_quat, for the readouts: the same yaw (Y), pitch (Z),
// roll (X) order, so a target and the attitude it produces read the same.
// glm::eulerAngles uses a different order and would not.
//   body +X in world = (cos y cos p, sin p, -sin y cos p)
//   body +Y in world, y-component = cos p cos r;  body +Z = -cos p sin r
[[nodiscard]] inline AttitudeTarget attitude_from_quat(const glm::quat& q) {
    const glm::mat3 m = glm::mat3_cast(q);
    AttitudeTarget t;
    t.pitch = std::asin(std::clamp(m[0].y, -1.0f, 1.0f));
    t.yaw = std::atan2(-m[0].z, m[0].x);
    t.roll = std::atan2(-m[2].y, m[1].y);
    return t;
}

// PD on the body-frame rotation error with rate damping, scaled by inertia:
// natural frequency 6 rad/s, critically damped. Body frame in and out,
// matching BodyState::omega_body and torque_acc.
[[nodiscard]] inline glm::vec3 attitude_moment(const glm::quat& q, const glm::vec3& omega_body,
                                               const glm::quat& q_target, const glm::vec3& inertia) {
    glm::quat e = glm::conjugate(q) * q_target;
    if (e.w < 0.0f) e = -e;  // shortest way round: never the long way through a flip
    const glm::vec3 err = 2.0f * glm::vec3(e.x, e.y, e.z);
    return inertia * (36.0f * err - 12.0f * omega_body);
}

// Moments to four thrusts around `total`, for vehicles/quadrotor.hpp's plus
// layout (rotors 0..3 at +X, +Z, -X, -Z, spin +1 -1 +1 -1):
//   M_x = L (T3 - T1),  M_z = L (T0 - T2),  M_y = -c (T0 - T1 + T2 - T3),
// with c = k_Q / k_T, since Q = k_Q w^2 = c * k_T w^2. A rotor with no torque
// coefficient (c <= 0) gives no yaw authority, so the yaw term is dropped
// rather than divided by zero.
[[nodiscard]] inline std::array<float, 4> mix_thrusts(float total, const glm::vec3& moment, float arm,
                                                      float kq_over_kt) {
    const float q = 0.25f * total;
    const float mx = moment.x / (2.0f * arm);
    const float mz = moment.z / (2.0f * arm);
    const float my = kq_over_kt > 0.0f ? moment.y / (4.0f * kq_over_kt) : 0.0f;
    return {q + mz - my, q - mx + my, q - mz - my, q + mx + my};
}

// As mix_thrusts, but with the yaw term limited so that, where roll and pitch
// alone fit inside [0, t_max] per rotor, yaw cannot push any rotor out of it.
// Yaw is the weak axis on a quadrotor (it rides on k_Q, ~1.6% of k_T here), so
// an unlimited yaw demand saturates the clamps and starves roll and pitch while
// the user holds a yaw key. Limiting it LAST keeps tilt authority. Where roll
// and pitch already exceed the range, yaw is dropped and the per-rotor clamp
// downstream does the rest.
[[nodiscard]] inline std::array<float, 4> mix_thrusts_yaw_last(float total, const glm::vec3& moment, float arm,
                                                               float kq_over_kt, float t_max) {
    const std::array<float, 4> rp = mix_thrusts(total, glm::vec3(moment.x, 0.0f, moment.z), arm, kq_over_kt);
    float my = kq_over_kt > 0.0f ? moment.y / (4.0f * kq_over_kt) : 0.0f;
    // T_i = rp_i + s_i * my with s = (-1, +1, -1, +1); keep each in [0, t_max].
    constexpr std::array<float, 4> s{-1.0f, 1.0f, -1.0f, 1.0f};
    float lo = -std::numeric_limits<float>::infinity();
    float hi = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < 4; ++i) {
        const float a = s[i] * (0.0f - rp[i]);   // bound from T_i >= 0
        const float b = s[i] * (t_max - rp[i]);  // bound from T_i <= t_max
        lo = std::max(lo, std::min(a, b));
        hi = std::min(hi, std::max(a, b));
    }
    my = lo <= hi ? std::clamp(my, lo, hi) : 0.0f;
    return {rp[0] - my, rp[1] + my, rp[2] - my, rp[3] + my};
}

namespace detail {

struct PinParams {
    uint32_t world = 0;
    uint32_t body_slot = 0;  // world-local slot
};

inline void pin_kinematic(const physics::SubstepContext& ctx, const void* p) noexcept {
    const auto& pin = *static_cast<const PinParams*>(p);
    if (pin.world >= ctx.worlds.size()) return;
    const auto& w = ctx.worlds[pin.world];
    if (pin.body_slot >= w.bodies.size()) return;
    w.bodies[pin.body_slot].pos = glm::vec3(0.0f);
    w.bodies[pin.body_slot].vel = glm::vec3(0.0f);
}

inline void pin_force(const physics::SubstepContext& ctx, const void* p) noexcept {
    const auto& pin = *static_cast<const PinParams*>(p);
    if (pin.world >= ctx.worlds.size()) return;
    const auto& w = ctx.worlds[pin.world];
    if (pin.body_slot >= w.bodies.size() || w.params == nullptr) return;
    BodyState& b = w.bodies[pin.body_slot];
    b.force_acc = -b.mass * w.params->gravity;  // Integrate adds +gravity back: net linear accel exactly 0
}

}  // namespace detail

class DroneSim {
  public:
    static constexpr uint64_t kDtNs = 2'000'000;
    static constexpr uint32_t kSubsteps = 2;
    static constexpr uint32_t kMaxStepsPerAdvance = 100;
    static constexpr float kMaxWallDtS = 0.25f;     // one frame never asks for more than this
    static constexpr float kMaxOmegaOverHover = 2.5f;

    [[nodiscard]] static Result<DroneSim> create(const DronePhysicsOptions& opts, const DroneCarry& carry = {}) {
        if (opts.vulkan) {
            return std::unexpected(Error{Code::unavailable,
                                         "the drone stand holds the drone with CPU behaviors, which the Vulkan "
                                         "step cannot run yet; staying on CPU"});
        }

        DroneSim d;
        d.opts_ = opts;
        d.params_ = drone_stand_params();

        Environment env;
        env.gravity = glm::vec3(0.0f, -opts.gravity, 0.0f);
        const float heading = glm::radians(opts.wind_heading_deg);
        env.wind = opts.wind_speed_mps * glm::vec3(std::cos(heading), 0.0f, std::sin(heading));
        env.air_density = opts.air_density;

        // No geometry: SDF distance is FLT_MAX and ground effect is 1.
        // force_elements = 4 rotors + 1 drag body; sensors = the one IMU.
        auto world = WorldBuilder()
                         .name("drone_sim_box")
                         .environment(env)
                         .capacities(Capacities{1, 5, 1, 1})
                         .build();
        if (!world) return std::unexpected(world.error());

        WorldInstanceDesc inst;
        inst.world = std::move(*world);
        inst.seed = 0xD20E5EEDu;
        inst.turbulence = dryden_params(opts.turbulence);
        WorldSetDesc set;
        set.worlds.push_back(std::move(inst));

        auto sim = Simulation::create(set, kDtNs, kSubsteps);
        if (!sim) return std::unexpected(sim.error());
        d.sim_ = std::make_unique<Simulation>(std::move(*sim));

        auto model = vehicles::make_quadrotor(d.params_);
        if (!model) return std::unexpected(model.error());
        auto id = d.sim_->register_model(std::move(*model));
        if (!id) return std::unexpected(id.error());

        VehicleSpawn where;
        where.orient = carry.orient;
        where.omega_body = carry.omega_body;
        // Trim for the throttle: thrust is k_T w^2, so a thrust fraction is a
        // sqrt on the hover speed.
        where.rotor_omega = carry.rotor_omega >= 0.0f
                                ? carry.rotor_omega
                                : vehicles::hover_command(d.params_, opts.gravity) * std::sqrt(std::max(opts.throttle, 0.0f));
        auto vehicle = d.sim_->spawn(0, *id, where);
        if (!vehicle) return std::unexpected(vehicle.error());
        d.vehicle_ = *vehicle;
        if (auto f = d.sim_->flush_structural(); !f) return std::unexpected(f.error());

        // One world, so the vehicle's slot is its world-local slot.
        d.pin_ = std::make_unique<detail::PinParams>(detail::PinParams{0u, d.vehicle_.body.slot});
        d.registry_ = std::make_unique<objects::BehaviorRegistry>();
        objects::BehaviorDesc kin;
        kin.name = "drone_stand_pin_kinematic";
        kin.slot = objects::BehaviorSlot::kinematic;
        kin.execute_cpu = &detail::pin_kinematic;
        kin.user_data = d.pin_.get();
        if (auto r = d.registry_->register_behavior(kin); !r) return std::unexpected(r.error());
        objects::BehaviorDesc force;
        force.name = "drone_stand_pin_force";
        force.slot = objects::BehaviorSlot::force;
        force.execute_cpu = &detail::pin_force;
        force.user_data = d.pin_.get();
        if (auto r = d.registry_->register_behavior(force); !r) return std::unexpected(r.error());
        d.sim_->set_behaviors(d.registry_.get());

        return Result<DroneSim>(std::move(d));
    }

    // Runs the controller before every step. Deterministic: no wall clock.
    [[nodiscard]] Result<void> step_fixed(uint32_t steps) {
        const auto& rotor0 = params_.rotors[0];
        const float kq_over_kt = rotor0.torque_coeff / rotor0.thrust_coeff;
        const float total = opts_.throttle * params_.mass * opts_.gravity;
        // The ceiling is set from standard gravity, so a zero-g scene keeps its
        // moment authority.
        const float omega_max =
            kMaxOmegaOverHover * vehicles::hover_command(params_, vehicles::kStandardGravity);
        const float t_max = rotor0.thrust_coeff * omega_max * omega_max;

        AttitudeTarget t = target;
        t.pitch = std::clamp(t.pitch, -kMaxTiltRad, kMaxTiltRad);
        t.roll = std::clamp(t.roll, -kMaxTiltRad, kMaxTiltRad);
        const glm::quat q_target = attitude_quat(t);

        for (uint32_t i = 0; i < steps; ++i) {
            auto body = sim_->body(vehicle_.body);
            if (!body) return std::unexpected(body.error());
            moment_cmd_ = attitude_moment((*body)->orient, (*body)->omega_body, q_target, params_.inertia_diag);
            const auto thrusts = mix_thrusts_yaw_last(total, moment_cmd_, params_.arm_length, kq_over_kt, t_max);
            std::array<float, 4> omegas{};
            for (std::size_t r = 0; r < omegas.size(); ++r) {
                omegas[r] = std::clamp(std::sqrt(std::max(thrusts[r], 0.0f) / rotor0.thrust_coeff), 0.0f, omega_max);
            }
            if (auto c = sim_->set_rotor_commands(vehicle_, omegas); !c) return c;
            if (auto s = sim_->step(1); !s) return s;
        }
        return {};
    }

    // Fixed-dt accumulator over the window's wall-clock frame time. Capped:
    // a stall (a minimised window, a breakpoint) never asks for more than
    // kMaxStepsPerAdvance steps, and the backlog is dropped rather than paid.
    [[nodiscard]] Result<void> advance(float wall_dt_s) {
        const float dt = wall_dt_s > 0.0f ? std::min(wall_dt_s, kMaxWallDtS) : 0.0f;  // NaN-safe
        constexpr double step_s = static_cast<double>(kDtNs) * 1e-9;
        acc_s_ += dt;
        auto n = static_cast<uint32_t>(std::floor(acc_s_ / step_s));
        if (n >= kMaxStepsPerAdvance) {
            n = kMaxStepsPerAdvance;
            acc_s_ = 0.0;
        } else {
            acc_s_ -= n * step_s;
        }
        return step_fixed(n);
    }

    // Rebuild for new options, carrying attitude, body rates and rotor speed.
    // On failure (e.g. the Vulkan refusal) the current simulation stays.
    [[nodiscard]] Result<void> apply_options(const DronePhysicsOptions& o) {
        auto body = sim_->body(vehicle_.body);
        if (!body) return std::unexpected(body.error());
        DroneCarry carry{(*body)->orient, (*body)->omega_body, 0.0f};
        for (uint32_t i = 0; i < vehicles::kQuadrotorRotorCount; ++i) {
            auto row = sim_->rotor(vehicle_, i);
            if (!row) return std::unexpected(row.error());
            carry.rotor_omega += (*row)->omega;
        }
        carry.rotor_omega /= static_cast<float>(vehicles::kQuadrotorRotorCount);

        auto next = create(o, carry);
        if (!next) return std::unexpected(next.error());
        const AttitudeTarget keep = target;
        *this = std::move(*next);
        target = keep;
        return {};
    }

    AttitudeTarget target;

    [[nodiscard]] const Simulation& sim() const noexcept { return *sim_; }
    [[nodiscard]] const VehicleRef& vehicle() const noexcept { return vehicle_; }
    [[nodiscard]] const vehicles::QuadrotorParams& params() const noexcept { return params_; }
    [[nodiscard]] const DronePhysicsOptions& options() const noexcept { return opts_; }

    [[nodiscard]] DroneReadouts readouts() const {
        DroneReadouts r;
        if (auto body = sim_->body(vehicle_.body)) {
            r.orientation = (*body)->orient;
            r.omega_body = (*body)->omega_body;
        }
        for (uint32_t i = 0; i < vehicles::kQuadrotorRotorCount; ++i) {
            if (auto row = sim_->rotor(vehicle_, i)) {
                r.rotor_omega[i] = (*row)->omega;
                r.rotor_thrust[i] = (*row)->thrust_coeff * (*row)->omega * (*row)->omega;
            }
        }
        r.moment_cmd = moment_cmd_;
        r.hover_omega = vehicles::hover_command(params_, opts_.gravity);
        r.hover_induced_velocity = vehicles::rotor_hover_induced_velocity(
            0.25f * params_.mass * opts_.gravity, opts_.air_density, params_.rotors[0].radius);
        if (vehicle_.imu_count > 0) {
            std::array<sensors::ImuSample, sensors::kRingDepth> buf{};
            if (auto poll = sim_->poll_imu(vehicle_.imu_sensors[0], 0, buf); poll && !poll->samples.empty()) {
                r.imu_accel = poll->samples.back().accel;
                r.imu_gyro = poll->samples.back().gyro;
            }
        }
        r.tick = sim_->tick().value;
        return r;
    }

  private:
    DroneSim() = default;

    // Heap-held so the addresses the registry and the Simulation borrow
    // (user_data -> pin_, behaviors_ -> registry_) survive a move of DroneSim.
    std::unique_ptr<Simulation> sim_;
    std::unique_ptr<objects::BehaviorRegistry> registry_;
    std::unique_ptr<detail::PinParams> pin_;
    VehicleRef vehicle_{};
    vehicles::QuadrotorParams params_{};
    DronePhysicsOptions opts_{};
    glm::vec3 moment_cmd_{0.0f};
    double acc_s_ = 0.0;
};

}  // namespace spade::sandbox
