// LiveRun (see the header).

#include "analysis/live_run.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <string>
#include <utility>

#include "core/fp32_math.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "sim/design_frame.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"

namespace spade::analysis {

namespace {

namespace af = physics::airflow;

constexpr uint64_t kNsPerSecond = 1'000'000'000;
constexpr float kRadPerDeg = std::numbers::pi_v<float> / 180.0f;

[[nodiscard]] double seconds(uint64_t ns) { return static_cast<double>(ns) / static_cast<double>(kNsPerSecond); }

// A rotation of `deg` about a unit axis, through sin32 and cos32, not libm (TD-3).
[[nodiscard]] glm::quat about(float deg, const glm::vec3& axis) {
    const float half = 0.5f * deg * kRadPerDeg;
    const float s = math::sin32(half);
    return glm::quat(math::cos32(half), s * axis.x, s * axis.y, s * axis.z);
}

// One axis of the hold: toward `target` by at most `max_deg`. Returns the
// rate it moved at, deg/s.
[[nodiscard]] float approach(float& now, float target, float max_deg, float h) {
    const float gap = target - now;
    const float moved = std::clamp(gap, -max_deg, max_deg);
    now = std::abs(gap) <= max_deg ? target : now + moved;
    return moved / h;
}

[[nodiscard]] bool same(const Controls& a, const Controls& b) {
    return a.armed == b.armed && a.throttle == b.throttle && a.pitch_deg == b.pitch_deg && a.roll_deg == b.roll_deg &&
           a.yaw_deg == b.yaw_deg;
}

[[nodiscard]] std::string_view rotor_model_name(RotorModel m) {
    switch (m) {
        case RotorModel::disc:
            return "actuator disc";
        case RotorModel::line:
            return "actuator line";
        case RotorModel::blade_resolved:
            return "blade-resolved";
    }
    return "rotor";
}

}  // namespace

struct LiveRun::Impl {
    ResolvedCase resolved;
    vehicles::ModelType model;
    af::MacGrid grid;
    af::FluidConfig fluid;
    af::FluidScratch scratch;
    af::PublishInput publish;
    std::unique_ptr<Simulation> sim;
    VehicleRef vehicle;
    uint64_t h_ns = 0;
    float yaw = 0.0f;  // the held attitude, degrees
    float pitch = 0.0f;
    float roll = 0.0f;
    LiveState state;
    ControlProgram flown;
    std::string fault;  // non-empty once the fluid has faulted
};

LiveRun::LiveRun(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
LiveRun::LiveRun(LiveRun&&) noexcept = default;
LiveRun& LiveRun::operator=(LiveRun&&) noexcept = default;
LiveRun::~LiveRun() = default;

Result<LiveRun> LiveRun::start(const ResolvedCase& resolved, const vehicles::ModelType& model) {
    const AnalysisCase& c = resolved.spec;
    if (!c.time.step || !(*c.time.step > 0.0f)) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "a live run needs the case's time.step: the solver's own step rule is "
                                     "Physics' to write, and doesn't exist yet"});
    }
    auto impl = std::make_unique<Impl>();
    impl->resolved = resolved;
    impl->model = model;
    impl->h_ns = static_cast<uint64_t>(static_cast<double>(*c.time.step) * static_cast<double>(kNsPerSecond) + 0.5);

    // The fluid: the resolved base grid, with the drone box's faces.
    Result<af::MacGrid> grid = af::make_grid(resolved.base, resolved.domain);
    if (!grid) return std::unexpected(grid.error());
    impl->grid = std::move(*grid);
    impl->fluid.rho = c.fluid.density;
    impl->fluid.nu = c.fluid.kinematic_viscosity;
    impl->fluid.h = static_cast<float>(seconds(impl->h_ns));
    Result<af::FluidScratch> scratch = af::make_fluid_scratch(impl->grid, resolved.domain, impl->fluid);
    if (!scratch) return std::unexpected(scratch.error());
    impl->scratch = std::move(*scratch);
    // The world's origin is the box's centre, where the drone is held.
    const af::GridShape& g = resolved.base;
    impl->publish.origin = -0.5f * g.dx * glm::vec3(static_cast<float>(g.nx), static_cast<float>(g.ny), static_cast<float>(g.nz));
    impl->publish.periodic = af::periodic_axes(resolved.domain);

    // The drone: one body of the model in a world of its own, without
    // geometry. Gravity is the world's, though the hold sets the body each step.
    Result<WorldDesc> world =
        WorldBuilder()
            .name("analysis_live_run")
            .environment(Environment{})
            .capacities(Capacities{1, static_cast<uint32_t>(model.rotors.size() + model.drag_bodies.size()),
                                   static_cast<uint32_t>(model.imu_mounts.size()), 1})
            .build();
    if (!world) return std::unexpected(world.error());
    WorldInstanceDesc instance;
    instance.world = std::move(*world);
    WorldSetDesc set;
    set.worlds.push_back(std::move(instance));
    Result<Simulation> sim = Simulation::create(set, impl->h_ns, 1);
    if (!sim) return std::unexpected(sim.error());
    impl->sim = std::make_unique<Simulation>(std::move(*sim));
    const Result<ModelTypeId> id = impl->sim->register_model(model);
    if (!id) return std::unexpected(id.error());
    const Result<VehicleRef> vehicle = impl->sim->spawn(0, *id, VehicleSpawn{});
    if (!vehicle) return std::unexpected(vehicle.error());
    impl->vehicle = *vehicle;
    if (Result<void> r = impl->sim->flush_structural(); !r) return std::unexpected(r.error());
    impl->state.rotor_omega.assign(model.rotors.size(), 0.0f);
    return LiveRun(std::move(impl));
}

Result<void> LiveRun::step(const Controls& controls) {
    Impl& r = *impl_;
    if (!r.fault.empty()) return std::unexpected(Error{Code::internal, r.fault});
    const uint64_t now_ns = r.state.steps * r.h_ns;

    // The flight, as a program: a point wherever the controls change.
    if (r.flown.points.empty() || !same(r.flown.points.back().controls, controls)) {
        r.flown.points.push_back(ControlPoint{static_cast<float>(seconds(now_ns)), controls});
    }

    // The hold: toward the controls' attitude at the hold's rate, per axis
    // (yaw about +Y, then pitch about +Z, then roll about +X).
    const float h = r.fluid.h;
    const float max_deg = kHoldRateDegPerS * h;
    const float yaw_rate = approach(r.yaw, controls.yaw_deg, max_deg, h) * kRadPerDeg;
    const float pitch_rate = approach(r.pitch, controls.pitch_deg, max_deg, h) * kRadPerDeg;
    const float roll_rate = approach(r.roll, controls.roll_deg, max_deg, h) * kRadPerDeg;
    const glm::quat qy = about(r.yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::quat qz = about(r.pitch, glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::quat qx = about(r.roll, glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::quat attitude = qy * qz * qx;
    // Each Euler rate about its own axis, brought into the design axes.
    const glm::vec3 omega = glm::conjugate(qz * qx) * glm::vec3(0.0f, yaw_rate, 0.0f) +
                            glm::conjugate(qx) * glm::vec3(0.0f, 0.0f, pitch_rate) + glm::vec3(roll_rate, 0.0f, 0.0f);
    FrameState design;
    design.orient = attitude;
    design.omega = omega;
    const FrameState body = to_body_state(design, r.model.design_to_principal, r.model.com_offset);
    if (Result<void> s = r.sim->set_body_state(r.vehicle.body, body.pos, body.orient, body.vel, body.omega); !s) {
        return std::unexpected(s.error());
    }
    if (Result<void> s = r.sim->step(1); !s) return std::unexpected(s.error());
    for (uint32_t i = 0; i < r.state.rotor_omega.size(); ++i) {
        const Result<const vehicles::RotorRow*> row = r.sim->rotor(r.vehicle, i);
        if (!row) return std::unexpected(row.error());
        r.state.rotor_omega[i] = (*row)->omega;
    }

    // The fluid.
    const af::FluidHealth health = af::fluid_step(r.grid, r.resolved.domain, r.fluid, r.scratch);
    if (health.fault != af::FluidFault::none) {
        r.fault = "the fluid faulted at step " + std::to_string(r.state.steps + 1) + ": " +
                  af::describe(health, r.resolved.base);
        return std::unexpected(Error{Code::internal, r.fault});
    }

    ++r.state.steps;
    r.state.t_s = seconds(r.state.steps * r.h_ns);
    r.state.attitude = attitude;
    r.state.omega_body = omega;
    r.state.armed = controls.armed;
    return {};
}

Result<LiveFrame> LiveRun::advance_frame(const Controls& controls, FlowVolume& out) {
    Impl& r = *impl_;
    const uint64_t next = r.state.frames + 1;
    // round(k / fps) s, in whole ns.
    const uint64_t end_ns = (next * kNsPerSecond + kFramesPerSecond / 2) / kFramesPerSecond;
    while (r.state.steps * r.h_ns < end_ns) {
        if (Result<void> s = step(controls); !s) return std::unexpected(s.error());
    }
    const Result<af::PublishCounts> published = af::publish_render_volume(r.grid, r.publish, out);
    if (!published) return std::unexpected(published.error());
    r.state.frames = next;
    return LiveFrame{next, r.state.t_s, r.state.steps};
}

const LiveState& LiveRun::state() const noexcept { return impl_->state; }

double LiveRun::step_s() const noexcept { return seconds(impl_->h_ns); }

std::string LiveRun::run_line() const {
    const Impl& r = *impl_;
    const af::GridShape& g = r.resolved.base;
    std::string out;
    const std::vector<ResolvedRotor>& rotors = r.resolved.rotors;
    const bool one_model = std::all_of(rotors.begin(), rotors.end(),
                                       [&](const ResolvedRotor& x) { return x.model == rotors.front().model; });
    out += rotors.empty() ? "no rotors" : one_model ? std::string(rotor_model_name(rotors.front().model)) : "mixed rotors";
    out += " \xc2\xb7 ";  // a middle dot
    out += g.nx == g.ny && g.ny == g.nz ? std::format("{}\xc2\xb3", g.nx)  // a superscript three
                                        : std::format("{}x{}x{}", g.nx, g.ny, g.nz);
    out += std::format(", dx {:g} mm", g.dx * 1000.0f);
    if (!rotors.empty()) out += std::format(", R/dx {:.0f}", rotors.front().cells_across_radius);
    out += std::format(" \xc2\xb7 t = {:.3f} s", r.state.t_s);
    return out;
}

const ControlProgram& LiveRun::recorded_controls() const noexcept { return impl_->flown; }

}  // namespace spade::analysis
