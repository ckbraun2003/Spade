// LiveRun (see the header).

#include "analysis/live_run.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <string>
#include <utility>

#include <bit>
#include <cstring>
#include <span>
#include <string_view>

#include "analysis/detail/atomic_file.hpp"
#include "core/fp32_math.hpp"
#include "core/path_text.hpp"
#include "core/rng.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/grid_io.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "sim/design_frame.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "state/snapshot.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/propulsion_flags.hpp"
#include "vehicles/rotor_discs.hpp"
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

// ---- the save file ----------------------------------------------------------
// Little-endian, as both toolchains are; one FNV-1a 64 over everything before
// the checksum's own 8 bytes.

static_assert(std::endian::native == std::endian::little, "LiveRun's save file is little-endian");
constexpr std::string_view kSaveMagic = "SPADELRS";

struct Writer {
    std::string out;
    template <class T>
    void put(const T& v) {
        char b[sizeof(T)];
        std::memcpy(b, &v, sizeof(T));
        out.append(b, sizeof(T));
    }
    void put(const glm::vec3& v) {
        put(v.x);
        put(v.y);
        put(v.z);
    }
    void put(const glm::quat& q) {
        put(q.w);
        put(q.x);
        put(q.y);
        put(q.z);
    }
    void bytes(std::string_view s) {
        put<uint64_t>(s.size());
        out.append(s);
    }
};

struct Reader {
    std::string_view in;
    std::size_t at = 0;
    bool ok = true;
    template <class T>
    [[nodiscard]] T get() {
        T v{};
        if (!ok || in.size() - at < sizeof(T)) {
            ok = false;
            return v;
        }
        std::memcpy(&v, in.data() + at, sizeof(T));
        at += sizeof(T);
        return v;
    }
    [[nodiscard]] glm::vec3 vec() {
        const float x = get<float>();
        const float y = get<float>();
        const float z = get<float>();
        return glm::vec3(x, y, z);
    }
    [[nodiscard]] glm::quat quat() {
        const float w = get<float>();
        const float x = get<float>();
        const float y = get<float>();
        const float z = get<float>();
        return glm::quat(w, x, y, z);
    }
    [[nodiscard]] std::string_view bytes() {
        const uint64_t n = get<uint64_t>();
        if (!ok || n > in.size() - at) {
            ok = false;
            return {};
        }
        const std::string_view s = in.substr(at, static_cast<std::size_t>(n));
        at += static_cast<std::size_t>(n);
        return s;
    }
};

[[nodiscard]] std::string_view as_text(std::span<const std::byte> b) {
    return std::string_view(reinterpret_cast<const char*>(b.data()), b.size());
}

[[nodiscard]] std::span<const std::byte> as_bytes(std::string_view s) {
    return std::as_bytes(std::span<const char>(s.data(), s.size()));
}

[[nodiscard]] std::string hex(uint64_t v) { return std::format("{:016x}", v); }

// The limits the propulsion is at, as the run's label announces them, in
// propulsion_flags' order; empty for none. The two refusals (no_bracket,
// unreachable) never reach here: the command is refused instead.
[[nodiscard]] std::string announced_limits(uint32_t flags) {
    namespace pf = vehicles::propulsion_flags;
    constexpr std::pair<uint32_t, std::string_view> kSaid[] = {
        {pf::duty_clamped, "duty clamped"},
        {pf::current_limited, "motors at their current limit"},
        {pf::speed_clamped, "shaft speed held at 0"},
        {pf::out_of_table, "propeller beyond its table"},
        {pf::battery_current_limited, "pack at its current limit"},
        {pf::battery_cutoff, "pack at its cutoff voltage"},
        {pf::battery_empty, "pack empty"},
        {pf::esc_total_limited, "ESC at its total current limit"},
    };
    std::string out;
    uint32_t said = 0;
    for (const auto& [bit, words] : kSaid) {
        if ((flags & bit) == 0) continue;
        if (!out.empty()) out += ", ";
        out += words;
        said |= bit;
    }
    if (const uint32_t rest = flags & ~said; rest != 0) {
        if (!out.empty()) out += ", ";
        out += std::format("propulsion flags {:#x}", rest);
    }
    return out;
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

// BodyState is over-aligned for the GPU mirror, so Impl is padded: the same
// disable, for the same reason, as sim/world_set.hpp's WorldInstanceDesc.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
struct LiveRun::Impl {
    ResolvedCase resolved;
    vehicles::ModelType model;
    af::MacGrid grid;
    af::FluidConfig fluid;
    af::FluidScratch scratch;
    af::PublishInput publish;
    std::unique_ptr<Simulation> sim;
    VehicleRef vehicle;
    std::optional<LivePropulsion> propulsion;  // set for a model with rotors
    double density = 0.0;                      // the case's, for the throttle map
    glm::vec3 grid_origin{0.0f};               // the grid's minimum corner, world frame
    std::vector<float> omega_cmd;              // this step's command, one per rotor
    std::vector<vehicles::RotorRow> rows;      // the rotors after the drone's step
    std::vector<af::ActuatorDisc> discs;       // their discs, in the grid's frame
    glm::quat q_bd{1.0f, 0.0f, 0.0f, 0.0f};  // the model's design -> principal, normalized
    BodyState held{};                        // the body row at the hold
    uint64_t h_ns = 0;
    float yaw = 0.0f;  // the held attitude, degrees
    float pitch = 0.0f;
    float roll = 0.0f;
    LiveState state;
    ControlProgram flown;
    std::string fault;  // non-empty once the fluid has faulted
};
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

LiveRun::LiveRun(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
LiveRun::LiveRun(LiveRun&&) noexcept = default;
LiveRun& LiveRun::operator=(LiveRun&&) noexcept = default;
LiveRun::~LiveRun() = default;

Result<LiveRun> LiveRun::start(const ResolvedCase& resolved, const vehicles::ModelType& model,
                               const std::optional<LivePropulsion>& propulsion) {
    const AnalysisCase& c = resolved.spec;
    if (!c.time.step || !(*c.time.step > 0.0f)) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "a live run needs the case's time.step: the solver's own step rule is "
                                     "Physics' to write, and doesn't exist yet"});
    }
    // L6: no path records still air under a drone that should be thrusting.
    if (!model.rotors.empty() && !propulsion) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "the model '" + model.name + "' has " + std::to_string(model.rotors.size()) +
                                         " rotors, and a live run needs their propulsion (the chain, idle and "
                                         "state of charge) to turn a throttle into rotor speed"});
    }
    if (propulsion) {
        // The throttle map's own checks, at rest, so a bad input is refused
        // here and not on the first step.
        if (Result<double> d = vehicles::armed_duty(false, 0.0, propulsion->idle); !d) {
            return std::unexpected(Error{d.error().code, "the propulsion's idle: " + d.error().context});
        }
        if (Result<vehicles::DutyCommand> at_rest =
                vehicles::rotor_omega_for_duty(propulsion->chain, 0.0, c.fluid.density, propulsion->soc);
            !at_rest) {
            return std::unexpected(Error{at_rest.error().code, "the propulsion: " + at_rest.error().context});
        }
    }
    auto impl = std::make_unique<Impl>();
    impl->resolved = resolved;
    impl->model = model;
    if (!model.rotors.empty()) impl->propulsion = propulsion;
    impl->density = c.fluid.density;
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
    impl->grid_origin = impl->publish.origin;
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
    impl->omega_cmd.assign(model.rotors.size(), 0.0f);
    impl->rows.resize(model.rotors.size());
    impl->discs.resize(model.rotors.size());
    impl->q_bd = glm::normalize(model.design_to_principal);
    const Result<const BodyState*> spawned = impl->sim->body(impl->vehicle.body);
    if (!spawned) return std::unexpected(spawned.error());
    impl->held = **spawned;
    const FrameState at_rest = to_body_state(FrameState{}, impl->q_bd, model.com_offset);
    impl->held.pos = at_rest.pos;
    impl->held.orient = at_rest.orient;
    impl->held.vel = at_rest.vel;
    impl->held.omega_body = at_rest.omega;
    return LiveRun(std::move(impl));
}

Result<void> LiveRun::step(const Controls& controls) {
    Impl& r = *impl_;
    if (!r.fault.empty()) return std::unexpected(Error{Code::internal, r.fault});

    // The rotors' command, before anything steps: the throttle through the
    // arm rule and the chain, the same speed for every rotor.
    uint32_t flags = 0;
    if (r.propulsion) {
        const Result<double> duty = vehicles::armed_duty(controls.armed, controls.throttle, r.propulsion->idle);
        if (!duty) return std::unexpected(duty.error());
        const Result<vehicles::DutyCommand> cmd =
            vehicles::rotor_omega_for_duty(r.propulsion->chain, *duty, r.density, r.propulsion->soc);
        if (!cmd) return std::unexpected(cmd.error());
        std::fill(r.omega_cmd.begin(), r.omega_cmd.end(), cmd->omega_cmd);
        flags = cmd->flags;
    }
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
    const FrameState body = to_body_state(design, r.q_bd, r.model.com_offset);
    if (Result<void> s = r.sim->set_body_state(r.vehicle.body, body.pos, body.orient, body.vel, body.omega); !s) {
        return std::unexpected(s.error());
    }
    if (!r.omega_cmd.empty()) {
        if (Result<void> s = r.sim->set_rotor_commands(r.vehicle, r.omega_cmd); !s) return std::unexpected(s.error());
    }
    if (Result<void> s = r.sim->step(1); !s) return std::unexpected(s.error());
    // The body as held, not as one step's gravity left it: what the rotors'
    // discs and the drawing pose from.
    const Result<const BodyState*> stepped = r.sim->body(r.vehicle.body);
    if (!stepped) return std::unexpected(stepped.error());
    r.held = **stepped;
    r.held.pos = body.pos;
    r.held.orient = body.orient;
    r.held.vel = body.vel;
    r.held.omega_body = body.omega;
    for (uint32_t i = 0; i < r.state.rotor_omega.size(); ++i) {
        const Result<const vehicles::RotorRow*> row = r.sim->rotor(r.vehicle, i);
        if (!row) return std::unexpected(row.error());
        r.rows[i] = **row;
        r.state.rotor_omega[i] = (*row)->omega;
    }

    // The rotors' discs at the held body, into the fluid's step.
    if (!r.rows.empty()) {
        if (Result<void> d = vehicles::rotor_discs(r.rows, r.held, r.grid_origin, r.resolved.base, r.discs); !d) {
            return std::unexpected(d.error());
        }
        for (const af::ActuatorDisc& d : r.discs) {
            af::spread_actuator(d, r.resolved.base, r.fluid.rho, r.fluid.h, r.scratch.du, r.scratch.dv, r.scratch.dw);
        }
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
    r.state.propulsion_flags = flags;
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

const BodyState& LiveRun::body() const noexcept { return impl_->held; }

std::string LiveRun::run_label() const {
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
    if (const std::string limits = announced_limits(r.state.propulsion_flags); !limits.empty()) {
        out += " \xc2\xb7 " + limits;
    }
    return out;
}

std::string LiveRun::run_line() const {
    return run_label() + std::format(" \xc2\xb7 t = {:.3f} s", impl_->state.t_s);
}

const ControlProgram& LiveRun::recorded_controls() const noexcept { return impl_->flown; }

Result<void> LiveRun::save(const std::filesystem::path& file) const {
    const Impl& r = *impl_;
    if (!r.fault.empty()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "a faulted run isn't saved (" + r.fault + "); its last good save is the one before"});
    }
    Writer w;
    w.out.append(kSaveMagic);
    w.put<uint32_t>(kLiveRunSaveVersion);
    w.put<uint64_t>(r.resolved.hash);
    w.put<uint64_t>(vehicles::model_identity(r.model));
    w.put<uint64_t>(vehicles::parts_identity(r.model));
    w.put<uint8_t>(r.propulsion ? 1 : 0);
    w.put<uint64_t>(r.propulsion ? vehicles::propulsion_identity(r.propulsion->chain) : 0);
    w.put<double>(r.propulsion ? r.propulsion->idle : 0.0);
    w.put<double>(r.propulsion ? r.propulsion->soc : 0.0);
    w.put<uint64_t>(r.h_ns);
    w.put<uint64_t>(r.state.steps);
    w.put<uint64_t>(r.state.frames);
    w.put(r.yaw);
    w.put(r.pitch);
    w.put(r.roll);
    w.put(r.state.attitude);
    w.put(r.state.omega_body);
    w.put<uint8_t>(r.state.armed ? 1 : 0);
    w.put<uint32_t>(r.state.propulsion_flags);
    w.put(r.held.pos);
    w.put(r.held.orient);
    w.put(r.held.vel);
    w.put(r.held.omega_body);
    std::string controls;
    if (!r.flown.points.empty()) {
        Result<std::string> text = program_to_yaml(r.flown);
        if (!text) return std::unexpected(text.error());
        controls = std::move(*text);
    }
    w.bytes(controls);
    const Result<SnapshotBlob> snapshot = r.sim->snapshot();
    if (!snapshot) return std::unexpected(snapshot.error());
    w.bytes(as_text(snapshot->bytes()));
    const std::vector<std::byte> grid = af::grid_to_bytes(r.grid);
    w.bytes(as_text(grid));
    w.put<uint64_t>(rng::fnv1a64(w.out));
    return detail::replace_file(file, w.out);
}

Result<LiveRun> LiveRun::restore(const std::filesystem::path& file, const ResolvedCase& resolved,
                                 const vehicles::ModelType& model, const std::optional<LivePropulsion>& propulsion) {
    const Result<std::string> read = detail::read_file_shared(file);
    if (!read) return std::unexpected(read.error());
    const std::string_view in(*read);
    const std::string name = "'" + path_text(file) + "'";
    const auto refuse = [&](Code code, const std::string& why) {
        return std::unexpected(Error{code, name + " " + why});
    };
    if (in.size() < kSaveMagic.size() + 4 || in.substr(0, kSaveMagic.size()) != kSaveMagic) {
        return refuse(Code::io_error, "is not a LiveRun save");
    }
    uint32_t version = 0;
    std::memcpy(&version, in.data() + kSaveMagic.size(), sizeof(version));
    if (version != kLiveRunSaveVersion) {
        return refuse(Code::schema_mismatch, "is a LiveRun save of version " + std::to_string(version) +
                                                 "; this build reads version " + std::to_string(kLiveRunSaveVersion));
    }
    const std::size_t head = kSaveMagic.size() + 4;
    if (in.size() < head + 8) return refuse(Code::invalid_argument, "is cut short");
    uint64_t stored = 0;
    std::memcpy(&stored, in.data() + in.size() - 8, sizeof(stored));
    if (stored != rng::fnv1a64(in.substr(0, in.size() - 8))) {
        return refuse(Code::invalid_argument, "fails its checksum: it is corrupted or cut short");
    }

    Reader rd{in.substr(head, in.size() - head - 8)};
    const uint64_t case_hash = rd.get<uint64_t>();
    const uint64_t model_id = rd.get<uint64_t>();
    const uint64_t parts_id = rd.get<uint64_t>();
    const bool powered = rd.get<uint8_t>() != 0;
    const uint64_t propulsion_id = rd.get<uint64_t>();
    const double idle = rd.get<double>();
    const double soc = rd.get<double>();
    if (!rd.ok) return refuse(Code::invalid_argument, "is cut short");
    if (case_hash != resolved.hash) {
        return refuse(Code::invalid_argument,
                      "is a save of another case (" + hex(case_hash) + "), not this one (" + hex(resolved.hash) + ")");
    }
    if (model_id != vehicles::model_identity(model)) {
        return refuse(Code::invalid_argument, "is a save of another model (" + hex(model_id) + "), not this one (" +
                                                  hex(vehicles::model_identity(model)) + ")");
    }
    if (parts_id != vehicles::parts_identity(model)) {
        return refuse(Code::invalid_argument, "is a save of another model's parts (" + hex(parts_id) + "), not these (" +
                                                  hex(vehicles::parts_identity(model)) + ")");
    }
    if (powered != propulsion.has_value()) {
        return refuse(Code::invalid_argument, powered ? "is a save of a run with its propulsion chain (" +
                                                            hex(propulsion_id) + "); restore it with that propulsion"
                                                      : "is a save of a run without a propulsion chain");
    }
    if (propulsion) {
        const uint64_t mine = vehicles::propulsion_identity(propulsion->chain);
        if (propulsion_id != mine) {
            return refuse(Code::invalid_argument, "is a save of another propulsion chain (" + hex(propulsion_id) +
                                                      "), not this one (" + hex(mine) + ")");
        }
        if (std::bit_cast<uint64_t>(idle) != std::bit_cast<uint64_t>(propulsion->idle)) {
            return refuse(Code::invalid_argument, std::format("flew at an idle of {}, not {}", idle, propulsion->idle));
        }
        if (std::bit_cast<uint64_t>(soc) != std::bit_cast<uint64_t>(propulsion->soc)) {
            return refuse(Code::invalid_argument,
                          std::format("flew at a state of charge of {}, not {}", soc, propulsion->soc));
        }
    }
    const uint64_t h_ns = rd.get<uint64_t>();
    const uint64_t steps = rd.get<uint64_t>();
    const uint64_t frames = rd.get<uint64_t>();
    const float yaw = rd.get<float>();
    const float pitch = rd.get<float>();
    const float roll = rd.get<float>();
    const glm::quat attitude = rd.quat();
    const glm::vec3 omega_body = rd.vec();
    const bool armed = rd.get<uint8_t>() != 0;
    const uint32_t propulsion_flags = rd.get<uint32_t>();
    const glm::vec3 held_pos = rd.vec();
    const glm::quat held_orient = rd.quat();
    const glm::vec3 held_vel = rd.vec();
    const glm::vec3 held_omega = rd.vec();
    const std::string_view controls = rd.bytes();
    const std::string_view snapshot = rd.bytes();
    const std::string_view grid = rd.bytes();
    if (!rd.ok || rd.at != rd.in.size()) return refuse(Code::invalid_argument, "is malformed");

    // The run as the case builds it, then its saved state over it.
    Result<LiveRun> run = start(resolved, model, propulsion);
    if (!run) return std::unexpected(run.error());
    Impl& r = *run->impl_;
    if (h_ns != r.h_ns) return refuse(Code::invalid_argument, "steps at another h");
    if (Result<void> g = af::grid_from_bytes(as_bytes(grid), r.grid); !g) return std::unexpected(g.error());
    Result<SnapshotBlob> blob =
        SnapshotBlob::from_bytes(std::vector<std::byte>(as_bytes(snapshot).begin(), as_bytes(snapshot).end()));
    if (!blob) return std::unexpected(blob.error());
    if (Result<void> s = r.sim->restore(*blob); !s) return std::unexpected(s.error());
    if (!controls.empty()) {
        Result<ControlProgram> program = program_from_yaml(controls);
        if (!program) return std::unexpected(program.error());
        r.flown = std::move(*program);
    }
    r.yaw = yaw;
    r.pitch = pitch;
    r.roll = roll;
    r.state.steps = steps;
    r.state.frames = frames;
    r.state.t_s = seconds(steps * r.h_ns);
    r.state.attitude = attitude;
    r.state.omega_body = omega_body;
    r.state.armed = armed;
    r.state.propulsion_flags = propulsion_flags;
    r.held.pos = held_pos;
    r.held.orient = held_orient;
    r.held.vel = held_vel;
    r.held.omega_body = held_omega;
    for (uint32_t i = 0; i < r.state.rotor_omega.size(); ++i) {
        const Result<const vehicles::RotorRow*> row = r.sim->rotor(r.vehicle, i);
        if (!row) return std::unexpected(row.error());
        r.state.rotor_omega[i] = (*row)->omega;
    }
    return run;
}

}  // namespace spade::analysis
