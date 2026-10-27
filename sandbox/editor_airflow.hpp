// ---------------------------------------------------------------------------
// editor_airflow.hpp -- the editor's airflow run (INT-15), display-free.
//
// The plan: docs/design/interface/plans/2026-10-22-editor-airflow-run-plan.md.
// T1: a region's case, as LiveRun will run it, and the line the inspector's
// Airflow section shows for it before it runs (the resolved grid, or the
// refusal verbatim), and the wireframe box the viewport draws around its
// drone.
// T2 (§9): AirflowRun, the air on a worker thread. The worker owns LiveRun
// alone; at each publish it hands the window a fresh, immutable AirFrame.
// ---------------------------------------------------------------------------
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <format>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "airflow_live.hpp"
#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/flow_volume.hpp"
#include "editor_gizmo.hpp"
#include "editor_panels.hpp"
#include "scene/scene_file.hpp"

namespace spade::sandbox::editor {

inline constexpr float kAirStepS = 0.001f;        // §4: airflow_live's step (P-2)
inline constexpr float kAirIdleDuty = 0.05f;      // §4: airflow_live's, until the chain pin (INT-11)
inline constexpr float kAirStateOfCharge = 1.0f;  // §4

namespace detail {

[[nodiscard]] inline Error air_refusal(const scene::SceneAirflowRegion& region, const std::string& why) {
    return Error{Code::invalid_argument, "airflow region '" + region.name + "': " + why};
}

// The region's drone and its model, or why there is none to run.
struct AirDrone {
    const scene::SceneVehicle* vehicle = nullptr;
    const vehicles::ModelType* model = nullptr;
};

[[nodiscard]] inline Result<AirDrone> air_drone(const scene::SceneDesc& s, const scene::SceneAirflowRegion& region) {
    if (region.kind != scene::AirflowKind::follow) {
        return std::unexpected(air_refusal(region, "Run air runs a follow region around a drone, and this one is fixed"));
    }
    const auto v = std::find_if(s.vehicles.begin(), s.vehicles.end(),
                                [&](const scene::SceneVehicle& x) { return x.name == region.follows; });
    if (v == s.vehicles.end()) {
        return std::unexpected(
            air_refusal(region, "it follows '" + region.follows + "', which is not a vehicle in this scene"));
    }
    const auto m = std::find_if(s.models.begin(), s.models.end(),
                                [&](const vehicles::ModelType& x) { return x.name == v->model; });
    if (m == s.models.end()) {
        return std::unexpected(air_refusal(region, "vehicle '" + v->name + "' uses model '" + v->model +
                                                       "', which is not in this scene"));
    }
    return AirDrone{&*v, &*m};
}

}  // namespace detail

// The model Run air flies for `region`: its drone's, with the parts out of
// the air (drawn, not solid), as the tour's chapter flies the reference
// quad: at the live grid a drone's arms and props are thinner than a cell
// (airflow_live.hpp). Physics' P-1, thin parts drawn only and thick ones
// solid, replaces this.
[[nodiscard]] inline Result<vehicles::ModelType> air_model(const scene::SceneDesc& s,
                                                           const scene::SceneAirflowRegion& region) {
    const Result<detail::AirDrone> drone = detail::air_drone(s, region);
    if (!drone) return std::unexpected(drone.error());
    vehicles::ModelType m = *drone->model;
    m.parts.clear();
    return m;
}

// The case Run air runs for `region`: its drone, held at the box's centre,
// with the region's grid and fluid. Refused, naming the region and why: a
// fixed region; a drone or model not in the scene; a drone placed turned
// (LiveRun starts it level, and the air's box never turns: the lead,
// 2026-10-26); a grid that is not a cube (the case's box is one length);
// rotors with no propulsion chain (LiveRun::start() refuses them; said here
// so the inspector shows it before Run air); or what analysis::resolve()
// refuses, its first issue verbatim.
[[nodiscard]] inline Result<analysis::ResolvedCase> air_case(const scene::SceneDesc& s,
                                                             const scene::SceneAirflowRegion& region) {
    const auto refused = [&](const std::string& why) { return std::unexpected(detail::air_refusal(region, why)); };
    const Result<detail::AirDrone> drone = detail::air_drone(s, region);
    if (!drone) return std::unexpected(drone.error());
    const scene::SceneVehicle* v = drone->vehicle;
    const vehicles::ModelType* m = drone->model;
    const glm::quat q = v->start.orient;
    if (std::abs(q.x) > 1e-6f || std::abs(q.y) > 1e-6f || std::abs(q.z) > 1e-6f) {
        return refused("'" + v->name +
                       "' is turned. Run air needs the drone unrotated: the air's box can't turn with it. Set its "
                       "rotation to zero to run it.");
    }
    const physics::airflow::GridShape& g = region.cells;
    if (g.nx != g.ny || g.ny != g.nz) {
        return refused(std::format("it is {}x{}x{} cells, and Run air's box is a cube, so the three must match", g.nx,
                                   g.ny, g.nz));
    }
    if (!m->rotors.empty() && !m->propulsion) {
        return refused("its drone's model '" + m->name + "' has no propulsion chain, so its rotors can't turn");
    }
    // The case is the flown model's (air_model(): parts out of the air), pinned
    // and resolved as LiveRun::start() will take it.
    const Result<vehicles::ModelType> flown = air_model(s, region);
    if (!flown) return std::unexpected(flown.error());
    analysis::AnalysisCase c;
    c.model = analysis::pin_model(v->model, *flown);
    c.power = analysis::Power{.idle = kAirIdleDuty, .state_of_charge = kAirStateOfCharge};
    c.fluid = analysis::Fluid{.density = region.density, .kinematic_viscosity = region.kinematic_viscosity};
    c.rotors.assign(flown->rotors.size(), analysis::RotorSetup{});
    c.grid.box = static_cast<float>(g.nx) * g.dx;
    c.grid.spacing = g.dx;
    c.time.step = kAirStepS;
    auto resolved = analysis::resolve(c, *flown);
    if (!resolved) {
        const analysis::CaseIssue& first = resolved.error().front();
        return refused(first.field + ": " + first.message);
    }
    return std::move(*resolved);
}

// A resolved case's grid as LiveRun's label gives it: "32³, dx 25 mm, R/dx 3".
[[nodiscard]] inline std::string grid_line(const analysis::ResolvedCase& r) {
    std::string out = std::format("{}\xc2\xb3, dx {:g} mm", r.base.nx, r.base.dx * 1000.0f);  // a superscript three
    if (!r.rotors.empty()) out += std::format(", R/dx {:.0f}", r.rotors.front().cells_across_radius);
    return out;
}

// The inspector's line for `region`: its grid_line(), or air_case()'s
// refusal verbatim.
[[nodiscard]] inline std::string region_line(const scene::SceneDesc& s, const scene::SceneAirflowRegion& region) {
    const Result<analysis::ResolvedCase> r = air_case(s, region);
    return r ? grid_line(*r) : r.error().context;
}

// What the worker hands the window at each publish (§9): a fresh, immutable
// copy, holding no view into the run, so a frame the window still holds
// outlives the run. Everything is where the drone is placed: the volume's
// origin, the held body and the rotor probes are shifted from the box's
// centre (LiveRun's frame) by the drone's position, before the hand-off and
// never after (Rendering's R-2). The held body is an alignas(16) std430
// row, so this holder gains padding; it is never uploaded or hashed
// (builtin_records.hpp disables C4324 the same way).
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
struct AirFrame {
    FlowVolume volume;
    uint64_t frames = 0;  // published so far, this one included
    uint64_t steps = 0;
    double t_s = 0.0;  // simulated
    BodyState body{};
    std::vector<end_goal::RotorProbe> probes;
    std::string run_label;
    std::string pressure_line;
    float speed_top = 1.0f;  // the colour scale's top: twice the hover's induced speed (airflow::speed_top())
    std::string error;  // "" while the air runs; else why it stopped, in the last frame
};
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

// The air on a worker thread (§9, the lead's D3 after M0: a 32^3 step is
// over a displayed frame's budget). The worker steps LiveRun one step at a
// time, so stop() waits at most one step, and each frame's last step and
// publish are advance_frame()'s, so a program's run is the one thread's to
// the bit. The window reads latest() each displayed frame. A refused step
// or publish, or an exception, ends the worker with its error in a last
// frame, never std::terminate. Not movable: the worker holds `this`.
class AirflowRun {
  public:
    // FOR TESTS: called with the run on the caller's thread before the
    // worker starts (LiveRun's test hooks).
    using Prepare = std::function<void(analysis::LiveRun&)>;

    // Builds the case and LiveRun on the caller's thread, then starts the
    // worker. With a program, each frame's controls are the program's at
    // the frame's start (the tour: deterministic); without one, the last
    // set_controls() (the Pilot section; unarmed until set). Refused before
    // any thread exists, as air_case() and LiveRun::start() refuse.
    [[nodiscard]] static Result<std::unique_ptr<AirflowRun>> start(
        const scene::SceneDesc& s, const scene::SceneAirflowRegion& region,
        std::optional<analysis::ControlProgram> program = std::nullopt, const Prepare& prepare = {}) {
        const Result<detail::AirDrone> drone = detail::air_drone(s, region);
        if (!drone) return std::unexpected(drone.error());
        const Result<analysis::ResolvedCase> resolved = air_case(s, region);
        if (!resolved) return std::unexpected(resolved.error());
        const Result<vehicles::ModelType> model = air_model(s, region);
        if (!model) return std::unexpected(model.error());
        Result<analysis::LiveRun> run = analysis::LiveRun::start(*resolved, *model);
        if (!run) return std::unexpected(detail::air_refusal(region, run.error().context));
        if (prepare) prepare(*run);
        const float top = airflow::speed_top(*model, region.density, 9.80665f);
        std::unique_ptr<AirflowRun> out(
            new AirflowRun(std::move(*run), drone->vehicle->start.pos, top, std::move(program)));
        AirflowRun* self = out.get();
        out->worker_ = std::jthread([self](std::stop_token stop) { self->work(stop); });
        return out;
    }

    AirflowRun(const AirflowRun&) = delete;
    AirflowRun& operator=(const AirflowRun&) = delete;
    ~AirflowRun() { stop(); }

    // The last frame handed over; none before the first publish.
    [[nodiscard]] std::shared_ptr<const AirFrame> latest() const {
        const std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }

    // The Pilot section's controls, read at each frame's start when the run
    // has no program.
    void set_controls(const analysis::Controls& controls) {
        const std::lock_guard<std::mutex> lock(mutex_);
        pilot_ = controls;
    }

    // Asks the worker to stop and waits for it: at most one step.
    void stop() {
        if (!worker_.joinable()) return;
        worker_.request_stop();
        worker_.join();
    }

  private:
    AirflowRun(analysis::LiveRun run, glm::vec3 at, float speed_top, std::optional<analysis::ControlProgram> program)
        : run_(std::move(run)), at_(at), speed_top_(speed_top), program_(std::move(program)) {}

    void work(const std::stop_token& stop) noexcept {
        try {
            try {
                loop(stop);
                return;
            } catch (const std::exception& e) {
                fail(std::string("an exception: ") + e.what());
            } catch (...) {
                fail("an exception");
            }
        } catch (...) {  // no memory even for the error: the worker ends, the last frame stands
        }
    }

    void loop(const std::stop_token& stop) {
        // A frame ends at round(k / 30 s) in whole ns, as advance_frame()
        // ends it; every step but the frame's last is one step() here, so a
        // stop waits one step. Were the two to disagree, advance_frame()
        // would only take more steps: the same run, a later stop.
        const auto h_ns = static_cast<uint64_t>(std::llround(run_.step_s() * 1e9));
        constexpr uint64_t kNs = 1'000'000'000;
        constexpr uint64_t kFps = 30;
        while (!stop.stop_requested()) {
            const analysis::Controls c = program_ ? analysis::controls_at(*program_, static_cast<float>(run_.state().t_s))
                                                  : pilot();
            const uint64_t end_ns = ((run_.state().frames + 1) * kNs + kFps / 2) / kFps;
            while ((run_.state().steps + 1) * h_ns < end_ns) {
                if (stop.stop_requested()) return;
                if (const Result<void> stepped = run_.step(c); !stepped) return fail(stepped.error().context);
            }
            if (const Result<analysis::LiveFrame> f = run_.advance_frame(c, scratch_); !f) return fail(f.error().context);
            hand_over();
        }
    }

    [[nodiscard]] analysis::Controls pilot() const {
        const std::lock_guard<std::mutex> lock(mutex_);
        return pilot_;
    }

    void hand_over() {
        auto frame = std::make_shared<AirFrame>();
        frame->volume = scratch_;  // a copy: the frame holds no view into the run
        frame->volume.header.origin += at_;
        frame->frames = run_.state().frames;
        frame->steps = run_.state().steps;
        frame->t_s = run_.state().t_s;
        frame->body = run_.body();
        frame->body.pos += at_;
        frame->probes = airflow::probes_of(run_);
        for (end_goal::RotorProbe& p : frame->probes) p.hub += at_;
        frame->run_label = run_.run_label();
        frame->speed_top = speed_top_;
        frame->pressure_line = run_.pressure_line();
        std::shared_ptr<const AirFrame> done = std::move(frame);
        const std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::move(done);
    }

    // The last frame again (or an empty one), with why the air stopped.
    void fail(const std::string& why) {
        const std::lock_guard<std::mutex> lock(mutex_);
        auto frame = latest_ ? std::make_shared<AirFrame>(*latest_) : std::make_shared<AirFrame>();
        frame->error = std::format("the air stopped at t = {:.3f} s: {}", run_.state().t_s, why);
        latest_ = std::move(frame);
    }

    analysis::LiveRun run_;  // the worker's alone once it starts
    glm::vec3 at_;
    float speed_top_;
    std::optional<analysis::ControlProgram> program_;
    FlowVolume scratch_;  // advance_frame()'s, the worker's
    mutable std::mutex mutex_;
    std::shared_ptr<const AirFrame> latest_;  // under mutex_
    analysis::Controls pilot_{};              // under mutex_
    std::jthread worker_;                     // last: joined first
};

// The inspector's line for `region` through `cache`, resolved again only
// when the scene's state (`opened`, `serial`) or the region's name changes:
// air_case() hashes and validates the model (the lead, 2026-10-26).
inline const AirLineCache& air_line(AirLineCache& cache, uint64_t opened, uint64_t serial, const scene::SceneDesc& s,
                                    const scene::SceneAirflowRegion& region) {
    if (cache.opened == opened && cache.serial == serial && cache.region == region.name) return cache;
    const Result<analysis::ResolvedCase> r = air_case(s, region);
    cache.opened = opened;
    cache.serial = serial;
    cache.region = region.name;
    cache.text = r ? grid_line(*r) : r.error().context;
    cache.refused = !r;
    ++cache.resolves;
    return cache;
}

// The band's speed (§9): simulated time over wall time across the last few
// frames handed over, as the window saw them ("air at 1/30 speed"). The wall
// clock is the caller's (the sandbox's steady_clock), never the run's.
class AirSpeed {
  public:
    // A frame seen at `wall_s`; one seen before (the same count) is ignored.
    void frame(uint64_t frames, double t_s, double wall_s) {
        if (!samples_.empty() && frames <= samples_.back().frames) return;
        samples_.push_back(Sample{frames, t_s, wall_s});
        if (samples_.size() > kWindow) samples_.pop_front();
    }

    [[nodiscard]] std::string line() const {
        if (samples_.empty()) return "waiting for the air's first frame";
        const double sim = samples_.back().t_s - samples_.front().t_s;
        const double wall = samples_.back().wall_s - samples_.front().wall_s;
        if (samples_.size() < 2 || !(wall > 0.0) || !(sim > 0.0)) return "the air is starting";
        const double ratio = sim / wall;
        if (ratio >= 1.0) return "air at full speed";
        return std::format("air at 1/{:.0f} speed", 1.0 / ratio);
    }

  private:
    struct Sample {
        uint64_t frames = 0;
        double t_s = 0.0;
        double wall_s = 0.0;
    };
    static constexpr std::size_t kWindow = 8;
    std::deque<Sample> samples_;
};

// A GizmoLine's axis for an airflow region's box (ui::axis_colour() draws it
// in its own colour, not an axis's).
inline constexpr int kRegionLine = 3;

// Each follow region's box as the viewport draws it, in a view of `width` x
// `height` pixels: its 12 edges, centred where its drone is placed (LiveRun
// holds the drone at the box's centre), n dx on a side. An edge with an end
// behind the camera is left out.
[[nodiscard]] inline std::vector<GizmoLine> region_box_lines(const scene::SceneDesc& s, const render::Camera& cam,
                                                            uint32_t width, uint32_t height) {
    std::vector<GizmoLine> lines;
    for (const scene::SceneAirflowRegion& region : s.airflow) {
        if (region.kind != scene::AirflowKind::follow) continue;
        const auto v = std::find_if(s.vehicles.begin(), s.vehicles.end(),
                                    [&](const scene::SceneVehicle& x) { return x.name == region.follows; });
        if (v == s.vehicles.end()) continue;
        const glm::vec3 half = 0.5f * region.cells.dx *
                               glm::vec3(static_cast<float>(region.cells.nx), static_cast<float>(region.cells.ny),
                                         static_cast<float>(region.cells.nz));
        const auto corner = [&](int i) {
            return v->start.pos + half * glm::vec3(i & 1 ? 1.0f : -1.0f, i & 2 ? 1.0f : -1.0f, i & 4 ? 1.0f : -1.0f);
        };
        for (int i = 0; i < 8; ++i) {
            for (const int bit : {1, 2, 4}) {
                if ((i & bit) != 0) continue;  // each edge once, from its low end
                const std::optional<glm::vec2> a = project(cam, corner(i), width, height);
                const std::optional<glm::vec2> b = project(cam, corner(i | bit), width, height);
                if (a && b) lines.push_back(GizmoLine{*a, *b, kRegionLine, false});
            }
        }
    }
    return lines;
}

}  // namespace spade::sandbox::editor
