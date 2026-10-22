// The editor's run: a Simulation built from the documents. Display-free
// (SL15b).
//
// NEVER EDITED IN PLACE (EDT-001). start() composes the scene with its world
// as they would be saved together (compose_together(): the pin re-aimed at
// the world's current hash, so unsaved world edits run) and instantiates it
// with the scene's physics records. rebuild() does the same for the edited
// documents and swaps the result in only when it succeeded: a refusal leaves
// this run exactly as it was and returns compose()'s or instantiate()'s error
// verbatim (EDT-009).
//
// THE CARRY (EDT-007) is by name. Each vehicle that keeps its name starts the
// new run where it was: its design-frame pose, velocity and rates
// (vehicle_state(), DBE-013), and its rotors' mean speed, which is also the
// command they hold (VehicleSpawn::rotor_omega is one scalar; per-rotor speeds
// wait on Core, editor plan Task 16). A renamed vehicle is a new vehicle and
// starts at its scene start; a removed one is dropped. The tick count
// continues across rebuilds; the new Simulation's own clock starts at 0.
//
// THE PILOT (N17). Each vehicle whose model has a propulsion chain is flown by
// its own flight controller (flight_controller.hpp), commanded before every
// step: the piloted one with the pilot's Controls, every other one disarmed.
// A fresh start spawns them at rest, their rotors at exactly 0 (the run's
// copy; the scene is not changed); a rebuild carries them as they were. A
// chainless vehicle keeps the command it spawned holding. A model the
// controller refuses (its layout, its chain) is not flown, and why_not() says
// why.
//
// THE RECORDS (EDT-018): turbulence, contacts and the broad-phase grid are not
// in the world file until v3, so they come from the scene file's stem
// (builtin_records.hpp) or a default, and are not saved with the world.
//
// docs/design/interface/plans/2026-10-05-editor-design.md §5;
// 2026-10-05-editor-plan.md Task 7.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "analysis/case.hpp"
#include "builtin_records.hpp"
#include "editor_files.hpp"
#include "flight_controller.hpp"
#include "scene/compose.hpp"
#include "sim/simulation.hpp"
#include "state/layout.hpp"

namespace spade::sandbox::editor {

using spade::sandbox::BuiltinRecords;

// The records a scene file runs with: its built-in records when its stem has
// them, else the quadrotor scenes' records, a valid default for a scene the
// editor saved. Deleted with world file v3, which carries them.
[[nodiscard]] inline BuiltinRecords records_for(const std::filesystem::path& scene_file) {
    std::string stem = scene_file.filename().string();
    if (const std::size_t dot = stem.find('.'); dot != std::string::npos) stem.resize(dot);
    if (std::optional<BuiltinRecords> r = builtin_records(stem)) return *r;
    return *builtin_records("hover");
}

class EditorRun {
  public:
    [[nodiscard]] static Result<EditorRun> start(const scene::SceneDesc& scene, const WorldDesc& world,
                                                 const BuiltinRecords& records) {
        return build(scene, world, records, true);
    }

    // `steps` steps. Before each, every flown vehicle's controller commands
    // its rotors: `piloted` with `controls`, every other one disarmed.
    [[nodiscard]] Result<void> step(uint32_t steps, std::string_view piloted = {},
                                    const analysis::Controls& controls = {}) {
        const bool any = std::any_of(flight_.begin(), flight_.end(), [](const auto& f) { return f.has_value(); });
        if (!any) {
            if (Result<void> s = run_->sim.step(steps); !s) return s;
            ticks_ += steps;
            return {};
        }
        std::vector<float> omega;
        for (uint32_t s = 0; s < steps; ++s) {
            for (std::size_t i = 0; i < flight_.size(); ++i) {
                if (!flight_[i]) continue;
                const VehicleRef& ref = run_->vehicles[i];
                const Result<const BodyState*> body = run_->sim.body(ref.body);
                if (!body) return std::unexpected(body.error());
                omega.assign(ref.rotor_count, 0.0f);
                const analysis::Controls c = names_[i] == piloted ? controls : analysis::Controls{};
                if (Result<void> r = flight_[i]->command(c, **body, omega); !r) return r;
                if (Result<void> r = run_->sim.set_rotor_commands(ref, omega); !r) return r;
            }
            if (Result<void> r = run_->sim.step(1); !r) return r;
            ++ticks_;
        }
        return {};
    }

    // Whether Run flies `name` (a drone whose chain the controller took).
    [[nodiscard]] bool flies(std::string_view name) const {
        const std::optional<std::size_t> i = index_of(name);
        return i && flight_[*i].has_value();
    }

    // Why `name` isn't flown; empty when it is, or when there is no such vehicle.
    [[nodiscard]] std::string why_not(std::string_view name) const {
        const std::optional<std::size_t> i = index_of(name);
        return i ? why_not_[*i] : std::string();
    }

    // The controller flying `name`; null when Run doesn't fly it. Made again by
    // every rebuild, so it holds the scene's chain as it is now.
    [[nodiscard]] const flight::FlightController* controller_of(std::string_view name) const {
        const std::optional<std::size_t> i = index_of(name);
        return i && flight_[*i] ? &*flight_[*i] : nullptr;
    }

    // The first vehicle Run flies, in scene order.
    [[nodiscard]] std::optional<std::string> first_flown() const {
        for (std::size_t i = 0; i < names_.size(); ++i) {
            if (flight_[i]) return names_[i];
        }
        return std::nullopt;
    }

    // Builds a run from the edited documents, carrying each vehicle that keeps
    // its name, and swaps it in. On a refusal this run is unchanged.
    [[nodiscard]] Result<void> rebuild(const scene::SceneDesc& scene, const WorldDesc& world) {
        scene::SceneDesc carried = scene;
        for (scene::SceneVehicle& v : carried.vehicles) {
            const std::optional<std::size_t> i = index_of(v.name);
            if (!i) continue;
            const VehicleRef& ref = run_->vehicles[*i];
            const Result<FrameState> state = run_->sim.vehicle_state(ref);
            if (!state) return std::unexpected(state.error());
            float sum = 0.0f;
            for (uint32_t r = 0; r < ref.rotor_count; ++r) {
                const Result<const vehicles::RotorRow*> row = run_->sim.rotor(ref, r);
                if (!row) return std::unexpected(row.error());
                sum += (*row)->omega;
            }
            v.start.pos = state->pos;
            v.start.orient = state->orient;
            v.start.vel = state->vel;
            v.start.omega_body = state->omega;
            v.start.rotor_omega = ref.rotor_count == 0 ? 0.0f : sum / static_cast<float>(ref.rotor_count);
        }
        Result<EditorRun> next = build(carried, world, records_, false);
        if (!next) return std::unexpected(next.error());
        const uint64_t ticks = ticks_;
        *this = std::move(*next);
        ticks_ = ticks;
        return {};
    }

    // Steps taken since start(), across rebuilds.
    [[nodiscard]] uint64_t tick() const noexcept { return ticks_; }
    [[nodiscard]] const Simulation& sim() const noexcept { return run_->sim; }
    [[nodiscard]] std::size_t vehicle_count() const noexcept { return names_.size(); }

    [[nodiscard]] std::optional<VehicleRef> vehicle_ref(std::string_view name) const {
        const std::optional<std::size_t> i = index_of(name);
        if (!i) return std::nullopt;
        return run_->vehicles[*i];
    }

    // The named vehicle's state in its design frame, or none if the run has no
    // vehicle of that name.
    [[nodiscard]] std::optional<FrameState> state_of(std::string_view name) const {
        const std::optional<std::size_t> i = index_of(name);
        if (!i) return std::nullopt;
        const Result<FrameState> state = run_->sim.vehicle_state(run_->vehicles[*i]);
        if (!state) return std::nullopt;
        return *state;
    }

    // Whether the physics records are saved with the world (EDT-018): not
    // until world file v3 carries them.
    [[nodiscard]] static constexpr bool records_saved_with_world() noexcept { return false; }

  private:
    EditorRun(std::unique_ptr<scene::SceneRun> run, std::vector<std::string> names, const BuiltinRecords& records)
        : run_(std::move(run)), names_(std::move(names)), records_(records) {}

    // start() (`fresh`: chained vehicles spawn at rest) and rebuild() (they
    // are carried as they were).
    [[nodiscard]] static Result<EditorRun> build(const scene::SceneDesc& scene, const WorldDesc& world,
                                                 const BuiltinRecords& records, bool fresh) {
        const auto model_of = [&](const scene::SceneVehicle& v) -> const vehicles::ModelType* {
            for (const vehicles::ModelType& m : scene.models) {
                if (m.name == v.model) return &m;
            }
            return nullptr;
        };
        scene::SceneDesc run_scene = scene;
        if (fresh) {
            for (scene::SceneVehicle& v : run_scene.vehicles) {
                const vehicles::ModelType* m = model_of(v);
                if (m != nullptr && m->propulsion) v.start.rotor_omega = 0.0f;
            }
        }
        const Result<scene::ComposedScene> composed = compose_together(run_scene, world);
        if (!composed) return std::unexpected(composed.error());
        Result<scene::SceneRun> run = scene::instantiate(*composed, instance_of(records), kBuiltinStepNs,
                                                         kBuiltinSubsteps);
        if (!run) return std::unexpected(run.error());
        // Flushed, so the controllers can command before the first step.
        if (Result<void> flushed = run->sim.flush_structural(); !flushed) return std::unexpected(flushed.error());
        std::vector<std::string> names;
        names.reserve(scene.vehicles.size());
        for (const scene::SceneVehicle& v : scene.vehicles) names.push_back(v.name);
        EditorRun out(std::make_unique<scene::SceneRun>(std::move(*run)), std::move(names), records);
        out.flight_.resize(scene.vehicles.size());
        out.why_not_.resize(scene.vehicles.size());
        for (std::size_t i = 0; i < scene.vehicles.size(); ++i) {
            const vehicles::ModelType* m = model_of(scene.vehicles[i]);
            if (m == nullptr || !m->propulsion) {
                out.why_not_[i] = "'" + scene.vehicles[i].name + "' has no propulsion chain to fly with";
                continue;
            }
            Result<flight::FlightController> fc =
                flight::FlightController::make(*m, world.environment.air_density);
            if (fc) {
                out.flight_[i] = std::move(*fc);
            } else {
                out.why_not_[i] = "'" + scene.vehicles[i].name + "' can't be flown: " + fc.error().context;
            }
        }
        return out;
    }

    [[nodiscard]] std::optional<std::size_t> index_of(std::string_view name) const {
        for (std::size_t i = 0; i < names_.size(); ++i) {
            if (names_[i] == name) return i;
        }
        return std::nullopt;
    }

    std::unique_ptr<scene::SceneRun> run_;  // never null; held by pointer so a rebuild swaps it whole
    std::vector<std::string> names_;  // scene order, parallel to run_->vehicles
    std::vector<std::optional<flight::FlightController>> flight_;  // parallel: the flown vehicles' controllers
    std::vector<std::string> why_not_;                             // parallel: why one isn't flown
    BuiltinRecords records_;
    uint64_t ticks_ = 0;
};

}  // namespace spade::sandbox::editor
