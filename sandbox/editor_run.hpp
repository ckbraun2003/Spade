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
// THE PROPS (L7c2, Core's posing rule): each vehicle's blade azimuths, one
// per rotor, held here outside the Simulation: 0 at start(), turned after
// every step by P11's advance_blade_azimuths() with the rows the step left,
// carried by name on a rebuild. Drawn only: nothing reads them back, so a
// Run steps one at a time whenever a vehicle has rotors, bit for bit as a
// batch would.
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
#include <format>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "analysis/case.hpp"
#include "builtin_records.hpp"
#include "editor_files.hpp"
#include "flight_controller.hpp"
#include "physics/blade_azimuth.hpp"
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
    // its rotors: `piloted` with `controls`, every other one disarmed. After
    // each, every vehicle's props turn (turn_props()). With no vehicle flown
    // and no rotor, the steps go in one batch.
    [[nodiscard]] Result<void> step(uint32_t steps, std::string_view piloted = {},
                                    const analysis::Controls& controls = {}) {
        const bool any = std::any_of(flight_.begin(), flight_.end(), [](const auto& f) { return f.has_value(); });
        const bool props = std::any_of(psi_.begin(), psi_.end(), [](const auto& p) { return !p.empty(); });
        if (!any && !props) {
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
            if (Result<void> r = turn_props(); !r) return r;
        }
        return {};
    }

    // The named vehicle's blade azimuths, one per rotor in its model's order,
    // in [0, 2 pi) (L7c2): 0 at start(), turned after every step, carried by
    // a rebuild while its rotor count holds. Drawn only. Empty for a vehicle
    // the run doesn't have.
    [[nodiscard]] std::span<const float> blade_azimuth(std::string_view name) const {
        const std::optional<std::size_t> i = index_of(name);
        return i ? std::span<const float>(psi_[*i]) : std::span<const float>();
    }

    // Its rotors' omegas as the last step left them (rad/s): what
    // posed_parts() takes with blade_azimuth(). Empty likewise.
    [[nodiscard]] std::span<const float> rotor_omega(std::string_view name) const {
        const std::optional<std::size_t> i = index_of(name);
        return i ? std::span<const float>(omega_[*i]) : std::span<const float>();
    }

    // The first step whose azimuths were refused, named, or "": a refused
    // rotor holds and the run goes on (psi is drawn only), and the status
    // line shows this once. A stepping run never makes one.
    [[nodiscard]] const std::string& azimuth_refused() const noexcept { return azimuth_refused_; }

    // FOR TESTS ONLY: sets the named vehicle's azimuths (another count is
    // ignored), so a test can put one outside [0, 2 pi) and see it refused.
    void set_test_blade_azimuth(std::string_view name, std::span<const float> psi) {
        const std::optional<std::size_t> i = index_of(name);
        if (i && psi.size() == psi_[*i].size()) std::copy(psi.begin(), psi.end(), psi_[*i].begin());
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
        // The props carry by name, while a vehicle's rotor count holds.
        for (std::size_t j = 0; j < next->names_.size(); ++j) {
            const std::optional<std::size_t> i = index_of(next->names_[j]);
            if (i && psi_[*i].size() == next->psi_[j].size()) next->psi_[j] = psi_[*i];
        }
        next->azimuth_refused_ = azimuth_refused_;
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

    // The grid's cell this run uses, and the vehicle it was sized for: ""
    // when the records' own cell fits every drone (build()).
    [[nodiscard]] float grid_cell() const noexcept { return cell_; }
    [[nodiscard]] const std::string& cell_sized_for() const noexcept { return sized_for_; }

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
        // The grid's cell fits the scene's largest drone: spawn() refuses a
        // cell under twice a model's proxy radius (the Phantom's 0.296 m, the
        // Iris+'s 0.365 m, against the editor's 0.5 m). Never under the
        // records' own cell, so a scene whose drones fit runs as it did.
        BuiltinRecords sized = records;
        std::string sized_for;
        for (const scene::SceneVehicle& v : scene.vehicles) {
            const vehicles::ModelType* m = model_of(v);
            if (m != nullptr && 2.0f * m->proxy_radius > sized.grid.cell_size) {
                sized.grid.cell_size = 2.0f * m->proxy_radius;
                sized_for = v.name;
            }
        }
        const Result<scene::ComposedScene> composed = compose_together(run_scene, world);
        if (!composed) return std::unexpected(composed.error());
        Result<scene::SceneRun> run = scene::instantiate(*composed, instance_of(sized), kBuiltinStepNs,
                                                         kBuiltinSubsteps);
        if (!run) return std::unexpected(run.error());
        // Flushed, so the controllers can command before the first step.
        if (Result<void> flushed = run->sim.flush_structural(); !flushed) return std::unexpected(flushed.error());
        std::vector<std::string> names;
        names.reserve(scene.vehicles.size());
        for (const scene::SceneVehicle& v : scene.vehicles) names.push_back(v.name);
        EditorRun out(std::make_unique<scene::SceneRun>(std::move(*run)), std::move(names), records);
        out.cell_ = sized.grid.cell_size;
        out.sized_for_ = std::move(sized_for);
        out.flight_.resize(scene.vehicles.size());
        out.why_not_.resize(scene.vehicles.size());
        out.psi_.resize(scene.vehicles.size());
        out.omega_.resize(scene.vehicles.size());
        for (std::size_t i = 0; i < scene.vehicles.size(); ++i) {
            out.psi_[i].assign(out.run_->vehicles[i].rotor_count, 0.0f);
            out.omega_[i].assign(out.run_->vehicles[i].rotor_count, 0.0f);
            if (Result<void> rows = out.read_rows(i); !rows) return std::unexpected(rows.error());
        }
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

    // Vehicle i's rotor rows into rows_, and their omegas into omega_[i].
    [[nodiscard]] Result<void> read_rows(std::size_t i) {
        const VehicleRef& ref = run_->vehicles[i];
        rows_.resize(ref.rotor_count);
        for (uint32_t r = 0; r < ref.rotor_count; ++r) {
            const Result<const vehicles::RotorRow*> row = run_->sim.rotor(ref, r);
            if (!row) return std::unexpected(row.error());
            rows_[r] = **row;
            omega_[i][r] = (*row)->omega;
        }
        return {};
    }

    // Each vehicle's blade azimuths by P11's rule, with the rows as the step
    // left them and h the step's duration (L7c2, Core's posing rule). Drawn
    // only: nothing is read back. A refused rotor holds and the step stands;
    // azimuth_refused() names the first.
    [[nodiscard]] Result<void> turn_props() {
        const float h = run_->sim.substep_h() * static_cast<float>(run_->sim.substeps());
        for (std::size_t i = 0; i < psi_.size(); ++i) {
            if (psi_[i].empty()) continue;
            if (Result<void> rows = read_rows(i); !rows) return rows;
            if (!physics::advance_blade_azimuths(psi_[i], rows_, h) && azimuth_refused_.empty()) {
                azimuth_refused_ = std::format(
                    "tick {}: a blade azimuth of '{}' was refused (advance_blade_azimuths()); it holds", ticks_,
                    names_[i]);
            }
        }
        return {};
    }

    std::unique_ptr<scene::SceneRun> run_;  // never null; held by pointer so a rebuild swaps it whole
    std::vector<std::string> names_;  // scene order, parallel to run_->vehicles
    std::vector<std::optional<flight::FlightController>> flight_;  // parallel: the flown vehicles' controllers
    std::vector<std::string> why_not_;                             // parallel: why one isn't flown
    std::vector<std::vector<float>> psi_;    // parallel: each vehicle's blade azimuths (L7c2), drawn only
    std::vector<std::vector<float>> omega_;  // parallel: its rotors' omegas as the last step left them
    std::vector<vehicles::RotorRow> rows_;   // read_rows()' scratch
    std::string azimuth_refused_;            // azimuth_refused()
    BuiltinRecords records_;
    float cell_ = 0.0f;      // grid_cell()
    std::string sized_for_;  // cell_sized_for()
    uint64_t ticks_ = 0;
};

// The inspector's Grid cell row (the lead, 2026-10-27): the records' cell,
// or, while a Run whose cell was sized for a drone is going, the run's and
// why.
[[nodiscard]] inline std::string grid_cell_line(const BuiltinRecords& records, const EditorRun* run) {
    if (run != nullptr && !run->cell_sized_for().empty()) {
        return std::format("{:.3f} m, sized for '{}'; the records' {:.3f} m", run->grid_cell(), run->cell_sized_for(),
                           records.grid.cell_size);
    }
    return std::format("{:.3f} m", records.grid.cell_size);
}

}  // namespace spade::sandbox::editor
