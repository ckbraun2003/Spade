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
// THE RECORDS (EDT-018): turbulence, contacts and the broad-phase grid are not
// in the world file until v3, so they come from the scene file's stem
// (builtin_records.hpp) or a default, and are not saved with the world.
//
// docs/design/interface/plans/2026-10-05-editor-design.md §5;
// 2026-10-05-editor-plan.md Task 7.

#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "builtin_records.hpp"
#include "editor_files.hpp"
#include "scene/compose.hpp"
#include "sim/simulation.hpp"

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
        const Result<scene::ComposedScene> composed = compose_together(scene, world);
        if (!composed) return std::unexpected(composed.error());
        Result<scene::SceneRun> run = scene::instantiate(*composed, instance_of(records), kBuiltinStepNs,
                                                         kBuiltinSubsteps);
        if (!run) return std::unexpected(run.error());
        std::vector<std::string> names;
        names.reserve(scene.vehicles.size());
        for (const scene::SceneVehicle& v : scene.vehicles) names.push_back(v.name);
        return EditorRun(std::make_unique<scene::SceneRun>(std::move(*run)), std::move(names), records);
    }

    [[nodiscard]] Result<void> step(uint32_t steps) {
        if (Result<void> s = run_->sim.step(steps); !s) return s;
        ticks_ += steps;
        return {};
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
        Result<EditorRun> next = start(carried, world, records_);
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

    [[nodiscard]] std::optional<std::size_t> index_of(std::string_view name) const {
        for (std::size_t i = 0; i < names_.size(); ++i) {
            if (names_[i] == name) return i;
        }
        return std::nullopt;
    }

    std::unique_ptr<scene::SceneRun> run_;  // never null; held by pointer so a rebuild swaps it whole
    std::vector<std::string> names_;  // scene order, parallel to run_->vehicles
    BuiltinRecords records_;
    uint64_t ticks_ = 0;
};

}  // namespace spade::sandbox::editor
