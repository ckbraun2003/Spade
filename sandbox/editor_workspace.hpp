// The editor's workspace: the open scene and world, the run, and what the
// panels ask for, decided. Display-free (SL15b), so every rule here is tested
// without a window; editor_session.cpp draws it.
//
// The panels queue (editor_panels.hpp) and the workspace applies, once per
// frame (INT-2):
// - edits go to their document; a refusal changes nothing and is shown
//   verbatim in the status line;
// - undo and redo follow the order of the edits across the scene and the
//   world, as one history;
// - a click in the view places the armed library item, resting where the
//   click meets the ground, or selects what is under it (editor_pick.hpp);
// - run, pause, step and stop drive an EditorRun. An edit while running
//   rebuilds the run with each vehicle's state carried (Task 7).
// The composed scene is redone whenever either document changes, and
// composition() says when, so the view uploads only then.
//
// docs/design/interface/plans/2026-10-05-editor-plan.md Task 10.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "builtin_records.hpp"
#include "core/error.hpp"
#include "drone_view.hpp"
#include "editor_airflow.hpp"
#include "editor_document.hpp"
#include "editor_files.hpp"
#include "editor_import.hpp"
#include "editor_library.hpp"
#include "editor_library_save.hpp"
#include "editor_panels.hpp"
#include "editor_pick.hpp"
#include "editor_run.hpp"
#include "editor_scene_edits.hpp"
#include "editor_world_edits.hpp"
#include "pilot_keys.hpp"
#include "scene/compose.hpp"
#include "world/sdf.hpp"

namespace spade::sandbox::editor {

// One Step on the toolbar: a 60 Hz frame's worth of built-in steps.
inline constexpr uint64_t kStepTicks = 4;

namespace detail {

// An asset's half extents about its origin, before its pose: a one-primitive
// collider's own, or a unit's.
[[nodiscard]] inline glm::vec3 asset_half_extents(const scene::SceneAsset& a) {
    if (a.collider.nodes.size() != 1) return glm::vec3(0.5f);
    const SdfNode& n = a.collider.nodes[0];
    const glm::vec4 q = n.params;
    switch (static_cast<SdfPrim>(n.kind)) {
        case SdfPrim::sphere: return glm::vec3(q.x);
        case SdfPrim::box: return glm::vec3(q.x, q.y, q.z);
        case SdfPrim::cylinder: return glm::vec3(q.x, q.y, q.x);
        case SdfPrim::capsule: return glm::vec3(q.x, q.x + q.y, q.x);
        case SdfPrim::torus: return glm::vec3(q.x + q.y, q.y, q.x + q.y);
        default: return glm::vec3(0.5f);
    }
}

}  // namespace detail

// Where "Frame selected" looks, and how much it must fit.
struct Framing {
    glm::vec3 centre{0.0f};
    float radius = 0.0f;
};

class EditorWorkspace {
  public:
    // Opens a workspace on `opened`, refusing one that will not compose.
    [[nodiscard]] static Result<std::unique_ptr<EditorWorkspace>> create(OpenedScene opened) {
        std::unique_ptr<EditorWorkspace> w(new EditorWorkspace(std::move(opened)));
        if (Result<void> composed = w->recompose(); !composed) return std::unexpected(composed.error());
        w->list_sources();
        w->load_library();
        w->refresh_panels();
        return w;
    }

    // N11: the meshes the scene's world resolved past the scene's own folder
    // (the library's), which Save copies beside the scene; the session sets
    // them whenever it resolves. And the assets folder, when a built-in scene
    // told where it is (assets/library/meshes is the library's mesh root).
    void set_fallback_meshes(std::vector<render::FallbackMesh> meshes) { fallback_meshes_ = std::move(meshes); }
    [[nodiscard]] const std::filesystem::path& assets_dir() const noexcept { return assets_; }
    // N15's staging folder (D1): the view's second root, after the scene's
    // folder; Save copies what the scene uses from it beside the scene.
    [[nodiscard]] const std::filesystem::path& import_staging() const noexcept { return staging_.path(); }
    // Keeps the staging folder from looking like a crashed session's
    // (ImportStaging::keep_alive()); apply_panels() calls it each frame.
    void keep_staging_alive(std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now()) {
        staging_.keep_alive(now);
    }
    [[nodiscard]] EditorPanels& panels() noexcept { return panels_; }
    [[nodiscard]] const EditorPanels& panels() const noexcept { return panels_; }
    [[nodiscard]] const OpenedScene& documents() const noexcept { return docs_; }
    [[nodiscard]] const EditorRun* run() const noexcept { return run_ ? &*run_ : nullptr; }
    // Run air's (INT-15): the region's air on its worker; none when stopped.
    [[nodiscard]] const AirflowRun* air() const noexcept { return air_.get(); }
    // Where Run air's fluid steps (slice 3): the window gives the GPU's
    // starter (editor_air_gpu.hpp); until given one, the CPU's.
    void set_air_starter(AirStarter starter) { air_starter_ = std::move(starter); }
    [[nodiscard]] const scene::ComposedScene& composed() const noexcept { return composed_; }
    [[nodiscard]] const BuiltinRecords& records() const noexcept { return records_; }
    // Changes whenever composed() does.
    [[nodiscard]] uint64_t composition() const noexcept { return composition_; }

    // Applies what the panels queued: the edits in order, then the commands.
    void apply_panels() {
        for (const SceneEdit& e : std::exchange(panels_.pending, {})) {
            const Serials before = serials();
            settle(before, apply(docs_.scene, e));
        }
        for (const WorldEdit& e : std::exchange(panels_.pending_world, {})) {
            const Serials before = serials();
            settle(before, apply(docs_.world, e));
        }
        for (const Command c : std::exchange(panels_.commands, {})) command(c);
        keep_staging_alive();
        refresh_panels();
        // A selection ends a part's or a drone's inspection (the lead's Q1),
        // and so does the library moving under it.
        if (panels_.inspected && (panels_.selection.kind != Selection::Kind::none ||
                                  *panels_.inspected >= panels_.library.size() ||
                                  !has_grades(panels_.library[*panels_.inspected]))) {
            panels_.inspected.reset();
        }
    }

    // A click in the view along the ray from `origin` in `direction`.
    void click(glm::vec3 origin, glm::vec3 direction) {
        if (panels_.armed) {
            place_armed(origin, direction);
        } else {
            const std::optional<std::string> hit = pick(docs_.scene.desc(), composed_, run(), origin, direction);
            panels_.selection = hit ? Selection{Selection::Kind::object, *hit} : Selection{};
        }
        refresh_panels();
    }

    // A gizmo drag, one undo step however many poses it passes through: its
    // first change is an ordinary edit, and each one after amends it. A
    // refusal shows and keeps the last pose that was accepted.
    void drag(const SceneEdit& e) {
        const Serials before = serials();
        if (!drag_step_) {
            drag_step_ = settle(before, apply(docs_.scene, e));
        } else if (const Result<void> amended = apply(docs_.scene, e, true); !amended) {
            panels_.status = amended.error().context;
        } else {
            panels_.status.clear();
            if (serials().scene != before.scene) after_change();
        }
        refresh_panels();
    }
    // The drag is over; the next one is a step of its own.
    void end_drag() noexcept { drag_step_ = false; }

    // Delete and Ctrl+D act on the selected object.
    void key_delete() {
        if (panels_.selection.kind != Selection::Kind::object) return;
        const Serials before = serials();
        settle(before, apply(docs_.scene, RemoveObject{panels_.selection.name}));
        refresh_panels();
    }
    void key_duplicate() {
        if (panels_.selection.kind != Selection::Kind::object) return;
        const scene::SceneDesc& s = docs_.scene.desc();
        const bool vehicle = find_vehicle(s, panels_.selection.name) != nullptr;
        const Serials before = serials();
        if (settle(before, apply(docs_.scene, DuplicateObject{panels_.selection.name}))) {
            const scene::SceneDesc& now = docs_.scene.desc();
            panels_.selection.name = vehicle ? now.vehicles.back().name : now.assets.back().name;
        }
        refresh_panels();
    }

    // N17: the pilot's keys for a frame of `dt` seconds, acting only while Run
    // flies a drone. Called every frame, with no keys held when ImGui has the
    // keyboard, so a released arrow levels the drone.
    void pilot_keys(const flight::PilotKeys& keys, float dt) {
        if ((!run_ && !air_) || panels_.pilot.vehicle.empty()) return;
        flight::apply_pilot_keys(keys, dt, panels_.pilot.controls);
    }

    // Steps the run by `dt` seconds of whole built-in steps, the remainder
    // carried, while it runs and is not paused.
    void advance(float dt) {
        if (air_) air_->set_controls(panels_.pilot.controls);  // the worker reads them at its next frame
        if (!run_ || paused_ || !(dt > 0.0f) || !std::isfinite(dt)) return;
        carry_ns_ += static_cast<uint64_t>(std::llround(static_cast<double>(dt) * 1e9));
        const uint64_t steps = carry_ns_ / kBuiltinStepNs;
        carry_ns_ %= kBuiltinStepNs;
        if (steps > 0) step_run(steps);
        refresh_panels();
    }

    // The panels' read side, from the documents and the run.
    void refresh_panels() {
        const scene::SceneDesc& s = docs_.scene.desc();
        panels_.scene = &s;
        panels_.scene_opened = opened_;
        panels_.scene_serial = docs_.scene.serial();
        panels_.world = &docs_.world.desc();
        panels_.records = &records_;
        panels_.grid_cell = grid_cell_line(records_, run_ ? &*run_ : nullptr);
        const std::string file = docs_.scene.path.empty() ? std::string("untitled") : docs_.scene.path.filename().string();
        panels_.title = file + (docs_.scene.dirty() || docs_.world.dirty() ? " *" : "");
        panels_.running = run_.has_value() || air_ != nullptr;
        panels_.air = air_ != nullptr;
        panels_.paused = paused_;
        panels_.tick = run_ ? run_->tick() : 0u;
        panels_.can_undo = !undo_.empty();
        panels_.can_redo = !redo_.empty();
        panels_.hierarchy.clear();
        panels_.hierarchy.push_back(HierarchyRow{Selection{Selection::Kind::scene, {}}, s.name, false});
        panels_.hierarchy.push_back(HierarchyRow{Selection{Selection::Kind::world, {}}, docs_.world.desc().name, false});
        for (const scene::SceneAsset& a : s.assets) {
            panels_.hierarchy.push_back(HierarchyRow{Selection{Selection::Kind::object, a.name}, a.name, false});
        }
        for (const scene::SceneVehicle& v : s.vehicles) {
            panels_.hierarchy.push_back(HierarchyRow{Selection{Selection::Kind::object, v.name}, v.name, true});
        }
        if (panels_.selection.kind == Selection::Kind::object && find_vehicle(s, panels_.selection.name) == nullptr &&
            find_asset(s, panels_.selection.name) == nullptr) {
            panels_.selection = Selection{};
        }
        if (panels_.armed && *panels_.armed >= panels_.library.size()) panels_.armed.reset();
        panels_.selected_source.clear();
        if (panels_.selection.kind == Selection::Kind::object) {
            const scene::SceneAsset* a = find_asset(s, panels_.selection.name);
            if (a != nullptr) panels_.selected_source = import_source(a->visual.mesh_ref);
        }
    }

    // The selected object's framing: a vehicle within its reach, where the run
    // has it; an asset within its posed box. None for nothing, the scene or
    // the world, which have no place of their own.
    [[nodiscard]] std::optional<Framing> selection_framing() const {
        if (panels_.selection.kind != Selection::Kind::object) return std::nullopt;
        const scene::SceneDesc& s = docs_.scene.desc();
        if (const scene::SceneVehicle* v = find_vehicle(s, panels_.selection.name)) {
            for (const vehicles::ModelType& m : s.models) {
                if (m.name != v->model) continue;
                glm::vec3 at = v->start.pos;
                if (run_) {
                    if (const std::optional<FrameState> now = run_->state_of(v->name)) at = now->pos;
                }
                return Framing{at, detail::reach_of(m)};
            }
            return std::nullopt;
        }
        if (const scene::SceneAsset* a = find_asset(s, panels_.selection.name)) {
            return Framing{a->pose.position, glm::length(detail::asset_half_extents(*a)) * a->pose.scale};
        }
        return std::nullopt;
    }

    [[nodiscard]] static const scene::SceneVehicle* find_vehicle(const scene::SceneDesc& s, std::string_view name) {
        for (const scene::SceneVehicle& v : s.vehicles) {
            if (v.name == name) return &v;
        }
        return nullptr;
    }
    [[nodiscard]] static const scene::SceneAsset* find_asset(const scene::SceneDesc& s, std::string_view name) {
        for (const scene::SceneAsset& a : s.assets) {
            if (a.name == name) return &a;
        }
        return nullptr;
    }

  private:
    enum class Doc { scene, world };

    // N15's staging folder (D1): the workspace's own, under the system's temp
    // folder, made by the first import, removed with the workspace, so an
    // import never saved leaves nothing behind. Its name is a random 64-bit
    // token (TestHygiene's one product use). A crashed session leaves its
    // folder behind, so starting sweeps the token folders whose newest file is
    // over a day old (the lead, 2026-10-22), and an open workspace touches its
    // own folder's "alive" at most once an hour (keep_alive(), from
    // apply_panels()), so only an orphan ever looks that old (Test/Docs: a
    // session open over a day could not otherwise save). Without a temp
    // folder there is no staging, and an import is refused by name, never
    // staged in the working directory (Test/Docs' review).
    class ImportStaging {
      public:
        ImportStaging() {
            std::error_code ec;
            const std::filesystem::path temp = std::filesystem::temp_directory_path(ec);
            if (ec || temp.empty()) {
                why_not_ = "the system's temp folder could not be found" +
                           (ec ? " (" + ec.message() + ")" : std::string()) + ", so nothing can be staged";
                return;
            }
            std::random_device rd;
            const uint64_t token = (static_cast<uint64_t>(rd()) << 32) ^ rd();
            path_ = temp / "spade-editor-imports" / std::to_string(token);
            sweep(path_.parent_path());
        }
        ~ImportStaging() {
            std::error_code ec;
            if (!path_.empty()) std::filesystem::remove_all(path_, ec);
        }
        ImportStaging(const ImportStaging&) = delete;
        ImportStaging& operator=(const ImportStaging&) = delete;
        // Empty when there is no temp folder; why_not() then says so.
        [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }
        [[nodiscard]] const std::string& why_not() const noexcept { return why_not_; }

        // Touches <path>/alive when the folder exists (an import was staged)
        // and an hour has passed since the last touch by `now`.
        void keep_alive(std::chrono::steady_clock::time_point now) {
            namespace fs = std::filesystem;
            if (path_.empty() || (touched_ && now - *touched_ < std::chrono::hours(1))) return;
            std::error_code ec;
            if (!fs::is_directory(path_, ec)) return;
            const fs::path alive = path_ / "alive";
            if (!fs::exists(alive, ec)) std::ofstream(alive, std::ios::binary).put('\n');
            fs::last_write_time(alive, fs::file_time_type::clock::now(), ec);
            if (!ec) touched_ = now;
        }

      private:
        // Removes each folder under `root` named like a token (digits only)
        // whose newest file is over a day old: a live session's is younger.
        static void sweep(const std::filesystem::path& root) {
            namespace fs = std::filesystem;
            const fs::file_time_type stale = fs::file_time_type::clock::now() - std::chrono::hours(24);
            std::vector<fs::path> gone;
            std::error_code ec;
            for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
                const std::string name = path_text(it->path().filename());
                const bool token = !name.empty() && std::all_of(name.begin(), name.end(), [](char c) {
                    return c >= '0' && c <= '9';
                });
                std::error_code e;
                if (!token || !it->is_directory(e)) continue;
                fs::file_time_type newest = fs::last_write_time(it->path(), e);
                for (fs::recursive_directory_iterator r(it->path(), e), done; !e && r != done; r.increment(e)) {
                    std::error_code t;
                    const fs::file_time_type when = r->last_write_time(t);
                    if (!t) newest = std::max(newest, when);
                }
                if (!e && newest < stale) gone.push_back(it->path());
            }
            for (const fs::path& p : gone) fs::remove_all(p, ec);
        }

        std::filesystem::path path_;
        std::string why_not_;
        std::optional<std::chrono::steady_clock::time_point> touched_;
    };
    struct Serials {
        uint64_t scene = 0;
        uint64_t world = 0;
    };

    explicit EditorWorkspace(OpenedScene opened)
        : docs_(std::move(opened)), records_(records_for(docs_.scene.path)) {}

    [[nodiscard]] Serials serials() const noexcept { return Serials{docs_.scene.serial(), docs_.world.serial()}; }

    // The File menu's scenes (SL13). A built-in scene (assets/scenes/, beside
    // assets/worlds/) tells where the built-ins are, which is kept; a scene
    // elsewhere is the user's, and its folder lists as saved. Read when a
    // scene is opened or saved, not every frame.
    // The default library, from the assets folder a built-in scene told of
    // (INT-14); what would not load is named in the panel.
    // Then the project's (EDT-020): the scene's own folder's library/, once
    // something was saved there.
    void load_library() {
        panels_.armed.reset();
        panels_.library.clear();
        panels_.library_problems.clear();
        if (assets_.empty()) {
            panels_.library_problems.push_back(
                "the default library: this scene is not in an assets folder, so its library/library.yaml was "
                "not found; open a built-in scene, or load a model or object file");
        } else {
            LibraryLoad load = load_library_items(assets_ / "library" / "library.yaml");
            panels_.library = std::move(load.items);
            panels_.library_problems = std::move(load.problems);
        }
        if (const Result<std::filesystem::path> dir = project_library_dir(docs_.scene.path); dir) {
            std::error_code ec;
            if (std::filesystem::exists(*dir / "library.yaml", ec)) {
                LibraryLoad project = load_library_items(*dir / "library.yaml");
                for (LibraryItem& item : project.items) panels_.library.push_back(std::move(item));
                for (std::string& problem : project.problems) panels_.library_problems.push_back(std::move(problem));
            }
        }
        ++panels_.library_generation;
    }

    // Save to library (EDT-020): the selected vehicle or asset into the
    // project library, drawn from the meshes the view draws with, then listed
    // after what the panel has. A refusal goes to the status line.
    void save_selection_to_library() {
        if (panels_.selection.kind != Selection::Kind::object) {
            panels_.status = "Save to library: select a vehicle or an object to save";
            return;
        }
        const Result<std::filesystem::path> dir = project_library_dir(docs_.scene.path);
        if (!dir) {
            panels_.status = dir.error().context;
            return;
        }
        std::vector<std::filesystem::path> roots{docs_.scene.path.parent_path()};
        if (!assets_.empty()) roots.push_back(assets_ / "library" / "meshes");
        const Result<LibrarySaved> saved =
            save_to_library(*dir, docs_.scene.desc(), panels_.selection.name, roots);
        if (!saved) {
            panels_.status = saved.error().context;
            return;
        }
        LibraryLoad project = load_library_items(*dir / "library.yaml");
        for (LibraryItem& item : project.items) {
            if (item.name != saved->name) continue;
            panels_.library.push_back(std::move(item));
            ++panels_.library_generation;
            break;
        }
        panels_.status = "saved '" + saved->name + "' to " + saved->file.generic_string() +
                         (saved->no_thumbnail.empty() ? std::string() : "; no thumbnail: " + saved->no_thumbnail);
    }

    void list_sources() {
        namespace fs = std::filesystem;
        const fs::path dir = docs_.scene.path.parent_path();
        std::error_code ec;
        const bool builtin = !dir.empty() && dir.filename() == "scenes" &&
                             fs::is_directory(dir.parent_path() / "worlds", ec);
        if (builtin) assets_ = dir.parent_path();
        panels_.sources = scene_sources(assets_, builtin ? fs::path() : dir);
    }

    // After an action: a refusal goes to the status line; a change goes onto
    // the history (in the order the documents changed), clears the redo side,
    // and is composed and, while running, carried into the run. True when
    // something changed.
    bool settle(Serials before, const Result<void>& result) {
        if (!result) {
            panels_.status = result.error().context;
            return false;
        }
        const Serials now = serials();
        bool changed = false;
        if (now.world != before.world) {
            undo_.push_back(Doc::world);
            changed = true;
        }
        if (now.scene != before.scene) {
            undo_.push_back(Doc::scene);
            changed = true;
        }
        while (undo_.size() > kHistoryBound) undo_.pop_front();
        panels_.status.clear();
        if (changed) {
            redo_.clear();
            after_change();
        }
        return changed;
    }

    void after_change() {
        // The air's drone was placed where the scene had it at Run air: a
        // change stops it (a Run carries edits; the air can't).
        if (air_) {
            air_.reset();
            panels_.pilot = PilotPanel{};
            panels_.status = "the air stopped: the scene changed; Run air again";
        }
        if (Result<void> composed = recompose(); !composed) {
            panels_.status = composed.error().context;
            return;
        }
        if (run_) {
            if (Result<void> carried = run_->rebuild(docs_.scene.desc(), docs_.world.desc()); !carried) {
                panels_.status = carried.error().context;
            }
        }
    }

    [[nodiscard]] Result<void> recompose() {
        Result<scene::ComposedScene> c = compose_together(docs_.scene.desc(), docs_.world.desc());
        if (!c) return std::unexpected(c.error());
        composed_ = std::move(*c);
        ++composition_;
        return {};
    }

    void place_armed(glm::vec3 origin, glm::vec3 direction) {
        const LibraryItem item = panels_.library.at(*panels_.armed);
        panels_.armed.reset();
        const float length = glm::length(direction);
        const glm::vec3 dir = length > 0.0f ? direction / length : glm::vec3(0.0f, -1.0f, 0.0f);
        const std::optional<float> t = detail::march(composed_.world.sdf, origin, dir);
        if (!t) {
            panels_.status = "place '" + item.name + "': the click did not meet the ground; click on the ground "
                             "where it should go";
            return;
        }
        SdfPose at;
        at.position = origin + dir * *t;
        at.position.y += rest_height(item);
        const Serials before = serials();
        const Result<std::string> placed = place(docs_.scene, item, at);
        if (!placed) {
            panels_.status = placed.error().context;
            return;
        }
        settle(before, Result<void>{});
        panels_.selection = Selection{Selection::Kind::object, *placed};
    }

    // Run air (INT-15, D1): the selected drone's follow region, else the
    // scene's first, on its worker; the drone held and flown by the Pilot
    // section, the rest of the scene not simulated. Refused, naming why.
    void run_air() {
        if (air_) return;
        if (run_) {
            panels_.status = "Run air: the scene is running; Stop it first";
            return;
        }
        const scene::SceneDesc& s = docs_.scene.desc();
        const scene::SceneAirflowRegion* region = nullptr;
        for (const scene::SceneAirflowRegion& r : s.airflow) {
            if (r.kind != scene::AirflowKind::follow) continue;
            const bool selected = panels_.selection.kind == Selection::Kind::object && r.follows == panels_.selection.name;
            if (selected || region == nullptr) region = &r;
            if (selected) break;
        }
        if (region == nullptr) {
            panels_.status = "Run air: no drone here has an airflow region; select a drone and Add airflow region";
            return;
        }
        Result<std::unique_ptr<AirflowRun>> started = AirflowRun::start(s, *region, std::nullopt, {}, air_starter_);
        if (!started) {
            panels_.status = started.error().context;
            return;
        }
        air_ = std::move(*started);
        panels_.pilot = PilotPanel{};
        panels_.pilot.vehicle = region->follows;
        panels_.status.clear();
    }

    // N17: the selection when it is a drone Run flies, else the first one;
    // disarmed, level, throttle 0. When none is flown, why.
    void choose_pilot() {
        panels_.pilot = PilotPanel{};
        const Selection& s = panels_.selection;
        if (s.kind == Selection::Kind::object && run_->flies(s.name)) {
            panels_.pilot.vehicle = s.name;
        } else if (const std::optional<std::string> first = run_->first_flown(); first) {
            panels_.pilot.vehicle = *first;
        } else {
            panels_.pilot.why_not = "nothing to fly: no drone here has a propulsion chain";
            for (const scene::SceneVehicle& v : docs_.scene.desc().vehicles) {
                if (const std::string why = run_->why_not(v.name); !why.empty()) {
                    panels_.pilot.why_not += "; " + why;
                    break;
                }
            }
        }
    }

    void step_run(uint64_t steps) {
        if (Result<void> stepped = run_->step(static_cast<uint32_t>(steps), panels_.pilot.vehicle,
                                              panels_.pilot.controls);
            !stepped) {
            panels_.status = stepped.error().context;
            paused_ = true;
        }
    }

    void command(Command c) {
        switch (c) {
            case Command::run:
                if (air_) {
                    panels_.status = "Run: the air is running; Stop it first";
                    return;
                }
                if (!run_) {
                    Result<EditorRun> started = EditorRun::start(docs_.scene.desc(), docs_.world.desc(), records_);
                    if (!started) {
                        panels_.status = started.error().context;
                        return;
                    }
                    run_.emplace(std::move(*started));
                    carry_ns_ = 0;
                    choose_pilot();
                }
                paused_ = false;
                return;
            case Command::run_air: run_air(); return;
            case Command::stop:
                air_.reset();  // joins the worker: at most one step
                run_.reset();
                panels_.pilot = PilotPanel{};
                paused_ = false;
                carry_ns_ = 0;
                return;
            case Command::pause:
                if (!air_) paused_ = true;  // a Run's; the air runs on
                return;
            case Command::resume: paused_ = false; return;
            case Command::step:
                if (run_) step_run(kStepTicks);
                return;
            case Command::undo: history(undo_, redo_, true); return;
            case Command::redo: history(redo_, undo_, false); return;
            case Command::save: {
                const Serials before = serials();
                if (const Result<void> staged = stage_placed_meshes(); !staged) {
                    panels_.status = staged.error().context;
                    return;
                }
                const std::vector<render::FallbackMesh> meshes = meshes_to_copy();
                save_into(before, save_scene(docs_.scene, docs_.world, meshes));
                return;
            }
            case Command::save_as: {
                const Serials before = serials();
                if (const Result<void> staged = stage_placed_meshes(); !staged) {
                    panels_.status = staged.error().context;
                    return;
                }
                const std::vector<render::FallbackMesh> meshes = meshes_to_copy();
                save_into(before, save_scene_as(docs_.scene, docs_.world, panels_.path, meshes));
                list_sources();
                load_library();  // a new folder: its project library
                return;
            }
            case Command::open: open(panels_.path); return;
            case Command::load_item: {
                Result<LibraryItem> item =
                    is_gltf(panels_.item_path) ? import_item(panels_.item_path) : item_from_file(panels_.item_path);
                if (!item) {
                    panels_.status = item.error().context;
                    return;
                }
                panels_.library.push_back(std::move(*item));
                ++panels_.library_generation;
                panels_.armed = panels_.library.size() - 1;
                panels_.status.clear();
                return;
            }
            case Command::save_to_library: save_selection_to_library(); return;
            case Command::import_parts: import_parts(panels_.item_path); return;
            case Command::arm: {
                flight::PilotKeys space;
                space.arm_pressed = true;
                pilot_keys(space, 0.0f);
                return;
            }
            case Command::level: {
                flight::PilotKeys r;
                r.level_pressed = true;
                pilot_keys(r, 0.0f);
                return;
            }
        }
    }

    // N15: a .gltf or .glb, by its extension, any case.
    [[nodiscard]] static bool is_gltf(const std::filesystem::path& file) {
        std::string ext = path_text(file.extension());
        for (char& c : ext) c = (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
        return ext == ".gltf" || ext == ".glb";
    }

    // N15: a glTF as an "imported" library item, armed by Load from file. Its
    // sidecar is staged (D1); its asset is visual only (a mesh obstacle's
    // collider waits for N9), drawn with the world's first material. Its
    // source stays empty, so placing it adds the asset, not an object file;
    // `file` keeps the path imported.
    [[nodiscard]] Result<LibraryItem> import_item(const std::filesystem::path& file) {
        const WorldDesc& world = docs_.world.desc();
        if (world.materials.empty()) {
            return std::unexpected(Error{Code::invalid_argument, detail::import_prefix(file) +
                                                                     "the world has no material to draw it with"});
        }
        if (staging_.path().empty()) {
            return std::unexpected(Error{Code::unavailable, detail::import_prefix(file) + staging_.why_not()});
        }
        Result<ImportedMesh> got = import_gltf_object(file, staging_.path(), docs_.scene.desc(), world,
                                                      docs_.scene.path.parent_path());
        if (!got) return std::unexpected(got.error());
        LibraryItem item;
        item.kind = LibraryItem::Kind::asset;
        item.name = got->name;
        scene::SceneAsset a;
        a.name = got->name;
        a.visual.mesh_ref = got->ref;
        a.visual.material = world.materials.front().name;
        item.asset = std::move(a);
        item.mesh = got->mesh;
        item.category = "imported";
        item.file = got->source;
        import_sources_[got->ref] = got->source;
        return item;
    }

    // N15: every mesh of `file` as a part of the selected drone's model (D2),
    // staged (D1), as one undoable edit. A refused edit unstages them.
    void import_parts(const std::filesystem::path& file) {
        const scene::SceneDesc& s = docs_.scene.desc();
        const scene::SceneVehicle* v =
            panels_.selection.kind == Selection::Kind::object ? find_vehicle(s, panels_.selection.name) : nullptr;
        if (v == nullptr) {
            panels_.status = "Import parts: select a drone to import the parts onto";
            return;
        }
        if (staging_.path().empty()) {
            panels_.status = detail::import_prefix(file) + staging_.why_not();
            return;
        }
        Result<std::vector<ImportedMesh>> got =
            import_gltf_parts(file, staging_.path(), s, docs_.world.desc(), docs_.scene.path.parent_path());
        if (!got) {
            panels_.status = got.error().context;
            return;
        }
        const AddMeshParts edit{v->name, *got};
        const Serials before = serials();
        const Result<void> applied = apply(docs_.scene, edit);
        std::error_code ec;
        for (const ImportedMesh& m : *got) {
            if (applied) {
                import_sources_[m.ref] = m.source;
            } else {
                std::filesystem::remove(m.sidecar, ec);
            }
        }
        settle(before, applied);
    }

    // L7b0: a placed drone's mesh parts name imported/<hash>.spmesh
    // (scene::add_model_with_meshes()), with their meshes in memory, as the
    // drone's file loaded them. Before Save, each such sidecar that neither
    // the staging folder nor the scene's folder holds yet is written into the
    // staging folder (scene::write_placed_meshes(), which the command line's
    // --save calls too), so meshes_to_copy() carries it. One that can't be
    // written saves nothing: the status says why.
    [[nodiscard]] Result<void> stage_placed_meshes() {
        const std::filesystem::path held[] = {docs_.scene.path.parent_path()};
        if (Result<void> r = scene::write_placed_meshes(docs_.scene.desc(), staging_.path(), held); !r) {
            const std::string why = staging_.path().empty() ? " (" + staging_.why_not() + ")" : std::string();
            return std::unexpected(
                Error{r.error().code, "save: " + r.error().context + why + "; the scene was not saved"});
        }
        return {};
    }

    // What Save copies beside the scene (copy_fallback_meshes(), all or
    // nothing, keeping a copy already there): the view's fallbacks (N11),
    // then every imported sidecar the scene names, an asset's or a part's
    // (N15, D1), from the staging folder, else the scene's folder as it was
    // (a Save As elsewhere).
    [[nodiscard]] std::vector<render::FallbackMesh> meshes_to_copy() const {
        namespace fs = std::filesystem;
        constexpr std::string_view kSidecar = ".spmesh";
        std::vector<render::FallbackMesh> out = fallback_meshes_;
        const scene::SceneDesc& s = docs_.scene.desc();
        std::vector<std::string> files;
        for (const scene::SceneAsset& a : s.assets) {
            if (a.visual.mesh_ref.starts_with("imported/")) files.push_back(a.visual.mesh_ref + std::string(kSidecar));
        }
        for (const vehicles::ModelType& m : s.models) {
            for (const vehicles::PartDesc& p : m.parts) {
                if (p.mesh_file.starts_with("imported/") && p.mesh_file.ends_with(kSidecar)) files.push_back(p.mesh_file);
            }
        }
        for (const std::string& file : files) {
            const std::string ref = file.substr(0, file.size() - kSidecar.size());
            const bool listed = std::any_of(out.begin(), out.end(), [&](const auto& m) { return m.ref == ref; });
            if (listed) continue;
            for (const fs::path& root : {staging_.path(), docs_.scene.path.parent_path()}) {
                if (root.empty()) continue;  // never the working directory
                const fs::path from = root / fs::path(std::u8string(file.begin(), file.end()));
                std::error_code ec;
                if (!fs::exists(from, ec)) continue;
                out.push_back(render::FallbackMesh{ref, from, 1});
                break;
            }
        }
        return out;
    }

    // N15: the path an imported mesh came from: this session's import, else
    // the provenance of its sidecar beside the scene or staged; empty for a
    // ref not under imported/. Read once per ref.
    [[nodiscard]] std::string import_source(const std::string& ref) {
        if (!ref.starts_with("imported/")) return {};
        if (const auto it = import_sources_.find(ref); it != import_sources_.end()) return it->second;
        std::string source;
        for (const std::filesystem::path& root : {docs_.scene.path.parent_path(), staging_.path()}) {
            if (root.empty()) continue;  // never the working directory
            const std::filesystem::path sidecar = root / std::filesystem::path(ref + ".spmesh");
            std::error_code ec;
            if (!std::filesystem::exists(sidecar, ec)) continue;
            if (const Result<LoadedMesh> m = load_mesh_file(sidecar); m) source = m->provenance.source;
            break;
        }
        import_sources_[ref] = source;
        return source;
    }

    // One step back (or forward) in the shared history.
    void history(std::deque<Doc>& from, std::deque<Doc>& to, bool back) {
        drag_step_ = false;  // an undo in a drag ends it: nothing may amend the state it went back to
        if (from.empty()) return;
        const Doc d = from.back();
        from.pop_back();
        const bool moved = d == Doc::scene ? (back ? docs_.scene.undo() : docs_.scene.redo())
                                           : (back ? docs_.world.undo() : docs_.world.redo());
        if (!moved) return;
        to.push_back(d);
        panels_.status.clear();
        after_change();
    }

    // A save can re-pin the scene to its world's new hash: an undoable edit,
    // so it joins the history like any other.
    void save_into(Serials before, const Result<void>& saved) {
        if (!saved) {
            panels_.status = saved.error().context;
            return;
        }
        const Serials now = serials();
        if (now.scene != before.scene) undo_.push_back(Doc::scene);
        if (now.world != before.world) undo_.push_back(Doc::world);
        panels_.status.clear();
    }

    void open(const std::filesystem::path& file) {
        Result<OpenedScene> opened = open_scene(file);
        if (!opened) {
            panels_.status = opened.error().context;
            return;
        }
        const Result<scene::ComposedScene> c = compose_together(opened->scene.desc(), opened->world.desc());
        if (!c) {
            panels_.status = c.error().context;
            return;
        }
        docs_ = std::move(*opened);
        ++opened_;  // its serials start over: a new cache key
        records_ = records_for(docs_.scene.path);
        composed_ = *c;
        ++composition_;
        run_.reset();
        paused_ = false;
        carry_ns_ = 0;
        undo_.clear();
        redo_.clear();
        panels_.selection = Selection{};
        panels_.pilot = PilotPanel{};
        panels_.armed.reset();
        import_sources_.clear();
        panels_.status.clear();
        list_sources();
        load_library();
    }

    OpenedScene docs_;
    uint64_t opened_ = 0;  // documents opened: the panels' cache key with the serial
    AirStarter air_starter_ = cpu_air_starter();
    std::unique_ptr<AirflowRun> air_;  // Run air's worker; reset (joined) by Stop and by a change
    BuiltinRecords records_;
    std::filesystem::path assets_;  // where the built-in scenes are, once a built-in was opened
    std::vector<render::FallbackMesh> fallback_meshes_;  // N11: what Save copies beside the scene
    ImportStaging staging_;                              // N15: where imports are staged until Save
    std::map<std::string, std::string> import_sources_;  // N15: an imported ref -> the path it came from
    scene::ComposedScene composed_;
    uint64_t composition_ = 0;
    std::optional<EditorRun> run_;
    bool paused_ = false;
    uint64_t carry_ns_ = 0;
    bool drag_step_ = false;  // a drag's step is committed; its next poses amend it
    std::deque<Doc> undo_;  // which document each step back undoes, oldest first
    std::deque<Doc> redo_;
    EditorPanels panels_;
};

}  // namespace spade::sandbox::editor
