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

#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "builtin_records.hpp"
#include "core/error.hpp"
#include "drone_view.hpp"
#include "editor_document.hpp"
#include "editor_files.hpp"
#include "editor_library.hpp"
#include "editor_panels.hpp"
#include "editor_pick.hpp"
#include "editor_run.hpp"
#include "editor_scene_edits.hpp"
#include "editor_world_edits.hpp"
#include "scene/compose.hpp"
#include "world/sdf.hpp"

namespace spade::sandbox::editor {

// One Step on the toolbar: a 60 Hz frame's worth of built-in steps.
inline constexpr uint64_t kStepTicks = 4;

namespace detail {

// How far above the ground a library item's origin sits when it rests there.
[[nodiscard]] inline float rest_height(const LibraryItem& item) {
    if (item.kind == LibraryItem::Kind::model) return kDroneBodyHalf.y;
    if (!item.asset || item.asset->collider.nodes.size() != 1) return 0.0f;
    const SdfNode& n = item.asset->collider.nodes[0];
    switch (static_cast<SdfPrim>(n.kind)) {
        case SdfPrim::sphere: return n.params.x;
        case SdfPrim::box: return n.params.y;
        case SdfPrim::cylinder: return n.params.y;
        case SdfPrim::capsule: return n.params.x + n.params.y;
        case SdfPrim::torus: return n.params.y;
        default: return 0.0f;
    }
}

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
        w->panels_.library = builtin_library();
        w->refresh_panels();
        return w;
    }

    [[nodiscard]] EditorPanels& panels() noexcept { return panels_; }
    [[nodiscard]] const EditorPanels& panels() const noexcept { return panels_; }
    [[nodiscard]] const OpenedScene& documents() const noexcept { return docs_; }
    [[nodiscard]] const EditorRun* run() const noexcept { return run_ ? &*run_ : nullptr; }
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
        refresh_panels();
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

    // Steps the run by `dt` seconds of whole built-in steps, the remainder
    // carried, while it runs and is not paused.
    void advance(float dt) {
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
        panels_.world = &docs_.world.desc();
        panels_.records = &records_;
        const std::string file = docs_.scene.path.empty() ? std::string("untitled") : docs_.scene.path.filename().string();
        panels_.title = file + (docs_.scene.dirty() || docs_.world.dirty() ? " *" : "");
        panels_.running = run_.has_value();
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
    struct Serials {
        uint64_t scene = 0;
        uint64_t world = 0;
    };

    explicit EditorWorkspace(OpenedScene opened)
        : docs_(std::move(opened)), records_(records_for(docs_.scene.path)) {}

    [[nodiscard]] Serials serials() const noexcept { return Serials{docs_.scene.serial(), docs_.world.serial()}; }

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
        at.position.y += detail::rest_height(item);
        const Serials before = serials();
        const Result<std::string> placed = place(docs_.scene, item, at);
        if (!placed) {
            panels_.status = placed.error().context;
            return;
        }
        settle(before, Result<void>{});
        panels_.selection = Selection{Selection::Kind::object, *placed};
    }

    void step_run(uint64_t steps) {
        if (Result<void> stepped = run_->step(static_cast<uint32_t>(steps)); !stepped) {
            panels_.status = stepped.error().context;
            paused_ = true;
        }
    }

    void command(Command c) {
        switch (c) {
            case Command::run:
                if (!run_) {
                    Result<EditorRun> started = EditorRun::start(docs_.scene.desc(), docs_.world.desc(), records_);
                    if (!started) {
                        panels_.status = started.error().context;
                        return;
                    }
                    run_.emplace(std::move(*started));
                    carry_ns_ = 0;
                }
                paused_ = false;
                return;
            case Command::stop:
                run_.reset();
                paused_ = false;
                carry_ns_ = 0;
                return;
            case Command::pause: paused_ = true; return;
            case Command::resume: paused_ = false; return;
            case Command::step:
                if (run_) step_run(kStepTicks);
                return;
            case Command::undo: history(undo_, redo_, true); return;
            case Command::redo: history(redo_, undo_, false); return;
            case Command::save: {
                const Serials before = serials();
                save_into(before, save_scene(docs_.scene, docs_.world));
                return;
            }
            case Command::save_as: {
                const Serials before = serials();
                save_into(before, save_scene_as(docs_.scene, docs_.world, panels_.path));
                return;
            }
            case Command::open: open(panels_.path); return;
        }
    }

    // One step back (or forward) in the shared history.
    void history(std::deque<Doc>& from, std::deque<Doc>& to, bool back) {
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
        records_ = records_for(docs_.scene.path);
        composed_ = *c;
        ++composition_;
        run_.reset();
        paused_ = false;
        carry_ns_ = 0;
        undo_.clear();
        redo_.clear();
        panels_.selection = Selection{};
        panels_.armed.reset();
        panels_.status.clear();
    }

    OpenedScene docs_;
    BuiltinRecords records_;
    scene::ComposedScene composed_;
    uint64_t composition_ = 0;
    std::optional<EditorRun> run_;
    bool paused_ = false;
    uint64_t carry_ns_ = 0;
    std::deque<Doc> undo_;  // which document each step back undoes, oldest first
    std::deque<Doc> redo_;
    EditorPanels panels_;
};

}  // namespace spade::sandbox::editor
