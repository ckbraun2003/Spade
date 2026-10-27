// What the editor's panels read and write. Display-free (SL15b).
//
// The window's ImGui code reads the top half, which the workspace fills each
// frame, and writes the bottom half: a selection, an edit, a toolbar command.
// Nothing here decides anything (INT-2): the workspace applies what the
// panels queue (editor_workspace.hpp), and a refusal comes back verbatim in
// `status`.
//
// docs/design/interface/plans/2026-10-05-editor-plan.md Task 10.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "analysis/case.hpp"  // Controls
#include "builtin_records.hpp"
#include "editor_gizmo.hpp"
#include "editor_library.hpp"
#include "editor_scene_edits.hpp"
#include "editor_views.hpp"
#include "editor_world_edits.hpp"
#include "viewport_rect.hpp"
#include "scene/scene_file.hpp"
#include "world/builder.hpp"

namespace spade::sandbox::editor {

// What the inspector shows: an asset or a vehicle by name, the scene's own
// settings, or its world's (editor spec §13: no panels of their own).
struct Selection {
    enum class Kind { none, scene, world, object };
    Kind kind = Kind::none;
    std::string name;  // the asset's or vehicle's, for Kind::object

    friend bool operator==(const Selection&, const Selection&) = default;
};

// The toolbar and the menus. Open and Save As use EditorPanels::path.
// Save to library saves the selection (EDT-020).
// arm and level act on the pilot (N17): arm toggles arming, level levels the
// tilt setpoint. run_air runs the selected drone's airflow region (INT-15);
// stop ends either run.
enum class Command { run, stop, pause, resume, step, undo, redo, save, save_as, open, load_item, save_to_library,
                     arm, level, import_parts, run_air };

struct HierarchyRow {
    Selection target;
    std::string label;
    bool vehicle = false;
};

// N17: who Run flies and what the pilot holds. The workspace picks the
// vehicle when Run starts (the selection if it is a drone with a chain, else
// the first such drone) and clears it all when Run stops; the pilot's keys and
// the Pilot section write `controls`.
struct PilotPanel {
    analysis::Controls controls{};  // armed, throttle, pitch/roll/yaw setpoints (degrees)
    std::string vehicle;            // the piloted vehicle's name; empty: none
    std::string why_not;            // why nothing is piloted, when nothing is
};

// The inspector's airflow-region line (INT-15) as last resolved, and the
// scene state it was resolved for (editor_airflow.hpp's air_line()).
struct AirLineCache {
    uint64_t opened = ~uint64_t{0};
    uint64_t serial = 0;
    std::string region;
    std::string text;
    bool refused = false;
    uint64_t resolves = 0;  // how many times it has resolved
};

struct EditorPanels {
    // ---- Read by the panels; the workspace fills these every frame.
    const scene::SceneDesc* scene = nullptr;
    // The scene document's state, a cache key: which document (counted up
    // each time one is opened, since serials start over) and its serial
    // (moved by every edit, undo and redo).
    uint64_t scene_opened = 0;
    uint64_t scene_serial = 0;
    // While Run air runs (INT-15): its speed, then its run label, which the
    // command bar shows in place of the tick, elided at the label's end
    // before Save and File would leave the bar; the session fills it.
    std::string air_band;
    AirLineCache air_line;  // kept by the panels between frames
    const WorldDesc* world = nullptr;
    const BuiltinRecords* records = nullptr;  // not saved with the world yet (EDT-018)
    std::string grid_cell;                    // its Grid cell row (grid_cell_line()): a Run's sized cell, and why
    std::string title;                        // the scene file's name, " *" when anything is unsaved
    bool running = false;
    bool air = false;  // Run air runs: Pause and Step are a Run's, off
    bool paused = false;
    uint64_t tick = 0;
    bool can_undo = false;
    bool can_redo = false;
    std::vector<HierarchyRow> hierarchy;      // the scene, the world, then its assets and vehicles in order
    std::vector<LibraryItem> library;         // the default (INT-14), the project's, then Load from file's
    std::vector<std::string> library_problems;  // what of the library would not load, verbatim
    uint64_t library_generation = 0;          // bumped whenever `library` changes
    // The window's: each library item's thumbnail as a texture (0: none),
    // parallel to `library`, uploaded again when the generation moves.
    std::vector<std::uintptr_t> library_textures;
    std::vector<SceneSource> sources;         // the File menu's scene picker (SL13)
    // The view (N19), filled by the session, which knows the backend.
    std::vector<ModeEntry> modes;             // the View menu: every base, supports()' answer for each
    std::vector<render::ViewLegend> legends;  // the view's last draw, one per layer
    std::string view_note;                    // why the view isn't drawing its chosen mode; empty when it is
    render::BaseLegend base_legend;           // the base's legend: depth's range, the normals key

    // ---- Written by the panels; the workspace applies and clears them.
    Selection selection;
    PilotPanel pilot;  // N17: written by the pilot's keys and the Pilot section
    std::string selected_source;  // N15: the path the selected asset's mesh was imported from; empty: none
    std::optional<std::size_t> armed;         // a library item the next click in the view places
    std::optional<std::size_t> inspected;     // a library part the inspector shows (INT-14); a selection ends it
    std::vector<SceneEdit> pending;
    std::vector<WorldEdit> pending_world;
    std::vector<Command> commands;
    std::string path;                         // what Open and Save As use
    std::string item_path;                    // what Load from file (Command::load_item) reads
    std::optional<bool> file_menu;            // a script's request: open (true) or close (false) the File menu
    std::optional<std::size_t> library_scroll_to;  // a script's request: bring this item's tile into view
    // Where each library item's thumbnail was drawn this frame, in framebuffer
    // pixels from the top-left; none when it was not wholly in view. The
    // tour's editor.library reads the frame there (Test/Docs' thumbnails).
    std::vector<std::optional<ViewportRect>> library_tiles;
    // Which tiles drew the "assumed" mark this frame (INT-14), parallel to
    // `library`, for the tour's assumed-marked.
    std::vector<bool> library_badges;
    GizmoTool tool = GizmoTool::move;         // the gizmo's (W, E)
    std::optional<render::ViewBase> mode_request;  // chosen from the View menu; the session applies it
    bool snap = false;                        // the gizmo snaps; Ctrl held flips it for a drag

    // ---- Both: the last refusal, verbatim (INT-2); empty when none.
    std::string status;
};

// Inspects library item `i` when it has grades (has_grades(): a part, or a
// library drone with its L5 sidecar; INT-14, the lead's Q1, 2026-10-22,
// Unity's project panel): the inspector shows it in place of the scene's
// selection, which clears. Arming is left as it is: a part arms nothing, a
// drone's tile arms it as well. An object, a drone without a sidecar or an
// index past the library is refused, and nothing changes.
inline bool inspect_item(EditorPanels& p, std::size_t i) {
    if (i >= p.library.size() || !has_grades(p.library[i])) return false;
    p.inspected = i;
    p.selection = Selection{};
    return true;
}

}  // namespace spade::sandbox::editor
