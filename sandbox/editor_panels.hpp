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

#include "builtin_records.hpp"
#include "editor_library.hpp"
#include "editor_scene_edits.hpp"
#include "editor_world_edits.hpp"
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
enum class Command { run, stop, pause, resume, step, undo, redo, save, save_as, open };

struct HierarchyRow {
    Selection target;
    std::string label;
    bool vehicle = false;
};

struct EditorPanels {
    // ---- Read by the panels; the workspace fills these every frame.
    const scene::SceneDesc* scene = nullptr;
    const WorldDesc* world = nullptr;
    const BuiltinRecords* records = nullptr;  // not saved with the world yet (EDT-018)
    std::string title;                        // the scene file's name, " *" when anything is unsaved
    bool running = false;
    bool paused = false;
    uint64_t tick = 0;
    bool can_undo = false;
    bool can_redo = false;
    std::vector<HierarchyRow> hierarchy;      // the scene, the world, then its assets and vehicles in order
    std::vector<LibraryItem> library;
    std::vector<SceneSource> sources;         // the File menu's scene picker (SL13)

    // ---- Written by the panels; the workspace applies and clears them.
    Selection selection;
    std::optional<std::size_t> armed;         // a library item the next click in the view places
    std::vector<SceneEdit> pending;
    std::vector<WorldEdit> pending_world;
    std::vector<Command> commands;
    std::string path;                         // what Open and Save As use

    // ---- Both: the last refusal, verbatim (INT-2); empty when none.
    std::string status;
};

}  // namespace spade::sandbox::editor
