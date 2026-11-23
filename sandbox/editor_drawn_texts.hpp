// What the editor draws, as text, for the live smoke's on-screen path watch. Display-free (SL15b).
//
// The watch must read each string AS DRAWN, not every string the panels hold: `path` (what Open and Save As use) is the session's internal scene
// path, full once a scene is saved, but it reaches the screen only in the File menu's "Scene file" field while that menu is open; `selected_source`
// is drawn only inside the inspector's "imported from ..." line when an asset is selected. The panels say which of the two they drew
// (EditorPanels::Drawn); the title is the file's name, and the rest are drawn whenever they are not empty.

#pragma once

#include <vector>

#include "editor_panels.hpp"
#include "live_smoke.hpp"

namespace spade::sandbox::editor {

[[nodiscard]] inline std::vector<live::ScreenText> drawn_texts(const EditorPanels& p) {
    // RED: every string, drawn or not (what the first watch did).
    return {{"item_path", p.item_path}, {"status", p.status},       {"title", p.title},           {"path", p.path},
            {"selected_source", p.selected_source}, {"view_note", p.view_note}, {"close_prompt", p.close_prompt}};
}

}  // namespace spade::sandbox::editor
