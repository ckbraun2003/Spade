// The editor's docked panels, drawn (editor spec §13; plan Task 10).
//
// No ImGui type appears here, for the reason gl_target_sink.hpp gives: the
// sandbox compiles where there are no GL headers.

#pragma once

namespace spade::sandbox {

namespace editor {
struct EditorPanels;
}

// Draws the toolbar, the hierarchy, the inspector and the library into the
// current ImGui frame. It reads `panels` and queues into it (selections,
// edits, commands); it decides nothing (INT-2). An edit is queued once, when
// its widget is let go, so a drag is one undoable step.
void draw_editor_panels(editor::EditorPanels& panels);

}  // namespace spade::sandbox
