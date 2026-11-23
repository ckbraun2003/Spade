// The live tour's LEDGER, as a header so that a test can read it (SL15b: an executable's internals are not reachable from a test). The tour
// (live_tour.cpp) runs against it: a step listed here that never runs fails the run, and a step it runs that is not listed is an anomaly.
#pragma once

#include <array>
#include <string_view>

namespace spade::sandbox::live {

// THE LEDGER. Every sub-step the tour runs, in order. A step listed here that
// never runs fails the run; a step the tour runs that is not listed is an
// anomaly.
struct StepName {
    std::string_view main;
    std::string_view sub;
};
// Five chapters (INT-9, the storyboard plans/2026-10-13-live-tour-storyboard.md; refurbished 2026-11-13, the audit):
// the engine, the airflow, the editor, the bodies, and what is new in this batch.
inline constexpr std::array kSteps = {
    // Chapter 1, the engine (the audit, 2026-11-13): the drone box in three steps, what it proves and no more.
    StepName{"drone_box", "open"},       StepName{"drone_box", "attitude"},
    StepName{"drone_box", "backend"},
    StepName{"airflow", "live"},
    StepName{"editor", "open"},          StepName{"editor", "picker"},
    StepName{"editor", "select"},        StepName{"editor", "place"},
    StepName{"editor", "move"},          StepName{"editor", "inspect"},
    StepName{"editor", "undo"},          StepName{"editor", "run"},
    StepName{"editor", "world"},         StepName{"editor", "conflict"},
    StepName{"editor", "views"},         StepName{"editor", "save"},
    StepName{"editor", "library"},       StepName{"editor", "add_drone"},
    StepName{"editor", "x500_feet"},     StepName{"editor", "airflow"},
    StepName{"editor", "add_objects"},
    StepName{"editor", "fly"},           StepName{"editor", "wind"},
    // Chapter 4, the bodies: L68's contact tumbling, four shapes in a row.
    StepName{"bodies", "row"},
    // Chapter 5, "new in this batch" (L64): its steps are THIS batch's, replaced at the next one.
    StepName{"batch", "shapes"},
};

// The extra stills a run writes beside the posters (Tour::snapshot), declared
// so that a later run into the same folder owns them.
inline constexpr std::array<std::string_view, 22> kSnapshots = {"editor_picker",   "editor_vehicle",  "editor_place",
                                                         "editor_gizmo",    "editor_inspect",  "editor_running",
                                                         "editor_world",    "editor_conflict", "editor_objects",
                                                         "editor_part",     "editor_drone",    "editor_placed_drone",
                                                         "editor_airflow",  "editor_fly_props",
                                                         // The user guide's stills (L73a item 10; the lead,
                                                         // 2026-11-11): written clean, with no caption bar and no
                                                         // title card, at steps the tour already has.
                                                         "drone_box_hud",   "editor_undo",     "editor_views",
                                                         "editor_save",     "editor_air_running", "editor_subgrid",
                                                         "editor_wind",     "editor_x500_feet"};

}  // namespace spade::sandbox::live
