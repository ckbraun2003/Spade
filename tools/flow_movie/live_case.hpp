// A film flown through Interface's LiveRun (D1; flow_movie --case): the
// case, the model LiveRun steps, the model whose parts each frame draws, and
// the control program it flies.
#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/error.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "vehicles/model_type.hpp"

namespace flow_movie {

struct LiveCase {
    spade::analysis::AnalysisCase c;
    spade::vehicles::ModelType run_model;   // what LiveRun steps
    spade::vehicles::ModelType draw_model;  // whose parts each frame draws
    spade::analysis::ControlProgram program;
    std::string program_name;  // "the showcase program", or the controls file's name
    double seconds = 0.0;      // how long to fly, simulated
    std::string name;          // the film's title
    std::string notes;         // what the film can't show yet, for its card
    std::string identity;      // the case, the program and both models, for the cache's mark
    float speed_max = 0.0f;    // the speed colours' top, m/s; 0: twice the wind (drawing only)
    // The vortex view's lambda2 iso, as a fraction of the frame's deepest
    // core (drawing only): a third shows a held body's wake; a powered
    // rotor's deepest core is a grid-scale spike, so its tip vortices need a
    // sixth.
    float vortex_fraction = 1.0f / 3.0f;
    // The grid, when it is not the case's cube: LiveRun holds the drone at
    // its centre either way. The case's box then names its longest side, and
    // identity names the grid.
    std::optional<spade::physics::airflow::GridShape> grid;
};

// The case resolved against its run model, with LiveCase::grid in place of
// the cube's when set. Refused as resolve() refuses, with every issue, and a
// grid past kGridByteCap.
[[nodiscard]] spade::Result<spade::analysis::ResolvedCase> resolve_live(const LiveCase& lc);

// The case's model, through the one path a case has to its model today:
// Core's scene reader on the scene file the case names, picking the model
// with the case's pinned identities. Swapped for Interface's
// load_case_model() when Core's model file lands; resolve() refuses a model
// the case doesn't pin either way.
[[nodiscard]] spade::Result<spade::vehicles::ModelType> case_model(const spade::analysis::AnalysisCase& c,
                                                                   const std::filesystem::path& case_file);

// A case file and its controls: the file's program, or the showcase program
// when `controls_file` is empty. `seconds` > 0 overrides how long to fly: by
// default the case's time.duration, or showcase_seconds() for the showcase.
[[nodiscard]] spade::Result<LiveCase> live_case_from_files(const std::filesystem::path& case_file,
                                                           const std::filesystem::path& controls_file,
                                                           double seconds);

// The reference quad (tests/reference_airframe.hpp, the airframe compile's
// one fixture, until Core's model file), compiled from its airframe and flown
// POWERED by its own propulsion chain: idle 5 % and the airframe's state of
// charge, LiveRun's tests' choice until the chain pin. It runs without its
// parts (the fluid meets the solid drone with P2) and is drawn with them,
// held at the box's centre in a 3 m/s wind, flying the showcase's moves
// around its HOVER: the throttle that holds m g = n k_T w^2, found by
// bisection on the throttle map, stands in for the showcase's 0.5.
//   quad_live_case(): the film's grid, a 0.8 m box at 12.5 mm (64^3, R/dx 6),
//     stepped every 0.5 ms;
//   smoke_live_case(): the smoke's, an 8^3 box of 0.8 m stepped every 4 ms.
[[nodiscard]] spade::Result<LiveCase> quad_live_case(double seconds);
[[nodiscard]] spade::Result<LiveCase> smoke_live_case(double seconds);

// D1's hover preview (docs/design/test-docs/2026-10-21-d1-hover-checks.md):
// the same quad with its rigid parts in the flow as solids (P2) at 4 mm,
// armed at its hover throttle from the start and held level in the 3 m/s
// wind, stepped every 0.38 ms (Physics: CFL 1 at the wash's ~10.5 m/s).
// Physics' box held the centre of mass 0.416 m from the inlet, 0.354 m above
// the bottom and centred in z, in 0.896 x 0.64 x 0.672 m; LiveRun holds it at
// the grid's centre, so kD1Grid is the smallest box about the drone that
// keeps each of those clearances as a minimum (0.48 m either side along x,
// 0.354 m either side along y), with the centre of mass on Physics' cell
// alignment (an x and z face, a y cell centre). `grid` replaces it, for a
// measurement.
inline constexpr spade::physics::airflow::GridShape kD1Grid{240, 177, 168, 0.004f};
[[nodiscard]] spade::Result<LiveCase> d1_live_case(double seconds,
                                                   const spade::physics::airflow::GridShape& grid = kD1Grid);

}  // namespace flow_movie
