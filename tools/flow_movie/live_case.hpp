// A film flown through Interface's LiveRun (D1; flow_movie --case): the
// case, the model LiveRun steps, the model whose parts each frame draws, and
// the control program it flies.
#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include <glm/glm.hpp>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/error.hpp"
#include "../../tests/d1_case.hpp"  // D1's one builder, shared with the tests
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
    // L63: where LiveRun holds the drone in that grid, from the box's centre (ResolvedCase::hold_from_centre; the drone
    // stays at the world origin and the box's centre sits at -hold). tests/d1_case.hpp's d1_hold(): zero in kD1Grid, so
    // a centred film is the centred run to the bit; 0.43 m up in kD1DeepGrid; a box shorter than kD1Grid stays centred
    // (zero). identity names it when it is not zero.
    glm::vec3 hold{0.0f};
    // L52: the Smagorinsky subgrid viscosity the film's DEVICE fluid is made with (0: off). LiveRun's own CPU fluid
    // has no such setting, so only a device film (main.cpp's maker) applies it; identity names it.
    float sgs_cs = 0.0f;
};

// The fastest the case's air is expected to move, m/s, at least 1: the
// speed colours' top when the film sets one; else momentum theory's 2 v_h at
// the rotors' full throttle, through the model's chain, plus the wind; else
// twice the wind. A film's frames are held to ten times it (frame_check.hpp).
[[nodiscard]] float expected_speed(const LiveCase& lc);

// The case resolved against its run model, with LiveCase::grid in place of
// the cube's when set and the drone held LiveCase::hold from its centre. Refused
// as resolve() refuses, with every issue, and a grid past kGridByteCap.
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

// D1's hover preview: tests/d1_case.hpp's d1_case() (the one builder flow_movie,
// Physics' B7 and Test/Docs' harness share), as a film. `grid` replaces
// kD1Grid, for a measurement. Without `parts_in_flow`, the parts are drawn
// only, and the film's title and card say "no parts" (--gpu, B7's G2).
// `heading_deg` is where the wind comes from, about +Y from the nose (Flight::heading_deg; Physics' d1_case():
// 0 the headwind, +90 air moving +z, a crosswind). The air only: the discs ignore an in-plane inflow and
// LiveRun holds the drone, so the hover throttle is the still-air one (Physics, 2026-11-08).
using spade::testing::kD1Grid;
[[nodiscard]] spade::Result<LiveCase> d1_live_case(double seconds,
                                                   const spade::physics::airflow::GridShape& grid = kD1Grid,
                                                   bool parts_in_flow = true, float heading_deg = 0.0f);

// D1 in STILL AIR (the lead, 2026-11-10: "a hovering-quadrotor run"): the same case with the wind taken off, as
// Physics' hover-download row sets it (flight kind hover, speed 0), the parts as solids and the throttle the
// still-air hover. `grid` as d1_live_case().
[[nodiscard]] spade::Result<LiveCase> d1_web_live_case(double seconds, const spade::physics::airflow::GridShape& grid,
                                                       float heading_deg, float step_s, float sgs_cs);

[[nodiscard]] spade::Result<LiveCase> d1_still_air_live_case(double seconds,
                                                             const spade::physics::airflow::GridShape& grid = kD1Grid);

}  // namespace flow_movie
