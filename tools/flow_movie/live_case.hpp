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
#include "vehicles/model_type.hpp"

namespace flow_movie {

struct LiveCase {
    spade::analysis::AnalysisCase c;
    spade::vehicles::ModelType run_model;   // what LiveRun steps
    spade::vehicles::ModelType draw_model;  // whose parts each frame draws
    // The rotors' chain, idle and charge, beside the model until the chain
    // pin (LiveRun::start()); none for a run without rotors.
    std::optional<spade::analysis::LivePropulsion> propulsion;
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
};

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

}  // namespace flow_movie
