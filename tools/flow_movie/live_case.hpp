// A film flown through Interface's LiveRun (D1; flow_movie --case): the
// case, the model LiveRun steps, the model whose parts each frame draws, and
// the control program it flies.
#pragma once

#include <filesystem>
#include <string>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "core/error.hpp"
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

// The smoke case, Interface's own tiny one: the hover scene's quad without
// its parts or rotors (until P2 and the rotors' coupling), an 8^3 box of
// 0.8 m stepped every 4 ms, in a 3 m/s wind, under the showcase program.
[[nodiscard]] spade::Result<LiveCase> smoke_live_case(double seconds);

}  // namespace flow_movie
