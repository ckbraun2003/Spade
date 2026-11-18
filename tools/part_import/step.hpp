#pragma once

// ---------------------------------------------------------------------------
// part_import/step.hpp -- the STEP route's pieces (L88, step 4; design note sections 4 and 5): where the converter is, how it is run, what
// it answers. The converter is python running tools/step_convert/step_convert.py (OpenCASCADE is never linked or shipped); its modes here:
//   --probe                                   {"ocp":bool,"python","occt","wheel","reason"}
//   --inspect IN                              {"unit","occt","wheel","solids":[{"name","volume","bbox_min","bbox_max"}]}  (file unit)
//   --emit-3mf OUT --deflection D --angle A [--solid NAME]...
//                                             a 3MF at OUT; {"unit","occt","wheel","cross_check":[{"name","brep_volume",
//                                             "mesh_volume","difference"}],"flagged":[names]}; exit 0, or 5 when flagged (3MF written)
// Its exit codes are step_convert.py's own: 0 done, 1 io, 2 usage, 3 OCP missing or the STEP unreadable (named), 4 a solid refused, 5 flagged.
// ---------------------------------------------------------------------------

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "options.hpp"
#include "process.hpp"
#include "report.hpp"
#include "run.hpp"

namespace spade::partimport {

struct StepTools {
    std::string python;             // a program name or a path
    std::filesystem::path converter;  // step_convert.py
};
struct ToolSearch {
    std::optional<StepTools> tools;
    std::vector<std::string> searched;  // what was looked at, in order, for a person to read
};
// --step-python / --step-convert, then SPADE_STEP_PYTHON / SPADE_STEP_CONVERT, then <exe_dir>/.venv-step's python and <exe_dir>/step_convert.py.
[[nodiscard]] ToolSearch find_step_tools(const Options& options, const RunEnv& env);

// How to install the converter, for the probe's answer.
[[nodiscard]] const char* step_install_text();

struct ConverterOutcome {
    bool started = false;
    bool killed = false;
    int exit = -1;
    std::string out, err, start_error;
};
[[nodiscard]] ConverterOutcome run_converter(const StepTools& tools, const std::vector<std::string>& args, const RunEnv& env);
// The stable problem for a converter run that did not end in 0 or 5.
[[nodiscard]] Problem converter_problem(const ConverterOutcome& outcome);

// "0.1mm" -> metres. A bare number or an unknown unit is a `usage` problem.
[[nodiscard]] Expected<double> parse_length_m(const std::string& text, const std::string& flag);
[[nodiscard]] Expected<double> parse_angle(const std::optional<std::string>& text);
[[nodiscard]] double unit_metres(const std::string& unit);  // 0 for an unknown unit

struct StepProbe {
    bool ocp = false;
    std::string python, occt, wheel, reason;
};
[[nodiscard]] Expected<StepProbe> parse_probe(const std::string& json_text);

struct StepSolid {
    std::string name;
    double volume_m3 = 0.0;
    glm::dvec3 lo{0}, hi{0};  // metres
};
struct StepInspect {
    std::string unit, occt, wheel;
    std::vector<StepSolid> solids;
    std::vector<std::string> skipped;  // parts with no solid (a surface or a shell), left out by the converter
};
[[nodiscard]] Expected<StepInspect> parse_inspect(const std::string& json_text);

struct CrossCheck {
    std::string name;
    double brep_volume = 0.0, mesh_volume = 0.0, difference = 0.0;
    bool flagged = false;
};
struct EmitReport {
    std::string unit, occt, wheel;
    std::vector<CrossCheck> checks;
    std::vector<std::string> skipped;  // parts with no solid, left out by the converter
};
[[nodiscard]] Expected<EmitReport> parse_emit(const std::string& json_text);

}  // namespace spade::partimport
