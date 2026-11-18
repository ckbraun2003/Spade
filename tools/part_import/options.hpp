#pragma once

// ---------------------------------------------------------------------------
// part_import/options.hpp -- spade_part_import's command line (design note, sections 1 and 2). One mode a run: --probe, --inspect <file> or
// --import <file>. Nothing is read to parse it: a usage problem is found before any file is touched.
// ---------------------------------------------------------------------------

#include <optional>
#include <string>
#include <vector>

#include "report.hpp"

namespace spade::partimport {

enum class Mode { none, probe, inspect, import };

struct Options {
    Mode mode = Mode::none;
    std::string file;  // inspect and import
    std::optional<std::string> store, unit, axes, scale;
    std::vector<std::string> objects;  // --object, repeatable: the picks, in the order given, a name twice kept once
    std::optional<std::string> deflection, angle, step_python, step_convert;  // STEP
    bool deep = false;                                                        // STEP inspect: tessellate and gate
    bool help = false;
};

// Code `usage` (an unknown option, a missing value, no mode or two, a mode with no file) with a sentence naming the argument.
[[nodiscard]] Expected<Options> parse_options(const std::vector<std::string>& args);

// The usage text: standard error's only content.
[[nodiscard]] const char* usage_text();

}  // namespace spade::partimport
