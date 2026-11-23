// lines.hpp -- --lines (the lead, 2026-11-17; Physics' web-lines): the rotors of a web D1 clip as ACTUATOR LINES, one per blade element
// set, in place of discs. The blade and its polar come from a prop part file under the assets (library/parts/*.prop.yaml): the part's
// BLADE (r/R, c/R, beta) at its diameter and its fitted polar, through scene::line_propeller(). One propeller for every rotor, as the
// live run's LiveLines holds one.
#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "analysis/live_run.hpp"
#include "core/error.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"

namespace flow_movie {

// The live run's lines from one prop part file; `n_sub` is the line's sub-azimuths a step (above 0). Refused, naming the file: one that
// cannot be read, one that is not a propeller part, one with no blade to make lines of, and an n_sub of 0.
[[nodiscard]] inline spade::Result<spade::analysis::LiveLines> live_lines_from_prop_file(const std::filesystem::path& file,
                                                                                        uint32_t n_sub) {
    const auto bad = [&file](spade::Code code, const std::string& why) {
        return std::unexpected(spade::Error{code, "prop part " + file.string() + ": " + why});
    };
    if (n_sub == 0u) return bad(spade::Code::invalid_argument, "n_sub is 0 (the line needs at least one sub-azimuth a step)");
    std::ifstream in(file, std::ios::binary);
    if (!in) return bad(spade::Code::io_error, "cannot be read");
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    auto part = spade::scene::prop_from_yaml(text);
    if (!part) return bad(part.error().code, part.error().context);
    auto prop = spade::scene::line_propeller(*part);
    if (!prop) return bad(prop.error().code, prop.error().context);
    spade::analysis::LiveLines lines;
    lines.blade = prop->blade;
    lines.polar = prop->polar;
    lines.n_sub = n_sub;
    lines.fit_kernel_cells = prop->fit_kernel_cells;  // the polar's declared kernel (0: none): the run holds the case to it
    return lines;
}

}  // namespace flow_movie
