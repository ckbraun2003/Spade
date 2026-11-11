// What a live film says of its subgrid viscosity (L59; the lead, 2026-11-11): LiveRun's own line when the cap held a
// cell or the wall limit bound a face, and else the run's peak nu_t on its own, so a Cs 0.17 run where nothing bound
// still shows how large its eddy viscosity got. "" at Cs 0 (nothing was met).
#pragma once

#include <format>
#include <string>

#include "analysis/live_run.hpp"

namespace flow_movie {

[[nodiscard]] inline std::string sgs_note(const spade::analysis::LiveSgs& sgs) {
    std::string line = spade::analysis::sgs_health_line(sgs);
    if (!line.empty()) return line;
    if (sgs.nu_t_max <= 0.0f) return "";
    return std::format("subgrid nu_t up to {:.3g} m2/s; nothing held at the cap, nothing wall-limited",
                       static_cast<double>(sgs.nu_t_max));
}

}  // namespace flow_movie
