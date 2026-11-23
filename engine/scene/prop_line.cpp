#include "scene/prop_line.hpp"

#include <cmath>
#include <string>

namespace spade::scene {

Result<LinePropeller> line_propeller(const PropPart& part) {
    const auto refuse = [](const std::string& why) -> Result<LinePropeller> {
        return std::unexpected(Error{Code::invalid_argument, "line propeller: " + why});
    };
    if (!part.blade) return refuse("the propeller has no blade (BLADE): a line cannot be made of it");
    const BladeGeometry& g = *part.blade;
    const double diameter = part.block.diameter;
    if (!std::isfinite(diameter) || !(diameter > 0.0)) return refuse("the diameter is not positive and finite");
    if (g.r_over_r.size() != g.c_over_r.size() || g.r_over_r.size() != g.beta_deg.size()) {
        return refuse("blade.r_over_r, blade.c_over_r and blade.beta_deg differ in length");
    }

    const double radius = 0.5 * diameter;
    LinePropeller out;
    out.blade.blades = part.block.blades;
    out.blade.radius = static_cast<float>(radius);
    for (std::size_t i = 0; i < g.r_over_r.size(); ++i) {
        out.blade.stations.push_back({static_cast<float>(g.r_over_r[i] * radius), static_cast<float>(g.c_over_r[i] * radius),
                                      static_cast<float>(g.beta_deg[i])});
    }
    if (Result<void> ok = physics::airflow::check_blade(out.blade); !ok) {
        return refuse("blade: " + ok.error().context);
    }
    if (g.alpha_0_deg && g.cd_scale) {
        out.polar = physics::airflow::estimated_polar(
            {static_cast<float>(*g.alpha_0_deg), static_cast<float>(*g.cd_scale)});
        out.fitted = true;
        if (g.fit_kernel_cells) out.fit_kernel_cells = static_cast<float>(*g.fit_kernel_cells);
    } else {
        out.polar = physics::airflow::estimated_polar();
        out.fitted = false;
    }
    return out;
}

}  // namespace spade::scene
