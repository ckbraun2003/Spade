#pragma once

#include "core/error.hpp"
#include "physics/airflow/blade.hpp"
#include "scene/part_file.hpp"

// ---------------------------------------------------------------------------
// A PROPELLER PART AS THE ACTUATOR LINE'S BLADE AND POLAR (L86i). The part file's BLADE (r/R, c/R and beta stations, the hub's
// cut-out) at the prop's diameter, and its polar: the estimated polar (blade.hpp) with the fitted alpha_0 and drag scale when the
// part carries them (B1: `blade.alpha_0_deg`, `blade.cd_scale`, fitted through the line with the tip correction on), else the
// estimated polar's own defaults (-3 degrees, scale 1: an estimate, not a fit, and `fitted` says so).
//
//   blades  the part's blade count; radius D / 2
//   station r = (r/R) R, chord = (c/R) R, beta as given; the blade's first station is the hub's cut-out
//
// Refused (invalid_argument), naming the field: a prop with no BLADE (no line can be made of it); a diameter not positive and
// finite; whatever check_blade() refuses.
// ---------------------------------------------------------------------------
namespace spade::scene {

struct LinePropeller {
    physics::airflow::BladeShape blade;
    physics::airflow::PolarTable polar;
    bool fitted = false;  // the polar's alpha_0 and drag scale are the part's fitted ones
    float fit_kernel_cells = 0.0f;  // the kernel width in cells the part declares it was fitted at; 0 = none declared
};

[[nodiscard]] Result<LinePropeller> line_propeller(const PropPart& part);

}  // namespace spade::scene
