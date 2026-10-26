// ---------------------------------------------------------------------------
// editor_airflow.hpp -- the editor's airflow run (INT-15), display-free.
//
// The plan: docs/design/interface/plans/2026-10-22-editor-airflow-run-plan.md.
// T1: a region's case, as LiveRun will run it, and the line the inspector's
// Airflow section shows for it before it runs: the resolved grid, or the
// refusal verbatim.
// ---------------------------------------------------------------------------
#pragma once

#include <algorithm>
#include <cmath>
#include <format>
#include <string>
#include <utility>

#include "analysis/case.hpp"
#include "scene/scene_file.hpp"

namespace spade::sandbox::editor {

inline constexpr float kAirStepS = 0.001f;        // §4: airflow_live's step (P-2)
inline constexpr float kAirIdleDuty = 0.05f;      // §4: airflow_live's, until the chain pin (INT-11)
inline constexpr float kAirStateOfCharge = 1.0f;  // §4

// The case Run air runs for `region`: its drone, held at the box's centre,
// with the region's grid and fluid. Refused, naming the region and why: a
// fixed region; a drone or model not in the scene; a drone placed turned
// (LiveRun starts it level, and the air's box never turns: the lead,
// 2026-10-26); a grid that is not a cube (the case's box is one length); or
// what analysis::resolve() refuses, its first issue verbatim.
[[nodiscard]] inline Result<analysis::ResolvedCase> air_case(const scene::SceneDesc& s,
                                                             const scene::SceneAirflowRegion& region) {
    const auto refused = [&](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, "airflow region '" + region.name + "': " + why});
    };
    if (region.kind != scene::AirflowKind::follow) {
        return refused("Run air runs a follow region around a drone, and this one is fixed");
    }
    const auto v = std::find_if(s.vehicles.begin(), s.vehicles.end(),
                                [&](const scene::SceneVehicle& x) { return x.name == region.follows; });
    if (v == s.vehicles.end()) return refused("it follows '" + region.follows + "', which is not a vehicle in this scene");
    const auto m = std::find_if(s.models.begin(), s.models.end(),
                                [&](const vehicles::ModelType& x) { return x.name == v->model; });
    if (m == s.models.end()) return refused("vehicle '" + v->name + "' uses model '" + v->model + "', which is not in this scene");
    const glm::quat q = v->start.orient;
    if (std::abs(q.x) > 1e-6f || std::abs(q.y) > 1e-6f || std::abs(q.z) > 1e-6f) {
        return refused("'" + v->name +
                       "' is turned. Run air needs the drone unrotated: the air's box can't turn with it. Set its "
                       "rotation to zero to run it.");
    }
    const physics::airflow::GridShape& g = region.cells;
    if (g.nx != g.ny || g.ny != g.nz) {
        return refused(std::format("it is {}x{}x{} cells, and Run air's box is a cube, so the three must match", g.nx,
                                   g.ny, g.nz));
    }
    analysis::AnalysisCase c;
    c.model = analysis::pin_model(v->model, *m);
    c.power = analysis::Power{.idle = kAirIdleDuty, .state_of_charge = kAirStateOfCharge};
    c.fluid = analysis::Fluid{.density = region.density, .kinematic_viscosity = region.kinematic_viscosity};
    c.rotors.assign(m->rotors.size(), analysis::RotorSetup{});
    c.grid.box = static_cast<float>(g.nx) * g.dx;
    c.grid.spacing = g.dx;
    c.time.step = kAirStepS;
    auto resolved = analysis::resolve(c, *m);
    if (!resolved) {
        const analysis::CaseIssue& first = resolved.error().front();
        return refused(first.field + ": " + first.message);
    }
    return std::move(*resolved);
}

// The inspector's line for `region`, as LiveRun's label gives its grid
// ("32³, dx 25 mm, R/dx 3"), or air_case()'s refusal verbatim.
[[nodiscard]] inline std::string region_line(const scene::SceneDesc& s, const scene::SceneAirflowRegion& region) {
    const Result<analysis::ResolvedCase> r = air_case(s, region);
    if (!r) return r.error().context;
    std::string out = std::format("{}\xc2\xb3, dx {:g} mm", r->base.nx, r->base.dx * 1000.0f);  // a superscript three
    if (!r->rotors.empty()) out += std::format(", R/dx {:.0f}", r->rotors.front().cells_across_radius);
    return out;
}

}  // namespace spade::sandbox::editor
