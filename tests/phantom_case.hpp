#pragma once

// phantom_case.hpp -- the camera quadcopter (library/drones/phantom3_class) as a still-air D1 case on a grid of the caller's, resolved as
// the website's still-air clip resolves it (tools/flow_movie's d1_still_air_live_case): hover in no wind, the step the caller names, the
// quad's rigid parts in the air, thin walls (the blades) left to the live run, the drone held d1_hold(grid) off the box's centre.
// L86i: the lines' run on the deep box and on the 2 mm near field (tests/d1_case.hpp's kD1NearGrid).
#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/error.hpp"
#include "d1_case.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "scene/model_file.hpp"
#include "vehicles/model_type.hpp"

namespace spade::testing {

struct PhantomCase {
    vehicles::ModelType model;
    analysis::ResolvedCase resolved;
    analysis::ShowcaseTrim trim;
};

inline constexpr const char* kLibraryPhantom = "library/drones/phantom3_class.model.yaml";

// A model file as a still-air D1 case on `grid`: `pin` names it in the case (the phantom's is its library path).
[[nodiscard]] inline Result<PhantomCase> still_air_case(const std::filesystem::path& file, const std::string& pin,
                                                        const physics::airflow::GridShape& grid, float step_s = 0.0002f,
                                                        std::optional<glm::vec3> hold = std::nullopt) {
    auto loaded = scene::load_model_file(file);
    if (!loaded) return std::unexpected(loaded.error());
    if (!loaded->model.propulsion) {
        return std::unexpected(Error{Code::invalid_argument, pin + " has no propulsion chain"});
    }
    PhantomCase out;
    out.model = std::move(loaded->model);
    analysis::AnalysisCase c;
    c.power = analysis::Power{.idle = 0.05f, .state_of_charge = 1.0f};
    c.model = analysis::pin_model(pin, out.model);
    c.rotors.assign(out.model.rotors.size(), analysis::RotorSetup{});
    // The case's cube names the horizontal longest side; resolve_with_grid()'s own grid replaces it (tests/d1_case.hpp).
    c.grid.box = grid.dx * static_cast<float>(std::max(grid.nx, grid.nz));
    c.grid.spacing = grid.dx;
    c.time.duration = 1.0f;
    c.time.step = step_s;
    c.flight.kind = analysis::FlightKind::hover;  // still air
    c.flight.speed = 0.0f;
    auto trim = analysis::showcase_trim(c, out.model);
    if (!trim) return std::unexpected(trim.error());
    out.trim = *trim;
    auto resolved = analysis::resolve(c, out.model, analysis::ThinWalls::drawn_by_the_live_run);
    if (!resolved) {
        std::string issues;
        for (const analysis::CaseIssue& i : resolved.error()) issues += (issues.empty() ? "" : "; ") + i.field + ": " + i.message;
        return std::unexpected(Error{Code::invalid_argument, "the case does not resolve: " + issues});
    }
    resolved->hold_from_centre = hold ? *hold : d1_hold(grid);
    resolved->base = grid;
    resolved->base_bytes = physics::airflow::fluid_bytes(resolved->base, resolved->domain);
    out.resolved = std::move(*resolved);
    return out;
}

[[nodiscard]] inline Result<PhantomCase> phantom_case(const physics::airflow::GridShape& grid, float step_s = 0.0002f) {
    return still_air_case(std::filesystem::path(SPADE_ASSETS_DIR) / kLibraryPhantom, std::string("assets/") + kLibraryPhantom, grid,
                          step_s);
}

}  // namespace spade::testing
