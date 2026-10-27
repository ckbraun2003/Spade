#pragma once

// ---------------------------------------------------------------------------
// d1_case.hpp -- D1's hover preview and the powered library quad it flies:
// the ONE builder (TD-9) that tools/flow_movie, Physics' B7 and Test/Docs'
// acceptance harness include (the lead, 2026-10-27). Header-only, so tests/
// and tools/ both include it and neither links the other; both define
// SPADE_ASSETS_DIR, where the library's model file is read from.
//
// THE POWERED QUAD is the reference quad as the editor places it: the
// library's model file (N4) with its propulsion chain (N5a), idle 5 % and a
// full charge (the case's, INT-11), in a 3 m/s wind, its controls the
// showcase's moves around its HOVER (Interface's showcase_trim(), by Physics'
// rule), or that hover alone, armed from the start.
//
// D1 (docs/design/test-docs/2026-10-21-d1-hover-checks.md) is that quad with
// its rigid parts in the flow as solids (P2) at 4 mm, held at hover from the
// start, stepped every 0.38 ms (Physics: CFL 1 at the wash's ~10.5 m/s), on
// kD1Grid. Physics' box held the centre of mass 0.416 m from the inlet,
// 0.354 m above the bottom and centred in z, in 0.896 x 0.64 x 0.672 m;
// LiveRun holds it at the grid's centre, so kD1Grid is the smallest box about
// the drone that keeps each of those clearances as a minimum (0.48 m either
// side along x, 0.354 m either side along y), with the centre of mass on
// Physics' cell alignment (an x and z face, a y cell centre).
// ---------------------------------------------------------------------------

#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/error.hpp"
#include "core/path_text.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "scene/model_file.hpp"
#include "vehicles/model_type.hpp"

namespace spade::testing {

inline constexpr physics::airflow::GridShape kD1Grid{240, 177, 168, 0.004f};
inline constexpr float kD1Step = 0.00038f;  // s

// The library's reference quad, under SPADE_ASSETS_DIR.
inline constexpr const char* kLibraryQuad = "library/drones/reference_quad.model.yaml";

struct PoweredQuadSetup {
    double seconds = 0.0;
    float box = 0.8f;  // the case's cube, m
    float spacing = 0.0125f;
    float step = 0.0005f;
    bool parts_in_flow = false;  // the rigid parts as the grid's solids (P2); else drawn only
    bool hover_only = false;     // armed at the hover throttle from the start; else the showcase's moves
};

struct PoweredQuad {
    analysis::AnalysisCase c;
    vehicles::ModelType run_model;   // what LiveRun steps
    vehicles::ModelType draw_model;  // the model with its parts, to draw
    analysis::ControlProgram program;
    std::string program_name;  // "held at hover", or "the showcase around hover"
    analysis::ShowcaseTrim trim;
    // The grid, when it is not the case's cube (D1): LiveRun holds the drone
    // at its centre either way, and the case's box names its longest side.
    std::optional<physics::airflow::GridShape> grid;
};

[[nodiscard]] inline Result<PoweredQuad> powered_library_quad(const PoweredQuadSetup& s) {
    const std::filesystem::path file = std::filesystem::path(SPADE_ASSETS_DIR) / kLibraryQuad;
    auto loaded = scene::load_model_file(file);
    if (!loaded) return std::unexpected(Error{loaded.error().code, "the reference quad: " + loaded.error().context});
    if (!loaded->model.propulsion) {
        return std::unexpected(Error{Code::invalid_argument, "the reference quad: " + path_text(file) +
                                                                 " has no propulsion chain, which a powered case "
                                                                 "flies on"});
    }
    PoweredQuad q;
    q.draw_model = std::move(loaded->model);
    q.run_model = q.draw_model;
    if (!s.parts_in_flow) q.run_model.parts.clear();
    q.c.power = analysis::Power{.idle = 0.05f, .state_of_charge = 1.0f};
    q.c.model = analysis::pin_model(std::string("assets/") + kLibraryQuad, q.run_model);
    q.c.rotors.assign(q.run_model.rotors.size(), analysis::RotorSetup{});
    q.c.grid.box = s.box;
    q.c.grid.spacing = s.spacing;
    q.c.time.duration = static_cast<float>(s.seconds);
    q.c.time.step = s.step;
    q.c.flight.kind = analysis::FlightKind::hover_in_wind;
    q.c.flight.speed = 3.0f;
    auto trim = analysis::showcase_trim(q.c, q.run_model);
    if (!trim) return std::unexpected(trim.error());
    q.trim = *trim;
    if (s.hover_only) {
        q.program = analysis::ControlProgram{
            {analysis::ControlPoint{0.0f, analysis::Controls{.armed = true, .throttle = trim->hover}}}};
        q.program_name = "held at hover";
    } else {
        q.program = analysis::showcase_program(*trim);
        q.program_name = "the showcase around hover";
    }
    return q;
}

// D1: the powered quad with its parts in the flow, held at hover, on `grid`
// (kD1Grid; another grid for a measurement). Without `parts_in_flow`, the
// parts are drawn only: B7's interim GPU film, until GpuFluid takes solids.
[[nodiscard]] inline Result<PoweredQuad> d1_case(double seconds, const physics::airflow::GridShape& grid = kD1Grid,
                                                 bool parts_in_flow = true) {
    const float longest = grid.dx * static_cast<float>(std::max({grid.nx, grid.ny, grid.nz}));
    auto q = powered_library_quad(PoweredQuadSetup{.seconds = seconds,
                                                   .box = longest,
                                                   .spacing = grid.dx,
                                                   .step = kD1Step,
                                                   .parts_in_flow = parts_in_flow,
                                                   .hover_only = true});
    if (!q) return q;
    q->grid = grid;
    return q;
}

// The case resolved against its run model, with `grid` in place of the
// cube's when set. Refused as resolve() refuses, with every issue, and a grid
// past kGridByteCap.
[[nodiscard]] inline Result<analysis::ResolvedCase> resolve_with_grid(
    const analysis::AnalysisCase& c, const vehicles::ModelType& run_model,
    const std::optional<physics::airflow::GridShape>& grid) {
    auto resolved = analysis::resolve(c, run_model);
    if (!resolved) {
        std::string issues;
        for (const analysis::CaseIssue& i : resolved.error()) {
            issues += (issues.empty() ? "" : "; ") + i.field + ": " + i.message;
        }
        return std::unexpected(Error{Code::invalid_argument, "the case does not resolve: " + issues});
    }
    if (grid) {
        resolved->base = *grid;
        resolved->base_bytes = physics::airflow::fluid_bytes(resolved->base, resolved->domain);
        if (resolved->base_bytes > physics::airflow::kGridByteCap) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "the grid " + std::to_string(grid->nx) + " x " + std::to_string(grid->ny) +
                                             " x " + std::to_string(grid->nz) + " needs " +
                                             std::to_string(resolved->base_bytes >> 20) +
                                             " MiB, past kGridByteCap"});
        }
    }
    return std::move(*resolved);
}

}  // namespace spade::testing
