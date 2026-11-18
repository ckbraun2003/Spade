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

#include <glm/glm.hpp>

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

// L63 (Cameron, 2026-11-11): the website's still-air clips (S1, S8, S2) in a deeper box, so the open bottom face is not
// the invisible plane the column ends on. The same 4 mm cells and the same 0.354 m from the drone to the top face, with
// 303.5 cells (1.214 m) below it: 240 x 392 x 168, 15.8 M cells. Where the drone sits in the box is d1_hold().
inline constexpr physics::airflow::GridShape kD1DeepGrid{240, 392, 168, 0.004f};

// L86i, option A (the lead, 2026-11-17): the website's NASA-style look needs the blades' helices (pitch about 27 mm) resolved, so a uniform
// 2 mm CROPPED near field: 0.56 x 0.50 x 0.56 m, 280 x 250 x 280 = 19.6 M cells. 300 x 280 x 300 (25.2 M cells) measured 5,341,050,984 bytes
// of grid, scratch and solids (212 B a cell) against kGridByteCap's 4,294,967,296, so this is the largest box of that shape under the cap
// (3.87 GiB). The phantom's rotors (R 0.1194 m, at +-0.1245, +-0.1257 m) have their kernel reach (R + 6 dx) 0.2555 m from the axis, inside
// the 0.28 m half width; d1_hold() puts the drone 0.073 m above the centre: 0.177 m of open air above it for the inflow, 0.323 m of wake below.
// ny - 177 is odd, so the drone keeps its y cell centre as in the 4 mm boxes.
inline constexpr physics::airflow::GridShape kD1NearGrid{280, 250, 280, 0.002f};

// L86k (the lead, 2026-11-17): the X500's R 0.173 m rotors at +-0.174 m reach 0.174 + 0.173 + 6 dx from the axis, 0.365 m at 3 mm, so the
// box is 0.80 m wide; at 3 mm that is 267 x 217 x 267 = 15.5 M cells (0.80 x 0.65 x 0.80 m), 3.3 GiB by the engine's 212 B a cell, under
// kGridByteCap. d1_hold() puts the drone 0.0624 m above the centre (ny - 177 = 40 rows, 20 x 3 mm = 0.06 m): 0.27 m of open air above and
// 0.38 m of wake below the hubs.
inline constexpr physics::airflow::GridShape kD1X500Grid{267, 217, 267, 0.003f};

// Where the drone (the world origin) sits relative to the box's centre, for a D1 box of `grid`: centred in x and z, and
// as far below the top face as kD1Grid holds it (0.354 m, 88.5 cells), so +y above the centre by half the extra rows.
// Zero for kD1Grid, exactly. kD1DeepGrid's is 107.5 cells, 0.43 m: the drone keeps its cell alignment (an x and z
// face, a y cell centre) because ny - 177 is odd, so half a row. A grid no taller than kD1Grid (a small gate box, 80 rows) keeps
// the centre: zero, never negative (a negative hold put the rotors' kernels out of a 112 x 80 x 112 grid; Rendering, 2026-11-11).
[[nodiscard]] inline glm::vec3 d1_hold(const physics::airflow::GridShape& grid) noexcept {
    if (grid.ny <= kD1Grid.ny) return glm::vec3(0.0f);
    return glm::vec3(0.0f, 0.5f * (static_cast<float>(grid.ny) - static_cast<float>(kD1Grid.ny)) * grid.dx, 0.0f);
}
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
    // Where the wind comes from, about +Y from the nose (Flight::heading_deg): 0 a headwind (air moving -x, the nose
    // on +X), + from the left (-Z, the air moving +z), - from the right. D1's record is heading 0.
    float heading_deg = 0.0f;
    // The library's model file under SPADE_ASSETS_DIR: the reference quad unless another library drone is asked for (the
    // lead, 2026-11-17, the website's clips). Every D1 test and film keeps the default.
    const char* model_file = kLibraryQuad;
};

struct PoweredQuad {
    analysis::AnalysisCase c;
    vehicles::ModelType run_model;   // what LiveRun steps
    vehicles::ModelType draw_model;  // the model with its parts, to draw
    analysis::ControlProgram program;
    std::string program_name;  // "held at hover", or "the showcase around hover"
    analysis::ShowcaseTrim trim;
    // The grid, when it is not the case's cube (D1): LiveRun holds the drone
    // at its centre, or at `hold` from it, and the case's box names its longest side.
    std::optional<physics::airflow::GridShape> grid;
    // Where LiveRun holds the drone in that grid, from the box's centre (d1_hold(); L63): zero in kD1Grid. Pass it to
    // resolve_with_grid(), which sets ResolvedCase::hold_from_centre.
    glm::vec3 hold{0.0f};
};

[[nodiscard]] inline Result<PoweredQuad> powered_library_quad(const PoweredQuadSetup& s) {
    const std::filesystem::path file = std::filesystem::path(SPADE_ASSETS_DIR) / s.model_file;
    auto loaded = scene::load_model_file(file);
    if (!loaded) return std::unexpected(Error{loaded.error().code, "the drone " + std::string(s.model_file) + ": " + loaded.error().context});
    if (!loaded->model.propulsion) {
        return std::unexpected(Error{Code::invalid_argument, "the drone: " + path_text(file) +
                                                                 " has no propulsion chain, which a powered case "
                                                                 "flies on"});
    }
    PoweredQuad q;
    q.draw_model = std::move(loaded->model);
    q.run_model = q.draw_model;
    if (!s.parts_in_flow) q.run_model.parts.clear();
    q.c.power = analysis::Power{.idle = 0.05f, .state_of_charge = 1.0f};
    q.c.model = analysis::pin_model(std::string("assets/") + s.model_file, q.run_model);
    q.c.rotors.assign(q.run_model.rotors.size(), analysis::RotorSetup{});
    q.c.grid.box = s.box;
    q.c.grid.spacing = s.spacing;
    q.c.time.duration = static_cast<float>(s.seconds);
    q.c.time.step = s.step;
    q.c.flight.kind = analysis::FlightKind::hover_in_wind;
    q.c.flight.speed = 3.0f;
    q.c.flight.heading_deg = s.heading_deg;  // before the trim, which reads the case
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
                                                 bool parts_in_flow = true, float heading_deg = 0.0f,
                                                 const char* model_file = kLibraryQuad) {
    // The case's cube names the HORIZONTAL longest side (240 x 4 mm for both D1 boxes): the cube resolve() checks against the
    // solver's cap must stay a cube the cap holds, and 392^3 cells (kD1DeepGrid's height) is 6.3 GiB. resolve_with_grid() puts
    // the real grid over it.
    const float longest = grid.dx * static_cast<float>(std::max(grid.nx, grid.nz));
    auto q = powered_library_quad(PoweredQuadSetup{.seconds = seconds,
                                                   .box = longest,
                                                   .spacing = grid.dx,
                                                   .step = kD1Step,
                                                   .parts_in_flow = parts_in_flow,
                                                   .hover_only = true,
                                                   .heading_deg = heading_deg,
                                                   .model_file = model_file});
    if (!q) return q;
    q->grid = grid;
    q->hold = d1_hold(grid);
    return q;
}

// The case resolved against its run model, with `grid` in place of the
// cube's when set, and the drone held `hold` from the box's centre (d1_hold(grid)
// for a D1 box; the default is the centre). Refused as resolve() refuses, with
// every issue, and a grid past kGridByteCap.
[[nodiscard]] inline Result<analysis::ResolvedCase> resolve_with_grid(
    const analysis::AnalysisCase& c, const vehicles::ModelType& run_model,
    const std::optional<physics::airflow::GridShape>& grid, const glm::vec3& hold = glm::vec3(0.0f),
    analysis::ThinWalls thin = analysis::ThinWalls::refused) {
    auto resolved = analysis::resolve(c, run_model, thin);
    if (!resolved) {
        std::string issues;
        for (const analysis::CaseIssue& i : resolved.error()) {
            issues += (issues.empty() ? "" : "; ") + i.field + ": " + i.message;
        }
        return std::unexpected(Error{Code::invalid_argument, "the case does not resolve: " + issues});
    }
    resolved->hold_from_centre = hold;
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
