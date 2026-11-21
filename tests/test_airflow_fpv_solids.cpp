// ---------------------------------------------------------------------------
// test_airflow_fpv_solids.cpp -- the lead's question (2026-11-21), no GPU: which of the library's 5-inch FPV's parts does the air take at 1.25 mm
// cells, in a 270 x 237 x 270 box (337.5 x 296.25 x 337.5 mm, 17.3 M cells), and which are drawn only (the classifier's wire / thin rules, the measured
// leak safeguard, L13's contained parts)? The run is LiveRun::start() on the CPU's fluid (the CPU run steps discs; the film's lines need the device,
// and the rotors' own parts are never solids either way): its set_solids() decides, and this test only reports what it decided.
//
//   AirflowFpvSolids.WhichPartsTheAirTakesAtOnePointTwoFiveMillimetres   opt-in: SPADE_FPV_SOLIDS=1 (CPU, about 4 GB of host memory, seconds to a minute)
//
// Knobs: SPADE_FPV_DXMM (the cell in mm, default 1.25; the box in cells stays 270 x 237 x 270), SPADE_FPV_DRONE (a library drone's stem, default fpv_5inch).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include <glm/glm.hpp>

#include "analysis/live_run.hpp"
#include "d1_case.hpp"
#include "phantom_case.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace {

namespace an = spade::analysis;
namespace af = spade::physics::airflow;

[[nodiscard]] const char* fpv_env(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] const char* reason_name(an::DrawnReason r) {
    switch (r) {
        case an::DrawnReason::marked: return "marked";
        case an::DrawnReason::thin: return "thin";
        case an::DrawnReason::wire: return "wire";
        case an::DrawnReason::porous: return "porous";
        case an::DrawnReason::leaks: return "leaks";
        case an::DrawnReason::exposed: return "exposed";
        case an::DrawnReason::sibling: return "sibling";
        case an::DrawnReason::contained: return "contained";
    }
    return "?";
}

TEST(AirflowFpvSolids, WhichPartsTheAirTakesAtOnePointTwoFiveMillimetres) {
    if (fpv_env("SPADE_FPV_SOLIDS") == nullptr) GTEST_SKIP() << "opt-in: SPADE_FPV_SOLIDS=1 (CPU, about 4 GB of host memory)";
    const std::string stem = fpv_env("SPADE_FPV_DRONE") != nullptr ? fpv_env("SPADE_FPV_DRONE") : "fpv_5inch";
    const float dx = fpv_env("SPADE_FPV_DXMM") != nullptr ? static_cast<float>(0.001 * std::strtod(fpv_env("SPADE_FPV_DXMM"), nullptr)) : 0.00125f;
    const af::GridShape grid{270, 237, 270, dx};

    const std::string library = "library/drones/" + stem + ".model.yaml";
    // First pass at the hold: the drone at the box's centre, to read the rotors' height from the model; then the hubs 1.5 R under the top face (the
    // X500's rule, d1_x500_hold()), a whole number of rows so the drone keeps its cell alignment.
    auto probe = spade::testing::still_air_case(std::filesystem::path(SPADE_ASSETS_DIR) / library, "assets/" + library, grid, 0.0002f, glm::vec3(0.0f));
    ASSERT_TRUE(probe.has_value()) << probe.error().context;
    const float radius = probe->model.rotors.at(0).radius;
    const float hub_y = probe->model.rotors.at(0).local_pos.y;
    const float hold_y = std::round((0.5f * static_cast<float>(grid.ny) * dx - 1.5f * radius - hub_y) / dx) * dx;
    auto made = spade::testing::still_air_case(std::filesystem::path(SPADE_ASSETS_DIR) / library, "assets/" + library, grid, 0.0002f,
                                               glm::vec3(0.0f, hold_y, 0.0f));
    ASSERT_TRUE(made.has_value()) << made.error().context;

    std::printf("[ fpv     ] %s at %.3f mm: box %ux%ux%u = %.1f x %.1f x %.1f mm, %.1f M cells, rotor R %.4f m (R/dx %.1f), hold y %.4f m, %zu model parts\n",
                stem.c_str(), static_cast<double>(dx) * 1000.0, grid.nx, grid.ny, grid.nz, static_cast<double>(grid.nx) * dx * 1000.0,
                static_cast<double>(grid.ny) * dx * 1000.0, static_cast<double>(grid.nz) * dx * 1000.0,
                static_cast<double>(grid.nx) * grid.ny * grid.nz / 1.0e6, static_cast<double>(radius), static_cast<double>(radius / dx),
                static_cast<double>(hold_y), made->model.parts.size());
    std::fflush(stdout);
    spade::Result<an::LiveRun> run = an::LiveRun::start(made->resolved, made->model);
    if (!run.has_value()) {
        std::printf("[ fpv     ] start() REFUSED: %s\n", run.error().context.c_str());
        FAIL() << "the run was refused: " << run.error().context;
    }
    const an::LiveSolids& solids = run->solids();
    std::printf("[ fpv     ] label: %s\n", run->run_label().c_str());
    std::printf("[ fpv     ] solids: any %d, %u solid cells; set_solids(): %s\n", solids.any ? 1 : 0, solids.solid_cells, solids.run_line.c_str());
    for (const an::DrawnPart& d : solids.drawn_only) {
        std::printf("[ fpv     ] drawn only: %s (%s%s%s)\n", d.name.c_str(), reason_name(d.why), d.inside.empty() ? "" : ", inside ",
                    d.inside.c_str());
    }
    if (solids.drawn_only.empty()) std::printf("[ fpv     ] no part is drawn only\n");
    EXPECT_TRUE(solids.any) << "no part is in the air";
}

}  // namespace
