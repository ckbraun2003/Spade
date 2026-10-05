// The live smoke's tour: spade_sandbox --live-smoke --out <dir>.
//
// A scripted walk through the sandbox's features in the real window, recorded
// to an MP4 so a person can watch what the engine does now. scripts/
// live-smoke.ps1 builds the sandbox, picks the output folder and runs it. It
// needs a display, so it is not a ctest; the verdict machinery it uses is
// display-free and tested (live_smoke.hpp, tests/test_sandbox_live_smoke.cpp).
//
// The tour is named main functions (drone_box, builder), each a run of named
// sub-steps. Every sub-step is a declared step in the ledger, drives the scene
// through the same session frame a user does (scene_sessions.hpp), and checks
// what it did, so a partial failure names the step it happened in.

#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "live_smoke.hpp"

namespace spade::sandbox::live {

struct TourOptions {
    std::filesystem::path out_dir;      // created if missing; only the run's own files are swept
    std::string label;                  // the commit and the date, for the title cards
    std::vector<Injection> injections;  // red runs only (--inject)
    uint32_t width = 1280;
    uint32_t height = 720;
    float blur = 0.0f;
    // The repository's assets/, where the built-in scene files are (--assets;
    // scripts/live-smoke.ps1 passes it). Empty: the scene main functions fail.
    std::filesystem::path assets_dir;
};

// Runs the tour and writes tour.mp4, one PNG per main function and
// journal.txt into options.out_dir. Returns 0 for PASS, 1 for FAIL, and 2
// when no verdict could be reached (no window, or the folder could not be
// written).
[[nodiscard]] int run_live_smoke(const TourOptions& options);

}  // namespace spade::sandbox::live
