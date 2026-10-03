// ---------------------------------------------------------------------------
// setup.hpp -- the viewer's simulation setup, shared by every way it runs.
//
// v1-free, like scenes.cpp. The window (bridge.cpp) and the headless capture
// step the SAME setup, so a baseline captured headless is the viewer's own
// trajectory, not a re-implementation of it (INT-4,
// docs/design/interface/plans/2026-10-02-v1-baselines.md). It is harvested
// forward with the scenes when v1 is quarantined, not quarantined with it.
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <vector>

#include "bridge.hpp"

namespace spade::viewer {

// The viewer's fixed step: Simulation::create() pins it for the run.
//
// 4 ms outer step / 4 substeps -> a 1 ms (1 kHz) effective substep, close to
// the coordinator's suggested ~1/240 s (4.1667 ms, which is not an exact
// nanosecond count -- dt_ns must divide evenly by substeps, see
// Simulation::create()'s doc comment) and landing exactly on the 1 kHz
// reference rate the engine's own docs cite repeatedly (physics/contacts.hpp,
// world/medium.hpp) -- a rounder, more defensible choice than an inexact
// 1/240 s would have been.
inline constexpr uint64_t kStepDtNs = 4'000'000;
inline constexpr uint32_t kSubsteps = 4;

// A built scene: its Simulation, and the vehicles the command hook drives.
struct SceneRun {
    spade::Simulation sim;
    std::vector<spade::VehicleRef> vehicle_refs;
};

// Creates the Simulation, spawns every body, registers every model, spawns
// every vehicle and flushes once, in that order. Throws std::runtime_error
// naming the scene on any failure, as the viewer always has.
[[nodiscard]] SceneRun make_simulation(const Scene& scene, spade::compute::BackendDesc backend);

}  // namespace spade::viewer
