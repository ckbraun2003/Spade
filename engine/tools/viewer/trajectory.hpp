// ---------------------------------------------------------------------------
// trajectory.hpp -- the viewer's headless trajectory baseline (INT-4).
//
// SL14b compares each successor scene with its viewer scene on bit-identical
// state trajectories. This steps a scene the way the window does -- the same
// setup (setup.hpp), the command hook before each step(1) -- with no window
// and no v1, and writes what that comparison needs. v1-free, like scenes.cpp.
// Plan: docs/design/interface/plans/2026-10-02-v1-baselines.md, part B.
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <string>

#include "bridge.hpp"

namespace spade::viewer {

// 12 s at 250 ticks/s: past flight's gate crossing (about 7.86 s).
inline constexpr uint64_t kTrajectoryDefaultTicks = 3000;

// Steps `scene` on the CPU backend for `ticks` ticks and writes the trajectory
// file to `path`. The format is documented in trajectory.cpp. Every line is
// deterministic except those that start with "perf". Returns 0, or 1 after
// printing what failed to stderr.
[[nodiscard]] int write_trajectory(const Scene& scene, uint64_t ticks, const std::string& path);

}  // namespace spade::viewer
