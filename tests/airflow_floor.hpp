#pragma once

// The airflow sanity floor's numbers, one definition for every test that
// checks it (B7's harness, tests/b7_harness.hpp; LiveRun's fluid on the
// device, tests/test_gpu_live_fluid.cpp). The lead's ruling, 2026-11-03, on
// Test/Docs' gap 6 (docs/design/backlog.md): a check, not a grade, so TD-14 is
// unchanged and a grade off the device of record waits for a second device to
// measure a band. A candidate's flow stays finite and under kPhysicalFactor x
// (2 v_h + the stream), and its mean speed is within kFloorBand of the CPU's,
// plus kFloorStill for air the step has all but stopped (S3's sheet leaves
// about 1e-7 m/s, where rounding alone is a large fraction). Loose by design:
// a run off the device of record parts from the CPU's in its last bits only,
// while a kernel gone wrong moves the mean by far more, or the flow past the
// bound. Fixed before any run off the device (Test/Docs, 2026-11-03).

#include <cmath>

namespace spade::testing {

inline constexpr double kPhysicalFactor = 4.0;
inline constexpr double kFloorBand = 0.10;
inline constexpr double kFloorStill = 1.0e-4;  // m/s

[[nodiscard]] inline bool mean_within_floor(double candidate, double cpu) {
    return std::fabs(candidate - cpu) <= kFloorBand * std::fabs(cpu) + kFloorStill;
}

}  // namespace spade::testing
