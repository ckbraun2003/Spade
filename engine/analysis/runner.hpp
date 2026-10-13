// The analysis's runner loop (W2b; INT-7; the CFD spec §5): steps a run in
// its folder to the case's duration, writing its progress, its history and
// its log, stopping between steps when the host makes the cancel file, and
// naming a fault when a step refuses.
//
// It reads no clock (M1B: no wall-clock symbol under engine/): the caller
// passes one in. The runner tool (tools/analysis_runner/, outside engine/)
// passes the steady clock; the tests pass a fake. The step is the caller's
// too: the stand-in in the tests, the airflow solver from W1.
//
// Owned by Interface (INT-6, INT-7).

#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "analysis/run.hpp"
#include "core/error.hpp"

namespace spade::analysis {

// The file a host makes in a run's folder to stop it between steps.
inline constexpr const char* kCancelFile = "cancel";

// What one step reached: the simulated time, and one value per history
// column, in the columns' order.
struct StepSample {
    double simulated_s = 0.0;
    std::vector<double> values;
};

// Step `step` (0 first) of the run; a refusal is a fault, named in the
// progress and the log.
using StepFn = std::function<Result<StepSample>(uint64_t step)>;

// Seconds on a monotonic clock, read only to measure (elapsed, ms per step,
// when to write the progress).
using Clock = std::function<double()>;

struct RunnerSetup {
    std::vector<std::string> columns;  // history.csv's columns after time_s, each with its unit ("thrust_N")
    double history_every_s = 0.0;      // simulated s between history rows; 0 for every step
    double progress_every_s = 0.5;     // clock s between progress writes; the last is always written
    uint64_t memory_bytes = 0;         // what the run holds, for its progress
};

// Runs the case in `folder` (made by create_run_folder()) to its duration:
//   running -> done, or cancelled (the cancel file; what was written stays,
//   labelled partial), or faulted (a step's refusal, or a sample of the
//   wrong width; named in the progress and the log).
// Returns the last progress, which is also what the folder holds. Refused
// (io_error) when the folder has no case.yaml, or its files cannot be written.
[[nodiscard]] Result<RunProgress> run_case(const std::filesystem::path& folder, const RunnerSetup& setup,
                                           const StepFn& step, const Clock& clock);

}  // namespace spade::analysis
