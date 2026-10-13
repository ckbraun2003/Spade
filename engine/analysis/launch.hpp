// The analysis's queued runs as their own processes (W2c; INT-7; the CFD
// spec §5): a host launches the runner tool, spade_analysis_runner (installed
// into bin/; spade::analysis_runner), on a run folder made by
// create_run_folder(). The runner is detached, so the host can close and
// reopen; it reads its folder's state from progress.yaml and stops a run by
// cancel_run(), or by the PID in run.yaml and no other way.
//
// The runner is named by an explicit, absolute path, never searched for; its
// arguments go as an array, never through a shell.
//
// Until the airflow solver lands (W1), only dry runs launch: a stand-in step
// with no airflow and zero loads, marked "dry-run" in every file it writes.
// A real run is refused by name.
//
// Display-free. Owned by Interface (INT-6, INT-7).

#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "analysis/run.hpp"
#include "core/error.hpp"

namespace spade::analysis {

// The process a launcher starts: the executable by its path, its arguments
// (without the program name) and the folder it starts in.
struct ProcessSpec {
    std::filesystem::path executable;
    std::vector<std::string> args;  // UTF-8
    std::filesystem::path working_dir;
};

// Starts `spec` as a detached process and returns its PID. The default is
// the real one; a test passes a fake.
using Launcher = std::function<Result<uint64_t>(const ProcessSpec& spec)>;

struct LaunchOptions {
    RunKind kind = RunKind::cfd;
    uint32_t pace_ms = 0;  // a dry run only: a pause after each step, so a host's progress display can be watched
};

inline constexpr uint32_t kMaxPaceMs = 10'000;

// Launches `runner` on the queued run in `folder` and writes its run.yaml.
// Refused, with nothing launched or recorded, when:
//   the runner path is relative (invalid_argument) or names no file (not_found);
//   the run is a real one: no airflow solver yet (W1) (unavailable);
//   pace_ms is over kMaxPaceMs (invalid_argument);
//   the folder has no case.yaml or progress.yaml (io_error);
//   the folder was launched before (its run.yaml names the PID) or is not
//   queued (invalid_argument);
//   the launcher refuses (its error).
// If the runner starts but run.yaml can't be written, the refusal names
// the PID, so the host can still stop it.
[[nodiscard]] Result<RunRecord> launch_run(const std::filesystem::path& runner, const std::filesystem::path& folder,
                                           const LaunchOptions& options, const Launcher& launcher = {});

// Asks the run in `folder` to stop: makes its cancel file, which the runner
// sees between steps. The run ends "cancelled", what it wrote kept as partial.
[[nodiscard]] Result<void> cancel_run(const std::filesystem::path& folder);

// Whether the process `pid` is still running (an exited child is reaped).
[[nodiscard]] bool process_running(uint64_t pid);

}  // namespace spade::analysis
