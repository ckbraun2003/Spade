// The analysis's runs (W2; INT-7; the CFD spec §5): a run's folder and its
// progress file, which a host reads while the runner writes it.
//
// THE FOLDER, runs/<case>/<date>_<hash16>_<commit12>/:
//   case.yaml      the resolved case, the text whose hash names the run
//   progress.yaml  replaced whole on every write (write_progress): a reader
//                  sees the old progress or the new, never a mix
//   run.yaml, history.csv, log.txt, cancel   the runner's (W2b)
//
// Display-free. Owned by Interface (INT-6, INT-7).

#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

#include "analysis/case.hpp"
#include "core/error.hpp"

namespace spade::analysis {

enum class RunState : uint32_t { queued = 0, calibrating = 1, running = 2, done = 3, refused = 4, faulted = 5, cancelled = 6 };

struct RunProgress {
    RunState state = RunState::queued;
    double simulated_s = 0.0;   // simulated time reached
    double target_s = 0.0;      // the case's duration
    uint64_t steps = 0;
    double elapsed_s = 0.0;     // wall time since the run started, measured outside stepped code
    double ms_per_step = 0.0;   // measured; 0 before the first step
    uint64_t memory_bytes = 0;  // what the run holds
    std::string message;        // a refusal, a fault, or why a run waits
};

[[nodiscard]] std::string progress_to_yaml(const RunProgress& p);
[[nodiscard]] Result<RunProgress> progress_from_yaml(std::string_view text);

// The folder's progress.yaml, written to a temporary file beside it and moved
// over it in one step (MoveFileExW / rename), retrying briefly while a reader
// holds it open; read with sharing that lets that move happen. A missing
// file is an io_error.
[[nodiscard]] Result<void> write_progress(const std::filesystem::path& folder, const RunProgress& p);
[[nodiscard]] Result<RunProgress> read_progress(const std::filesystem::path& folder);

// runs_dir/<case_name>/<date>_<hash16>_<commit12>/: `date` as YYYY-MM-DD,
// `commit` the engine's commit (its first 12 characters are used).
[[nodiscard]] std::filesystem::path run_folder(const std::filesystem::path& runs_dir, std::string_view case_name,
                                               const ResolvedCase& resolved, std::string_view date,
                                               std::string_view commit);

// Makes the run's folder with its case.yaml and a queued progress. Refused if
// the folder exists: a run is never overwritten.
[[nodiscard]] Result<std::filesystem::path> create_run_folder(const std::filesystem::path& runs_dir,
                                                              std::string_view case_name, const ResolvedCase& resolved,
                                                              std::string_view date, std::string_view commit);

}  // namespace spade::analysis
