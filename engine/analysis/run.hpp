// The analysis's runs (W2; INT-7; the CFD spec §5): a run's folder and its
// progress file, which a host reads while the runner writes it.
//
// THE FOLDER, runs/<case>/<date>_<hash16>_<commit12>/:
//   case.yaml      the resolved case, the text whose hash names the run
//   progress.yaml  replaced whole on every write (write_progress): a reader
//                  sees the old progress or the new, never a mix
//   run.yaml       who runs it: its kind, the runner and the runner's PID,
//                  written by launch_run() (W2c, analysis/launch.hpp)
//   history.csv, log.txt   the runner's (W2b)
//   cancel         made by the host (cancel_run()) to stop the run between steps
//
// Display-free. Owned by Interface (INT-6, INT-7).

#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

#include "analysis/case.hpp"
#include "core/build_identity.hpp"
#include "core/error.hpp"

namespace spade::analysis {

enum class RunState : uint32_t { queued = 0, calibrating = 1, running = 2, done = 3, refused = 4, faulted = 5, cancelled = 6 };

// What a run computes: the airflow, or a dry run's stand-in step (no airflow,
// zero loads), which exercises the launch, the progress and the cancel. A
// dry run says so in run.yaml, its progress, its log and its history, so its
// output is never mistaken for a CFD result (L6).
enum class RunKind : uint32_t { cfd = 0, dry_run = 1 };

// "cfd" or "dry-run", as the run's files spell it.
[[nodiscard]] std::string_view run_kind_name(RunKind kind) noexcept;

// A dry run holds no grid, so the queue starts it without waiting for memory
// (the lead's ruling, 2026-10-19); its queued progress and its log say so.
inline constexpr std::string_view kDryRunNoMemoryGate = "dry run: no memory gate";

struct RunProgress {
    RunKind kind = RunKind::cfd;
    RunState state = RunState::queued;
    double simulated_s = 0.0;   // simulated time reached
    double target_s = 0.0;      // the case's duration
    uint64_t steps = 0;
    double elapsed_s = 0.0;     // wall time since the run started, measured outside stepped code
    double ms_per_step = 0.0;   // measured; 0 before the first step
    uint64_t memory_bytes = 0;  // what the run holds; while queued, what it will need
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

// The folder's run.yaml: the run's kind, the runner it was launched with and
// that runner's PID, by which alone a host stops it (INT-7).
struct RunRecord {
    RunKind kind = RunKind::cfd;
    uint64_t pid = 0;
    std::string runner;  // the runner's executable, as launched (UTF-8)
};

[[nodiscard]] std::string run_record_to_yaml(const RunRecord& r);
[[nodiscard]] Result<RunRecord> run_record_from_yaml(std::string_view text);
// Written whole, as the progress is; a missing file is an io_error.
[[nodiscard]] Result<void> write_run_record(const std::filesystem::path& folder, const RunRecord& r);
[[nodiscard]] Result<RunRecord> read_run_record(const std::filesystem::path& folder);

// runs_dir/<case_name>/<date>_<hash16>_<commit12>[_dry]/: `case_name` UTF-8,
// `date` the host's UTC day as YYYY-MM-DD, `commit` the engine's commit (its
// first 12 characters are used). A dry run's name ends "_dry", so it never
// shares a folder with a real run of the same case. The name a first run
// gets; create_run_folder() says which folder a run really has.
[[nodiscard]] std::filesystem::path run_folder(const std::filesystem::path& runs_dir, std::string_view case_name,
                                               const ResolvedCase& resolved, std::string_view date,
                                               std::string_view commit, RunKind kind);

// Makes the run's folder with its case.yaml and a queued progress (its kind;
// its memory need, 0 for a dry run), and returns it: a host never builds the
// name itself. NEVER OVERWRITES: when run_folder()'s name is taken (a re-run
// the same day), the lowest free "_2", "_3"... is taken, each made with an
// exclusive create, so two calls never share a folder. ALL OR NOTHING: on any
// refusal, no folder is left behind. Refused by name (invalid_argument): a
// case name that can't be one folder (empty, "." or "..", a path separator, a
// character Windows forbids in a name, a trailing dot or space, a device name
// such as CON or NUL, or not UTF-8), a date that isn't YYYY-MM-DD, and a
// commit that isn't lowercase hex.
[[nodiscard]] Result<std::filesystem::path> create_run_folder(const std::filesystem::path& runs_dir,
                                                              std::string_view case_name, const ResolvedCase& resolved,
                                                              std::string_view date, std::string_view commit,
                                                              RunKind kind);

// The same, named by a build's identity (core/build_identity.hpp): the folder
// above at built.commit, or <date>_<hash16>_<commit12>-dirty[_dry] when
// built.dirty, git describe's mark for a build with uncommitted changes, kept
// on the commit. REFUSED (unavailable) when built.commit is "unknown": such a
// run could not be traced to the code that made it.
[[nodiscard]] Result<std::filesystem::path> create_run_folder(const std::filesystem::path& runs_dir,
                                                              std::string_view case_name, const ResolvedCase& resolved,
                                                              std::string_view date, const BuildIdentity& built,
                                                              RunKind kind);

// The same, at the build this engine is (spade::build_identity()): how a host
// names its runs without git. The form with an explicit commit is for tests,
// whose paths must not move with every commit.
[[nodiscard]] Result<std::filesystem::path> create_run_folder(const std::filesystem::path& runs_dir,
                                                              std::string_view case_name, const ResolvedCase& resolved,
                                                              std::string_view date, RunKind kind);

}  // namespace spade::analysis
