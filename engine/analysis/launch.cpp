// The analysis's queued runs as their own processes (see the header).

#include "analysis/launch.hpp"

#include <fstream>
#include <string>
#include <system_error>
#include <utility>

#include "analysis/detail/process.hpp"
#include "analysis/runner.hpp"
#include "core/path_text.hpp"

namespace spade::analysis {

namespace {

namespace fs = std::filesystem;

[[nodiscard]] Error refuse(Code code, std::string why) { return Error{code, std::move(why)}; }

}  // namespace

Result<RunRecord> launch_run(const fs::path& runner, const fs::path& folder, const LaunchOptions& options,
                             const Launcher& launcher) {
    // The runner by an explicit path, never a search (INT-7).
    if (!runner.is_absolute()) {
        return std::unexpected(refuse(Code::invalid_argument, "the analysis runner's path must be absolute, not '" +
                                                                  path_text(runner) + "': it is never searched for"));
    }
    std::error_code ec;
    if (!fs::is_regular_file(runner, ec)) {
        return std::unexpected(
            refuse(Code::not_found, "the analysis runner '" + path_text(runner) + "' does not exist (L6)"));
    }
    if (options.kind != RunKind::dry_run) {
        return std::unexpected(refuse(Code::unavailable,
                                      "no airflow solver yet (W1): a real run is refused; a dry run "
                                      "(RunKind::dry_run, a stand-in step) exercises the runner"));
    }
    if (options.pace_ms > kMaxPaceMs) {
        return std::unexpected(refuse(Code::invalid_argument, "a dry run's pace is at most " +
                                                                  std::to_string(kMaxPaceMs) + " ms per step, not " +
                                                                  std::to_string(options.pace_ms)));
    }
    const fs::path dir = fs::absolute(folder, ec);
    if (ec) return std::unexpected(refuse(Code::io_error, "cannot resolve '" + path_text(folder) + "': " + ec.message()));
    if (!fs::is_regular_file(dir / "case.yaml", ec)) {
        return std::unexpected(refuse(Code::io_error, "'" + path_text(dir) + "' is not a run folder: it has no case.yaml"));
    }
    if (const Result<RunRecord> earlier = read_run_record(dir); earlier) {
        return std::unexpected(refuse(Code::invalid_argument, "the run '" + path_text(dir) + "' was launched already (pid " +
                                                                  std::to_string(earlier->pid) + "): a folder runs once"));
    }
    Result<RunProgress> progress = read_progress(dir);
    if (!progress) return std::unexpected(progress.error());
    if (progress->state != RunState::queued) {
        return std::unexpected(refuse(Code::invalid_argument, "the run '" + path_text(dir) + "' is not queued"));
    }
    // A folder runs as what it was made as: its name and its need say so.
    if (progress->kind != options.kind) {
        return std::unexpected(refuse(Code::invalid_argument, "the run '" + path_text(dir) + "' was made as a " +
                                                                  std::string(run_kind_name(progress->kind)) +
                                                                  " run, not a " +
                                                                  std::string(run_kind_name(options.kind)) + " run"));
    }
    // The progress says what the run is before its process exists.
    progress->kind = options.kind;
    if (Result<void> r = write_progress(dir, *progress); !r) return std::unexpected(r.error());

    ProcessSpec spec{runner, {path_text(dir), "--dry-run"}, dir};
    if (options.pace_ms > 0) {
        spec.args.push_back("--pace-ms");
        spec.args.push_back(std::to_string(options.pace_ms));
    }
    const Result<uint64_t> pid = launcher ? launcher(spec) : detail::launch_detached(spec);
    if (!pid) return std::unexpected(pid.error());

    const RunRecord record{options.kind, *pid, path_text(runner)};
    if (Result<void> r = write_run_record(dir, record); !r) {
        return std::unexpected(refuse(r.error().code, "the runner started as pid " + std::to_string(*pid) +
                                                          ", but its run.yaml could not be written: " +
                                                          r.error().context));
    }
    return record;
}

Result<void> cancel_run(const fs::path& folder) {
    std::error_code ec;
    if (!fs::is_regular_file(folder / "progress.yaml", ec)) {
        return std::unexpected(
            refuse(Code::io_error, "'" + path_text(folder) + "' is not a run folder: it has no progress.yaml"));
    }
    const Result<RunProgress> progress = read_progress(folder);
    if (!progress) return std::unexpected(progress.error());
    const RunState s = progress->state;
    if (s == RunState::done || s == RunState::refused || s == RunState::faulted || s == RunState::cancelled) {
        return std::unexpected(refuse(Code::invalid_argument, "the run '" + path_text(folder) +
                                                                  "' has finished: there is nothing to cancel"));
    }
    std::ofstream flag(folder / kCancelFile, std::ios::binary | std::ios::trunc);
    flag << "cancel\n";
    flag.close();
    if (!flag) return std::unexpected(refuse(Code::io_error, "cannot write '" + path_text(folder / kCancelFile) + "'"));
    return {};
}

bool process_running(uint64_t pid) { return detail::process_running(pid); }

}  // namespace spade::analysis
