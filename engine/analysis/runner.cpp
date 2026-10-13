// The analysis's runner loop (see the header).

#include "analysis/runner.hpp"

#include <algorithm>
#include <format>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>

#include "analysis/case.hpp"
#include "core/path_text.hpp"

namespace spade::analysis {

namespace {

namespace fs = std::filesystem;

// The shortest text that reads back to the same double.
[[nodiscard]] std::string number(double v) { return std::format("{}", v); }

[[nodiscard]] Error io(std::string what, const fs::path& file) {
    return Error{Code::io_error, std::move(what) + " '" + path_text(file) + "'"};
}

// One line appended to the run's log.
[[nodiscard]] Result<void> log_line(const fs::path& folder, const std::string& line) {
    std::ofstream out(folder / "log.txt", std::ios::binary | std::ios::app);
    if (!out) return std::unexpected(io("cannot write", folder / "log.txt"));
    out << line << '\n';
    if (!out) return std::unexpected(io("failed writing", folder / "log.txt"));
    return {};
}

}  // namespace

Result<RunProgress> run_case(const fs::path& folder, const RunnerSetup& setup, const StepFn& step,
                             const Clock& clock) {
    const Result<AnalysisCase> c = load_case(folder / "case.yaml");
    if (!c) return std::unexpected(c.error());

    RunProgress p;
    p.state = RunState::running;
    p.target_s = static_cast<double>(c->time.duration);
    p.memory_bytes = setup.memory_bytes;

    std::ofstream history(folder / "history.csv", std::ios::binary | std::ios::trunc);
    if (!history) return std::unexpected(io("cannot write", folder / "history.csv"));
    history << "time_s";
    for (const std::string& column : setup.columns) history << ',' << column;
    history << '\n';

    if (Result<void> r = log_line(folder, "started: " + number(p.target_s) + " s to simulate"); !r) {
        return std::unexpected(r.error());
    }
    if (Result<void> r = write_progress(folder, p); !r) return std::unexpected(r.error());

    const double start = clock();
    double last_write = start;
    // The run ends at the case's duration, read as a float: within a
    // millionth of it counts as there.
    const double end = p.target_s - 1e-6 * std::max(1.0, p.target_s);
    double next_sample = setup.history_every_s;
    std::error_code ec;
    for (uint64_t k = 0; p.simulated_s < end; ++k) {
        if (fs::exists(folder / kCancelFile, ec)) {
            p.state = RunState::cancelled;
            p.message = "cancelled by the host after " + std::to_string(p.steps) + " steps; its results are partial";
            break;
        }
        const Result<StepSample> s = step(k);
        if (!s) {
            p.state = RunState::faulted;
            p.message = "step " + std::to_string(k) + " faulted: " + s.error().context;
            break;
        }
        if (s->values.size() != setup.columns.size()) {
            p.state = RunState::faulted;
            p.message = "step " + std::to_string(k) + " gave " + std::to_string(s->values.size()) + " values for " +
                        std::to_string(setup.columns.size()) + " columns";
            break;
        }
        p.steps = k + 1;
        p.simulated_s = s->simulated_s;
        if (setup.history_every_s <= 0.0 || s->simulated_s >= next_sample - 1e-12) {
            history << number(s->simulated_s);
            for (const double v : s->values) history << ',' << number(v);
            history << '\n';
            while (setup.history_every_s > 0.0 && next_sample <= s->simulated_s + 1e-12) {
                next_sample += setup.history_every_s;
            }
        }
        const double now = clock();
        p.elapsed_s = now - start;
        p.ms_per_step = p.elapsed_s * 1000.0 / static_cast<double>(p.steps);
        if (now - last_write >= setup.progress_every_s) {
            history.flush();
            if (Result<void> r = write_progress(folder, p); !r) return std::unexpected(r.error());
            last_write = now;
        }
    }
    history.flush();
    if (!history) return std::unexpected(io("failed writing", folder / "history.csv"));
    if (p.state == RunState::running) p.state = RunState::done;

    const double now = clock();
    p.elapsed_s = now - start;
    p.ms_per_step = p.steps > 0 ? p.elapsed_s * 1000.0 / static_cast<double>(p.steps) : 0.0;
    if (Result<void> r = write_progress(folder, p); !r) return std::unexpected(r.error());
    const std::string ended = p.state == RunState::done
                                  ? "done: " + std::to_string(p.steps) + " steps, " + number(p.simulated_s) + " s"
                                  : p.message;
    if (Result<void> r = log_line(folder, ended); !r) return std::unexpected(r.error());
    return p;
}

}  // namespace spade::analysis
