// The analysis's run queue (see the header).

#include "analysis/queue.hpp"

#include <algorithm>
#include <format>
#include <fstream>
#include <optional>
#include <string>
#include <system_error>
#include <utility>

#include "analysis/detail/atomic_file.hpp"
#include "analysis/detail/process.hpp"
#include "analysis/run.hpp"
#include "analysis/runner.hpp"
#include "core/path_text.hpp"

namespace spade::analysis {

namespace {

namespace fs = std::filesystem;

constexpr const char* kQueueFile = "queue.txt";

[[nodiscard]] bool finished(RunState s) {
    return s == RunState::done || s == RunState::refused || s == RunState::faulted || s == RunState::cancelled;
}

[[nodiscard]] std::string gib(uint64_t bytes) {
    return std::format("{:.1f} GiB", static_cast<double>(bytes) / static_cast<double>(1ull << 30));
}

// A queue entry as a path, and back: UTF-8, '/'-separated, relative to runs_dir.
[[nodiscard]] fs::path entry_path(const std::string& entry) {
    return fs::path(std::u8string(entry.begin(), entry.end()));
}

[[nodiscard]] Result<std::string> entry_of(const fs::path& runs_dir, const fs::path& folder) {
    std::error_code ec;
    const fs::path root = fs::weakly_canonical(runs_dir, ec);
    const fs::path run = fs::weakly_canonical(folder, ec);
    const fs::path rel = run.lexically_relative(root);
    if (ec || rel.empty() || *rel.begin() == ".." || rel == ".") {
        return std::unexpected(Error{Code::invalid_argument, "the run '" + path_text(folder) +
                                                                 "' is not under the runs folder '" +
                                                                 path_text(runs_dir) + "'"});
    }
    const std::u8string text = rel.generic_u8string();
    return std::string(text.begin(), text.end());
}

[[nodiscard]] Result<std::vector<std::string>> read_queue(const fs::path& runs_dir) {
    std::error_code ec;
    if (!fs::exists(runs_dir / kQueueFile, ec)) return std::vector<std::string>{};
    const Result<std::string> text = detail::read_file_shared(runs_dir / kQueueFile);
    if (!text) return std::unexpected(text.error());
    std::vector<std::string> entries;
    std::size_t start = 0;
    while (start < text->size()) {
        std::size_t end = text->find('\n', start);
        if (end == std::string::npos) end = text->size();
        std::string line = text->substr(start, end - start);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) entries.push_back(std::move(line));
        start = end + 1;
    }
    return entries;
}

[[nodiscard]] Result<void> write_queue(const fs::path& runs_dir, const std::vector<std::string>& entries) {
    std::string text;
    for (const std::string& e : entries) text += e + "\n";
    return detail::replace_file(runs_dir / kQueueFile, text);
}

void log_line(const fs::path& folder, const std::string& line) {
    std::ofstream out(folder / "log.txt", std::ios::binary | std::ios::app);
    out << line << '\n';
}

// Writes `p` back only when its message changes.
[[nodiscard]] Result<void> say(const fs::path& folder, RunProgress p, const std::string& message) {
    if (p.message == message) return {};
    p.message = message;
    return write_progress(folder, p);
}

}  // namespace

Result<void> enqueue_run(const QueueSetup& setup, const fs::path& folder) {
    const Result<std::string> entry = entry_of(setup.runs_dir, folder);
    if (!entry) return std::unexpected(entry.error());
    const Result<RunProgress> p = read_progress(folder);
    if (!p) return std::unexpected(p.error());
    std::error_code ec;
    if (p->state != RunState::queued || fs::exists(folder / "run.yaml", ec)) {
        return std::unexpected(Error{Code::invalid_argument, "the run '" + *entry + "' is not queued"});
    }
    Result<std::vector<std::string>> entries = read_queue(setup.runs_dir);
    if (!entries) return std::unexpected(entries.error());
    if (std::find(entries->begin(), entries->end(), *entry) != entries->end()) {
        return std::unexpected(Error{Code::invalid_argument, "the run '" + *entry + "' is in the queue already"});
    }
    entries->push_back(*entry);
    return write_queue(setup.runs_dir, *entries);
}

Result<QueueStep> step_queue(const QueueSetup& setup, const QueueProbes& probes) {
    const std::function<bool(uint64_t)> running =
        probes.running ? probes.running : std::function<bool(uint64_t)>([](uint64_t pid) { return process_running(pid); });
    const std::function<uint64_t()> free_bytes =
        probes.free_bytes ? probes.free_bytes : std::function<uint64_t()>([] { return detail::free_memory_bytes(); });

    const Result<std::vector<std::string>> entries = read_queue(setup.runs_dir);
    if (!entries) return std::unexpected(entries.error());

    // Settle each entry: finished runs and gone folders leave the queue, a
    // dead runner is a fault, a cancelled queued run never starts.
    QueueStep out;
    std::vector<std::string> keep;
    std::string active_entry;
    std::vector<std::pair<std::string, RunProgress>> queued;
    std::error_code ec;
    for (const std::string& entry : *entries) {
        const fs::path folder = setup.runs_dir / entry_path(entry);
        if (!fs::is_directory(folder, ec)) {
            out.dropped.push_back(folder);
            continue;
        }
        const Result<RunRecord> record = read_run_record(folder);
        // Whether the runner lives, asked BEFORE its progress is read: a
        // runner writes its last progress, then exits.
        const bool alive = record && running(record->pid);
        Result<RunProgress> p = read_progress(folder);
        if (!p) return std::unexpected(p.error());
        if (finished(p->state)) continue;
        if (record) {
            if (!alive) {
                p->state = RunState::faulted;
                p->message = "the runner (pid " + std::to_string(record->pid) +
                             ") exited without finishing; its results are partial";
                if (Result<void> r = write_progress(folder, *p); !r) return std::unexpected(r.error());
                log_line(folder, "faulted: " + p->message);
                continue;
            }
            if (active_entry.empty()) {
                active_entry = entry;
                out.active = folder;
            }
            keep.push_back(entry);
            continue;
        }
        if (p->state != RunState::queued) {
            // In progress with no run.yaml: not launched by this queue.
            out.dropped.push_back(folder);
            continue;
        }
        if (fs::exists(folder / kCancelFile, ec)) {
            p->state = RunState::cancelled;
            p->message = "cancelled before it started";
            if (Result<void> r = write_progress(folder, *p); !r) return std::unexpected(r.error());
            log_line(folder, "cancelled: before it started");
            continue;
        }
        keep.push_back(entry);
        queued.emplace_back(entry, std::move(*p));
    }

    // With nothing in progress, start the first run that fits; a refused
    // launch is that run's refusal, and the next is tried.
    std::size_t first_waiting = 0;
    if (active_entry.empty()) {
        for (; first_waiting < queued.size(); ++first_waiting) {
            auto& [entry, p] = queued[first_waiting];
            const fs::path folder = setup.runs_dir / entry_path(entry);
            const uint64_t free = free_bytes();
            if (p.memory_bytes + setup.margin_bytes > free) {
                out.why = "waiting: needs " + gib(p.memory_bytes) + " plus the " + gib(setup.margin_bytes) +
                          " margin, and " + gib(free) + " is free";
                if (Result<void> r = say(folder, p, out.why); !r) return std::unexpected(r.error());
                break;
            }
            if (Result<void> r = say(folder, p, ""); !r) return std::unexpected(r.error());
            const Result<RunRecord> launched = launch_run(setup.runner, folder, setup.options, probes.launch);
            if (!launched) {
                Result<RunProgress> now = read_progress(folder);
                if (!now) return std::unexpected(now.error());
                now->state = RunState::refused;
                now->message = launched.error().context;
                if (Result<void> r = write_progress(folder, *now); !r) return std::unexpected(r.error());
                log_line(folder, "refused: " + now->message);
                keep.erase(std::find(keep.begin(), keep.end(), entry));
                continue;
            }
            out.started = folder;
            out.active = folder;
            active_entry = entry;
            ++first_waiting;
            break;
        }
    }
    // Behind a run in progress, the first waiting run says so.
    if (!active_entry.empty() && first_waiting < queued.size()) {
        out.why = "waiting: " + active_entry + " runs first";
        const auto& [entry, p] = queued[first_waiting];
        if (Result<void> r = say(setup.runs_dir / entry_path(entry), p, out.why); !r) return std::unexpected(r.error());
    }
    for (std::size_t i = first_waiting; i < queued.size(); ++i) {
        out.waiting.push_back(setup.runs_dir / entry_path(queued[i].first));
    }

    if (keep != *entries) {
        if (Result<void> r = write_queue(setup.runs_dir, keep); !r) return std::unexpected(r.error());
    }
    return out;
}

}  // namespace spade::analysis
