// The analysis's run folder and progress file (see the header).

#include "analysis/run.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <format>
#include <string>
#include <system_error>

#include <yaml-cpp/yaml.h>

#include "analysis/detail/atomic_file.hpp"
#include "core/build_identity.hpp"
#include "core/path_text.hpp"
#include "world/detail/yaml_text.hpp"

namespace spade::analysis {

namespace {

namespace fs = std::filesystem;
using yaml_text::at;
using yaml_text::quote_yaml;
using yaml_text::string_field;

constexpr std::array<std::string_view, 7> kStateNames = {"queued", "calibrating", "running", "done",
                                                         "refused", "faulted", "cancelled"};
constexpr std::array<std::string_view, 2> kKindNames = {"cfd", "dry-run"};
constexpr const char* kProgressFile = "progress.yaml";
constexpr const char* kRecordFile = "run.yaml";

// Re-runs of one case on one day, at most: past this, a host is looping.
constexpr int kMaxRunsPerDay = 1000;

// Whether `s` is well-formed UTF-8: no overlong forms, no surrogates, nothing
// past U+10FFFF.
[[nodiscard]] bool utf8(std::string_view s) {
    for (std::size_t i = 0; i < s.size();) {
        const auto b = static_cast<unsigned char>(s[i]);
        std::size_t n = 0;
        uint32_t cp = 0;
        if (b < 0x80) {
            ++i;
            continue;
        }
        if ((b & 0xE0) == 0xC0) {
            n = 1;
            cp = b & 0x1Fu;
        } else if ((b & 0xF0) == 0xE0) {
            n = 2;
            cp = b & 0x0Fu;
        } else if ((b & 0xF8) == 0xF0) {
            n = 3;
            cp = b & 0x07u;
        } else {
            return false;
        }
        if (i + n >= s.size()) return false;  // cut short
        for (std::size_t k = 1; k <= n; ++k) {
            const auto c = static_cast<unsigned char>(s[i + k]);
            if ((c & 0xC0) != 0x80) return false;
            cp = (cp << 6) | (c & 0x3Fu);
        }
        constexpr uint32_t kLeast[] = {0, 0x80, 0x800, 0x10000};
        if (cp < kLeast[n] || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) return false;
        i += n + 1;
    }
    return true;
}

// Why `name` can't be one folder's name on every platform Spade runs on, or
// empty when it can.
[[nodiscard]] std::string not_a_folder_name(std::string_view name) {
    if (name.empty()) return "it is empty";
    if (name == "." || name == "..") return "it is '" + std::string(name) + "'";
    if (!utf8(name)) return "it isn't UTF-8";
    for (const char c : name) {
        if (c == '/' || c == '\\') return "it holds a path separator";
        if (static_cast<unsigned char>(c) < 0x20) return "it holds a control character";
        if (std::string_view("<>:\"|?*").find(c) != std::string_view::npos) {
            return std::string("it holds '") + c + "', which Windows forbids in a name";
        }
    }
    if (name.back() == '.' || name.back() == ' ') return "it ends with a dot or a space, which Windows drops";
    // A device name, with or without an extension (CON, NUL.txt), any case.
    std::string stem(name.substr(0, name.find('.')));
    for (char& c : stem) c = static_cast<char>(c >= 'a' && c <= 'z' ? c - 'a' + 'A' : c);
    const auto device = [&] {
        if (stem == "CON" || stem == "PRN" || stem == "AUX" || stem == "NUL") return true;
        return stem.size() == 4 && (stem.starts_with("COM") || stem.starts_with("LPT")) && stem[3] >= '0' &&
               stem[3] <= '9';
    };
    if (device()) return "'" + std::string(name) + "' is a device name on Windows";
    return {};
}

[[nodiscard]] bool is_date(std::string_view d) {
    if (d.size() != 10 || d[4] != '-' || d[7] != '-') return false;
    for (std::size_t i = 0; i < d.size(); ++i) {
        if (i != 4 && i != 7 && (d[i] < '0' || d[i] > '9')) return false;
    }
    return true;
}

[[nodiscard]] bool is_commit(std::string_view c) {
    return !c.empty() &&
           std::all_of(c.begin(), c.end(), [](char x) { return (x >= '0' && x <= '9') || (x >= 'a' && x <= 'f'); });
}

// A UTF-8 name as a path, on every platform: a narrow string is read in the
// system's code page on Windows.
[[nodiscard]] fs::path utf8_path(std::string_view name) {
    return fs::path(std::u8string(reinterpret_cast<const char8_t*>(name.data()), name.size()));
}

// Doubles as the shortest text that reads back to the same value.
[[nodiscard]] std::string number(double v) { return std::format("{}", v); }

[[nodiscard]] Result<double> double_field(const YAML::Node& map, const char* key) {
    const Result<std::string> text = string_field(map, key, "progress");
    if (!text) return std::unexpected(text.error());
    double v = 0.0;
    const std::from_chars_result r = std::from_chars(text->data(), text->data() + text->size(), v);
    if (r.ec != std::errc{} || r.ptr != text->data() + text->size()) {
        return std::unexpected(at(map[key], std::string("progress.") + key + ": '" + *text + "' is not a number"));
    }
    return v;
}

[[nodiscard]] Result<RunKind> kind_field(const YAML::Node& map, const char* what) {
    const Result<std::string> kind = string_field(map, "kind", what);
    if (!kind) return std::unexpected(kind.error());
    for (std::size_t i = 0; i < kKindNames.size(); ++i) {
        if (kKindNames[i] == *kind) return static_cast<RunKind>(i);
    }
    return std::unexpected(at(map["kind"], std::string(what) + ".kind: '" + *kind + "' is not cfd or dry-run"));
}

[[nodiscard]] Result<RunProgress> parse_progress(const YAML::Node& root) {
    if (Result<void> ok = yaml_text::check_map(root, "progress",
                                               {"kind", "state", "simulated_s", "target_s", "steps", "elapsed_s",
                                                "ms_per_step", "memory_bytes", "message"},
                                               "progress schema v1");
        !ok) {
        return std::unexpected(ok.error());
    }
    RunProgress p;
    const Result<RunKind> kind = kind_field(root, "progress");
    if (!kind) return std::unexpected(kind.error());
    p.kind = *kind;
    const Result<std::string> state = string_field(root, "state", "progress");
    if (!state) return std::unexpected(state.error());
    bool known = false;
    for (std::size_t i = 0; i < kStateNames.size(); ++i) {
        if (kStateNames[i] == *state) {
            p.state = static_cast<RunState>(i);
            known = true;
        }
    }
    if (!known) return std::unexpected(at(root["state"], "progress.state: '" + *state + "' is not a run state"));
    for (const auto& [key, out] : {std::pair<const char*, double*>{"simulated_s", &p.simulated_s},
                                   {"target_s", &p.target_s},
                                   {"elapsed_s", &p.elapsed_s},
                                   {"ms_per_step", &p.ms_per_step}}) {
        const Result<double> v = double_field(root, key);
        if (!v) return std::unexpected(v.error());
        *out = *v;
    }
    const Result<uint64_t> steps = yaml_text::uint_field<uint64_t>(root, "steps", "progress");
    if (!steps) return std::unexpected(steps.error());
    p.steps = *steps;
    const Result<uint64_t> memory = yaml_text::uint_field<uint64_t>(root, "memory_bytes", "progress");
    if (!memory) return std::unexpected(memory.error());
    p.memory_bytes = *memory;
    const Result<std::string> message = string_field(root, "message", "progress");
    if (!message) return std::unexpected(message.error());
    p.message = *message;
    return p;
}

[[nodiscard]] Result<RunRecord> parse_record(const YAML::Node& root) {
    if (Result<void> ok = yaml_text::check_map(root, "run", {"kind", "pid", "runner"}, "run schema v1"); !ok) {
        return std::unexpected(ok.error());
    }
    RunRecord r;
    const Result<RunKind> kind = kind_field(root, "run");
    if (!kind) return std::unexpected(kind.error());
    r.kind = *kind;
    const Result<uint64_t> pid = yaml_text::uint_field<uint64_t>(root, "pid", "run");
    if (!pid) return std::unexpected(pid.error());
    r.pid = *pid;
    const Result<std::string> runner = string_field(root, "runner", "run");
    if (!runner) return std::unexpected(runner.error());
    r.runner = *runner;
    return r;
}

template <class T, class Parse>
[[nodiscard]] Result<T> parse_text(std::string_view text, const char* what, Parse parse) {
    try {
        return parse(YAML::Load(std::string(text)));
    } catch (const YAML::Exception& e) {
        return std::unexpected(Error{Code::invalid_argument, std::string(e.what())});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, std::string(what) + " parse failed: " + e.what()});
    }
}

}  // namespace

std::string_view run_kind_name(RunKind kind) noexcept { return kKindNames[static_cast<std::size_t>(kind)]; }

std::string progress_to_yaml(const RunProgress& p) {
    std::string out;
    out += "kind: " + quote_yaml(run_kind_name(p.kind)) + "\n";
    out += "state: " + quote_yaml(kStateNames[static_cast<std::size_t>(p.state)]) + "\n";
    out += "simulated_s: " + number(p.simulated_s) + "\n";
    out += "target_s: " + number(p.target_s) + "\n";
    out += "steps: " + std::to_string(p.steps) + "\n";
    out += "elapsed_s: " + number(p.elapsed_s) + "\n";
    out += "ms_per_step: " + number(p.ms_per_step) + "\n";
    out += "memory_bytes: " + std::to_string(p.memory_bytes) + "\n";
    out += "message: " + quote_yaml(p.message) + "\n";
    return out;
}

Result<RunProgress> progress_from_yaml(std::string_view text) {
    return parse_text<RunProgress>(text, "progress", parse_progress);
}

std::string run_record_to_yaml(const RunRecord& r) {
    std::string out;
    out += "kind: " + quote_yaml(run_kind_name(r.kind)) + "\n";
    out += "pid: " + std::to_string(r.pid) + "\n";
    out += "runner: " + quote_yaml(r.runner) + "\n";
    return out;
}

Result<RunRecord> run_record_from_yaml(std::string_view text) { return parse_text<RunRecord>(text, "run", parse_record); }

Result<void> write_run_record(const fs::path& folder, const RunRecord& r) {
    return detail::replace_file(folder / kRecordFile, run_record_to_yaml(r));
}

Result<RunRecord> read_run_record(const fs::path& folder) {
    const Result<std::string> text = detail::read_file_shared(folder / kRecordFile);
    if (!text) return std::unexpected(text.error());
    return run_record_from_yaml(*text);
}

Result<void> write_progress(const fs::path& folder, const RunProgress& p) {
    return detail::replace_file(folder / kProgressFile, progress_to_yaml(p));
}

Result<RunProgress> read_progress(const fs::path& folder) {
    const Result<std::string> text = detail::read_file_shared(folder / kProgressFile);
    if (!text) return std::unexpected(text.error());
    return progress_from_yaml(*text);
}

namespace {

// <date>_<hash16>_<commit12>[-dirty][_dry]: git describe's mark stays on the
// commit, and the kind ends the name.
[[nodiscard]] std::string folder_name(const ResolvedCase& resolved, std::string_view date, std::string_view commit,
                                      bool dirty, RunKind kind) {
    return std::string(date) + "_" + std::format("{:016x}", resolved.hash) + "_" +
           std::string(commit.substr(0, std::min<std::size_t>(commit.size(), 12))) + (dirty ? "-dirty" : "") +
           (kind == RunKind::dry_run ? "_dry" : "");
}

}  // namespace

fs::path run_folder(const fs::path& runs_dir, std::string_view case_name, const ResolvedCase& resolved,
                    std::string_view date, std::string_view commit, RunKind kind) {
    return runs_dir / utf8_path(case_name) / utf8_path(folder_name(resolved, date, commit, false, kind));
}

namespace {

// The run's folder, its case.yaml and its queued progress: what every form of
// create_run_folder() makes. All or nothing.
[[nodiscard]] Result<fs::path> make_run_folder(const fs::path& runs_dir, std::string_view case_name,
                                               const ResolvedCase& resolved, std::string_view date,
                                               std::string_view commit, bool dirty, RunKind kind) {
    if (const std::string why = not_a_folder_name(case_name); !why.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "the case name can't name a run's folder: " + why});
    }
    if (!is_date(date)) {
        return std::unexpected(Error{Code::invalid_argument, "the date '" + std::string(date) +
                                                                 "' isn't YYYY-MM-DD (the host's UTC day)"});
    }
    if (!is_commit(commit)) {
        return std::unexpected(
            Error{Code::invalid_argument, "the commit '" + std::string(commit) + "' isn't a commit id (lowercase hex)"});
    }
    const Result<std::string> text = case_to_yaml(resolved.spec);
    if (!text) return std::unexpected(text.error());

    const fs::path first =
        runs_dir / utf8_path(case_name) / utf8_path(folder_name(resolved, date, commit, dirty, kind));
    const fs::path parent = first.parent_path();
    std::error_code ec;
    const bool had_parent = fs::exists(parent, ec);
    fs::create_directories(parent, ec);
    if (ec) return std::unexpected(Error{Code::io_error, "cannot make '" + path_text(parent) + "': " + ec.message()});
    // Undoes what this call made, so a refusal leaves nothing behind.
    const auto undo = [&](const fs::path& folder) {
        std::error_code ignored;
        if (!folder.empty()) fs::remove_all(folder, ignored);
        if (!had_parent && fs::is_empty(parent, ignored)) fs::remove(parent, ignored);
    };
    // The first free name, each made exclusively: an existing folder is
    // another run's, so the next suffix is tried.
    fs::path folder;
    for (int n = 1; n <= kMaxRunsPerDay && folder.empty(); ++n) {
        const fs::path candidate =
            n == 1 ? first : parent / utf8_path(path_text(first.filename()) + "_" + std::to_string(n));
        if (fs::create_directory(candidate, ec)) {
            folder = candidate;
        } else if (ec) {
            undo({});
            return std::unexpected(Error{Code::io_error, "cannot make '" + path_text(candidate) + "': " + ec.message()});
        }
    }
    if (folder.empty()) {
        undo({});
        return std::unexpected(Error{Code::invalid_argument, "the case '" + std::string(case_name) + "' has " +
                                                                 std::to_string(kMaxRunsPerDay) +
                                                                 " runs today at this commit already"});
    }
    if (Result<void> r = detail::replace_file(folder / "case.yaml", *text); !r) {
        undo(folder);
        return std::unexpected(r.error());
    }
    RunProgress queued;
    queued.kind = kind;
    queued.target_s = static_cast<double>(resolved.spec.time.duration);
    // A real run's need, which the queue waits for; a dry run holds no grid.
    queued.memory_bytes = kind == RunKind::dry_run ? 0 : resolved.base_bytes;
    if (kind == RunKind::dry_run) queued.message = std::string(kDryRunNoMemoryGate);
    if (Result<void> r = write_progress(folder, queued); !r) {
        undo(folder);
        return std::unexpected(r.error());
    }
    return folder;
}

}  // namespace

Result<fs::path> create_run_folder(const fs::path& runs_dir, std::string_view case_name, const ResolvedCase& resolved,
                                   std::string_view date, std::string_view commit, RunKind kind) {
    return make_run_folder(runs_dir, case_name, resolved, date, commit, false, kind);
}

Result<fs::path> create_run_folder(const fs::path& runs_dir, std::string_view case_name, const ResolvedCase& resolved,
                                   std::string_view date, const BuildIdentity& built, RunKind kind) {
    if (built.commit == "unknown") {
        return std::unexpected(Error{Code::unavailable,
                                     "this engine can't name its commit (built outside a git checkout and not from a "
                                     "git archive), so its runs can't be traced: pass the commit"});
    }
    return make_run_folder(runs_dir, case_name, resolved, date, built.commit, built.dirty, kind);
}

Result<fs::path> create_run_folder(const fs::path& runs_dir, std::string_view case_name, const ResolvedCase& resolved,
                                   std::string_view date, RunKind kind) {
    return create_run_folder(runs_dir, case_name, resolved, date, spade::build_identity(), kind);
}

}  // namespace spade::analysis
