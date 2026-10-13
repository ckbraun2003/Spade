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
#include "world/detail/yaml_text.hpp"

namespace spade::analysis {

namespace {

namespace fs = std::filesystem;
using yaml_text::at;
using yaml_text::quote_yaml;
using yaml_text::string_field;

constexpr std::array<std::string_view, 7> kStateNames = {"queued", "calibrating", "running", "done",
                                                         "refused", "faulted", "cancelled"};
constexpr const char* kProgressFile = "progress.yaml";

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

[[nodiscard]] Result<RunProgress> parse_progress(const YAML::Node& root) {
    if (Result<void> ok = yaml_text::check_map(root, "progress",
                                               {"state", "simulated_s", "target_s", "steps", "elapsed_s",
                                                "ms_per_step", "memory_bytes", "message"},
                                               "progress schema v1");
        !ok) {
        return std::unexpected(ok.error());
    }
    RunProgress p;
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

}  // namespace

std::string progress_to_yaml(const RunProgress& p) {
    std::string out;
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
    try {
        return parse_progress(YAML::Load(std::string(text)));
    } catch (const YAML::Exception& e) {
        return std::unexpected(Error{Code::invalid_argument, std::string(e.what())});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, std::string("progress parse failed: ") + e.what()});
    }
}

Result<void> write_progress(const fs::path& folder, const RunProgress& p) {
    return detail::replace_file(folder / kProgressFile, progress_to_yaml(p));
}

Result<RunProgress> read_progress(const fs::path& folder) {
    const Result<std::string> text = detail::read_file_shared(folder / kProgressFile);
    if (!text) return std::unexpected(text.error());
    return progress_from_yaml(*text);
}

fs::path run_folder(const fs::path& runs_dir, std::string_view case_name, const ResolvedCase& resolved,
                    std::string_view date, std::string_view commit) {
    const std::string name = std::string(date) + "_" + std::format("{:016x}", resolved.hash) + "_" +
                             std::string(commit.substr(0, std::min<std::size_t>(commit.size(), 12)));
    return runs_dir / fs::path(std::string(case_name)) / name;
}

Result<fs::path> create_run_folder(const fs::path& runs_dir, std::string_view case_name, const ResolvedCase& resolved,
                                   std::string_view date, std::string_view commit) {
    const fs::path folder = run_folder(runs_dir, case_name, resolved, date, commit);
    std::error_code ec;
    fs::create_directories(folder.parent_path(), ec);
    if (ec) return std::unexpected(Error{Code::io_error, "cannot make '" + path_text(folder.parent_path()) + "': " + ec.message()});
    if (!fs::create_directory(folder, ec)) {
        if (ec) return std::unexpected(Error{Code::io_error, "cannot make '" + path_text(folder) + "': " + ec.message()});
        return std::unexpected(Error{Code::invalid_argument,
                                     "the run '" + path_text(folder) + "' exists already: a run is never overwritten"});
    }
    const Result<std::string> text = case_to_yaml(resolved.spec);
    if (!text) return std::unexpected(text.error());
    if (Result<void> r = detail::replace_file(folder / "case.yaml", *text); !r) return std::unexpected(r.error());
    RunProgress queued;
    queued.target_s = static_cast<double>(resolved.spec.time.duration);
    if (Result<void> r = write_progress(folder, queued); !r) return std::unexpected(r.error());
    return folder;
}

}  // namespace spade::analysis
