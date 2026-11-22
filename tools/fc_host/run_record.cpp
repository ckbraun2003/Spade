#include "fc_host/run_record.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <fstream>
#include <string_view>
#include <system_error>

#include "world/sha256.hpp"

namespace spade::fc_host {

namespace {

[[nodiscard]] Error invalid(std::string what) { return Error{Code::invalid_argument, "fc_host: build_run_record: " + std::move(what)}; }

[[nodiscard]] std::string_view as_text(const std::vector<std::byte>& bytes) { return std::string_view(reinterpret_cast<const char*>(bytes.data()), bytes.size()); }

[[nodiscard]] std::vector<std::byte> bytes_of(std::string_view text) {
    std::vector<std::byte> out;
    out.reserve(text.size());
    for (const char c : text) out.push_back(static_cast<std::byte>(c));
    return out;
}

// The shortest text that reads back as the same number, with no locale in it.
template <class T>
[[nodiscard]] std::string number(T value) {
    char buf[48];
    const auto [end, ec] = std::to_chars(buf, buf + sizeof(buf), value);
    (void)ec;
    return std::string(buf, end);
}

// A user's folder in text: a drive path under Users, a Unix home, or a Unix Users path (the checker's K10 looks for the same). Never quoted back.
[[nodiscard]] bool has_user_path(std::string_view text) {
    std::string low(text);
    for (char& c : low) c = c == '\\' ? '/' : static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return low.find("/users/") != std::string::npos || low.find("/home/") != std::string::npos;
}

[[nodiscard]] bool is_hex64(std::string_view s) {
    return s.size() == 64 && std::all_of(s.begin(), s.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}

// One line, not empty, no user's folder.
[[nodiscard]] Result<void> check_line(const char* field, const std::string& value) {
    if (value.empty()) return std::unexpected(invalid(std::string(field) + " is empty"));
    if (value.find_first_of("\n\r\t") != std::string::npos) return std::unexpected(invalid(std::string(field) + " must be one line with no tab"));
    if (has_user_path(value)) return std::unexpected(invalid(std::string(field) + " holds a user's folder path"));
    return {};
}

[[nodiscard]] int grade_rank(const std::string& g) {
    if (g == "reference") return 0;
    if (g == "banded") return 1;
    if (g == "best-effort") return 2;
    return -1;
}

[[nodiscard]] bool stage_name_ok(const std::string& name) {
    return !name.empty() && std::all_of(name.begin(), name.end(), [](char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'; });
}

[[nodiscard]] Result<void> check_stage(const StageLatency& s) {
    if (!stage_name_ok(s.name)) return std::unexpected(invalid("stage name '" + s.name + "' must be lower-case letters, digits and underscores (it becomes a key)"));
    for (const double v : {s.min_ns, s.mean_ns, s.p99_ns, s.max_ns}) {
        if (!std::isfinite(v) || v < 0.0) return std::unexpected(invalid("stage " + s.name + ": a latency must be finite and >= 0"));
    }
    if (!(s.min_ns <= s.mean_ns && s.mean_ns <= s.max_ns && s.min_ns <= s.p99_ns && s.p99_ns <= s.max_ns)) {
        return std::unexpected(invalid("stage " + s.name + ": the latency statistics are out of order (min <= mean <= max and min <= p99 <= max)"));
    }
    return {};
}

}  // namespace

Result<RunRecord> build_run_record(const LockstepHost& host, const RunIdentity& id, const RunMeters& meters) {
    // MODE. Phase 1 writes measured runs; budget mode (costs.yaml) is phase 2.
    if (id.mode == "budget") return std::unexpected(invalid("mode budget is phase 2 (costs.yaml and costs_hash are not written yet); phase 1 writes measured runs"));
    if (id.mode != "measured") return std::unexpected(invalid("mode must be measured (got '" + id.mode + "')"));

    // THE IDENTITY: every value one line, not empty, free of a user's folder.
    const std::pair<const char*, const std::string*> required[] = {
        {"run_id", &id.run_id},
        {"box", &id.box},
        {"tree", &id.tree},
        {"spade_build_id", &id.spade_build_id},
        {"spade_commit", &id.spade_commit},
        {"engine_config_hash", &id.engine_config_hash},
        {"world_file_hash", &id.world_file_hash},
        {"model_hash", &id.model_hash},
        {"link_kind", &id.link_kind},
        {"link_version", &id.link_version},
        {"software_identity", &id.software_identity},
        {"package_hash", &id.package_hash},
        {"grade_world", &id.grade_world},
        {"grade_mode", &id.grade_mode},
        {"grade_link", &id.grade_link}};
    for (const auto& [field, value] : required) {
        if (Result<void> r = check_line(field, *value); !r) return std::unexpected(r.error());
    }
    if (!is_hex64(id.package_hash)) return std::unexpected(invalid("package_hash must be 64 lower-case hex digits"));
    int weakest = 0;
    const std::pair<const char*, const std::string*> grades[] = {{"grade_world", &id.grade_world}, {"grade_mode", &id.grade_mode}, {"grade_link", &id.grade_link}};
    for (const auto& [field, value] : grades) {
        const int rank = grade_rank(*value);
        if (rank < 0) return std::unexpected(invalid(std::string(field) + " must be reference, banded or best-effort"));
        weakest = std::max(weakest, rank);
    }
    const char* const grade_run = weakest == 0 ? "reference" : (weakest == 1 ? "banded" : "best-effort");
    if (id.profile_board_yaml.empty()) return std::unexpected(invalid("profile_board_yaml is empty (a run copies the board profile it used)"));
    if (has_user_path(id.profile_board_yaml)) return std::unexpected(invalid("profile_board_yaml holds a user's folder path"));

    // THE METERS.
    if (!meters.memory_peak_bytes) return std::unexpected(invalid("memory_peak_bytes is absent (a measured run says its memory peak; 0 is not a stand-in)"));
    if (meters.stages.empty()) return std::unexpected(invalid("no stage latency statistics (a run has at least one stage)"));
    for (std::size_t i = 0; i < meters.stages.size(); ++i) {
        if (Result<void> r = check_stage(meters.stages[i]); !r) return std::unexpected(r.error());
        for (std::size_t j = 0; j < i; ++j) {
            if (meters.stages[j].name == meters.stages[i].name) return std::unexpected(invalid("stage " + meters.stages[i].name + " is listed twice"));
        }
    }

    // THE HOST'S FILES.
    const Result<std::vector<std::byte>> trace = host.trace_bytes();
    if (!trace) return std::unexpected(trace.error());
    const std::vector<std::byte> commands = host.commands_bytes();

    // run.yaml
    std::string run;
    const auto line = [&run](std::string_view key, std::string_view value) {
        run.append(key).append(": ").append(value).push_back('\n');
    };
    std::string periods = "[";
    for (std::size_t i = 0; i < host.sensor_periods_ticks().size(); ++i) periods += (i == 0 ? "" : ", ") + number(host.sensor_periods_ticks()[i]);
    periods += "]";
    const std::optional<BootNote> boot = host.boot_note();
    std::string seeds = "[" + number(host.world_seed());
    if (boot) seeds += ", " + number(boot->config.seed);
    seeds += "]";
    line("run_id", id.run_id);
    line("mode", id.mode);
    line("box", id.box);
    line("tree", id.tree);
    line("spade_build_id", id.spade_build_id);
    line("spade_commit", id.spade_commit);
    line("engine_config_hash", id.engine_config_hash);
    line("world_file_hash", id.world_file_hash);
    line("model_hash", id.model_hash);
    line("seeds", seeds);
    line("dt_ns", number(host.dt_ns()));
    line("substeps", number(host.substeps()));
    line("sensor_periods_ticks", periods);
    line("command_latency_ticks", number(host.config().command_latency_ticks));
    line("profile_hash", sha256_hex(id.profile_board_yaml));
    line("link_kind", id.link_kind);
    line("link_version", id.link_version);
    line("software_identity", id.software_identity);
    line("package_hash", id.package_hash);
    line("grade_world", id.grade_world);
    line("grade_mode", id.grade_mode);
    line("grade_link", id.grade_link);
    line("grade_run", grade_run);
    line("fc_compute_not_modelled", "true");
    line("trace_ticks", number(host.trace_ticks()));
    line("trace_hash", host.trace_hash());
    if (boot) {
        line("boot_steps", number(boot->steps));
        line("boot_end_time_usec", number(boot->end_time_usec));
        line("boot_sigma_accel", number(boot->config.sigma_accel));
        line("boot_sigma_gyro", number(boot->config.sigma_gyro));
        line("boot_sigma_pressure_pa", number(boot->config.sigma_pressure_pa));
        line("boot_sigma_mag_ut", number(boot->config.sigma_mag_ut));
    }

    // meters.yaml
    std::string met;
    met += "overruns: " + number(meters.overruns) + "\n";
    met += "late_samples: " + number(meters.late_samples) + "\n";
    met += "skipped_samples: " + number(meters.skipped_samples) + "\n";
    met += "memory_peak_bytes: " + number(*meters.memory_peak_bytes) + "\n";
    met += "stages:\n";
    for (const StageLatency& s : meters.stages) {
        met += "  " + s.name + ":\n    latency_ns:\n";
        met += "      min: " + number(s.min_ns) + "\n";
        met += "      mean: " + number(s.mean_ns) + "\n";
        met += "      p99: " + number(s.p99_ns) + "\n";
        met += "      max: " + number(s.max_ns) + "\n";
    }

    RunRecord record;
    record.files["run.yaml"] = bytes_of(run);
    record.files["trace.bin"] = *trace;
    record.files["commands.bin"] = commands;
    record.files["meters.yaml"] = bytes_of(met);
    record.files["profile.board.yaml"] = bytes_of(id.profile_board_yaml);

    // manifest.yaml: the SHA-256 of every other file. Unsigned: the consumer signs it.
    std::string manifest = "files:\n";
    for (const auto& [name, bytes] : record.files) manifest += "  " + name + ": " + sha256_hex(as_text(bytes)) + "\n";
    record.files["manifest.yaml"] = bytes_of(manifest);
    return record;
}

Result<void> write_run_folder(const RunRecord& record, const std::filesystem::path& dir) {
    const auto fail = [](Code code, std::string what) { return std::unexpected(Error{code, "fc_host: write_run_folder: " + std::move(what)}); };
    for (const auto& [name, bytes] : record.files) {
        (void)bytes;
        if (name.empty() || name.find_first_of("/\\") != std::string::npos || name == "." || name == "..") {
            return fail(Code::invalid_argument, "a file name must be a plain name in the folder (got '" + name + "')");
        }
    }
    std::error_code ec;
    if (std::filesystem::exists(dir, ec)) {
        if (!std::filesystem::is_directory(dir, ec)) return fail(Code::invalid_argument, "the target exists and is not a folder");
        if (!std::filesystem::is_empty(dir, ec)) return fail(Code::invalid_argument, "the folder is not empty (a record is never overwritten)");
    }
    std::filesystem::create_directories(dir, ec);
    if (ec) return fail(Code::io_error, "cannot create the folder: " + ec.message());
    for (const auto& [name, bytes] : record.files) {
        std::ofstream out(dir / name, std::ios::binary);
        out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        out.close();
        if (!out) return fail(Code::io_error, "cannot write " + name);
    }
    return {};
}

}  // namespace spade::fc_host
