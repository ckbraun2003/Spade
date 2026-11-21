#pragma once

// ---------------------------------------------------------------------------
// fc_run_folder_check.hpp -- what a test asserts about a flight-computer run folder (checks K1-K11 of
// docs/design/test-docs/plans/2026-11-21-fc-tests.md, section 3).
//
// Header-only test support, like engine/testing/replay.hpp: compiled into the tests and never installed. It is WRITTEN APART FROM
// Core's run-record writer (tools/fc_host/record, docs/design/core/plans/2026-11-21-fc-core.md section 4) on purpose (TD-4): its
// own small reader, its own file list, its own hash check. A checker that shared the writer's code would share its bugs.
//
// THE FORMAT IS A PROPOSAL. Core's section names the files (run.yaml, trace.bin, commands.bin, meters.yaml, profile.board.yaml,
// costs.yaml, manifest.yaml) and what run.yaml holds, but not the key names or the manifest's layout. The names below are mine and
// live in one place (kRequiredRunKeys and the reads in check_*), so Core's writer, when it lands, changes them here and nowhere
// else. The layout assumed:
//   run.yaml, meters.yaml   flat `key: value` lines, nested by two-space indentation, read as dotted keys (`stages.chain.latency_ns.p99`)
//   manifest.yaml           `files:` then `  <relative name>: <64 lower-case hex SHA-256 of the file's bytes>`
//
// VERDICTS. `pass`, `fail`, `absent` (a thing that has to be there is not: never a pass) and `unchecked` (the check needs a format or
// a log this checker does not have yet: never a pass either). ok() is "no fail and no absent"; complete() is also "nothing unchecked",
// which is what a release will ask once Core's trace format and PX4's log are readable.
//
// The checks that depend on them are reported `unchecked` with their reason, so one cannot go quiet unnoticed:
//   K3.trace_hash  recomputing the rolling hash from trace.bin (the hash and the record layout are not pinned)
//   K4.log         the autopilot log past its header: parse, time span, firmware version, parameter hash
//   K6.recount     the overrun recount from per-tick costs
//   K11.replay     fc_host --replay reproducing the trace
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "world/sha256.hpp"

namespace spade::fc {

using Files = std::map<std::string, std::string>;  // relative name (forward slashes) -> the file's bytes

enum class Verdict { pass, fail, absent, unchecked };

struct CheckResult {
    std::string id;
    Verdict verdict = Verdict::pass;
    std::string detail;
};

struct Report {
    std::vector<CheckResult> results;

    [[nodiscard]] bool ok() const {
        return std::none_of(results.begin(), results.end(),
                            [](const CheckResult& c) { return c.verdict == Verdict::fail || c.verdict == Verdict::absent; });
    }
    [[nodiscard]] bool complete() const {
        return ok() && std::none_of(results.begin(), results.end(),
                                    [](const CheckResult& c) { return c.verdict == Verdict::unchecked; });
    }
    [[nodiscard]] const CheckResult* find(std::string_view id, Verdict v) const {
        for (const CheckResult& c : results)
            if (c.id == id && c.verdict == v) return &c;
        return nullptr;
    }
    [[nodiscard]] std::string text() const {
        static constexpr const char* names[] = {"pass", "FAIL", "absent:", "unchecked:"};
        std::string s;
        for (const CheckResult& c : results)
            s += c.id + " " + names[static_cast<int>(c.verdict)] + (c.detail.empty() ? "" : "  " + c.detail) + "\n";
        return s;
    }
};

struct RunFolder {
    Files files;
};

// ----- a small reader for the flat YAML the record uses -----------------------------------------------------------------------------

// Lines of `key: value` or `key:` (a section opener), nested by two spaces, comments after `#` at the start of a line. Returns dotted
// keys. A tab, an odd indent, a repeated key or a line with no colon is an error (named in *err), and the reading stops there.
inline std::map<std::string, std::string> read_flat_yaml(std::string_view text, std::string* err) {
    std::map<std::string, std::string> out;
    std::vector<std::string> path;  // the section keys open at each indent level
    size_t pos = 0;
    int line_no = 0;
    auto fail = [&](const std::string& why) {
        if (err != nullptr) *err = "line " + std::to_string(line_no) + ": " + why;
    };
    while (pos <= text.size()) {
        const size_t nl = text.find('\n', pos);
        std::string_view line = text.substr(pos, nl == std::string_view::npos ? std::string_view::npos : nl - pos);
        pos = nl == std::string_view::npos ? text.size() + 1 : nl + 1;
        ++line_no;
        if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
        size_t indent = 0;
        while (indent < line.size() && line[indent] == ' ') ++indent;
        if (indent < line.size() && line[indent] == '\t') {
            fail("a tab");
            return out;
        }
        std::string_view body = line.substr(indent);
        if (body.empty() || body.front() == '#') continue;
        if (indent % 2 != 0 || indent / 2 > path.size()) {
            fail("an odd or skipped indent");
            return out;
        }
        const size_t colon = body.find(':');
        if (colon == std::string_view::npos || colon == 0) {
            fail("no `key:`");
            return out;
        }
        path.resize(indent / 2);
        std::string key(body.substr(0, colon));
        std::string value(body.substr(colon + 1));
        const size_t a = value.find_first_not_of(' ');
        value = a == std::string::npos ? std::string() : value.substr(a);
        while (!value.empty() && value.back() == ' ') value.pop_back();
        if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\'')))
            value = value.substr(1, value.size() - 2);
        std::string full;
        for (const std::string& p : path) full += p + ".";
        full += key;
        if (value.empty()) {
            path.push_back(key);  // a section opener (it may also be an empty scalar; only a child tells)
            continue;
        }
        if (out.count(full) != 0) {
            fail("duplicate key " + full);
            return out;
        }
        out[full] = value;
    }
    return out;
}

// ----- the folder -------------------------------------------------------------------------------------------------------------------

// Every regular file under `dir`, by forward-slash relative name. A folder that is not there reads as empty (and so is absent).
inline RunFolder load_run_folder(const std::filesystem::path& dir) {
    RunFolder f;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) return f;
    for (std::filesystem::recursive_directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        if (!it->is_regular_file(ec)) continue;
        std::ifstream in(it->path(), std::ios::binary);
        std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        f.files[std::filesystem::relative(it->path(), dir).generic_string()] = std::move(bytes);
    }
    return f;
}

// ----- the proposed key names (one place) -------------------------------------------------------------------------------------------

inline constexpr std::string_view kRequiredRunKeys[] = {
    "run_id", "mode", "spade_build_id", "spade_commit", "engine_config_hash", "world_file_hash", "model_hash", "seeds", "dt_ns",
    "substeps", "sensor_periods_ticks", "profile_hash", "link_kind", "link_version", "software_identity", "package_hash",
    "grade_world", "grade_mode", "grade_link", "grade_run", "fc_compute_not_modelled", "trace_ticks", "trace_hash"};

namespace detail {

inline bool is_hex64(std::string_view s) {
    return s.size() == 64 && std::all_of(s.begin(), s.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}

inline bool parse_uint(const std::string& s, uint64_t* out) {
    if (s.empty() || !std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; })) return false;
    char* end = nullptr;
    *out = std::strtoull(s.c_str(), &end, 10);
    return end != nullptr && *end == '\0';
}

inline bool parse_finite(const std::string& s, double* out) {
    if (s.empty()) return false;
    char* end = nullptr;
    *out = std::strtod(s.c_str(), &end);
    return end != nullptr && *end == '\0' && std::isfinite(*out);
}

// 0 reference, 1 banded, 2 best-effort; -1 for anything else. The weakest has the highest rank.
inline int grade_rank(const std::string& g) {
    if (g == "reference") return 0;
    if (g == "banded") return 1;
    if (g == "best-effort") return 2;
    return -1;
}

inline bool looks_binary(const std::string& bytes) { return bytes.find('\0') != std::string::npos; }

// A user's folder in text: a drive path under Users, a Unix home, or a Unix Users path. Never quoted back (the path is the secret).
inline bool has_user_path(const std::string& bytes) {
    std::string low = bytes;
    std::transform(low.begin(), low.end(), low.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    for (char& c : low)
        if (c == '\\') c = '/';
    return low.find("/users/") != std::string::npos || low.find("/home/") != std::string::npos;
}

}  // namespace detail

// ----- K4 (the part that needs no PX4 log parser): the .ulg header ------------------------------------------------------------------

// A ULog file opens with the 7 magic bytes `ULog 01 12 35`, a version byte and a 64-bit start time in microseconds: 16 bytes.
inline CheckResult check_ulog_header(std::string_view bytes) {
    CheckResult r{"K4.header", Verdict::pass, {}};
    static constexpr char kMagic[7] = {'U', 'L', 'o', 'g', '\x01', '\x12', '\x35'};
    if (bytes.size() < 16) {
        r.verdict = Verdict::fail;
        r.detail = "the autopilot log is shorter than a ULog header (" + std::to_string(bytes.size()) + " bytes)";
    } else if (bytes.substr(0, 7) != std::string_view(kMagic, 7)) {
        r.verdict = Verdict::fail;
        r.detail = "the autopilot log does not open with the ULog magic";
    }
    return r;
}

// ----- the folder check -------------------------------------------------------------------------------------------------------------

inline Report check_run_folder(const RunFolder& folder) {
    using detail::is_hex64;
    Report rep;
    const Files& files = folder.files;
    auto add = [&](std::string id, Verdict v, std::string d = {}) { rep.results.push_back({std::move(id), v, std::move(d)}); };
    auto has = [&](const std::string& name) { return files.count(name) != 0; };

    // run.yaml first: the mode says which files are required.
    std::map<std::string, std::string> run;
    std::string run_err;
    if (has("run.yaml")) run = read_flat_yaml(files.at("run.yaml"), &run_err);
    const std::string mode = run.count("mode") != 0 ? run.at("mode") : std::string("budget");

    // K1: every required file, by name.
    {
        std::vector<std::string> want = {"run.yaml", "trace.bin", "commands.bin", "meters.yaml", "profile.board.yaml", "manifest.yaml"};
        if (mode == "budget") want.push_back("costs.yaml");
        std::string missing;
        for (const std::string& n : want)
            if (!has(n)) missing += (missing.empty() ? "" : ", ") + n;
        if (missing.empty())
            add("K1", Verdict::pass);
        else
            add("K1", Verdict::absent, "missing: " + missing);
    }

    // K2: the manifest. A SHA-256 for every file, every file matches, no file unlisted, no listed file missing.
    if (has("manifest.yaml")) {
        std::string err;
        const auto m = read_flat_yaml(files.at("manifest.yaml"), &err);
        std::string bad;
        auto note = [&](const std::string& s) { bad += (bad.empty() ? "" : "; ") + s; };
        if (!err.empty()) note("manifest.yaml unreadable (" + err + ")");
        size_t listed = 0;
        for (const auto& [key, hash] : m) {
            if (key.rfind("files.", 0) != 0) continue;
            ++listed;
            const std::string name = key.substr(6);
            if (!is_hex64(hash)) {
                note(name + ": not a 64-digit lower-case SHA-256");
            } else if (!has(name)) {
                note(name + ": listed but not in the folder");
            } else if (spade::sha256_hex(files.at(name)) != hash) {
                note(name + ": does not match its SHA-256");
            }
        }
        if (listed == 0) note("no `files:` entries");
        for (const auto& [name, bytes] : files) {
            (void)bytes;
            if (name != "manifest.yaml" && m.count("files." + name) == 0) note(name + ": in the folder but not in the manifest");
        }
        if (bad.empty())
            add("K2", Verdict::pass);
        else
            add("K2", Verdict::fail, bad);
    } else {
        add("K2", Verdict::absent, "manifest.yaml is missing");
    }

    // K3: run.yaml holds every field; the numbers are numbers.
    if (has("run.yaml")) {
        std::string bad;
        auto note = [&](const std::string& s) { bad += (bad.empty() ? "" : "; ") + s; };
        if (!run_err.empty()) note("run.yaml unreadable (" + run_err + ")");
        for (const std::string_view key : kRequiredRunKeys)
            if (run.count(std::string(key)) == 0) note("missing key " + std::string(key));
        if (mode == "budget" && run.count("costs_hash") == 0) note("missing key costs_hash");
        uint64_t n = 0;
        for (const char* key : {"dt_ns", "substeps"})
            if (run.count(key) != 0 && (!detail::parse_uint(run.at(key), &n) || n == 0)) note(std::string(key) + " is not a positive integer");
        if (run.count("trace_ticks") != 0 && !detail::parse_uint(run.at("trace_ticks"), &n)) note("trace_ticks is not a count");
        if (run.count("trace_hash") != 0 && !is_hex64(run.at("trace_hash"))) note("trace_hash is not a SHA-256");
        if (bad.empty())
            add("K3", Verdict::pass);
        else
            add("K3", Verdict::fail, bad);
    } else {
        add("K3", Verdict::absent, "run.yaml is missing");
    }
    add("K3.trace_hash", Verdict::unchecked, "the trace record layout and the rolling hash are not pinned (Core)");
    add("K4.log", Verdict::unchecked, "the autopilot log is opaque to the record; its parse, span, firmware and parameter hash come with the PX4 run");

    // K5: the copies match their hashes; the mode is one of two; a measured run names its box and tree.
    if (has("run.yaml")) {
        std::string bad;
        auto note = [&](const std::string& s) { bad += (bad.empty() ? "" : "; ") + s; };
        if (mode != "budget" && mode != "measured") note("mode is neither budget nor measured");
        if (has("profile.board.yaml") && run.count("profile_hash") != 0 && spade::sha256_hex(files.at("profile.board.yaml")) != run.at("profile_hash"))
            note("profile.board.yaml does not match profile_hash");
        if (mode == "budget" && has("costs.yaml") && run.count("costs_hash") != 0 && spade::sha256_hex(files.at("costs.yaml")) != run.at("costs_hash"))
            note("costs.yaml does not match costs_hash");
        if (mode == "measured")
            for (const char* key : {"box", "tree"})
                if (run.count(key) == 0 || run.at(key).empty()) note(std::string("a measured run has no ") + key);
        if (bad.empty())
            add("K5", Verdict::pass);
        else
            add("K5", Verdict::fail, bad);
    } else {
        add("K5", Verdict::absent, "run.yaml is missing");
    }

    // K6: the meters are present and finite.
    if (has("meters.yaml")) {
        std::string err;
        const auto m = read_flat_yaml(files.at("meters.yaml"), &err);
        std::string bad;
        auto note = [&](const std::string& s) { bad += (bad.empty() ? "" : "; ") + s; };
        if (!err.empty()) note("meters.yaml unreadable (" + err + ")");
        for (const char* key : {"overruns", "late_samples", "skipped_samples"}) {
            uint64_t n = 0;
            if (m.count(key) == 0)
                note(std::string("missing ") + key);
            else if (!detail::parse_uint(m.at(key), &n))
                note(std::string(key) + " is not a count");
        }
        std::set<std::string> stages;
        for (const auto& [key, value] : m) {
            const size_t a = key.find("stages.");
            const size_t b = key.find(".latency_ns.");
            if (a != 0 || b == std::string::npos) continue;
            stages.insert(key.substr(7, b - 7));
            double d = 0;
            if (!detail::parse_finite(value, &d) || d < 0) note(key + " is not a finite, non-negative number");
        }
        if (stages.empty()) note("no stage latency statistics");
        for (const std::string& s : stages)
            for (const char* stat : {"min", "mean", "p99", "max"})
                if (m.count("stages." + s + ".latency_ns." + stat) == 0) note("stage " + s + " has no latency_ns." + stat);
        if (mode == "measured") {
            uint64_t n = 0;
            if (m.count("memory_peak_bytes") == 0 || !detail::parse_uint(m.at("memory_peak_bytes"), &n)) note("a measured run needs a memory_peak_bytes count");
        }
        if (bad.empty())
            add("K6", Verdict::pass);
        else
            add("K6", Verdict::fail, bad);
    } else {
        add("K6", Verdict::absent, "meters.yaml is missing");
    }
    add("K6.recount", Verdict::unchecked, "the per-tick costs a recount needs are not in the record yet (Core)");

    // K7: the package hash is a hash.
    if (has("run.yaml")) {
        if (run.count("package_hash") != 0 && is_hex64(run.at("package_hash")))
            add("K7", Verdict::pass);
        else
            add("K7", Verdict::fail, "package_hash is not a 64-digit lower-case SHA-256");
    } else {
        add("K7", Verdict::absent, "run.yaml is missing");
    }

    // K9: the not-modelled flag; the run's grade is the weakest of the world's, the mode's and the link's.
    if (has("run.yaml")) {
        std::string bad;
        auto note = [&](const std::string& s) { bad += (bad.empty() ? "" : "; ") + s; };
        if (run.count("fc_compute_not_modelled") == 0 || run.at("fc_compute_not_modelled") != "true")
            note("fc_compute_not_modelled is not true");
        int weakest = -2;
        bool graded = true;
        for (const char* key : {"grade_world", "grade_mode", "grade_link", "grade_run"}) {
            const int rank = run.count(key) != 0 ? detail::grade_rank(run.at(key)) : -1;
            if (rank < 0) {
                note(std::string(key) + " is missing or not one of reference, banded, best-effort");
                graded = false;
            } else if (std::string(key) != "grade_run") {
                weakest = std::max(weakest, rank);
            }
        }
        if (graded && detail::grade_rank(run.at("grade_run")) != weakest) note("grade_run is not the weakest of grade_world, grade_mode and grade_link");
        if (bad.empty())
            add("K9", Verdict::pass);
        else
            add("K9", Verdict::fail, bad);
    } else {
        add("K9", Verdict::absent, "run.yaml is missing");
    }

    // K10: no user folder in any text file. The detail names the file and never the path.
    {
        std::string bad;
        for (const auto& [name, bytes] : files)
            if (!detail::looks_binary(bytes) && detail::has_user_path(bytes)) bad += (bad.empty() ? "" : ", ") + name;
        if (bad.empty())
            add("K10", Verdict::pass);
        else
            add("K10", Verdict::fail, "a user folder path is in: " + bad);
    }

    add("K11.replay", Verdict::unchecked, "fc_host --replay is Core's and not built yet");
    return rep;
}

}  // namespace spade::fc
