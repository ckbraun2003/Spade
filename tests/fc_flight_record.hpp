#pragma once
// ---------------------------------------------------------------------------
// fc_flight_record.hpp -- the runner's side of the flight film's run record (docs/design/core/plans/2026-11-21-fc-core.md sections 3.5 and 4): the identity and the
// meters a LIVE run supplies to fc_host::build_run_record, built from values the engine and the run really have, and refusing (never filling in) what it lacks.
//
//   - The ENGINE'S CONFIGURATION DIGEST is RUNNER-MADE from ENGINE-MADE values: SHA-256 of a canonical text whose first line is the replay config hash the engine
//     keeps (a uint64, as 16 lower-case hex digits) and then one line per module in set order: its name, its version and its configuration hash (the engine's
//     FNV-1a 64 over the module's image, 16 hex digits). The engine exposes no 64-digit digest itself; if Core would rather it did, this helper goes.
//   - The PACKAGE (the software under test: the PX4 image) arrives from its owner: package_hash, the image id (64 lower-case hex; docker's `sha256:` prefix is
//     taken off), is an INPUT. Missing or malformed refuses the record; there is no fallback. The software identity is composed here so nobody reads it as an
//     upstream release: "image id of the local build of PX4 <version> on <base image>; recipe sha256 <the Dockerfile's SHA-256>", the base image being the
//     Dockerfile's FROM line, which must be pinned by digest (a FROM that is not is refused: nothing is fetched to find one).
//   - The GRADES are claims on one axis (core/grade.hpp): grade_world is the engine's own grade for the world's module set on its backend (Simulation::grade()),
//     grade_mode is best-effort (measured mode has no golden and no band), grade_link is best-effort (a live autopilot has no reference to band against).
//   - The meters: the process's peak memory (measure_memory_peak_bytes: an error, never 0) and the wait and the host gap of every exchange (summarise_stage).
//     The counts a modelled host would detect (overruns, late and skipped samples) are 0: a live link has none to count.
//   - Nothing here carries a path, a user or a machine name: the box is the fixed string "windows-docker-desktop", and the files are named by their hashes.
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include "core/build_identity.hpp"
#include "core/error.hpp"
#include "core/grade.hpp"
#include "fc_host/lockstep.hpp"
#include "fc_host/run_record.hpp"
#include "fc_link/run_support.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"
#include "world/sha256.hpp"

namespace fc_flight_test {

namespace detail {

[[nodiscard]] inline std::string hex16(uint64_t v) {
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(v));
    return std::string(buf);
}

[[nodiscard]] inline bool is_hex64(const std::string& s) {
    return s.size() == 64 && std::all_of(s.begin(), s.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}

[[nodiscard]] inline spade::Error refusal(const std::string& what) { return spade::Error{spade::Code::invalid_argument, "build_flight_record: " + what}; }

// The first `FROM` line's image reference, when it is pinned by digest.
[[nodiscard]] inline spade::Result<std::string> pinned_base_image(const std::filesystem::path& dockerfile) {
    std::ifstream in(dockerfile);
    if (!in) return std::unexpected(refusal("recipe: the Dockerfile cannot be read"));
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("FROM ", 0) != 0) continue;
        std::string image = line.substr(5);
        while (!image.empty() && (image.back() == '\r' || image.back() == ' ')) image.pop_back();
        if (image.find("@sha256:") == std::string::npos) return std::unexpected(refusal("recipe: the Dockerfile's FROM line is not pinned by digest (nothing is fetched to find one)"));
        return image;
    }
    return std::unexpected(refusal("recipe: the Dockerfile has no FROM line"));
}

[[nodiscard]] inline std::string build_id() {
#if defined(_MSC_VER)
    std::string id = "msvc";
#elif defined(__clang__)
    std::string id = "clang";
#else
    std::string id = "gcc";
#endif
#ifdef NDEBUG
    return id + "-release";
#else
    return id + "-debug";
#endif
}

}  // namespace detail

// SHA-256 (64 lower-case hex) of the canonical text described above.
[[nodiscard]] inline std::string engine_config_digest(uint64_t replay_config_hash, std::span<const spade::modules::ConfigurationEntry> modules) {
    std::string text = "replay_config_hash " + detail::hex16(replay_config_hash) + "\n";
    for (const spade::modules::ConfigurationEntry& m : modules) text += m.module + " v" + std::to_string(m.version) + " " + detail::hex16(m.hash) + "\n";
    return spade::sha256_hex(text);
}

struct FlightRecordInputs {
    std::string run_id;
    std::string package_hash;          // the software under test: the image id, 64 lower-case hex (a leading `sha256:` is taken off); from its owner
    std::string link_version;          // e.g. "v1.17.0"
    std::filesystem::path dockerfile;  // the recipe of the image: hashed, and its FROM line names the base image
    std::filesystem::path model_file;  // hashed: model_hash
    std::filesystem::path world_file;  // hashed: world_file_hash (the file that assembles the world)
    std::string profile_board_yaml;    // the text of the profile the run used
    std::vector<double> wait_ns;       // per exchange: send to reply accepted
    std::vector<double> gap_ns;        // per exchange: the host gap before it
};

// The run record of a live run, from the host's trace so far.
[[nodiscard]] inline spade::Result<spade::fc_host::RunRecord> build_flight_record(const spade::fc_host::LockstepHost& host, spade::Simulation& sim,
                                                                                  const FlightRecordInputs& in) {
    using spade::fc_host::RunIdentity;
    using spade::fc_host::RunMeters;
    std::string package = in.package_hash;
    if (package.rfind("sha256:", 0) == 0) package.erase(0, 7);
    if (!detail::is_hex64(package)) return std::unexpected(detail::refusal("package_hash must be the image id: 64 lower-case hex"));
    const auto base = detail::pinned_base_image(in.dockerfile);
    if (!base) return std::unexpected(base.error());
    const auto recipe_hash = spade::fc_link::hash_file(in.dockerfile);
    if (!recipe_hash) return std::unexpected(detail::refusal("recipe: " + recipe_hash.error().context));
    const auto model_hash = spade::fc_link::hash_file(in.model_file);
    if (!model_hash) return std::unexpected(detail::refusal("model_hash: " + model_hash.error().context));
    const auto world_hash = spade::fc_link::hash_file(in.world_file);
    if (!world_hash) return std::unexpected(detail::refusal("world_file_hash: " + world_hash.error().context));

    const auto replay = sim.arenas().array(sim.replay_config_array());
    if (!replay || replay->empty()) return std::unexpected(detail::refusal("the engine's replay config row cannot be read"));
    const spade::Grade grade = sim.grade().grade;
    if (grade == spade::Grade::absent) return std::unexpected(detail::refusal("the world's module set has no grade"));
    const std::vector<spade::modules::ConfigurationEntry> modules = sim.configuration_report();

    RunMeters meters;
    const auto peak = spade::fc_link::measure_memory_peak_bytes();
    if (!peak) return std::unexpected(detail::refusal("memory peak: " + peak.error().context));
    meters.memory_peak_bytes = *peak;
    const auto wait = spade::fc_link::summarise_stage("px4_wait", in.wait_ns);
    if (!wait) return std::unexpected(detail::refusal("px4_wait: " + wait.error().context));
    const auto gap = spade::fc_link::summarise_stage("host_gap", in.gap_ns);
    if (!gap) return std::unexpected(detail::refusal("host_gap: " + gap.error().context));
    meters.stages = {*wait, *gap};

    const std::string commit(spade::build_identity().commit);
    RunIdentity id;
    id.run_id = in.run_id;
    id.mode = "measured";
    id.box = "windows-docker-desktop";
    id.tree = commit.substr(0, 12);
    id.spade_build_id = detail::build_id();
    id.spade_commit = commit;
    id.engine_config_hash = engine_config_digest((*replay)[0].config_hash, modules);
    id.world_file_hash = *world_hash;
    id.model_hash = *model_hash;
    id.link_kind = "px4";
    id.link_version = in.link_version;
    id.software_identity = "image id of the local build of PX4 " + in.link_version + " on " + *base + "; recipe sha256 " + *recipe_hash;
    id.package_hash = package;
    id.grade_world = std::string(spade::grade_name(grade));
    id.grade_mode = "best-effort";
    id.grade_link = "best-effort";
    id.profile_board_yaml = in.profile_board_yaml;
    return spade::fc_host::build_run_record(host, id, meters);
}

}  // namespace fc_flight_test
