#pragma once

// ---------------------------------------------------------------------------
// run_record.hpp -- the folder a flight-computer run leaves behind (docs/design/core/plans/2026-11-21-fc-core.md section 4; the checks a test makes of it
// are tests/fc_run_folder_check.hpp's K1-K11, written apart from this writer on purpose).
//
// A run folder holds, as flat `key: value` text (two-space nesting) and binary files:
//   run.yaml            who ran what: the identity strings the caller gives, the numbers the host knows (dt_ns, substeps, sensor_periods_ticks, seeds,
//                       trace_ticks, trace_hash, the boot fields), the grades with grade_run the WEAKEST of the world's, the mode's and the link's, and
//                       fc_compute_not_modelled: true -- the host never models the flight computer's own compute time, and the record says so.
//   trace.bin           the host's trace (lockstep.hpp: trace.bin v1)
//   commands.bin        the commands the rotors were given, the input of a replay (lockstep.hpp: commands.bin v1)
//   meters.yaml         the counts and stage latencies the RUNNER measured (the host measures none: it has no wall clock, and Spade's engine reads none)
//   profile.board.yaml  a copy of the board profile the run used; run.yaml's profile_hash is its SHA-256
//   manifest.yaml       `files:` then `  <name>: <SHA-256 of that file's bytes>` for every other file. SPADE DOES NOT SIGN: the consumer signs the
//                       manifest; no key is ever read or written here.
//
// Phase 1 writes mode `measured`. `budget` mode (costs.yaml, costs_hash) is phase 2 and is refused, saying so.
// Every text value is one line, non-empty, and free of a user's folder path (the refusal names the field and never quotes the value).
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "fc_host/lockstep.hpp"

namespace spade::fc_host {

// Everything the host cannot know and the runner does: who built what, which world and model, which link and firmware, which board profile.
struct RunIdentity {
    std::string run_id;
    std::string mode = "measured";
    std::string box;   // measured: the machine
    std::string tree;  // measured: the source tree
    std::string spade_build_id;
    std::string spade_commit;
    std::string engine_config_hash;
    std::string world_file_hash;
    std::string model_hash;
    std::string link_kind;
    std::string link_version;
    std::string software_identity;
    std::string package_hash;  // 64 lower-case hex
    std::string grade_world;   // reference | banded | best-effort
    std::string grade_mode;
    std::string grade_link;
    std::string profile_board_yaml;  // the text of the board profile the run used
};

struct StageLatency {
    std::string name;  // [a-z0-9_]+ : it becomes a key
    double min_ns = 0.0;
    double mean_ns = 0.0;
    double p99_ns = 0.0;
    double max_ns = 0.0;
};

// What the runner measured. A measured run must say its memory peak: absent is refused, never written as 0.
struct RunMeters {
    uint64_t overruns = 0;
    uint64_t late_samples = 0;
    uint64_t skipped_samples = 0;
    std::optional<uint64_t> memory_peak_bytes;
    std::vector<StageLatency> stages;
};

struct RunRecord {
    std::map<std::string, std::vector<std::byte>> files;  // relative name -> bytes; manifest.yaml included
};

// Builds the record from the host's trace so far, the identity and the meters. Refuses (invalid_argument, naming the field) what it cannot write
// truthfully; nothing is half-built.
[[nodiscard]] Result<RunRecord> build_run_record(const LockstepHost& host, const RunIdentity& identity, const RunMeters& meters);

// Writes the record into `dir`, creating it. Refuses a folder that already holds anything (a record is never overwritten) and leaves it as it was.
[[nodiscard]] Result<void> write_run_folder(const RunRecord& record, const std::filesystem::path& dir);

}  // namespace spade::fc_host
