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
// ---------------------------------------------------------------------------
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "fc_host/lockstep.hpp"
#include "fc_host/run_record.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

namespace fc_flight_test {

// SHA-256 (64 lower-case hex) of the canonical text described above. RED: not written yet.
[[nodiscard]] inline std::string engine_config_digest(uint64_t, std::span<const spade::modules::ConfigurationEntry>) { return std::string(); }

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

// The run record of a live run, from the host's trace so far. RED: not written yet.
[[nodiscard]] inline spade::Result<spade::fc_host::RunRecord> build_flight_record(const spade::fc_host::LockstepHost&, const spade::Simulation&,
                                                                                  const FlightRecordInputs&) {
    return std::unexpected(spade::Error{spade::Code::unavailable, "build_flight_record: not written yet"});
}

}  // namespace fc_flight_test
