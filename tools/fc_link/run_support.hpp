#pragma once

// ---------------------------------------------------------------------------
// fc_link/run_support.hpp -- the runner's side of the run record (docs/design/core/plans/2026-11-21-fc-core.md section 4; Core's writer is
// tools/fc_host/run_record.hpp). Core's host measures nothing (it has no wall clock), so the runner measures what meters.yaml holds and hashes
// what run.yaml names. Three helpers, each refusing rather than guessing:
//   - measure_memory_peak_bytes(): the process's peak memory so far. An error, NEVER 0: a measured run must say its peak, and an absent one is
//     refused by Core's writer, so a platform that cannot tell must not return a number that looks like one.
//   - hash_file(): the SHA-256 of a file's bytes (the world file, the model, the package), 64 lower-case hex. Its error never carries the path:
//     the caller knows which file it asked for, and a record must not hold a folder of the user's machine.
//   - summarise_stage(): one StageLatency from the runner's per-tick wall-clock samples (nanoseconds): min, mean, nearest-rank p99, max.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>

#include "core/error.hpp"
#include "fc_host/run_record.hpp"

namespace spade::fc_link {

// The process's peak working set (Windows) or high-water resident set (Linux) so far, in bytes. unavailable when the platform cannot say; never 0.
[[nodiscard]] Result<uint64_t> measure_memory_peak_bytes();

// The Linux reading: the `VmHWM:` line of /proc/self/status text, in kB, as bytes. An absent line, a malformed number or 0 is an error (never 0).
// Exposed so the parse is tested on every platform.
[[nodiscard]] Result<uint64_t> parse_peak_resident_kb(std::string_view proc_status_text);

// The SHA-256 of the file's bytes. io_error (without the path) when it cannot be read.
[[nodiscard]] Result<std::string> hash_file(const std::filesystem::path& file);

// min, mean, p99 (nearest rank: the sample at index ceil(0.99 n) - 1 of the sorted samples) and max of `samples_ns`, named `name` ([a-z0-9_]+: it
// becomes a key in meters.yaml). Empty samples, a bad name or a non-finite or negative sample is invalid_argument, naming what.
[[nodiscard]] Result<fc_host::StageLatency> summarise_stage(std::string_view name, std::span<const double> samples_ns);

}  // namespace spade::fc_link
