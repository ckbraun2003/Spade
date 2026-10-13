// The validation report (the CFD spec §6; TD-9): the one reader of the
// validation record format (docs/design/test-docs/01-verification.md,
// "Validation records"). The sandbox's Validation readout and Rendering's
// validation reel read records through it, never on their own.
//
// A records file holds prose and fenced blocks; only ```text blocks hold
// records. A record's opening line, "=== validation record: <identity> ===",
// is its identity, and the newest record of a configuration is the LAST
// block with that line, not the file's last block. Then, one per line:
// commit, date, path, config, and for each measure in order: measure,
// reference, tolerance, verdict. Everything is kept as written (UTF-8);
// nothing is retyped.
//
// Display-free. Owned by Interface (INT-6).

#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"

namespace spade::analysis {

// within and MISS judge a measure against its tolerance; a report-only
// measure has none, and is never a pass.
enum class Verdict : uint32_t { within = 0, miss = 1, report_only = 2 };

// "within", "MISS", "report only", as the records spell them.
[[nodiscard]] std::string_view verdict_name(Verdict verdict) noexcept;

// A tolerance and where it was fixed: "<= 0.02 (plan A Task 11, spec §11.2,
// fixed before the first run)" is bound "<= 0.02" and the origin inside the
// parentheses.
struct Tolerance {
    std::string bound;   // "<= 0.02", "within 10%"
    std::string origin;  // where it was fixed, before the first run
};

struct ValidationMeasure {
    std::string quantity;   // the measure line before its LAST " = ", notes and all, verbatim
    std::string value;      // after it, verbatim: "0.0065", "7.4124 m/s"
    double number = 0.0;    // the value's first token
    std::string unit;       // the rest of the value; empty when it has no unit
    std::string reference;  // verbatim, with its table or figure and data file
    std::optional<Tolerance> tolerance;  // none for a report-only measure
    Verdict verdict = Verdict::report_only;
};

struct ValidationRecord {
    std::string source;    // the records file's name: "c1-cavity.md"
    std::string identity;  // the opening line's case and configuration: "C1 lid-driven cavity, Re 400, 128x128"
    std::string commit;    // verbatim, with any note: "36153e0 (physics/airflow-core, ...)"
    std::string sha;       // the commit's first token: "36153e0"
    std::string date;      // "2026-10-12"
    std::string path;      // the path that ran it: "CPU reference, MSVC 1943, release"
    std::string config;    // the configuration's line, verbatim
    std::vector<ValidationMeasure> measures;  // in the record's order; at least one
};

// The newest record of each configuration in one records file's text, in the
// order the configurations first appear. `source` names the file in the
// records and in a refusal. CRLF line ends and a byte-order mark are read as
// plain. Refused by name, with the file and line (L6), and never skipped: a
// block that doesn't open with its opening line, a missing or out-of-order
// key (a record not yet pasted has no commit or date), a record with no
// measure, a measure without " = " or whose value isn't a number, a verdict
// that isn't one of the three, a tolerance that doesn't say where it was
// fixed, a report-only verdict with a tolerance or a pass without one, and a
// fence left open.
[[nodiscard]] Result<std::vector<ValidationRecord>> validation_records(std::string_view text, std::string_view source);

// Every *.md file in `records_dir` (for the airflow cases,
// tests/data/airflow/records/), in file name order, each through
// validation_records(). A missing folder is an io_error.
[[nodiscard]] Result<std::vector<ValidationRecord>> validation_report(const std::filesystem::path& records_dir);

}  // namespace spade::analysis
