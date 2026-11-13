// One machine-readable line for Test/Docs' perf ledger: `PERF <metric> <value> <unit>` on stdout.
// A test prints one per timed step (the ledger takes the median over runs), behind the same opt-in
// environment variable as the test's own "measure:" lines. Metric names are `physics.<name>`;
// a phase of one step is `physics.diag_<phase>` (a diagnostic, not a ledger row).
#pragma once

#include <cstdio>

namespace spade::test {

inline void perf_line(const char* metric, double value, const char* unit) {
    std::printf("PERF %s %.3f %s\n", metric, value, unit);
    std::fflush(stdout);
}

}  // namespace spade::test
