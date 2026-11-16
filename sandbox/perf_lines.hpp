// L74 (the lead, 2026-11-13): the editor's two timing notes, measurement only. Off unless SPADE_EDITOR_PERF is set in
// the environment (any text but "" and "0"); then the window prints, on stdout (so also in the launcher's log):
//
//   PERF interface.editor_first_frame <ms> ms       once: the process's creation to its first presented frame
//   PERF interface.run_air_frame <ms> ms            when a Run air stops: the median wall time of one air frame
//   PERF interface.diag_run_air_step <ms> ms        ... of one step (the solve, with its device wait)
//   PERF interface.diag_run_air_publish <ms> ms     ... of the frame's publish: the frame's last step and publish,
//                                                   less one step (the flow read back to the host, its checks)
//   PERF interface.diag_run_air_assemble <ms> ms    ... of the host's assembly of the frame it hands the window
//
// The line is the form Test/Docs' harness reads (tests/bench/PERF.md): `PERF <name> <value> <unit>`, the value to four
// places. The air figures are wall time around the calls the worker already makes, medians over the run, so a first
// frame's shader build does not move them. No drawn frame, no scene byte and no journal line changes with the switch.

#pragma once

#include <algorithm>
#include <cstdlib>
#include <format>
#include <string>
#include <string_view>
#include <vector>

namespace spade::sandbox::perf {

inline constexpr const char* kEnvName = "SPADE_EDITOR_PERF";

[[nodiscard]] inline std::string line(std::string_view name, double value, std::string_view unit) {
    return std::format("PERF {} {:.4f} {}", name, value, unit);  // std::format with no L: the C locale, never a comma
}

// The middle, or the mean of the two middles; 0 for none.
[[nodiscard]] inline double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return n % 2u == 1u ? v[n / 2u] : 0.5 * (v[n / 2u - 1u] + v[n / 2u]);
}

[[nodiscard]] inline bool enabled_from(const char* value) noexcept {
    return value != nullptr && value[0] != '\0' && !(value[0] == '0' && value[1] == '\0');
}

[[nodiscard]] inline bool enabled() noexcept {
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)  // std::getenv: read once per Run air start, never written
#endif
    return enabled_from(std::getenv(kEnvName));
#ifdef _MSC_VER
#pragma warning(pop)
#endif
}

// What one Run air run keeps while it measures: the worker's alone until it stops, then the owner's.
class AirPerf {
  public:
    // One step() of the worker's inner loop, in ms.
    void step(double ms) { add(steps_, ms); }

    // One frame: advance_frame() (its last step and the publish), the host's assembly after it, the whole frame.
    void frame(double advance_ms, double assemble_ms, double total_ms) {
        add(advance_, advance_ms);
        add(assemble_, assemble_ms);
        add(total_, total_ms);
    }

    // The four lines, in the order the harness reads them; none when no frame finished (not zeros).
    [[nodiscard]] std::vector<std::string> lines() const {
        if (total_.empty()) return {};
        const double step_ms = median(steps_);
        return {line("interface.run_air_frame", median(total_), "ms"),
                line("interface.diag_run_air_step", step_ms, "ms"),
                line("interface.diag_run_air_publish", std::max(0.0, median(advance_) - step_ms), "ms"),
                line("interface.diag_run_air_assemble", median(assemble_), "ms")};
    }

  private:
    // A long run keeps its first samples only: a bounded vector, not a leak.
    static constexpr std::size_t kMaxSamples = 200'000;
    static void add(std::vector<double>& v, double ms) {
        if (v.size() < kMaxSamples) v.push_back(ms);
    }
    std::vector<double> steps_, advance_, assemble_, total_;
};

}  // namespace spade::sandbox::perf
