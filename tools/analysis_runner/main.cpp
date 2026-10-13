// spade_analysis_runner: the headless process a queued analysis run runs in
// (W2c; INT-7; the CFD spec §5). A host starts it with
// spade::analysis::launch_run(); it runs the case in its folder to the
// case's duration through spade::analysis::run_case(), writing the folder's
// progress, history and log, and stops between steps when the host makes the
// folder's cancel file.
//
// Until the airflow solver lands (W1), it runs only a dry run, and only when
// asked with --dry-run: a stand-in step with no airflow and zero loads,
// marked "dry-run" in every file it writes. Without --dry-run it refuses, in
// the folder, by name.
//
// It lives outside engine/, so it may read the steady clock and sleep (M1B
// keeps wall-clock symbols out of engine/), and it uses only installed
// spade:: headers and targets, as a consumer would.
//
//   spade_analysis_runner <run folder> --dry-run [--pace-ms N]
//
// Exits 0 when the run is done or cancelled, 1 when it is refused or
// faults, 2 on a usage error.

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "analysis/case.hpp"
#include "analysis/launch.hpp"
#include "analysis/run.hpp"
#include "analysis/runner.hpp"
#include "core/path_text.hpp"

namespace {

namespace an = spade::analysis;
namespace fs = std::filesystem;

constexpr const char* kUsage =
    "usage: spade_analysis_runner <run folder> --dry-run [--pace-ms N]\n"
    "  Runs the analysis case in <run folder> (made by spade::analysis::create_run_folder) to its duration.\n"
    "  --dry-run     a stand-in step: no airflow, zero loads; marked dry-run in every file, never a CFD result\n"
    "  --pace-ms N   a dry run only: pause N ms (at most 10000) after each step\n"
    "  Without --dry-run the run is refused: no airflow solver yet (W1).\n";

int usage(const std::string& why) {
    std::fprintf(stderr, "spade_analysis_runner: %s\n%s", why.c_str(), kUsage);
    return 2;
}

void log_line(const fs::path& folder, const std::string& line) {
    std::ofstream out(folder / "log.txt", std::ios::binary | std::ios::app);
    out << line << '\n';
}

// Refuses the run where the host reads it: its progress says refused, with
// why, and so does its log.
int refuse(const fs::path& folder, const std::string& why) {
    an::RunProgress p = an::read_progress(folder).value_or(an::RunProgress{});
    p.state = an::RunState::refused;
    p.message = why;
    if (const spade::Result<void> r = an::write_progress(folder, p); !r) {
        std::fprintf(stderr, "spade_analysis_runner: %s\n", r.error().context.c_str());
    }
    log_line(folder, "refused: " + why);
    std::fprintf(stderr, "spade_analysis_runner: refused: %s\n", why.c_str());
    return 1;
}

// The dry run's stand-in step: the case's own step, or a hundredth of its
// duration when the case leaves the step to the solver; zero loads per rotor.
int dry_run(const fs::path& folder, uint32_t pace_ms) {
    const spade::Result<an::AnalysisCase> c = an::load_case(folder / "case.yaml");
    if (!c) return refuse(folder, c.error().context);
    const double duration = static_cast<double>(c->time.duration);
    const double dt = c->time.step ? static_cast<double>(*c->time.step) : duration / 100.0;
    if (!(dt > 0.0)) return refuse(folder, "the dry run needs a step above 0 s");

    an::RunnerSetup setup;
    setup.kind = an::RunKind::dry_run;
    for (std::size_t i = 0; i < c->rotors.size(); ++i) {
        setup.columns.push_back("rotor" + std::to_string(i) + "_thrust_N");
        setup.columns.push_back("rotor" + std::to_string(i) + "_torque_N_m");
    }
    const std::size_t width = setup.columns.size();
    const an::StepFn step = [&](uint64_t k) -> spade::Result<an::StepSample> {
        if (pace_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(pace_ms));
        return an::StepSample{std::min(dt * static_cast<double>(k + 1), duration), std::vector<double>(width, 0.0)};
    };
    const an::Clock clock = [] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    };
    const spade::Result<an::RunProgress> end = an::run_case(folder, setup, step, clock);
    if (!end) return refuse(folder, end.error().context);
    return end->state == an::RunState::faulted ? 1 : 0;
}

int run(const std::vector<fs::path>& args) {
    if (args.empty()) return usage("no run folder");
    const fs::path& folder = args[0];
    bool dry = false;
    uint32_t pace_ms = 0;
    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string arg = spade::path_text(args[i]);
        if (arg == "--dry-run") {
            dry = true;
        } else if (arg == "--pace-ms" && i + 1 < args.size()) {
            const std::string n = spade::path_text(args[++i]);
            const std::from_chars_result r = std::from_chars(n.data(), n.data() + n.size(), pace_ms);
            if (r.ec != std::errc{} || r.ptr != n.data() + n.size() || pace_ms > an::kMaxPaceMs) {
                return usage("--pace-ms takes a whole number of ms up to 10000, not '" + n + "'");
            }
        } else {
            return usage("unknown argument '" + arg + "'");
        }
    }
    if (pace_ms > 0 && !dry) return usage("--pace-ms paces a dry run only");
    std::error_code ec;
    if (!fs::is_regular_file(folder / "case.yaml", ec)) {
        std::fprintf(stderr, "spade_analysis_runner: '%s' is not a run folder: it has no case.yaml\n",
                     spade::path_text(folder).c_str());
        return 1;
    }
    if (!dry) {
        return refuse(folder, "no airflow solver yet (W1): a real run is refused; --dry-run runs the stand-in step");
    }
    return dry_run(folder, pace_ms);
}

}  // namespace

// Wide arguments on Windows, so a folder's name arrives whole whatever the
// code page.
#ifdef _WIN32
int wmain(int argc, wchar_t** argv) { return run(std::vector<fs::path>(argv + 1, argv + argc)); }
#else
int main(int argc, char** argv) { return run(std::vector<fs::path>(argv + 1, argv + argc)); }
#endif
