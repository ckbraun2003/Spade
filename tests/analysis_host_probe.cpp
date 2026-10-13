// A stand-in host for test_analysis_launch.cpp's TheRunOutlivesTheHostThatLaunchedIt:
// it launches a paced dry run of the folder it is given and exits at once,
// as a host that closes would. The test then finds the runner still going.
//
//   spade_analysis_host_probe <runner> <run folder>
//
// Exits 0 once the run is launched, 1 on a refusal (printed), 2 on usage.

#include <cstdio>
#include <filesystem>

#include "analysis/launch.hpp"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: spade_analysis_host_probe <runner> <run folder>\n");
        return 2;
    }
    spade::analysis::LaunchOptions options;
    options.kind = spade::analysis::RunKind::dry_run;
    options.pace_ms = 20;
    const spade::Result<spade::analysis::RunRecord> r =
        spade::analysis::launch_run(std::filesystem::path(argv[1]), std::filesystem::path(argv[2]), options);
    if (!r) {
        std::fprintf(stderr, "spade_analysis_host_probe: %s\n", r.error().context.c_str());
        return 1;
    }
    return 0;
}
