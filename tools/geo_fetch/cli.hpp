#pragma once

// ---------------------------------------------------------------------------
// geo_fetch/cli.hpp -- spade_geo_fetch's command line, its progress lines and its exit codes (L85, SP-5; the design note,
// section 2). run_cli() is the whole program minus main(): the tests call it with a fake network and string streams.
// ---------------------------------------------------------------------------

#include <iosfwd>
#include <string>
#include <vector>

#include "fetch.hpp"
#include "world/map_cache.hpp"

namespace spade::geofetch {

struct CliEnv {
    FetchEnv fetch;                // the network, the clock, the area's geodesy (progress is set by run_cli)
    EnvLookup env;                 // the process environment, for the default cache root
    MapPlatform platform = MapPlatform::windows;
};

inline constexpr int kExitUsage = 2;

// Runs the tool on `args` (argv without the program name). Plain-text progress goes to `out`, or one JSON object a line with
// --json; the last line of `out` always names the result. A usage problem goes to `err`. Returns the exit code.
[[nodiscard]] int run_cli(const std::vector<std::string>& args, CliEnv& env, std::ostream& out, std::ostream& err);

}  // namespace spade::geofetch
