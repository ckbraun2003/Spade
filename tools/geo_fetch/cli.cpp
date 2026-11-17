#include "cli.hpp"

#include <charconv>
#include <chrono>
#include <cmath>
#include <format>
#include <ostream>
#include <system_error>

namespace spade::geofetch {

namespace {

constexpr const char* kUsage =
    "spade_geo_fetch: fetch a map area into the local cache (USGS 3DEP elevation, USGS NAIP image, NOAA NGS geoid height)\n"
    "\n"
    "usage: spade_geo_fetch --lat <deg> --lon <deg> [options]\n"
    "\n"
    "  --lat <deg>, --lon <deg>   the anchor, WGS84 degrees (north and east positive; Bonneville is 40.76 -113.90)\n"
    "  --side <m>                 side of the square area, centred on the anchor: 10 to 1000 (default 1000)\n"
    "  --cache <dir>              the cache directory (default: $SPADE_MAP_CACHE, else the user's cache folder);\n"
    "                             refused inside a git work tree\n"
    "  --cache-limit-mb <n>       size limit for the whole cache (default 512)\n"
    "  --dem-cell <m>             elevation grid cell (default 1; the grid is at most 1024 samples a side)\n"
    "  --image-cell <m>           image pixel (default 0.6; at most 4096 pixels a side)\n"
    "  --check                    look in the cache only, never the network: hit (0) or miss (3)\n"
    "  --refresh                  fetch again even when the area is cached, and replace it\n"
    "  --timeout-s <n>            seconds to wait for each answer after connecting (default 30)\n"
    "  --json                     progress and the result as one JSON object a line\n"
    "  --help                     this text\n"
    "\n"
    "The last line of standard output names the result. Exit codes: 0 hit or fetched, 2 usage, 3 miss (--check),\n"
    "10 offline, 11 a service answered and the answer is no use, 12 the request was refused, 13 the cache is full.\n";

struct Options {
    FetchRequest request;
    bool json = false;
    bool help = false;
    bool have_lat = false;
    bool have_lon = false;
    bool have_cache = false;
};

[[nodiscard]] bool parse_number(const std::string& s, double& out) {
    if (s.empty()) return false;
    const char* first = s.data();
    const char* last = s.data() + s.size();
    const std::from_chars_result r = std::from_chars(first, last, out);
    return r.ec == std::errc() && r.ptr == last && std::isfinite(out);
}

// Returns an empty string on success, else the usage problem.
[[nodiscard]] std::string parse_args(const std::vector<std::string>& args, Options& o) {
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        const auto value = [&](double& into) -> std::string {
            if (i + 1 >= args.size()) return a + " needs a value";
            ++i;
            if (!parse_number(args[i], into)) return a + " needs a number, not '" + args[i] + "'";
            return {};
        };
        std::string problem;
        double mb = 0.0, seconds = 0.0;
        if (a == "--lat") {
            problem = value(o.request.lat_deg);
            o.have_lat = true;
        } else if (a == "--lon") {
            problem = value(o.request.lon_deg);
            o.have_lon = true;
        } else if (a == "--side") {
            problem = value(o.request.side_m);
        } else if (a == "--dem-cell") {
            problem = value(o.request.dem_cell_m);
        } else if (a == "--image-cell") {
            problem = value(o.request.image_cell_m);
        } else if (a == "--cache-limit-mb") {
            problem = value(mb);
            if (problem.empty()) {
                if (mb < 0.0) return "--cache-limit-mb must not be negative";
                o.request.cache_limit_bytes = static_cast<std::uint64_t>(mb * 1024.0 * 1024.0);
            }
        } else if (a == "--timeout-s") {
            problem = value(seconds);
            if (problem.empty()) {
                if (!(seconds >= 1.0)) return "--timeout-s must be at least 1";
                o.request.timeout_s = static_cast<int>(seconds);
            }
        } else if (a == "--cache") {
            if (i + 1 >= args.size()) return "--cache needs a directory";
            o.request.cache_root = args[++i];
            o.have_cache = true;
        } else if (a == "--check") {
            o.request.check_only = true;
        } else if (a == "--refresh") {
            o.request.refresh = true;
        } else if (a == "--json") {
            o.json = true;
        } else if (a == "--help" || a == "-h") {
            o.help = true;
        } else {
            return "unknown option " + a;
        }
        if (!problem.empty()) return problem;
    }
    if (o.help) return {};
    if (!o.have_lat) return "--lat is required";
    if (!o.have_lon) return "--lon is required";
    return {};
}

[[nodiscard]] std::string json_string(const std::string& s) {
    std::string out = "\"";
    for (const char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    out += std::format("\\u{:04x}", static_cast<unsigned>(static_cast<unsigned char>(c)));
                } else {
                    out.push_back(c);
                }
        }
    }
    out.push_back('"');
    return out;
}

[[nodiscard]] const char* describe(const std::string& stage) {
    if (stage == "geoid") return "asking NOAA NGS for the geoid height";
    if (stage == "elevation") return "asking USGS 3DEP for the elevation grid";
    if (stage == "image") return "asking USGS NAIP for the image";
    return "writing the cache entry";
}

}  // namespace

int run_cli(const std::vector<std::string>& args, CliEnv& env, std::ostream& out, std::ostream& err) {
    Options o;
    if (const std::string problem = parse_args(args, o); !problem.empty()) {
        err << "spade_geo_fetch: " << problem << "\n(spade_geo_fetch --help lists the options)\n";
        return kExitUsage;
    }
    if (o.help) {
        out << kUsage;
        return 0;
    }
    if (!o.have_cache) o.request.cache_root = default_map_cache_root(env.env, env.platform);

    // Progress, a line per event: plain text, or JSON. The bytes of a download are throttled to one a 100 ms and the last one
    // of a stage always goes out.
    using Clock = std::chrono::steady_clock;
    Clock::time_point last_bytes{};
    bool any_bytes = false;
    env.fetch.progress = [&](const ProgressEvent& e) {
        if (e.state == ProgressEvent::State::bytes) {
            const auto now = Clock::now();
            const bool final = e.total != 0 && e.received >= e.total;
            if (!final && any_bytes && now - last_bytes < std::chrono::milliseconds(100)) return;
            last_bytes = now;
            any_bytes = true;
        }
        if (o.json) {
            switch (e.state) {
                case ProgressEvent::State::start:
                    out << "{\"event\":\"stage\",\"stage\":" << json_string(e.stage) << ",\"state\":\"start\"}\n";
                    break;
                case ProgressEvent::State::bytes:
                    out << "{\"event\":\"progress\",\"stage\":" << json_string(e.stage) << ",\"bytes\":" << e.received
                        << ",\"total\":" << e.total << "}\n";
                    break;
                case ProgressEvent::State::done:
                    out << "{\"event\":\"stage\",\"stage\":" << json_string(e.stage) << ",\"state\":\"done\"}\n";
                    break;
            }
        } else {
            switch (e.state) {
                case ProgressEvent::State::start: out << e.stage << ": " << describe(e.stage) << "\n"; break;
                case ProgressEvent::State::bytes:
                    out << e.stage << ": " << e.received;
                    if (e.total != 0) out << " of " << e.total;
                    out << " bytes\n";
                    break;
                case ProgressEvent::State::done: out << e.stage << ": done\n"; break;
            }
        }
    };

    const FetchResult r = fetch_map(o.request, env.fetch);
    const bool ok = r.kind == FetchResultKind::hit || r.kind == FetchResultKind::fetched;
    if (o.json) {
        out << "{\"event\":\"result\",\"result\":\"" << name_of(r.kind) << "\"";
        if (!r.key.empty()) out << ",\"key\":" << json_string(r.key);
        if (ok) {
            out << ",\"dir\":" << json_string(r.dir.string()) << ",\"bytes\":" << r.bytes
                << ",\"geoid_height_m\":" << std::format("{}", r.geoid_height_m)
                << ",\"anchor_ellipsoid_height_m\":" << std::format("{}", r.anchor_ellipsoid_height_m);
        } else {
            if (!r.service.empty()) out << ",\"service\":" << json_string(r.service);
            out << ",\"message\":" << json_string(r.message);
        }
        out << "}\n";
    } else if (ok) {
        out << name_of(r.kind) << " " << r.key << " " << r.dir.string() << "\n";
    } else if (r.kind == FetchResultKind::miss) {
        out << "miss " << r.key << "\n";
    } else {
        out << name_of(r.kind) << ": " << r.message << "\n";
    }
    return exit_code_of(r.kind);
}

}  // namespace spade::geofetch
