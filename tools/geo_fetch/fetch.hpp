#pragma once

// ---------------------------------------------------------------------------
// geo_fetch/fetch.hpp -- one map fetch, end to end (L85, SP-5; the design note, sections 2 to 5): check the cache, ask NGS for
// the geoid height, 3DEP for the elevation, NAIP for the image, convert NAVD88 to ellipsoid heights, store the entry, trim the
// cache. Everything outside it is injected (the network, the clock, the area's geodesy), so the tests run it on fixtures.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>

#include "geo_services.hpp"
#include "http_client.hpp"

namespace spade::geofetch {

struct FetchRequest {
    double lat_deg = 0.0;   // the anchor, WGS84 degrees
    double lon_deg = 0.0;
    double side_m = 1000.0;
    double dem_cell_m = 1.0;
    double image_cell_m = 0.6;
    std::filesystem::path cache_root;
    std::uint64_t cache_limit_bytes = 512ull * 1024ull * 1024ull;
    bool check_only = false;  // cache only, never the network
    bool refresh = false;     // fetch again even on a hit
    int timeout_s = 30;
};

enum class FetchResultKind {
    hit,            // 0   the entry was in the cache; no request was made
    fetched,        // 0
    miss,           // 3   check_only: not in the cache
    offline,        // 10  no network (or no HTTP layer on this platform)
    service_error,  // 11  a service answered and the answer is no use; `service` names it
    refused,        // 12  the request itself
    cache_full,     // 13  one entry alone is bigger than the limit
};

struct FetchResult {
    FetchResultKind kind = FetchResultKind::refused;
    std::string message;   // for anything but hit and fetched: a plain sentence
    std::string service;   // "3dep", "naip" or "ngs" for a service_error
    std::string key;
    std::filesystem::path dir;
    std::uint64_t bytes = 0;  // the entry's size on disk
    double geoid_height_m = 0.0;
    double anchor_ellipsoid_height_m = 0.0;
};

// The process exit code of a result (the table in fetch.hpp's enum).
[[nodiscard]] int exit_code_of(FetchResultKind kind);
[[nodiscard]] const char* name_of(FetchResultKind kind);  // "hit", "fetched", "miss", "offline", ...

struct ProgressEvent {
    enum class State { start, bytes, done };
    std::string stage;  // "geoid", "elevation", "image", "store"
    State state = State::start;
    std::uint64_t received = 0;  // State::bytes
    std::uint64_t total = 0;     // State::bytes; 0 when unknown
};

struct FetchEnv {
    HttpClient& http;
    std::function<Bbox(double lat_deg, double lon_deg, double side_m)> area;  // the box both layers cover (SP-1's geodesy)
    std::function<std::string()> utc_now;                                    // "2026-11-17T01:23:45Z"
    std::function<void(int milliseconds)> sleep;                             // between a retry; may be a no-op
    std::function<void(const ProgressEvent&)> progress;                      // may be empty
    std::string tmp_token = "0";                                             // unique per process (the store's temp name)
};

[[nodiscard]] FetchResult fetch_map(const FetchRequest& request, const FetchEnv& env);

}  // namespace spade::geofetch
