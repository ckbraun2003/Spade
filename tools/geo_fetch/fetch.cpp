#include "fetch.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
#include <iterator>
#include <limits>
#include <span>
#include <system_error>
#include <vector>

#include "area.hpp"
#include "cache_store.hpp"
#include "core/geodesy.hpp"
#include "raster_formats.hpp"
#include "world/map_cache.hpp"
#include "world/mesh_import.hpp"
#include "world/sha256.hpp"

namespace spade::geofetch {

namespace {

namespace fs = std::filesystem;

constexpr std::uint32_t kMaxGridSide = 1024;   // the elevation grid, samples a side
constexpr std::uint32_t kMaxImageSide = 4096;  // the image, pixels a side
constexpr int kRetryAfterMs = 2000;

[[nodiscard]] FetchResult failure(FetchResultKind kind, std::string message, std::string service = {}) {
    FetchResult r;
    r.kind = kind;
    r.message = std::move(message);
    r.service = std::move(service);
    return r;
}

void emit(const FetchEnv& env, const std::string& stage, ProgressEvent::State state, std::uint64_t received = 0,
          std::uint64_t total = 0) {
    if (env.progress) env.progress(ProgressEvent{stage, state, received, total});
}

[[nodiscard]] const char* label_of(const std::string& service) {
    if (service == "ngs") return "NOAA NGS";
    if (service == "3dep") return "USGS 3DEP";
    return "USGS NAIP";
}

// What a service said, for the error of an answer the tool refused: its first 120 bytes with anything but printable ASCII shown as a dot
// (a body can hold any byte, and the message goes to a terminal or a log). Empty for an empty body.
[[nodiscard]] std::string answer_begins(const std::string& body) {
    constexpr std::size_t kQuoteBytes = 120;
    if (body.empty()) return {};
    std::string q = " (the answer begins: \"";
    for (std::size_t i = 0; i < body.size() && i < kQuoteBytes; ++i) {
        const auto c = static_cast<unsigned char>(body[i]);
        q.push_back(c >= 0x20 && c < 0x7f ? static_cast<char>(c) : '.');
    }
    if (body.size() > kQuoteBytes) q += "...";
    return q + "\")";
}

// The raw bytes of an answer the tool refused, kept in <cache>/failed/<service>.bin (the next refusal of that service replaces it), so a
// person can look at what a service sent without asking again. Untrusted data: nothing reads it back, and it is outside the cache's
// entries and their size limit. Returns " (the raw answer is in <path>)", or "" when there is nothing to keep or it cannot be written.
[[nodiscard]] std::string keep_failed(const fs::path& root, const std::string& service, const std::string& body) {
    constexpr std::size_t kMaxKeptBytes = 16u * 1024u * 1024u;
    if (body.empty() || root.empty()) return {};
    std::error_code ec;
    const fs::path dir = root / "failed";
    fs::create_directories(dir, ec);
    if (ec) return {};
    const fs::path file = dir / (service + ".bin");
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    out.write(body.data(), static_cast<std::streamsize>(std::min(body.size(), kMaxKeptBytes)));
    out.close();
    if (!out) return {};
    return " (the raw answer is in " + file.string() + ")";
}

// The number of cells that cover `side` with cells of `cell` metres, rounded up; 0 when it is not a sane number.
[[nodiscard]] std::uint32_t cells_over(double side, double cell) {
    if (!(cell > 0.0) || !std::isfinite(cell) || !(side > 0.0)) return 0;
    const double n = side / cell;
    if (!(n < 1.0e6)) return 0;
    auto whole = static_cast<std::uint32_t>(n);
    if (static_cast<double>(whole) < n) ++whole;
    return whole;
}

[[nodiscard]] std::uint64_t dir_bytes(const fs::path& dir) {
    std::uint64_t total = 0;
    std::error_code ec;
    for (fs::recursive_directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code e2;
        if (it->is_regular_file(e2)) total += it->file_size(e2);
    }
    return total;
}

[[nodiscard]] std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// One download, with the retry rule: a timeout or a 5xx is asked once more after two seconds; offline is not retried.
struct Download {
    bool ok = false;
    std::string body;
    std::string retrieved_utc;
    FetchResult failed;
};

[[nodiscard]] Download download(const FetchEnv& env, const fs::path& cache_root, const std::string& stage, const std::string& service,
                                const std::string& url, int timeout_s) {
    Download d;
    emit(env, stage, ProgressEvent::State::start);
    HttpResponse resp;
    for (int attempt = 0; attempt < 2; ++attempt) {
        resp = env.http.get(url, timeout_s, [&](std::uint64_t received, std::uint64_t total) {
            emit(env, stage, ProgressEvent::State::bytes, received, total);
        });
        if (resp.error == HttpError::none && resp.status == 200) {
            d.ok = true;
            d.body = std::move(resp.body);
            d.retrieved_utc = env.utc_now ? env.utc_now() : std::string();
            emit(env, stage, ProgressEvent::State::done);
            return d;
        }
        const bool retryable = resp.error == HttpError::timeout || (resp.error == HttpError::none && resp.status >= 500);
        if (attempt == 0 && retryable) {
            if (env.sleep) env.sleep(kRetryAfterMs);
            continue;
        }
        break;
    }
    const std::string who = label_of(service);
    switch (resp.error) {
        case HttpError::offline:
            d.failed = failure(FetchResultKind::offline, "cannot reach " + who + ": " + resp.detail);
            break;
        case HttpError::no_http_layer: d.failed = failure(FetchResultKind::offline, resp.detail); break;
        case HttpError::timeout:
            d.failed = failure(FetchResultKind::service_error, service + ": no answer in time (" + resp.detail + ")", service);
            break;
        case HttpError::tls:
            d.failed = failure(FetchResultKind::service_error, service + ": the secure connection was refused (" + resp.detail + ")", service);
            break;
        case HttpError::too_large:
            d.failed = failure(FetchResultKind::service_error, service + ": the answer is larger than the cap", service);
            break;
        case HttpError::none:
            d.failed = failure(FetchResultKind::service_error, service + ": HTTP " + std::to_string(resp.status) + answer_begins(resp.body) +
                                                              keep_failed(cache_root, service, resp.body),
                                   service);
            break;
    }
    return d;
}

void put_le32(std::string& out, float v) {
    const auto bits = std::bit_cast<std::uint32_t>(v);
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<char>((bits >> (8 * i)) & 0xff));
}

[[nodiscard]] FetchResult result_from_georef(const MapGeoref& g, const fs::path& dir, FetchResultKind kind) {
    FetchResult r;
    r.kind = kind;
    r.key = g.key;
    r.dir = dir;
    r.bytes = dir_bytes(dir);
    r.geoid_height_m = g.geoid_height_m;
    r.anchor_ellipsoid_height_m = g.anchor_ground_ellipsoid_height_m;
    r.geoid_model = g.geoid_model;
    r.elevation_width = g.elevation.width;
    r.elevation_height = g.elevation.height;
    r.image_width = g.image.width;
    r.image_height = g.image.height;
    r.west_deg = g.west_deg;
    r.south_deg = g.south_deg;
    r.east_deg = g.east_deg;
    r.north_deg = g.north_deg;
    return r;
}

constexpr const char* kDatumNote =
    "Heights are NAVD88 plus the GEOID18 geoid height, whose ellipsoid is NAD83(2011), not WGS84: they can differ from WGS84 "
    "heights by 1 to 2 m, and the services' EPSG:4326 coordinates are NAD83 underneath, so the image can sit 1 to 2 m off WGS84. "
    "Recorded, not corrected.";

}  // namespace

int exit_code_of(FetchResultKind kind) {
    switch (kind) {
        case FetchResultKind::hit:
        case FetchResultKind::fetched: return 0;
        case FetchResultKind::miss: return 3;
        case FetchResultKind::offline: return 10;
        case FetchResultKind::service_error: return 11;
        case FetchResultKind::refused: return 12;
        case FetchResultKind::cache_full: return 13;
    }
    return 12;
}

const char* name_of(FetchResultKind kind) {
    switch (kind) {
        case FetchResultKind::hit: return "hit";
        case FetchResultKind::fetched: return "fetched";
        case FetchResultKind::miss: return "miss";
        case FetchResultKind::offline: return "offline";
        case FetchResultKind::service_error: return "service_error";
        case FetchResultKind::refused: return "refused";
        case FetchResultKind::cache_full: return "cache_full";
    }
    return "refused";
}

FetchResult fetch_map(const FetchRequest& q, const FetchEnv& env) {
    // ---- the request itself ------------------------------------------------------------------------------------------
    // The anchor the engine would accept (core/geodesy.hpp: latitude within 89, east is undefined at a pole): a map the engine could
    // not be anchored to is not fetched.
    if (const Result<void> anchor = geo::validate_anchor({q.lat_deg, q.lon_deg, 0.0}); !anchor) {
        return failure(FetchResultKind::refused, anchor.error().context);
    }
    if (!(q.side_m >= kMapMinSideM && q.side_m <= kMapMaxSideM)) {
        return failure(FetchResultKind::refused, std::format("the side must be between {} and {} m (1 km at most)", kMapMinSideM, kMapMaxSideM));
    }
    const std::uint32_t dem_n = cells_over(q.side_m, q.dem_cell_m);
    const std::uint32_t image_n = cells_over(q.side_m, q.image_cell_m);
    if (dem_n < 2 || dem_n > kMaxGridSide) {
        return failure(FetchResultKind::refused, std::format("the elevation grid would be {} samples a side; 2 to {} are allowed", dem_n, kMaxGridSide));
    }
    if (image_n < 2 || image_n > kMaxImageSide) {
        return failure(FetchResultKind::refused, std::format("the image would be {} pixels a side; 2 to {} are allowed", image_n, kMaxImageSide));
    }
    if (q.cache_root.empty()) {
        return failure(FetchResultKind::refused, "no cache directory: use --cache, or set SPADE_MAP_CACHE");
    }
    if (inside_git_work_tree(q.cache_root)) {
        return failure(FetchResultKind::refused,
                       "the cache directory " + q.cache_root.string() + " is inside a git work tree; a map is never committed");
    }

    // ---- the cache -----------------------------------------------------------------------------------------------------
    const std::string key = map_key(q.lat_deg, q.lon_deg, q.side_m);
    const fs::path dir = map_entry_dir(q.cache_root, key);
    if (entry_complete(dir) && !q.refresh) {
        const Result<MapGeoref> g = parse_georef(slurp(dir / "georef.yaml"));
        if (g) return result_from_georef(*g, dir, FetchResultKind::hit);
        // a damaged georef: the entry is refetched below, and replaced
    }
    if (q.check_only) {
        FetchResult r = failure(FetchResultKind::miss, "not in the cache");
        r.key = key;
        return r;
    }

    // ---- the network -----------------------------------------------------------------------------------------------------
    const Bbox box = env.area ? env.area(q.lat_deg, q.lon_deg, q.side_m) : area_bbox(q.lat_deg, q.lon_deg, q.side_m);
    if (!(box.west_deg < box.east_deg && box.south_deg < box.north_deg)) {
        return failure(FetchResultKind::refused, "the area crosses the 180th meridian, which is not supported");
    }

    // An answer the tool refuses is a service_error naming the service; its raw bytes go to <cache>/failed/<service>.bin, and the message
    // says so. `quote` adds the answer's first bytes to the message, for an answer that is not the format it should be.
    const auto refuse = [&](const std::string& service, std::string message, const Download& dl, bool quote) {
        if (quote) message += answer_begins(dl.body);
        return failure(FetchResultKind::service_error, message + keep_failed(q.cache_root, service, dl.body), service);
    };

    Download geoid_dl = download(env, q.cache_root, "geoid", "ngs", geoid_url(q.lat_deg, q.lon_deg), q.timeout_s);
    if (!geoid_dl.ok) return geoid_dl.failed;
    const Result<GeoidAnswer> geoid = parse_geoid_json(geoid_dl.body);
    if (!geoid) return refuse("ngs", geoid.error().context, geoid_dl, true);
    const double n_geoid = geoid->height_m;

    const std::string elevation_request = elevation_url(box, dem_n, dem_n);
    Download dem_dl = download(env, q.cache_root, "elevation", "3dep", elevation_request, q.timeout_s);
    if (!dem_dl.ok) return dem_dl.failed;
    const Result<TiffGrid> tiff = read_tiff_f32(dem_dl.body);
    if (!tiff) return refuse("3dep", tiff.error().context, dem_dl, true);
    if (tiff->width != dem_n || tiff->height != dem_n) {
        return refuse("3dep", std::format("3dep: the TIFF is {} x {}, {} x {} was asked", tiff->width, tiff->height, dem_n, dem_n), dem_dl, false);
    }
    std::size_t bad_samples = 0;
    for (const float v : tiff->samples) {
        if (!std::isfinite(v) || v < -1000.0f || v > 10000.0f) ++bad_samples;
    }
    if (bad_samples != 0) {
        return refuse("3dep", std::format("3dep: the answer holds {} no-data or impossible heights", bad_samples), dem_dl, false);
    }

    const std::string image_request = image_url(box, image_n, image_n);
    Download img_dl = download(env, q.cache_root, "image", "naip", image_request, q.timeout_s);
    if (!img_dl.ok) return img_dl.failed;
    const Result<PngHeader> png = read_png_header(img_dl.body);
    if (!png) return refuse("naip", png.error().context, img_dl, true);
    if (png->width != image_n || png->height != image_n) {
        return refuse("naip", std::format("naip: the PNG is {} x {}, {} x {} was asked", png->width, png->height, image_n, image_n), img_dl, false);
    }
    if (png->bit_depth != 8 || png->color_type != 2) {
        return refuse("naip", std::format("naip: the PNG is not 8-bit RGB (colour type {}, {} bits)", png->color_type, png->bit_depth), img_dl, false);
    }

    // The whole file, not only its header: a PNG that decode_png refuses is not stored, so the reader never meets one the tool kept.
    const Result<world::Image8> decoded = decode_png(std::as_bytes(std::span<const char>(img_dl.body)));
    if (!decoded) return refuse("naip", "naip: the PNG cannot be read: " + decoded.error().context, img_dl, false);

    // ---- NAVD88 to the ellipsoid: h = H + N, in double, stored as float32 -----------------------------------------------------
    std::string heights;
    heights.reserve(tiff->samples.size() * 4);
    for (const float v : tiff->samples) put_le32(heights, static_cast<float>(static_cast<double>(v) + n_geoid));

    // The ground at the anchor: the grid sampled there (bilinear, pixel-is-area), plus N.
    const auto at = [&](std::uint32_t row, std::uint32_t col) { return static_cast<double>(tiff->samples[row * dem_n + col]); };
    const auto clamp_index = [&](double v) { return v < 0.0 ? 0.0 : (v > static_cast<double>(dem_n - 1) ? static_cast<double>(dem_n - 1) : v); };
    const double u = clamp_index((q.lon_deg - box.west_deg) / (box.east_deg - box.west_deg) * dem_n - 0.5);
    const double v = clamp_index((box.north_deg - q.lat_deg) / (box.north_deg - box.south_deg) * dem_n - 0.5);
    const auto c0 = static_cast<std::uint32_t>(u);
    const auto r0 = static_cast<std::uint32_t>(v);
    const std::uint32_t c1 = c0 + 1 < dem_n ? c0 + 1 : c0;
    const std::uint32_t r1 = r0 + 1 < dem_n ? r0 + 1 : r0;
    const double fc = u - static_cast<double>(c0);
    const double fr = v - static_cast<double>(r0);
    const double top = at(r0, c0) * (1.0 - fc) + at(r0, c1) * fc;
    const double bottom = at(r1, c0) * (1.0 - fc) + at(r1, c1) * fc;
    const double ground = top * (1.0 - fr) + bottom * fr + n_geoid;

    // ---- the georef, the size check, the store ----------------------------------------------------------------------------------
    emit(env, "store", ProgressEvent::State::start);
    MapGeoref g;
    g.key = key;
    g.anchor_lat_deg = q.lat_deg;
    g.anchor_lon_deg = q.lon_deg;
    g.side_m = q.side_m;
    g.west_deg = box.west_deg;
    g.south_deg = box.south_deg;
    g.east_deg = box.east_deg;
    g.north_deg = box.north_deg;
    g.elevation = {"height.f32", dem_n, dem_n, q.side_m / dem_n};
    g.image = {"image.png", image_n, image_n, q.side_m / image_n};
    g.geoid_model = geoid->model;
    g.geoid_height_m = n_geoid;
    g.anchor_ground_ellipsoid_height_m = ground;
    g.elevation_source = {"USGS 3DEP", elevation_request, dem_dl.retrieved_utc, {}};
    g.image_source = {"USGS NAIP", image_request, img_dl.retrieved_utc, {}};
    g.geoid_source = {"NOAA NGS", geoid_url(q.lat_deg, q.lon_deg), geoid_dl.retrieved_utc, {}};
    g.datum_note = kDatumNote;
    g.fetched_utc = env.utc_now ? env.utc_now() : std::string();
    g.height_sha256 = sha256_hex(heights);
    g.image_sha256 = sha256_hex(img_dl.body);

    const std::uint64_t entry_bytes = heights.size() + img_dl.body.size() + serialize_georef(g).size();
    if (entry_bytes > q.cache_limit_bytes) {
        return failure(FetchResultKind::cache_full,
                       std::format("one map entry is {} bytes and the cache limit is {} bytes", entry_bytes, q.cache_limit_bytes));
    }
    std::error_code ec;
    if (fs::exists(dir, ec)) fs::remove_all(dir, ec);  // a refresh, or a damaged entry: the new one replaces it
    const Result<StoredEntry> stored = store_entry(q.cache_root, g, heights, img_dl.body, env.tmp_token);
    if (!stored) return failure(FetchResultKind::refused, "cannot write the cache: " + stored.error().context);
    (void)trim_cache(q.cache_root, q.cache_limit_bytes, key);
    emit(env, "store", ProgressEvent::State::done);

    FetchResult r = result_from_georef(g, stored->dir, FetchResultKind::fetched);
    r.bytes_geoid = geoid_dl.body.size();
    r.bytes_elevation = dem_dl.body.size();
    r.bytes_image = img_dl.body.size();
    return r;
}

}  // namespace spade::geofetch
