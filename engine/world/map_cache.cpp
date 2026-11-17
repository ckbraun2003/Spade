#include "world/map_cache.hpp"

#include <cstdint>
#include <exception>
#include <format>
#include <system_error>

#include <yaml-cpp/yaml.h>

namespace spade {

namespace {

namespace fs = std::filesystem;

// Round half away from zero, in integer arithmetic only (no libm, so the key is the same on every platform).
[[nodiscard]] std::int64_t round_scaled(double v, double scale) {
    const double x = v * scale;
    return static_cast<std::int64_t>(x < 0.0 ? x - 0.5 : x + 0.5);
}

[[nodiscard]] std::string quote(const std::string& s) {
    std::string out = "\"";
    for (const char c : s) {
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out.push_back(c); break;
        }
    }
    out.push_back('"');
    return out;
}

void raster_text(std::string& out, const char* name, const MapRaster& r, const char* cell_key) {
    out += std::format("{}:\n  file: {}\n  width: {}\n  height: {}\n  {}: {}\n", name, quote(r.file), r.width, r.height, cell_key,
                       r.cell_m);
}

void source_text(std::string& out, const char* name, const MapSource& s) {
    out += std::format("  {}:\n    name: {}\n    url: {}\n    retrieved_utc: {}\n", name, quote(s.name), quote(s.url),
                       quote(s.retrieved_utc));
    if (!s.acquired.empty()) out += std::format("    acquired: {}\n", quote(s.acquired));
}

struct Bad {
    std::string what;
};

[[nodiscard]] YAML::Node need(const YAML::Node& parent, const char* key, const std::string& path) {
    if (!parent.IsMap()) throw Bad{"'" + path.substr(0, path.empty() ? 0 : path.size() - 1) + "' is not a mapping"};
    YAML::Node n = parent[key];
    if (!n.IsDefined() || n.IsNull()) throw Bad{"missing '" + path + key + "'"};
    return n;
}

template <class T>
[[nodiscard]] T value(const YAML::Node& parent, const char* key, const std::string& path) {
    const YAML::Node n = need(parent, key, path);
    try {
        return n.as<T>();
    } catch (const std::exception&) {
        throw Bad{"'" + path + key + "' has the wrong type"};
    }
}

[[nodiscard]] MapRaster read_raster(const YAML::Node& root, const char* name, const char* cell_key) {
    const std::string path = std::string(name) + ".";
    const YAML::Node n = need(root, name, "");
    MapRaster r;
    r.file = value<std::string>(n, "file", path);
    r.width = value<std::uint32_t>(n, "width", path);
    r.height = value<std::uint32_t>(n, "height", path);
    r.cell_m = value<double>(n, cell_key, path);
    return r;
}

[[nodiscard]] MapSource read_source(const YAML::Node& sources, const char* name) {
    const std::string path = std::string("sources.") + name + ".";
    const YAML::Node n = need(sources, name, "sources.");
    MapSource s;
    s.name = value<std::string>(n, "name", path);
    s.url = value<std::string>(n, "url", path);
    s.retrieved_utc = value<std::string>(n, "retrieved_utc", path);
    if (n["acquired"].IsDefined() && !n["acquired"].IsNull()) s.acquired = value<std::string>(n, "acquired", path);
    return s;
}

}  // namespace

std::string map_key(double lat_deg, double lon_deg, double side_m) {
    const std::int64_t la = round_scaled(lat_deg, 1e6);
    const std::int64_t lo = round_scaled(lon_deg, 1e6);
    const std::int64_t side = round_scaled(side_m, 1.0);
    std::string key;
    key += la < 0 ? 's' : 'n';
    key += std::to_string(la < 0 ? -la : la);
    key += '_';
    key += lo < 0 ? 'w' : 'e';
    key += std::to_string(lo < 0 ? -lo : lo);
    key += "_s";
    key += std::to_string(side);
    return key;
}

fs::path map_entry_dir(const fs::path& root, std::string_view key) {
    return root / ("v" + std::to_string(kMapCacheLayoutVersion)) / std::string(key);
}

fs::path default_map_cache_root(const EnvLookup& env, MapPlatform platform) {
    const auto get = [&](const char* name) -> std::string {
        const char* v = env ? env(name) : nullptr;
        return v != nullptr ? std::string(v) : std::string();
    };
    if (const std::string own = get("SPADE_MAP_CACHE"); !own.empty()) return fs::path(own);
    if (platform == MapPlatform::windows) {
        if (const std::string local = get("LOCALAPPDATA"); !local.empty()) return fs::path(local) / "Spade" / "maps";
        return {};
    }
    if (const std::string xdg = get("XDG_CACHE_HOME"); !xdg.empty()) return fs::path(xdg) / "spade" / "maps";
    if (const std::string home = get("HOME"); !home.empty()) return fs::path(home) / ".cache" / "spade" / "maps";
    return {};
}

bool inside_git_work_tree(const fs::path& dir) {
    std::error_code ec;
    fs::path p = fs::absolute(dir, ec);
    if (ec) p = dir;
    p = p.lexically_normal();
    for (;;) {
        std::error_code e2;
        if (fs::exists(p / ".git", e2)) return true;
        const fs::path parent = p.parent_path();
        if (parent == p || parent.empty()) return false;
        p = parent;
    }
}

std::string serialize_georef(const MapGeoref& g) {
    std::string out;
    out += std::format("georef_version: {}\n", g.georef_version);
    out += std::format("key: {}\n", quote(g.key));
    out += std::format("anchor:\n  lat_deg: {}\n  lon_deg: {}\n", g.anchor_lat_deg, g.anchor_lon_deg);
    out += std::format("side_m: {}\n", g.side_m);
    out += std::format("bbox_deg:\n  west: {}\n  south: {}\n  east: {}\n  north: {}\n", g.west_deg, g.south_deg, g.east_deg,
                       g.north_deg);
    raster_text(out, "elevation", g.elevation, "cell_m");
    out += std::format("  vertical: {}\n", quote(g.vertical));
    raster_text(out, "image", g.image, "pixel_m");
    out += std::format("geoid:\n  model: {}\n  height_m: {}\n", quote(g.geoid_model), g.geoid_height_m);
    out += std::format("anchor_ground:\n  ellipsoid_height_m: {}\n", g.anchor_ground_ellipsoid_height_m);
    out += std::format("sha256:\n  height_f32: {}\n  image_png: {}\n", quote(g.height_sha256), quote(g.image_sha256));
    out += "sources:\n";
    source_text(out, "elevation", g.elevation_source);
    source_text(out, "image", g.image_source);
    source_text(out, "geoid", g.geoid_source);
    out += std::format("datum_note: {}\n", quote(g.datum_note));
    out += std::format("fetched_utc: {}\n", quote(g.fetched_utc));
    return out;
}

Result<MapGeoref> parse_georef(std::string_view text) {
    const auto invalid = [](const std::string& why) {
        return std::unexpected(Error{Code::schema_mismatch, "map_invalid: georef.yaml: " + why});
    };
    try {
        const YAML::Node root = YAML::Load(std::string(text));
        if (!root.IsMap()) return invalid("empty, or not a mapping");
        MapGeoref g;
        g.georef_version = value<int>(root, "georef_version", "");
        if (g.georef_version != 1) return invalid("georef_version must be 1 (found " + std::to_string(g.georef_version) + ")");
        g.key = value<std::string>(root, "key", "");
        const YAML::Node anchor = need(root, "anchor", "");
        g.anchor_lat_deg = value<double>(anchor, "lat_deg", "anchor.");
        g.anchor_lon_deg = value<double>(anchor, "lon_deg", "anchor.");
        g.side_m = value<double>(root, "side_m", "");
        const YAML::Node bbox = need(root, "bbox_deg", "");
        g.west_deg = value<double>(bbox, "west", "bbox_deg.");
        g.south_deg = value<double>(bbox, "south", "bbox_deg.");
        g.east_deg = value<double>(bbox, "east", "bbox_deg.");
        g.north_deg = value<double>(bbox, "north", "bbox_deg.");
        g.elevation = read_raster(root, "elevation", "cell_m");
        g.vertical = value<std::string>(need(root, "elevation", ""), "vertical", "elevation.");
        g.image = read_raster(root, "image", "pixel_m");
        const YAML::Node geoid = need(root, "geoid", "");
        g.geoid_model = value<std::string>(geoid, "model", "geoid.");
        g.geoid_height_m = value<double>(geoid, "height_m", "geoid.");
        g.anchor_ground_ellipsoid_height_m = value<double>(need(root, "anchor_ground", ""), "ellipsoid_height_m", "anchor_ground.");
        const YAML::Node hashes = need(root, "sha256", "");
        g.height_sha256 = value<std::string>(hashes, "height_f32", "sha256.");
        g.image_sha256 = value<std::string>(hashes, "image_png", "sha256.");
        const YAML::Node sources = need(root, "sources", "");
        g.elevation_source = read_source(sources, "elevation");
        g.image_source = read_source(sources, "image");
        g.geoid_source = read_source(sources, "geoid");
        g.datum_note = value<std::string>(root, "datum_note", "");
        g.fetched_utc = value<std::string>(root, "fetched_utc", "");
        return g;
    } catch (const Bad& b) {
        return invalid(b.what);
    } catch (const std::exception&) {
        return invalid("not valid YAML");
    }
}

}  // namespace spade
