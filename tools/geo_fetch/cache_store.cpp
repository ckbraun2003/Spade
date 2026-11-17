#include "cache_store.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <system_error>

#include "world/sha256.hpp"

namespace spade::geofetch {

namespace {

namespace fs = std::filesystem;

[[nodiscard]] fs::path v1_dir(const fs::path& root) { return root / ("v" + std::to_string(kMapCacheLayoutVersion)); }

[[nodiscard]] bool is_temporary(const fs::path& p) { return p.filename().string().rfind(".tmp-", 0) == 0; }

[[nodiscard]] bool write_file(const fs::path& path, std::string_view bytes) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    out.flush();
    return static_cast<bool>(out);
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

}  // namespace

bool entry_complete(const fs::path& entry_dir) {
    std::error_code ec;
    return fs::is_regular_file(entry_dir / "georef.yaml", ec);
}

Result<StoredEntry> store_entry(const fs::path& root, MapGeoref georef, std::string_view height_f32, std::string_view image_png,
                                std::string_view tmp_token) {
    const fs::path base = v1_dir(root);
    const fs::path tmp = base / (".tmp-" + std::string(tmp_token) + "-" + georef.key);
    const fs::path final_dir = base / georef.key;
    std::error_code ec;
    fs::remove_all(tmp, ec);
    fs::create_directories(tmp, ec);
    if (ec) return std::unexpected(Error{Code::io_error, "cannot create " + tmp.string() + ": " + ec.message()});

    georef.elevation.file = "height.f32";
    georef.image.file = "image.png";
    georef.height_sha256 = sha256_hex(height_f32);
    georef.image_sha256 = sha256_hex(image_png);
    const std::string georef_text = serialize_georef(georef);
    // georef.yaml last: a directory without it is an unfinished entry.
    if (!write_file(tmp / "height.f32", height_f32) || !write_file(tmp / "image.png", image_png) ||
        !write_file(tmp / "georef.yaml", georef_text)) {
        fs::remove_all(tmp, ec);
        return std::unexpected(Error{Code::io_error, "cannot write the map entry in " + tmp.string()});
    }

    if (fs::exists(final_dir, ec)) {
        if (entry_complete(final_dir)) {
            fs::remove_all(tmp, ec);
            return StoredEntry{final_dir, false};
        }
        fs::remove_all(final_dir, ec);  // an unfinished entry: replaced
    }
    fs::rename(tmp, final_dir, ec);
    if (ec) {
        const bool raced = entry_complete(final_dir);  // another fetch of the same area won
        std::error_code ignore;
        fs::remove_all(tmp, ignore);
        if (raced) return StoredEntry{final_dir, false};
        return std::unexpected(Error{Code::io_error, "cannot move " + tmp.string() + " to " + final_dir.string() + ": " + ec.message()});
    }
    return StoredEntry{final_dir, true};
}

std::uint64_t cache_size_bytes(const fs::path& root) {
    std::uint64_t total = 0;
    std::error_code ec;
    for (fs::directory_iterator it(v1_dir(root), ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code dir_ec;
        if (!it->is_directory(dir_ec) || is_temporary(it->path())) continue;
        total += dir_bytes(it->path());
    }
    return total;
}

TrimResult trim_cache(const fs::path& root, std::uint64_t limit_bytes, std::string_view keep_key) {
    struct Item {
        std::string key;
        fs::path dir;
        std::string fetched;  // "" for an unreadable georef: the oldest
        bool complete = false;
        std::uint64_t bytes = 0;
    };
    std::vector<Item> items;
    std::error_code ec;
    for (fs::directory_iterator it(v1_dir(root), ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code dir_ec;
        if (!it->is_directory(dir_ec) || is_temporary(it->path())) continue;
        Item item;
        item.key = it->path().filename().string();
        item.dir = it->path();
        item.complete = entry_complete(item.dir);
        item.bytes = dir_bytes(item.dir);
        if (item.complete) {
            const Result<MapGeoref> g = parse_georef(slurp(item.dir / "georef.yaml"));
            if (g) item.fetched = g->fetched_utc;
        }
        items.push_back(std::move(item));
    }
    // Incomplete entries first, then the oldest fetched; the key breaks a tie, so the order is the same everywhere.
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
        if (a.complete != b.complete) return !a.complete;
        if (a.fetched != b.fetched) return a.fetched < b.fetched;
        return a.key < b.key;
    });
    std::uint64_t total = 0;
    for (const Item& i : items) total += i.bytes;

    TrimResult result;
    for (const Item& i : items) {
        if (i.key == keep_key) continue;
        if (i.complete && total <= limit_bytes) break;
        std::error_code rm;
        fs::remove_all(i.dir, rm);
        total -= i.bytes;
        result.removed.push_back(i.key);
    }
    result.total_after = total;
    return result;
}

}  // namespace spade::geofetch
