#pragma once

// ---------------------------------------------------------------------------
// geo_fetch/cache_store.hpp -- writing the map cache and keeping it inside its size limit (L85, SP-5; the design note,
// section 4). The layout is world/map_cache.hpp's. Only the fetch tool writes; the engine's reader never does.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "world/map_cache.hpp"

namespace spade::geofetch {

// True when the entry directory holds georef.yaml: the file written last, so its presence means the entry is complete.
[[nodiscard]] bool entry_complete(const std::filesystem::path& entry_dir);

struct StoredEntry {
    std::filesystem::path dir;
    bool stored = false;  // false: a complete entry for the key was already there (two fetches at once) and was kept
};

// Writes <root>/v1/<key>/ from the two files' bytes and the georef (whose hashes it fills in from the bytes): into a temporary
// directory in <root>/v1, georef.yaml last, then renamed into place. `tmp_token` makes the temporary name unique per process.
// An incomplete directory already at the key is replaced; a complete one is kept and the temporary directory removed.
[[nodiscard]] Result<StoredEntry> store_entry(const std::filesystem::path& root, MapGeoref georef, std::string_view height_f32,
                                              std::string_view image_png, std::string_view tmp_token);

// The bytes of every file in the complete and incomplete entries under <root>/v1 (temporary directories are not counted).
[[nodiscard]] std::uint64_t cache_size_bytes(const std::filesystem::path& root);

struct TrimResult {
    std::vector<std::string> removed;  // keys, in the order removed
    std::uint64_t total_after = 0;
};

// Removes incomplete entries, then the oldest complete ones (by georef fetched_utc, then by key; an unreadable georef counts
// as the oldest) until the cache is within `limit_bytes`. `keep_key` is never removed.
TrimResult trim_cache(const std::filesystem::path& root, std::uint64_t limit_bytes, std::string_view keep_key);

}  // namespace spade::geofetch
