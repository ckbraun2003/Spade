#include "cache_store.hpp"

namespace spade::geofetch {

// RED: stubs.
bool entry_complete(const std::filesystem::path&) { return false; }
Result<StoredEntry> store_entry(const std::filesystem::path&, MapGeoref, std::string_view, std::string_view, std::string_view) {
    return StoredEntry{};
}
std::uint64_t cache_size_bytes(const std::filesystem::path&) { return 0; }
TrimResult trim_cache(const std::filesystem::path&, std::uint64_t, std::string_view) { return {}; }

}  // namespace spade::geofetch
