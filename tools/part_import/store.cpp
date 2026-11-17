#include "store.hpp"

#include "world/sha256.hpp"

namespace spade::partimport {

std::string cache_key_text(const KeyInputs& in) {
    return "spade-part-import-key-1;converter=" + in.converter + ";tolerance=" + json_number(in.tolerance_m) + ";angle=" +
           json_number(in.angle_rad) + ";unit=" + in.unit + ";axes=" + in.axes.text() + ";scale=" + json_number(in.scale) +
           ";object=" + in.object;
}

std::string cache_key(const KeyInputs& in) { return sha256_hex(cache_key_text(in)).substr(0, 16); }

std::string safe_file_name(std::string_view) { return {}; }
std::vector<std::string> unique_file_names(const std::vector<std::string>&) { return {}; }
std::filesystem::path entry_dir(const std::filesystem::path& store, std::string_view sha256, std::string_view key) {
    return store / std::string(sha256) / std::string(key);
}
Expected<EntryRecord> write_entry(const std::filesystem::path&, std::string_view, std::string_view, const KeyInputs&,
                                  std::span<const StoredObject>, std::string_view) {
    return std::unexpected(Problem{"internal", "not built"});
}
std::optional<EntryRecord> find_entry(const std::filesystem::path&, std::string_view, std::string_view) { return std::nullopt; }
std::size_t remove_stale_temps(const std::filesystem::path&, std::chrono::seconds) { return 0; }

}  // namespace spade::partimport
