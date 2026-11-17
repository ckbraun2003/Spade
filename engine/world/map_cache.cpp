#include "world/map_cache.hpp"

namespace spade {

// RED: stubs until the tests in tests/test_map_cache.cpp are written and run.
std::string map_key(double, double, double) { return {}; }
std::filesystem::path map_entry_dir(const std::filesystem::path&, std::string_view) { return {}; }
std::filesystem::path default_map_cache_root(const EnvLookup&, MapPlatform) { return {"stub"}; }
bool inside_git_work_tree(const std::filesystem::path&) { return false; }
std::string serialize_georef(const MapGeoref&) { return {}; }
Result<MapGeoref> parse_georef(std::string_view) { return MapGeoref{}; }

}  // namespace spade
