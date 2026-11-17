#pragma once

// ---------------------------------------------------------------------------
// world/terrain_map.hpp -- the cache reader: the geo map the fetch tool stored, as the terrain's input (L85, geo G2, SP-5; the design
// note docs/design/interface/plans/2026-11-17-geo-map-fetch-design.md). THE DEFINITION IS IN spade::mesh_import, NOT spade::world:
// the aerial image is a PNG and decode_png lives in that archive, which links world, so a world-only consumer that calls this fails
// to link. Link spade::mesh_import. The result type is world/terrain_input.hpp's.
//
// READ ONLY. It reads the local cache and nothing else: no network, no write, no environment variable; the caller names the cache
// directory (default_map_cache_root() in map_cache.hpp finds the usual one).
// ---------------------------------------------------------------------------

#include <cstdint>
#include <filesystem>

#include "core/error.hpp"
#include "world/terrain_input.hpp"

namespace spade {

// The most a cached file may be, checked before it is read (the grid is at most 1024 x 1024 float32; the answer is capped at 64 MiB).
inline constexpr std::uint64_t kMaxMapHeightBytes = 1024ull * 1024ull * 4ull;
inline constexpr std::uint64_t kMaxMapImageBytes = 64ull * 1024ull * 1024ull;

// Reads the entry for the area at (anchor_lat_deg, anchor_lon_deg) and `side_m` from `cache_root`: the key, the georef, the two files,
// their SHA-256 hashes, the PNG. The result is already in metres: heights up from the anchor's ground (the georef's ellipsoid height
// at the anchor is subtracted), the extent east and north of the CALLER'S anchor (the key rounds the anchor to 0.11 m, the georef's
// box is the geometry), the image decoded.
//
//   not_found         "map_missing: ..." no entry for the key (no georef.yaml): the host loads plain ground and says why
//   schema_mismatch   "map_invalid: <file>: ..." the entry is there and cannot be trusted: a hash that does not match, a file of the
//                     wrong size or missing, a georef that fails to parse or names another key, a height that is not finite, an
//                     image decode_png refuses (its own message follows), a size over the caps
//   invalid_argument  the anchor fails geo::validate_anchor, or the side is outside 10 to 1000 m
[[nodiscard]] Result<world::TerrainInput> read_terrain_map(const std::filesystem::path& cache_root, double anchor_lat_deg,
                                                           double anchor_lon_deg, double side_m);

}  // namespace spade
