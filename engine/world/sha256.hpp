#pragma once

// ---------------------------------------------------------------------------
// world/sha256.hpp -- SHA-256 (FIPS 180-4) of a byte string, as 64 lower-case hex characters.
//
// The geo map cache (world/map_cache.hpp; the SP-5 design note, docs/design/interface/plans/2026-11-17-geo-map-fetch-design.md
// section 4) records the hash of each file it stores, and its reader checks it: a mismatch is a damaged entry. One small
// portable implementation, shared by the reader and the fetch tool. Integer arithmetic only, so it is bit-equal on every
// platform and needs nothing from the platform's libm.
// ---------------------------------------------------------------------------

#include <string>
#include <string_view>

namespace spade {

// The SHA-256 of `bytes`, 64 lower-case hex digits.
[[nodiscard]] std::string sha256_hex(std::string_view bytes);

}  // namespace spade
