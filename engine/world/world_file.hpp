#pragma once

// ---------------------------------------------------------------------------
// The versioned YAML world file (engine design D7, spec section 8 "Worlds &
// the file format").
//
// A world file is the SERIALIZED FORM OF A WorldDesc -- the WorldBuilder's
// product, not its call sequence. The builder compiles a fluent authoring
// chain into a flat postfix SDF program plus a transform table; this format
// writes exactly that, so a file is an image of what the engine will actually
// evaluate rather than a recipe someone has to re-derive.
//
// THE TWO PROPERTIES THAT MAKE IT USABLE AS AN ENGINE ARTIFACT
//
//   1. BIT-EXACT. Every float is written with 9 significant digits, which is
//      exactly what an IEEE-754 binary32 needs to survive a decimal round trip
//      (FLT_DECIMAL_DIG). A world saved and reloaded is the SAME world down to
//      the last bit of every parameter -- which is what lets the determinism
//      corpus (tests/golden/scenarios/*.scenario.yaml) be pinned against
//      worlds that live in files instead of in C++.
//
//      That claim is only worth as much as its weakest conversion, so both
//      directions avoid the standard library's LOCALE-SENSITIVE text paths:
//      emission goes through std::to_chars, parsing through std::from_chars,
//      and yaml-cpp's own as<float>() (iostream-shaped, hence locale-shaped)
//      is never used. See the long note in world_file.cpp.
//
//   2. ONE VALIDATION. Loading is parse -> validate_world_desc() -> WorldDesc,
//      calling the SAME function WorldBuilder::build() calls. There is no
//      second, file-specific notion of a valid world that could drift from the
//      builder's.
//
// On top of that shared validation the loader enforces what only a text format
// can get wrong: the schema version, unknown keys (an error at every level --
// a v1 reader must never silently drop a field a later version added),
// missing keys, duplicate keys, malformed numbers, and out-of-range indices.
//
// DIAGNOSTICS, precisely. Every error raised by the PARSE LAYER -- the checks
// listed above, the ones that are about the document -- carries the offending
// node's YAML line and column. Errors raised by the SHARED VALIDATION do not,
// and deliberately so: validate_world_desc() sees a WorldDesc, which has no
// idea it came from a file, and giving it one would be exactly the
// file-specific validation path property 2 exists to prevent. Those errors
// name the offending FIELD instead ("duplicate spawn point name 'start'").
// load_world_file() prefixes the file path to both kinds.
//
// NO EXCEPTIONS ESCAPE. yaml-cpp reports parse failures by throwing; those are
// caught at the boundary and returned as Error, like every other Spade API.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

#include "core/error.hpp"
#include "world/builder.hpp"

namespace spade {

// The schema version this build WRITES. A file carrying a version this build
// does not read at all (below kWorldFileMinReadVersion or above this) is
// rejected rather than guessed at: version gating IS the upgrade path, which
// is precisely why unknown keys are an error.
inline constexpr uint32_t kWorldFileVersion = 2;

// The oldest schema version this build still READS, via an upgrade path: a
// `world_version: 1` file loads successfully and its WorldDesc gains schema
// v2's three new sections at their defaults (one default material, default
// lighting, no props) -- see world_file.cpp's parse_world(). This build never
// WRITES version 1 again; kWorldFileVersion above is the only version
// world_to_yaml() ever produces.
inline constexpr uint32_t kWorldFileMinReadVersion = 1;

// Serializes a VALIDATED world to canonical schema-v2 YAML text (LF line
// endings). Fails with the validation error if `world` is not valid --
// nothing invalid is ever written -- and with invalid_argument if the world
// holds something schema v2 cannot represent.
//
// CANONICAL means the text is a pure function of the WorldDesc: fixed key
// order, fixed float spelling, fixed comment blocks. world_to_yaml(
// *world_from_yaml(t)) == t for any t this function produced.
[[nodiscard]] Result<std::string> world_to_yaml(const WorldDesc& world);

// Parses schema-v1-or-v2 YAML text (see kWorldFileMinReadVersion) and returns
// the validated world. Diagnostics do NOT carry a file path (there is no file
// here -- load_world_file() adds it).
//
// Codes: schema_mismatch when the file states a `world_version` this build
// does not read at all -- the one failure a caller may answer by upgrading
// rather than by editing, so it is distinguishable without matching on a
// string; invalid_argument for every other rejection (malformed YAML, unknown
// or missing or duplicate keys, bad numbers, and everything
// validate_world_desc() rejects); capacity_exceeded for an SDF program past
// kMaxSdfDepth.
[[nodiscard]] Result<WorldDesc> world_from_yaml(std::string_view yaml_text);

// world_to_yaml() + write. Writes bytes verbatim (binary mode: the LF line
// endings in the text are the ones that reach the disk, on every platform).
// Does NOT create parent directories -- creating a tree as a side effect of
// "save this file" is the kind of surprise a tool should ask for explicitly.
[[nodiscard]] Result<void> save_world_file(const WorldDesc& world,
                                           const std::filesystem::path& path);

// Read + world_from_yaml(). io_error if the file cannot be read; otherwise
// world_from_yaml()'s error with the path prefixed.
[[nodiscard]] Result<WorldDesc> load_world_file(const std::filesystem::path& path);

}  // namespace spade
