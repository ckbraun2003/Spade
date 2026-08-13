#pragma once

// ---------------------------------------------------------------------------
// C5: the world reference (engine design spec, "worlds loadable by path or
// handle") -- a WorldDesc a caller can name either by FILE PATH or by an
// already-resolved, in-memory value, plus resolve_world() to collapse either
// spelling to the WorldDesc a Simulation is actually built from.
//
// WHY A VARIANT AND NOT AN OVERLOAD SET. A path and a desc are the same
// THING at two different distances -- "the world this fleet uses" -- and the
// call sites that build fleets (world_set_from below, and the tools that will
// call it) want to accept either without duplicating themselves per shape.
// std::variant makes that ambiguity a type instead of a convention: there is
// exactly one resolve step, and it is impossible to forget which branch you
// are in.
//
// resolve_world() IS INLINE, HEADER-ONLY, AND STILL SAFE TO SHIP THAT WAY.
// The path branch calls load_world_file() (world/world_file.hpp), a compiled
// entry point in the spade_world archive -- the inline wrapper adds no new
// I/O or parsing logic of its own, so it cannot drift from the compiled
// loader's behaviour the way a re-implementation could. The desc branch is a
// copy. Both are cheap enough, and small enough, that a header is the right
// home: this is the world LAYER (may depend on world_file.hpp), never the sim
// layer, which is why world_set_from -- the thing that actually BUILDS a
// fleet -- lives in sim/world_set.hpp instead of here.
// ---------------------------------------------------------------------------

#include <filesystem>
#include <variant>

#include "core/error.hpp"
#include "world/builder.hpp"
#include "world/world_file.hpp"

namespace spade {

// Either a path to a schema-v1 world file, or an already-resolved WorldDesc
// held in memory. The second alternative is what lets a caller who built a
// world programmatically (WorldBuilder, a test fixture, a generated scene)
// feed it through the same fleet constructor a file-backed caller uses.
using WorldRef = std::variant<std::filesystem::path, WorldDesc>;

// Collapses a WorldRef to the WorldDesc it names: the path alternative loads
// and validates the file (load_world_file()); the desc alternative runs
// validate_world_desc() (world/builder.hpp) on its own copy before returning
// it (S5 final-review fix wave, I5 -- coordinator-ruled, fail-closed, the
// same posture as every other ratified program ruling). Both alternatives
// therefore validate, which is what makes builder.hpp's "THE ONE VALIDATION"
// comment and world_file.hpp's "ONE VALIDATION" claim true for every producer
// of a WorldDesc rather than only the ones that happen to route through
// WorldBuilder::build() or the YAML loader: a WorldRef's desc alternative
// exists precisely so a caller can hand in a WorldDesc built some OTHER way
// (a test fixture, a generated scene, a hand-assembled aggregate), and
// nothing upstream of resolve_world() can guarantee that value was ever
// validated at all. This is not a re-validation in the sense load_world_file()
// avoids (world_from_yaml() already calls validate_world_desc() once, and
// load_world_file() does not call it a second time on top) -- it is the
// FIRST and only validation this desc ever receives.
//
// Errors are load_world_file()'s or validate_world_desc()'s, verbatim --
// code and context both, including the file path load_world_file() already
// prefixes onto every diagnostic. There is nothing for this function to add.
[[nodiscard]] inline Result<WorldDesc> resolve_world(const WorldRef& ref) {
    if (const std::filesystem::path* path = std::get_if<std::filesystem::path>(&ref)) {
        return load_world_file(*path);
    }
    WorldDesc desc = std::get<WorldDesc>(ref);
    if (const Result<uint32_t> validated = validate_world_desc(desc); !validated) {
        return std::unexpected(validated.error());
    }
    return desc;
}

}  // namespace spade
